# Copyright (C) 2019 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the 3-Clause BSD License which accompanies this
# distribution, and is available at
# https://opensource.org/licenses/BSD-3-Clause
#
# Contributors:
# Christoph Suerig <christoph.suerig@dlr.de>

# Don't connect with the Copyright comment above!
# Version 19.07.2019


import os
from threading import Thread

import rafcon.gui.singleton as gui_singletons
from rafcon.core.states.hierarchy_state import HierarchyState
from rafcon.gui.singleton import state_machine_manager_model
from rafcon.utils import log
from rafcon.utils.gui_functions import call_gui_callback

from rafcontpp.control.execution_controller import ExecutionController
from rafcontpp.logic.mapper import Mapper
from rafcontpp.logic.pddl_action_loader import PddlActionLoader
from rafcontpp.logic.predicate_merger import PredicateMerger
from rafcontpp.logic.type_merger import TypeMerger
from rafcontpp.model.datastore import Datastore, DATASTORE_STORAGE_PATH
from rafcontpp.model.datastore import SEMANTIC_DATA_DICT_NAME, ALLOW_OVERRIDE_NAME
from rafcontpp.view.confirm_dialog import ConfirmDialog
from rafcontpp.view.state_pool_info_window import StatePoolInfoWindow

logger = log.get_logger(__name__)
# other string, if other planner is choosen.
OTHER = 'Other...'
# select planner string, if nothing is choosen.
SEL_PLANNER = '-- Select planner --'
# printed next to planner, if it is not available.
NOT_AVAILABLE = ' (!) Unavailable'
# the content of the planning wait window
WAIT_WINDOW_CONTENT = 'Planning State machine, please wait...\r\n\r\n ' \
                      'This could take a long time. If you want,\r\n ' \
                      'you can close this Window, and use rafcon\r\n' \
                      'while the Task is planned in the background.\r\n ' \
                      'If you generate into a State, please stay in the Tab,\r\n' \
                      'and don\'t close the State machine.\r\n'


class PlanningSetupFormController:
    """
    PlanningSetupFormController
    This is the controller of the Planning Setup Form.
    """

    def __init__(self, datastore):
        """
        :param datastore: A Datastore
        """
        assert isinstance(datastore, Datastore)
        self.__datastore = datastore

    def on_apply(self, button, setup_form, state_pool_string,
                 type_db_path, planner_text, planner_script_path, planner_argv_text,
                 facts_path, generate_into_state, sm_name, sm_save_dir, keep_related_files, file_save_dir,
                 rt_data_path, as_reference):
        """
        on_apply filles the datastore with new data entered into the setup form, saves it in the configuration,
        destroys the setup form, triggeres the pipeline e.g. the execution controller, and increments the plan task
        button, to indicate a running task.

        :param button: Unused.
        :param setup_form: The setup form, to be able to destroy it.
        :param state_pool_string: The statepool paths as colon seperated string.
        :param type_db_path: The path of the type db file as string.
        :param planner_text: The text of the planner field e.g. Other... or Fast-Downward Planning System.
        :param planner_script_path: The path of a planner script, or an empty String if there is none.
        :param planner_argv_text: An argv vector for the planner.
        :param facts_path: The Path of the facts file as String.
        :param sm_name: The desired name of the State machine.
        :param sm_save_dir: The path where to save the State machine as string.
        :param keep_related_files: True if related files should be keept, Fals if they should be deleted.
        :param file_save_dir: The path where to save related files, or an empty string if files shouldn't be keept.
        :param rt_data_path: The path where to find the runtime data file. Or an empty string if there is no runtime data.
        :param as_reference: True if the runtime data path should be used as reference, False if it should directly be copied into the State. if rt_data_path is empty, this field is irrelevant.
        :return: void
        """
        # prepare datastore with new data from dialog.
        # save datastore to configuration file.
        # destroy dialog.
        # start the pipeline to generate a sm.
        everything_filled, not_filled = self.__prepare_datastore(self.__datastore, state_pool_string,
                                                                 type_db_path, planner_text, planner_script_path,
                                                                 planner_argv_text,
                                                                 facts_path, generate_into_state, sm_name, sm_save_dir,
                                                                 keep_related_files, file_save_dir,
                                                                 rt_data_path, as_reference)

        if everything_filled:
            self.__datastore.validate_ds()
            self.__datastore.save_datastore_parts_in_file(DATASTORE_STORAGE_PATH)
            setup_form.hide()  # its more smoothly to first hide and then destroy
            main_window = gui_singletons.main_window_controller.view['main_window']
            planning_wait_window = ConfirmDialog(main_window, WAIT_WINDOW_CONTENT)
            planning_wait_window.show()
            setup_form.destroy()
            from rafcontpp.view.planning_button import increment_button
            increment_button()  # increment the button, to indicate that a new planning process has started.
            # start pipeline
            logger.info("Start pipeline...")
            planning_thread = None
            try:
                planning_thread = ExecutionController(self.__datastore).on_execute_pre_planning()
            finally:
                Thread(target=self.__wait_and_hide,
                       args=[planning_thread, planning_wait_window],
                       name='PlanningObserverThread').start()
        else:
            logger.error(" Field missing! {}".format(not_filled))
            ConfirmDialog(setup_form, " ERROR: Field missing!\r\n\r\n {}".format(not_filled)).show()

    def on_destroy(self, button, setup_form, state_pool_string,
                   type_db_path, planner_text, planner_script_path, planner_argv_text,
                   facts_path, generate_into_state, sm_name, sm_save_dir, keep_related_files, file_save_dir,
                   rt_data_path, as_reference):
        """
        on_destroy destroys the setup form, but saves the current configuration into a file.

        :param button: Unused.
        :param setup_form: The setup form, to be able to destroy it.
        :param state_pool_string: The statepool paths as colon seperated string.
        :param type_db_path: The path of the type db file as string.
        :param planner_text: The text of the planner field e.g. Other... or Fast-Downward Planning System.
        :param planner_script_path: The path of a planner script, or an empty String if there is none.
        :param planner_argv_text: An argv vector for the planner.
        :param facts_path: The Path of the facts file as String.
        :param sm_name: The desired name of the State machine.
        :param sm_save_dir: The path where to save the State machine as string.
        :param keep_related_files: True if related files should be keept, Fals if they should be deleted.
        :param file_save_dir: The path where to save related files, or an empty string if files shouldn't be keept.
        :param rt_data_path: The path where to find the runtime data file. Or an empty string if there is no runtime data.
        :param as_reference: True if the runtime data path should be used as reference, False if it should directly be copied into the State. if rt_data_path is empty, this field is irrelevant.
        :return: void
        """
        # destroy dialog
        # save data to datastore
        # save datastore to file.
        try:
            self.__prepare_datastore(self.__datastore, state_pool_string,
                                     type_db_path, planner_text, planner_script_path, planner_argv_text,
                                     facts_path, generate_into_state, sm_name, sm_save_dir, keep_related_files,
                                     file_save_dir,
                                     rt_data_path, as_reference)
            self.__datastore.save_datastore_parts_in_file(DATASTORE_STORAGE_PATH)
        finally:
            setup_form.destroy()

    def on_show_state_pool_info(self, button, setup_form, state_pool_string,
                                type_db_path, planner_text, planner_script_path, planner_argv_text,
                                facts_path, generate_into_state, sm_name, sm_save_dir, keep_related_files,
                                file_save_dir,
                                rt_data_path, as_reference):
        """
         Show state pool info, uses the provided state pools and the type file to collect details, and show
         them in a state pool info window.

        :param button: Unused.
        :param setup_form: The setup form, to be able to set it as parent.
        :param state_pool_string: The statepool paths as colon seperated string.
        :param type_db_path: The path of the type db file as string.
        :param planner_text: The text of the planner field e.g. Other... or Fast-Downward Planning System.
        :param planner_script_path: The path of a planner script, or an empty String if there is none.
        :param planner_argv_text: An argv vector for the planner.
        :param facts_path: The Path of the facts file as String.
        :param sm_name: The desired name of the State machine.
        :param sm_save_dir: The path where to save the State machine as string.
        :param keep_related_files: True if related files should be keept, Fals if they should be deleted.
        :param file_save_dir: The path where to save related files, or an empty string if files shouldn't be keept.
        :param rt_data_path: The path where to find the runtime data file. Or an empty string if there is no runtime data.
        :param as_reference: True if the runtime data path should be used as reference, False if it should directly be copied into the State. if rt_data_path is empty, this field is irrelevant.
        :return: void
        """

        available_predicates = []
        type_tree = None
        tmp_datastore = Datastore(None, None, None, None, None, None, None, None, None, None, )
        tmp_datastore.add_state_pools(self.__string_to_string_array(state_pool_string), True)
        tmp_datastore.set_type_db_path(type_db_path)
        merge_preds = True

        try:  # generate maps
            mapper = Mapper(tmp_datastore)
            mapper.generate_action_state_map()
            mapper.generate_state_action_map()
            mapper.generate_available_actions()
        except Exception as e:
            merge_preds = False
        try:  # load actions
            loader = PddlActionLoader(tmp_datastore)
            loader.load_pddl_actions()
        except Exception as e:
            merge_preds = False
        try:  # generate type tree
            type_merger = TypeMerger(tmp_datastore)
            type_tree = type_merger.merge_types()
            tmp_datastore.set_available_types(type_tree)

        except Exception as e:
            merge_preds = False
        try:  # generate available predicates.
            for state in tmp_datastore.get_pddl_action_map():
                action = tmp_datastore.get_pddl_action_map()[state]
                for predicate in action.predicates:
                    if predicate not in available_predicates:
                        available_predicates.append(predicate)
            if merge_preds:
                pred_merger = PredicateMerger(tmp_datastore)
                available_predicates = pred_merger.merge_predicates(available_predicates)[0]
        except Exception as e:
            available_predicates = [e.message]

        state_pool_info = StatePoolInfoWindow(setup_form)
        state_pool_info.set_state_pools(tmp_datastore.get_state_pools())
        state_pool_info.set_types(type_tree)
        state_pool_info.set_action_state_mapping(tmp_datastore.get_action_state_map())
        state_pool_info.set_predicates(available_predicates)
        state_pool_info.show()

    def on_choose_state_pool(self, chooser, chooser_entry):
        """
        Receives a directory chooser and a text entry, reads the path, set in the chooser and appends it to the text of
        the text entry.

        :param chooser: A GtkFileChooserButton
        :param chooser_entry:  A GtkEntry
        :return: void
        """
        # append choosen state pool to state pool text entry.
        to_append = chooser.get_filename()
        pools = chooser_entry.get_text()
        if len(pools) > 0 and pools[len(pools) - 1] != ':':
            pools += ':'
        chooser_entry.set_text(pools + to_append + ':')

    def on_choose_runtime_data(self, chooser, runtime_data_entry):
        """
        Receivs a file chooser and a text entry, reads the filepath of the chooser and sets it as text into the entry.

        :param chooser: A GtkFileChooserButton
        :param runtime_data_entry: A GtkEntry
        :return: void
        """
        runtime_data_entry.set_text(chooser.get_filename())

    def __prepare_datastore(self, datastore_to_prepare, state_pool_string,
                            type_db_path, planner_text, planner_script_path, planner_argv_text,
                            facts_path, generate_into_state, sm_name, sm_save_dir, keep_related_files, file_save_dir,
                            rt_data_path, as_reference):
        """
        __prepare_datastore saves the given data into the datastore, and checks if data is missing.

        :param setup_form: The setup form, to be able to destroy it.
        :param state_pool_string: The statepool paths as colon seperated string.
        :param type_db_path: The path of the type db file as string.
        :param planner_text: The text of the planner field e.g. Other... or Fast-Downward Planning System.
        :param planner_script_path: The path of a planner script, or an empty String if there is none.
        :param planner_argv_text: An argv vector for the planner.
        :param facts_path: The Path of the facts file as String.
        :param sm_name: The desired name of the State machine.
        :param sm_save_dir: The path where to save the State machine as string.
        :param keep_related_files: True if related files should be keept, Fals if they should be deleted.
        :param file_save_dir: The path where to save related files, or an empty string if files shouldn't be keept.
        :param rt_data_path: The path where to find the runtime data file. Or an empty string if there is no runtime data.
        :param as_reference: True if the runtime data path should be used as reference, False if it should directly be copied into the State. if rt_data_path is empty, this field is irrelevant.
        :return: void
        """
        # saves all data from the dialog into the datastore.
        # looks if everything necessary was filled.
        dtp = datastore_to_prepare
        everything_filled = True
        not_filled = None
        logger.debug('State pool: ' + str(self.__string_to_string_array(state_pool_string)))
        dtp.add_state_pools(self.__string_to_string_array(state_pool_string), True)
        dtp.set_type_db_path(type_db_path)
        choosen_planner = planner_text.replace(NOT_AVAILABLE, '')
        # set planner
        script_path = planner_script_path
        if choosen_planner == OTHER:
            choosen_planner = script_path
        if choosen_planner == SEL_PLANNER:
            everything_filled = False
            not_filled = 'a Planner'
        if choosen_planner != SEL_PLANNER:
            dtp.set_planner(choosen_planner)
        dtp.set_planner_script_path(script_path)
        # set planner argv
        if len(planner_argv_text) > 0:
            dtp.set_planner_argv(planner_argv_text.split(' '))
        else:
            dtp.set_planner_argv([])
        dtp.set_facts_path(facts_path)
        dtp.set_generate_into_state(generate_into_state)
        if generate_into_state:
            selected_state = self.__get_current_selected_state_if_valid()
            if selected_state:
                dtp.set_target_state(selected_state)
            else:
                everything_filled = False
                not_filled = 'State to plan into not accurately selected:' \
                             ' None or multiple States selected, or no Permission!'
        dtp.set_sm_name(sm_name)
        dtp.set_sm_save_dir(sm_save_dir)
        dtp.set_keep_related_files(keep_related_files)
        dtp.set_file_save_dir(file_save_dir)
        # runtime section
        runtime_data_path = rt_data_path.strip()
        dtp.set_use_runtime_path_as_ref(as_reference)
        dtp.set_runtime_data_path(runtime_data_path)
        if not dtp.use_runtime_path_as_ref() and runtime_data_path and len(runtime_data_path) > 0:
            if not os.path.isfile(dtp.get_runtime_data_path()):
                everything_filled = False
                not_filled = 'Runtime Data contains no valid Filepath!'
        return (everything_filled, not_filled)

    def __wait_and_hide(self, thread, planning_wait_window):
        """
        wait and hide should be executed in another thread, it joins the planning thread, closes the wait window
        and decrements the planning button.

        :param thread: The thread to wait for
        :param planning_wait_window: A window to hide and destroy, when the thread to wait for terminated.
        :return: void
        """
        # logger.debug('wait_and_hide executed from thread: {}'.format(threading.current_thread().getName()))# todo remove
        if thread and thread.is_alive():
            thread.join()
        call_gui_callback(planning_wait_window.hide)  # its more smoothly to first hide and then destroy
        call_gui_callback(planning_wait_window.destroy)
        from rafcontpp.view.planning_button import decrement_button
        call_gui_callback(decrement_button)  # decrement button, to indicate, that the planning process is finish.

    def __string_to_string_array(self, string):
        """
        A little method, that splits a colon seperated string into a string array.

        :param string: A string, containing colon seperated substrings.
        :return: A list of substrings e.g. a:b:c --> [a,b,c]
        """
        result = []
        if string and len(string) > 0:
            result = list(filter(None, string.split(':')))
        return result

    def __get_current_selected_state_if_valid(self):
        """
        Takes and validates the current selected state. Returns None if no, multiple or no valid state is selected.

        :return: HierarchyState: A valid root state or None
        """
        selected_sm = state_machine_manager_model.get_selected_state_machine_model()
        if not (selected_sm and selected_sm.selection.states):
            logger.error("No State selected!")
            return None
        if not len(selected_sm.selection.states) == 1:
            logger.error("Multiple States selected!")
            return None
        selected_state_model = selected_sm.selection.get_selected_state()
        selected_state = selected_state_model.state
        if not isinstance(selected_state, HierarchyState):
            logger.error("Can only plan into Hierarchy States!")
            return None
        permission_granted = \
            str(selected_state.semantic_data[SEMANTIC_DATA_DICT_NAME][ALLOW_OVERRIDE_NAME]).lower() == 'true'
        if not (permission_granted or len(selected_state.states) == 0):
            logger.error("Can't plan into None empty Hierarchy State without permission!")
            return None

        return selected_state
