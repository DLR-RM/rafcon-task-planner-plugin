# Contributors:
# Christoph Suerig <christoph.suerig@dlr.de>
# Version 18.04.2019


import os
import threading
from threading import Thread
from rafcon.utils.gui_functions import call_gui_callback
from rafcontpp.control.execution_controller import ExecutionController
from rafcontpp.model.datastore import Datastore, DATASTORE_STORAGE_PATH
from rafcontpp.logic.mapper import Mapper
from rafcontpp.logic.pddl_action_loader import PddlActionLoader
from rafcontpp.logic.predicate_merger import PredicateMerger
from rafcontpp.logic.type_merger import TypeMerger
from rafcon.utils import log

logger = log.get_logger(__name__)
#other string, if other planner is choosen.
OTHER = 'Other...'
#select planner string, if nothing is choosen.
SEL_PLANNER = '-- Select planner --'
#printed next to planner, if it is not available.
NOT_AVAILABLE = ' (!) Unavailable'
class PlanningSetupFormController:




    def __init__(self, datastore):
        assert isinstance(datastore, Datastore)
        self.__datastore = datastore


    def on_apply(self, button, setup_form, planning_wait_window, state_pool_string,
                            type_db_path,planner_text,planner_script_path,planner_argv_text,
                            facts_path,sm_name,sm_save_dir,keep_related_files, file_save_dir,
                                                                    rt_data_path, as_reference):
        #prepare datastore with new data from dialog.
        #save datastore to configuration file.
        #destroy dialog.
        #start the pipeline to generate a sm.
        everything_filled, not_filled = self.__prepare_datastore(self.__datastore, state_pool_string,
                            type_db_path,planner_text,planner_script_path,planner_argv_text,
                            facts_path,sm_name,sm_save_dir,keep_related_files, file_save_dir,
                                                                    rt_data_path, as_reference)

        if everything_filled:
            self.__datastore.validate_ds()
            self.__datastore.save_datastore_parts_in_file(DATASTORE_STORAGE_PATH)
            setup_form.hide()#its more smoothly to first hide and then destroy
            planning_wait_window.show()
            setup_form.destroy()
            from rafcontpp.view.planning_button import increment_button
            increment_button()#increment the button, to indecate that a new planning process has started.
            #start pipeline
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

    def __wait_and_hide(self,thread, planning_wait_window):
        '''
        wait and hide should be executed in another thread, it joins the planning thread, closes the wait window
        and decrements the planning button.
        :param thread: the thread to wait for
        '''
        #logger.debug('wait_and_hide executed from thread: {}'.format(threading.current_thread().getName()))#todo remove
        if thread and thread.is_alive():
            thread.join()
        call_gui_callback(planning_wait_window.hide)#its more smoothly to first hide and then destroy
        call_gui_callback(planning_wait_window.destroy)
        from rafcontpp.view.planning_button import decrement_button
        call_gui_callback(decrement_button)# decrement button, to indicate, that the planning process is finish.


    def on_destroy(self, button, setup_form, state_pool_string,
                            type_db_path,planner_text,planner_script_path,planner_argv_text,
                            facts_path,sm_name,sm_save_dir,keep_related_files, file_save_dir,
                                                                    rt_data_path, as_reference):
        #destroy dialog
        #save data to datastore
        #save datastore to file.
        try:
            self.__prepare_datastore(self.__datastore,state_pool_string,
                            type_db_path,planner_text,planner_script_path,planner_argv_text,
                            facts_path,sm_name,sm_save_dir,keep_related_files, file_save_dir,
                                                                    rt_data_path, as_reference)
            self.__datastore.save_datastore_parts_in_file(DATASTORE_STORAGE_PATH)
        finally:
            setup_form.destroy()


    #start=========================================================================================
    def on_show_data_info(self, button, call_back, state_pool_string,
                          type_db_path, planner_text, planner_script_path, planner_argv_text,
                          facts_path, sm_name, sm_save_dir, keep_related_files, file_save_dir,
                          rt_data_path, as_reference):
        available_predicates = []
        predicates_string = ''
        type_string = ''
        action_string = ''
        tmp_datastore = Datastore(None,None,None,None,None,None,None,None,None,None,)
        self.__prepare_datastore(tmp_datastore,state_pool_string,type_db_path,planner_text,planner_script_path,
                                 planner_argv_text,facts_path,sm_name,sm_save_dir,False,None,rt_data_path,True)
        merge_preds = True

        try:
            mapper = Mapper(tmp_datastore)
            mapper.generate_action_state_map()
            mapper.generate_state_action_map()
            mapper.generate_available_actions()
            loader = PddlActionLoader(tmp_datastore)
            loader.load_pddl_actions()
            type_merger = TypeMerger(tmp_datastore)
            type_tree = type_merger.merge_types()
            type_string = type_tree.get_as_string()
        except LookupError as e:
            merge_preds = False
            type_string = 'ERROR!: {}'.format(e.message)

        action_names = []
        for state in tmp_datastore.get_pddl_action_map():
            action = tmp_datastore.get_pddl_action_map()[state]
            action_names.append("{} (in state {})".format(action.name,state))
            for predicate in action.predicates:
                if predicate not in available_predicates:
                    available_predicates.append(predicate)

        if merge_preds:
            pred_merger = PredicateMerger(tmp_datastore)
            available_predicates = pred_merger.merge_predicates(available_predicates)[0]


        for predicate in available_predicates:
            predicates_string += predicate+'\r\n'

        for action_name in sorted(action_names):
            action_string+= action_name+'\r\n'


        call_back(predicates_string,type_string, action_string)

    #end===========================================================================================

    def on_choose_state_pool(self,chooser, chooser_entry):
        #append choosen state pool to state pool text entry.
        to_append = chooser.get_filename()
        pools = chooser_entry.get_text()
        if len(pools)> 0 and pools[len(pools)-1] != ':':
            pools+=':'
        chooser_entry.set_text(pools + to_append + ':')




    def __prepare_datastore(self, datastore_to_prepare, state_pool_string,
                            type_db_path,planner_text,planner_script_path,planner_argv_text,
                            facts_path,sm_name,sm_save_dir,keep_related_files, file_save_dir,
                                                                    rt_data_path, as_reference):
        #saves all data from the dialog into the datastore.
        #looks if everything necessary was filled.
        dtp = datastore_to_prepare
        everything_filled = True
        not_filled = None
        logger.debug('State pool: ' + str(self.__string_to_string_array(state_pool_string)))
        dtp.add_state_pools(self.__string_to_string_array(state_pool_string),True)
        dtp.set_type_db_path(type_db_path)
        choosen_planner = planner_text.replace(NOT_AVAILABLE,'')
        #set planner
        script_path = planner_script_path
        if choosen_planner == OTHER:
            choosen_planner = script_path
        if choosen_planner == SEL_PLANNER:
            everything_filled = False
            not_filled = 'a Planner'
        if choosen_planner != SEL_PLANNER:
            dtp.set_planner(choosen_planner)
        dtp.set_planner_script_path(script_path)
        #set planner argv
        if len(planner_argv_text) > 0:
            dtp.set_planner_argv(planner_argv_text.split(' '))
        else:
            dtp.set_planner_argv([])

        dtp.set_facts_path(facts_path)
        dtp.set_sm_name(sm_name)
        dtp.set_sm_save_dir(sm_save_dir)
        dtp.set_keep_related_files(keep_related_files)
        if dtp.keep_related_files:
            dtp.set_file_save_dir(file_save_dir)

        #runtime section
        runtime_data_path = rt_data_path.strip()
        dtp.set_use_runtime_path_as_ref(as_reference)
        dtp.set_runtime_data_path(runtime_data_path)
        if not dtp.use_runtime_path_as_ref() and runtime_data_path and len(runtime_data_path)>0:
            if not os.path.isfile(dtp.get_runtime_data_path()):
                everything_filled = False
                not_filled = 'Runtime Data contains no valid Filepath!'


        return (everything_filled,not_filled)


    def __string_to_string_array(self,string):
        # helper method for state pool text entry
        return list(filter(None,string.split(':')))
