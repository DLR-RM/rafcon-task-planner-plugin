# Contributors:
# Christoph Suerig <christoph.suerig@dlr.de>
# Version 17.05.2019


import os
import threading
from threading import Thread
from rafcon.utils.gui_functions import call_gui_callback
from rafcontpp.control.execution_controller import ExecutionController
from rafcontpp.model.datastore import Datastore, DATASTORE_STORAGE_PATH
from rafcontpp.view.planning_wait_window import PlanningWaitWindow
from rafcon.utils import log

logger = log.get_logger(__name__)
#other string, if other planner is choosen.
OTHER = 'Other...'
#select planner string, if nothing is choosen.
SEL_PLANNER = '-- Select planner --'
#printed next to planner, if it is not available.
NOT_AVAILABLE = ' (!) Unavailable'
class PlanningSetupFormController:
    '''
    PlanningSetupFormController
    This is the controller of the Planning Setup Form.
    '''




    def __init__(self, datastore):
        '''

        :param datastore: a Datastore
        '''
        assert isinstance(datastore, Datastore)
        self.__datastore = datastore


    def on_apply(self, button, setup_form, state_pool_string,
                            type_db_path,planner_text,planner_script_path,planner_argv_text,
                            facts_path,sm_name,sm_save_dir,keep_related_files, file_save_dir,
                                                                    rt_data_path, as_reference):
        '''
        on_apply filles the datastore with new data entered into the setup form, saves it in the configuration,
        destroys the setup form, triggeres the pipeline e.g. the execution controller, and increments the plan task
        button, to indicate a running task.
        :param button: Not used.
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
        '''

        #prepare datastore with new data from dialog.
        #save datastore to configuration file.
        #destroy dialog.
        #start the pipeline to generate a sm.
        everything_filled, not_filled = self.__prepare_datastore(state_pool_string,
                            type_db_path,planner_text,planner_script_path,planner_argv_text,
                            facts_path,sm_name,sm_save_dir,keep_related_files, file_save_dir,
                                                                    rt_data_path, as_reference)

        if everything_filled:
            self.__datastore.validate_ds()
            self.__datastore.save_datastore_parts_in_file(DATASTORE_STORAGE_PATH)
            setup_form.hide()#its more smoothly to first hide and then destroy
            planning_wait_window = PlanningWaitWindow()
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



    def on_destroy(self, button, setup_form, state_pool_string,
                            type_db_path,planner_text,planner_script_path,planner_argv_text,
                            facts_path,sm_name,sm_save_dir,keep_related_files, file_save_dir,
                                                                    rt_data_path, as_reference):
        '''
        on_destroy destroys the setup form, but saves the current configuration into a file.
        :param button: Not used.
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
        '''
        #destroy dialog
        #save data to datastore
        #save datastore to file.
        try:
            self.__prepare_datastore(state_pool_string,
                            type_db_path,planner_text,planner_script_path,planner_argv_text,
                            facts_path,sm_name,sm_save_dir,keep_related_files, file_save_dir,
                                                                    rt_data_path, as_reference)
            self.__datastore.save_datastore_parts_in_file(DATASTORE_STORAGE_PATH)
        finally:
            setup_form.destroy()


    def on_choose_state_pool(self,chooser, chooser_entry):
        '''
        Receives a directory chooser and a text entry, reads the path, set in the chooser and appends it to the text of
        the text entry.
        :param chooser: a GtkFileChooserButton
        :param chooser_entry:  a GtkEntry
        '''
        #append choosen state pool to state pool text entry.
        to_append = chooser.get_filename()
        pools = chooser_entry.get_text()
        if len(pools)> 0 and pools[len(pools)-1] != ':':
            pools+=':'
        chooser_entry.set_text(pools + to_append + ':')


    def on_choose_runtime_data(self,chooser,runtime_data_entry):
        '''
        Receivs a file chooser and a text entry, reads the filepath of the chooser and sets it as text into the entry.
        :param chooser: a GtkFileChooserButton
        :param runtime_data_entry: a GtkEntry
        '''
        runtime_data_entry.set_text(chooser.get_filename())




    def __prepare_datastore(self, state_pool_string,
                            type_db_path,planner_text,planner_script_path,planner_argv_text,
                            facts_path,sm_name,sm_save_dir,keep_related_files, file_save_dir,
                                                                    rt_data_path, as_reference):
        '''
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

        '''
        #saves all data from the dialog into the datastore.
        #looks if everything necessary was filled.
        everything_filled = True
        not_filled = None
        logger.debug('State pool: ' + str(self.__string_to_string_array(state_pool_string)))
        self.__datastore.add_state_pools(self.__string_to_string_array(state_pool_string),True)
        self.__datastore.set_type_db_path(type_db_path)
        choosen_planner = planner_text.replace(NOT_AVAILABLE,'')
        #set planner
        script_path = planner_script_path
        if choosen_planner == OTHER:
            choosen_planner = script_path
        if choosen_planner == SEL_PLANNER:
            everything_filled = False
            not_filled = 'a Planner'
        if choosen_planner != SEL_PLANNER:
            self.__datastore.set_planner(choosen_planner)
        self.__datastore.set_planner_script_path(script_path)
        #set planner argv
        if len(planner_argv_text) > 0:
            self.__datastore.set_planner_argv(planner_argv_text.split(' '))
        else:
            self.__datastore.set_planner_argv([])

        self.__datastore.set_facts_path(facts_path)
        self.__datastore.set_sm_name(sm_name)
        self.__datastore.set_sm_save_dir(sm_save_dir)
        self.__datastore.set_keep_related_files(keep_related_files)
        if self.__datastore.keep_related_files:
            self.__datastore.set_file_save_dir(file_save_dir)

        #runtime section
        runtime_data_path = rt_data_path.strip()
        self.__datastore.set_use_runtime_path_as_ref(as_reference)
        self.__datastore.set_runtime_data_path(runtime_data_path)
        if not self.__datastore.use_runtime_path_as_ref() and runtime_data_path and len(runtime_data_path)>0:
            if not os.path.isfile(self.__datastore.get_runtime_data_path()):
                everything_filled = False
                not_filled = 'Runtime Data contains no valid Filepath!'


        return (everything_filled,not_filled)

    def __wait_and_hide(self, thread, planning_wait_window):
        '''
        wait and hide should be executed in another thread, it joins the planning thread, closes the wait window
        and decrements the planning button.
        :param thread: the thread to wait for
        '''
        # logger.debug('wait_and_hide executed from thread: {}'.format(threading.current_thread().getName()))#todo remove
        if thread and thread.is_alive():
            thread.join()
        call_gui_callback(planning_wait_window.hide)  # its more smoothly to first hide and then destroy
        call_gui_callback(planning_wait_window.destroy)
        from rafcontpp.view.planning_button import decrement_button
        call_gui_callback(decrement_button)  # decrement button, to indicate, that the planning process is finish.


    def __string_to_string_array(self,string):
        '''
        a little method, that splits a colon seperated string into a string array.
        :param string: a string, containing colon seperated substrings.
        :return: a list of substrings e.g. a:b:c --> [a,b,c]
        '''
        result = []
        if string and len(string) >0:
            result = list(filter(None,string.split(':')))
        return result
