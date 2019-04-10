# Contributors:
# Christoph Suerig <christoph.suerig@dlr.de>
# Version 08.03.2019

import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk
from gi.repository import Gdk
import os
from threading import Thread
from rafcontpp.control.execution_controller import ExecutionController
from rafcontpp.model.datastore import Datastore, DATASTORE_STORAGE_PATH
from rafcon.utils import log
import rafcon.gui.singleton as gui_singletons

logger = log.get_logger(__name__)
#other string, if other planner is choosen.
OTHER = 'Other...'
#select planner string, if nothing is choosen.
SEL_PLANNER = '-- Select planner --'
#printed next to planner, if it is not available.
NOT_AVAILABLE = ' (!) Unavailable'
class PlanningSetupForm:




    def __init__(self, datastore, ):
        assert isinstance(datastore, Datastore)
        self.__datastore = datastore
        self.__builder = Gtk.Builder()
        self.__dialog = None
        self.__state_pool_chooser_entry = None
        self.__planning_wait_window = self.__init_planning_wait_window()


    def initialize(self):
        '''
        initialize initiates the components with data present in the datastore, also it adds listeners for
        each part e.g. a file chooser.
        :return: nothing
        '''
        glade_path = os.path.abspath(os.path.join(os.path.dirname(os.path.realpath(__file__)), "glade", "planning_setup_form.glade"))
        self.__builder.add_from_file(glade_path)
        #get items
        self.__dialog = self.__builder.get_object('plannig_setup_form_dialog')
        self.__dialog.set_title('Task Planner Plugin Configuration')
        main_window = gui_singletons.main_window_controller.view['main_window']
        self.__dialog.set_transient_for(main_window)
        self.__dialog.set_modal(main_window)
        state_pool_chooser = self.__builder.get_object('state_pools_chooser')
        self.__state_pool_chooser_entry = self.__builder.get_object('state_pools_chooser_entry')
        type_db_chooser = self.__builder.get_object('type_db_chooser')
        planner_dropdown = self.__builder.get_object('planner_dropdown')
        script_path_chooser = self.__builder.get_object('script_path_chooser')
        planner_argv_entry = self.__builder.get_object('planner_argv_entry')
        facts_file_chooser = self.__builder.get_object('facts_file_chooser')
        sm_name_entry = self.__builder.get_object('rtpp_sm_name_entry')
        sm_save_dir = self.__builder.get_object('sm_save_dir_chooser')
        keep_related_files = self.__builder.get_object('keep_produced_files_checkbox')
        file_save_dir = self.__builder.get_object('file_save_dir_chooser')
        runtime_data_field = self.__builder.get_object('rtpp_planning_setup_form_runtime_data_path_entry')
        runtime_data_direct = self.__builder.get_object('rtpp_planning_setup_form_runtime_data_direct_radio')
        runtime_data_reference = self.__builder.get_object('rtpp_planning_setup-form_runtime_data_reference_radio')
        #init items
        state_pool_chooser.set_filename(self.__datastore.get_state_pools()[0])
        self.__state_pool_chooser_entry.set_text(self.__string_array_to_string(self.__datastore.get_state_pools()))
        type_db_chooser.set_filename(self.__datastore.get_type_db_path())
        self.__init_drop_down(planner_dropdown, script_path_chooser)
        planner_argv_entry.set_text(''.join(e+" " for e in self.__datastore.get_planner_argv()).rstrip())
        facts_file_chooser.set_filename(self.__datastore.get_facts_path())
        sm_name_entry.set_text(self.__datastore.get_sm_name())
        sm_save_dir.set_filename(self.__datastore.get_sm_save_dir())
        keep_related_files.set_active(self.__datastore.keep_related_files())
        file_save_dir.set_filename(self.__datastore.get_file_save_dir())
        if self.__datastore.get_runtime_data_path():
            runtime_data_field.set_text(self.__datastore.get_runtime_data_path())
        if self.__datastore.use_runtime_path_as_ref():
            runtime_data_reference.set_active(True)
        else:
            runtime_data_direct.set_active(True)
        self.__dialog.show_all()
        self.__builder.get_object('planning_form_start_button').connect('clicked', self.__on_apply)
        self.__builder.get_object('planning_form_cancel_button').connect('clicked', self.__on_destroy)
        state_pool_chooser.connect('file-set',self.__on_choose_state_pool)
        #automatically choose Other... if planner script is set.
        script_path_chooser.connect('file-set', lambda x: (planner_dropdown.set_active(len(planner_dropdown.get_model()) - 1)))




    def __init_planning_wait_window(self):
        '''
        a window that says, that planning needs a while...
        :return: the dialog window
        '''


        planning_wait_dialog_path = os.path.abspath(
            os.path.join(os.path.dirname(os.path.realpath(__file__)), "glade", "planning_wait_dialog.glade"))
        builder = Gtk.Builder()
        builder.add_from_file(planning_wait_dialog_path)
        planning_wait_dialog = builder.get_object('rtpp_planning_wait_window')
        planning_wait_dialog.set_title('Task Planner Plugin')
        main_window = gui_singletons.main_window_controller.view['main_window']
        planning_wait_dialog.set_transient_for(main_window)
        planning_wait_dialog.set_position(Gtk.WindowPosition.CENTER_ALWAYS)
        window_button = builder.get_object('rtpp_planning_wait_window_ok_button')
        window_button.connect('clicked', lambda x: planning_wait_dialog.destroy())
        return planning_wait_dialog



    def __init_drop_down(self,drop_down, script_path_chooser):
        #initiates the planner drop down with all built in planners and the script path chooser for the planenr script
        #look if planner is available
        active_index = 0

        drop_down.append_text(SEL_PLANNER)

        for index, planner in enumerate(self.__datastore.get_built_in_planners().keys()):
            #dynamically import and check if planner is available.
            to_import = self.__datastore.get_built_in_planners()[planner]
            script_import = __import__(to_import[0], fromlist=(to_import[1]))
            if getattr(script_import, to_import[1])().is_available():
                drop_down.append_text(planner) #add planner to dropdown if available
            else:
                drop_down.append_text(planner+ NOT_AVAILABLE) # also add if not availavle, but with a hint.
            #set active planner to last used planner
            if planner == self.__datastore.get_planner():
                active_index = index +1

        drop_down.append_text(OTHER)
        #set active planner to Other if script was used last.
        if active_index == 0 and self.__datastore.get_planner() is not None and len(self.__datastore.get_planner()) > 0:
            active_index = len(drop_down.get_model()) - 1
        #initiate planner script field.
        script_path_chooser.set_filename(self.__datastore.get_planner_script_path())
        drop_down.set_active(active_index)





    def __on_apply(self, button):
        #prepare datastore with new data from dialog.
        #save datastore to configuration file.
        #destroy dialog.
        #start the pipeline to generate a sm.
        everything_filled, not_filled = self.__prepare_datastore()

        if everything_filled:
            self.__datastore.validate_ds()
            self.__datastore.save_datastore_parts_in_file(DATASTORE_STORAGE_PATH)
            self.__planning_wait_window.show()
            self.__dialog.destroy()
            from rafcontpp.view.planning_button import increment_button
            increment_button()#increment the button, to indecate that a new planning process has started.
            #start pipeline
            logger.info("Start pipeline...")
            planning_thread = None
            try:
                planning_thread = ExecutionController(self.__datastore).on_execute_pre_planning()
            finally:
                Thread(target=self.__wait_and_hide, args=[planning_thread]).start()

        else:
            logger.error(" Field missing! {}".format(not_filled))

    def __wait_and_hide(self,thread):
        '''
        wait and hide should be executed in another thread, it joins the planning thread, closes the wait window
        and decrements the planning button.
        :param thread: the thread to wait for
        '''
        if thread:
            thread.join()
        Gdk.threads_enter()
        self.__planning_wait_window.destroy()
        from rafcontpp.view.planning_button import decrement_button
        decrement_button()# decrement button, to indicate, that the planning process is finish.
        Gdk.threads_leave()

    def __on_destroy(self, button):
        #destroy dialog
        #save data to datastore
        #save datastore to file.
        try:
            self.__prepare_datastore()
            self.__datastore.save_datastore_parts_in_file(DATASTORE_STORAGE_PATH)
        finally:
            self.__dialog.destroy()

    def __on_choose_state_pool(self,chooser):
        #append choosen state pool to state pool text entry.
        to_append = chooser.get_filename()
        pools = self.__state_pool_chooser_entry.get_text()
        if len(pools)> 0 and pools[len(pools)-1] != ':':
            pools+=':'
        self.__state_pool_chooser_entry.set_text(pools + to_append + ':')




    def __prepare_datastore(self):
        #saves all data from the dialog into the datastore.
        #looks if everything necessary was filled.
        everything_filled = True
        not_filled = None
        logger.debug('State pool: ' + str(self.__string_to_string_array(self.__state_pool_chooser_entry.get_text())))
        self.__datastore.add_state_pools(self.__string_to_string_array(self.__state_pool_chooser_entry.get_text()),True)
        self.__datastore.set_type_db_path(self.__builder.get_object('type_db_chooser').get_filename())
        choosen_planner = self.__builder.get_object('planner_dropdown').get_active_text().replace(NOT_AVAILABLE,'')
        #set planner
        script_path = self.__builder.get_object('script_path_chooser').get_filename()
        if choosen_planner == OTHER:
            choosen_planner = script_path
        if choosen_planner == SEL_PLANNER:
            everything_filled = False
            not_filled = 'a Planner'
        if choosen_planner != SEL_PLANNER:
            self.__datastore.set_planner(choosen_planner)
        self.__datastore.set_planner_script_path(script_path)
        planner_argv_text = self.__builder.get_object('planner_argv_entry').get_text()
        #set planner argv
        if len(planner_argv_text) > 0:
            self.__datastore.set_planner_argv(planner_argv_text.split(' '))
        else:
            self.__datastore.set_planner_argv([])

        self.__datastore.set_facts_path(self.__builder.get_object('facts_file_chooser').get_filename())
        self.__datastore.set_sm_name(self.__builder.get_object('rtpp_sm_name_entry').get_text())
        self.__datastore.set_sm_save_dir(self.__builder.get_object('sm_save_dir_chooser').get_filename())
        self.__datastore.set_keep_related_files(self.__builder.get_object('keep_produced_files_checkbox').get_active())
        if self.__datastore.keep_related_files:
            self.__datastore.set_file_save_dir(self.__builder.get_object('file_save_dir_chooser').get_filename())

        #runtime section

        runtime_data_path = self.__builder.get_object('rtpp_planning_setup_form_runtime_data_path_entry').get_text()
        runtime_data_path = runtime_data_path.strip()
        runtime_data_reference = self.__builder.get_object('rtpp_planning_setup-form_runtime_data_reference_radio')
        self.__datastore.set_use_runtime_path_as_ref(runtime_data_reference.get_active())
        self.__datastore.set_runtime_data_path(runtime_data_path)
        if not self.__datastore.use_runtime_path_as_ref() and runtime_data_path and len(runtime_data_path)>0:
            if not os.path.isfile(self.__datastore.get_runtime_data_path()):
                everything_filled = False
                not_filled = 'Runtime Data contains no valid Filepath!'

        return (everything_filled,not_filled)




    def __string_array_to_string(self,list):
        #helper method for state pool text entry
            toReturn = ''

            for element in list:
                toReturn += element +':'

            return toReturn

    def __string_to_string_array(self,string):
        # helper method for state pool text entry
        return list(filter(None,string.split(':')))
