# Contributors:
# Christoph Suerig <christoph.suerig@dlr.de>
# Version 24.05.2019

import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk
from gi.repository import Gdk
import os
from threading import Thread
from rafcontpp.control.execution_controller import ExecutionController
from rafcontpp.model.datastore import Datastore, DATASTORE_STORAGE_PATH
from rafcontpp.control.planning_setup_form_controller import PlanningSetupFormController
from rafcontpp.control.planning_setup_form_controller import NOT_AVAILABLE, OTHER, SEL_PLANNER
from rafcon.utils import log
import rafcon.gui.singleton as gui_singletons
logger = log.get_logger(__name__)

class PlanningSetupForm:




    def __init__(self, datastore):
        assert isinstance(datastore, Datastore)
        self.__datastore = datastore
        self.__builder = Gtk.Builder()
        self.__dialog = None
        self.__state_pool_chooser_entry = None
        self.__controller = PlanningSetupFormController(datastore)
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
        self.__runtime_data_reference = self.__builder.get_object('rtpp_planning_setup-form_runtime_data_reference_radio')
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
            self.__runtime_data_reference.set_active(True)
        else:
            runtime_data_direct.set_active(True)
        self.__dialog.show_all()
        self.__builder.get_object('planning_form_start_button').connect('clicked', self.__call_controller_on_apply)
        self.__builder.get_object('planning_form_cancel_button').connect('clicked', self.__call_controller_on_destroy)
        self.__builder.get_object('planning_form_show_state_pool_info_button').connect('clicked', self.__call_controller_on_show_state_pool_info)
        state_pool_chooser.connect('file-set',self.__controller.on_choose_state_pool,self.__state_pool_chooser_entry)
        #automatically choose Other... if planner script is set.
        script_path_chooser.connect('file-set', lambda x: (planner_dropdown.set_active(len(planner_dropdown.get_model()) - 1)))


    def __call_controller_on_apply(self, button):
        '''
        this function is needed, to get the data when method is called, and not old data from declaration time.
        :param button:
        :return:
        '''

        self.__controller.on_apply(button, self.__dialog, self.__planning_wait_window,*self.__get_entered_data())

    def __call_controller_on_destroy(self, button):
        '''
        this function is needed, to get the data when method is called, and not old data from declaration time.
        :param button:
        :return:
        '''
        self.__controller.on_destroy(button, self.__dialog, *self.__get_entered_data())

    def __call_controller_on_show_state_pool_info(self, button):
        '''
        this function is needed, to get the data when method is called, and not old data from declaration time.
        :param button:
        :return:
        '''
        self.__controller.on_show_state_pool_info(button, self.__dialog, *self.__get_entered_data())


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


    def __get_entered_data(self):
        state_pool_text = self.__state_pool_chooser_entry.get_text()
        type_db_path = self.__builder.get_object('type_db_chooser').get_filename()
        planner_text = self.__builder.get_object('planner_dropdown').get_active_text()
        planner_script_path = self.__builder.get_object('script_path_chooser').get_filename()
        planner_argv = self.__builder.get_object('planner_argv_entry').get_text()
        facts_path = self.__builder.get_object('facts_file_chooser').get_filename()
        sm_name = self.__builder.get_object('rtpp_sm_name_entry').get_text()
        sm_save_dir = self.__builder.get_object('sm_save_dir_chooser').get_filename()
        keep_related_files = self.__builder.get_object('keep_produced_files_checkbox').get_active()
        file_save_dir = self.__builder.get_object('file_save_dir_chooser').get_filename()
        rt_data_path = self.__builder.get_object('rtpp_planning_setup_form_runtime_data_path_entry').get_text()
        as_reference = self.__runtime_data_reference.get_active()
        return (state_pool_text, type_db_path, planner_text, planner_script_path, planner_argv,
                facts_path, sm_name, sm_save_dir, keep_related_files, file_save_dir, rt_data_path, as_reference)


    def __string_array_to_string(self,list):
        #helper method for state pool text entry
            toReturn = ''

            for element in list:
                toReturn += element +':'

            return toReturn

