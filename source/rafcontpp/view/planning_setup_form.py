import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk
from gi.repository import GObject
import os
from rafcontpp.controll.execution_controller import ExecutionController
from rafcontpp.model.datastore import Datastore, DATASTORE_STORAGE_PATH
from rafcon.utils import log

logger = log.get_logger(__name__)
OTHER = 'Other...'
SEL_PLANNER = '--select planner--'

class PlanningSetupForm:

    __dialog = None


    def __init__(self, datastore):
        self.__datastore = datastore
        self.__builder = Gtk.Builder()

    def initialize(self):
        glade_path = os.path.abspath(os.path.join(os.path.dirname(os.path.realpath(__file__)), "glade", "planning_setup_form.glade"))
        self.__builder.add_from_file(glade_path)
        #get items
        self.__dialog = self.__builder.get_object('plannig_setup_form_dialog')
        self.__dialog.set_title('Task Planner Plugin Configuration')
        self.__dialog.set_transient_for()
        state_pool_chooser = self.__builder.get_object('state_pools_chooser')
        action_pool_chooser = self.__builder.get_object('action_pools_chooser')
        type_db_chooser = self.__builder.get_object('type_db_chooser')
        planner_dropdown = self.__builder.get_object('planner_dropdown')
        script_path_chooser = self.__builder.get_object('script_path_chooser')
        planner_argv_entry = self.__builder.get_object('planner_argv_entry')
        facts_file_chooser = self.__builder.get_object('facts_file_chooser')
        sm_save_dir = self.__builder.get_object('sm_save_dir_chooser')
        keep_related_files = self.__builder.get_object('keep_produced_files_checkbox')
        file_save_dir = self.__builder.get_object('file_save_dir_chooser')
        #init items
        state_pool_chooser.set_filename(self.__datastore.get_state_pools()[0])
        action_pool_chooser.set_filename(self.__datastore.get_action_pools()[0])
        type_db_chooser.set_filename(self.__datastore.get_type_db_path())
        self.__init_drop_down(planner_dropdown, script_path_chooser)
        planner_argv_entry.set_text(''.join(e+" " for e in self.__datastore.get_planner_argv()).rstrip())
        facts_file_chooser.set_filename(self.__datastore.get_facts_path())
        sm_save_dir.set_filename(self.__datastore.get_sm_save_dir())
        keep_related_files.set_active(self.__datastore.keep_related_files())
        file_save_dir.set_filename(self.__datastore.get_file_save_dir())
        self.__dialog.show_all()
        self.__builder.get_object('planning_form_start_button').connect('clicked', self.__on_apply)
        self.__builder.get_object('planning_form_cancel_button').connect('clicked', self.__on_destroy)
        script_path_chooser.connect('file-set', lambda x: (planner_dropdown.set_active(len(planner_dropdown.get_model()) - 1)))






    def __init_drop_down(self,drop_down, script_path_chooser):
        active_index = 0

        drop_down.append_text(SEL_PLANNER)

        for index, planner in enumerate(self.__datastore.get_built_in_planners().keys()):
            drop_down.append_text( planner)
            if planner == self.__datastore.get_planner():
                active_index = index +1

            drop_down.append_text(OTHER)

        if active_index == 0 and self.__datastore.get_planner() is not None and len(self.__datastore.get_planner()) > 0:
            active_index = len(drop_down.get_model()) - 1

        script_path_chooser.set_filename(self.__datastore.get_planner_script_path())

        drop_down.set_active(active_index)





    def __on_apply(self, button):
        everything_filled, not_filled = self.__prepare_datastore()

        if everything_filled:
            self.__builder.get_object('plannig_setup_form_dialog').destroy()
            self.__datastore.save_datastore_parts_in_file(DATASTORE_STORAGE_PATH)
            logger.info("start pipeline...")
            ExecutionController(self.__datastore).on_execute()
        else:
            logger.error("Field "+not_filled+" missing!")


    def __on_destroy(self, button):
        try:
            self.__prepare_datastore()
            self.__datastore.save_datastore_parts_in_file(DATASTORE_STORAGE_PATH)
        finally:
            self.__dialog.destroy()





    def __prepare_datastore(self):
        everything_filled = True
        not_filled = None
        self.__datastore.add_state_pools(self.__builder.get_object('state_pools_chooser').get_filename())
        self.__datastore.add_action_pools(self.__builder.get_object('action_pools_chooser').get_filename())
        self.__datastore.set_type_db_path(self.__builder.get_object('type_db_chooser').get_filename())
        choosen_planner = self.__builder.get_object('planner_dropdown').get_active_text()
        #set planner
        script_path = self.__builder.get_object('script_path_chooser').get_filename()
        if choosen_planner == OTHER:
            choosen_planner = script_path
        if choosen_planner == SEL_PLANNER:
            everything_filled = False
            not_filled = 'Planner'
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
        self.__datastore.set_sm_save_dir(self.__builder.get_object('sm_save_dir_chooser').get_filename())
        self.__datastore.set_keep_related_files(self.__builder.get_object('keep_produced_files_checkbox').get_active())
        if self.__datastore.keep_related_files:
            self.__datastore.set_file_save_dir(self.__builder.get_object('file_save_dir_chooser').get_filename())

        return (everything_filled,not_filled)




