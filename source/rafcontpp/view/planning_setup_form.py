import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk
from gi.repository import GObject
import os
from rafcontpp.controll.execution_controller import ExecutionController
from rafcontpp.model.datastore import Datastore, DATASTORE_STORAGE_PATH
from rafcon.utils import log

logger = log.get_logger(__name__)


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
        self.__dialog.set_title('Rafcon Task Planner Plugin Configuration')
        self.__dialog.set_transient_for()
        state_pool_chooser = self.__builder.get_object('state_pools_chooser')
        action_pool_chooser = self.__builder.get_object('action_pools_chooser')
        type_db_chooser = self.__builder.get_object('type_db_chooser')
        planner_dropdown = self.__builder.get_object('planner_dropdown')
        script_path_chooser = self.__builder.get_object('script_path_chooser')
        facts_file_chooser = self.__builder.get_object('facts_file_chooser')
        sm_save_dir = self.__builder.get_object('sm_save_dir_chooser')
        keep_related_files = self.__builder.get_object('keep_produced_files_checkbox')
        file_save_dir = self.__builder.get_object('file_save_dir_chooser')
        #init items
        state_pool_chooser.set_filename(self.__datastore.get_state_pools()[0])
        action_pool_chooser.set_filename(self.__datastore.get_action_pools()[0])
        type_db_chooser.set_filename(self.__datastore.get_type_db_path())
        self.__init_drop_down(planner_dropdown, script_path_chooser)
        facts_file_chooser.set_filename(self.__datastore.get_facts_path())
        sm_save_dir.set_filename(self.__datastore.get_sm_save_dir())
        keep_related_files.set_active(self.__datastore.keep_related_files())
        if self.__datastore.keep_related_files():
            file_save_dir.set_filename(self.__datastore.get_file_save_dir())
        self.__dialog.show_all()
        self.__builder.get_object('planning_form_start_button').connect('clicked', self.__on_apply)
        self.__builder.get_object('planning_form_cancel_button').connect('clicked', self.__on_destroy)




    def __init_drop_down(self,drop_down, script_path_chooser):
        active_index = 0

        drop_down.append_text('--select planner--')

        for index, planner in enumerate(self.__datastore.get_built_in_planners().keys()):
            drop_down.append_text( planner)
            if planner == self.__datastore.get_planner():
                active_index = index +1

            drop_down.append_text('Other...')

        if active_index == 0 and self.__datastore.get_planner() is not None and len(self.__datastore.get_planner()) > 0:
            active_index = len(drop_down.get_model()) - 1
            script_path_chooser.set_filename(self.__datastore.get_planner())

        drop_down.set_active(active_index)





    def __on_apply(self, button):
        everything_filled, not_filled = self.__prepare_datastore()

        if everything_filled:
            self.__builder.get_object('plannig_setup_form_dialog').destroy()
            self.__datastore.save_datastore_parts_in_file(DATASTORE_STORAGE_PATH)
            logger.info("start pipeline...")
            ExecutionController(self.__datastore).on_execute()
        else:
            logger.error("Field "+ not_filled+" missing!")


    def __on_destroy(self, button):
        self.__prepare_datastore()
        self.__datastore.save_datastore_parts_in_file(DATASTORE_STORAGE_PATH)
        self.__dialog.destroy()


    def __prepare_datastore(self):
        everything_filled = True
        not_filled = None
        self.__datastore.add_state_pools(self.__builder.get_object('state_pools_chooser').get_filename())
        self.__datastore.add_action_pools(self.__builder.get_object('action_pools_chooser').get_filename())
        self.__datastore.set_type_db_path(self.__builder.get_object('type_db_chooser').get_filename())
        choosen_planner = self.__builder.get_object('planner_dropdown').get_active_text()
        if choosen_planner == 'Other...':
            choosen_planner = self.__builder.get_object('script_path_chooser').get_filename()
        if choosen_planner == '--select planner--':
            everything_filled = False
        self.__datastore.set_planner(choosen_planner)
        self.__datastore.set_facts_path(self.__builder.get_object('facts_file_chooser').get_filename())
        self.__datastore.set_sm_save_dir(self.__builder.get_object('sm_save_dir_chooser').get_filename())
        self.__datastore.set_keep_related_files(self.__builder.get_object('keep_produced_files_checkbox').get_active())
        if self.__datastore.keep_related_files:
            self.__datastore.set_file_save_dir(self.__builder.get_object('file_save_dir_chooser').get_filename())

        return (everything_filled,not_filled)



