import gtk
import os
from de.dlr.rmc.rafcontpp.model.datastore import Datastore
from de.dlr.rmc.rafcontpp.controll.execution_controller import ExecutionController

class PlanningSetupForm:


    def __init__(self, datastore):
        self.__datastore = datastore
        self.__builder = gtk.Builder()

    def initialize(self):
        glade_path = os.path.abspath(os.path.join(os.path.dirname(os.path.realpath(__file__)), "glade", "planning_setup_form.glade"))
        self.__builder.add_from_file(glade_path)
        #get items
        dialog = self.__builder.get_object('plannig_setup_form_dialog')
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
        state_pool_chooser.set_current_foler(self.__datastore.get_state_pools())
        action_pool_chooser.set_filenames(self.__datastore.get_action_pools())
        type_db_chooser.set_filename(self.__datastore.get_type_db_path())
        self.__init_drop_down(planner_dropdown, script_path_chooser)
        facts_file_chooser.set_filename(self.__datastore.get_facts_path())
        sm_save_dir.set_filename(self.__datastore.get_sm_save_dir())
        keep_related_files.set_active(self.__datastore.keep_related_files())
        if self.__datastore.keep_related_files():
            file_save_dir.set_filename(self.__datastore.get_file_save_dir())

        self.__builder.get_object('planning_form_start_button').connect('clicked', self.__on_apply())
        self.__builder.get_object('planning_form_cancel_button').connect('clicked', dialog.destroy())
        dialog.run()
        dialog.show_all()



    def __init_drop_down(self,drop_down, script_path_chooser):
        active_index = 0
        drop_down.append_text('--select planner--')
        for planner in self.__datastore.get_built_in_planners().keys():
            drop_down.append_text(planner)
        drop_down.append_text('Other...')

        for index, item in drop_down.ListStore:
            if item == self.__datastore.get_planner():
                active_index = index
                break;

        if active_index == 0 and self.__datastore.get_planner() is not None and len(self.__datastore.get_planner()) > 0:
            active_index = len(drop_down.ListStore) - 1
            script_path_chooser.set_filename(self.__datastore.get_planner())

        drop_down.set_active(active_index)





    def __on_apply(self):
        self.__datastore.set_state_pools(self.__builder.get_object('state_pools_chooser').get_filenames())
        self.__datastore.set_action_pools(self.__builder.get_object('action_pools_chooser'))
        self.__datastore.set_type_db_path(self.__builder.get_object('type_db_chooser').get_filename())
        choosen_planner = self.__builder.get_object('planner_dropdown').ActiveText
        if choosen_planner == 'other...':
            choosen_planner = self.__builder.get_object('script_path_chooser').get_filename()
        self.__datastore.set_planner(choosen_planner)
        self.__datastore.set_facts_path(self.__builder.get_object('facts_file_chooser').get_filename())
        self.__datastoredatastore.set_sm_save_dir(self.__builder.get_object('sm_save_dir_chooser').get_filename())
        self.__datastore.set_keep_related_files(self.__builder.get_object('keep_produced_files_checkbox').get_active())
        if self.__datastore.keep_related_files:
            self.__datastore.set_file_save_dir(self.__builder.get_object('file_save_dir_chooser').get_filename())

        self.__builder.get_object('plannig_setup_form_dialog').destroy()
        #TODO save datastore here...
        ExecutionController(self.__datastore).on_execute()



