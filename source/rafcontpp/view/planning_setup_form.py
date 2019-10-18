# Copyright (C) 2018-2019 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the 3-Clause BSD License which accompanies this
# distribution, and is available at
# https://opensource.org/licenses/BSD-3-Clause
#
# Contributors:
# Christoph Suerig <christoph.suerig@dlr.de>

# Don't connect with the Copyright comment above!
# Version 20.08.2019

import os
import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk
import rafcon.gui.singleton as gui_singletons
from rafcon.utils import log
from rafcontpp.model.datastore import Datastore
from rafcontpp.control.planning_setup_form_controller import PlanningSetupFormController
from rafcontpp.control.planning_setup_form_controller import NOT_AVAILABLE, OTHER, SEL_PLANNER



logger = log.get_logger(__name__)


class PlanningSetupForm:

    def __init__(self, datastore):
        assert isinstance(datastore, Datastore)
        self.__datastore = datastore
        self.__builder = Gtk.Builder()
        self.__dialog = None
        self.__state_pool_chooser_entry = None
        self.__runtime_data_reference = None
        self.__controller = PlanningSetupFormController(datastore)

    def initialize(self):
        """
        initialize initiates the components with data present in the datastore, also it adds listeners for
        each part e.g. a file chooser.

        :return: void
        """
        glade_path = os.path.abspath(
            os.path.join(os.path.dirname(os.path.realpath(__file__)), "glade", "planning_setup_form.glade"))
        self.__builder.add_from_file(glade_path)
        # get items
        self.__dialog = self.__builder.get_object('plannig_setup_form_dialog')
        self.__dialog.set_title('Task Planner Plugin Configuration')
        main_window = gui_singletons.main_window_controller.view['main_window']
        self.__dialog.set_transient_for(main_window)
        state_pool_chooser = self.__builder.get_object('state_pools_chooser')
        self.__state_pool_chooser_entry = self.__builder.get_object('state_pools_chooser_entry')
        type_db_chooser = self.__builder.get_object('type_db_chooser')
        planner_dropdown = self.__builder.get_object('planner_dropdown')
        script_path_chooser = self.__builder.get_object('script_path_chooser')
        planner_argv_entry = self.__builder.get_object('planner_argv_entry')
        facts_file_chooser = self.__builder.get_object('facts_file_chooser')
        self.__sm_into_selected_state = self.__builder.get_object('rtpp_planning_setup_form_selected_state')
        sm_into_independent_sm = self.__builder.get_object('rtpp_planning_setup-form_independent_sm_radio')
        sm_name_entry = self.__builder.get_object('rtpp_sm_name_entry')
        sm_save_dir = self.__builder.get_object('sm_save_dir_chooser')
        keep_related_files = self.__builder.get_object('keep_produced_files_checkbox')
        file_save_dir = self.__builder.get_object('file_save_dir_chooser')
        runtime_data_field = self.__builder.get_object('rtpp_planning_setup_form_runtime_data_path_entry')
        runtime_data_chooser = self.__builder.get_object('rtpp_planning_setup_form_runtime_data_file_chooser')
        runtime_data_direct = self.__builder.get_object('rtpp_planning_setup_form_runtime_data_direct_radio')
        self.__runtime_data_reference = self.__builder.get_object(
            'rtpp_planning_setup-form_runtime_data_reference_radio')
        # init items
        state_pool_chooser.set_filename(self.__datastore.get_state_pools()[0])
        self.__state_pool_chooser_entry.set_text(self.__string_array_to_string(self.__datastore.get_state_pools()))
        type_db_chooser.set_filename(self.__datastore.get_type_db_path())
        self.__init_drop_down(planner_dropdown, script_path_chooser)
        planner_argv_entry.set_text(''.join(e + " " for e in self.__datastore.get_planner_argv()).rstrip())
        facts_file_chooser.set_filename(self.__datastore.get_facts_path())
        self.__sm_into_selected_state.set_active(self.__datastore.generate_into_state())
        sm_into_independent_sm.set_active(not self.__datastore.generate_into_state())
        sm_name_entry.set_text(self.__datastore.get_sm_name())
        sm_save_dir.set_filename(self.__datastore.get_sm_save_dir())
        keep_related_files.set_active(self.__datastore.keep_related_files())
        keep_related_files_active = self.__datastore.keep_related_files()
        self.__datastore.set_keep_related_files(True)
        file_save_dir.set_filename(self.__datastore.get_file_save_dir())
        self.__datastore.set_keep_related_files(keep_related_files_active)
        # initialize runtime data section
        if self.__datastore.get_runtime_data_path():
            runtime_data_path = self.__datastore.get_runtime_data_path()
            runtime_data_field.set_text(runtime_data_path)
            if os.path.isfile(runtime_data_path):
                runtime_data_chooser.set_filename(runtime_data_path)
            self.__runtime_data_reference.set_active(self.__datastore.use_runtime_path_as_ref())
            runtime_data_direct.set_active(not self.__datastore.use_runtime_path_as_ref())
        self.__dialog.show_all()
        # connect
        self.__builder.get_object('planning_form_start_button').connect('clicked', self.__call_controller_on_apply)
        self.__builder.get_object('planning_form_cancel_button').connect('clicked', self.__call_controller_on_destroy)
        self.__builder.get_object('planning_form_show_state_pool_info_button').connect('clicked',
                                                                                       self.__call_controller_on_show_state_pool_info)
        state_pool_chooser.connect('file-set', self.__controller.on_choose_state_pool, self.__state_pool_chooser_entry)
        runtime_data_chooser.connect('file-set', self.__controller.on_choose_runtime_data, runtime_data_field)
        # automatically choose Other... if planner script is set.
        script_path_chooser.connect('file-set',
                                    lambda x: (planner_dropdown.set_active(len(planner_dropdown.get_model()) - 1)))

    def __call_controller_on_apply(self, button):
        """
        This function is needed, to get the data when method is called, and not old data from declaration time.

        :param button: Unused.
        :return: void
        """
        self.__controller.on_apply(button, self.__dialog, *self.__get_entered_data())

    def __call_controller_on_destroy(self, button):
        """
        This function is needed, to get the data when method is called, and not old data from declaration time.

        :param button: Unused
        :return: void
        """
        self.__controller.on_destroy(button, self.__dialog, *self.__get_entered_data())

    def __call_controller_on_show_state_pool_info(self, button):
        """
        This function is needed, to get the data when method is called, and not old data from declaration time.

        :param button: Unused
        :return: void
        """
        self.__controller.on_show_state_pool_info(button, self.__dialog, *self.__get_entered_data())

    def __init_drop_down(self, drop_down, script_path_chooser):
        """
        Initializes the planner drop down menu.
        """
        # initiates the planner drop down with all built in planners and the script path chooser for the planenr script
        # look if planner is available
        active_index = 0
        drop_down.append_text(SEL_PLANNER)
        for index, planner in enumerate(self.__datastore.get_built_in_planners().keys()):
            # dynamically import and check if planner is available.
            to_import = self.__datastore.get_built_in_planners()[planner]
            script_import = __import__(to_import[0], fromlist=(to_import[1]))
            if getattr(script_import, to_import[1])().is_available():
                drop_down.append_text(planner)  # add planner to dropdown if available
            else:
                drop_down.append_text(planner + NOT_AVAILABLE)  # also add if not availavle, but with a hint.
            # set active planner to last used planner
            if planner == self.__datastore.get_planner():
                active_index = index + 1
        drop_down.append_text(OTHER)
        # set active planner to Other if script was used last.
        if active_index == 0 and self.__datastore.get_planner() is not None and len(self.__datastore.get_planner()) > 0:
            active_index = len(drop_down.get_model()) - 1
        # initiate planner script field.
        script_path_chooser.set_filename(self.__datastore.get_planner_script_path())
        drop_down.set_active(active_index)

    def __get_entered_data(self):
        """
        Reads entered data from the planning setup form, and returns the raw values.

        :return: (String, String, String, String, String, String, Boolean, String, String, Boolean, String, String, Boolean): The raw values.
        """
        state_pool_text = self.__state_pool_chooser_entry.get_text()
        type_db_path = self.__builder.get_object('type_db_chooser').get_filename()
        planner_text = self.__builder.get_object('planner_dropdown').get_active_text()
        planner_script_path = self.__builder.get_object('script_path_chooser').get_filename()
        planner_argv = self.__builder.get_object('planner_argv_entry').get_text()
        facts_path = self.__builder.get_object('facts_file_chooser').get_filename()
        generate_into_state = self.__sm_into_selected_state.get_active()
        sm_name = self.__builder.get_object('rtpp_sm_name_entry').get_text()
        sm_save_dir = self.__builder.get_object('sm_save_dir_chooser').get_filename()
        keep_related_files = self.__builder.get_object('keep_produced_files_checkbox').get_active()
        file_save_dir = self.__builder.get_object('file_save_dir_chooser').get_filename()
        rt_data_path = self.__builder.get_object('rtpp_planning_setup_form_runtime_data_path_entry').get_text()
        as_reference = self.__runtime_data_reference.get_active()
        return (state_pool_text, type_db_path, planner_text, planner_script_path, planner_argv,
                facts_path, generate_into_state, sm_name, sm_save_dir, keep_related_files,
                file_save_dir, rt_data_path, as_reference)

    def __string_array_to_string(self, list):
        # helper method for state pool text entry
        toReturn = ''
        for element in list:
            toReturn += element + ':'
        return toReturn
