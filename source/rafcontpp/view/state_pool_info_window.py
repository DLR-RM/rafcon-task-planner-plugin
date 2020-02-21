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
# Version 14.06.2019
import os
import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk


class StatePoolInfoWindow:
    """
    The State Pool Infow Window is a info window displaying detailed information about the configured state pool(s)
    It for example shows the actions contained in the pools, the types used and the predicates available.
    """

    def __init__(self, parent):
        """
        initializes the State Pool Info window.

        :param parent: the parent window. e.g. a Gtk.Dialog
        :return: void
        """
        glade_path = os.path.abspath(
            os.path.join(os.path.dirname(os.path.realpath(__file__)), "glade", "state_pool_info_window.glade"))
        self.__window_builder = Gtk.Builder()
        self.__window_builder.add_from_file(glade_path)
        self.__state_pool_info_dialog = self.__window_builder.get_object('rtpp_data_info_view_window')
        self.__state_pool_info_dialog.set_title('State Pool Info')
        self.__state_pool_info_dialog.set_transient_for(parent)
        self.__state_pool_info_dialog.set_position(Gtk.WindowPosition.CENTER_ALWAYS)
        self.__state_pool_info_dialog.set_modal(parent)
        self.__state_pool_info_dialog.set_size_request(*parent.get_size())
        window_button = self.__window_builder.get_object('rtpp_predicates_view_close_button')
        window_button.connect('clicked', lambda x: self.__state_pool_info_dialog.destroy())

    def set_state_pools(self, state_pools):
        """
        Sets the statepools in the info window.

        :param statepools: a list containing all selected state pools.
        :return: void
        """
        state_pool_string = ''
        if state_pools and len(state_pools) > 0:
            state_pool_string = self.__list_to_multi_line_string(sorted(state_pools))
        state_pool_label = self.__window_builder.get_object('rtpp_data_info_view_state_pools_label')
        state_pool_label.set_text(state_pool_string)
        state_pool_label.set_alignment(0, 0)
        state_pool_label.set_selectable(True)

    def set_predicates(self, predicates):
        """
        Sets the predicates list as string into the info window.

        :param predicates: A list of predicate Strings.
        :return: void
        """
        predicate_string = ''
        if predicates and len(predicates) > 0:
            predicate_string = self.__list_to_multi_line_string(sorted(predicates))
        predicate_label = self.__window_builder.get_object('rtpp_data_info_view_predicates_label')
        predicate_label.set_text(predicate_string)
        predicate_label.set_alignment(0, 0)
        predicate_label.set_selectable(True)

    def set_types(self, type_tree):
        """
        Sets the type tree into the info window.

        :param types: A type_tree.
        :return: void
        """
        types_string = ''
        if type_tree:
            processing_list = type_tree.get_as_list()
            current_parent = None
            for type_name in processing_list:
                parent = type_tree.get_parent_of(type_name)
                if parent:
                    types_string += "{} extends {} \r\n".format(type_name, parent)
                else:
                    types_string += "{}\r\n".format(type_name)
        types_label = self.__window_builder.get_object('rtpp_data_info_view_type_label')
        types_label.set_text(types_string)
        types_label.set_alignment(0, 0)
        types_label.set_selectable(True)

    def set_action_state_mapping(self, action_state_map):
        """
        Sets the action state mapping into the info window.

        :param action_state_map: a map, containing action names as key and state names as values.
        :return: void
        """
        asm_string = ''
        if action_state_map:
            as_list = ["Action <----> State ({}) ".format(len(action_state_map.keys())), ""]
            for action in sorted(action_state_map.keys()):
                as_list.append("{} <----> {}".format(action, action_state_map[action]))
            asm_string = self.__list_to_multi_line_string(as_list)
        action_label = self.__window_builder.get_object('rtpp_data_info_view_action_state_label')
        action_label.set_text(asm_string)
        action_label.set_alignment(0, 0)
        action_label.set_selectable(True)

    def show(self):
        """
        shows the dialog.

        :return: void
        """
        self.__state_pool_info_dialog.show_all()

    def destroy(self):
        """
        destroys the dialog.

        :return: void
        """
        self.__state_pool_info_dialog.destroy()

    def __list_to_multi_line_string(self, list):
        """
        :param list: a list of elements
        :return: String: A string with each element in a new line.
        """
        result_string = ''
        if list and len(list) > 0:
            result_string = os.linesep.join(map(str, list))
        return result_string
