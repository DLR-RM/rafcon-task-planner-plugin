# Contributors:
# Christoph Suerig <christoph.suerig@dlr.de>
# Version 14.06.2019
import gi

gi.require_version('Gtk', '3.0')
from gi.repository import Gtk
import os


class StatePoolInfoWindow:
    """
    The State Pool Infow Window is a info window displaying detailed information about the configured state pool(s)
    It for example shows the actions contained in the pools, the types used and the predicates available.
    """

    def __init__(self, parent):
        """
        initializes the State Pool Info window.
        :param parent: the parent window. e.g. a Gtk.Dialog
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
        :param predicates: A list of predicate Strings
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
        """
        types_string = ''
        if type_tree:
            types_string = type_tree.get_as_pddl_string()

        types_label = self.__window_builder.get_object('rtpp_data_info_view_type_label')
        types_label.set_text(types_string)
        types_label.set_alignment(0, 0)
        types_label.set_selectable(True)

    def set_action_state_mapping(self, action_state_map):
        """
        Sets the action state mapping into the info window.
        :param action_state_map: a map, containing action names as key and state names as values.
        """

        asm_string = ''

        if action_state_map:
            as_list = ["Action <----> State ", ""]
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

        """
        self.__state_pool_info_dialog.show_all()

    def destroy(self):
        """
        destroys the dialog.
        """
        self.__state_pool_info_dialog.destroy()

    def __list_to_multi_line_string(self, list):
        """

        :param list: a list of elements
        :return: A string with each element in a new line.
        """
        result_string = ''
        if list and len(list) > 0:
            result_string = os.linesep.join(map(str, list))
        return result_string
