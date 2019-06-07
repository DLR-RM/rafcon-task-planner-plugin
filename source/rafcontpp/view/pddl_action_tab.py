# Contributors:
# Christoph Suerig <christoph.suerig@dlr.de>
# Version 17.04.2019


import os
import gi
gi.require_version('Gtk', '3.0')
import time
from gi.repository import Gtk
from rafcontpp.model.datastore import SEMANTIC_DATA_DICT_NAME
from rafcontpp.logic.pddl_action_parser import PddlActionParser
from rafcontpp.logic.pddl_requirement_finder import PddlRequirementFinder
from rafcontpp.control.pddl_action_tab_controller import PddlActionTabController
from rafcon.core.states.library_state import LibraryState
from rafcon.utils import log
logger = log.get_logger(__name__)

#list with all pddl requirements
requ_list = [':strips', ':adl', ':typing', ':equality',
                ':negative-preconditions', ':disjunctive-preconditions', ':negative-preconditions',
                ':conditional-effects', ':existential-preconditions',
                ':universal-preconditions', ':derived-predicates',
                ':action-costs', ':quantified-preconditions',
                ':action-expansions', ':foreach-expansions',
                ':dag-expansions', ':expression-evaluation', ':fluents', ':open-world',':true-negation',
                'durative-actions', ':duration-inequalities', ':continous-effects']

class PddlActionTab:
    '''PddlActionTabController
    PddlActionTabController, controlles the gui elements of the pddl action tab for each state.
    it handles the dataflow between this tab, and the semantic section of the state, where the data
    of this tab is stored in. It also provides some auto fill wizzard for its elements.

    '''
    #true if auto save enabled
    auto_save_enabled = True
    #a list containing all auto apply buttons
    auto_save_check_buttons = []
    # a semaphore for the auto save handler function
    # this semaphore works as follows:
    # all auto apply check buttons of all action tabs are connected to the change signal, and stored in the auto_save_check_buttons array
    # if one tab is changed, all tabs are. the initiator checkbutton will propagate the changes to the others and
    # change their truth value.
    # (all buttons add 1 to the semaphore)
    # if the semaphores value is more then one, thats the indicator, that a button has to do nothing.
    # if the semaphore equals the length of the list, its the indicator, that this button is the last one,
    # it has to do nothing, despite resetting the semaphore.
    auto_save_semaphore = 0


    def __init__(self, state):
        '''

        :param action_tab_gtk_builder: gtk builder, contining its gui elements.
        :param state: the state, it belongs to.
        '''
        self.__state = state
        self.__controller = PddlActionTabController(state)
        glade_path = os.path.abspath(
            os.path.join(os.path.dirname(os.path.realpath(__file__)), "glade", "pddl_action_tab.glade"))
        builder = Gtk.Builder()
        builder.add_from_file(glade_path)
        self.__gtk_builder = builder
        # load action tab

        self.__description_text_view = self.__gtk_builder.get_object('description_textview')
        self.__pddl_action_source_view = self.__gtk_builder.get_object('pddl_action_sourceview')
        self.__pddl_predicates_text_view = self.__gtk_builder.get_object('pddl_predicates_textview')
        self.__pddl_types_text_view = self.__gtk_builder.get_object('pddl_types_textview')
        view_port = self.__gtk_builder.get_object('requirements_viewport')
        self.__auto_apply_button = self.__gtk_builder.get_object('rtpp_action_tab_auto_save_checkbox')
        #__requ_bb_dict contains all requirements button boxes
        self.__requ_cb_dict = self.__add_requirements_boxes(view_port)

    def __del__(self):
        #remove the save button from the list, if the action tab was closed.
        if self.__auto_apply_button in PddlActionTabController.auto_apply_check_buttons:
            self.__controller.remove_auto_apply_button(self.__auto_apply_button)



    def init_tab(self):
        '''
        loads the data into the action tab and subscribes on signals of some gui elements.
        :return: the action tab.
        '''

        #  set elements uneditable if state is library state
        if isinstance(self.__state, LibraryState):
            self.__load_from_semantic_section(True)
            self.__description_text_view.set_editable(False)
            self.__description_text_view.set_cursor_visible(False)
            self.__pddl_action_source_view.set_editable(False)
            self.__pddl_action_source_view.set_cursor_visible(False)
            self.__pddl_predicates_text_view.set_editable(False)
            self.__pddl_predicates_text_view.set_cursor_visible(False)
            self.__pddl_types_text_view.set_editable(False)
            self.__pddl_types_text_view.set_cursor_visible(False)
            self.__auto_apply_button.set_sensitive(False)
            self.__gtk_builder.get_object('rtpp_pddl_tab_auto_fill_button').set_sensitive(False)
            self.__gtk_builder.get_object('rtpp_pddl_tab_apply').set_sensitive(False)

            #disable requirements check boxes
            for c_button in self.__requ_cb_dict.values():
                c_button.set_sensitive(False)
        else:
            self.__load_from_semantic_section(False)
            #observe parts
            auto_fill_button = self.__gtk_builder.get_object('rtpp_pddl_tab_auto_fill_button')
            pred_buf = self.__pddl_predicates_text_view.get_buffer()
            types_buf = self.__pddl_types_text_view.get_buffer()
            req_dict = self.__requ_cb_dict
            auto_fill_button.connect('clicked', self.__call_controller_auto_complete, pred_buf, types_buf, req_dict)
            apply_button = self.__gtk_builder.get_object('rtpp_pddl_tab_apply')
            desc_buf = self.__description_text_view.get_buffer()
            action_buf = self.__pddl_action_source_view.get_buffer()

            apply_button.connect('clicked',self.__controller.on_apply_changes,
                                 desc_buf, action_buf, pred_buf, types_buf, req_dict)
            self.__auto_apply_button.set_active(PddlActionTabController.auto_apply_enabled)
            self.__auto_apply_button.connect('toggled', self.__controller.auto_apply_toogled)
            self.__controller.add_auto_apply_button(self.__auto_apply_button)

            self.__description_text_view.get_buffer().connect('changed',self.__controller.save_data,'description',False)
            self.__pddl_action_source_view.get_buffer().connect('changed', self.__controller.save_data,'pddl_action',False)
            self.__pddl_predicates_text_view.get_buffer().connect('changed', self.__controller.save_data,'pddl_predicates',False)
            self.__pddl_types_text_view.get_buffer().connect('changed', self.__controller.save_data, 'pddl_types',False)

            #connect to requirements check boxes
            for c_button in self.__requ_cb_dict.values():
                c_button.connect('toggled',self.__controller.save_requirements,req_dict, False)

        action_tab = self.__gtk_builder.get_object('rtpp_action_box')
        action_tab.show_all()
        return action_tab


    def __add_requirements_boxes(self, gtk_viewport):
        '''
        adds dynamically boxes for all requirements, specified in requ_list (above) to the gui.
        :param gtk_viewport: the base element, where to store the boxes in.
        :return: a dictionary, contining the information, which button box belongs to which requirement
        '''
        button_dict = {} #key: id value: checkButtonObject
        grid = Gtk.Grid()
        grid.insert_row(0)
        grid.insert_column(0)
        grid.insert_column(0)

        row_counter = 1
        column_counter = 0
        for requirement in requ_list:

            check_button = Gtk.CheckButton.new_with_label(requirement)
            button_dict[requirement] = check_button
            grid.attach(check_button, column_counter % 3, row_counter - 1, 1, 1)
            column_counter += 1

            if column_counter % 3 == 0:
                grid.insert_row(row_counter)
                row_counter += 1


        gtk_viewport.add(grid)
        grid.show_all()
        return button_dict



    def __load_from_semantic_section(self, is_library_state):
        '''
        loads the pddl data from the semantic section of the state and writes it into
        the action tab gui elements.
        :param is_library_state: true, if state is a library state
        :return: Nothing
        '''

        rtpp_dict = self.__state.semantic_data[SEMANTIC_DATA_DICT_NAME]

        if is_library_state:
            rtpp_dict = self.__state.state_copy.semantic_data[SEMANTIC_DATA_DICT_NAME]

        self.__description_text_view.get_buffer().set_text(self.__filter_input(str(rtpp_dict['description'])))
        source_view_string = self.__filter_input(str(rtpp_dict['pddl_action']))

        if len(source_view_string) == 0:
            source_view_string = '\n\n\n\n\n\n\n\n\n'
        self.__pddl_action_source_view.get_buffer().set_text(source_view_string)
        self.__pddl_predicates_text_view.get_buffer().set_text(self.__filter_input(str(rtpp_dict['pddl_predicates'])))
        self.__pddl_types_text_view.get_buffer().set_text(self.__filter_input(str(rtpp_dict['pddl_types'])))
        #load requirements
        for requ in requ_list:
            self.__requ_cb_dict[requ].set_active(requ in rtpp_dict['requirements'])

    def __call_controller_auto_complete(self, button, pred_buf, types_buf, req_dict):
        self.__controller.auto_complete(button, pred_buf, types_buf, req_dict, self.__get_pddl_action())


    def __filter_input(self,input):
        '''
        filters an input string
        :param input:
        :return:
        '''
        if input == '{}':
            input = ''
        return input

    def __get_pddl_action(self):
        '''
        takes the string of the pddl action tab action source view, and returns it as pddl
        action representation.
        :return: a PddlActionRepresentation.
        '''
        start, end = self.__pddl_action_source_view.get_buffer().get_bounds()
        action_text = self.__pddl_action_source_view.get_buffer().get_text(start, end, True).strip()
        action = PddlActionParser(action_text).parse_action()
        return action
