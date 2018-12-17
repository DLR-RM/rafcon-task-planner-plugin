# Contributors:
# Christoph Suerig <christoph.suerig@dlr.de>
# Version 01.12.2018


import re
import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk
from gi.repository import GObject
from rafcontpp.model.datastore import SEMANTIC_DATA_DICT_NAME
from rafcontpp.logic.pddl_action_parser import PddlActionParser
from rafcon.core.states.library_state import LibraryState
from rafcon.utils import log

logger = log.get_logger(__name__)
#list with all pddl requirements
requ_list = [':strips', ':adl', ':typing', ':equality',
                ':negative-preconditions', ':disjunctive-preconditions',
                ':conditional-effects', ':existential-preconditions',
                ':universal-preconditions', ':derived-predicates',
                ':action-costs', ':quantified-preconditions',
                ':action-expansions', ':foreach-expansions',
                ':dag-expansions', ':expression-evaluation', ':fluents']


class PddlActionTabController:
    '''PddlActionTabController
    PddlActionTabController, controlles the gui elements of the pddl action tab for each state.
    it handles the dataflow between this tab, and the semantic section of the state, where the data
    of this tab is stored in. It also provides some auto fill wizzard for its elements.

    '''


    def __init__(self,action_tab_gtk_builder, state):
        '''

        :param action_tab_gtk_builder: gtk builder, contining its gui elements.
        :param state: the state, it belongs to.
        '''
        self.__state = state
        self.__gtk_builder = action_tab_gtk_builder
        self.__description_text_view = self.__gtk_builder.get_object('description_textview')
        self.__pddl_action_source_view = self.__gtk_builder.get_object('pddl_action_sourceview')
        self.__pddl_predicates_text_view = self.__gtk_builder.get_object('pddl_predicates_textview')
        self.__pddl_types_text_view = self.__gtk_builder.get_object('pddl_types_textview')
        view_port = self.__gtk_builder.get_object('requirements_viewport')
        #__requ_bb_dict contains all requirements button boxes
        self.__requ_cb_dict = self.__add_requirements_boxes(view_port)


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




    def start_control_tab(self):
        '''
        loads the data into the action tab and subscribes on signals of some gui elements.
        :return:
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
            self.__gtk_builder.get_object('rtpp_pddl_tab_auto_fill_button').set_sensitive(False)
            #disable requirements check boxes
            for c_button in self.__requ_cb_dict.values():
                c_button.set_sensitive(False)
        else:
            self.__load_from_semantic_section(False)
            #observe parts
            auto_fill_button = self.__gtk_builder.get_object('rtpp_pddl_tab_auto_fill_button')
            auto_fill_button.connect('clicked', self.__auto_fill)
            self.__description_text_view.get_buffer().connect('changed',self.__save_data,'description')
            self.__pddl_action_source_view.get_buffer().connect('changed', self.__save_data,'pddl_action')
            self.__pddl_predicates_text_view.get_buffer().connect('changed', self.__save_data,'pddl_predicates')
            self.__pddl_types_text_view.get_buffer().connect('changed', self.__save_data, 'pddl_types')

            #connect to requirements check boxes
            for c_button in self.__requ_cb_dict.values():
                c_button.connect('toggled',self.__save_requirements)



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



    def __save_data(self, buffer, key):
        '''
        reads the values of the tab elements and saves them under the specified key in the semantic section
        :param buffer: a buffer contining the values to store
        :param key: a key of the semantic dict in the requirements section
        :return: Nothing
        '''
        start, end = buffer.get_bounds()
        self.__state.semantic_data[SEMANTIC_DATA_DICT_NAME][key] = buffer.get_text(start, end,True).strip('\n')
        self.__state.get_state_machine().marked_dirty = True


    def __save_requirements(self,checkbox):
        '''
        saves all requirements specified in the gui
        :param checkbox: unused
        :return: nothing
        '''
        self.__state.semantic_data[SEMANTIC_DATA_DICT_NAME]['requirements'] = str(self.__get_requirements())
        self.__state.get_state_machine().marked_dirty = True

    def __get_requirements(self):
        '''
        creates a list with all requirements checked in the gui.
        :return: a list with all requirements checked in the gui
        '''
        requirements = []

        for key in self.__requ_cb_dict.keys():
            if self.__requ_cb_dict[key].get_active():
                requirements.append(key)

        return requirements


    def __auto_fill(self,button):
        '''
        tries to auto complete the predicates, types and Requirements fields.
        :param button: unused
        :return: Nothing
        '''
        #get pddl action from source view
        buffer = self.__pddl_action_source_view.get_buffer()
        start, end = buffer.get_bounds()
        raw_action = buffer.get_text(start, end, True)
        pddl_action = PddlActionParser(raw_action).parse_action()

        self.__predicates_auto_fill(pddl_action)
        self.__types_auto_fill(pddl_action)
        self.__requirements_auto_fill(raw_action)




    def __requirements_auto_fill(self, raw_action):
        '''
        a poor try to auto complete the requirements section in pddl Action tab.
        is not complete, because the question it tries to answer is not decidable at this time.
        :param raw_action: the pddl action string.
        :return: Nothing
        '''
        raw_action = raw_action.upper()
        bev_effects = raw_action[:raw_action.find(':EFFECT')]

        if raw_action.find(' - ') > -1:
            self.__requ_cb_dict[':typing'].set_active(True)

        if raw_action.find('=') > -1:
            self.__requ_cb_dict[':equality'].set_active(True)

        if re.search('\(\s*(WHEN)[\s|\(]', raw_action):
            self.__requ_cb_dict[':conditional-effects'].set_active(True)
            self.__requ_cb_dict[':adl'].set_active(True)

        if re.search('\(\s*(FORALL)[\s|\(]', raw_action):
            self.__requ_cb_dict[':adl'].set_active(True)

        if re.search('\(\s*(AND)[\s|\(]', bev_effects):
            self.__requ_cb_dict[':strips'].set_active(True)

        if re.search('\(\s*(NOT)[\s|\(]', bev_effects):
            self.__requ_cb_dict[':negative-preconditions'].set_active(True)

        if re.search('\(\s*(OR)[\s|\(]', bev_effects):
            self.__requ_cb_dict[':adl'].set_active(True)
            self.__requ_cb_dict[':disjunctive-preconditions'].set_active(True)

        #TODO more autofill
        self.__save_requirements(self.__requ_cb_dict[':strips'])



    def __types_auto_fill(self,pddl_action):
        '''
        takes the types from a pddl action, compares it with the types, already filled in to the type section
        and adds missing types.
        :param pddl_action: a PddlActionRepresentation
        :return: Noting
        '''

        pattern = re.compile('-{1}\s+[^\s\)]*')
        types = pddl_action.types

        #merge add unknown
        types_buffer = self.__pddl_types_text_view.get_buffer()
        start, end = types_buffer.get_bounds()
        type_field = types_buffer.get_text(start, end, True)
        #to be able to compare
        upper_type_field = ' '+ type_field.upper()+' '
        upper_type_field = upper_type_field.replace(',',' ')
        for type in types:
            if upper_type_field.find(' '+type.upper()+' ') == -1:
                type_field = type_field + ", "+type

        #set type field.
        types_buffer.set_text(type_field.strip(',').strip())
        self.__save_data(types_buffer, 'pddl_types')



    def __predicates_auto_fill(self,pddl_action):
        '''
        takes the predicates used in the pddl action, compares them with the predicates, the section is already filled
        with, and completes the missing. some predicates with the same name can be found more than on time, if
        these two predicates use values of different types.
        :param pddl_action:
        :return:
        '''

        found_predicates = pddl_action.predicates

        # merge add unknown
        start, end = self.__pddl_predicates_text_view.get_buffer().get_bounds()
        pred_field = self.__pddl_predicates_text_view.get_buffer().get_text(start, end, True)
        upper_pred = pred_field.upper()

        for fpred in found_predicates:
            if upper_pred.find(fpred.upper()) == -1:
                pred_field = pred_field +'\r\n' + fpred
                upper_pred = pred_field.upper()


        pred_field = pred_field.strip('\r\n')
        self.__pddl_predicates_text_view.get_buffer().set_text(pred_field)
        self.__save_data(self.__pddl_predicates_text_view.get_buffer(),'pddl_predicates')




    def __filter_input(self,input):
        '''
        filters an input string
        :param input:
        :return:
        '''
        if input == '{}':
            input = ''
        return input

