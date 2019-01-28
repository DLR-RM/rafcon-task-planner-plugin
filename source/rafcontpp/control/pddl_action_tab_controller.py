# Contributors:
# Christoph Suerig <christoph.suerig@dlr.de>
# Version 28.01.2019


import re
import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk
from rafcontpp.model.datastore import SEMANTIC_DATA_DICT_NAME
from rafcontpp.logic.pddl_action_parser import PddlActionParser
from rafcontpp.logic.pddl_requirement_finder import PddlRequirementFinder
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
                ':dag-expansions', ':expression-evaluation', ':fluents', ':open-world',':true-negation']#todo: remove undecidable


class PddlActionTabController:
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
        self.__auto_save_button = self.__gtk_builder.get_object('rtpp_action_tab_auto_save_checkbox')
        #__requ_bb_dict contains all requirements button boxes
        self.__requ_cb_dict = self.__add_requirements_boxes(view_port)

    def __del__(self):
        #remove the save button from the list, if the action tab was closed.
        if self.__auto_save_button in PddlActionTabController.auto_save_check_buttons:
            PddlActionTabController.auto_save_check_buttons.remove(self.__auto_save_button)


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
            self.__auto_save_button.set_sensitive(False)
            self.__gtk_builder.get_object('rtpp_pddl_tab_auto_fill_button').set_sensitive(False)
            self.__gtk_builder.get_object('rtpp_pddl_tab_apply').set_sensitive(False)

            #disable requirements check boxes
            for c_button in self.__requ_cb_dict.values():
                c_button.set_sensitive(False)
        else:
            self.__load_from_semantic_section(False)
            #observe parts
            auto_fill_button = self.__gtk_builder.get_object('rtpp_pddl_tab_auto_fill_button')
            auto_fill_button.connect('clicked', self.__auto_complete)
            apply_button = self.__gtk_builder.get_object('rtpp_pddl_tab_apply')
            apply_button.connect('clicked',self.__on_apply_changes)
            self.__auto_save_button.set_active(PddlActionTabController.auto_save_enabled)
            self.__auto_save_button.connect('toggled', self.__auto_save_toogled)
            PddlActionTabController.auto_save_check_buttons.append(self.__auto_save_button)

            self.__description_text_view.get_buffer().connect('changed',self.__save_data,'description',False)
            self.__pddl_action_source_view.get_buffer().connect('changed', self.__save_data,'pddl_action',False)
            self.__pddl_predicates_text_view.get_buffer().connect('changed', self.__save_data,'pddl_predicates',False)
            self.__pddl_types_text_view.get_buffer().connect('changed', self.__save_data, 'pddl_types',False)

            #connect to requirements check boxes
            for c_button in self.__requ_cb_dict.values():
                c_button.connect('toggled',self.__save_requirements,False)



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



    def __save_data(self, buffer, key,saved_manually):
        '''
        reads the values of the tab elements and saves them under the specified key in the semantic section,
        saves only if saved_manually or auto_save_enabled is true.
        :param buffer: a buffer contining the values to store
        :param key: a key of the semantic dict in the requirements section
        :param saved_manually: True if saved manually, false otherwhise
        :return: Nothing
        '''
        if saved_manually or PddlActionTabController.auto_save_enabled:
            start, end = buffer.get_bounds()
            self.__state.add_semantic_data([SEMANTIC_DATA_DICT_NAME],buffer.get_text(start, end,True).strip('\n'),key)




    def __save_requirements(self,checkbox,saved_manually):
        '''
        saves all requirements specified in the gui,
        saves only if saved_manually or auto_save_enabled is true.
        :param checkbox: unused
        :param saved_manually: True if saved manually, false otherwhise
        :return: nothing
        '''
        if saved_manually or PddlActionTabController.auto_save_enabled:
            self.__state.add_semantic_data([SEMANTIC_DATA_DICT_NAME], str(self.__get_requirements()), 'requirements')


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

    def __on_apply_changes(self,button):
        self.__save_data(self.__description_text_view.get_buffer(),'description',True)
        self.__save_data(self.__pddl_action_source_view.get_buffer(),'pddl_action',True)
        self.__save_data(self.__pddl_predicates_text_view.get_buffer(),'pddl_predicates',True)
        self.__save_data(self.__pddl_types_text_view.get_buffer(),'pddl_types',True)
        self.__save_requirements(None,True)


    #now called auto apply
    def __auto_save_toogled(self,checkbox):
        #count semaphore up
        PddlActionTabController.auto_save_semaphore+=1
        #only modify if you are the first one
        if PddlActionTabController.auto_save_semaphore == 1:
            PddlActionTabController.auto_save_enabled = checkbox.get_active()
            for button in PddlActionTabController.auto_save_check_buttons:
                button.set_active(checkbox.get_active())
        #reset semaphore if you are the last one
        if PddlActionTabController.auto_save_semaphore >= len(PddlActionTabController.auto_save_check_buttons):
            PddlActionTabController.auto_save_semaphore = 0




    def __auto_complete(self, button):
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

        self.__predicates_auto_complete(pddl_action)
        self.__types_auto_complete(pddl_action)
        self.__requirements_auto_complete(raw_action)




    def __requirements_auto_complete(self, raw_action):
        '''
        requirements auto complete, will set requirements as specified in pddl 1.2 paper.
        :param raw_action: the pddl action string.
        :return: Nothing
        '''
        need = PddlRequirementFinder(raw_action)
        #represents the hierarchy specified in pddl 1.2
        self.__requ_cb_dict[':adl'].set_active(need.adl() or self.__requ_cb_dict[':adl'].get_active())
        if not need.adl():
            self.__requ_cb_dict[':strips'].set_active(need.strips()or self.__requ_cb_dict[':strips'].get_active())
            self.__requ_cb_dict[':typing'].set_active(need.typing()or self.__requ_cb_dict[':typing'].get_active())
            self.__requ_cb_dict[':equality'].set_active(need.equality()or self.__requ_cb_dict[':equality'].get_active())
            self.__requ_cb_dict[':conditional-effects'].set_active(need.conditional_effects()or self.__requ_cb_dict[':conditional-effects'].get_active())
            self.__requ_cb_dict[':disjunctive-preconditions'].set_active(need.disjunctive_preconditions()or self.__requ_cb_dict[':disjunctive-preconditions'].get_active())
            self.__requ_cb_dict[':quantified-preconditions'].set_active(need.quantified_preconditions()or self.__requ_cb_dict[':quantified-preconditions'].get_active())
            if not need.quantified_preconditions():
                self.__requ_cb_dict[':existential-preconditions'].set_active(need.existential_preconditions()or self.__requ_cb_dict[':existential-preconditions'].get_active())
                self.__requ_cb_dict[':universal-preconditions'].set_active(need.universal_preconditions()or self.__requ_cb_dict[':universal-preconditions'].get_active())

        self.__requ_cb_dict[':foreach-expansions'].set_active(need.foreach_expansions()or self.__requ_cb_dict[':foreach-expansions'].get_active())
        self.__requ_cb_dict[':dag-expansions'].set_active(need.dag_expansions()or self.__requ_cb_dict[':dag-expansions'].get_active())
        self.__requ_cb_dict[':action-expansions'].set_active(need.action_expansions()or self.__requ_cb_dict[':action-expansions'].get_active())
        self.__requ_cb_dict[':fluents'].set_active(need.fluents()or self.__requ_cb_dict[':fluents'].get_active())
        self.__requ_cb_dict[':expression-evaluation'].set_active(need.expression_evaluation()or self.__requ_cb_dict[':expression-evaluation'].get_active())

        self.__save_requirements(self.__requ_cb_dict[':strips'],True)



    def __types_auto_complete(self, pddl_action):
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
        self.__save_data(types_buffer, 'pddl_types',True)



    def __predicates_auto_complete(self, pddl_action):
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
        self.__save_data(self.__pddl_predicates_text_view.get_buffer(),'pddl_predicates',True)




    def __filter_input(self,input):
        '''
        filters an input string
        :param input:
        :return:
        '''
        if input == '{}':
            input = ''
        return input

