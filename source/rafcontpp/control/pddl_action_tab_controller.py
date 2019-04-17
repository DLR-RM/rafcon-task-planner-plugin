# Contributors:
# Christoph Suerig <christoph.suerig@dlr.de>
# Version 28.01.2019


import re
import gi
gi.require_version('Gtk', '3.0')
import time
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


    def __init__(self, state):
        '''
        :param state: the state, it belongs to.
        '''
        self.__state = state


    def add_auto_save_button(self, button):
        PddlActionTabController.auto_save_check_buttons.append(button)

    def remove_auto_save_button(self, button):
        PddlActionTabController.auto_save_check_buttons.remove(button)


    def save_data(self, buffer, key, saved_manually):
        '''
        reads the values of the tab elements and saves them under the specified key in the semantic section,
        saves only if saved_manually or auto_save_enabled is true.
        :param buffer: a buffer contining the values to store
        :param key: a key of the semantic dict in the requirements section
        :param saved_manually: True if saved manually, false otherwhise
        :return: Nothing
        '''
        #start_time = time.time()#TODO remove, just for debugging
        if saved_manually or PddlActionTabController.auto_save_enabled:
            start, end = buffer.get_bounds()
            to_save = buffer.get_text(start, end,True).strip('\n')
            self.__state.add_semantic_data([SEMANTIC_DATA_DICT_NAME],to_save,key)
        #if key == 'pddl_action' and len(to_save.replace(' ','').replace('\n','')) == 0:

        #logger.debug('saving description needed: {0:.4f} '.format(time.time() - start_time))#TODO remove, just for debugging




    def save_requirements(self, checkbox, req_dict, saved_manually):
        '''
        saves all requirements specified in the gui,
        saves only if saved_manually or auto_save_enabled is true.
        :param checkbox: unused
        :param req_dict: a dictionary, containing the requirements checkboxes and names as keys.
        :param saved_manually: True if saved manually, false otherwhise
        :return: nothing
        '''
        if saved_manually or PddlActionTabController.auto_save_enabled:
            self.__state.add_semantic_data([SEMANTIC_DATA_DICT_NAME], str(self.__get_requirements(req_dict)), 'requirements')


    def __get_requirements(self, req_dict):
        '''
        creates a list with all requirements checked in the gui.
        :param req_dict: a dictionary, containing the requirements checkboxes and names as keys.
        :return: a list with all requirements checked in the gui
        '''
        requirements = []

        for key in req_dict.keys():
            if req_dict[key].get_active():
                requirements.append(key)

        return requirements

    def on_apply_changes(self,button,desc_buf,action_buf,pred_buf,types_buf,req_dict):
        '''
        saves all changes to the rafcon semantic tab.
        :param button: not used.
        :param desc_buf: the buffer of the description text view.
        :param action_buf: the buffer of the pddl action text view.
        :param pred_buf: the buffer of the predicates text view.
        :param types_buf: the buffer of the types text view.
        :param req_dict: a dictionary, containing the requirements checkboxes and their names as keys.
        :return: Nothing.
        '''
        self.save_data(desc_buf, 'description', True)
        self.save_data(action_buf, 'pddl_action', True)
        self.save_data(pred_buf, 'pddl_predicates', True)
        self.save_data(types_buf, 'pddl_types', True)
        self.save_requirements(None, req_dict, True)



    def auto_apply_toogled(self,checkbox):
        '''
        enables and disables the auto apply checkbox in all action tabs of all states.
        :param checkbox: the caller checkbox, containing the up to date state of auto apply.
        :return: Nothing.
        '''
        #count semaphore up
        #this semaphore does not help for multi threading, but for recursive calls.
        PddlActionTabController.auto_save_semaphore+=1
        #only modify if you are the first one
        if PddlActionTabController.auto_save_semaphore == 1:
            PddlActionTabController.auto_save_enabled = checkbox.get_active()
            for button in PddlActionTabController.auto_save_check_buttons:
                button.set_active(checkbox.get_active())
        #reset semaphore if you are the last one
        if PddlActionTabController.auto_save_semaphore >= len(PddlActionTabController.auto_save_check_buttons):
            PddlActionTabController.auto_save_semaphore = 0




    def auto_complete(self, button,pred_buf, types_buf, requ_dict, pddl_action):
        '''
        tries to auto complete the predicates, types and Requirements fields, sets and saves them.
        :param button: unused
        :param pred_buf: the text buffer of the predicates field.
        :param types_buf: the text buffer of the types field.
        :param requ_dict: a dictionary, containing the requirements checkboxes and names as keys.
        :param pddl_action: a PddlActionRepresentation, of the current Action in the Tab.
        :return: Nothing
        '''

        self.__predicates_auto_complete(pddl_action,pred_buf)
        self.__types_auto_complete(pddl_action, types_buf)
        self.__requirements_auto_complete(pddl_action.action, requ_dict)




    def __requirements_auto_complete(self, raw_action, requ_dict):
        '''
        requirements auto complete, will set requirements as specified in pddl 1.2 paper.
        :param raw_action: the pddl action string.
        :param req_dict: a dictionary, containing the requirements checkboxes and names as keys.
        :return: Nothing
        '''
        need = PddlRequirementFinder(raw_action)
        #represents the hierarchy specified in pddl 1.2
        requ_dict[':adl'].set_active(need.adl() or requ_dict[':adl'].get_active())
        if not need.adl():
            requ_dict[':strips'].set_active(need.strips()or requ_dict[':strips'].get_active())
            requ_dict[':typing'].set_active(need.typing()or requ_dict[':typing'].get_active())
            requ_dict[':equality'].set_active(need.equality()or requ_dict[':equality'].get_active())
            requ_dict[':conditional-effects'].set_active(need.conditional_effects()or requ_dict[':conditional-effects'].get_active())
            requ_dict[':disjunctive-preconditions'].set_active(need.disjunctive_preconditions()or requ_dict[':disjunctive-preconditions'].get_active())
            requ_dict[':quantified-preconditions'].set_active(need.quantified_preconditions()or requ_dict[':quantified-preconditions'].get_active())
            if not need.quantified_preconditions():
                requ_dict[':existential-preconditions'].set_active(need.existential_preconditions()or requ_dict[':existential-preconditions'].get_active())
                requ_dict[':universal-preconditions'].set_active(need.universal_preconditions()or requ_dict[':universal-preconditions'].get_active())

        requ_dict[':foreach-expansions'].set_active(need.foreach_expansions()or requ_dict[':foreach-expansions'].get_active())
        requ_dict[':dag-expansions'].set_active(need.dag_expansions()or requ_dict[':dag-expansions'].get_active())
        requ_dict[':action-expansions'].set_active(need.action_expansions()or requ_dict[':action-expansions'].get_active())
        requ_dict[':fluents'].set_active(need.fluents()or requ_dict[':fluents'].get_active())
        requ_dict[':expression-evaluation'].set_active(need.expression_evaluation()or requ_dict[':expression-evaluation'].get_active())

        self.save_requirements(requ_dict[':strips'],requ_dict,True)



    def __types_auto_complete(self, pddl_action, types_buffer):
        '''
        takes the types from a pddl action, compares it with the types, already filled in to the type section
        and adds missing types.
        :param types_buffer: the buffer of the types text view.
        :param pddl_action: a PddlActionRepresentation.
        :return: Noting
        '''

        pattern = re.compile('-{1}\s+[^\s\)]*')
        types = pddl_action.types

        #merge add unknown
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
        self.save_data(types_buffer, 'pddl_types',True)



    def __predicates_auto_complete(self, pddl_action, pred_field_buf):
        '''
        takes the predicates used in the pddl action, compares them with the predicates, the section is already filled
        with, and completes the missing. some predicates with the same name can be found more than on time, if
        these two predicates use values of different types.
        :param pddl_action: a PddlActionRepresentation, to filter predicates from.
        :param pred_field_buf: the Predicates gui text view buffer.
        :return: Nothing
        '''

        found_predicates = pddl_action.predicates

        # merge add unknown
        start, end = pred_field_buf.get_bounds()
        pred_field = pred_field_buf.get_text(start, end, True)
        upper_pred = pred_field.upper()

        for fpred in found_predicates:
            if upper_pred.find(fpred.upper()) == -1:
                pred_field = pred_field +'\r\n' + fpred
                upper_pred = pred_field.upper()


        pred_field = pred_field.strip('\r\n')
        pred_field_buf.set_text(pred_field)
        self.save_data(pred_field_buf,'pddl_predicates',True)

