#Contributors:
#Christoph Suerig <christoph.suerig@dlr.de>
#Version 05.07.2019
import os
import unicodedata
from rafcontpp.model.datastore import SEMANTIC_DATA_DICT_NAME, PDDL_ACTION_SUB_DICT_NAME
from rafcontpp.model.pddl_action_representation import PddlActionRepresentation
from rafcontpp.model.pddl_action_representation import action_to_upper
from rafcontpp.logic.pddl_action_parser import PddlActionParser
from rafcon.core.singleton import library_manager
from rafcon.utils import log

logger = log.get_logger(__name__)



class PddlActionLoader:


    def __init__(self, datastore):
        """

        :param datastore: a datastore containing all necessary data.
        """

        self.__datastore = datastore


    def load_pddl_actions(self):
        """
        load_pddl_actions reads the actions from the states and
        parses them into PddlActionRepresentations.
        Then it sets the pddl action map in datastore.
        """
        state_libs = self.__datastore.get_state_pools()
        lib_names = []
        for pool in state_libs:
            lib_names.append(os.path.basename(pool))

        pddl_actions = {}
        for lib_name in lib_names:
            state_pool = library_manager.libraries[lib_name]
            for state in state_pool:
                lib_state = library_manager.get_library_instance(lib_name, state)
                sem_data = lib_state.state_copy.semantic_data

                if SEMANTIC_DATA_DICT_NAME in sem_data \
                        and str(state) in self.__datastore.get_state_action_map().keys():
                    action_dict = sem_data[SEMANTIC_DATA_DICT_NAME][PDDL_ACTION_SUB_DICT_NAME]
                    #parse from unicode to string r means raw
                    r_pred_str = unicodedata.normalize('NFKD', action_dict["pddl_predicates"]).encode('utf-8', 'ignore')
                    r_action = unicodedata.normalize('NFKD', action_dict["pddl_action"]).encode('utf-8', 'ignore')
                    r_types = ''
                    r_reqs = ''
                    if isinstance(action_dict["pddl_types"], unicode):
                        r_types = unicodedata.normalize('NFKD', action_dict["pddl_types"]).encode('utf-8', 'ignore')
                    if isinstance(action_dict["requirements"], unicode):
                        r_reqs = unicodedata.normalize('NFKD', action_dict["requirements"]).encode('utf-8', 'ignore')

                    action_parser = PddlActionParser(r_action)
                    action_name = action_parser.parse_action_name().upper()


                    c_action = action_to_upper(PddlActionRepresentation(
                        action_name,
                        r_action,
                        self.parse_predicate_string(r_pred_str),
                        self.parse_type_string(r_types),
                        self.parse_requirement_string(r_reqs),
                        action_parser.parse_parameters()))

                    if c_action.name in pddl_actions.keys():
                        logger.warning('Multiple definition of Action: ' + c_action.name +
                                       '. Action definition of State ' + str(state) + ' not used.')
                    else:
                        pddl_actions[c_action.name] = c_action

        #just check, if all needed actions could be parsed.
        for action_name in self.__datastore.get_available_actions():
            if action_name not in pddl_actions.keys():
                logger.error("No action found for action called: \"" + action_name + "\"")

        self.__datastore.set_pddl_action_map(pddl_actions)

        #parse typestring, make a list out of one string...

    def parse_type_string(self, type_string):
        """parse_type_string
        parse_type_string gets a type string of fromat type1,type2 or type1 type2 and parses it into an
        array containing the types.
        :param type_string: a typestring of format type1,type2 or type1 type2.
        :return: an array containing the types of the string, or an empty array if string is None or empty.
        """
        ts = []
        if type_string:
            ts = type_string.replace(',', ' ')
            ts = ts.split(' ')
            ts = list(filter(None, ts))
        return ts

    def parse_requirement_string(self, requ_string):
        """parse_requirement_string
        gets an requirements string and parses it into a requirement array.
        :param requ_string: a string contining requriements.
        :return: an array contining requirements.
        """
        req_list = []
        if requ_string:
            rs = requ_string.strip('[').strip(']')
            rs = rs.replace('\'', '')
            rs = rs.replace(' ', '')
            req_list = rs.split(',')
        return req_list

    def parse_predicate_string(self,pred_string):
        """
        construct predicate array from predicate string.
        :param pred_string: A string containing all predicates of the action.
        :return: a array containing all predicates of the action.
        """
        predicates = []
        predicate_string = pred_string if pred_string is not None else ''
        start_index = predicate_string.find('(')
        end_index = predicate_string.find(')') + 1
        while start_index > -1 and start_index < end_index:
            predicates.append(predicate_string[start_index:end_index])
            predicate_string = predicate_string[end_index:]
            start_index = predicate_string.find('(')
            end_index = predicate_string.find(')') + 1
        logger.debug("predicates:  "+str(predicates))
        return predicates

