import os
import unicodedata
import json
from rafcontpp.model.type_tree import TypeTree
from rafcontpp.model.datastore import SEMANTIC_DATA_DICT_NAME
from rafcontpp.logic.predicate_merger import PredicateMerger
from rafcontpp.model.pddl_action_representation import PddlActionRepresentation
from rafcontpp.model.pddl_action_representation import parse_action_name
from rafcontpp.model.pddl_action_representation import action_to_upper
from rafcon.core.singleton import library_manager
from rafcon.utils import log

logger = log.get_logger(__name__)

class DomainGenerator:
    '''DomainGenerator
    The DomainGenerator uses the data, provided by the datastore to generate a domain.pddl file for the planner.
    '''
    __datastore = None

    def __init__(self, datastore):
        if datastore is None:
            logger.error("Datastore in DomainGenerator can not be None!")
            raise ValueError("Datastore in DomainGenerator can not be None!")

        self.__datastore = datastore

    def generate_domain(self):
        """
        generateDomain generates a domain and returns its path.
        :param domain_name: the name of the domain, mentioned in the facts file.
        :param pddl_actions: a list of pddl acitons as PddlActionRepresentation.
        :param type_dict: a type dictionary, with type:parent entries.
        :param domain_dir: the location to save the generated domain file at.
        :return: the path of the generated domain file
        """
        domain_name = self.__parse_domain_name()
        self.__datastore.set_domain_name(domain_name.lower())
        type_dict = self.__dict_to_upper(json.load(open(self.__datastore.get_type_db_path(), "r")))
        pddl_actions = self.__get_pddl_actions_from_file()
        domain_path = os.path.abspath(os.path.join(self.__datastore.get_file_save_dir(), domain_name + ".pddl"))
        type_tree = self.__merge_types(pddl_actions, type_dict)
        merged_preds = self.__merge_predicates(pddl_actions,type_tree)
        merged_requirs = self.__merge_requirements(pddl_actions)
        logger.debug('writing domain file to: '+str(domain_path))
        domain_file = open(domain_path, "w")
        domain_file.write(self.__get_head(domain_name) + "\r\n")
        domain_file.write(self.__get_requirements(merged_requirs) + "\r\n")
        domain_file.write(self.__get_types(type_tree) + "\r\n")
        domain_file.write(self.__get_predicates(merged_preds) + "\r\n")
        domain_file.write(self.__get_actions(pddl_actions) + "\r\n")
        domain_file.write(")")
        domain_file.flush()
        domain_file.close()
        self.__datastore.set_domain_path(domain_path)
        self.__datastore.add_generated_file(domain_name+'.pddl')
        return domain_path




    def __parse_domain_name(self):
        '''parse_domain_name
        parse_domain_name parses the domain name out of the given facts file.
        :return: the domain name
        '''
        facts_file = self.__datastore.get_facts_path()
        input = ""
        c_char = 'a'
        facts = open(facts_file,'r')
        bracket_counter = 0
        while bracket_counter < 2:
            c_char = facts.read(1)
            input = input + c_char
            if c_char == ')':
                bracket_counter+=1


        input = input.upper()
        input = input[(input.index(':DOMAIN')+7):-1]
        domain_name = input.replace(' ','').replace('\r','').replace('\n','').replace('\t','')
        logger.debug('Parsed domain name, name is: '+domain_name)
        return domain_name

    def __get_head(self, domain_name):
        return "(DEFINE (DOMAIN " + domain_name + ")"

    def __merge_requirements(self, pddl_actions):
        """
        mergeRequirements takes the requirements of the used pddl-actions, and removes dublicates.
        :param pddl_actions: a list with PddlActionRepresentations
        :return: all pddl-requirements without dublicates.
        """
        requirements = []

        for action in pddl_actions:
            for requirement in action.requirements:
                if requirement not in requirements:
                    requirements.append(requirement)
        return requirements

    def __get_requirements(self, requirements):
        """
        takes the requirements list and returns it as string in a pddl-format.
        :param requirements: a list with all requirments
        :return: a string with requirments in a pddl-conform format.
        """
        requirements_section = "(:REQUIREMENTS "
        for requirement in requirements:
            requirements_section = requirements_section + requirement + " "
        return requirements_section + ")"

    def __merge_types(self, pddl_actions, type_dict):
        """
        merge types uses the typetree data structure, to create a type tree, with types needed in the actions.
        :param pddl_actions: a list with PddlActionRepresentations
        :param type_dict: a type dictionary
        :return: a type tree containing all (relevant) types.
        """
        tree = None
        if pddl_actions is not None and type_dict is not None:
            for action in pddl_actions:
                for type in action.types:
                    if tree:
                        if not tree.add_type_branch(type, type_dict) and not tree.is_in_tree(type):
                            raise LookupError("no Type \"" + type + "\" found in type dictionary!")
                    else:
                        c_type = type
                        while c_type in type_dict:
                            c_type = type_dict[c_type]
                        tree = TypeTree(c_type)
                        tree.recursive_insert(type, type_dict)
        return tree

    def __get_types(self, merged_types):
        """
        takes a type tree and returns a string representation usable in a pddl-domain
        :param merged_types: a type tree containing all (relevant) types.
        :return: a type string usable in pddl.
        """
        types_in_pddl = ""
        if merged_types:
            types = "(:TYPES \r\n"
            types_in_pddl = types + merged_types.get_as_string() + ")"
        return types_in_pddl

    def __merge_predicates(self, pddl_actions,type_tree):
        """
        mergePredicates takes all predicates mentioned in the PddlActionRepresentations, and removes dublicates.
        :param pddl_actions: a list of PddlActionRepresentations.
        :return: a list of predicates, without dublicates.
        """
        #pre merge predicates
        predicates = []
        for action in pddl_actions:
            for predicate in action.predicates:
                if predicate not in predicates:
                    predicates.append(predicate)

        merger = PredicateMerger(type_tree)
        predicates = merger.merge_predicates(predicates)
        return predicates

    def __get_predicates(self, predicates):
        """
        takes a list of predicates as string, and returns a pddl confrom predicates section.
        :param predicate_section: a list of predicates as strings.
        :return: a pddl conform predicates section as string.
        """
        predicate_section = "(:PREDICATES\r\n"
        for predicate in predicates:
            predicate_section = predicate_section + predicate + "\r\n"
        return predicate_section + ")"

    def __get_actions(self, pddl_actions):
        """
        takes a list of PddlActionRepresentations, and returns a pddl conform action section.
        :param pddl_actions: a list of PddlActionRepresentations.
        :return: a pddl conform action section as string.
        """
        actions = ""
        for action in pddl_actions:
            actions += action.action + "\r\n\r\n"
        return actions




    def __get_pddl_actions_from_file(self):
        """
        getPddlActionsFromFile reads the actions from the pddl-action-database and
        parses them into the internal format (PddlActionPrepresentation).
        :param action_file_path: the path of the aciton-database
        :param action_names: the name of the actions needed.
        :return: a list of needed actions in PlanActionRepresentation format.
        """
        state_libs = self.__datastore.get_state_pools()
        lib_names = []
        for pool in state_libs:
            lib_names.append(os.path.basename(pool))

        pddl_actions = []
        found_actions = []
        for lib_name in lib_names:
            state_pool = library_manager.libraries[lib_name]
            for state in state_pool:
                lib_state = library_manager.get_library_instance(lib_name, state)
                sem_data = lib_state.state_copy.semantic_data

                if SEMANTIC_DATA_DICT_NAME in sem_data:
                    raw_action = sem_data[SEMANTIC_DATA_DICT_NAME]
                    #parse from unicode to string r means raw
                    r_action_name = unicodedata.normalize('NFKD', raw_action['pddl_action']).encode('utf-8','ignore')
                    r_pred_str = unicodedata.normalize('NFKD', raw_action["pddl_predicates"]).encode('utf-8','ignore')
                    r_action = unicodedata.normalize('NFKD', raw_action["pddl_action"]).encode('utf-8','ignore')
                    r_types = unicodedata.normalize('NFKD', raw_action["pddl_types"]).encode('utf-8','ignore')

                    action_name = parse_action_name(r_action_name.upper())
                    found_actions.append(action_name)
                    #construct predicate array from predicate string.
                    predicates = []
                    predicate_string = r_pred_str
                    while predicate_string.find('(') < predicate_string.find(')'):
                        start = predicate_string.find('(')
                        end = predicate_string.find(')')+1
                        predicates.append(predicate_string[start:end])
                        logger.debug("parsed predicate: " + predicate_string[start:end])
                        predicate_string = predicate_string[end:]
                    pddl_actions.append(action_to_upper(PddlActionRepresentation(
                                                                action_name,
                                                                r_action,
                                                                predicates,
                                                                self.__parse_type_string(r_types),
                                                                raw_action["requirements"])))
        #just check, if all needed actions could be parsed.
        for action_name in self.__datastore.get_available_actions():
            if action_name not in found_actions:
                logger.error("No action found for action called: \"" + action_name + "\"")

        return pddl_actions

    #parse typestring, make a list out of one string...
    def __parse_type_string(self, type_string):
        ts = type_string.replace(',',' ')
        ts = ts.split(' ')
        ts = list(filter(None,ts))
        return ts

    def __dict_to_upper(self, dict):
        '''
         receives a dict of string:string and returns it in upper case
        :param dict: a string:string dict
        :return: a new dict in upper case.
        '''
        upper_dict = dict

        if dict:
            upper_dict = {}

            for key, value in dict.iteritems():
                upper_dict[key.upper()] = value.upper()
        return upper_dict


