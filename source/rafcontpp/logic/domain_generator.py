# Contributors:
# Christoph Suerig <christoph.suerig@dlr.de>
# Version 01.11.2018

import os
import json
import re
from rafcontpp.model.type_tree import TypeTree
from rafcontpp.logic.predicate_merger import PredicateMerger
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
        self.__datastore.set_problem_name(self.__parse_problem_name())#i do it here because i have no better module, in future it should be done somwhere else
        self.__datastore.set_domain_name(domain_name)
        type_dict = self.__dict_to_upper(json.load(open(self.__datastore.get_type_db_path(), "r")))
        pddl_actions = self.__datastore.get_pddl_action_map().values()
        domain_path = os.path.abspath(os.path.join(self.__datastore.get_file_save_dir(), domain_name + "_domain.pddl"))
        type_tree = self.__merge_types(pddl_actions, type_dict)
        self.__datastore.set_available_types(type_tree)
        preds = self.__merge_predicates(pddl_actions)
        self.__datastore.set_available_predicates(preds[1])
        merged_preds = preds[0]
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
        self.__datastore.add_generated_file(domain_name + "_domain.pddl")
        return domain_path




    def __parse_domain_name(self):#TODO: dont do this in domain_generator!
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

        domain_name = re.findall('\(:DOMAIN\s+([^\s|^\)]+)',input,re.IGNORECASE)[0]
        logger.debug('Parsed domain name is: '+domain_name)
        return domain_name

    def __parse_problem_name(self):#TODO: dont do this in domain_generator! (its also not used here, but in sm generator)
        '''parse_problem_name
        parse_problem_name parses the problem name out of the given facts file.
        :return: the problem name
        '''
        facts_file = self.__datastore.get_facts_path()
        input = ""
        c_char = 'a'
        facts = open(facts_file, 'r')
        while c_char != ')':
            c_char = facts.read(1)
            input = input + c_char

        problem_name = re.findall('\(PROBLEM\s+([^\s|^\)]+)', input, re.IGNORECASE)[0]
        logger.debug('Parsed problem name is: ' + problem_name)
        return problem_name


    def __get_head(self, domain_name):
        ''' get_head
        takes a domain name, and returns the head of a pddl domain
        :param domain_name: the name of the domain
        :return: the head of a pddl domain
        '''
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
                            logger.error("no Type \"" + type + "\" found in type dictionary!")
                            raise LookupError("no Type \"" + type + "\" found in type dictionary!")
                    else:
                        c_type = type
                        while c_type in type_dict.keys():
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

    def __merge_predicates(self, pddl_actions):
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

        merger = PredicateMerger(self.__datastore)
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


