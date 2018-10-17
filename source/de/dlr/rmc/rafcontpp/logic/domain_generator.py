import os
import datastore
from rafcon.utils import log
from typetree import TypeTree
logger = log.get_logger(__name__)

class DomainGenerator:

    __datastore = None

    def __init__(self, datastore):
        if datastore is None:
            logger.error("Datastore in DomainGenerator can not be None!")
            raise ValueError("Datastore in DomainGenerator can not be None!")

        self.__datastore = datastore

    def __parse_domain_name(self):
        facts_file = self.__datastore.get_facts_path()
        input = ""
        c_char = 'a'
        facts = open(facts_file,'r')
        while c_char != ')':
            c_char = facts.read(1)
            input = input + c_char

        input = input.upper()
        input = input[(input.index('PROBLEM')+7):-1]
        domain_name = input.replace(' ','').replace('\r','').replace('\n','').replace('\t','')
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

    def __merge_predicates(self, pddl_actions):
        """
        mergePredicates takes all predicates mentioned in the PddlActionRepresentations, and removes dublicates.
        :param pddl_actions: a list of PddlActionRepresentations.
        :return: a list of predicates, without dublicates.
        """

        predicates = []
        for action in pddl_actions:
            for predicate in action.predicates:
                if predicate not in predicates:
                    predicates.append(predicate)
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
            c_action = ""
            for line in action.action:
                c_action = c_action + line + "\r\n"
            actions = actions + c_action + "\r\n"
        return actions

    def __dict_to_upper(self, dict):
        upper_dict = dict

        if dict:
            upper_dict = {}

            for key, value in dict.iteritems():
                upper_dict[key.upper()] = value.upper()
        return upper_dict

    def __get_pddl_actions_from_file(self):
        """
        getPddlActionsFromFile reads the actions from the pddl-action-database and
        parses them into the internal format (PddlActionPrepresentation).
        :param action_file_path: the path of the aciton-database
        :param action_names: the name of the actions needed.
        :return: a list of needed actions in PlanActionRepresentation format.
        """
        # TODO change for multi action pools!
        ac_dict = json.load(open(self.__datastore.get_action_db_path(), "r"))
        pddl_actions = []
        for action_name in action_names:
            if action_name in ac_dict:
                raw_action = ac_dict[action_name]
                pddl_actions.append(self.__action_to_upper(PddlActionRepresentation(raw_action["name"],
                                                             raw_action["action"],
                                                             raw_action["predicates"],
                                                             raw_action["types"],
                                                             raw_action["requirements"])))
            else:
                logger.error("No action found in database for action called: \"" + action_name + "\"")
        return pddl_actions


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
        type_dict = self.__dict_to_upper(json.load(open(self.__datastore.get_type_db_path, "r")))
        domain_path = os.path.abspath(os.path.join(self.__datastore.get_domain_path(), domain_name + ".pddl"))
        merged_preds = self.__merge_predicates(pddl_actions)#TODO
        merged_requirs = self.__merge_requirements(pddl_actions)
        merged_types = self.__merge_types(pddl_actions, type_dict)
        domain_file = open(domain_path, "w")
        domain_file.write(self.__get_head(domain_name) + "\r\n")
        domain_file.write(self.__get_requirements(merged_requirs) + "\r\n")
        domain_file.write(self.__get_types(merged_types) + "\r\n")
        domain_file.write(self.__get_predicates(merged_preds) + "\r\n")
        domain_file.write(self.__get_actions(pddl_actions) + "\r\n")
        domain_file.write(")")
        domain_file.flush()
        domain_file.close()
        return domain_path


