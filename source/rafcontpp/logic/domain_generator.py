# Copyright (C) 2018-2019 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the 3-Clause BSD License which accompanies this
# distribution, and is available at
# https://opensource.org/licenses/BSD-3-Clause
#
# Contributors:
# Christoph Suerig <christoph.suerig@dlr.de>

# Don't connect with the Copyright comment above!
# Version 09.09.2019

import os
from datetime import datetime

from rafcon.utils import log

from rafcontpp.logic.predicate_merger import PredicateMerger
from rafcontpp.logic.type_merger import TypeMerger
from rafcontpp.model.datastore import PLUGIN_VERSION
logger = log.get_logger(__name__)


class DomainGenerator:
    """
    The DomainGenerator uses the data provided by the datastore to generate a domain.pddl file for the planner.
    """

    def __init__(self, datastore):
        """

        :param datastore: A Datastore containing: pddl facts representation, pddl action map, file save directory.
        """
        if datastore is None:
            logger.error("Datastore in DomainGenerator can not be None!")
            raise ValueError("Datastore in DomainGenerator can not be None!")
        self.__datastore = datastore

    def generate_domain(self):
        """
        generateDomain generates a domain and returns its path.

        :return: String: The path of the generated domain file.
        """
        facts = self.__datastore.get_pddl_facts_representation()
        domain_name = facts.domain_name
        problem_name = facts.problem_name
        pddl_actions = self.__datastore.get_pddl_action_map().values()
        domain_path = os.path.abspath(os.path.join(self.__datastore.get_file_save_dir(), domain_name + "_domain.pddl"))
        type_merger = TypeMerger(self.__datastore)
        type_tree = type_merger.merge_types()
        self.__datastore.set_available_types(type_tree)
        preds = self.__merge_predicates(pddl_actions)
        self.__datastore.set_available_predicates(preds[1])
        merged_preds = preds[0]
        merged_requirs = self.__merge_requirements(pddl_actions)
        logger.debug('writing domain file to: ' + str(domain_path))
        domain_file = open(domain_path, "w")
        domain_file.write("{}\r\n".format(self.__get_comment_section()))
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

    def __get_head(self, domain_name):
        """
        Takes a domain name, and returns the head of a pddl domain, e.g. "(define (domain test_domain)".

        :param domain_name: The name of the domain.
        :return: String: The head of a pddl domain.
        """
        return "(DEFINE (DOMAIN {})".format(domain_name)

    def __merge_requirements(self, pddl_actions):
        """
        mergeRequirements takes the requirements of the used pddl-actions, and removes dublicates.

        :param pddl_actions: A list with PddlActionRepresentations
        :return: [String]: All pddl-requirements without dublicates.
        """
        requirements = []
        for action in pddl_actions:
            for requirement in action.requirements:
                if requirement not in requirements:
                    requirements.append(requirement)
        return requirements

    def __get_requirements(self, requirements):
        """
        Takes the requirements list and returns it as string in a pddl-format.

        :param requirements: A list with all requirments
        :return: String: Requirments in a pddl-conform format.
        """
        requirements_section = "(:REQUIREMENTS "
        for requirement in requirements:
            requirements_section = requirements_section + requirement + " "
        return requirements_section + ")"

    def __get_types(self, merged_types):
        """
        Takes a type tree and returns a string representation usable in a pddl-domain.

        :param merged_types: A type tree containing all (relevant) types.
        :return: String: A type section usable in pddl.
        """
        types_in_pddl = ""
        if merged_types:
            types = "(:TYPES \r\n"
            types_in_pddl = types + merged_types.get_as_pddl_string() + ")"
        return types_in_pddl

    def __merge_predicates(self, pddl_actions):
        """
        mergePredicates takes all predicates mentioned in the PddlActionRepresentations, and removes dublicates.

        :param pddl_actions: A list of PddlActionRepresentations.
        :return: ([String],[(String,[(String,int)])]): A tuple with a list of predicates and a list of predicates as tuple without dublicates.
        """
        # pre merge predicates
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
        Takes a list of predicates as string, and returns a pddl confrom predicates section.

        :param predicate_section: A list of predicates as strings.
        :return: String: A pddl conform predicates section.
        """
        predicate_section = "(:PREDICATES\r\n"
        for predicate in predicates:
            predicate_section = predicate_section + predicate + "\r\n"
        return predicate_section + ")"

    def __get_actions(self, pddl_actions):
        """
        Takes a list of PddlActionRepresentations, and returns a pddl conform action section.

        :param pddl_actions: A list of PddlActionRepresentations.
        :return: String: A pddl conform action section.
        """
        as_map = self.__datastore.get_action_state_map()
        actions = "\r\n\r\n"
        current_comment = ""
        for action in pddl_actions:
            current_comment = ";; This action is defined in state: \"{}\"".format(as_map[action.name])
            actions += "{}\r\n{}\r\n\r\n".format(current_comment,action.action)
        return actions

    def __get_comment_section(self):
        """
        This method generated a header comment for the domain, containing the version, the time, date and
        the 'author'.

        :return: String: A domain header comment, already containing the pddl comment indicator character ';'.
        """
        comment_section = ";; This Domain was automatically generated by RAFCON Task Planner Plugin (RTPP)\r\n"
        comment_section = "{};; Version: RTPP {}\r\n".format(comment_section, PLUGIN_VERSION)
        comment_section = "{};; Date: {}\r\n".format(comment_section, datetime.now().strftime("%d.%m.%Y"))
        comment_section = "{};; Time: {}h\r\n".format(comment_section, datetime.now().strftime("%H:%M:%S"))
        # build borders.
        comment_section = ";;;;\r\n{};;;;\r\n".format(comment_section)
        return comment_section