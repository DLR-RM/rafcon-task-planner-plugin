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
# Version: 17.05.2019
import re

from rafcon.utils import log

from rafcontpp.model.pddl_action_representation import PddlActionRepresentation

logger = log.get_logger(__name__)


class PddlActionParser:
    """ PddlActionParser
    PddlActionParser is able to parse an pddl action string e.g. a usual pddl action into a PddlActionRepresentation
    by extracting predicates, types, variables and the action name.
    """

    def __init__(self, action_string):
        """
        :param action_string: A pddl action string
        """
        if action_string is None or len(action_string) == 0:
            logger.error('Can not parse action from None or Empty String!')
            raise ValueError('Can not parse action from None or Empty String!')
        # matches variables with types e.g ?obj - Physobj
        self.__type_var_pattern = re.compile('((\?[^\s|^\)|^\(]+\s+)+-\s+[^\s|^\)|^\(]+)')
        # matches only type strings result  f.e. Location
        self.__type_pattern = re.compile('-\s+([^\s|^\)|^?|^-]+)')
        # matches only varialbes f.e. ?myVar
        self.__var_pattern = re.compile('\?[^\s|^\)]+')
        # matches applied predicates
        self.__predicate_pattern = re.compile('(\(\s*[^\?|^\s|^\(]+(\s+\?[^\)|^\(|^\s]*)+\s*\))')
        # matches the action name, ignores cases
        self.__action_name_pattern = re.compile('\(:action\s*([^\s|^:]+)', re.IGNORECASE)
        # a dictionary, which contains all variables and their types.
        self.__var_type_dict = {}
        # the pddl action as string, comments removed.
        self.__action_string = self.__clean_comments(action_string)

    def parse_action(self):
        """
        parse_action takes the given action string, and parses it into a PddlActionRepresenation,
        raises ValueError, if the action string is none or empty.

        :return: PddlActionRepresentation: A PddlActionRepresentation of the pddl action.
        """
        self.__create_var_type_dict()
        name = self.parse_action_name()
        predicates = self.__parse_and_generalize_predicates()
        types = list(set(self.__var_type_dict.values()))
        parameters = self.parse_parameters()
        return PddlActionRepresentation(name, self.__action_string, predicates, types, [], parameters)

    def parse_action_name(self):
        """
        parse_action_name reads the action form the given action and returns it.

        :return: String: The name of the action.
        """
        parsed = re.findall(self.__action_name_pattern, self.__action_string)
        if len(parsed) == 0:
            logger.error("Couldn't parse action name from \"{}\"".format(self.__action_string))
            raise ValueError("Couldn't parse action name!")
        return parsed[0]

    def parse_parameters(self):
        """
        parse_parameters parses the parameters out of an pddl action string.

        :return: [String]: A list with parameter names, without '?'
        """
        params = []
        action_string_upper = self.__action_string.upper()
        if ':PARAMETERS' in action_string_upper:
            start_index = action_string_upper.index(':PARAMETERS')
            if ')' in self.__action_string[start_index:]:
                end_index = start_index + self.__action_string[start_index:].index(')')
                type_vars = self.__type_var_pattern.findall(self.__action_string[start_index:end_index])
                for type_var in type_vars:
                    for var in self.__var_pattern.findall(type_var[0]):
                        params.append(var.strip('?'))
        else:
            logger.error('No Parameters found in: ' + self.__action_string)
            raise ValueError('No Parameters found in: ' + self.__action_string)
        return params

    def __clean_comments(self, action_string):
        """
        Takes the action string and removes all comments.

        :return: String: The action string without comments.
        """
        comment_pattern = re.compile('(;[^\n]*)')
        return comment_pattern.sub('', action_string)

    def __create_var_type_dict(self):
        """
        create_var_type_dict creates a dictionary, which contains all variables with their type,
        defined in the action.

        :return: {String:String}: A dictionary containing variable : type pairs.
        """
        type_vars = self.__type_var_pattern.findall(self.__action_string)
        for type_var in type_vars:
            types = self.__type_pattern.findall(type_var[0])
            type = types[0]
            for var in self.__var_pattern.findall(type_var[0]):
                self.__var_type_dict[var] = type
        return self.__var_type_dict

    def __parse_and_generalize_predicates(self):
        """
        This method extracts all applied pradicates from the action, then it generalizes them.
        e.g. add types to the variables and remove dublicats.
        applied predicate example:       (at ?a ?b)
        generalized predicate example: (at ?a - Location ?b - Robot)

        :return: [String]: A list with all parsed predicates.
        """
        # matches applied predicates
        a_pred_name_pattern = re.compile('\(([^\s]+)\s')
        applied_predicates = [i[0] for i in re.findall(self.__predicate_pattern, self.__action_string)]
        if not self.__var_type_dict:
            self.__create_var_type_dict()
        # change applied predicates to normal ones
        parsed_predicates = {}
        # iterate through all used predicates
        for applied_predicate in applied_predicates:
            generalized_predicate = '('
            c_pred_name = a_pred_name_pattern.findall(applied_predicate)[0]
            if not self.__is_built_in_pred(c_pred_name):
                c_pred_vars = self.__var_pattern.findall(applied_predicate)
                generalized_predicate += c_pred_name
                # the last type used
                last_type = ''
                # a concatination of all types, used to produce an identifier for the predicate
                type_concat = ''
                # iterate through all variables of the predicate
                for c_pred_var in c_pred_vars:
                    if c_pred_var in self.__var_type_dict:
                        c_type = self.__var_type_dict[c_pred_var]
                        if last_type == '':
                            last_type = c_type
                            type_concat += c_type
                        if last_type != c_type:
                            generalized_predicate += ' - ' + last_type
                            last_type = c_type
                            type_concat += c_type
                        generalized_predicate += ' ' + c_pred_var
                    else:
                        logger.error('Variable: ' + c_pred_var + ' not defined!')
                        raise ValueError('Variable: ' + c_pred_var + ' not defined!')
                generalized_predicate += ' - ' + last_type + ')'
                # add predicate to a dictionary, to eliminate duplicats.
                # two predicates with the same name, but different types are handled as two
                # predicates at this time. Because of unknowen type hierarchies, its not decidable
                # at this time how to merge the predicates.
                parsed_predicates[c_pred_name + type_concat] = generalized_predicate
        return parsed_predicates.values()

    def __is_built_in_pred(self, name):
        """
        Checks if the Predicate is a PDDL built-in predicate.

        :param name: The name of a predicate.
        :return: Boolean: True if the predicate is a PDDL built-in predicate, false otherwise.
        """
        is_built_in = False
        if name:
            is_built_in = True if name == '=' else is_built_in
        return is_built_in
