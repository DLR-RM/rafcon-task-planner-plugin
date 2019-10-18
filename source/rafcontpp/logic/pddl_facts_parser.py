# Copyright (C) 2019 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the 3-Clause BSD License which accompanies this
# distribution, and is available at
# https://opensource.org/licenses/BSD-3-Clause
#
# Contributors:
# Christoph Suerig <christoph.suerig@dlr.de>

# Don't connect with the Copyright comment above!
# Version 17.05.2019


import re

from rafcon.utils import log

logger = log.get_logger(__name__)


class PddlFactsParser:

    def __init__(self, facts_string):
        """
        The facts file parser will later be used to parse a raw facts file into a pddl facts file representation.

        :param facts_string: The facts file content as string.
        """
        if not facts_string:
            logger.error('facts_string can\'t be None!')
            raise ValueError('facts_string can\'t be None!')
        self.__facts_string = self.__clean_comments(facts_string)
        # matches the whole objects section, without (:objects and )
        self.__objects_section_pattern = re.compile('\(:objects([^\)]+)\)', re.IGNORECASE | re.MULTILINE)
        self.__objects_type_pattern = re.compile('([^-]+\s+-\s+[^\s]+)')
        # matches the domain_name in a facts file
        self.__domain_name_pattern = re.compile('\(\s*:domain\s+([^\s|^\)]+)', re.IGNORECASE)
        # machtes the problem name in a facts file
        self.__problem_name_pattern = re.compile('\(\s*problem\s+([^\s|^\)]+)', re.IGNORECASE)

    def parse_objects(self):
        """
        Parse object parses the Objects present in the facts file, and returns a map of all objects with their type,
        in format: object: type

        :return: {String:String}: A map of format: key: Object, value: type.
        """
        obj_type_map = {}
        obj_sec_tup = self.__objects_section_pattern.findall(self.__facts_string)
        if obj_sec_tup and len(obj_sec_tup) == 1:
            obj_sec = obj_sec_tup[0]  # a string containing the object section
            obj_types = self.__objects_type_pattern.findall(obj_sec)  # a list with all object - type patterns
            for obj_type in obj_types:
                obj_type_list = obj_type.split()  # split list
                obj_type_list.remove('-')
                c_type = obj_type_list[
                    len(obj_type_list) - 1]  # type is always the last item in list (because of the match)
                obj_list = list(obj_type_list)
                obj_list.pop()  # remove the type, to get only the objects in the list.
                for obj in obj_list:
                    if obj in obj_type_map.keys() and c_type != obj_type_map[obj]:
                        logger.error(
                            'Object {0:} assigned as type {1:} and type {2:}.'.format(obj, c_type, obj_type_map[obj]))
                        raise ValueError(
                            'Object {0:} assigned as type {1:} and type {2:}.'.format(obj, c_type, obj_type_map[obj]))
                    else:
                        obj_type_map[obj] = c_type
        else:
            logger.error('No Objects found in facts file!')
            raise ValueError()
        return obj_type_map

    def parse_domain_name(self):
        """
        parse_domain_name parses the domain name out of the given facts file.

        :return: String: The domain name.
        """
        parsed = self.__domain_name_pattern.findall(self.__facts_string)
        if len(parsed) == 0:
            logger.error("Couldn't parse domain name from facts file!")
            raise ValueError("Couldn't parse domain name!")
        return parsed[0]

    def parse_problem_name(self):
        """
        parse_problem_name parses the problem name out of the given facts file.

        :return: String: The problem name.
        """
        parsed = self.__problem_name_pattern.findall(self.__facts_string)
        if len(parsed) == 0:
            logger.error("Couldn't parse problem name from facts file!")
            raise ValueError("Couldn't parse problem name!")
        return parsed[0]

    def __clean_comments(self, facts_string):
        """
        Takes the facts string and removes all comments.

        :return: String: The facts string without comments.
        """
        comment_pattern = re.compile('(;[^\n]*)')
        return comment_pattern.sub('', facts_string)
