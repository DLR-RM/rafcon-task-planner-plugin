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


class PddlFactsRepresentation:

    def __init__(self, facts_string, obj_type_map, domain_name, problem_name):
        """
        This is the representation of the pddl facts file (not completed yet.)

        :param facts_string: The facts file content as string.
        :param obj_type_map: A map, containing all objects with their types.
        :param domain_name: The name of the domain the facts file is for.
        :param problem_name: The name of the problem / task.
        """
        self.facts = facts_string
        # contains the objects of the facts file and their types as dict value.
        self.obj_type_map = obj_type_map
        self.domain_name = domain_name
        self.problem_name = problem_name

    def get_original_object_name(self, object_name):
        """
        takes the name of an object, which is now maybe uppercase, or lowercase, and returns the original format.

        :param object_name: The maybe changed representation of an object (maybe uppercase now).
        :return: String: The object name in original format, or the given object_name, if no original was found.
        """
        ori_obj_name = object_name
        if object_name:
            object_name = object_name.upper()
            for obj in self.obj_type_map.keys():
                if obj.upper() == object_name:
                    ori_obj_name = obj
                    break
        return ori_obj_name
