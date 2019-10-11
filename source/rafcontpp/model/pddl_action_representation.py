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
# Version 12.11.2018

from rafcon.utils import log

logger = log.get_logger(__name__)


class PddlActionRepresentation:
    """
    This Module represents a PDDL Action. Currently just name, predicates, types, requirements and parameters.
    """
    def __init__(self, name, action, predicates, types, requirements, parameters):
        """
        :param name: The name of the action.
        :param action: The pddl action as string.
        :param predicates: A list of predicates used in the action.
        :param types: A list of types used in the action.
        :param requirements: A list of requirements.
        :param parameters: The parameter names of the action without the ? symbol.
        """
        self.name = name
        self.action = action
        self.predicates = predicates
        self.types = types
        self.requirements = requirements
        self.parameters = parameters

    def __str__(self):
        return "name: " + self.name + "\r\naction: " + self.action + "\r\npredicates: " + str(self.predicates)

def action_to_upper(action):
    """
     action to upper receives an action in pddl_action_representation, and returns it in upper case.

    :param action: A action in PddlActionRepresentation
    :return: PddlActionRepresentation: The action in upper case
    """
    if action:
        action.name = action.name.upper()
        action.types = [type.upper() for type in action.types]
        action.predicates = [pred.upper() for pred in action.predicates]
        action.requirements = [req.upper() for req in action.requirements]
        action.action = action.action.upper()
    return action
