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


class PlanStep:
    """Intern format, to represent a pddl-plan step.
       In this format, a plan step is splitted into the action name, and the action parameters.
       """

    def __init__(self, name, parameter):
        """
        Represents a step of the plan, in other words
        one parameterized action of the plan.

        :param name: The name of the action as String.
        :param args: The parameters of the action as [String].
        """
        self.name = name.upper()
        self.parameter = parameter

    def __str__(self):
        return "name: " + self.name + " args: " + self.parameter.__str__()
