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
# Version 31.05.2019
import re

from rafcon.utils import log

logger = log.get_logger(__name__)


class PddlRequirementFinder():
    """
    The PddlRequirementFiner, tries to figure out requirments from a given action definition.
    Its doing this according to PDDL 2.1. It's not completed yet.
    """

    def __init__(self, action_string):
        """

        :param action_string: An string, containing a pddl action.
        """
        self.action = action_string
        if not self.action or len(self.action) == 0:
            logger.error("Can't find requirements in None or empty action string.")
            raise ValueError("Can't find requirements in None or empty action string.")

    def strips(self):
        return True

    def typing(self):
        return self.action.find(' - ') > -1

    def disjunctive_preconditions(self):
        not_pattern = re.compile(':precondition.*\(\s*not.*:effect', re.IGNORECASE | re.MULTILINE | re.DOTALL)
        or_pattern = re.compile(':precondition.*\(\s*or.*:effect', re.IGNORECASE | re.MULTILINE | re.DOTALL)
        imply_pattern = re.compile(':precondition.*\(\s*imply.*:effect', re.IGNORECASE | re.MULTILINE | re.DOTALL)
        return not_pattern.search(self.action) is not None \
               or imply_pattern.search(self.action) is not None or or_pattern.search(self.action) is not None

    def existential_preconditions(self):
        pattern = re.compile(':precondition.*\(\s*exists.*:effect', re.IGNORECASE | re.MULTILINE | re.DOTALL)
        return bool(pattern.search(self.action))

    def universal_preconditions(self):
        pattern = re.compile(':precondition.*\(\s*forall.*:effect', re.IGNORECASE | re.MULTILINE | re.DOTALL)
        return bool(pattern.search(self.action))

    def quantified_preconditions(self):
        return self.existential_preconditions() and self.universal_preconditions()

    def equality(self):
        return self.action.find('=') > -1

    def conditional_effects(self):
        when_pattern = re.compile('effect.*\(\s*when[\s+|\(]', re.IGNORECASE | re.MULTILINE | re.DOTALL)
        forall_pattern = re.compile('effect.*\(\s*forall[\s+|\(]', re.IGNORECASE | re.MULTILINE | re.DOTALL)
        return when_pattern.search(self.action) is not None or forall_pattern.search(self.action) is not None

    def action_expansions(self):
        if self.dag_expansions():
            return True
        if self.foreach_expansions():
            return True
        expansions_pattern = re.compile(':expansion', re.IGNORECASE)
        if re.search(expansions_pattern, self.action):
            return True
        return False

    def foreach_expansions(self):
        requires = False
        expansions_pattern = re.compile(':expansion', re.IGNORECASE)
        if expansions_pattern.search(self.action):
            expansion = self.action[re.search(expansions_pattern, self.action).start():]
            foreach_pattern = re.compile('foreach', re.IGNORECASE)
            requires = re.search(foreach_pattern, expansion) is not None
        return requires

    def dag_expansions(self):
        requires = False
        expansions_pattern = re.compile(':expansion', re.IGNORECASE)
        if expansions_pattern.search(self.action):
            expansion = self.action[re.search(expansions_pattern, self.action).start():]
            foreach_pattern = re.compile('constrained', re.IGNORECASE)
            requires = re.search(foreach_pattern, expansion) is not None
        return requires

    def fluents(self):
        requires = False
        in_dec_pattern = re.compile('effect.*\(\s*(increase|decrease)[\s+|\(]',
                                    re.IGNORECASE | re.MULTILINE | re.DOTALL)
        if re.search(in_dec_pattern, self.action):
            requires = True
        return requires

    def expression_evaluation(self):
        if self.fluents():
            return True
        return False

    def domain_axioms(self):
        if self.expression_evaluation():
            return True
        return False

    def adl(self):
        if not self.strips():
            return False
        if not self.typing():
            return False
        if not self.disjunctive_preconditions():
            return False
        if not self.equality():
            return False
        if not self.quantified_preconditions():
            return False
        if not self.conditional_effects():
            return False
        return True
