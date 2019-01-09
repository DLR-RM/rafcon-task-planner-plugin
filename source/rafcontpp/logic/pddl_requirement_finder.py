import re
from rafcon.utils import log
logger = log.get_logger(__name__)



class PddlRequirementFinder():

    def __init__(self, action_string):
        self.action = action_string
        if not self.action or len(self.action) == 0:
            logger.error("Can't find requirements in None or empty action string.")
            raise ValueError("Can't find requirements in None or empty action string.")

    def strips(self):
        return True

    def typing(self):
        return self.action.find(' - ') > -1

    def disjunctive_preconditions(self):
        pattern = re.compile(':precondition.*not.*:effect',re.IGNORECASE | re.MULTILINE | re.DOTALL)
        return re.search(pattern,self.action) is None

    def existential_preconditions(self):
        pattern = re.compile(':precondition.*exists.*:effect', re.IGNORECASE | re.MULTILINE | re.DOTALL)
        return re.search(pattern, self.action) is None

    def universal_preconditions(self):
        pattern = re.compile(':precondition.*forall.*:effect', re.IGNORECASE | re.MULTILINE | re.DOTALL)
        return re.search(pattern, self.action) is None
    #slast was universal_preconditions

    def equality(self):
        return self.action.find('=') > -1

    def conditional_effects(self):
        when_pattern = re.compile('effect.*\(\s*when[\s+|\(]', re.IGNORECASE | re.MULTILINE | re.DOTALL)
        return re.search(when_pattern,self.action) is not None

    def action_expansions(self):
        expansions_pattern = re.compile(':expansion', re.IGNORECASE)
        return re.search(expansions_pattern,self.action) is not None

    def foreach_expansions(self):
        requires = False
        if self.action_expansions():
            expansions_pattern = re.compile(':expansion', re.IGNORECASE)
            expansion = self.action[re.search(expansions_pattern,self.action).start():]
            foreach_pattern = re.compile('foreach',re.IGNORECASE)
            requires = re.search(foreach_pattern,expansion) is not None
        return requires

    def dag_expansions(self):
        #TODO implement
        return False

    def fluents(self):
        requires = False
        in_dec_pattern = re.compile('effect.*\(\s*(increase|decrease)[\s+|\(]', re.IGNORECASE | re.MULTILINE | re.DOTALL)
        if re.search(in_dec_pattern,self.action) is not None:
            requires = True
        if re.search('<|>',self.action) is not None:
            requires = True

        return requires



