#
#
# Contributors:
# Christoph Suerig <christoph.suerig@dlr.de>
# Version 12.11.2018
from rafcon.utils import log

logger = log.get_logger(__name__)

class PddlActionRepresentation:
    """Represents a PDDL-Action.

    """
    def __init__(self, name, action, predicates, types, requirements):
        self.name = name
        self.action = action
        self.predicates = predicates
        self.types = types
        self.requirements = requirements

    def __str__(self):
        return "name: " + self.name + "\r\naction: " + self.action + "\r\npredicates: " + str(self.predicates)



def action_to_upper(action):
    '''
     action to upper receives a action in pddl_action_representation, and retuns it in upper case
    :param action: a action in PddlActionRepresentation
    :return: the action as upper case
    '''

    if action:
        upper_types = []
        action.types = [type.upper() for type in action.types]
        action.predicates = [pred.upper() for pred in action.predicates]
        action.requirements = [req.upper() for req in action.requirements]
        action.action = action.action.upper()

    return action


