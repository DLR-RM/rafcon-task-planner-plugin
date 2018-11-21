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




def parse_action_name(pddl_action_string):
    action_name = None
    start_index = pddl_action_string.find(':ACTION') +7
    end_index = 0
    if start_index > 7 and start_index < len(pddl_action_string):
        end_index = pddl_action_string.find(':PARAMETERS')

    if end_index > start_index and end_index < len(pddl_action_string):
        action_name = pddl_action_string[start_index:end_index].strip()

    if action_name is None:
        logger.warn('Couldn \'t parse action name from: '+pddl_action_string)

    return action_name


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


