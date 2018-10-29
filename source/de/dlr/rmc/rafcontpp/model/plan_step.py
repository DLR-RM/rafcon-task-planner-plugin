

class PlanStep:
    """Intern format, to represent a pddl-plan step.
       In this format, a plan step is splitted into the action name, and the action parameters.
       """

    def __init__(self, name, parameter):
        '''
        Represents a step of the plan, in other words
        one parameterized action of the plan.
        :param name: The name of the action as String
        :param args: The parameters of the action as [String]
        '''
        self.name = name.upper()
        self.parameter = parameter

    def __str__(self):
        return "name: " + self.name + " args: " + self.parameter.__str__()