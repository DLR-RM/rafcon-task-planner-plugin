

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
        return "name: " + self.name + " action: " + self.action + " predicates: " + self.predicates