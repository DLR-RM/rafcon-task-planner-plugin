#
#
#Contributors:
#Christoph Suerig <christoph.suerig@dlr.de>
#Version 17.05.2019


class PddlFactsRepresentation:


    def __init__(self, facts_string, obj_type_map, domain_name, problem_name):
        """
        This is the representation of the pddl facts file (not completed yet.)
        :param facts_string: the facts file content as string.
        :param obj_type_map: a map, containing all objects with their types.
        :param domain_name: the name of the domain the facts file is for.
        :param problem_name: the name of the problem / task.
        """
        self.facts = facts_string
        #contains the objects of the facts file and their types as dict value.
        self.obj_type_map = obj_type_map
        self.domain_name = domain_name
        self.problem_name = problem_name


    def get_original_object_name(self, object_name):
        """
        takes the name of an object, which is now maybe uppercase, or lowercase, and returns the original format.
        :param object_name: the maybe changed representation of an object (maybe upper-case now)
        :return: the object name in original format, or the given object_name, if no original was found.
        """
        ori_obj_name = object_name
        if object_name:
            object_name =  object_name.upper()
            for obj in self.obj_type_map.keys():
                if obj.upper() == object_name:
                    ori_obj_name = obj
                    break
        return ori_obj_name