# Contributors:
# Christoph Suerig <christoph.suerig@dlr.de>
# Version 24.05.2019
import json
from rafcontpp.model.type_tree import TypeTree
from rafcon.utils import log
logger = log.get_logger(__name__)

class TypeMerger:

    def __init__(self, datastore):

        self.__datastore = datastore


    def merge_types(self):
        """
        merge types uses the typetree data structure, to create a type tree, with types needed in the actions.
        :return: a type tree containing all (relevant) types.
        """
        pddl_actions = self.__datastore.get_pddl_action_map().values()
        type_dict = self.__dict_to_upper(json.load(open(self.__datastore.get_type_db_path(), "r")))
        tree = None
        if pddl_actions is not None and type_dict is not None:
            for action in pddl_actions:
                for type in action.types:
                    if tree:
                        if not tree.add_type_branch(type, type_dict) and not tree.is_in_tree(type):
                            logger.error("No Type \"" + type + "\" found in type dictionary!")
                            raise LookupError("No Type \"" + type + "\" found in type dictionary!")
                    else:
                        c_type = type
                        while c_type in type_dict.keys():
                            c_type = type_dict[c_type]
                        tree = TypeTree(c_type)
                        tree.recursive_insert(type, type_dict)
        return tree


    def __dict_to_upper(self, dict):
        """
         receives a dict of string:string and returns it in upper case
        :param dict: a string:string dict
        :return: a new dict in upper case.
        """
        upper_dict = dict

        if dict:
            upper_dict = {}

            for key, value in dict.iteritems():
                upper_dict[key.upper()] = value.upper()
        return upper_dict