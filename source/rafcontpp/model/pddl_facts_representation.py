#
#
# Contributors:
# Christoph Suerig <christoph.suerig@dlr.de>
# Version 13.03.2019


class PddlFactsRepresentation:


    def __init__(self,obj_type_map):
        #contains the objects of the facts file and their types as dict value.
        self.__obj_type_map = obj_type_map


    def get_original_object_name(self, object_name):
        '''
        takes the name of an object, which is now maybe uppercase, or lowercase, and returns the original format.
        :param object_name: the maybe changed representation of an object (maybe upper-case now)
        :return: the object name in original format, or the given object_name, if no original was found.
        '''
        ori_obj_name = object_name
        if object_name:
            object_name =  object_name.upper()
            for obj in self.__obj_type_map.keys():
                if obj.upper() == object_name:
                    ori_obj_name = obj
                    break
        return ori_obj_name