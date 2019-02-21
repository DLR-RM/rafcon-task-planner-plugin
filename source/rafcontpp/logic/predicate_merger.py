#
#
#
#
#
#
# Contributors:
# Christoph Suerig <christoph.suerig@dlr.de>
# Version: 20.02.2019
from rafcon.utils import log

logger = log.get_logger(__name__)

class PredicateMerger:
    '''PredicateMerger
    '''

    __name_index = 0

    def __init__(self, datastore):
        '''
        :param datastore: a datastore containing all necassary data
        '''
        self.__datastore = datastore


    def merge_predicates(self,predicates):
        '''merge predicates merges all predicates, sets all available predicates in datastore
        and returns all merged predicates as strings.
        :param predicates: a list [string] with predicates
        :return: a tuple (list [string] with merged predicates, and a list [of format ('LOCATED',[(VEHICLE,1),(PHYSOBJ,3)])] all available predicates)
        '''
        if predicates is None:
            raise ValueError('predicates can not be None!')
        preds_map = {}
        available_predicates = []
        merged_preds_as_string = []
        for predicate in predicates:
            parsed_pred = self.__parse_predicate(predicate)
            if parsed_pred[0] in preds_map.keys():
              preds_map[parsed_pred[0]].append(parsed_pred)
            else:
                preds_map[parsed_pred[0]] = [parsed_pred]

        for key in preds_map.keys():
            c_pred = self.__reduce_predicate_list(preds_map[key])
            available_predicates.append(c_pred)
            merged_preds_as_string.append(self.__tuple_to_predicate_string(c_pred))

        return (merged_preds_as_string,available_predicates)


    def __parse_predicate(self,predicate_string):
        '''parse_predicate
        parse_predicate gets a predicate string and parses it into a useful tuple of (predicate_Name,[(type_name,occurance)])
        :param predicate_string: a predicate as string e.g (LOCATED ?VEH - VEHICLE ?OBJ ?sObj ?thirdObj - PHYSOBJ)
        :return: a parsed predicate as tuple e.g. ('LOCATED',[(VEHICLE,1),(PHYSOBJ,3)])
        '''
        pred_name = None
        pred_types = []
        pred = predicate_string
        if '(' in predicate_string and '?' in predicate_string:
            start = predicate_string.index('(')
            end = predicate_string.index('?')
            if start+1 >= end-1:
                logger.error("Can't parse predicate: "+predicate_string)
                raise ValueError("Can't parse predicate: "+predicate_string)

            pred_name = predicate_string[start+1:end-1].replace(' ','')
            pred = predicate_string[end:]
        else:
            logger.error("Can't parse predicate: " + predicate_string)
            raise ValueError("Can't parse predicate: " + predicate_string)


        if not '-' in pred:
            logger.error("Can't parse predicate: " + predicate_string)
            raise ValueError("Can't parse predicate: " + predicate_string)

        while '-' in pred:
            c_type_s = pred.index('-')+1
            c_type_e = 0
            if '?' in pred[c_type_s:]:
                c_type_e += pred[c_type_s:].index('?') + c_type_s

            elif ')' in pred:
                c_type_e += pred.index(')')
            else:
                logger.error("Can't parse predicate: " + predicate_string)
                raise ValueError("Can't parse predicate: " + predicate_string)
            pred_types.append((pred[c_type_s:c_type_e].replace(' ', ''),pred[:c_type_e].count('?')))
            pred = pred[c_type_e:]

        return (pred_name,pred_types)


    def __reduce_predicate_list(self,predicate_list):
        '''reduce_predicate_list
        reduce_predicate_list gets a list of predicates, with the same name but different types,
        and reduces them to one predicate, with the most open types.
        :param predicate_list: a list with predicates, all having the same name, of format ('LOCATED',[(VEHICLE,1),(PHYSOBJ,3)])
        :return: one predicate tuple, containing the most general types. of format ('LOCATED',[(VEHICLE,1),(PHYSOBJ,3)])
        '''
        type_tree = self.__datastore.get_available_types()
        #the resulting predicate
        reduced_list = predicate_list[0]

        for predicate in predicate_list:
            err_str = "Can't merge predicates, they are Incompatible! (variable names where changed) first: " +\
                      self.__tuple_to_predicate_string(reduced_list) + \
                      " second: " + self.__tuple_to_predicate_string(predicate)
            #cant merge, if they have different names, or different number of argument types.
            if reduced_list[0] != predicate[0] or len(reduced_list[1]) != len(predicate[1]):
                logger.error(err_str)
                raise ValueError(err_str)
            for index, type_tuple in enumerate(predicate[1]):
                #cant merge, if they have different number of arguments per type.
                if type_tuple[1] != reduced_list[1][index][1]:
                    logger.error(err_str)
                    raise ValueError(err_str)


                # look if type_tuple is parent
                if type_tree.is_parent_of(type_tuple[0],reduced_list[1][index][0]):
                    #set reduced tuple to parent
                    reduced_list[1][index] = type_tuple
                # if they are not equal, and current reduced_list type is not parent, whe have to find the "smalest" parent
                elif reduced_list[1][index][0] != type_tuple[0] \
                        and not type_tree.is_parent_of(reduced_list[1][index][0],type_tuple[0]):
                        #cant be none, because we ensure, that tyes are not the same, so they cant be root type.
                        parent_type_tuple = type_tree.get_parent_of(type_tuple[0])
                        parent_reduced_list = type_tree.get_parent_of(reduced_list[1][index][0])
                        while parent_type_tuple:
                            c_parent_reduced_list = parent_reduced_list
                            while c_parent_reduced_list:
                                if c_parent_reduced_list == parent_type_tuple:
                                    parent_reduced_list = c_parent_reduced_list
                                    break
                                else:
                                    c_parent_reduced_list = type_tree.get_parent_of(c_parent_reduced_list)
                             #break if found
                            if parent_type_tuple == parent_reduced_list:
                                break
                            #go higher in hierarchy
                            parent_type_tuple = type_tree.get_parent_of(parent_type_tuple)

                        if parent_type_tuple is not None and parent_type_tuple == parent_reduced_list:
                            #set parent type as predicate type
                            reduced_list[1][index] = (parent_type_tuple,reduced_list[1][index][1])
                            #just to warn the user...
                            if  (type_tree.get_parent_of(parent_type_tuple) is None):
                                logger.warn('Predicate merged to root Type predicate: '+self.__tuple_to_predicate_string(reduced_list))

                        else:
                            logger.error(err_str)
                            raise ValueError(err_str)





        return reduced_list





    def __tuple_to_predicate_string(self,predicate_tuple):
        '''
        receives a predicate tuple and returns it as predicate string.
        :param predicate_tuple: a tuple in format (PREDICATE_NAME,[(TYPE,NUM_VARIABLES)])
        :return: a predicate string (PREDICATENAME ?0 ?1 - Type)
        '''

        pred_string = '('+predicate_tuple[0]
        variable_counter = 0 #need this counter do guarantee distinctly variable names.
        for type_tup in predicate_tuple[1]:
            c_count = 0
            while c_count < type_tup[1]:
                pred_string += ' ?' + type_tup[0][:1] + str(variable_counter) + str(c_count)
                variable_counter += 1
                c_count += 1
            pred_string += ' - '+type_tup[0]

        pred_string += ')'

        return str(pred_string)

