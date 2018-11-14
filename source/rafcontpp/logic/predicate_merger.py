from rafcontpp.model.type_tree import TypeTree
from rafcon.utils import log

logger = log.get_logger(__name__)

class PredicateMerger:
    '''PredicateMerger
    PredicateMerger
    '''

    __current_name_index = 0

    def __init__(self, type_tree):
        self.__type_tree = type_tree


    def merge_predicates(self,predicates):
        '''

        :param predicates: a list [string] with predicates
        :return: a list [string] with merged predicates
        '''
        preds_map = {}
        merged_predicates = []
        for predicate in predicates:
            parsed_pred = self.__parse_predicate(predicate)
            if parsed_pred[0] in preds_map.keys():
              preds_map[parsed_pred[0]].append(parsed_pred)
            else:
                preds_map[parsed_pred[0]] = [parsed_pred]

        for key in preds_map.keys():
            merged_predicates.append(self.__tuple_to_predicate_string(self.__reduce_predicate_list(preds_map[key])))

        return merged_predicates






    def __get_current_name_index(self):
        #self.__current_name_index += 1
        return self.__current_name_index


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
                logger.debug('start_index: '+str(start))
                logger.debug('end_index: '+str(end))
                logger.error("Can't parse predicate name: "+predicate_string)
                raise ValueError("Can't parse predicate name: "+predicate_string)

            pred_name = predicate_string[start+1:end-1].replace(' ','')
            pred = predicate_string[end:]
        else:
            logger.error("Can't parse predicate name: " + predicate_string)
            raise ValueError("Can't parse predicate name: " + predicate_string)



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
        reduce_predicate_list gets a list of predicated, with the same name but different types,
        and reduces them to one predicate, with the most open types.
        :param predicate_list: a list with predicates, all having the same name
        :return: one predicate tuple, containing the most open types.
        '''

        reduced_list = predicate_list[0]

        for predicate in predicate_list:
            err_str = "Can't merge predicates, they are Incompatible! first: " +\
                      self.__tuple_to_predicate_string(reduced_list) + \
                      " second: " + self.__tuple_to_predicate_string(predicate)
            if reduced_list[0] != predicate[0] or len(reduced_list[1]) != len(predicate[1]):
                logger.error(err_str)
                raise ValueError(err_str)
            for index, type_tuple in enumerate(predicate[1]):

                if type_tuple[1] != reduced_list[1][index][1]:
                    logger.error(err_str)
                    raise ValueError(err_str)


                # look if type_tuple is parent
                if self.__type_tree.is_parent_of(type_tuple[0],reduced_list[1][index][0]):
                    #set reduced tuple to parent
                    reduced_list[1][index] = type_tuple
                # if they are not equal, and current reduced_list type is not parent, there must be an error!
                elif reduced_list[1][index][0] != type_tuple[0] \
                        and not self.__type_tree.is_parent_of(reduced_list[1][index][0],type_tuple[0]):
                    logger.error(err_str)
                    raise ValueError(err_str)

        if len(predicate_list) > 1:
            logger.warning('Predicate ' + self.__tuple_to_predicate_string(reduced_list) +
                           ' had to be merged, so you may have inconsistencies!')

        return reduced_list





    def __tuple_to_predicate_string(self,predicate_tuple):
        #(LOCATED ?VEH - VEHICLE ?OBJ - PHYSOBJ)
        pred_string = '('+predicate_tuple[0]

        for type_tup in predicate_tuple[1]:
            c_count = 0
            while c_count < type_tup[1]:
                pred_string += ' ?' + type_tup[0][:1] + str(self.__get_current_name_index()) + str(c_count)
                c_count +=1
            pred_string += ' - '+type_tup[0]

        pred_string += ')'

        return str(pred_string)

