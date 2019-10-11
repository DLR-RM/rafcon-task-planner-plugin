# Copyright (C) 2018-2019 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the 3-Clause BSD License which accompanies this
# distribution, and is available at
# https://opensource.org/licenses/BSD-3-Clause
#
# Contributors:
# Christoph Suerig <christoph.suerig@dlr.de>

# Don't connect with the Copyright comment above!
# Version: 07.06.2019
from rafcon.utils import log

logger = log.get_logger(__name__)


class PredicateMerger:
    """
    This class merges predicates with the same identifier, but different types of the same branch together.
    """

    __name_index = 0

    def __init__(self, datastore):
        """
        :param datastore: A datastore containing available types.
        """
        self.__datastore = datastore

    def merge_predicates(self, predicates):
        """
        Merge predicates merges all predicates, sets all available predicates in datastore
        and returns all merged predicates as strings.

        :param predicates: A list [string] with predicates.
        :return: ([String],[(String,[(String,int)])]): A tuple (list [string] with merged predicates, and a list [of format ('LOCATED',[(VEHICLE,1),(PHYSOBJ,3)])] all available predicates)
        """
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
        return (merged_preds_as_string, available_predicates)

    def __parse_predicate(self, predicate_string):
        """
        parse_predicate gets a predicate string and parses it into a useful tuple of (predicate_Name,[(type_name,occurance)]).

        :param predicate_string: A predicate as string e.g (LOCATED ?VEH - VEHICLE ?OBJ ?sObj ?thirdObj - PHYSOBJ).
        :return: (String,[(String,int)]): A parsed predicate as tuple e.g. ('LOCATED',[(VEHICLE,1),(PHYSOBJ,3)]).
        """
        pred_name = None
        pred_types = []
        pred = predicate_string
        if '(' in predicate_string and '?' in predicate_string:
            start = predicate_string.index('(')
            end = predicate_string.index('?')
            if start + 1 >= end - 1:
                logger.error("Can't parse predicate: " + predicate_string)
                raise ValueError("Can't parse predicate: " + predicate_string)
            pred_name = predicate_string[start + 1:end - 1].replace(' ', '')
            pred = predicate_string[end:]
        else:
            logger.error("Can't parse predicate: " + predicate_string)
            raise ValueError("Can't parse predicate: " + predicate_string)

        if not '-' in pred:
            logger.error("Can't parse predicate: " + predicate_string)
            raise ValueError("Can't parse predicate: " + predicate_string)
        while '-' in pred:
            c_type_s = pred.index('-') + 1
            c_type_e = 0
            if '?' in pred[c_type_s:]:
                c_type_e += pred[c_type_s:].index('?') + c_type_s
            elif ')' in pred:
                c_type_e += pred.index(')')
            else:
                logger.error("Can't parse predicate: " + predicate_string)
                raise ValueError("Can't parse predicate: " + predicate_string)
            pred_types.append((pred[c_type_s:c_type_e].replace(' ', ''), pred[:c_type_e].count('?')))
            pred = pred[c_type_e:]
        return (pred_name, pred_types)

    def __reduce_predicate_list(self, predicate_list):
        """
        reduce_predicate_list gets a list of predicates, with the same name but different types,
        and reduces them to one predicate, with the most open types.

        :param predicate_list: A list of predicates with the same name, format: ('LOCATED',[(VEHICLE,1),(PHYSOBJ,3)])
        :return: (String,[(String,int)]): One predicate tuple, containing the most general types. of format ('LOCATED',[(VEHICLE,1),(PHYSOBJ,3)])
        """
        type_tree = self.__datastore.get_available_types()
        # the resulting predicate
        result_predicate = predicate_list[0]

        for predicate in predicate_list:
            err_str = "Can't merge predicates, they are Incompatible! (variable names where changed) first: " + \
                      self.__tuple_to_predicate_string(result_predicate) + \
                      " second: " + self.__tuple_to_predicate_string(predicate)
            # cant merge, if they have different names, or different number of argument types.
            if result_predicate[0] != predicate[0] or len(result_predicate[1]) != len(predicate[1]):
                logger.error(err_str)
                raise ValueError(err_str)
            for index, type_tuple in enumerate(predicate[1]):
                # cant merge, if they have different number of arguments per type.
                if type_tuple[1] != result_predicate[1][index][1]:
                    logger.error(err_str)
                    raise ValueError(err_str)
                # try to merge the types used in predicates e.g. Robot or Vehicle
                if result_predicate[1][index][0] != type_tuple[0]:
                    smallest_parent = type_tree.get_smallest_parent(type_tuple[0], result_predicate[1][index][0])
                    if smallest_parent:
                        # set smallest_parent type as predicate type
                        result_predicate[1][index] = (smallest_parent, result_predicate[1][index][1])
                        # just to warn the user, that a predicate is a root-type-predicate.
                        if (type_tree.get_parent_of(smallest_parent) is None):
                            logger.warn('Predicate merged to root Type predicate: ' + self.__tuple_to_predicate_string(
                                result_predicate))
                    else:
                        logger.error(err_str)
                        raise ValueError(err_str)
        return result_predicate

    def __tuple_to_predicate_string(self, predicate_tuple):
        """
        Receives a predicate tuple and returns it as predicate string.

        :param predicate_tuple: A tuple in format (PREDICATE_NAME,[(TYPE,NUM_VARIABLES)])
        :return: String: A predicate string e.g (PREDICATENAME ?0 ?1 - Type).
        """
        pred_string = '(' + predicate_tuple[0]
        tuple_counter = 0  # need this counter do guarantee distinct variable names.
        for type_tup in predicate_tuple[1]:
            variable_counter = 0
            tuple_counter += 1
            while variable_counter < type_tup[
                1]:  # NUM_VARIABLES: type_tup[1] contains the number of variables of one type.
                pred_string += ' ?' + type_tup[0][:1] + str(tuple_counter) + str(variable_counter)
                variable_counter += 1
            pred_string += ' - ' + type_tup[0]
        pred_string += ')'
        return str(pred_string)
