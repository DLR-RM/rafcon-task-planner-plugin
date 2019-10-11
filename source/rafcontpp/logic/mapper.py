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
# Version 05.07.2019
import os

from rafcon.core.config import global_config
from rafcon.core.singleton import library_manager
from rafcon.utils import log

from rafcontpp.logic.pddl_action_parser import PddlActionParser
from rafcontpp.model.datastore import SEMANTIC_DATA_DICT_NAME, PDDL_ACTION_SUB_DICT_NAME

logger = log.get_logger(__name__)


class Mapper:
    """Mapper
    The Mapper maps pddl actions and rafcon states together.
    """

    def __init__(self, datastore):
        """
        :param datastore: A datastore containing state pools.
        """
        if datastore is None:
            logger.error("Datastore in Mapper can not be None!")
            raise ValueError("Datastore in Mapper can not be None!")
        self.__datastore = datastore

    def generate_action_state_map(self):
        """
        generates a map, with pddl action names as key, and RAFCON States as Values.
        Writes the map into the datastore.

        :return: void
        """
        state_libs = self.__datastore.get_state_pools()
        libraries = global_config.get_config_value("LIBRARY_PATHS")
        lib_names = []
        for pool in state_libs:
            logger.debug("adding library path: " + str(pool))
            lib_name = os.path.basename(pool)
            lib_names.append(lib_name)
            libraries[lib_name] = os.path.abspath(pool)
        global_config.set_config_value("LIBRARY_PATHS", libraries)  # also refreshes libraries
        action_state_map = {}
        for lib_name in lib_names:
            state_pool = library_manager.libraries[lib_name]
            for state in state_pool:
                lib_state = library_manager.get_library_instance(lib_name, state)
                sem_data = lib_state.state_copy.semantic_data
                if isinstance(sem_data[SEMANTIC_DATA_DICT_NAME][PDDL_ACTION_SUB_DICT_NAME]['pddl_action'], unicode):
                    action_string = str(
                        sem_data[SEMANTIC_DATA_DICT_NAME][PDDL_ACTION_SUB_DICT_NAME]['pddl_action']).upper()
                    action_name = PddlActionParser(action_string).parse_action_name()
                    if action_name in action_state_map:
                        logger.warning(
                            "Multiple associations of action {}! Associated with states: {} and {}. Using association with {}."
                            .format(action_name, action_state_map[action_name], state, action_state_map[action_name]))
                    else:
                        action_state_map[action_name] = state
                else:
                    logger.warning("State {} is not associated with any PDDL Action!".format(state))
        if not action_state_map:
            logger.error("No States with semantic PDDL_Action data found!")
            raise ValueError("No States with semantic PDDL_Action data found!")
        logger.debug('action_state_map has ' + str(len(action_state_map.keys())) + ' entires.')
        self.__datastore.set_action_state_map(action_state_map)

    def generate_state_action_map(self):
        """
        Generates a map with RAFCON States as Keys and PDDL Action names as values.
        If no action_state_map exists, it calls generate action_state_map. It's writing the map into the datastore.

        :return: void
        """
        if self.__datastore.get_action_state_map() is None:
            self.generate_action_state_map()
        action_state_map = self.__datastore.get_action_state_map()
        state_action_map = {}
        for action in action_state_map.keys():
            c_state = action_state_map[action]
            if c_state in state_action_map:
                logger.warning(
                    "Multiple associations of state {}! Associated with actions {} and {}. Using association with {}"
                    .format(c_state, state_action_map[c_state].name, action.name, state_action_map[c_state].name))
            else:
                state_action_map[c_state] = action
            if not state_action_map:
                logger.warning("could not generate state_action_map, action_state_map was empty!")
        logger.debug('state_action_map has ' + str(len(state_action_map.keys())) + ' entries.')
        self.__datastore.set_state_action_map(state_action_map)

    def generate_available_actions(self):
        """
        Takes the action_state_map, and extracts the keys, in order to get a list of all available PDDL Actions.
        If no action_state_map exists, it calls generate_action_state_map() first. It writes the list with available
        actions into the datastore.

        :return: void
        """
        if self.__datastore.get_action_state_map() is None:
            self.generate_action_state_map()
        action_state_map = self.__datastore.get_action_state_map()
        logger.debug('list of available actions has ' + str(len(action_state_map.keys())) + ' entires.')
        self.__datastore.set_available_actions(action_state_map.keys())
