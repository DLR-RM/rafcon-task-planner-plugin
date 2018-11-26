import os
from rafcontpp.model.datastore import SEMANTIC_DATA_DICT_NAME
from rafcontpp.logic.pddl_action_parser import PddlActionParser
from rafcon.core.singleton import library_manager
from rafcon.core.config import global_config
from rafcon.utils import log

logger = log.get_logger(__name__)



class Mapper:
    '''Mapper
    The Mapper maps actions and states together.
    '''

    def __init__(self,datastore):

        if datastore is None:
            logger.error("Datastore in Mapper can not be None!")
            raise ValueError("Datastore in Mapper can not be None!")
        self.__datastore = datastore



    def generate_action_state_map(self):
        '''
        generates a map, with pddl action names as key, and RAFCON States as Values.
        :return: Noting, its writing the map into the datastore
        '''

        state_libs = self.__datastore.get_state_pools()
        libraries = global_config.get_config_value("LIBRARY_PATHS")
        lib_names=[]
        for pool in state_libs:
            logger.debug("adding library path: "+str(pool))
            lib_name = os.path.basename(pool)
            lib_names.append(lib_name)
            libraries[lib_name] = os.path.abspath(pool)

        global_config.set_config_value("LIBRARY_PATHS", libraries)
        library_manager.refresh_libraries()
        action_state_map = {}
        for lib_name in lib_names:
            state_pool = library_manager.libraries[lib_name]
            for state in state_pool:
                lib_state = library_manager.get_library_instance(lib_name, state)
                sem_data = lib_state.state_copy.semantic_data

                if SEMANTIC_DATA_DICT_NAME in sem_data \
                        and isinstance(sem_data[SEMANTIC_DATA_DICT_NAME]['pddl_action'], unicode):
                    action_string = str(sem_data[SEMANTIC_DATA_DICT_NAME]['pddl_action']).upper()
                    action_name = PddlActionParser(action_string).parse_action_name()
                    if action_name in action_state_map:
                        logger.warning("Multiple association of action " + str(action_name)
                                       + " associated with states " + str(action_state_map[action_name])
                                       + " and " + str(state))
                    else:
                        action_state_map[action_name] = state
                else:
                    logger.warning("State " + state + " is not associated with any PDDL Action!" )

        if not action_state_map:
            logger.warning("No States with semantic PDDL_Action data found!")
        logger.debug('action_state_map has '+str(len(action_state_map.keys())) + ' entires.')
        self.__datastore.set_action_state_map(action_state_map)



    def generate_state_action_map(self):
        '''
        generates a map with RAFCON States as Keys and PDDL Action names as values.
        if no action_state_map exists, it calls generate action_state_map
        :return: nothing, its writing the map into the datastore
        '''
        if self.__datastore.get_action_state_map() is None:
            self.generate_action_state_map()
        action_state_map = self.__datastore.get_action_state_map()

        state_action_map = {}

        for action in action_state_map.keys():
            c_state = action_state_map[action]

            if c_state in state_action_map:
                logger.warning("Multiple associations of state " + str(c_state)
                               + " associated with actions: " + str(state_action_map[c_state])
                               + " and " +str(c_state))
            else:
                state_action_map[c_state] = action

            if not state_action_map:
                logger.warning("could not generate state_action_map, action_state_map was empty!")
        logger.debug('state_action_map has '+str(len(state_action_map.keys()))+' entries.')
        self.__datastore.set_state_action_map(state_action_map)



    def generate_available_actions(self):
        '''
        takes the action_state_map, and extracts the keys, in order to get a list of all available PDDL Actions.
        if no action_state_map exists, it calls generate_action_state_map() first.
        :return: nothing, it writes the list with available actions into the datastore.
        '''
        if self.__datastore.get_action_state_map() is None:
            self.generate_action_state_map()
        action_state_map = self.__datastore.get_action_state_map()
        logger.debug('list of available actions has '+str(len(action_state_map.keys())) + ' entires.')
        self.__datastore.set_available_actions(action_state_map.keys())

