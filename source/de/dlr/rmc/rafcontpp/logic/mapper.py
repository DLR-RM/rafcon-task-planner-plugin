import os
import path
from rafcon.core.storage import storage
from rafcon.core.singleton import library_manager
from rafcon.core.state_machine import StateMachine
from rafcon.core.config import global_config
from rafcon.core.states.hierarchy_state import HierarchyState
from rafcon.utils import log
from de.dlr.rmc.rafcontpp.model.datastore import Datastore

logger = log.get_logger(__name__)



class Mapper():

    __datastore = None

    def __init__(self,datastore):

        if datastore is None:
            logger.error("Datastore in Mapper can not be None!")
            raise ValueError("Datastore in Mapper can not be None!")

        self.__datastore = datastore



    def generate_action_state_map(self):
        state_libs = self.__datastore.get_state_pools()
        libraries = {}
        lib_names=[]
        for pool in state_libs:
            lib_name = os.path.basename(os.path.dirname(pool))
            lib_names.append(lib_name)
            libraries[lib_name] = os.path.abspath(pool)

        global_config.load()
        global_config.set_config_value("LIBRARY_PATHS", libraries)
        library_manager.initialize()
        action_state_map={}
        for lib_name in lib_names:
            state_pool = library_manager.libraries[lib_name]
            for state in state_pool:
                lib_state = library_manager.get_library_instance(lib_name, state)
                sem_data = lib_state.state_copy.semantic_data

                if 'PDDL_Action' in sem_data:
                    action_name = sem_data['PDDL_Action']
                    if action_name in action_state_map:
                        logger.warning("Multiple association of action "+ str(action_name)
                                       + " associated with states " + str(action_state_map[action_name])
                                       + " and " + str(state))
                    else:
                        action_state_map[action_name.upper()] = state
                else:
                    logger.warning("State "+ state + "is not associated with any PDDL Action!" )

        if not action_state_map:
            logger.warning("No States with semantic PDDL_Action data found!")
        self.__datastore.set_action_state_map(action_state_map)



    def generate_state_action_map(self):

        if self.__datastore.get_action_state_map() is None:
            self.generate_action_state_map()
        action_state_map = self.__datastore.get_action_state_map()

        state_action_map = {}

        for action in action_state_map.keys():
            c_state = action_state_map[action]

            if c_state in state_action_map:
                logger.warning("Multiple associations of state " + str(c_state)
                               + " associated with actions: " + str(state_action-map[c_state])
                               + " and " +str(c_state))
            else:
                state_action_map[c_state] = action

            if not state_action_map:
                logger.warning("could not generate state_action_map, action_state_map was empty!")
        self.__datastore.set_state_action_map(state_action_map)



    def generate_available_actions(self):
        if self.__datastore.get_action_state_map() is None:
            self.generate_action_state_map()
        action_state_map = self.__datastore.get_action_state_map()

        self.__datastore.set_available_actions(action_state_map.keys())

