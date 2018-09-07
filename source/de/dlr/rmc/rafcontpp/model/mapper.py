import datastore
import os
import path
from rafcon.core.storage import storage
from rafcon.core.singleton import library_manager
from rafcon.core.state_machine import StateMachine
from rafcon.core.config import global_config
from rafcon.core.states.hierarchy_state import HierarchyState
from rafcon.utils import log

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
                sem_data = lib_state.state_copy_semantic_data

                if 'PDDL_Action' in sem_data:
                    action_name = sem_data['PDDL_Action']
                    if action_name in action_state_map:
                        logger.warning("Multiple association of action "+ str(action_name)
                                       + " associated with states " + str(action-state_map[action_name])
                                       + " and " + str(state))
                    else:
                        action_state_map[action_name] = state
                else:
                    logger.warning("State "+ state + "is not associated with any PDDL Action!" )
        if not action_state_map:
            logger.warning("No States with semantic PDDL_Action data found!")

        self.__datastore.set_action_state_map(action_state_map)



    def generate_state_action_map(self):


    def generate_available_actions(self):