# Copyright (C) 2014-2018 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Christoph Suerig <christoph.suerig@dlr.de>



import os
from rafcon.core.storage import storage
from rafcon.core.singleton import library_manager
from rafcon.core.state_machine import StateMachine
from rafcon.core.config import global_config
from rafcon.core.states.hierarchy_state import HierarchyState
from rafcon.utils import log
from de.dlr.rmc.rafcontpp.model.plan_step import PlanStep
from de.dlr.rmc.rafcontpp.model.datastore import Datastore

logger = log.get_logger(__name__)

#TODO: executing this, removes all other libraries from rafcon, thats bad, fix this problem, also it updates
#the libraries for several times.

class StateMachineGenerator:

    def __init__(self,datastore):
        self.__datastore = datastore

    def generate_state_machine(self):
        logger.info('Creating Statemachine...')
        sm_name = self.__datastore.get_domain_name()+'_statemachine'
        sm_path = os.path.abspath(os.path.join(self.__datastore.get_sm_save_dir(), sm_name))
        a_s_map = self.__datastore.get_action_state_map()
        root_state = HierarchyState(sm_name)
        last_state = None
        for action in self.__datastore.get_plan():
            if action.name in a_s_map:
                #load and prepare state
                current_state = self.__load_state(a_s_map[action.name])
                #fill dataport default value with params of action
                if current_state.input_data_ports:
                    for index, param in enumerate(action.parameter):
                        if index in current_state.input_data_ports:
                            current_state.input_data_ports[index].default_value = param

                #add state to state machine
                root_state.add_state(current_state)
                if last_state is None:
                    root_state.set_start_state(current_state.state_id)
                else:
                    root_state.add_transition(last_state.state_id, 0, current_state.state_id, None)
                last_state = current_state
            else:
                logger.error("No State found for action: \"" + action.name + "\"")
                raise LookupError("No State found for action: \"" + action.name + "\"")
        root_state.add_transition(last_state.state_id, 0, root_state.state_id, 0)
        # everything connected, create statemachine object and save.
        state_machine = StateMachine(root_state=root_state)
        storage.save_state_machine_to_path(state_machine, sm_path)
        logger.info("State machine " + sm_name + " created. It contains " + str(
            len(self.__datastore.get_plan())) + " states.")


    def __load_state(self, wanted_state):

        state_libs = self.__datastore.get_state_pools()
        libraries = {}
        lib_names = []
        return_state = None
        for pool in state_libs:
            lib_name = os.path.basename(os.path.dirname(pool))
            lib_names.append(lib_name)
            libraries[lib_name] = os.path.abspath(pool)

        global_config.load()
        global_config.set_config_value("LIBRARY_PATHS", libraries)
        library_manager.initialize()
        for lib_name in lib_names:
            state_pool = library_manager.libraries[lib_name]
            if wanted_state in state_pool:
                return_state = library_manager.get_library_instance(lib_name,wanted_state)
                break

        return return_state


