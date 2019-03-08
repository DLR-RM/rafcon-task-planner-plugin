# Copyright (C) 2014-2018 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Christoph Suerig <christoph.suerig@dlr.de>
# Version 08.03.1019



import os
import time
from rafcon.core.storage import storage
from rafcon.core.singleton import library_manager
from rafcon.core.singleton import state_machine_manager
from rafcon.core.state_machine import StateMachine
from rafcon.core.states.hierarchy_state import HierarchyState
from rafcon.utils import log

logger = log.get_logger(__name__)


class StateMachineGenerator:
    '''StateMachineGenerator
    The StateMachineGenerator takes the plan from the datastore and molds a state machine out of it.

    '''

    def __init__(self, datastore):
            self.__datastore = datastore

    def generate_state_machine(self):
        ''' generate_state_machine
        generate_state_machine generates a state machine, fills the data ports and opens the state machine in rafcon.
        :param self:
        :return: nothing
        '''
        sm_name = self.__datastore.get_sm_name()
        sm_name = self.__datastore.get_problem_name()+'_state_machine' if len(sm_name) == 0 else sm_name
        sm_path = os.path.abspath(os.path.join(self.__datastore.get_sm_save_dir(), sm_name))
        a_s_map = self.__datastore.get_action_state_map()
        pddl_action_dict = self.__datastore.get_pddl_action_map()
        logger.info('Creating State machine \"'+sm_name+'\"...')
        start_time = time.time()
        root_state = HierarchyState(sm_name)
        last_state = None
        for plan_step in self.__datastore.get_plan():
            #the name of a plan step is an action name.
            if plan_step.name in a_s_map:
                #load and prepare state
                current_state = self.__load_state(a_s_map[plan_step.name])
                if current_state.input_data_port_runtime_values:
                   c_pddl_action = pddl_action_dict[plan_step.name]
                   c_input_data_ports = current_state.input_data_ports
                   for key in c_input_data_ports.keys():
                       #c_pddl_action.parameters contains parameter names
                       if c_input_data_ports[key].name in c_pddl_action.parameters:
                           index = c_pddl_action.parameters.index(c_input_data_ports[key].name)
                           #plan_step.parameter contains parameter values
                           current_state.input_data_port_runtime_values[key] = plan_step.parameter[index]
                       else:
                           logger.warn("Action "+c_pddl_action.name+" has no Parameter "
                                       +c_input_data_ports[key].name+", which is needed in State "+current_state.name)
                #add state to state machine
                root_state.add_state(current_state)
                if last_state is None:
                    root_state.set_start_state(current_state.state_id)
                else:
                    root_state.add_transition(last_state.state_id, 0, current_state.state_id, None)
                last_state = current_state
            else:
                logger.error("No State found for action: \"" + plan_step.name + "\"")
                raise LookupError("No State found for action: \"" + plan_step.name + "\"")
        root_state.add_transition(last_state.state_id, 0, root_state.state_id, 0)
        # everything connected, create statemachine object and save.
        state_machine = StateMachine(root_state=root_state)
        storage.save_state_machine_to_path(state_machine, sm_path)
        library_manager.refresh_libraries()
        logger.info("State machine \"" + sm_name + "\" created. It contains " + str(
            len(self.__datastore.get_plan())) + " states, generation took {0:.4f} seconds.".format(time.time()- start_time))
        #open state machine
        self.__open_state_machine(state_machine,sm_path)



    def __open_state_machine(self,state_machine, state_machine_path):

        logger.debug('Opening state machine...')
        if state_machine_manager.is_state_machine_open(state_machine.file_system_path):
            old_sm = state_machine_manager.get_open_state_machine_of_file_system_path(state_machine.file_system_path)
            state_machine_manager.remove_state_machine(old_sm.state_machine_id)
        new_state_machine = storage.load_state_machine_from_path(state_machine_path)
        state_machine_manager.add_state_machine(new_state_machine)



    def __load_state(self, wanted_state):
        '''load_state
        load_state gets a state and loads it from the libraries.
        :param self:
        :param wanted_state: a state that should be loaded
        :return: the loaded state
        '''
        state_libs = self.__datastore.get_state_pools()
        libraries = {}
        lib_names = []
        return_state = None
        for pool in state_libs:
            lib_name = os.path.basename(os.path.abspath(pool))
            lib_names.append(lib_name)
            libraries[lib_name] = os.path.abspath(pool)

        for lib_name in lib_names:
            state_pool = library_manager.libraries[lib_name]
            if wanted_state in state_pool:
                return_state = library_manager.get_library_instance(lib_name, wanted_state)
                break

        return return_state



