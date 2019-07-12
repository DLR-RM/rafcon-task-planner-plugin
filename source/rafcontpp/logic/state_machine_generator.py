# Copyright (C) 2014-2018 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Christoph Suerig <christoph.suerig@dlr.de>
# Version 12.07.1019



import os
import time
import json
from rafcontpp.logic.state_machine_layouter import StateMachineLayouter
from rafcontpp.model.datastore import ALLOW_OVERRIDE_NAME, SEMANTIC_DATA_DICT_NAME
from rafcontpp.model import interruptable_thread
from rafcon.gui.models.signals import ActionSignalMsg
from rafcon.gui.singleton import state_machine_manager_model
from rafcon.gui.utils import wait_for_gui
from rafcon.gui.config import global_gui_config
from rafcon.core.storage import storage
from rafcon.core.singleton import library_manager
from rafcon.core.singleton import state_machine_manager
from rafcon.core.state_machine import StateMachine
from rafcon.core.states.hierarchy_state import HierarchyState
from rafcon.core.states.execution_state import ExecutionState
from rafcon.utils.gui_functions import call_gui_callback
from rafcon.utils import log
logger = log.get_logger(__name__)


class StateMachineGenerator:
    '''StateMachineGenerator
    The StateMachineGenerator takes the plan from the datastore and molds a state machine out of it.

    '''

    def __init__(self, datastore):
            self.__datastore = datastore
            self.__gui_involved = False

    def generate_state_machine(self):
        ''' generate_state_machine
        generate_state_machine generates a state machine, fills the data ports and opens the state machine in rafcon.

        '''
        sm_name = self.__datastore.get_sm_name()
        sm_name = self.__datastore.get_pddl_facts_representation.problem_name+'_state_machine' if len(sm_name) == 0 else sm_name
        sm_path = os.path.abspath(os.path.join(self.__datastore.get_sm_save_dir(), sm_name))
        a_s_map = self.__datastore.get_action_state_map()
        pddl_action_dict = self.__datastore.get_pddl_action_map()
        start_time = time.time()
        state_order_list = []
        #!IMPORTANT: root state is not necessarily the root state of the sm, but the state we generate our sm into.
        state_machine, root_state, is_independent_sm = self.__validate_and_get_root_state_and_state_machine(
                                                                    self.__datastore.get_target_state(),sm_name,sm_path)
        self.__gui_involved = not is_independent_sm
        sm_name = sm_name if is_independent_sm else root_state.name # set sm name to root state name if planning into a selected machine.
        logger.info('Creating State machine \"' + sm_name + '\"...')

        if self.__gui_involved:#emit signal, to stop gui drawing in order to speed up the process.
            call_gui_callback(global_gui_config.set_config_value,'HISTORY_ENABLED', False)
            state_machine_m = state_machine_manager_model.state_machines[state_machine.state_machine_id]
            root_state_m = state_machine_m.get_state_model_by_path(root_state.get_path())
            root_state_m.action_signal.emit(ActionSignalMsg(action='substitute_state', origin='model',
                                                            action_parent_m=root_state_m,
                                                            affected_models=[root_state_m], after=False))
        last_state = None
        # add global data init state and set start state
        runtime_data_path = self.__datastore.get_runtime_data_path()
        if runtime_data_path and len(runtime_data_path)>0:
            last_state = self.__get_runtime_data_init_state(runtime_data_path, self.__datastore.use_runtime_path_as_ref())
            self.__gui_wrapper(root_state.add_state, last_state)
            self.__gui_wrapper(root_state.set_start_state,last_state.state_id)
            state_order_list.append(last_state.state_id)

        facts = self.__datastore.get_pddl_facts_representation()
        current_thread = interruptable_thread.current_thread()
        for plan_step in self.__datastore.get_plan():
            #the name of a plan step is an action name.
            if current_thread and current_thread.is_interrupted():
                break
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
                           current_state.input_data_port_runtime_values[key] = \
                                                            facts.get_original_object_name(plan_step.parameter[index])
                       else:
                           logger.warn("Action "+c_pddl_action.name+" has no Parameter "
                                       +c_input_data_ports[key].name+", which is needed in State "+current_state.name)
                #add state to state machine
                self.__gui_wrapper(root_state.add_state, current_state)
                
                state_order_list.append(current_state.state_id)
                if last_state is None:
                    self.__gui_wrapper(root_state.set_start_statecurrent_state.state_id)
                else:
                    self.__gui_wrapper(root_state.add_transition,last_state.state_id, 0, current_state.state_id, None)
                    
                last_state = current_state
            else:
                logger.error("No State found for action: \"" + plan_step.name + "\"")
                raise LookupError("No State found for action: \"" + plan_step.name + "\"")

        if current_thread and not current_thread.is_interrupted():
            #at the end add transition from last state to outcome of root state.
            self.__gui_wrapper(root_state.add_transition,last_state.state_id, 0, root_state.state_id, 0)
            library_manager.refresh_libraries()
            # everything connected, save statemachine object.
            if state_machine.file_system_path:
                storage.save_state_machine_to_path(state_machine, state_machine.file_system_path)
            logger.info("State machine \"" + sm_name + "\" created.")
            logger.info(sm_name+" contains " + str(len(root_state.states)) + " states.")
            logger.info("State machine generation took {0:.4f} seconds.".format(time.time()- start_time))
            #format state machine
            layouter = StateMachineLayouter()
            fixed_size = root_state.id() != state_machine.root_state.id()
            self.__gui_wrapper(layouter.layout_state_machine, state_machine, root_state,fixed_size, state_order_list)
            #open state machine
            if is_independent_sm:
                self.__open_state_machine(state_machine, state_machine.file_system_path)
            else:
                root_state_m.action_signal.emit(ActionSignalMsg(action='substitute_state', origin='model',
                                                                action_parent_m=root_state_m,
                                                                affected_models=[root_state_m], after=True))
        elif self.__gui_involved:
            logger.info("")
            root_state_m.action_signal.emit(ActionSignalMsg(action='substitute_state', origin='model',
                                                                action_parent_m=root_state_m,
                                                                affected_models=[root_state_m], after=True))
            call_gui_callback(global_gui_config.set_config_value,'HISTORY_ENABLED', True)



    def __gui_wrapper(self, function, *args):
        '''
        if the state we plan into is already opened in the gui, all functions to do so have to be executed in a
        call_gui_callback. this warpper will execute the function normally if gui is not involved, else wrapped in
        a call_gui_callback.
        :param function: a gui sensitive function that should be executed
        :param args: the args of the function
        '''

        if self.__gui_involved:
            call_gui_callback(function,*args)
        else:
            function(*args)


    def __open_state_machine(self,state_machine, state_machine_path):
        '''
        gets a state machine and opens it in rafcon. If an old version is still open, it closes it first.
        :param state_machine: the name of the state machine
        :param state_machine_path: the path of the state machine.
        :return:
        '''

        logger.info('Opening state machine...')
        if state_machine_manager.is_state_machine_open(state_machine.file_system_path):
            old_sm = state_machine_manager.get_open_state_machine_of_file_system_path(state_machine.file_system_path)
            call_gui_callback(state_machine_manager.remove_state_machine, old_sm.state_machine_id)
        new_state_machine = storage.load_state_machine_from_path(state_machine_path)
        call_gui_callback(state_machine_manager.add_state_machine,new_state_machine)




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

    def __validate_and_get_root_state_and_state_machine(self, root_state, sm_name, sm_path):
        '''
        validates a given root state. e.g. it looks if its a hierarchy state, and if its empty.
        if its not, it will clear the given state, if it has the permission to do so. If None is provided,
        it will create a new root state and a new state machine.
        :param root_state: A state to use as root state, or None if there is no root state yet.
        :param sm_name: the name of the state machine
        :param sm_path: the path of the state machine
        :return: (State_machine, valid_Root_state)
        '''
        valid_root_state = None
        state_machine = None
        is_independent = False
        if root_state == None:
            # set root-state id to old root-state id, in case the state machine is replanned.
            # why is it important? - if you added the planned sm as a library, replan it and refresh it,
            # rafcon will throw an error, if the refreshed library has a different root-state id.
            if os.path.isdir(sm_path):
                # rootstate id of rootstate in old sm.
                old_sm_rs_id = storage.load_state_machine_from_path(sm_path).root_state.state_id
                valid_root_state = HierarchyState(name=sm_name, state_id=old_sm_rs_id)
            else:
                valid_root_state = HierarchyState(sm_name)
            state_machine = StateMachine(root_state=valid_root_state)
            storage.save_state_machine_to_path(state_machine, sm_path)
            is_independent = True

        elif isinstance(root_state,HierarchyState):
            if len(root_state.states) == 0 \
                    or root_state.semantic_data[SEMANTIC_DATA_DICT_NAME][ALLOW_OVERRIDE_NAME] == 'True':
                #empty the root state!
                self.__clear_state(root_state)
                valid_root_state = root_state
                state_machine = root_state.get_state_machine()

            else:
                logger.error("Can't plan into None empty Hierarchy State without permission!")
                logger.info("Creating independent State machine...")
                state_machine, valid_root_state, is_independent = self.__validate_and_get_root_state_and_state_machine(
                                                                                                None, sm_name, sm_path)


        else:
            logger.error("Can't Plan into State {}, can only plan into Hierarchystate!".format(root_state))
            logger.info("Creating independent State machine...")
            state_machine, valid_root_state, is_independent = self.__validate_and_get_root_state_and_state_machine(
                                                                                                None, sm_name, sm_path)

        return (state_machine, valid_root_state, is_independent)


    def __clear_state(self, root_state):
        '''
        gets a state and removes all its child states with respect to the gui.
        :param root_state: the state to clear
        '''
        state_machine = root_state.get_state_machine()
        state_machine_m = state_machine_manager_model.state_machines[state_machine.state_machine_id]
        state_m = state_machine_m.get_state_model_by_path(root_state.get_path())
        state_m.action_signal.emit(ActionSignalMsg(action='substitute_state', origin='model',
                                                        action_parent_m=state_m,
                                                        affected_models=[state_m], after=False))
        for state in root_state.states.values():
           call_gui_callback(root_state.remove_state, state.state_id, True, True, True)

        state_m.action_signal.emit(ActionSignalMsg(action='substitute_state', origin='model',
                                                        action_parent_m=state_m,
                                                        affected_models=[state_m], after=True))

    def __get_runtime_data_init_state(self, data_init_file_path, use_as_ref):
        '''
        creates an execution state, containing the, or a reference to the runtime data, as well as the code needed
        to write it into a dictionary in the global variables.
        :param data_init_file_path: The path of a file containing a json dict
        :param use_as_ref: True if the path should be included as reference, False if the dictionary itself should be included.
        :return: an Execution state, that will update the rtpp_data dict in the global variables.
        '''

        data_init_state = ExecutionState(name='Runtime Data Initialization (rtpp_data)')
        data_to_load = None
        if use_as_ref:
            data_to_load = 'json.load(open("{}", "r"))'.format(data_init_file_path)
            data_init_state.script_text = 'import json{}'.format(data_init_state.script_text)
        else:
            data_to_load = json.dumps(json.load(open(data_init_file_path, "r")), indent=2, separators=(',', ': '))
        execute_str = "self.logger.info('Updating rtpp_data.')\r\n"
        execute_str = "{}    rtpp_data = gvm.get_variable('rtpp_data')\r\n".format(execute_str)
        execute_str = "{}    rtpp_data = rtpp_data if rtpp_data else {}\r\n".format(execute_str,{})
        execute_str = "{}    rtpp_data.update({})\r\n".format(execute_str, data_to_load)
        execute_str = '{}    gvm.set_variable(\'{}\',{})'.format(execute_str,'rtpp_data',' rtpp_data')

        data_init_state.script_text = data_init_state.script_text.replace('self.logger.debug("Hello world")',execute_str)
        return data_init_state


