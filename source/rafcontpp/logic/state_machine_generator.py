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
# Version 21.08.1019


import copy
import json
import os
import time

from rafcon.core.singleton import library_manager
from rafcon.core.singleton import state_machine_manager
from rafcon.core.state_machine import StateMachine
from rafcon.core.states.execution_state import ExecutionState
from rafcon.core.states.hierarchy_state import HierarchyState
from rafcon.core.storage import storage
from rafcon.gui.helpers.state import substitute_state_as
from rafcon.gui.models.signals import ActionSignalMsg
from rafcon.gui.singleton import state_machine_manager_model
from rafcon.gui.utils import wait_for_gui
from rafcon.utils import log
from rafcon.utils.gui_functions import call_gui_callback
from rafcontpp.logic.state_machine_layouter import StateMachineLayouter
from rafcontpp.model import interruptable_thread
from rafcontpp.model.datastore import ALLOW_OVERRIDE_NAME, SEMANTIC_DATA_DICT_NAME

logger = log.get_logger(__name__)


class StateMachineGenerator:
    """StateMachineGenerator
    The StateMachineGenerator takes the plan from the datastore and molds a state machine out of it.
    """

    def __init__(self, datastore):
        """

        :param datastore: A datastore containing: the target state, the state machine save directory, the action state map,
        the pddl action map, the facts representation, the runtime data path, the plan, the state pools, and the state
        machine name.
        """
        self.__datastore = datastore

    def generate_state_machine(self):
        """
        generate_state_machine generates a state machine, fills the data ports and opens the state machine in rafcon,
        if it's a new one.

        :return: void
        """
        target_state = self.__datastore.get_target_state()
        generate_independent = target_state == None
        if target_state:
            successful, target_state = self.__generate_state_machine_into_state(target_state)
            generate_independent = not successful
        if generate_independent:
            indi_state_machine = self.__generate_independent_state_machine(target_state)
            self.__open_state_machine(indi_state_machine, indi_state_machine.file_system_path)
            logger.info('Generated and opened State machine: "{}".'.format(self.__get_actual_sm_name()))

    def __generate_independent_state_machine(self, target_state=None):
        """
        generate_independent_state_machine generates a new state machine. It optinally receives a target_state,
        which is then treated as root state for the new state machine. If the target state is not valid, a new root
        state is used.

        :param target_state: A potential target state, optional.
        :return: StateMachine: The generated state machine.
        """
        logger.info('Generating independent State machine "{}"...'.format(self.__get_actual_sm_name()))
        start_time = time.time()
        sm_name = self.__get_actual_sm_name()
        sm_path = os.path.abspath(os.path.join(self.__datastore.get_sm_save_dir(), sm_name))
        target_state = target_state if self.__is_valid_target_state(target_state)[0] else HierarchyState(sm_name)
        state_order_list = []
        # set root-state id to old root-state id, in case the state machine is replanned.
        # why is it important? - if you added the planned sm as a library, replan it and refresh it,
        # rafcon will throw an error, if the refreshed library has a different root-state id.
        if os.path.isdir(sm_path):
            # rootstate id of rootstate in old sm.
            old_sm_rs_id = storage.load_state_machine_from_path(sm_path).root_state.state_id
            target_state = HierarchyState(name=sm_name, state_id=old_sm_rs_id)
        target_state, state_order_list = self.__generate_core_machine(target_state)
        state_machine = StateMachine(root_state=target_state)
        storage.save_state_machine_to_path(state_machine, sm_path)
        logger.info('State machine "{}" created.'.format(state_machine.root_state.name))
        logger.info(sm_name + " contains " + str(len(target_state.states)) + " states.")
        logger.info("State machine generation took {0:.4f} seconds.".format(time.time() - start_time))
        sm_layouter = StateMachineLayouter()
        sm_layouter.layout_state_machine(state_machine, target_state, False, state_order_list)
        return state_machine

    def __generate_state_machine_into_state(self, target_state):
        """
        generate_state_machine_into_state receives a target state, copies that state, generates a state machine into
        the copy, and then substitutes the original state with the copy.

        :param target_state: The target state to generate the state machine into a copy of it.
        :return: (Boolean, HierarchyState): True if successful, False otherwise. May return a state which should
        be used as root state for a new independent state machine, or None.
        """
        logger.info('Generating State machine into "{}"...'.format(target_state.name))
        start_time = time.time()
        valid, error = self.__is_valid_target_state(target_state)
        state_to_generate_into = None
        successful = False
        if self.__is_valid_target_state(target_state):
            state_machine = target_state.get_state_machine()
            state_machine_m = state_machine_manager_model.state_machines[state_machine.state_machine_id]
            target_state_m = state_machine_m.get_state_model_by_path(target_state.get_path())
            target_state_copy = copy.deepcopy(target_state)
            if self.__clear_state(target_state_copy):
                if not target_state.is_root_state:
                    target_state_copy, state_order_list = self.__generate_core_machine(target_state_copy)
                    call_gui_callback(substitute_state_as, target_state_m, target_state_copy, False)
                    wait_for_gui()
                    if state_machine.file_system_path:
                        storage.save_state_machine_to_path(state_machine, state_machine.file_system_path)
                    logger.info('State machine generated into state "{}" of state machine "{}".'
                                .format(target_state_copy.name, state_machine.root_state.name))
                    logger.info(
                        '"{}" contains {} states.'.format(target_state_copy.name, len(target_state_copy.states)))
                    logger.info("State machine generation took {0:.4f} seconds.".format(time.time() - start_time))
                    sm_layouter = StateMachineLayouter()
                    target_state_m_copy = state_machine_m.get_state_model_by_path(target_state_copy.get_path())
                    target_state_m_copy.action_signal.emit(ActionSignalMsg(action='substitute_state', origin='model',
                                                                           action_parent_m=target_state_m_copy,
                                                                           affected_models=[target_state_m_copy],
                                                                           after=False))
                    call_gui_callback(sm_layouter.layout_state_machine, state_machine,
                                      target_state_copy, True, state_order_list)
                    wait_for_gui()
                    target_state_m_copy.action_signal.emit(ActionSignalMsg(action='substitute_state', origin='model',
                                                                           action_parent_m=target_state_m_copy,
                                                                           affected_models=[target_state_m_copy],
                                                                           after=True))
                    successful = True
                    logger.info("Generated and integrated State machine: {}.".format(target_state_copy.name))
                else:
                    logger.error("Target state was root state!")
                    state_to_generate_into = target_state_copy
                    self.__datastore.set_sm_name(str(state_to_generate_into.name))
            else:
                logger.error('Could\'nt clear target state.')
        else:
            logger.error(error)
        return successful, state_to_generate_into

    def __clear_state(self, state):
        """
        Clear_state receives a hierarchy state, and removes all child states.

        :param state: A hierarchy state
        :return: Boolean: True if clearing was successful, False otherwhise.
        """
        for child_state in state.states.values():
            call_gui_callback(state.remove_state, child_state.state_id, True, True, True)
        wait_for_gui()
        return len(state.states) == 0

    def __is_valid_target_state(self, target_state):
        """
        is_valid_target_state receives a potential target state, and checks if it is valid.
        e.g. if its a hierarchy state, if its None, if permission is granted.

        :param target_state: A state to validate
        :return: (Boolean, String): True if state is valid, and an error message, in case it's not.
        """
        is_valid = True
        error_message = 'OK'
        if target_state is None:
            is_valid = False
            error_message = 'Can\'t plan into None.'
        elif not isinstance(target_state, HierarchyState):
            is_valid = False
            error_message = 'Can\'t Plan into State {}, can only plan into HierarchyStates!'.format(target_state)
        elif len(target_state.states) > 0:
            permission_granted = \
                str(target_state.semantic_data[SEMANTIC_DATA_DICT_NAME][ALLOW_OVERRIDE_NAME]).lower() == 'true'
            if not permission_granted:
                is_valid = False
                error_message = "Can't plan into None empty HierarchyState without permission!"
        return is_valid, error_message

    def __generate_core_machine(self, target_state):
        """
        Takes a root state, and generates the state machine into it.

        :param target_state: The root state
        :return:(HierarchyState,[State]): The root state containing the state machine, and the state order list, is a list of all states in the sm in right order. Can return (None,[]) if process was interrupted.
        """
        a_s_map = self.__datastore.get_action_state_map()
        pddl_action_dict = self.__datastore.get_pddl_action_map()
        facts = self.__datastore.get_pddl_facts_representation()
        current_thread = interruptable_thread.current_thread()
        state_order_list = []
        last_state = None
        # add global data init state and set start state
        runtime_data_path = self.__datastore.get_runtime_data_path()
        if runtime_data_path and len(runtime_data_path) > 0:
            last_state = self.__get_runtime_data_init_state(runtime_data_path,
                                                            self.__datastore.use_runtime_path_as_ref())
            target_state.add_state(last_state)
            target_state.set_start_state(last_state.state_id)
            state_order_list.append(last_state.state_id)
        for plan_step in self.__datastore.get_plan():
            # the name of a plan step is an action name.
            if current_thread and current_thread.is_interrupted():
                break
            if plan_step.name in a_s_map:
                # load and prepare state
                current_state = self.__load_state(a_s_map[plan_step.name])
                if current_state.input_data_port_runtime_values:
                    c_pddl_action = pddl_action_dict[plan_step.name]
                    c_input_data_ports = current_state.input_data_ports
                    for key in c_input_data_ports.keys():
                        # c_pddl_action.parameters contains parameter names
                        if c_input_data_ports[key].name in c_pddl_action.parameters:
                            index = c_pddl_action.parameters.index(c_input_data_ports[key].name)
                            # plan_step.parameter contains parameter values
                            current_state.input_data_port_runtime_values[key] = \
                                facts.get_original_object_name(plan_step.parameter[index])
                        else:
                            logger.warn("Action " + c_pddl_action.name + " has no Parameter "
                                        + c_input_data_ports[
                                            key].name + ", which is needed in State " + current_state.name)
                # add state to state machine
                target_state.add_state(current_state)
                # add the state to the order list (for later formatting)
                state_order_list.append(current_state.state_id)
                # add transitions.
                if last_state is None:
                    target_state.set_start_state(current_state.state_id)
                else:
                    target_state.add_transition(last_state.state_id, 0, current_state.state_id, None)

                last_state = current_state
            else:
                logger.error("No State found for action: \"" + plan_step.name + "\"")
                raise LookupError("No State found for action: \"" + plan_step.name + "\"")
        # at the end add transition from last state to outcome of root state.
        target_state.add_transition(last_state.state_id, 0, target_state.state_id, 0)
        library_manager.refresh_libraries()
        if not current_thread or current_thread.is_interrupted():
            target_state = None
            state_order_list = []
        return target_state, state_order_list

    def __load_state(self, wanted_state):
        """
        load_state gets a state and loads it from the libraries.

        :param wanted_state: A state that should be loaded.
        :return: LibraryState: The loaded state
        """
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

    def __open_state_machine(self, state_machine, state_machine_path):
        """
        Reveives a state machine and opens it in rafcon. If an old version is still open, it closes it first.

        :param state_machine: The name of the state machine
        :param state_machine_path: The path of the state machine.
        :return: void
        """
        logger.info('Opening state machine...')
        if state_machine_manager.is_state_machine_open(state_machine.file_system_path):
            old_sm = state_machine_manager.get_open_state_machine_of_file_system_path(state_machine.file_system_path)
            call_gui_callback(state_machine_manager.remove_state_machine, old_sm.state_machine_id)
        new_state_machine = storage.load_state_machine_from_path(state_machine_path)
        call_gui_callback(state_machine_manager.add_state_machine, new_state_machine)

    def __get_runtime_data_init_state(self, data_init_file_path, use_as_ref):
        """
        Creates an execution state, containing the, or a reference to the runtime data, as well as the code needed
        to write it into a dictionary in the global variables.

        :param data_init_file_path: The path of a file containing a json dict.
        :param use_as_ref: True if the path should be included as reference, False if the dictionary itself should be included.
        :return: ExecutionState: An Execution state, that will update the rtpp_data dict in the global variables.
        """
        data_init_state = ExecutionState(name='Runtime Data Initialization (rtpp_data)')
        data_to_load = None
        execute_str = ""
        if use_as_ref:
            data_to_load = 'json.load(open("{}", "r"))'.format(data_init_file_path)
            execute_str = 'import json\r\n'
        else:
            data_to_load = json.dumps(json.load(open(data_init_file_path, "r")), indent=2, separators=(',', ': '))
        #only append stuff to the execute_str here!
        execute_str = "{}\r\ndef execute(self, inputs, outputs, gvm):\r\n".format(execute_str)
        execute_str = "{}    self.logger.info('Updating rtpp_data.')\r\n".format(execute_str)
        execute_str = "{}    rtpp_data = gvm.get_variable('rtpp_data')\r\n".format(execute_str)
        execute_str = "{}    rtpp_data = rtpp_data if rtpp_data else {}\r\n".format(execute_str, {})
        execute_str = "{}    rtpp_data.update({})\r\n".format(execute_str, data_to_load)
        execute_str = '{}    gvm.set_variable(\'{}\',{})\r\n'.format(execute_str, 'rtpp_data', ' rtpp_data')
        execute_str = "{}    return \"success\"".format(execute_str)
        data_init_state.script_text = execute_str
        return data_init_state

    def __get_actual_sm_name(self):
        """
        Looks if a state machine name is present in the datastore, if no name is present it uses the problem name followed by _state_machine.

        :return: String: The actual state machine name.
        """
        sm_name = self.__datastore.get_sm_name()
        sm_name = self.__datastore.get_pddl_facts_representation().problem_name + '_state_machine' if len(
            sm_name) == 0 else sm_name
        return sm_name
