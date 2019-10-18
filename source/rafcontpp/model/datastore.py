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
# Version 09.09.2019


import copy
import getpass
import json
import os
import threading
import time

from rafcon.utils import log

from rafcontpp.model.pddl_action_representation import PddlActionRepresentation
from rafcontpp.model.plan_step import PlanStep
from rafcontpp.model.type_tree import TypeTree

logger = log.get_logger(__name__)
# a map containing all built in planners e.g. planners with integration script.
built_in_planners = {
    'Fast Downward Planning System': ('rafcontpp.planner.fast_downward_integration', 'FdIntegration'),
    'Fast-Forward Planning System v2.3': ('rafcontpp.planner.fast_forward_integration', 'FfIntegration')
}
# The version string of the Plug-in.
PLUGIN_VERSION = "v1.5"
# The storage path of the config file.
DATASTORE_STORAGE_PATH = os.path.join(os.path.expanduser('~'), os.path.normpath('.config/rafcon/rafcontpp_conf.json'))
#the temp file save dir, to save files in if files should not be kept.
TMP_FILE_SAVE_DIR = os.path.join('/tmp','rtpp-{}'.format(getpass.getuser()))
# The name of the semantic data dict in rafcon state
SEMANTIC_DATA_DICT_NAME = 'RAFCONTPP'
# The name of the sub dictionary, where the pddl action is stored in.
PDDL_ACTION_SUB_DICT_NAME = 'PDDL_ACTION'
# The key to allow the Override of the State Content
ALLOW_OVERRIDE_NAME = 'Allow_Override'
# A lock to synchronize planning thread map accesses.
planning_threads_lock = threading.Lock()
# tuples of all registered (currently running) planning threads format: (thread,problem name)
planning_threads = {}


def get_planning_threads():
    """
    :return: {long: InterruptableThread}: A copy of the planning_threads dict.
    """
    with planning_threads_lock:
        threads_copy = copy.copy(planning_threads)
    return threads_copy


def datastore_from_file(file_path):
    """ datastore_from_file
    datastore_from_file tries to create a partial datastore (just input values, no clculated ones)
    from a .json file e.g. config file. if there is no config file present, it returns a datastore with default values

    :param file_path: the path to the config file
    :return: Datastore: A partial initialized datastore, or a datastore with default values.
    """
    ds = None
    if not os.path.isfile(file_path):
        logger.warning("Can't restore configuration from: {}".format(file_path))
        logger.info("Creating default configuration...")
        default_dir = str(os.path.expanduser('~'))
        ds = Datastore([default_dir], '',
                       default_dir, built_in_planners.keys()[0], default_dir, [], default_dir, default_dir, False)
    else:
        data = json.load(open(file_path, "r"))
        logger.debug('Loading Configuration form: ' + file_path)
        sm_name = data['sm_name'] if 'sm_name' in data else ''  # To provide backward compatibility.
        ds = Datastore(data['state_pools'],
                       sm_name,
                       data['sm_save_dir'],
                       data['planner'],
                       data['planner_script_path'],
                       data['planner_argv'],
                       data['facts_path'],
                       data['type_db_path'],
                       data['keep_related_files'],
                       data['file_save_dir'])
        runtime_data_path = data[
            'runtime_data_path'] if 'runtime_data_path' in data else ''  # To provide backward compatibility.
        runtime_as_ref = data[
            'runtime_as_ref'] if 'runtime_as_ref' in data else False  # To provide backward compatibility.
        plan_into_state = data[
            'plan_into_state'] if 'plan_into_state' in data else False  # To provide backward compatibility.
        ds.set_runtime_data_path(runtime_data_path)
        ds.set_use_runtime_path_as_ref(runtime_as_ref)
        ds.set_generate_into_state(plan_into_state)
        logger.info("Read configuration successfully!")
    return ds


class Datastore:
    """ Datastore
    Datastore is a datastore, which holds all data of the plugin. Every module can get, and store its data here.
    """

    def __init__(self, state_pools, sm_name, sm_save_dir, planner, planner_script_path, planner_argv,
                 facts_path, type_db_path, keep_related_files, file_save_dir='/tmp'):
        """
         Constructor of Datastore

        :param state_pools: a list of file paths.
        :param sm_name: the name of the state machine which will be generated.
        :param sm_save_dir: the directory, where to save the generated state machine.
        :param planner: the name / script path of the used planner.
        :param planner_argv: a String array, with arguments for the planner.
        :param facts_path: path of the facts file.
        :param type_db_path: path of the type_db.
        :param keep_related_files: true, if generated files e.g. the domain file or the plan should be saved.
        :param file_save_dir: a path, where to save all related files.
        """
        # a list of directories, containing states with pddl notation.
        self.__state_pools = state_pools
        # the name of the state machine, which will be generated.
        self.__sm_name = sm_name
        # the location, to save the generated state machine in.
        self.__sm_save_dir = sm_save_dir
        # the complete path of the facts file (e.g. /home/facts.pddl).
        self.__facts_path = facts_path
        # the complete path of the type db file
        self.__type_db_path = type_db_path
        # True if files should be keeped, false otherwhise
        self.__keep_related_files = keep_related_files
        # the directory, where to save all produced files in
        self.__file_save_dir = file_save_dir
        # the shortcut, or the name of the planner script
        self.__planner = planner
        # additional arguments for the planner as string array.
        self.__planner_argv = planner_argv
        # the path of a custom planner script. this variable is not really useful for the plugin, and its not used, BUT:
        # its useful for usability, to be able to save the script path persistent.
        self.__planner_script_path = planner_script_path
        # the complete path of the domain file (e.g. /home/domain.pddl).
        self.__domain_path = None
        # a map containing pddl action names as keys, and rafcon states as values.
        self.__action_state_map = None
        # a map containing rafcon states as keys and pddl action names as values.
        self.__state_action_map = None
        # a map containing action names as keys and pddl action representations as values
        self.__pddl_action_map = None
        # a list, contining the names of all available actions.
        self.__available_actions = None
        # a PddlFactsRepresentation Object, containing the parsed facts file.
        self.__pddl_facts_representation = None
        # a typeTree containing all available types
        self.__available_types = None
        # a list of (String,[(String,integer)]) prediactes
        self.__available_predicates = None
        # the plan, as a list of plan steps.
        self.__plan = None
        # a list with the name of all files, generated during the pipeline execution.
        self.__generated_files = []
        # the complete path of the runtime data dict, which holds data required during the run of the generated sm.
        self.__runtime_data_path = None
        # if true the runtime_data is red during runtime, otherwhise its red when generating the sm.
        self.__use_runtime_data_path_as_reference = False
        # target state, the state to plan into.
        self.__target_state = None
        # true, if the state machine should be generated into the target state. false if independent sm should be used.
        self.__generate_into_state = False

    def validate_ds(self):  # TODO validate everything!
        """
        validate_ds runs some checks on the state_pools, the sm-save_dir, the facts_path, the type_db_path,
        and the file_save_dir. if a check fails, it raises a ValueError.

        :return: void
        """
        # validate state_pools
        for dir in self.__state_pools:
            if not os.path.isdir(dir):
                logger.error("state pool directory not found: " + str(dir))
                raise ValueError('Is not a directory: ' + str(dir))
        # validate sm_save_dir
        if not os.path.isdir(self.__sm_save_dir):
            logger.error("state machine save dir: directory not found! " + str(self.__sm_save_dir))
            raise ValueError('Is not a directory: ' + str(self.__sm_save_dir))
        # validate facts_file
        if not os.path.isfile(self.__facts_path):
            logger.error("No facts file : " + str(self.__facts_path))
            raise ValueError('Is not a file: ' + str(self.__facts_path))
        # validate type_db_path
        if not os.path.isfile(self.__type_db_path):
            logger.error("No type database : " + str(self.__type_db_path))
            raise ValueError('Is not a file: ' + str(self.__type_db_path))
        # validate file_save_dir
        if self.__keep_related_files and (not os.path.isdir(self.__file_save_dir)):
            logger.error("file save dir is not a directory: " + str(self.__file_save_dir))
            raise ValueError('Is not a directory: ' + str(self.__file_save_dir))

    def register_thread(self, interruptable_thread):
        """
        gets a thread, addes it synchronized to a global map, and returns the map key (which is the register time.).

        :param interruptable_thread: A InterruptableThread, the datastore should store.
        :return: long: The key used to register the thread. (That's the register time as unixtimestamp.)
        """
        with planning_threads_lock:
            register_time = time.time()  # unix timestamp
            # set task name to sm name, or problem name, if no sm name is available, and to state name if the sm is planned
            # into a state.
            task_name = self.get_pddl_facts_representation().problem_name if len(
                self.get_sm_name()) == 0 else self.get_sm_name()
            if self.__target_state:
                task_name = self.__target_state.name
            planning_threads[register_time] = (interruptable_thread, task_name, self.get_planner())
        return register_time

    def remove_thread(self, key):
        """
        Receives a timestamp as key, and removes the thread synchronized from the global map.

        :param key: The time, the thread was registered
        :return: Boolean: True, if removing was successful, false otherwise
        """
        successful = False
        with planning_threads_lock:
            if key in planning_threads.keys():
                del planning_threads[key]
                successful = not (key in planning_threads.keys())
        return successful

    def get_state_pools(self):
        return self.__state_pools

    def add_state_pools(self, state_pools, set_pool):
        """
        :param state_pools: the state pools to add
        :param set_pool: if true, state pools are not added, but set, and old list gets lost.
        """
        if not state_pools:
            logger.error("state_pools can't be None")
            raise ValueError("state_pools can't be None")
        if set_pool:
            self.__state_pools = []
        if state_pools and isinstance(state_pools, str):
            if state_pools not in self.__state_pools:
                self.__state_pools.append(state_pools)
        elif state_pools:
            for state_pool in state_pools:
                self.add_state_pools(state_pool, False)

    def get_file_save_dir(self):
        return self.__file_save_dir

    def set_file_save_dir(self, file_save_dir):
        if self.__keep_related_files and not os.path.isdir(file_save_dir):
            logger.error('file_save_dir must be a directory')
            raise ValueError('Is not a directory: ' + str(file_save_dir))
        self.__file_save_dir = file_save_dir

    def get_sm_name(self):
        return self.__sm_name

    def set_sm_name(self, name):
        self.__sm_name = name

    def get_sm_save_dir(self):
        return self.__sm_save_dir

    def set_sm_save_dir(self, sm_save_dir):
        if not os.path.isdir(sm_save_dir):
            logger.error('state machine save directory must be a directory!')
            raise ValueError('Is not a direcotry: ' + str(sm_save_dir))
        self.__sm_save_dir = sm_save_dir

    def get_domain_path(self):
        return self.__domain_path

    def set_domain_path(self, domain_path):
        if not os.path.isfile(domain_path):
            logger.error("No domain file: " + str(domain_path))
            raise ValueError('Is not a file: ' + str(domain_path))
        self.__domain_path = domain_path

    def get_facts_path(self):
        return self.__facts_path

    def set_facts_path(self, facts_path):
        if not os.path.isfile(facts_path):
            logger.error('No facts file: ' + str(facts_path))
            raise ValueError('Is not a File: ' + str(facts_path))
        self.__facts_path = facts_path

    def get_type_db_path(self):
        return self.__type_db_path

    def set_type_db_path(self, type_db):
        if not os.path.isfile(type_db):
            logger.error('No type db found at: ' + str(type_db))
            raise ValueError('Is not a File: ' + str(type_db))
        self.__type_db_path = type_db

    def get_planner_argv(self):
        return self.__planner_argv

    def set_planner_argv(self, planner_argv):
        if planner_argv is None:
            logger.error("can't set None value as planner_argv")
            raise ValueError("can't set None value as planner_argv")
        self.__planner_argv = planner_argv

    def get_planner(self):
        return self.__planner

    def set_planner(self, planner):
        self.__planner = planner

    def get_action_state_map(self):
        return self.__action_state_map

    def set_action_state_map(self, action_state_map):
        if action_state_map is None:
            logger.error("can't set None value as action_state_map")
            raise ValueError("can't set None value as action_state_map")
        self.__action_state_map = action_state_map

    def get_state_action_map(self):
        return self.__state_action_map

    def set_state_action_map(self, state_action_map):
        if state_action_map is None:
            logger.error("can't set None value as state_action_map.")
            raise ValueError("can't set None value as state_action_map.")
        self.__state_action_map = state_action_map

    def get_available_actions(self):
        return self.__available_actions

    def set_available_actions(self, available_actions):
        if available_actions is None:
            logger.error("Can't set None value as available_actions.")
            raise ValueError("Can't set None value as available_actions.")
        self.__available_actions = available_actions

    def set_pddl_facts_representation(self, facts_representation):
        if facts_representation is None:
            logger.error("Can't set None value as pddl_facts_representation.")
            raise ValueError("Can't set None value as pddl_facts_representation.")
        self.__pddl_facts_representation = facts_representation

    def get_runtime_data_path(self):
        return self.__runtime_data_path

    def set_runtime_data_path(self, runtime_data_path):
        if runtime_data_path and len(runtime_data_path) > 0 and not os.path.isfile(runtime_data_path):
            logger.warning('Runtime Data Path is not a Path: {}'.format(runtime_data_path))
        self.__runtime_data_path = runtime_data_path

    def use_runtime_path_as_ref(self):
        return self.__use_runtime_data_path_as_reference

    def set_use_runtime_path_as_ref(self, use):
        self.__use_runtime_data_path_as_reference = use

    def get_pddl_facts_representation(self):
        return self.__pddl_facts_representation

    def get_plan(self):
        return self.__plan

    def set_plan(self, plan):
        if plan is None:
            logger.error("can't set None value as plan.")
            raise ValueError("can't set None value as plan.")
        if len(plan) > 0 and (not isinstance(plan[0], PlanStep)):
            logger.error("plan hast to be of type [PlanStep].")
            raise TypeError("plan hast to be of type [PlanStep].")
        self.__plan = plan

    def keep_related_files(self):
        return self.__keep_related_files

    def set_keep_related_files(self, keep_related_files):
        self.__keep_related_files = keep_related_files

    def get_built_in_planners(self):
        return built_in_planners

    def add_generated_file(self, file_name):
        if file_name and isinstance(file_name, str):
            self.__generated_files.append(file_name)
        elif file_name:
            self.__generated_files.extend(file_name)

    def get_generated_files(self):
        return self.__generated_files

    def get_planner_script_path(self):
        return self.__planner_script_path

    def set_planner_script_path(self, psp):
        self.__planner_script_path = psp

    def get_pddl_action_map(self):
        return self.__pddl_action_map

    def set_pddl_action_map(self, action_map):
        if action_map and isinstance(action_map.values()[0], PddlActionRepresentation):
            self.__pddl_action_map = action_map
        else:
            logger.error("pddl action map has to be of type Dict{str:PddlActionRepresentation}")
            raise TypeError("pddl action map has to be of type Dict{str:PddlActionRepresentation}")

    def get_available_types(self):
        return self.__available_types

    def set_available_types(self, availabe_types):
        assert isinstance(availabe_types, TypeTree)
        self.__available_types = availabe_types

    def get_available_predicates(self):
        return self.__available_predicates

    def set_available_predicates(self, available_predicates):
        self.__available_predicates = available_predicates

    def get_target_state(self):
        return self.__target_state

    def set_target_state(self, target_state):
        self.__target_state = target_state

    def generate_into_state(self):
        return self.__generate_into_state

    def set_generate_into_state(self, plan_into_state):
        self.__generate_into_state = plan_into_state

    def save_datastore_parts_in_file(self, file_path):
        """ save_datastore_parts_in_file
        save_datastore_parts_in_file saves all plugin inputs, which are present in the datastore in a file.

        :param file_path: The path of the configuration file
        :return: void
        """
        data_to_save = {
            'state_pools': self.__state_pools,
            'type_db_path': self.__type_db_path,
            'planner': self.__planner,
            'planner_script_path': self.__planner_script_path,
            'planner_argv': self.__planner_argv,
            'facts_path': self.__facts_path,
            'plan_into_state': self.__generate_into_state,
            'sm_name': self.__sm_name,
            'sm_save_dir': self.__sm_save_dir,
            'keep_related_files': self.__keep_related_files,
            'file_save_dir': self.__file_save_dir,
            'runtime_data_path': self.__runtime_data_path,
            'runtime_as_ref': self.__use_runtime_data_path_as_reference
        }
        logger.debug('Writing Configuration to path: ' + file_path)
        conf_file = open(file_path, "w")
        conf_file.write(json.dumps(data_to_save, indent=2, sort_keys=True, separators=(',', ': ')))
        conf_file.flush()
        conf_file.close()
        logger.info('Saved Configuration successfully!')
