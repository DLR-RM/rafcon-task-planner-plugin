import os
import path
from de.dlr.rmc.rafcontpp.model.plan_step import PlanStep
from rafcon.utils import log

logger = log.get_logger(__name__)


class Datastore:

    #the complete path of the domain file (e.g. /home/domain.pddl).
    __domain_path = None
    #the name of the domain (e.g. BlocksWorld).
    __domain_name = None
    #a map containing pddl actions as keys, and rafcon states as values.
    __action_state_map = None
    #a map containing rafcon states as keys and pddl actions as values.
    __state_action_map = None
    #a list, contining the names of all available actions.
    __available_actions = None
    #the plan, as a list of plan steps.
    __plan = None
    #a list with the name of all files, generated during the pipeline execution.
    __generated_files = []
    #a dict containing the shortcuts of the build in planner scripts.
    __built_in_planners = {
        'Fast Downward Planning System': ('de.dlr.rmc.rafcontpp.planner.fdintegration', 'FdIntegration')
    }

    def __init__(self, state_pools, action_pools,sm_save_dir, planner, planner_argv,
               facts_path,type_db_path,keep_related_files, file_save_dir=os.path.join(os.getcwd(),'related_files')):
        #validate state_pools
        for dir in state_pools:
            if not os.path.isdir(dir):
                logger.error("state pool directory not found: " + str(dir))
                raise ValueError('Is not a directory: '+ str(dir))
        #validate action_pools
        for dir in action_pools:
            if not os.path.isfile(dir):
                logger.error("action pool file not found: " + str(dir))
                raise ValueError('Is not a file: '+ str(dir))
        #validate sm_save_dir
        if not os.path.isdir(sm_save_dir):
            logger.error("state machine save dir: directory not found! " + str(sm_save_dir))
            raise ValueError('Is not a directory: ' + str(sm_save_dir))
        #validate facts_file
        if not os.path.isfile(facts_path):
            logger.error("No facts file : " + str(facts_path))
            raise ValueError('Is not a file: ' + str(facts_path))
        #validate type_db_path
        if not os.path.isfile(type_db_path):
            logger.error("No type database : " + str(type_db_path))
            raise ValueError('Is not a file: ' + str(type_db_path))
        #validate file_save_dir
        if keep_related_files and (not os.path.isdir(file_save_dir)):
            logger.error("file save dir is not a directory: " + str(file_save_dir))
            raise ValueError('Is not a directory: ' + str(file_save_dir))
        #a list of directories, containing states with pddl notation.
        self.__state_pools = state_pools
        #a list of action_db files
        self.__action_pools = action_pools
        #the location, to save the generated state machine in.
        self.__sm_save_dir = sm_save_dir
        #the complete path of the facts file (e.g. /home/facts.pddl).
        self.__facts_path = facts_path
        #the complete path of the type db file
        self.__type_db_path = type_db_path
        # True if files should be keeped, false otherwhise
        self.__keep_related_files = keep_related_files
        # the directory, where to save all produced files in
        self.__file_save_dir = file_save_dir
        # the shortcut, or the name of the planner script
        self.__planner = planner
        #additional arguments for the planner.
        self.__planner_argv = planner_argv










    def get_state_pools(self):
        return self.__state_pools

    def get_action_pools(self):
        return self.__action_pools

    def get_file_save_dir(self):
        return self.__file_save_dir

    def get_sm_save_dir(self):
        return self.__sm_save_dir

    def get_domain_path(self):
            return self.__domain_path

    def set_domain_path(self, domain_path):
        if not os.path.isfile(domain_path):
            logger.error("No domain file: " + str(domain_path))
            raise ValueError('Is not a file: ' + str(domain_path))
        self.__domain_path = domain_path

    def set_domain_name(self,domain_name):
        self.__domain_name = domain_name

    def get_domain_name(self):
        return str(self.__domain_name)

    def get_facts_path(self):
        return self.__facts_path

    def get_type_db_path(self):
        return self.__type_db_path

    def get_planner_argv(self):
        return self.__planner_argv

    def get_planner(self):
        return self.__planner

    def set_planner_argv(self,planner_argv):
        if planner_argv is None:
            logger.error("can't set None value as planner_argv")
            raise ValueError("can't set None value as planner_argv")
        self.__planner_argv = planner_argv

    def get_action_state_map(self):
        return self.__action_state_map

    def set_action_state_map(self,action_state_map):
        if action_state_map is None:
            logger.error("can't set None value as action_state_map")
            raise ValueError("can't set None value as action_state_map")
        self.__action_state_map = action_state_map

    def get_state_action_map(self):
        return self.__state_action_map

    def set_state_action_map(self,state_action_map):
        if state_action_map is None:
            logger.error("can't set None value as state_action_map")
            raise ValueError("can't set None value as state_action_map")
        self.__state_action_map = state_action_map

    def get_available_actions(self):
        return self.__available_actions

    def set_available_actions(self,available_actions):
        if available_actions is None:
            logger.error("can't set None value as available_actions")
            raise ValueError("can't set None value as available_actions")
        self.__available_actions = available_actions

    def get_plan(self):
        return self.__plan

    def set_plan(self,plan):
        if plan is None:
            logger.error("can't set None value as plan")
            raise ValueError("can't set None value as plan")
        if len(plan) > 0 and (not isinstance(plan[0],PlanStep)):
            logger.error("plan hast to be of type [PlanStep]")
            raise TypeError("plan hast to be of type [PlanStep]")
        self.__plan = plan

    def keep_related_files(self):
        return self.__keep_related_files

    def get_built_in_planners(self):
        return self.__built_in_planners

    def add_generated_file(self,file_name):
        if file_name and isinstance(file_name,str):
            self.__generated_files.append(file_name)
        elif file_name:
            self.__generated_files.extend(file_name)


    def get_generated_files(self):
        return self.__generated_files



