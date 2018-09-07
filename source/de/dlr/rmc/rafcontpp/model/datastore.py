import os
import path
from rafcon.source.utils.singelton import Singelton
from rafcon.utils import log

logger = log.get_logger(__name__)


class Datastore(Singelton):

    __state_pools = None
    __action_pools = None
    __file_save_dir = None
    __sm_save_dir = None
    __domain_path = None
    __facts_path = None
    __type_db_path = None
    __planner_argv = None
    __action_state_map = None
    __state_action_map = None
    __available_actions = None
    __plan = None
    __keep_related_files = False



    def __init(self, state_pools, action_pools,sm_save_dir,
               facts_path,type_db_path,keep_related_files, file_save_dir=os.path.join(os.getcwd(),'related_files')):
        #validate state_pools
        for dir in state_pools:
            if not os.path.isdir(dir):
                logger.error("state pool directory not found: " + str(dir))
                raise ValueError('Is not a directory: '+ str(dir))
        #validate action_pools
        for dir in action_pools:
            if not os.path.isdir(dir):
                logger.error("action pool directory not found: " + str(dir))
                raise ValueError('Is not a directory: '+ str(dir))
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
        self.__state_pools = state_pools
        self.__action_pools = action_pools
        self.__sm_save_dir = sm_save_dir
        self.__facts_path = facts_path
        self.__type_db_path = type_db_path
        self.__keep_related_files = keep_related_files
        self.__file_save_dir = file_save_dir
        self.__planner_argv = []

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

    def get_facts_path(self):
        return self.__facts_path

    def get_type_db_path(self):
        return self.__type_db_path

    def get_planner_argv(self):
        return self.__planner_argv

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
        self.__plan = plan

    def keep_related_files(self):
        return self.__keep_related_files






    class Logger(object):
        __metaclass__ = Singleton
