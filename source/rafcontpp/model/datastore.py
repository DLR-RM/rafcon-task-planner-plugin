import os
import json
from rafcontpp.model.plan_step import PlanStep
from rafcon.utils import log

logger = log.get_logger(__name__)
# a map containing all built in planners e.g. planners with integration script.
built_in_planners = {
    'Fast Downward Planning System': ('rafcontpp.planner.fdintegration', 'FdIntegration')
}
#the storage path of the config file.
DATASTORE_STORAGE_PATH = os.path.join(os.path.expanduser('~'), os.path.normpath('.config/rafcon/rafcontpp_conf.json'))

def datastore_from_file(file_path):
    ''' datastore_from_file
    datastore_from_file tries to create a partial datastore (just input values, no clculated ones)
    from a .json file e.g. config file. if there is no config file present, it returns a datastore with default values
    :param file_path: the path to the config file
    :return: a partial initialized datastore, or a datastore with default values.
    '''
    ds = None
    if not os.path.isfile(file_path):
        logger.warning("Can't restore configuration from: " + str(file_path))
        logger.info("Creating default configuration...")
        default_dir = str(os.path.expanduser('~'))
        ds = Datastore([default_dir], [default_dir], default_dir, built_in_planners.keys()[0], [], default_dir,
                  default_dir, False)
        ds.set_planner_script_path(default_dir)

    else:
        data = json.load(open(file_path, "r"))
        logger.info('Loading Configuration form: '+file_path)
        ds = Datastore(data['state_pools'],
                     data['action_pools'],
                     data['sm_save_dir'],
                     data['planner'],
                     data['planner_argv'],
                     data['facts_path'],
                     data['type_db_path'],
                     data['keep_related_files'],
                     data['file_save_dir'])
        ds.set_planner_script_path(data['planner_script_path'])
        ds.validate_ds()
        logger.info("Red configuration successfully!")
    return ds


class Datastore:
    ''' Datastore
    Datastore is a datastore, which holds all data of the plugin. Every module can get, and store its data here.
    '''



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
    #the path of a custom planner script. this variable is not really useful for the plugin, and its not used, BUT:
    #its useful or usability, to be able to save the script path persistent.
    __planner_script_path = None


    def __init__(self, state_pools, action_pools,sm_save_dir, planner, planner_argv,
               facts_path,type_db_path,keep_related_files, file_save_dir=os.path.join(os.getcwd(), 'related_files')):

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




    def validate_ds(self): #TODO validate everything!

        # validate state_pools
        for dir in self.__state_pools:
            if not os.path.isdir(dir):
                logger.error("state pool directory not found: " + str(dir))
                raise ValueError('Is not a directory: ' + str(dir))
        # validate action_pools
        for dir in self.__action_pools:
            if not os.path.isfile(dir):
                logger.error("action pool file not found: " + str(dir))
                raise ValueError('Is not a file: ' + str(dir))
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



    def get_state_pools(self):
        return self.__state_pools

    def add_state_pools(self,state_pools,set_pool):
        '''

        :param state_pools: the state pools to add
        :param set_pool: if true, state pools are not added, but set, and old list gets lost.
        :return: nothing
        '''
        if not state_pools:
            logger.error("state_pools can't be None")
            raise ValueError("state_pools can't be None")
        if set_pool:
            self.__state_pools = []
        if state_pools and isinstance(state_pools,str):
            if state_pools not in self.__state_pools:
                self.__state_pools.append(state_pools)
        elif state_pools:
            for state_pool in state_pools:
                    self.add_state_pools(state_pool,False)

    def get_action_pools(self):
        return self.__action_pools

    def add_action_pools(self,action_pools):
        if not action_pools:
            logger.error("action_pools can't be None")
            raise ValueError("action_pools can't be None")
        if action_pools and isinstance(action_pools, str):
            if action_pools not in self.__action_pools:
                self.__action_pools.append(action_pools)
        elif action_pools:
            for action_pool in action_pools:
                self.add_action_pools(action_pool)


    def get_file_save_dir(self):
        return self.__file_save_dir

    def set_file_save_dir(self,file_save_dir):
        if self.__keep_related_files and not os.path.isdir(file_save_dir):
            logger.error('file_save_dir must be a directory')
            raise ValueError('Is not a directory: '+str(file_save_dir))
        self.__file_save_dir = file_save_dir

    def get_sm_save_dir(self):
        return self.__sm_save_dir

    def set_sm_save_dir(self, sm_save_dir):
        if not os.path.isdir(sm_save_dir):
            logger.error('state machine save directory must be a directory!')
            raise ValueError('Is not a direcotry: '+str(sm_save_dir))
        self.__sm_save_dir = sm_save_dir

    def get_domain_path(self):
            return self.__domain_path

    def set_domain_path(self, domain_path):
        if not os.path.isfile(domain_path):
            logger.error("No domain file: " + str(domain_path))
            raise ValueError('Is not a file: ' + str(domain_path))
        self.__domain_path = domain_path

    def get_domain_name(self):
        return str(self.__domain_name)

    def set_domain_name(self,domain_name):
        self.__domain_name = domain_name

    def get_facts_path(self):
        return self.__facts_path

    def set_facts_path(self, facts_path):
        if not os.path.isfile(facts_path):
            logger.error('No facts file: '+str(facts_path))
            raise ValueError('Is not a File: '+str(facts_path))
        self.__facts_path = facts_path

    def get_type_db_path(self):
        return self.__type_db_path

    def set_type_db_path(self,type_db):
        if not os.path.isfile(type_db):
            logger.error('No type db found at: '+ str(type_db))
            raise ValueError('Is not a File: '+str(type_db))
        self.__type_db_path = type_db

    def get_planner_argv(self):
        return self.__planner_argv

    def set_planner_argv(self,planner_argv):
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

    def set_keep_related_files(self,keep_related_files):
        self.__keep_related_files = keep_related_files

    def get_built_in_planners(self):
        return built_in_planners

    def add_generated_file(self,file_name):
        if file_name and isinstance(file_name,str):
            self.__generated_files.append(file_name)
        elif file_name:
            self.__generated_files.extend(file_name)


    def get_generated_files(self):
        return self.__generated_files

    def get_planner_script_path(self):
        return self.__planner_script_path

    def set_planner_script_path(self,psp):
        self.__planner_script_path = psp




    def save_datastore_parts_in_file(self, file_path):
        ''' save_datastore_parts_in_file
        save_datastore_parts_in_file saves all plugin inputs, which are present in the datastore in a file.
        :param self:
        :param file_path: the path of the configuration file
        :return: nothing
        '''
        data_to_save = {
            'state_pools': self.__state_pools,
            'action_pools': self.__action_pools,
            'type_db_path': self.__type_db_path,
            'planner' : self.__planner,
            'planner_script_path': self.__planner_script_path,
            'planner_argv': self.__planner_argv,
            'facts_path': self.__facts_path,
            'sm_save_dir': self.__sm_save_dir,
            'keep_related_files': self.__keep_related_files,
            'file_save_dir': self.__file_save_dir
        }
        logger.info('Writing Configuration to path: '+file_path)
        conf_file = open(file_path, "w")
        conf_file.write(json.dumps(data_to_save))
        conf_file.flush()
        conf_file.close()



