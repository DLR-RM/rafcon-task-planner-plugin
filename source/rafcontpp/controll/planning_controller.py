import inspect
import os
import sys
from rafcon.utils import log

logger = log.get_logger(__name__)

class PlanningController:

    def __init__(self,datastore):
        self.__datastore = datastore


    def execute_planning(self):

        planning_successful = False
        planner_choice = self.__datastore.get_planner()
        to_import = self.__get_built_in_script(planner_choice)
        if to_import is None:
            planner_choice = self.__split_and_add_to_path(planner_choice)
            to_import = self.__discover_class(planner_choice)

        if to_import is None:
            raise ImportError("Couldn't Import "+planner_choice)

        script_import = __import__(to_import[0],fromlist=(to_import[1]))
        PlannerModule = getattr(script_import,to_import[1])
        logger.info('Using Planner script: '+str(to_import[0]))
        planner = PlannerModule()
        logger.info("Planning...")
        planning_report = planner.plan_scenario(self.__datastore.get_domain_path(),
                                                self.__datastore.get_facts_path(),
                                                self.__datastore.get_planner_argv(),
                                                self.__datastore.get_file_save_dir())
        logger.info("finished planning.")
        if planning_report.planning_successful():
            self.__datastore.set_plan(planning_report.get_plan())
            planning_successful = True
            if len(planning_report.get_plan()) >0:
                logger.info("Planning Successful! Plan has length: "+str(len(planning_report.get_plan())))
            else:
                logger.info("Planning Successful, but no Plan was found! ")
        else:
            logger.error("Planning failed! :: "+planning_report.get_error_message())

        self.__datastore.add_generated_file(planning_report.get_generated_files())

        return planning_successful






    def __split_and_add_to_path(self,script_path):
        path = os.path.dirname(script_path)
        script_name = os.path.basename(script_path)
        #remove file extension
        if '.' in script_name:
            script_name = script_name.split('.')[0]
        #add path to PYTHONPATH
        sys.path.append(path)
        return script_name

    def __discover_class(self, script):
        class_name = None
        script_import = __import__(script)
        for cname, some_obj in inspect.getmembers(script_import):
            if inspect.isclass(some_obj) and cname != 'PlannerInterface':
                for me_name, class_obj in inspect.getmembers(some_obj):
                    if inspect.ismethod(class_obj) and me_name == 'plan_scenario':
                        class_name = cname
                        break;
        return (script, class_name)



    def __get_built_in_script(self, shortcut):
        built_in_planner = self.__datastore.get_built_in_planners()

        planner = None
        if shortcut in built_in_planner.keys():
            planner = built_in_planner[shortcut]

        return planner