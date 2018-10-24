import inspect
import DynamicImporter
from de.dlr.rmc.rafcontpp.model.datastore import DataStore
from de.dlr.rmc.rafcontpp.model.planning_report import PlanningReport
from rafcon.utils import log

logger = log.get_logger(__name__)

class PlanningController:

    def __init__(self,datastore):
        self.__datastore = datastore


    def execute_planning(self):

        planning_successfull = False
        planner_choice = self.__datastore.get_planner()
        to_import = self.__get_built_in(planner_choice)
        if to_import is None:
            to_import = self.__discover_class(planner_choice)

        if to_import is None:
            raise ImportError("Couldn't Import "+planner_choice)

        module = __import__(to_import[0])
        PlannerModule = getattr(module,to_import[1])

        planner = PlannerModule()
        planning_report = planner.plan_scenario(self.__datastore.get_domain_path(),
                                                self.__datastore.get_facts_path(),
                                                self.__datastore.get_planner_argv(),
                                                self.__datastore.get_file_save_dir())

        if planning_report.planning_successful():
            self.__datastore.set_plan(planning_report.get_plan())
            planning_successfull = True
            logger.info("Planning Successfull! Plan has length: "+str(len(planning_report.get_plan())))
        else:
            logger.error("Planning failed! ::"+planning_report.get_error_message())

        return planning_successfull








    def __discover_class(self, script):
        class_name = None
        for cname, some_obj in inspect.getmembers(script):
            if inspect.isclass(some_obj) & cname != 'PlannerInterface':
                for me_name, class_obj in inspect.getmembers(some_obj):
                    if inspect.ismethod(class_obj) & me_name == 'plan_scenario':
                        class_name = cname
                        break;
        return (script, class_name)



    def __get_built_in_script(self, shortcut):
        build_in_planner = self.__datastore.get_built_in_planners()

        planner = None
        if shortcut in built_in_planner:
            planner = built_in_planner[shortcut]

        return planner