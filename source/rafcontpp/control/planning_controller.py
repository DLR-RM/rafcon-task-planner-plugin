# Contributors:
# Christoph Suerig <christoph.suerig@dlr.de>
# Version 25.04.2019
import inspect
import time
import os
import signal
import sys
from multiprocessing import Process, Queue, current_process
from rafcontpp.model.interruptable_thread import InterruptableThread
from rafcon.utils import log

logger = log.get_logger(__name__)

class PlanningController:
    '''PlanningController
       PlanningController handles everything about the topic planning. it loads built-in scripts as well, als
       importing custom planner integrations. It also starts the planning process, and feeds the datastore with
       the plan, given by the planner.

    '''

    def __init__(self,datastore):
        '''

        :param datastore:  a datastore, containing all necessary data.
        '''
        self.__datastore = datastore


    def execute_planning(self, callback_func):
        '''execute_planning
        execute_planning loads built-in scripts, imports custom scripts, and executes them.
        it starts the planner in a new thread, and calls the callback_function when finish.
        :param callback_func is callback_func(Boolean):void planning will be executed async, and call this function when finish.
        :return: the planning thread
        '''

        planning_successful = False
        planner_choice = self.__datastore.get_planner()
        if not planner_choice or len(planner_choice) == 0:
            logger.error('Can\'t Import None as Planner')
            raise ImportError('Can\'t Import None as Planner')
        logger.debug('Try to resolve given planner string.')
        to_import = self.__get_built_in_script(planner_choice)
        if to_import is None:
            logger.debug('Planner string was no built-in planner')
            planner_choice = self.__split_and_add_to_path(planner_choice)
            try:
                to_import = self.__discover_class(planner_choice)
            except ImportError:
                to_import = None

        if to_import is None:
            logger.error("Couldn't discover planner "+planner_choice)
            raise ImportError("Couldn't discover planner "+planner_choice)

        logger.debug('Try to Import planner')
        script_import = __import__(to_import[0],fromlist=(to_import[1]))
        PlannerModule = getattr(script_import,to_import[1])
        logger.info('Using Planner script: '+str(to_import[0]))
        planner = PlannerModule()

        planning_thread = InterruptableThread(target=self.__plan_and_report,
                                              args=(callback_func, planner),
                                              name='PlanningThread')
        planning_thread.setDaemon(True)
        planning_thread.start()

        return planning_thread


    def __plan_and_report(self,current_thread,callback_function, planner):
        '''
        plan and report triggers the planner, and evaluates the planning report
        e.g. storing the plan in the datastore. Due planning could take a long time
        this method should be called async.
        :param current_thread: the Interruptable Thread, this function runs in. (automatically added by the thread.)
        :param callback_function: a call back function called after planning
        :param planner: the planner to plan with
        :return: nothing
        '''
        #---------------------------------------------------------------------------------------------------------------
        def execute_in_sub_process(queue):
            '''
            This mehod executes the actual planning.
            should be executed in a sub process.(this decision was made, to be able to kill the planning process)
            :param queue: a message queue, which will contain the planning report
            '''
            os.setsid()#give a new process group id, to be able to kill the whole group resulting from this sub process and the planner.
            planning_report = planner.plan_scenario(self.__datastore.get_domain_path(),
                                                    self.__datastore.get_facts_path(),
                                                    self.__datastore.get_planner_argv(),
                                                    self.__datastore.get_file_save_dir())

            queue.put(planning_report)

        #---------------------------------------------------------------------------------------------------------------
        logger.debug("planner argv: " + str(self.__datastore.get_planner_argv()))
        start_time = time.time()
        planning_report = None
        queue = Queue()#for interprocess communication, to get the planning_report
        planning_process = Process(target=execute_in_sub_process, args=[queue], name='PlanningSubprocess')
        planning_process.daemon = True
        planning_process.start()
        logger.info("Planning...")
        #wait for the planning thread to terminate, check if thread was interrupted
        while not current_thread.is_interrupted() and planning_process.is_alive():
            planning_process.join(2)

        #NOT interrupted path
        if not current_thread.is_interrupted():
            logger.info("Finished planning after {0:.4f} seconds.".format(time.time() - start_time))
            if not queue.empty():
                planning_report = queue.get_nowait()
                if planning_report.planning_successful():
                    self.__datastore.set_plan(planning_report.get_plan())
                    if len(planning_report.get_plan()) > 0:
                        logger.info("Planning Successful! Plan has length: " + str(len(planning_report.get_plan())))
                    else:
                        logger.info("Planning Successful, but no Plan was found!")
                else:
                    logger.error("Planning failed! Planner reported:: " + planning_report.get_error_message())
                if planning_report.get_generated_files():
                    self.__datastore.add_generated_file(planning_report.get_generated_files())
                callback_function(planning_report.planning_successful())
            else:
                logger.error('Planner provided no Planning Report!')
                callback_function(False)
        #Interrupted path
        else:
            planning_process_pgid = os.getpgid(planning_process.pid)
            #terminate planning_process with all spawned sub processes
            os.killpg(planning_process_pgid, signal.SIGTERM)
            times_waited = 1
            max_wait = 3
            while planning_process and planning_process.is_alive():
                logger.info('Waiting for the Planner to terminate({}/{})...'.format(times_waited,max_wait))
                if times_waited == max_wait:
                    logger.info('Killing Planner...')
                    os.killpg(planning_process_pgid, signal.SIGKILL)#somethimes terminating is not enough.
                planning_process.join(2)
                times_waited +=1


            logger.info("Planning was canceled after {0:.4f} seconds.".format(time.time() - start_time))
            callback_function(False)





    def __split_and_add_to_path(self,script_path):
        '''
            splits the script path into the directory path, and the script name, adds the directory Path to
            PYTHONPATH and returns the scripname

        :param script_path: the path of a custom planner script like /home/planner_script.py
        :return: the Name of the script e.g. planner_script
        '''
        path = os.path.dirname(script_path)
        script_name = os.path.basename(script_path)
        #remove file extension
        if '.' in script_name:
            script_name = script_name.split('.')[0]
        #add path to PYTHONPATH only if needed.
        if path not in sys.path:
            sys.path.append(path)
            logger.debug(sys.path)
        return script_name

    def __discover_class(self, script):
        ''' discover_class
        discover_class receives a script like "planner_script", it imports it and discovers the class, which
        implements the plan_scenario method.
        :param script: the name of a custom planner integration module
        :return: a (script_name, class_name) tuple
        '''

        class_name = None
        script_import = __import__(script)
        for cname, some_obj in inspect.getmembers(script_import):
            if inspect.isclass(some_obj) and cname != 'PlannerInterface':
                for me_name, class_obj in inspect.getmembers(some_obj):
                    if inspect.ismethod(class_obj) and me_name == 'plan_scenario':
                        class_name = cname
                        break
        return (script, class_name)



    def __get_built_in_script(self, shortcut):
        ''' get_built_in_script
        get_built_in_script trys to resolve a given shortcut e.g. the name of a built in planner in the list
        :param shortcut: some string, hopefully a shortcut
        :return: (script_name,script_path) tuple, or None if the shortcut was not found in the list.
        '''
        built_in_planner = self.__datastore.get_built_in_planners()

        planner = None
        if shortcut in built_in_planner.keys():
            planner = built_in_planner[shortcut]

        return planner