# Contributors:
# Christoph Suerig <christoph.suerig@dlr.de>
# Version 08.03.2019

import os
import time
import traceback
from rafcontpp.logic.mapper import Mapper
from rafcontpp.logic.domain_generator import DomainGenerator
from rafcontpp.logic.state_machine_generator import StateMachineGenerator
from rafcontpp.logic.pddl_action_loader import PddlActionLoader
from rafcontpp.control.planning_controller import PlanningController
from rafcon.utils import log

logger = log.get_logger(__name__)


class ExecutionController:
    """ExecutionController
       ExecutionController controlles the execution of the planning pipeline, from mapping up to generating the state
       machine
    """

    def __init__(self, datastore):
        """

        :param datastore: a datastore, containing all relevant data.
        """
        self.__datastore = datastore
        self.__planning_thread_register_time = -1


    def on_execute_pre_planning(self):
        """on_execute
         on_execute executes the pipeline in order to plan a scenario and create a state machine for it
        :return: the planning thread.
        """
        try:
            #pipeline, after input reading...
            #prepare dicts
            start_time = time.time()
            logger.debug('Handover to mapper')
            mapper = Mapper(self.__datastore)
            mapper.generate_action_state_map()                      #--> as_map
            mapper.generate_state_action_map()                      #--> sa_map
            mapper.generate_available_actions()                     #--> available actions
            #load actions into datastore
            logger.debug('Handover to action loader')
            loader = PddlActionLoader(self.__datastore)
            loader.load_pddl_actions()
            #create domain
            logger.debug('Handover to domain generator')
            domain_generator = DomainGenerator(self.__datastore)
            domain_generator.generate_domain()                      #--> domain
            #generate plan
            logger.debug('Handover to planning controller')
            planning_controller = PlanningController(self.__datastore)
            logger.info('Planning preparation took {0:.4f} seconds.'.format(time.time()-start_time))
            planning_thread = planning_controller.execute_planning(self.on_execute_post_planning)
            self.__planning_thread_register_time = self.__datastore.register_thread(planning_thread)
            return planning_thread


        except Exception:
            traceback.print_exc()
            self.on_execute_post_planning(False)



    def on_execute_post_planning(self,planning_successful):

        try:
            # check if a plan was found.
            if planning_successful and len(self.__datastore.get_plan()) > 0:
                logger.info('A Plan was found!')
                sm_generator = StateMachineGenerator(self.__datastore)
                logger.debug('Handover to state machine generator')
                sm_generator.generate_state_machine()  # --> generates state machine and opens it.


            else:
                logger.info("No Plan was found, therefore no state machine was generated!")
        finally:

            if not self.__datastore.keep_related_files():
                logger.info('Cleaning files...')
                for file_name in self.__datastore.get_generated_files():
                    file = os.path.join(self.__datastore.get_file_save_dir(), file_name)
                    if os.path.isfile(file):
                        os.remove(file)
                        logger.debug('Successfully removed file: ' + str(file))
                    else:
                        logger.warning("Couldn't remove " + str(file))
            else:
                logger.debug('Keeping files')

            #remove the planning_thread at the end of the Task.
            if self.__planning_thread_register_time is not -1:
                if not self.__datastore.remove_thread(self.__planning_thread_register_time):
                    logger.debug("could not remove planning thread.")
