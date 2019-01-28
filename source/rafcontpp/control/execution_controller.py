# Contributors:
# Christoph Suerig <christoph.suerig@dlr.de>
# Version 01.12.2018

import os
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


    def on_execute(self):
        """on_execute
         on_execute executes the pipeline in order to plan a scenario and create a state machine for it
        :return: nothing
        """
        try:
            #pipeline, after input reading...
            #prepare dicts
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
            planning_successful = planning_controller.execute_planning()
            #check if a plan was found.
            if planning_successful and len(self.__datastore.get_plan()) > 0:
                logger.info('Planning successful!')
                sm_generator = StateMachineGenerator(self.__datastore)
                logger.debug('Handover to state machine generator')
                sm_generator.generate_state_machine() #--> generates state machine and opens it.


            else:
                logger.error("No Plan was found, therefore no state machine was generated!")

        finally:

            if not self.__datastore.keep_related_files():
                logger.info('Cleaning files...')
                for file_name in self.__datastore.get_generated_files():
                    file = os.path.join(self.__datastore.get_file_save_dir(), file_name)
                    if os.path.isfile(file):
                        os.remove(file)
                        logger.debug('Successfully removed file: ' + str(file))
                    else:
                        logger.warning("Coundn't remove "+str(file))
            else:
                logger.debug('Keeping files')