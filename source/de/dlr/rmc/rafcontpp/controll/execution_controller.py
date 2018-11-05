import os
from de.dlr.rmc.rafcontpp.model.datastore import Datastore
from de.dlr.rmc.rafcontpp.logic.mapper import Mapper
from de.dlr.rmc.rafcontpp.logic.domain_generator import DomainGenerator
from de.dlr.rmc.rafcontpp.logic.state_machine_generator import StateMachineGenerator
from de.dlr.rmc.rafcontpp.controll.planning_controller import PlanningController
from rafcon.utils import log

logger = log.get_logger(__name__)


class ExecutionController:


    def __init__(self, datastore):
        self.__datastore = datastore

    def on_execute(self):
        try:
            #pipeline, after input reading...
            #prepare dicts
            mapper = Mapper(self.__datastore)
            mapper.generate_action_state_map()                      #--> as_map
            mapper.generate_state_action_map()                      #--> sa_map
            mapper.generate_available_actions()                     #--> available actions
            #create domain
            domain_generator = DomainGenerator(self.__datastore)
            domain_generator.generate_domain()                      #--> domain
            #generate plan
            planning_controller = PlanningController(self.__datastore)
            planning_successful = planning_controller.execute_planning()
            #check if a plan was found.
            if planning_successful and len(self.__datastore.get_plan()) > 0:
                sm_generator = StateMachineGenerator(self.__datastore)
                sm_generator.generate_state_machine() #--> generate state machine
                #now open sm!

            else:
                logger.warning("No Plan was found, therefore no state machine was generated!")

        finally:

            if not self.__datastore.keep_related_files():
                for file_name in self.__datastore.get_generated_files():
                    file = os.path.join(self.__datastore.get_file_save_dir(), file_name)
                    logger.info('Successfully removed file: '+str(file))
                    if os.path.isfile(file):
                        os.remove(file)
                    else:
                        logger.warning("Coundn't remove "+str(file))
