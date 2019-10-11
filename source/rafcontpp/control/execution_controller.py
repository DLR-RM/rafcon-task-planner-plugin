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
# Version 24.05.2019

import os
import threading
import time
import traceback

from rafcon.utils import log

from rafcontpp.control.planning_controller import PlanningController
from rafcontpp.logic.domain_generator import DomainGenerator
from rafcontpp.logic.mapper import Mapper
from rafcontpp.logic.pddl_action_loader import PddlActionLoader
from rafcontpp.logic.pddl_facts_parser import PddlFactsParser
from rafcontpp.logic.state_machine_generator import StateMachineGenerator
from rafcontpp.model.pddl_facts_representation import PddlFactsRepresentation

logger = log.get_logger(__name__)


class ExecutionController:
    """ExecutionController
       The ExecutionController controls the execution of the planning pipeline from mapping up to generating the state
       machine. It's structured in two parts: a pre-planning, and a post-planning part.
    """

    def __init__(self, datastore):
        """
        :param datastore: A datastore, containing all relevant data.
        """
        self.__datastore = datastore
        self.__planning_thread_register_time = -1

    def on_execute_pre_planning(self):
        """
        on_execute_pre_planning prepares the Datastore and gets everything ready for the planning process. Therefore
        it triggeres different modules to map actions with states, loads all available actions and generate a domain file.
        In the end initiates the Planning Process, and returns its thread.

        :return: InterruptableThread: The planning Thread.
        """
        try:
            # pipeline, after input reading...
            # prepare dicts
            logger.verbose('Main thread is: {}'.format(threading.current_thread().getName()))
            start_time = time.time()
            logger.debug('Handover to facts parser')
            facts_repr = self.__parse_facts_file()
            self.__datastore.set_pddl_facts_representation(facts_repr)
            logger.debug('Handover to mapper.')
            mapper = Mapper(self.__datastore)
            mapper.generate_action_state_map()  # --> as_map
            mapper.generate_state_action_map()  # --> sa_map
            mapper.generate_available_actions()  # --> available actions
            # load actions into datastore
            logger.debug('Handover to action loader.')
            loader = PddlActionLoader(self.__datastore)
            loader.load_pddl_actions()
            # create domain
            logger.debug('Handover to domain generator.')
            domain_generator = DomainGenerator(self.__datastore)
            domain_generator.generate_domain()  # --> domain
            # generate plan
            logger.debug('Handover to planning controller')
            planning_controller = PlanningController(self.__datastore)
            logger.info('Planning preparation took {0:.4f} seconds.'.format(time.time() - start_time))
            planning_thread = planning_controller.execute_planning(self.on_execute_post_planning)
            self.__planning_thread_register_time = self.__datastore.register_thread(planning_thread)
            return planning_thread
        except Exception:
            traceback.print_exc()
            self.on_execute_post_planning(False)

    def on_execute_post_planning(self, planning_successful):
        """
        This function takes care of the post planning steps in the pipeline, e.g. it triggeres the state
        machine generation procedure. typically its executed from another thread.

        :param planning_successful: True if planning was successful, False otherwise.
        :return: void
        """
        try:
            logger.verbose('post planning executed from thread: {}'.format(threading.current_thread().getName()))
            # check if a plan was found.
            if planning_successful and len(self.__datastore.get_plan()) > 0:
                logger.info('A Plan was found!')
                sm_generator = StateMachineGenerator(self.__datastore)
                logger.debug('Handover to state machine generator.')
                sm_generator.generate_state_machine()  # --> generates state machine and opens it.
            else:
                logger.info("No Plan was found, therefore no state machine was generated!")
        except Exception as e:
            traceback.print_exc()
            logger.error("Error during State machine generation Process! :: {}".format(e.message))
        finally:
            if not self.__datastore.keep_related_files():
                logger.info('Cleaning files...')
                for file_name in self.__datastore.get_generated_files():
                    file = os.path.join(self.__datastore.get_file_save_dir(), file_name)
                    if os.path.isfile(file):
                        os.remove(file)
                        logger.debug('Successfully removed file: ' + str(file))
                    else:
                        logger.warning("Couldn't remove: " + str(file))
            else:
                logger.debug('Keeping files.')
            # remove the planning_thread at the end of the Task.
            if self.__planning_thread_register_time is not -1:
                if not self.__datastore.remove_thread(self.__planning_thread_register_time):
                    logger.debug("could not remove planning thread.")
                else:
                    logger.debug('removed planning thread successfully.')

    def __parse_facts_file(self):
        """
        Parses the facts file, and creates a facts representation.
        This is done here, because there no better place yet. Later on it will be done in a dedicated facts module.
        TODO delete, and do somewhere else.

        :return: PddlFactsRepresentation: A representation of the parsed facts file.
        """
        facts_file = open(self.__datastore.get_facts_path(), 'r')
        facts_string = facts_file.read()
        facts_parser = PddlFactsParser(facts_string)
        obj_type_map = facts_parser.parse_objects()
        domain_name = facts_parser.parse_domain_name()
        problem_name = facts_parser.parse_problem_name()
        return PddlFactsRepresentation(facts_string, obj_type_map, domain_name, problem_name)
