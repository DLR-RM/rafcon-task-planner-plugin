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
# Version 31.05.2019
import os
import shutil
import subprocess
import time

from rafcontpp.model.plan_step import PlanStep
from rafcontpp.model.planner_interface import PlannerInterface
from rafcontpp.model.planning_report import PlanningReport


class FdIntegration(PlannerInterface):
    """
    This is the integration Script for the Fast downward Planning System by Malte Helmert Et al. (fast-downward.org)
    """

    def plan_scenario(self, domain_path, facts_path, planner_argv, storage_path):
        # get own directory for creating files in and so on...
        old_cwd = os.getcwd()
        new_cwd = os.path.join(old_cwd, 'fd_planning_tmp {0:.8f}'.format(time.time()))
        os.mkdir(new_cwd)
        os.chdir(new_cwd)
        command = 'fast-downward ' + domain_path + ' ' + facts_path + ' '
        plan_path = os.path.abspath(os.path.join(os.curdir, "sas_plan"))
        outsas = os.path.abspath(os.path.join(os.curdir, "output.sas"))
        plan = []
        if len(planner_argv) == 0:
            command += '--search \"astar(blind())\"'
        else:
            for arg in planner_argv:
                command += arg + ' '
            command = command.rstrip()
        # run Fast-downward
        fd_process = subprocess.Popen([command], stdout=subprocess.PIPE, shell=True)
        (out, err) = fd_process.communicate()
        fd_exit = fd_process.returncode
        # read plan, if possible
        if fd_exit == 0:
            plan = self.__parse_raw_plan(plan_path)
        copied_files = self.__copy_and_clean(plan_path, outsas, storage_path)
        # reset to old cwd
        os.chdir(old_cwd)
        os.rmdir(new_cwd)
        return PlanningReport(fd_exit < 4,
                              plan,
                              copied_files,
                              str(fd_exit)
                              + ': ' + self.__translate_fd_exit_code(fd_exit) + " used command was: " + command)

    def is_available(self):
        """
        :return: Boolean: True, if the planner is available in the system, false otherwise.
        """
        devnull = open(os.devnull, "wb")
        process = subprocess.Popen('fast-downward', stdout=devnull, stderr=devnull, shell=True)
        process.wait()
        status = process.returncode
        devnull.close()
        return status != 127  # 127 is the code for command not found.

    def __parse_raw_plan(self, plan_path):
        """
        :param plan_path: The path of the plan file.
        :return: [PlanStep]: A parsed plan.
        """
        parsed_plan = []
        if os.path.isfile(plan_path):
            plan_file = open(plan_path, "r")
            raw_plan = plan_file.readlines()
            raw_plan.pop()
            # parsing the action out of the raw file.
            for action in raw_plan:
                # remove useless chars
                action = action[:-2]
                action = action[1:]
                parts = action.split(" ")
                parsed_plan.append(PlanStep(parts[0], parts[1:]))
        return parsed_plan

    def __copy_and_clean(self, plan_path, outsas_path, storage_path):
        """
        Copy and clean moves all generated files to the storage_path, and writes their names into a list.

        :param plan_path: The path of the plan.
        :param outsas_path: The path of the outsas file.
        :param storage_path: The storage path.
        :return: [String]: A list with the names of the moved files.
        """

        copied_files = []
        if os.path.isfile(plan_path):
            shutil.move(plan_path, os.path.join(storage_path, 'sas_plan'))
            copied_files.append('sas_plan')
        if os.path.isfile(outsas_path):
            shutil.move(outsas_path, os.path.join(storage_path, 'output.sas'))
            copied_files.append('output.sas')
        return copied_files

    def __translate_fd_exit_code(self, fd_exit):
        """
        receives an error code and returns the corresponding error message.

        :param fd_exit: An exit code
        :return: String: The corresponding error message.
        """

        translated_exit_code = str(fd_exit)
        codes = {
            0: "No Error!",
            1: "at least one plan was found and another component ran out of memory.",
            2: "at least one plan was found and another component ran out of time.",
            3: "at least one plan was found, another component ran out of memory, and yet another one ran out of time. ",
            10: "Translator proved task to be unsolvable.",
            11: "Task is provably unsolvable with current bound.",
            12: "Search ended without finding a solution. ",
            20: "Translate, Memory exhausted.",
            21: "Translate, Time exhausted. Not supported on Windows because we use SIGXCPU to kill the planner.",
            22: "Search, Memory exhausted.",
            23: "Search, Timeout occurred. Not supported on Windows because we use SIGXCPU to kill the planner. ",
            24: "Search, one component ran out of memory and another one out of time. ",
            30: "Critical error: something went wrong (e.g. translator bug, but also malformed PDDL input).",
            31: "Usage error: wrong command line options",
            32: "Something went wrong that should not have gone wrong (e.g. planner bug).",
            33: "Wrong command line options or SAS+ file.",
            34: "Requested unsupported feature.",
            35: "Something went wrong in the driver (e.g. failed setting resource limits, ill-defined portfolio, complete plan generated after an incomplete one).",
            36: "Usage error: wrong or missing command line options, including (implicitly) specifying non-existing paths (e.g. for input files or build directory).",
            37: "Requested unsupported feature (e.g. limiting memory on macOS).",
            127: "Couldn't start FastDownward Planner: command \"fast-downward.py\" not found!"
        }

        if fd_exit in codes:
            translated_exit_code = str(fd_exit) + " => " + codes.get(fd_exit, "Unknown")
        return translated_exit_code
