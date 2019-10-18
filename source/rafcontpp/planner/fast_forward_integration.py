# Copyright (C) 2019 DLR
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
import subprocess
import time

from rafcontpp.model.plan_step import PlanStep
from rafcontpp.model.planner_interface import PlannerInterface
from rafcontpp.model.planning_report import PlanningReport


class FfIntegration(PlannerInterface):
    """
    This is the integration script for the Fast-Forward Planning System version 2.3 by Hoffmann
    (https://fai.cs.uni-saarland.de/hoffmann/ff.html)
    """

    def plan_scenario(self, domain_path, facts_path, planner_argv, storage_path):
        # get own directory for creating files in and so on...
        old_cwd = os.getcwd()
        new_cwd = os.path.join(old_cwd, 'ff_planning_tmp {0:.8f}'.format(time.time()))
        os.mkdir(new_cwd)
        os.chdir(new_cwd)
        plan_path = os.path.abspath(os.path.join(storage_path, "ff_plan"))
        command = 'ff -o ' + domain_path + ' -f ' + facts_path + ' '
        for arg in planner_argv:
            command += arg + ' '
        command = command.rstrip()
        plan = []
        # run Fast Forward
        ff_process = subprocess.Popen([command], stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True)
        (stdout, stderr) = ff_process.communicate()
        ff_exit = ff_process.returncode
        # read plan, if possible
        files_to_delete = []
        if ff_exit == 0:
            parsed_console = self.__parse_console_output(stdout)
            plan = parsed_console[1]
            plan_file = open(plan_path, "w")
            for line in parsed_console[0]:
                plan_file.write(line + '\r\n')
            plan_file.flush()
            plan_file.close()
            files_to_delete = ['ff_plan']
        else:
            stderr = stdout
        # reset to old cwd
        os.chdir(old_cwd)
        os.rmdir(new_cwd)
        return PlanningReport(ff_exit == 0, plan, files_to_delete, str(ff_exit) + ': ' + str(stderr))

    def is_available(self):
        """
        :return: Boolean: True, if the planner is available in the system, false otherwhise.
        """
        devnull = open(os.devnull, "wb")
        process = subprocess.Popen('ff', stdout=devnull, stderr=devnull, shell=True)
        process.wait()
        status = process.returncode
        devnull.close()
        return status != 127  # 127 is the code for command not found.

    def __parse_console_output(self, to_parse):
        """
        Receives the console output, and parses the plan out of it.

        :param to_parse: The console output.
        :return: [PlanStep]: A parsed plan.
        """
        console_output = to_parse.splitlines()
        raw_plan = []
        parsed_plan = []
        if 'ff: found legal plan as follows' in console_output:
            start_index = console_output.index('ff: found legal plan as follows') + 2
            while start_index < len(console_output):
                c_line = console_output[start_index]
                if ':' not in c_line:
                    break
                action_index = c_line.index(':') + 2
                action = c_line[action_index:]
                raw_plan.append(action)
                parts = action.split(" ")
                parsed_plan.append(PlanStep(parts[0], parts[1:]))
                start_index += 1
        return (raw_plan, parsed_plan)
