#
#
#Contributors:
#Christoph Suerig <christoph.suerig@dlr.de>
#Version 12.11.2018
class PlannerInterface:



    def plan_scenario(self,domain_path, facts_path, planner_argv, storage_path):
        """
        plan_scenario
        plan_scenario, uses the given arguments, do plan a task, parse the plan into a list of PlanSteps, and returns
        them in a PlanningReport.
        :param domain_path: The absolute path of the domain file as string
        :param facts_path: The absolute path of the facts file as string
        :param planner_argv: Some additional planner args as string array.
        :param storage_path: The location, where to save all produced artefacts (like a plan file) in. as string.
        :return: a PlanningReport (rafcontpp.model.planning_report)
        """

        raise NotImplementedError('This Method has to be overwritten!')