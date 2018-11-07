class PlannerInterface:



    def plan_scenario(self,domain_path, facts_path, planner_argv, storage_path):
        '''
        plan_scenario
        plan_scenario, uses the given arguments, do plan a task, parse the plan into a list of PlanSteps, and returns
        them in a PlanningReport.
        :param domain_path: The absolute path of the domain file
        :param facts_path: The absolute path of the facts file
        :param planner_argv: Some additional planner args, optional
        :param storage_path: The location, where to save all produced artefacts (like a plan file) in.
        :return: a PlanningReport (source.de.dlr.rmc.rafconttp.model.planningreport)
        '''

        raise NotImplementedError('This Method has to be overwritten!')