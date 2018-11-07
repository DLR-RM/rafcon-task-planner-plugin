from rafcontpp.model.plan_step import PlanStep

class PlanningReport:

    def __init__(self, planning_successful, plan, generated_files, error_message='No Error!'):
        if plan and len(plan) > 0 and (not isinstance(plan[0],PlanStep)):
            raise TypeError('Plan has to be a list of type PlanStep!')

        self.__planning_successful = planning_successful
        self.__plan = plan
        self.__generated_files = generated_files
        self.__error_message = error_message

    __planning_successful = False
    __plan = None
    __error_message = None


    def planning_successful(self):
        return self.__planning_successful

    def get_plan(self):
        return self.__plan

    def get_error_message(self):
        return self.__error_message

    def get_generated_files(self):
        return self.__generated_files
