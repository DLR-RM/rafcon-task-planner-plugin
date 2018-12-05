import pytest
import os

from .plan_parser import PlanParser, PlanActionRepresentation

@pytest.fixture
def plan_as_string_list():
    plan = open("./test_data/sas_plan","r")
    return plan.readlines()


@pytest.fixture
def no_plan_as_string_list():
    plan = open("./test_data/this_is_no_plan","r")
    return plan.readlines()


def test_parse_raw_file(plan_as_string_list):
    #arrange
    parser = PlanParser()
    plan = []
    #act
    plan = parser.parse_raw_file(plan_as_string_list)
    #assert, the plan found was 27 steps long.
    assert len(plan) == 27

def test_parse_raw_file_no_plan(no_plan_as_string_list):
    #arrange
    parser = PlanParser()
    plan = None
    #act
    plan = parser.parse_raw_file(no_plan_as_string_list)
    #assert
    assert plan == []

@pytest.mark.parametrize("pddl_actions,action_file_path,expected",[
    ([],"./test_data/actiondb.json",[]),
    (["move"],"./test_data/actiondb.json",[True]),
    (["navigate","load-into","move"],"./test_data/actiondb.json",[True,True,True]),
    (["navigate","this_is_no_action","move"],"./test_data/actiondb.json",[True,False,True]),
    (["this_is_no_action"],"./test_data/actiondb.json",[False])

])
def test_get_pddl_actions_from_file(pddl_actions,action_file_path,expected):
    #arrange
    parser = PlanParser()
    result = None
    #act
    result = parser.get_pddl_actions_from_file(action_file_path,pddl_actions)
    #assert
    for i in range(0, len(expected)-1):
        is_in_list = False
        for action in result:
            if action.name == pddl_actions[i]:
                is_in_list = True
                break;
        assert is_in_list == expected[i]

