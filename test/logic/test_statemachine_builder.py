import pytest
import os
import shutil
from rafcon.planner.statemachine_builder import StateMachineBuilder
from rafcon.planner.plan_parser import PlanParser, PlanActionRepresentation

@pytest.fixture
def lib_path():
    return "./test_data/"


@pytest.fixture
def lib_name():
    return "argos_pool"


@pytest.fixture
def plan():
    parser = PlanParser()
    plan = open("./test_data/sas_plan", "r")
    return parser.parse_raw_file(plan.readlines())


@pytest.fixture
def save_path():
     return "./test_data"


@pytest.fixture
def plan_with_unknowen_action():
    return [PlanActionRepresentation("unknowen_action",["no","args"])]

def test_create_action_state_dict_normal_use(lib_name,lib_path):
    #arrange
    builder = StateMachineBuilder()
    result = None
    #act
    result = builder.create_action_state_dict(lib_name,lib_path)
    #assert
    assert result


def test_get_states_from_actions(lib_name,lib_path,plan):
    #arrange
    builder = StateMachineBuilder()
    ac_dict = builder.create_action_state_dict(lib_name,lib_path)
    result = None
    #act
    result = builder.get_states_from_actions(plan,ac_dict,lib_name,lib_path)
    #assert
    assert (len(plan) > 0) & (len(plan) == len(result))


def test_get_states_from_actions_no_plan(lib_name,lib_path):
    #arrange
    builder = StateMachineBuilder()
    ac_dict = builder.create_action_state_dict(lib_name,lib_path)
    plan = []
    result = None
    #act
    result = builder.get_states_from_actions(plan,ac_dict,lib_name,lib_path)
    #assert
    assert len(plan) == len(result)

def test_get_stetes_from_actions_unknowen_action(lib_name,lib_path,plan_with_unknowen_action):
        #arrange
    builder = StateMachineBuilder()
    ac_dict = builder.create_action_state_dict(lib_name,lib_path)

    #act / assert
    with pytest.raises(LookupError):
        builder.get_states_from_actions(plan_with_unknowen_action,ac_dict,lib_name,lib_path)


def test_build_state_machine(lib_name, lib_path,plan,save_path):
    #arrange
    builder = StateMachineBuilder()
    ac_dict = builder.create_action_state_dict(lib_name,lib_path)
    states = builder.get_states_from_actions(plan,ac_dict,lib_name,lib_path)
    sm_name = "test_sm"
    path_of_sm = None
    #act
    path_of_sm = builder.build_state_machine(states,sm_name,save_path)
    #assert
    assert os.path.isdir(path_of_sm)
    #tidy up
    shutil.rmtree(path_of_sm)


def test_build_state_machine_no_states(lib_name,lib_path,save_path):
    #arrange
    builder = StateMachineBuilder()
    states = []
    sm_name = "test_sm"
    path_of_sm = os.path.abspath(os.path.join(save_path,sm_name))
    result = True
    #act
    builder.build_state_machine(states,sm_name,save_path)
    result = os.path.isdir(path_of_sm)
    #tidy up
    if result:
        shutil.rmtree(path_of_sm)
    #assert
    assert result == False
