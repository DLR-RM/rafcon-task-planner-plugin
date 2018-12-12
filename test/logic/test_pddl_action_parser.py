import pytest
import os
from rafcontpp.logic.pddl_action_parser import PddlActionParser



@pytest.fixture
def action_string():
    return '(:action my_gripping_action' \
                ':parameters( ?a - Location ?b ?d - Object ?c -  Gripper)' \
                ':preconditions (and (at ?a ?b) (at ?a ?c) (empty ?c))' \
                ':effect(and (not(at ?a ?b))(not(empty ?c))(grasped ?b ?c)))'

@pytest.fixture
def predicates():
    preds = ['(at ?a - Location ?b - Object)', '(at ?a - Location ?c - Gripper)',
             '(empty ?c - Gripper)', '(grasped ?b - Object ?c - Gripper)']
    preds.sort()
    return preds[:]

@pytest.fixture
def types():
    typs = ['Object', 'Location', 'Gripper']
    typs.sort()
    return typs

@pytest.fixture
def action_name():
    return 'my_gripping_action'

@pytest.fixture
def parameter_line():
    return ':parameters( ?a - Location ?b ?d - Object ?c -  Gripper)'

@pytest.fixture
def parameters():
    return ['a','b','d','c']

@pytest.fixture
def preconditions():
    return':preconditions (and (at ?a ?b) (at ?a ?c) (empty ?c))'

@pytest.fixture
def effects():
    return ':effect(and (not(at ?a ?b))(not(empty ?c))(grasped ?b ?c)))'


def test_parse_normal_action():
    #arrange
    parser = PddlActionParser(action_string())
    #act
    pddl_action = parser.parse_action()
    pddl_action.predicates.sort()
    pddl_action.types.sort()
    #assert
    assert pddl_action.predicates == predicates()
    assert pddl_action.types == types()
    assert pddl_action.name == action_name()
    assert pddl_action.action == action_string()
    assert pddl_action.requirements == []

def test_parse_action_name():
    # arrange
    parser = PddlActionParser(action_string())
    # act
    pddl_action_name = parser.parse_action_name()
    # assert
    assert pddl_action_name == action_name()

def test_parse_empty_action():
    #arrange
    parser = PddlActionParser('')
    #assert
    with pytest.raises(ValueError):
        parser.parse_action()

def test_parse_empty_action_name():
    #arrange
    parser = PddlActionParser('')
    #assert
    with pytest.raises(ValueError):
        parser.parse_action_name()

def test_parse_none_action():
    #arrange
    parser = PddlActionParser(None)
    #assert
    with pytest.raises(ValueError):
        parser.parse_action()

def test_parse_none_action_name():
    #arrange
    parser = PddlActionParser('')
    #assert
    with pytest.raises(ValueError):
        parser.parse_action_name()

def test_parse_missing_name_action_string():
    #arrange
    parser = PddlActionParser(action_string()[action_string().find(':'):])
    #assert
    with pytest.raises(ValueError):
        parser.parse_action()

def test_parse_missing_parameters_action_string():
    #arrange
    parser = PddlActionParser(action_string().replace(parameter_line(), ''))
    #assert
    with pytest.raises(ValueError):
        parser.parse_action()

def test_parse_missing_preconditions_action_string():
    #arrange
    parser = PddlActionParser(action_string().replace(preconditions(),''))
    #act
    pddl_action = parser.parse_action()
    pddl_action.predicates.sort()
    pddl_action.types.sort()
    #assert
    assert pddl_action.predicates != predicates()
    assert pddl_action.types == types()
    assert pddl_action.name == action_name()
    assert pddl_action.action == action_string().replace(preconditions(),'')
    assert pddl_action.requirements == []

def test_parse_missing_effects_action_string():
    # arrange
    parser = PddlActionParser(action_string().replace(effects(), ''))
    # act
    pddl_action = parser.parse_action()
    pddl_action.predicates.sort()
    preds = predicates()
    preds.remove('(grasped ?b - Object ?c - Gripper)')
    preds.sort()
    # assert

    assert pddl_action.predicates == preds
    assert pddl_action.types.sort() == types().sort()
    assert pddl_action.name == action_name()
    assert pddl_action.action == action_string().replace(effects(), '')
    assert pddl_action.requirements == []


def test_parameters_parsing_action_string():
    #arrange
    parser = PddlActionParser(action_string())
    # act
    params = parser.parse_parameters()
    # assert
    assert params == parameters()

def test_parameters_parsing_empty_action_string():
    #arrange
    parser = PddlActionParser('')
    # assert
    with pytest.raises(ValueError):
        parser.parse_parameters()





