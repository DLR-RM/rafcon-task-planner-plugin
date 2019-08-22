import pytest

from rafcontpp.logic.pddl_action_parser import PddlActionParser


@pytest.fixture
def action_string_with_comments():
    return '(:action my_gripping_action ;fancy name\n' \
                ':parameters( ?a - Location ?b ?d - Object ?c -  Gripper);my Fancy comment\n' \
                ':preconditions (and (at ?a ?b) (at ?a ?c) (empty ?c));;(not-full ?d) would be wrong\n' \
                ':effect(and (not(at ?a ?b))(not(empty ?c))(grasped ?b ?c))); (gripped; ?a ?b ?c) is no predicate\n'

def action_string_without_comments():
    return '(:action my_gripping_action \n' \
           ':parameters( ?a - Location ?b ?d - Object ?c -  Gripper)\n' \
           ':preconditions (and (at ?a ?b) (at ?a ?c) (empty ?c))\n' \
           ':effect(and (not(at ?a ?b))(not(empty ?c))(grasped ?b ?c)))\n'

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
    return ':parameters( ?a - Location ?b ?d - Object ?c -  Gripper)\n'

@pytest.fixture
def parameters():
    return ['a','b','d','c']

@pytest.fixture
def preconditions():
    return':preconditions (and (at ?a ?b) (at ?a ?c) (empty ?c))\n'

@pytest.fixture
def effects():
    return ':effect(and (not(at ?a ?b))(not(empty ?c))(grasped ?b ?c)))\n'


def test_parse_normal_action():
    #arrange
    parser = PddlActionParser(action_string_with_comments())
    #act
    pddl_action = parser.parse_action()
    pddl_action.predicates.sort()
    pddl_action.types.sort()
    #assert
    assert pddl_action.predicates == predicates()
    assert pddl_action.types == types()
    assert pddl_action.name == action_name()
    assert pddl_action.action == action_string_without_comments()
    assert pddl_action.requirements == []

def test_parse_action_name():
    # arrange
    parser = PddlActionParser(action_string_with_comments())
    # act
    pddl_action_name = parser.parse_action_name()
    # assert
    assert pddl_action_name == action_name()

def test_init_empty_action():
    #assert
    with pytest.raises(ValueError):
        parser = PddlActionParser('')

def test_init_none_action():

    #assert
    with pytest.raises(ValueError):
        parser = PddlActionParser(None)


def test_parse_missing_name_action_string():
    #arrange
    parser = PddlActionParser(action_string_with_comments()[action_string_with_comments().find(':'):])
    #assert
    with pytest.raises(ValueError):
        parser.parse_action()

def test_parse_missing_parameters_action_string():
    #arrange
    parser = PddlActionParser(action_string_without_comments().replace(parameter_line(),''))
    #assert
    with pytest.raises(ValueError):
        parser.parse_action()

def test_parse_missing_preconditions_action_string():
    #arrange
    parser = PddlActionParser(action_string_without_comments().replace(preconditions(),''))
    #act
    pddl_action = parser.parse_action()
    pddl_action.predicates.sort()
    pddl_action.types.sort()
    #assert
    assert pddl_action.predicates == ['(at ?a - Location ?b - Object)',
                                      '(empty ?c - Gripper)',
                                      '(grasped ?b - Object ?c - Gripper)']
    assert pddl_action.types == types()
    assert pddl_action.name == action_name()
    assert pddl_action.action == action_string_without_comments().replace(preconditions(),'')
    assert pddl_action.requirements == []

def test_parse_missing_effects_action_string():
    # arrange
    parser = PddlActionParser(action_string_without_comments().replace(effects(), ''))
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
    assert pddl_action.action == action_string_without_comments().replace(effects(), '')
    assert pddl_action.requirements == []


def test_parameters_parsing_action_string():
    #arrange
    parser = PddlActionParser(action_string_with_comments())
    # act
    params = parser.parse_parameters()
    # assert
    assert params == parameters()


def test_normal_parse_tricy_action_of_bug1():
    action_string = "(:action teleport" \
                    ":parameters (?turtle - Turtle ?location - Location)" \
                    ":precondition (and (not (alive ?turtle))(not(exists(?pos - Location)(and (at ?pos ?turtle)))))" \
                    ":effect (and (at ?location ?turtle)))"

    parser = PddlActionParser(action_string)
    action = parser.parse_action()
    assert ['(alive ?turtle - Turtle)','(at ?location - Location ?turtle - Turtle)'] == action.predicates







