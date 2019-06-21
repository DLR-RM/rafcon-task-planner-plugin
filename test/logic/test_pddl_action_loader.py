import pytest
import os
from rafcontpp.logic.pddl_action_loader import PddlActionLoader
from rafcontpp.model.datastore import datastore_from_file


@pytest.fixture
def datastore():
    base_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'test_data')
    facts_path = os.path.join(base_path, 'test_facts.pddl')
    domain_path = os.path.join(base_path, 'restaurant_domain_static.pddl')
    file_save_dir = base_path
    ds = datastore_from_file('')
    ds.set_facts_path(facts_path)
    ds.set_domain_path(domain_path)
    ds.set_file_save_dir(file_save_dir)
    ds.set_planner('Fast Downward Planning System')
    return ds

@pytest.mark.parametrize("string, expected",[
    ('',[]),
    (None,[]),
    ('type1',['type1']),
    ('type1, type2',['type1','type2']),
    ('type1 type2',['type1','type2']),
    ('type1, type1',['type1','type1']),
    ('type1, type2, type3',['type1','type2','type3'])

])
def test_parse_type_string(string, expected):
    #arrange
    sut = PddlActionLoader(datastore())
    #act
    result = sut.parse_type_string(string)
    #assert
    assert expected == result


@pytest.mark.parametrize("string, expected",[
    ('[]',['']),
    (None,[]),
    ('[:typing]',[':typing']),
    ('[:typing, :strips]',[':typing',':strips']),
    ('[:typing, :adl, :fluents]',[':typing',':adl',':fluents'])

])
def test_parse_requirement_string(string, expected):
    # arrange
    sut = PddlActionLoader(datastore())
    # act
    result = sut.parse_requirement_string(string)
    # assert
    assert expected == result


@pytest.mark.parametrize("string, expected",[
    ('[]',[]),
    (None,[]),
    ('no predicates in here',[]),
    ('(be-full ?person - Person)(has ?person - Person ?food - Food)',['(be-full ?person - Person)','(has ?person - Person ?food - Food)']),
    ('(be-full ?person - Person)    (has ?person - Person ?food - Food)',['(be-full ?person - Person)', '(has ?person - Person ?food - Food)']),
    ('(be-full ?person - Person)\r\n(has ?person - Person ?food - Food)',['(be-full ?person - Person)','(has ?person - Person ?food - Food)']),

])
def test_parse_predicate_string(string, expected):
    # arrange
    sut = PddlActionLoader(datastore())
    # act
    result = sut.parse_predicate_string(string)
    # assert
    assert expected == result
