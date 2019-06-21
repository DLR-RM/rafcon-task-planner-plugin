import pytest
from rafcontpp.model.pddl_action_representation import PddlActionRepresentation
from rafcontpp.model.pddl_action_representation import action_to_upper

def test_action_to_upper():
    #arrange
    action = PddlActionRepresentation('myAction','(action:)',['(at ?a - Object)'],['Object'],[':strips'],['param1','param2'])
    #act
    action = action_to_upper(action)
    #assert
    assert 'MYACTION' == action.name
    assert '(ACTION:)' == action.action
    assert ['(AT ?A - OBJECT)'] == action.predicates
    assert ['OBJECT'] == action.types
    assert [':STRIPS'] == action.requirements
    assert ['param1','param2'] == action.parameters