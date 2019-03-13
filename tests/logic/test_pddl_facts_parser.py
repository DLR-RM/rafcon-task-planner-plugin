import pytest
import os
from rafcontpp.logic.pddl_facts_parser import PddlFactsParser



@pytest.mark.parametrize("obj_section",[
    (' '),
    ('(:objects)'),
    ('aseeesafffasadfsafsa'),
    ('(:Objects a b c - Type b - Type2)')


])

def test_error_parse_object(obj_section):
    #arrange
    sut = PddlFactsParser(obj_section)
    #act
    with pytest.raises(ValueError):
        sut.parse_objects()

@pytest.mark.parametrize('obj_section,expected',[
    ('(:Objects a b c - Type)',{'a':'Type','b':'Type','c':'Type'}),
    ('(:Objects a b c c - Type)',{'a':'Type','b':'Type','c':'Type'}),
    ('(:Objects a b c - Type d e f   -   type2)',{'a':'Type','b':'Type','c':'Type','d':'type2','e':'type2','f':'type2'}),



])
def test_parse_object(obj_section,expected):
    # arrange
    sut = PddlFactsParser(obj_section)
    #act
    map = sut.parse_objects()
    #assert
    assert expected == map
