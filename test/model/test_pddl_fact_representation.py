import pytest

from rafcontpp.model.pddl_facts_representation import PddlFactsRepresentation


@pytest.fixture
def obj_map():
    return{
        'a':'TypeA',
        'b':'TypeB',
        'Cc':'TypeC'
    }


@pytest.mark.parametrize('obj,expected',[
    (None,None),
    ('',''),
    ('A','a'),
    ('a','a'),
    ('cC','Cc'),
    ('CC','Cc')

])
def test_get_original_object_name(obj, expected):
    #arrange
    sut = PddlFactsRepresentation("",obj_map(),"","")
    #act
    ori_name = sut.get_original_object_name(obj)
    #assert
    assert expected == ori_name