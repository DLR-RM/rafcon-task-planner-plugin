import pytest
import os
from rafcon.planner.domain_generator import TypeTree, DomainGenerator, PddlActionRepresentation

#Test TypeTree
@pytest.fixture
def filled_type_dict():
    return {
        "Location": "Object",
        "Vehicle": "Object",
        "Spot": "Location",
        "Car": "Vehicle",
        "Roadstar": "Car"
    }


def test_create_type_tree():
    # arrange
    root_type = "some_type"
    sut = TypeTree(root_type)
    # assert
    assert sut.is_in_tree(root_type)


def test_create_None_type_tree():
    #arrange
    root_type = None
    #assert
    with pytest.raises(ValueError):
        sut = TypeTree(root_type)


@pytest.mark.parametrize("root_type,type_name,parent_name,expected", [
    # insert fail. because parent not in tree
    ("some_type", "a_type", "not_some_type", False),
    # usual insert.
    ("some_type", "a_type", "some_type", True),
    # try to insert null value
    ("some_type", None, "some_type", False),
    # try to insert something twice.
    ("some_type", "some_type", "some_type", False)
])
def test_insert(root_type, type_name, parent_name, expected):
    # arrange
    sut = TypeTree(root_type)
    inserted = False
    # act
    inserted = sut.insert(type_name, parent_name)
    # assert
    assert inserted == expected

@pytest.mark.parametrize("root_type,type_to_insert,expected",[
    #usual recursive insert
    ("Object","Roadstar",True),
    #try to insert none
    ("Object",None,False),
    #try to insert not in dict
    ("Object","type_not_in_dict",False),
    #try to insert type whic is above root
    ("Roadstar","Car",False)
])
def test_recursive_insert(root_type,type_to_insert,expected,filled_type_dict):
    #arrange
    sut = TypeTree(root_type)
    inserted = False
    #act
    inserted = sut.recursive_insert(type_to_insert,filled_type_dict)
    #assert
    assert inserted == expected
    assert sut.is_in_tree(type_to_insert) == expected

@pytest.mark.parametrize("root_type,type_to_insert,other_type_in_branch,expected",[
    #add usual branch
    ("Object","Car","Roadstar",True),
    #try to insert none
    ("Object",None,None,False),
    #insert not in dict
    ("Object","not_in_dict","child_not_in_dict",False),
    #insert bottom
    ("Object","Spot","Location",True),
    #insert top
    ("Object","Vehicle","Roadstar",True),
    #insert above root
    ("Car","Vehicle","Roadstar",False)


])
def test_add_type_branch(root_type,type_to_insert, other_type_in_branch, expected,filled_type_dict):
    #arrange
    sut = TypeTree(root_type)
    inserted = False
    #act
    inserted = sut.add_type_branch(type_to_insert,filled_type_dict)
    #assert
    assert inserted == expected
    assert sut.is_in_tree(type_to_insert) == expected
    assert sut.is_in_tree(other_type_in_branch) == expected

@pytest.mark.parametrize("root_type,type_to_insert,type_to_look,expected",[
    #usual lookup
    ("Object","Spot","Spot",True),
    #lookup none
    ("Object","Location",None,False),
    #lookup not in
    ("Object","Location","Not_In",False)
])
def test_is_in_tree(root_type,type_to_insert,type_to_look,expected,filled_type_dict):
    #prepare
    sut = TypeTree(root_type)
    isInTree = False
    sut.recursive_insert(type_to_insert,filled_type_dict)
    #act
    isInTree = sut.is_in_tree(type_to_look)
    #assert
    assert expected == isInTree


@pytest.mark.parametrize("root_type,to_insert,expected",[
    #a tree which has a root only.
    ("Object",[],"Object"),
    #a tree which has a root only, and is not Object.
    ("not_Object",[],"not_Object"),
    #a tree with branches
    ("Object",["Spot","Roadstar"],"Location Vehicle - Object\r\nSpot - Location\r\nCar - Vehicle\r\nRoadstar - Car\r\n"),
    #a tree with one branch, inserted above root
    ("Car",["Object","Location","Roadstar"],"Roadstar - Car\r\n")

])
def test_get_as_string(root_type,to_insert,expected,filled_type_dict):
    #prepare
    sut = TypeTree(root_type)
    tree_as_string = ""
    for type_name in to_insert:
        sut.add_type_branch(type_name,filled_type_dict)
    #act
    tree_as_string = sut.get_as_string()
    #assert
    assert expected == tree_as_string


#Test DomainGenerator

@pytest.fixture
def pddl_actions():
    return[
        PddlActionRepresentation("a1",["(:action a1)"],
                                 ["(at ?loc - Location)","(in ?obj - Object)"],
                                 ["Object","Location"],[":adl",":strips",":typing"]),
        PddlActionRepresentation("a2", ["(:action a2)"],
                                 ["(at ?loc - Location)", "(under ?obj - Object)"],
                                 ["Object", "Location"], [":strips", ":typing"]),
        PddlActionRepresentation("a3", ["(:action a3)"],
                                 ["(at ?loc - Location)", "(in ?obj - Object)","(under ?obj - Object)"],
                                 ["Object", "Location"], [":adl", ":typing"])
        ]


@pytest.mark.parametrize("actions,type_dict, expected",[
    #test with no actions
    ([],filled_type_dict(),"(DEFINE (DOMAIN test_domain)\r\n(:REQUIREMENTS )\r\n\r\n(:PREDICATES\r\n)\r\n\r\n)"),
    #test usual case with prepared actions
    (pddl_actions(),filled_type_dict(),"(DEFINE (DOMAIN test_domain)\r\n(:REQUIREMENTS :adl :strips :typing )"
                   +"\r\n(:TYPES \r\nLocation - Object\r\nSpot - Location\r\n)\r\n(:PREDICATES"
                   +"\r\n(at ?loc - Location)\r\n(in ?obj - Object)\r\n(under ?obj - Object)\r\n)"
                   +"\r\n(:action a1)\r\n\r\n(:action a2)\r\n\r\n(:action a3)\r\n\r\n\r\n)")

])
def test_generate_domain(actions,type_dict, expected):
    #prepare
    sut = DomainGenerator()
    domain_name = "test_domain"
    #path where to save the domain
    domain_dir = "./"
    #reference on the generated domain
    domain_path = ""
    red_domain = ""
    #act
    domain_path = sut.generate_domain(domain_name,actions,type_dict,domain_dir)
    #assert
    if os.path.isfile(domain_path):
        domain_file = open(domain_path, "r")
        red_domain = domain_file.read()
        domain_file.close()
        os.remove(domain_path)
    print "\r\n"+expected +"\r\n"+red_domain
    assert expected == red_domain

def test_generate_domain_no_type_dict(pddl_actions):
    sut = DomainGenerator()
    domain_name = "test_domain"
    # path where to save the domain
    domain_dir = "./"
    # reference on the generated domain
    domain_path = ""
    red_domain = ""
   # assert
    with pytest.raises(LookupError):
        domain_path = sut.generate_domain(domain_name, pddl_actions, {}, domain_dir)

