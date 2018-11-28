import pytest
import os
from rafcontpp.logic.domain_generator import DomainGenerator
from rafcontpp.model.datastore import datastore_from_file
from rafcontpp.model.pddl_action_representation import PddlActionRepresentation


#Test DomainGenerator

@pytest.fixture
def pddl_actions():
    return[
        PddlActionRepresentation("a1",["(:action a1)"],
                                 ["(at ?loc - Location)","(in ?obj - Object)"],
                                 ["Object","Location"],[":adl",":strips",":typing"], []),
        PddlActionRepresentation("a2", ["(:action a2)"],
                                 ["(at ?loc - Location)", "(under ?obj - Object)"],
                                 ["Object", "Location"], [":strips", ":typing"], []),
        PddlActionRepresentation("a3", ["(:action a3)"],
                                 ["(at ?loc - Location)", "(in ?obj - Object)","(under ?obj - Object)"],
                                 ["Object", "Location"], [":adl", ":typing"], [])
        ]

@pytest.fixture
def filled_type_dict():
    return {
        "Location": "Object",
        "Vehicle": "Object",
        "Spot": "Location",
        "Car": "Vehicle",
        "Roadstar": "Car"
    }


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
    ds = datastore_from_file(None)
    sut = DomainGenerator(ds)
    ds.set_domain_name("test_domain")
    #path where to save the domain
    domain_dir = "./"
    ds.set_domain_path('./')
    #reference on the generated domain
    domain_path = "./"

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

