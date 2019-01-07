import pytest
import os
from rafcontpp.logic.domain_generator import DomainGenerator
from rafcontpp.model.pddl_action_representation import PddlActionRepresentation
from rafcontpp.model.pddl_action_representation import action_to_upper
from rafcontpp.model.datastore import datastore_from_file


@pytest.fixture
def pddl_action_map():
    map =  {
        'EAT': PddlActionRepresentation('eat','(:action eat '
                                             ':parameters (?person - Person ?food - Food) '
                                             ':precondition (and (has ?person ?food))'
                                             ':effect (and (not (has ?person ?food))(be-full ?person)))',
                                       ['(be-full ?person - Person)','(has ?person - Person ?food - Food)'],
                                       ['Food','Person'],
                                       [':strips',':typing'],
                                       ['person','food']
                                       ),
        'COOK': PddlActionRepresentation('cook','(:action cook'
                                                ':parameters (?chef - Chef ?food - Food)'
                                                ':precondition (wants ?chef ?food)'
                                                ':effect (and (has ?chef ?food)(not(wants ?chef ?food))))',
                                         ['(has ?chef - Chef ?food - Food)','(wants ?chef - Chef ?food - Food)'],
                                         ['Chef','Food'],
                                         [':strips',':typing'],
                                         ['chef','food']
                                         ),
        'GIVE': PddlActionRepresentation('give','(:action give'
                                                ':parameters (?interlocutor1 ?interlocutor2 - Person ?food - Food ?loc - Location)'
                                                ':precondition (and (at ?loc ?interlocutor1)'
                                                '(at ?loc ?interlocutor2)'
                                                '(wants ?interlocutor2 ?food)'
                                                '(has ?interlocutor1 ?food))'
                                                ':effect (and (not (wants ?interlocutor2 ?food))'
                                                '(not (has ?interlocutor1 ?food))'
                                                '(has ?interlocutor2 ?food)))',
                                         ['(at ?loc - Location ?interlocutor2 - Person)',
                                          '(has ?interlocutor2 - Person ?food - Food)',
                                          '(wants ?interlocutor2 - Person ?food - Food)'],
                                         ['Food','Person','Location'],
                                         [':strips',':typing'],
                                         ['interlocutor1','interlocutor2','food','loc']
                                         ),
        'GO-TO': PddlActionRepresentation('go-to','(:action got-to'
                                                  ':parameters (?start ?destination - Location ?name - Waiter)'
                                                  ':precondition (and (at ?start ?name)):effect (and (not (at ?start ?name))(at ?destination ?name)))',
                                          ['(at ?destination - Location ?name - Waiter)'],
                                          ['Waiter','Location'],
                                          [':typing',':strips'],
                                          ['start','destination','name']
                                          ),
        'ORDER': PddlActionRepresentation('order','(:action order'
                                                  ':parameters (?interlocutor1 ?interlocutor2 - Person ?food - Food ?loc - Location)'
                                                  ':precondition (and (at ?loc ?interlocutor1)(at ?loc ?interlocutor2)'
                                                  '(wants ?interlocutor2 ?food))'
                                                  ':effect (and (wants ?interlocutor1 ?food)))',
                                          ['(at ?loc - Location ?interlocutor2 - Person)',
                                           '(wants ?interlocutor1 - Person ?food - Food)'],
                                          ['Food','Person','Location'],
                                          [':strips',':typing'],
                                          ['interlocutor1','interlocutor2','food','loc']
                                          )


    }

    for key in map.keys():
        map[key] = action_to_upper(map[key])

    return map

@pytest.fixture
def datastore():
     base_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'test_data')
     facts_path = os.path.join(base_path, 'test_facts.pddl')
     type_db_path = os.path.join(base_path, 'test_type_db.json')
     file_save_dir = base_path
     ds = datastore_from_file('')
     ds.set_facts_path(facts_path)
     ds.set_type_db_path(type_db_path)
     ds.set_file_save_dir(file_save_dir)
     ds.set_pddl_action_map(pddl_action_map())
     return ds

def test_domain_generator_datastore_none():
    #assert
    with pytest.raises(ValueError):
        DomainGenerator(None)

def test_generate_domain():
    #arrange
    ds = datastore()
    sut = DomainGenerator(ds)
    #act
    sut.generate_domain()
    domain_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'test_data','restaurant_domain.pddl')
    generated_domain_file = open(domain_path)
    generated_domain = generated_domain_file.read()
    os.remove(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'test_data','restaurant_domain.pddl'))
    #assert
    static_domain_file = open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'test_data','restaurant_domain_static.pddl'))
    static_domain = static_domain_file.read()
    assert static_domain == generated_domain
    assert domain_path == ds.get_domain_path()
