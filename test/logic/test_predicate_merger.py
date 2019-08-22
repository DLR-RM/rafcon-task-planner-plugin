import pytest

from rafcontpp.logic.predicate_merger import PredicateMerger
from rafcontpp.model.datastore import datastore_from_file
from rafcontpp.model.type_tree import TypeTree


@pytest.fixture
def get_type_tree():
    dict = { 'Location': 'Object',
             'Vehicle': 'Object',
             'Car': 'Vehicle',
             'City': 'Location'}
    tree = TypeTree('Object')
    tree.add_type_branch('Object',dict)
    return tree


#BEWARE THIS Test is not stable, because of the variable names (e.g. ?C120)!
@pytest.mark.parametrize("predicates,expected",[
    ([],[]),
    (['(in ?a - Vehicle ?c - City)','(in ?a - Vehicle ?c - City)'], ['(in ?V10 - Vehicle ?C20 - City)']),
    (['(in ?a - Car ?c - City)','(in ?a - Vehicle ?l - Location)'], ['(in ?V10 - Vehicle ?L20 - Location)']),
    (['(in ?a - Vehicle ?c - City)','(in ?a - Car ?l - Location)'], ['(in ?V10 - Vehicle ?L20 - Location)']),
    (['(in ?a - Vehicle ?c - City)','(at ?a - Car ?l - Location)'],
     ['(in ?V10 - Vehicle ?C20 - City)', '(at ?C10 - Car ?L20 - Location)']),
    (['(in ?a - Vehicle ?c - City)','(in ?a - Location ?l - City)'], ['(in ?O10 - Object ?C20 - City)']),
    (['(in ?a - Car ?c - Location)','(in ?a - Location ?l - City)'], ['(in ?O10 - Object ?L20 - Location)']),
    (['(equal ?a ?b - Car)','(equal ?a ?b - Vehicle)'],['(equal ?V10 ?V11 - Vehicle)']),
    (['(test-pred ?a - Car ?b - City ?c - Car)'],['(test-pred ?C10 - Car ?C20 - City ?C30 - Car)'])

])
def test_merge_predicates(predicates,expected):
    #arrange
    ds = datastore_from_file('./')
    ds.set_available_types(get_type_tree())
    sut = PredicateMerger(ds)
    #act
    merged = sut.merge_predicates(predicates)[0]
    #assert
    for predicate in expected:
        assert predicate in merged

@pytest.mark.parametrize("predicates",[
    (None),
    (['(in ?a - NoType ?c - City)','(in ?a - City ?l - Location)']),
    (['Some','array','not','containing','predicates']),
    (['(in ?noType ?alsoNoType)','(in ?b ?a)']),
    (['(under ?not - closed']),
    (['(at - noVariables)'])

])
def test_merge_predicates_fail(predicates):
    # arrange
    ds = datastore_from_file('./')
    ds.set_available_types(get_type_tree())
    sut = PredicateMerger(ds)
    # assert
    with pytest.raises(ValueError):
        merged = sut.merge_predicates(predicates)




