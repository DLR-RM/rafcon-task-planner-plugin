import pytest
import os
from rafcontpp.logic.predicate_merger import PredicateMerger
from rafcontpp.model.type_tree import TypeTree
from rafcontpp.model.datastore import datastore_from_file


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
    (['(in ?a - Vehicle ?c - City)','(in ?a - Vehicle ?c - City)'], ['(in ?V00 - Vehicle ?C10 - City)']),
    (['(in ?a - Car ?c - City)','(in ?a - Vehicle ?l - Location)'], ['(in ?V00 - Vehicle ?L10 - Location)']),
    (['(in ?a - Vehicle ?c - City)','(in ?a - Car ?l - Location)'], ['(in ?V00 - Vehicle ?L10 - Location)']),
    (['(in ?a - Vehicle ?c - City)','(at ?a - Car ?l - Location)'],
     ['(in ?V00 - Vehicle ?C10 - City)', '(at ?C00 - Car ?L10 - Location)']),
    (['(in ?a - Vehicle ?c - City)','(in ?a - Location ?l - City)'], ['(in ?V00 - Object ?L10 - City)'])



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
    (['(in ?a - Vehicle ?c - City)','(in ?a - City ?l - Location)']),
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




