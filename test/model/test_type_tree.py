import pytest
from rafcontpp.model.type_tree import TypeTree


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
    ("some_type", "some_type", "some_type", True)
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
    #add usual branch, in this case the whole dict.
    ("Object","Object","Roadstar",True),
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
    assert expected == inserted
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

@pytest.mark.parametrize("parent,child,expected",[
    ('Object','Car',True),
    ('Location','Spot',True),
    ('Location','Location',False),
    ('Location','Object',False),
    ('Location','Not_in_tree',False),
    ('Not_in_tree','Location',False),
    ('Not_in_tree','Not_in_tree',False),
    ('Car',None,False),
    (None,'Car',False),
])
def test_is_parent_of(parent,child,expected):
    #prepare
    tree = TypeTree('Object')
    tree.add_type_branch('Roadstar',filled_type_dict())
    tree.add_type_branch('Spot',filled_type_dict())
    #act
    is_parent = tree.is_parent_of(parent,child)
    #assert
    assert expected == is_parent


def test_type_tree_to_list():
    #prepare
    tree = TypeTree('Object')
    tree.add_type_branch('Object',filled_type_dict())
    list = []
    #act
    list = tree.get_as_list()
    #assert
    assert 'Object' in list
    for key in filled_type_dict().keys():
        assert key in list
    assert len(filled_type_dict().keys()) + 1 == len(list)
