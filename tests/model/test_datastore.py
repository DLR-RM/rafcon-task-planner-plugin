import pytest
import os
from rafcontpp.model.datastore import datastore_from_file
from rafcontpp.model.datastore import Datastore

@pytest.fixture
def datastore_file_path():
    return os.path.join(os.path.dirname(os.path.abspath(__file__)),'test_data','test_conf.json')

@pytest.fixture
def get_static_test_conf_path():
    return os.path.join(os.path.dirname(os.path.abspath(__file__)),'test_data','static_test_conf.json')

@pytest.fixture
def state_pools():
    return ["My/state/pools"]

@pytest.fixture
def sm_save_dir():
    return "my/sm/save/dir"

@pytest.fixture
def planner():
    return "my_planner"

@pytest.fixture
def planner_script_path():
    return "my_planner_script_path"

@pytest.fixture
def planner_argv():
    return ["my","planner","argv"]

@pytest.fixture
def facts_path():
    return "my_facts_path"

@pytest.fixture
def type_db_path():
    return "my/type/db/path"

@pytest.fixture
def keep_related_files():
    return True

@pytest.fixture
def file_save_dir():
    return "my/file/save/dir"




def test_datastore_from_file():
    #act
    sut = datastore_from_file(get_static_test_conf_path())
    #assert
    assert state_pools() == sut.get_state_pools()
    assert sm_save_dir() == sut.get_sm_save_dir()
    assert planner() == sut.get_planner()
    assert planner_script_path() == sut.get_planner_script_path()
    assert planner_argv() == sut.get_planner_argv()
    assert facts_path() == sut.get_facts_path()
    assert keep_related_files() == sut.keep_related_files()
    assert file_save_dir() == sut.get_file_save_dir()

def test_save_datastore_to_file():
    #arrange
    to_save = Datastore(state_pools(),
                        sm_save_dir(),
                        planner(),
                        planner_script_path(),
                        planner_argv(),
                        facts_path(),
                        type_db_path(),
                        keep_related_files(),
                        file_save_dir())

    #act
    to_save.save_datastore_parts_in_file(datastore_file_path())
    #assert

    sut = datastore_from_file(datastore_file_path())
    os.remove(datastore_file_path())

    # assert
    assert state_pools() == sut.get_state_pools()
    assert sm_save_dir() == sut.get_sm_save_dir()
    assert planner() == sut.get_planner()
    assert planner_script_path() == sut.get_planner_script_path()
    assert planner_argv() == sut.get_planner_argv()
    assert facts_path() == sut.get_facts_path()
    assert keep_related_files() == sut.keep_related_files()
    assert file_save_dir() == sut.get_file_save_dir()



def test_validate_datastore():
    #arrange
     sut = datastore_from_file(get_static_test_conf_path())

    #assert
     with pytest.raises(ValueError):
        sut.validate_ds()




