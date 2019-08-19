import os

import pytest

from rafcontpp.control.planning_controller import PlanningController
from rafcontpp.model.datastore import datastore_from_file


def datastore():
     base_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'test_data')
     facts_path = os.path.join(base_path, 'test_facts.pddl')
     domain_path = os.path.join(base_path, 'restaurant_domain_static.pddl')
     file_save_dir = base_path
     ds = datastore_from_file('')
     ds.set_facts_path(facts_path)
     ds.set_domain_path(domain_path)
     ds.set_file_save_dir(file_save_dir)
     ds.set_planner('Fast Downward Planning System')
     return ds


def test_execute_planning_built_in_script():
    #arrange
    ds = datastore()
    sut = PlanningController(ds)
    #act

    def call_back(planning_successful):
        for file in ds.get_generated_files():
            c_file = os.path.join(ds.get_file_save_dir(),file)
            if os.path.isfile(c_file):
                os.remove(c_file)
        #assert
        assert ['sas_plan', 'output.sas'] == ds.get_generated_files()
        assert planning_successful

    sut.execute_planning(call_back).join()

def test_execute_planning_custom_script():
    #arrange
    ds = datastore()
    ds.set_planner(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'test_data','custom_script.py'))
    sut = PlanningController(ds)
    # act


    def call_back(planning_successful):
        for file in ds.get_generated_files():
            c_file = os.path.join(ds.get_file_save_dir(), file)
            if os.path.isfile(c_file):
                os.remove(c_file)
            # assert
        assert ['sas_plan', 'output.sas'] == ds.get_generated_files()
        assert planning_successful

    sut.execute_planning(call_back).join()

def test_execute_planning_none_script():
    #arrange
    ds = datastore()
    ds.set_planner(None)
    sut = PlanningController(ds)
    def call_back(something):
        something = False
        assert False
    #act
    with pytest.raises(ImportError):
        sut.execute_planning(call_back).join()

def test_execute_planning_empty_script_path():
    #arrange
    ds = datastore()
    ds.set_planner('')
    sut = PlanningController(ds)
    #act
    def call_back(something):
        something = False
        assert False
    with pytest.raises(ImportError):
        sut.execute_planning(call_back).join()

def test_execute_planning_not_existing_script():
    #arrange
    ds = datastore()
    ds.set_planner('/this/path/does/not/exist/no_script.py')
    sut = PlanningController(ds)
    #act
    def call_back(something):
        something = False
        assert False
    with pytest.raises(ImportError):
        sut.execute_planning(call_back).join()