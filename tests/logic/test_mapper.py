import pytest
import os
import utils
from utils import call_gui_callback
from rafcontpp.logic.mapper import Mapper
from rafcontpp.model.datastore import datastore_from_file
@pytest.fixture
def datastore():
     base_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'test_data')
     facts_path = os.path.join(base_path, 'test_facts.pddl')
     type_db_path = os.path.join(base_path, 'test_type_db.json')
     file_save_dir = base_path
     state_pool_path = os.path.join(base_path, 'test_state_pool')
     ds = datastore_from_file('')
     ds.set_facts_path(facts_path)
     ds.set_type_db_path(type_db_path)
     ds.set_file_save_dir(file_save_dir)
     ds.add_state_pools([state_pool_path],True)
     return ds

@pytest.fixture
def action_state_map():
     return {'COOK': 'cook', 'ORDER': 'order', 'EAT': 'eat', 'GOT-TO': 'go_to', 'GIVE': 'give'}


@pytest.fixture
def state_action_map():
     return {'cook': 'COOK', 'go_to': 'GOT-TO', 'give': 'GIVE', 'order': 'ORDER', 'eat': 'EAT'}

@pytest.fixture
def available_actions():
     return ['COOK', 'ORDER', 'EAT', 'GOT-TO', 'GIVE']

def test_generate_action_state_map():
     #arrange
     ds = datastore()
     mapper = Mapper(ds)
     utils.run_gui(gui_config={'HISTORY_ENABLED': False, 'AUTO_BACKUP_ENABLED': False})
     # act

     try:
          call_gui_callback(mapper.generate_action_state_map)
          assert action_state_map() == ds.get_action_state_map()
     finally:
          testing_utils.close_gui()
          testing_utils.shutdown_environment(caplog=None)

def test_generate_state_action_map():
     #arrange
     ds = datastore()
     mapper = Mapper(ds)
     utils.run_gui(gui_config={'HISTORY_ENABLED': False, 'AUTO_BACKUP_ENABLED': False})
     # act

     try:
          call_gui_callback(mapper.generate_state_action_map)
          assert state_action_map() == ds.get_state_action_map()
     finally:
          testing_utils.close_gui()
          testing_utils.shutdown_environment(caplog=None)

def test_generate_available_actions():
     #arrange
     ds = datastore()
     mapper = Mapper(ds)
     utils.run_gui(gui_config={'HISTORY_ENABLED': False, 'AUTO_BACKUP_ENABLED': False})
     # act

     try:
          call_gui_callback(mapper.generate_available_actions)
          assert available_actions() == ds.get_available_actions()
     finally:
          testing_utils.close_gui()
          testing_utils.shutdown_environment(caplog=None)