import pytest
import os
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


#NOT TESTABLE AT THE MOMENT, BECAUSE RAFCON HAS TO BE UP FOR THIS TEST TO WORK...
def test_generate_action_state_map():
     #arrange
     ds = datastore()
     mapper = Mapper(ds)
     #act
     #mapper.generate_action_state_map()
