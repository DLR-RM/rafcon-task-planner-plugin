import os

import pytest
import utils
from utils import call_gui_callback

from rafcontpp.control.execution_controller import ExecutionController
from rafcontpp.model.datastore import datastore_from_file


@pytest.fixture
def datastore():
    base_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'test_data')
    ds_conf_path = os.path.join(base_path, 'rafcontpp_test_conf.json')
    ds = datastore_from_file(ds_conf_path)
    ds.set_sm_save_dir(base_path)
    ds.set_file_save_dir(base_path)
    test_state_pool_path = os.path.join(base_path,'test_state_pool')
    ds.add_state_pools(test_state_pool_path,False)
    type_db_path = os.path.join(base_path,'rtpp_test_typehierarchy.json')
    ds.set_type_db_path(type_db_path)
    facts_path = os.path.join(base_path,'test_facts.pddl')
    ds.set_facts_path(facts_path)
    ds.set_keep_related_files(False)
    return ds

def test_on_execute_pre_planning():
    #arrange
    plugin_path = os.path.dirname(os.path.abspath(__file__))
    plugin_path = plugin_path[:plugin_path.rfind('/')]
    plugin_path = plugin_path[:plugin_path.rfind('/')]
    plugin_path = os.path.join(plugin_path,'source','rafcontpp')
    os.environ['RAFCON_PLUGIN_PATH'] = plugin_path

    utils.run_gui(gui_config={'HISTORY_ENABLED': False, 'AUTO_BACKUP_ENABLED': False})
    sut = ExecutionController(datastore())
    #act
    try:
        planning_thread = call_gui_callback(sut.on_execute_pre_planning)
        planning_thread.join()
    finally:
        utils.close_gui()
        utils.shutdown_environment(caplog=None)