import pytest
import os
import shutil
from rafcon.planner.plan_module_main import plan_module_main


@pytest.fixture
def domain_name():
    return "argos_domain"

@pytest.fixture
def lib_name():
    return "argos_pool"

@pytest.fixture
def statemachine_name():
    return"a_test_statemachine"

@pytest.fixture
def lib_path():
    return "./test_data/"
@pytest.fixture
def save_path():
     return "./test_data"

@pytest.fixture
def action_db_path():
    return "./test_data/actiondb.json"

@pytest.fixture
def typedict_path():
    return "./test_data/typedb.json"

@pytest.fixture
def domain_path():
    return "./test_data"

@pytest.fixture
def facts_path():
    return "./test_data/argos_facts.pddl"




def test_plan_module_main(lib_name, lib_path, statemachine_name, save_path, action_db_path, typedict_path, domain_name,
                          domain_path, facts_path):
    #arrange
    sm_path = os.path.abspath(os.path.join(save_path, statemachine_name))
    #act
    plan_module_main(lib_name, lib_path, statemachine_name, save_path, action_db_path, typedict_path, domain_name,
                    domain_path, facts_path, False)

    #assert
    assert os.path.isdir(sm_path)
    #tidy up
    shutil.rmtree(sm_path)

def test_plan_module_main_keep_files(lib_name, lib_path, statemachine_name, save_path, action_db_path, typedict_path,
                                    domain_name,domain_path, facts_path):
    #arrange
    sm_path = os.path.abspath(os.path.join(save_path, statemachine_name))
    dom_path = os.path.join(domain_path,domain_name+".pddl")
    sas_path = "./sas_plan"
    out_sas_path = "./output.sas"

    #remove old files.
    if os.path.isfile(sas_path):
        os.remove(sas_path)
    if os.path.isfile(dom_path):
        os.remove(dom_path)
    if os.path.isfile(out_sas_path):
        os.remove(out_sas_path)

    plan_module_main(lib_name, lib_path, statemachine_name, save_path, action_db_path, typedict_path, domain_name,
                     domain_path, facts_path, True)

    #act
    kept_sas_plan = os.path.isfile(sas_path)
    kept_domain_file = os.path.isfile(dom_path)
    kept_out_sas = os.path.isfile(out_sas_path)

    #tidy up
    if kept_sas_plan:
        os.remove(sas_path)
    if kept_domain_file:
        os.remove(dom_path)
    if kept_out_sas:
        os.remove(out_sas_path)
    if os.path.isdir(sm_path):
        shutil.rmtree(sm_path)
    #assert

    assert kept_sas_plan
    assert kept_domain_file
    assert kept_out_sas


def test_plan_module_main_delete_files(lib_name, lib_path, statemachine_name, save_path, action_db_path, typedict_path,
                                     domain_name,domain_path, facts_path):
    #arrange
    sm_path = os.path.abspath(os.path.join(save_path, statemachine_name))
    dom_path = os.path.join(domain_path,domain_name+".pddl")
    sas_path = "./sas_plan"
    out_sas_path = "./output.sas"

    plan_module_main(lib_name, lib_path, statemachine_name, save_path, action_db_path, typedict_path, domain_name,
                     domain_path, facts_path, False)
    #act
    kept_sas_plan = os.path.isfile(sas_path)
    kept_domain_file = os.path.isfile(dom_path)
    kept_out_sas = os.path.isfile(out_sas_path)

    #tidy up
    if kept_sas_plan:
        os.remove(sas_path)
    if kept_domain_file:
        os.remove(dom_path)
    if kept_out_sas:
        os.remove(out_sas_path)
    if os.path.isdir(sm_path):
        shutil.rmtree(sm_path)
    #assert

    assert not kept_sas_plan
    assert not kept_domain_file
    assert not kept_out_sas

@pytest.mark.parametrize("li_name,l_path,sm_name, s_path, actdb_path, typdict_path,dom_name,dom_path, fac_path",[

    (lib_name(), "", statemachine_name(), save_path(), action_db_path(), typedict_path(), domain_name(),
     domain_path(), facts_path()),
    (lib_name(), lib_path(), statemachine_name(), "", action_db_path(), typedict_path(), domain_name(),
     domain_path(), facts_path()),
    (lib_name(), lib_path(), statemachine_name(), save_path(), "", typedict_path(), domain_name(),
     domain_path(), facts_path()),
    (lib_name(), lib_path(), statemachine_name(), save_path(), action_db_path(), "", domain_name(),
     domain_path(), facts_path()),
    (lib_name(), lib_path(), statemachine_name(), save_path(), action_db_path(), typedict_path(), domain_name(),
     "", facts_path()),
    (lib_name(), lib_path(), statemachine_name(), save_path(), action_db_path(), typedict_path(), domain_name(),
     domain_path(), "")
])
def test_plan_module_main_wrong_input(li_name,l_path,sm_name, s_path, actdb_path, typdict_path,dom_name,dom_path, fac_path):
    with pytest.raises(ValueError):
        plan_module_main(lib_name, l_path, sm_name, s_path, actdb_path, typdict_path, dom_name,
                         dom_path, fac_path, False)