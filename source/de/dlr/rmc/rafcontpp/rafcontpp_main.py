
from de.dlr.rmc.rafcontpp.controll.execution_controller import ExecutionController
from de.dlr.rmc.rafcontpp.model.datastore import Datastore, datastore_from_file


def rafcontpp_main():
    state_pools = ['/home/suer_ch/USERSTORE/Rafcon_Planning_Testmaterial/pools/argos_pool/']
    action_pools = ['/home/suer_ch/USERSTORE/Rafcon_Planning_Testmaterial/Dbs/actiondb.json']
    sm_save_dir = '/home/suer_ch/USERSTORE/Rafcon_Planning_Testmaterial/Planned_Statemachines'
    planner = '/volume/USERSTORE/suer_ch/Rafcon_Planning_Testmaterial/planner_integrations/ffintegration.py'
    planner_argv= []
    facts_path = '/home/suer_ch/USERSTORE/Rafcon_Planning_Testmaterial/PDDL_Facts/argos_facts.pddl'
    type_db_path = '/home/suer_ch/USERSTORE/Rafcon_Planning_Testmaterial/Dbs/typedb.json'
    keep_related_files = True
    file_save_dir = '/home/suer_ch/USERSTORE/Rafcon_Planning_Testmaterial/related_files'

    #ds = Datastore(state_pools, action_pools,sm_save_dir, planner, planner_argv, facts_path,type_db_path,keep_related_files, file_save_dir)
    savePath = '/home/suer_ch/USERSTORE/Rafcon_Planning_Testmaterial/Dbs/rafcontpp_conf.json'
    ds = datastore_from_file(savePath)
    excon = ExecutionController(ds)
    excon.on_execute()
    ds.save_datastore_parts_in_file(savePath)

rafcontpp_main()