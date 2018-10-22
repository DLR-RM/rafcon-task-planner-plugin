
from de.dlr.rmc.rafcontpp.model.datastore import DataStore
from de.dlr.rmc.rafcontpp.logic.mapper import Mapper
from de.dlr.rmc.rafcontpp.logic.domain_generator import DomainGenerator

class ExecutionController:


    def __init__(self, datastore):

        self.__datastore = datastore

    def on_execute(self):
        #TODO read inputs first!

        #pipeline, after input reading...
        #prepare dicts
        mapper = Mapper(self.__datastore)
        mapper.generate_action_state_map()
        mapper.generate_state_action_map()
        mapper.generate_available_actions()
        #create domain
        domain_generator = DomainGenerator(self.__datastore)
        domain_generator.generate_domain()

