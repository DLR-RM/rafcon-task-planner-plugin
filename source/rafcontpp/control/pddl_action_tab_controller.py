


SEMANTIC_DATA_DICT_NAME = 'RAFCONTPP_PDDL_ACTION'

class PddlActionTabController:


    def __init__(self,action_tab_gtk_builder, state):
        self.__state = state
        self.__gtk_builder = action_tab_gtk_builder
        self.__action_name_text_view = self.__gtk_builder.get_object('pddl_action_name_textview')
        self.__description_text_view = self.__gtk_builder.get_object('description_textview')
        self.__description_text_view = self.__gtk_builder.get_object('description_textview')
        self.__pddl_action_source_view = self.__gtk_builder.get_object('pddl_action_sourceview')
        self.__pddl_predicates_text_view = self.__gtk_builder.get_object('pddl_predicates_textview')
        self.__pddl_types_text_view = self.__gtk_builder.get_object('pddl_types_textview')
        # TODO requirements!


    def start_control_tab(self):

        self.__load_from_semantic_section()

        #observe parts
        self.__action_name_text_view.get_buffer().connect('changed', self.__save_data,'action_name')
        self.__description_text_view.get_buffer().connect('changed', self.__save_data,'description')
        self.__pddl_action_source_view.get_buffer().connect('changed', self.__save_data,'pddl_action')
        self.__pddl_predicates_text_view.get_buffer().connect('changed', self.__save_data,'pddl_predicates')
        self.__pddl_types_text_view.get_buffer().connect('changed', self.__save_data, 'pddl_types')



    def __load_from_semantic_section(self):
        rtpp_dict = self.__state.semantic_data[SEMANTIC_DATA_DICT_NAME]
        self.__action_name_text_view.get_buffer().set_text(self.__filter_input(str(rtpp_dict['action_name'])))
        self.__description_text_view.get_buffer().set_text(self.__filter_input(str(rtpp_dict['description'])))
        self.__pddl_action_source_view.get_buffer().set_text(self.__filter_input(str(rtpp_dict['pddl_action'])))
        self.__pddl_predicates_text_view.get_buffer().set_text(self.__filter_input(str(rtpp_dict['pddl_predicates'])))
        self.__pddl_types_text_view.get_buffer().set_text(self.__filter_input(str(rtpp_dict['pddl_types'])))
        #TODO requirments!


    def __save_data(self, buffer, key):
        start, end = buffer.get_bounds()
        print "saving: "+buffer.get_text(start, end,True) +" under: "+key
        self.__state.semantic_data[SEMANTIC_DATA_DICT_NAME][key] = buffer.get_text(start, end,True)


    def __filter_input(self,input):
        if input == '{}':
            input = ''
        return input

