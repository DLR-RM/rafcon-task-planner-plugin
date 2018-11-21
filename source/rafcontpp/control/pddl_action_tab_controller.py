from rafcontpp.model.datastore import SEMANTIC_DATA_DICT_NAME
from rafcon.utils import log

logger = log.get_logger(__name__)
# map of glade item ids with pddl requirements
id_requ_map = {
    'rtpp_action_strips_box': ':strips',
    'rtpp_action_adl_box': ':adl',
    'rtpp_action_typing_box': ':typing',
    'rtpp_action_equality_box': ':equality',
    'rtpp_action_neg_prec_box': ':negative-preconditions',
    'rtpp_action_dis_prec_box': ':disjunctive-predonditions',
    'rtpp_action_cond_eff_box': ':conditional-effects',
    'rtpp_action_ex_prec_box': ':existential-preconditions',
    'rtpp_action_uni_prec_box': ':universal-preconditions',
    'rtpp_action_deri_pred_box': ':derived-predicates',
    'rtpp_action_action_costs_box': ':action-costs'
}


class PddlActionTabController:



    def __init__(self,action_tab_gtk_builder, state):
        self.__state = state
        self.__gtk_builder = action_tab_gtk_builder
        self.__description_text_view = self.__gtk_builder.get_object('description_textview')
        self.__description_text_view = self.__gtk_builder.get_object('description_textview')
        self.__pddl_action_source_view = self.__gtk_builder.get_object('pddl_action_sourceview')
        self.__pddl_predicates_text_view = self.__gtk_builder.get_object('pddl_predicates_textview')
        self.__pddl_types_text_view = self.__gtk_builder.get_object('pddl_types_textview')


    def start_control_tab(self):

        self.__load_from_semantic_section()

        #observe parts
        self.__description_text_view.get_buffer().connect('changed', self.__save_data,'description')
        self.__pddl_action_source_view.get_buffer().connect('changed', self.__requirements_auto_fill)
        self.__pddl_action_source_view.get_buffer().connect('changed', self.__types_auto_fill)
        self.__pddl_action_source_view.get_buffer().connect('changed', self.__save_data,'pddl_action')
        self.__pddl_predicates_text_view.get_buffer().connect('changed', self.__save_data,'pddl_predicates')
        self.__pddl_types_text_view.get_buffer().connect('changed', self.__save_data, 'pddl_types')
        for item_id in id_requ_map.keys():
            self.__gtk_builder.get_object(item_id).connect('toggled',self.__save_requirements)



    def __load_from_semantic_section(self):
        rtpp_dict = self.__state.semantic_data[SEMANTIC_DATA_DICT_NAME]
        self.__description_text_view.get_buffer().set_text(self.__filter_input(str(rtpp_dict['description'])))
        self.__pddl_action_source_view.get_buffer().set_text(self.__filter_input(str(rtpp_dict['pddl_action'])))
        self.__pddl_predicates_text_view.get_buffer().set_text(self.__filter_input(str(rtpp_dict['pddl_predicates'])))
        self.__pddl_types_text_view.get_buffer().set_text(self.__filter_input(str(rtpp_dict['pddl_types'])))
        for key in id_requ_map.keys():
            self.__gtk_builder.get_object(key).set_active(id_requ_map[key] in rtpp_dict['requirements'])


    def __save_data(self, buffer, key):
        start, end = buffer.get_bounds()
        self.__state.semantic_data[SEMANTIC_DATA_DICT_NAME][key] = buffer.get_text(start, end,True)

    def __save_requirements(self,checkbox):
        self.__state.semantic_data[SEMANTIC_DATA_DICT_NAME]['requirements'] = str(self.__get_requirements())

    def __get_requirements(self):

        requirements = []
        for key in id_requ_map.keys():
            if self.__gtk_builder.get_object(key).get_active():
                requirements.append(id_requ_map[key])
        return requirements



    def __requirements_auto_fill(self,buffer):
        start, end = buffer.get_bounds()
        raw_action = buffer.get_text(start, end,True).upper()
        bev_effects = raw_action[:raw_action.find(':EFFECT')]
        if raw_action.find(' - ') > -1:
            self.__gtk_builder.get_object('rtpp_action_typing_box').set_active(True)

        if raw_action.find('(WHEN ') > -1:
            self.__gtk_builder.get_object('rtpp_action_cond_eff_box').set_active(True)
            self.__gtk_builder.get_object('rtpp_action_adl_box').set_active(True)

        if bev_effects.find('(AND ') > -1:
            self.__gtk_builder.get_object('rtpp_action_strips_box').set_active(True)

        if bev_effects.find('(NOT ') > -1:
            self.__gtk_builder.get_object('rtpp_action_neg_prec_box').set_active(True)

        if bev_effects.find('(OR ') > -1:
            self.__gtk_builder.get_object('rtpp_action_dis_prec_box').set_active(True)
            self.__gtk_builder.get_object('rtpp_action_adl_box').set_active(True)

        #TODO more autofill
        self.__save_requirements(self.__gtk_builder.get_object('rtpp_action_dis_prec_box'))



    def __types_auto_fill(self,buffer):
        start, end = buffer.get_bounds()
        raw_action = buffer.get_text(start, end, True).upper()
        action = raw_action
        types = []

        while action.find(' - ') > -1:
            substr = action[action.find(' - ')+3:].strip()
            action = substr
            types.append(substr[:substr.find(' ')].replace(')', ''))

        #merge add unknown
        start, end =  self.__pddl_types_text_view.get_buffer().get_bounds()
        type_field =  self.__pddl_types_text_view.get_buffer().get_text(start, end, True)
        #to be able to compare
        upper_type_field = type_field.upper()
        for type in types:
            if upper_type_field.find(type.upper()) == -1:
                type_field = type_field + ", "+type

        #set type field.
        self.__pddl_types_text_view.get_buffer().set_text(type_field.strip(',').strip())


    def __filter_input(self,input):
        if input == '{}':
            input = ''
        return input

