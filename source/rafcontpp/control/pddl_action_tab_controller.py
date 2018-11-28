import re
import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk
from gi.repository import GObject
from rafcontpp.model.datastore import SEMANTIC_DATA_DICT_NAME
from rafcontpp.logic.pddl_action_parser import PddlActionParser
from rafcon.core.states.library_state import LibraryState
from rafcon.utils import log

logger = log.get_logger(__name__)
# map of glade item ids with pddl requirements
id_requ_map = {
    'rtpp_action_strips_box': ':strips',
    'rtpp_action_adl_box': ':adl',
    'rtpp_action_typing_box': ':typing',
    'rtpp_action_equality_box': ':equality',
    'rtpp_action_neg_prec_box': ':negative-preconditions',
    'rtpp_action_dis_prec_box': ':disjunctive-preconditions',
    'rtpp_action_cond_eff_box': ':conditional-effects',
    'rtpp_action_ex_prec_box': ':existential-preconditions',
    'rtpp_action_uni_prec_box': ':universal-preconditions',
    'rtpp_action_deri_pred_box': ':derived-predicates',
    'rtpp_action_action_costs_box': ':action-costs',
    'rtpp_action_quanti_precon_box': ':quantified-preconditions',
    'rtpp_action_action_expa_box': ':action-expansions',
    'rtpp_action_foreach_expa_box': ':foreach-expansions',
    'rtpp_action_dag_exp_box': ':dag-expansions',
    'rtpp_action_expr_eval_box': ':expression-evaluation',
    'rtpp_action_fluents_box': ':fluents'

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
        view_port = self.__gtk_builder.get_object('test_view_port')
        #__requ_bb_dict contains all requirements button boxes
        self.__requ_bb_dict = self.__add_requirements_boxes(view_port)


    def __add_requirements_boxes(self, gtk_viewport):

        button_dict = {} #key: id value: (label,checkButtonObject)
        grid = Gtk.Grid()
        grid.insert_row(0)
        grid.insert_column(0)
        grid.insert_column(0)

        row_counter = 1
        column_counter = 0
        for key in id_requ_map:


            check_button = Gtk.CheckButton.new_with_label(id_requ_map[key])
            button_dict[key] = (id_requ_map[key], check_button)
            grid.attach(check_button, column_counter % 3, row_counter - 1, 1, 1)
            column_counter += 1

            if column_counter % 3 == 0:
                grid.insert_row(row_counter)
                row_counter += 1


        gtk_viewport.add(grid)
        grid.show_all()
        return button_dict




    def start_control_tab(self):

        self.__load_from_semantic_section()
        if isinstance(self.__state, LibraryState):
            self.__description_text_view.set_editable(False)
            self.__description_text_view.set_cursor_visible(False)
            self.__pddl_action_source_view.set_editable(False)
            self.__pddl_action_source_view.set_cursor_visible(False)
            self.__pddl_predicates_text_view.set_editable(False)
            self.__pddl_predicates_text_view.set_cursor_visible(False)
            self.__pddl_types_text_view.set_editable(False)
            self.__pddl_types_text_view.set_cursor_visible(False)
            for item_id in id_requ_map.keys():
                self.__gtk_builder.get_object(item_id).set_enabled(False)
        else:
            #observe parts
            auto_fill_button = self.__gtk_builder.get_object('rtpp_pddl_tab_auto_fill_button')
            auto_fill_button.connect('clicked', self.__auto_fill)
            self.__description_text_view.get_buffer().connect('changed',self.__save_data,'description')
            self.__pddl_action_source_view.get_buffer().connect('changed', self.__save_data,'pddl_action')
            self.__pddl_predicates_text_view.get_buffer().connect('changed', self.__save_data,'pddl_predicates')
            self.__pddl_types_text_view.get_buffer().connect('changed', self.__save_data, 'pddl_types')

            for item_id in id_requ_map.keys():
                self.__gtk_builder.get_object(item_id).connect('toggled',self.__save_requirements)



    def __load_from_semantic_section(self):
        rtpp_dict = self.__state.semantic_data[SEMANTIC_DATA_DICT_NAME]
        self.__description_text_view.get_buffer().set_text(self.__filter_input(str(rtpp_dict['description'])))

        source_view_string = self.__filter_input(str(rtpp_dict['pddl_action']))
        if len(source_view_string) == 0:
            source_view_string = '\n\n\n\n'
        self.__pddl_action_source_view.get_buffer().set_text(source_view_string)
        self.__pddl_predicates_text_view.get_buffer().set_text(self.__filter_input(str(rtpp_dict['pddl_predicates'])))
        self.__pddl_types_text_view.get_buffer().set_text(self.__filter_input(str(rtpp_dict['pddl_types'])))
        for key in id_requ_map.keys():
            self.__gtk_builder.get_object(key).set_active(id_requ_map[key] in rtpp_dict['requirements'])
            


    def __save_data(self, buffer, key):
        start, end = buffer.get_bounds()
        self.__state.semantic_data[SEMANTIC_DATA_DICT_NAME][key] = buffer.get_text(start, end,True)
        sekf.__state.s

    def __save_requirements(self,checkbox):
        self.__state.semantic_data[SEMANTIC_DATA_DICT_NAME]['requirements'] = str(self.__get_requirements())

    def __get_requirements(self):

        requirements = []
        for key in id_requ_map.keys():
            if self.__gtk_builder.get_object(key).get_active():
                requirements.append(id_requ_map[key])
        return requirements


    def __auto_fill(self,button):
        #get pddl action from source view
        buffer = self.__pddl_action_source_view.get_buffer()
        start, end = buffer.get_bounds()
        raw_action = buffer.get_text(start, end, True)
        pddl_action = PddlActionParser(raw_action).parse_action()

        self.__predicates_auto_fill(pddl_action)
        self.__types_auto_fill(pddl_action)
        self.__requirements_auto_fill(raw_action)




    def __requirements_auto_fill(self, raw_action):

        raw_action = raw_action.upper()
        bev_effects = raw_action[:raw_action.find(':EFFECT')]

        if raw_action.find(' - ') > -1:
            self.__gtk_builder.get_object('rtpp_action_typing_box').set_active(True)

        if raw_action.find('=') > -1:
            self.__gtk_builder.get_object('rtpp_action_equality_box').set_active(True)

        if re.search('\(\s*(WHEN)[\s|\(]',raw_action):
            self.__gtk_builder.get_object('rtpp_action_cond_eff_box').set_active(True)
            self.__gtk_builder.get_object('rtpp_action_adl_box').set_active(True)

        if re.search('\(\s*(FORALL)[\s|\(]', raw_action):
            self.__gtk_builder.get_object('rtpp_action_adl_box').set_active(True)

        if re.search('\(\s*(AND)[\s|\(]',bev_effects):
            self.__gtk_builder.get_object('rtpp_action_strips_box').set_active(True)

        if re.search('\(\s*(NOT)[\s|\(]',bev_effects):
            self.__gtk_builder.get_object('rtpp_action_neg_prec_box').set_active(True)

        if re.search('\(\s*(OR)[\s|\(]',bev_effects):
            self.__gtk_builder.get_object('rtpp_action_dis_prec_box').set_active(True)
            self.__gtk_builder.get_object('rtpp_action_adl_box').set_active(True)

        #TODO more autofill
        self.__save_requirements(self.__gtk_builder.get_object('rtpp_action_dis_prec_box'))



    def __types_auto_fill(self,pddl_action):

        pattern = re.compile('-{1}\s+[^\s\)]*')
        types = pddl_action.types

        #merge add unknown
        types_buffer = self.__pddl_types_text_view.get_buffer()
        start, end = types_buffer.get_bounds()
        type_field = types_buffer.get_text(start, end, True)
        #to be able to compare
        upper_type_field = type_field.upper()
        for type in types:
            if upper_type_field.find(type.upper()) == -1:
                type_field = type_field + ", "+type

        #set type field.
        types_buffer.set_text(type_field.strip(',').strip())
        self.__save_data(types_buffer, 'pddl_types')



    def __predicates_auto_fill(self,pddl_action):

        found_predicates = pddl_action.predicates


        # merge add unknown
        start, end = self.__pddl_predicates_text_view.get_buffer().get_bounds()
        pred_field = self.__pddl_predicates_text_view.get_buffer().get_text(start, end, True)
        upper_pred = pred_field.upper()

        for fpred in found_predicates:
            if upper_pred.find(fpred.upper()) == -1:
                pred_field = pred_field +'\r\n' + fpred
                upper_pred = pred_field.upper()


        pred_field = pred_field.strip('\r\n')
        self.__pddl_predicates_text_view.get_buffer().set_text(pred_field)
        self.__save_data(self.__pddl_predicates_text_view.get_buffer(),'pddl_predicates')




    def __filter_input(self,input):
        if input == '{}':
            input = ''
        return input

