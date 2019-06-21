import pytest
import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk
from rafcontpp.logic.pddl_action_parser import PddlActionParser
from rafcontpp.control.pddl_action_tab_controller import PddlActionTabController



def get_requ_dict():
    requ_list = [':strips', ':adl', ':typing', ':equality',
                 ':negative-preconditions', ':disjunctive-preconditions', ':negative-preconditions',
                 ':conditional-effects', ':existential-preconditions',
                 ':universal-preconditions', ':derived-predicates',
                 ':action-costs', ':quantified-preconditions',
                 ':action-expansions', ':foreach-expansions',
                 ':dag-expansions', ':expression-evaluation', ':fluents', ':open-world', ':true-negation',
                 'durative-actions', ':duration-inequalities', ':continous-effects']
    button_dict = {}
    for requirement in requ_list:
        check_button = Gtk.CheckButton.new_with_label(requirement)
        button_dict[requirement] = check_button
    return button_dict




@pytest.mark.parametrize('pddl_action, expected_pred,expected_types, expected_requs',[
    (PddlActionParser("(:ACTION KILL"
                      ":PARAMETERS ( ?TURTLE - TURTLE)"
                      ":PRECONDITION (ALIVE ?TURTLE)"
                      ":EFFECT (AND (DEAD ?TURTLE)(NOT (ALIVE ?TURTLE))))").parse_action(),
     "(ALIVE ?TURTLE - TURTLE)\r\n(DEAD ?TURTLE - TURTLE)",
     "TURTLE",
     [":typing",":strips"]
     ),
    (PddlActionParser("(:action move"
                      ":parameters (?from ?destination - Location ?turtle - Turtle)"
                      ":precondition (and "
                      "(at ?from ?turtle) (alive ?turtle)(not(= ?from ?destination))"
                      "(or(track ?from ?destination)(track ?destination ?from))"
                      ":effect (and (not (at ?from ?turtle)) (at ?destination ?turtle) (was-at ?from ?turtle)"
                      "(hungry ?turtle)))").parse_action(),
     "(was-at ?from - Location ?turtle - Turtle)\r\n"
     "(alive ?turtle - Turtle)\r\n"
     "(at ?destination - Location ?turtle - Turtle)\r\n"
     "(track ?destination ?from - Location)\r\n"
     "(hungry ?turtle - Turtle)",
     "Turtle, Location",
     [":typing",":strips",":equality",":disjunctive-preconditions"]
    )
])

def test_auto_complete(pddl_action, expected_pred,expected_types, expected_requs):
    #arrange
    sut = PddlActionTabController(None)
    pred_buf = Gtk.TextBuffer()
    types_buf = Gtk.TextBuffer()
    requ_dict = get_requ_dict()
    #act
    sut.auto_complete(Gtk.Button(),pred_buf,types_buf,requ_dict,pddl_action)
    #assert
    start, end = pred_buf.get_bounds()
    assert expected_pred == pred_buf.get_text(start, end, True)
    start, end = types_buf.get_bounds()
    assert expected_types == types_buf.get_text(start, end, True)
    for requ in requ_dict:
        assert ((requ in expected_requs) == requ_dict[requ].get_active())
