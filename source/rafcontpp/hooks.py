#hooks.py
#hooks.py is the plugins main file. These methods will be called from rafcon.
#
# Contributors:
# Christoph Suerig <christoph.suerig@dlr.de>
# Version 12.11.2018
import os
import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk
from rafcontpp.view import planning_button
from rafcontpp.control.pddl_action_tab_controller import PddlActionTabController
from rafcon.gui.helpers.label import create_label_widget_with_icon, create_tab_header_label
from rafcon.core.states.library_state import LibraryState
from rafcon.utils import log
logger = log.get_logger(__name__)


def pre_init():
    """ The pre_init function of the auto layout plugin. Currently method refresh selected state machine is used to
    trigger a auto layout.
    :return:
    """
    logger.info("Run pre-initiation hook of {0} plugin.".format(__file__.split(os.path.sep)[-2]))


def main_window_setup(main_window_controller):
    '''
    called on window setup.
    :param main_window_controller:
    :return:
    '''
    logger.info("Run main window setup of {0} plugin.".format(__file__.split(os.path.sep)[-2]))
    #add the plan task button to rafcons menu bar.
    planning_button.initialize()




def post_init(*args, **kwargs):
    """
    The post_init function of the execution hooks plugin. The observer of the execution hooks manager are initialized
    and hooks execution is enabled.
    :return:
    """
    logger.info("Run post-initiation hook of {0} plugin".format(__file__.split(os.path.sep)[-2]))


def post_state_editor_register_view(state_editor):
    '''
    called every time, a State Editor is created.
    adds the action tab.
    :param state_editor:
    :return:
    '''

    state_editor_view = state_editor.view
    state = state_editor.model.state
    glade_path = os.path.abspath(
        os.path.join(os.path.dirname(os.path.realpath(__file__)),"view","glade", "pddl_action_tab.glade"))
    gtk_builder = Gtk.Builder()
    #load action tab
    gtk_builder.add_from_file(glade_path)
    # get items
    tab = gtk_builder.get_object('rtpp_action_box')
    tab.show_all()
    #add tab, with lable to State Editor
    state_editor_view["main_notebook_2"].append_page(
        tab, create_label_widget_with_icon('f1ec', _(''),'PDDL Action definition'))
    state_editor_view["main_notebook_2"].set_tab_reorderable(tab, True)
    #init add listener functions, and initiate tab data in controler
    PddlActionTabController(gtk_builder, state).start_control_tab()