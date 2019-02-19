import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk
from gi.repository import GObject
from rafcontpp.view.planning_setup_form import PlanningSetupForm
from rafcontpp.model.datastore import datastore_from_file, DATASTORE_STORAGE_PATH
from rafcon.gui.helpers.label import create_label_widget_with_icon
from rafcon.gui.views.tool_bar import ToolBarView
import threading
import rafcon.gui.utils
import rafcon.gui.helpers.state_machine
import rafcon.gui.singleton
from rafcon.utils import log
logger = log.get_logger(__name__)
plan_task_label = "Plan Task"
tool_tip_text = "Open planning Configuration"
progress_text = " Task(s) in Progress."
plan_sm_button = None
button_counter = 0
lock = threading.Lock()
def initialize():
    tool_bar_ctrl = rafcon.gui.singleton.main_window_controller.get_controller('tool_bar_controller')
    rafcon.gui.utils.wait_for_gui()
    assert isinstance(tool_bar_ctrl.view, ToolBarView)

    # add new button
    global plan_sm_button
    plan_sm_button = Gtk.ToolButton(label='Plan Task')
    plan_sm_button.set_label_widget(create_label_widget_with_icon('f1ec', _(plan_task_label),tool_tip_text))
    plan_sm_button.set_stock_id(Gtk.STOCK_CLEAR)
    tool_bar_ctrl.view.get_top_widget().add(plan_sm_button)
    plan_sm_button.show_all()
    plan_sm_button.connect('clicked', __on_button_clicked)


def increment_button():
    with lock:
        global button_counter
        button_counter += 1
        plan_sm_button.set_label_widget(create_label_widget_with_icon('f1ec',
                                        _(plan_task_label + ' ({})'.format(button_counter)),
                                        tool_tip_text +'\n'+ str(button_counter)+progress_text))

def decrement_button():
    with lock:
        global button_counter
        button_counter -= 1
        if button_counter <= 0:
            button_counter = 0
            plan_sm_button.set_label_widget(create_label_widget_with_icon('f1ec', _(plan_task_label), tool_tip_text))
        else:
            plan_sm_button.set_label_widget(
                create_label_widget_with_icon('f1ec', _(plan_task_label + ' ({})'.format(button_counter)),
                                              tool_tip_text +'\n'+ str(button_counter)+progress_text))



def __on_button_clicked(*args):
    logger.debug('opening planning form!')
    PlanningSetupForm(datastore_from_file(DATASTORE_STORAGE_PATH)).initialize()