import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk
from gi.repository import GObject

from rafcontpp.view.planning_setup_form import PlanningSetupForm
from rafcontpp.model.datastore import datastore_from_file, DATASTORE_STORAGE_PATH
from rafcon.gui.helpers.label import create_label_widget_with_icon
from rafcon.gui.views.tool_bar import ToolBarView
import rafcon.gui.utils
import rafcon.gui.helpers.state_machine
import rafcon.gui.singleton
from rafcon.utils import log
logger = log.get_logger(__name__)


def initialize():
    tool_bar_ctrl = rafcon.gui.singleton.main_window_controller.get_controller('tool_bar_controller')
    rafcon.gui.utils.wait_for_gui()
    assert isinstance(tool_bar_ctrl.view, ToolBarView)

    # add new button


    # button.set_label_widget(create_label_widget_with_icon(constants.BUTTON_REFR, _("Refresh Selected"),
    #                                                                        "Refresh selected state machine"))
    plan_sm_button = Gtk.ToolButton(label='Rafcon Task Planner Plugin')
    plan_sm_button.set_label_widget(create_label_widget_with_icon('f1ec', _("Rafcon Task Planner Plugin"),
                                                                            "Open planning Form"))
    plan_sm_button.set_stock_id(Gtk.STOCK_CLEAR)
    tool_bar_ctrl.view.get_top_widget().add(plan_sm_button)
    plan_sm_button.show_all()
    plan_sm_button.connect('clicked', __on_button_clicked)





def __on_button_clicked(*args):
    logger.debug('opening planning form!')
    PlanningSetupForm(datastore_from_file(DATASTORE_STORAGE_PATH)).initialize()