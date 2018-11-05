import gtk
from de.dlr.rmc.rafcontpp.view.planning_setup_form import PlanningSetupForm
from de.dlr.rmc.rafcontpp.rafcontpp_main import rafcontpp_main
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

    plan_sm_button = gtk.ToolButton(label='Plan state machine')
    plan_sm_button.set_stock_id(gtk.STOCK_CLEAR)
    tool_bar_ctrl.view.get_top_widget().add(plan_sm_button)
    plan_sm_button.show_all()
    plan_sm_button.connect('clicked', __on_button_clicked)





def __on_button_clicked(*args):
    logger.debug('opened planning form!')
    rafcontpp_main()
    #PlanningSetupForm().initialize()