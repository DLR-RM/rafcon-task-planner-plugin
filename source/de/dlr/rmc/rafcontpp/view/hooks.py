import gtk
import os
from de.dlr.rmc.rafcontpp.rafcontpp_main import rafcontpp_main
from rafcon.gui.views.tool_bar import ToolBarView
import rafcon.gui.utils
import rafcon.gui.helpers.state_machine
import rafcon.gui.singleton
from rafcon.utils import log
logger = log.get_logger(__name__)


def pre_init():
    """ The pre_init function of the auto layout plugin. Currently method refresh selected state machine is used to
    trigger a auto layout.
    :return:
    """
    logger.info("Run pre-initiation hook of {0} plugin.".format(__file__.split(os.path.sep)[-2]))


def main_window_setup(main_window_controller):
    import rafcon.gui.helpers.state_machine
    import rafcon.gui.singleton

    tool_bar_ctrl = rafcon.gui.singleton.main_window_controller.get_controller('tool_bar_controller')
    from rafcon.gui.views.tool_bar import ToolBarView
    import rafcon.gui.utils
    rafcon.gui.utils.wait_for_gui()
    assert isinstance(tool_bar_ctrl.view, ToolBarView)

    #add new button
    import gtk
    plan_sm_button = gtk.ToolButton(label='Plan state machine')
    plan_sm_button.set_stock_id(gtk.STOCK_CLEAR)
    tool_bar_ctrl.view.get_top_widget().add(plan_sm_button)
    plan_sm_button.show_all()

    def open_form(*args):
        logger.debug('opened planning form!')
        rafcontpp_main()
    plan_sm_button.connect('clicked', open_form)


def post_init(*args, **kwargs):
    """
    The post_init function of the execution hooks plugin. The observer of the execution hooks manager are initialized
    and hooks execution is enabled.
    :return:
    """
    logger.info("Run post-initiation hook of {0} plugin".format(__file__.split(os.path.sep)[-2]))
