import os
from de.dlr.rmc.rafcontpp.view import planning_button
from rafcon.utils import log
logger = log.get_logger(__name__)


def pre_init():
    """ The pre_init function of the auto layout plugin. Currently method refresh selected state machine is used to
    trigger a auto layout.
    :return:
    """
    logger.info("Run pre-initiation hook of {0} plugin.".format(__file__.split(os.path.sep)[-2]))
    #TODO load datastore here!


def main_window_setup(main_window_controller):
    logger.info("Run main window setup of {0} plugin.".format(__file__.split(os.path.sep)[-2]))
    planning_button.initialize()




def post_init(*args, **kwargs):
    """
    The post_init function of the execution hooks plugin. The observer of the execution hooks manager are initialized
    and hooks execution is enabled.
    :return:
    """
    logger.info("Run post-initiation hook of {0} plugin".format(__file__.split(os.path.sep)[-2]))
