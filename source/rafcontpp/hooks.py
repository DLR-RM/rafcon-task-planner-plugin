# Copyright (C) 2018-2019 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the 3-Clause BSD License which accompanies this
# distribution, and is available at
# https://opensource.org/licenses/BSD-3-Clause
#
# Contributors:
# Christoph Suerig <christoph.suerig@dlr.de>

# Don't connect with the Copyright comment above!
# Version 28.02.2020
import os

from rafcon.gui.helpers.label import create_tab_header_label
from rafcon.utils import log

from rafcontpp.view import planning_button
from rafcontpp.view.planning_button import RTPP_ICON
from rafcontpp.view.pddl_action_tab import PddlActionTab

logger = log.get_logger(__name__)




def pre_init():
    """ The pre_init function of the auto layout plugin. Currently method refresh selected state machine is used to
    trigger a auto layout.

    :return: void
    """
    logger.info("Run pre-initiation hook of {0} plugin.".format(__file__.split(os.path.sep)[-2]))


def main_window_setup(main_window_controller):
    """
    called on window setup.

    :param main_window_controller: Unused
    :return: void
    """
    logger.info("Run main window setup of {0} plugin.".format(__file__.split(os.path.sep)[-2]))
    # add the plan task button to rafcons menu bar.
    planning_button.initialize()


def post_init(*args, **kwargs):
    """
    The post_init function of the execution hooks plugin. The observer of the execution hooks manager are initialized
    and hooks execution is enabled.

    :return: void
    """
    logger.info("Run post-initiation hook of {0} plugin".format(__file__.split(os.path.sep)[-2]))


def post_state_editor_register_view(state_editor):
    """
    called every time, a State Editor is created.
    adds the action tab.

    :param state_editor: The state editor
    :return: void
    """
    state_editor_view = state_editor.view
    state = state_editor.model.state
    action_tab = PddlActionTab(state).init_tab()
    # add tab, with label to State Editor
    # f1ec is from font awesome
    icon = {_('PDDL Action'): RTPP_ICON}  # its done like this, because the helper function needs a map as input
    main_notebook_2 = state_editor_view["main_notebook_2"]
    main_notebook_2.append_page(action_tab, create_tab_header_label('PDDL Action', icon))
    main_notebook_2.set_tab_reorderable(action_tab, True)
