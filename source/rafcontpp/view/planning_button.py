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

import threading
import time

import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk
import rafcon.gui.utils
import rafcon.gui.helpers.state_machine
import rafcon.gui.singleton as gui_singletons
from rafcon.gui.views.tool_bar import ToolBarView
from rafcon.gui.helpers.label import create_label_widget_with_icon
from rafcon.utils import log
from rafcontpp.view.planning_setup_form import PlanningSetupForm
from rafcontpp.model.datastore import datastore_from_file, DATASTORE_STORAGE_PATH, get_planning_threads

logger = log.get_logger(__name__)

RTPP_ICON = '&#xf1ec;'

plan_task_label = "Plan Task"
tool_tip_text = "Opens the planning Configuration, to plan a new task."
plan_sm_button = None
button_counter = 0
lock = None


def initialize():
    tool_bar_ctrl = gui_singletons.main_window_controller.get_controller('tool_bar_controller')
    rafcon.gui.utils.wait_for_gui()
    assert isinstance(tool_bar_ctrl.view, ToolBarView)
    global lock
    lock = threading.Lock()
    global button_counter
    button_counter = 0
    # add new button
    global plan_sm_button
    plan_sm_button = Gtk.MenuToolButton(label='Plan Task')
    plan_sm_button.set_label_widget(create_label_widget_with_icon(RTPP_ICON, _(plan_task_label), tool_tip_text))
    plan_sm_button.set_stock_id(Gtk.STOCK_CLEAR)
    tool_bar_ctrl.view.get_top_widget().add(plan_sm_button)
    plan_sm_button.show_all()
    plan_sm_button.set_menu(Gtk.Menu())
    plan_sm_button.set_arrow_tooltip_text('A List of running Tasks. Click on a Task to Cancel it.')
    plan_sm_button.connect('clicked', __on_button_clicked)
    plan_sm_button.connect('show-menu', __on_show_menu)


def increment_button():
    """
    The Plan Task button can be incremented, and then looks like: Plan Task (n)
    this method increments the 'n', synchronized and also redraws the button thread save.

    :return: void
    """
    with lock:
        # logger.debug('increment button executed from: {}'.format(threading.current_thread().getName()))# todo remove
        global button_counter
        button_counter += 1
        plan_sm_button.set_label_widget(create_label_widget_with_icon(RTPP_ICON,
                                                                      _(plan_task_label + ' ({})'.format(
                                                                          button_counter)),
                                                                      tool_tip_text + '\n' + str(
                                                                          button_counter) + __get_progress_text()))


def decrement_button():
    """
    The Plan Task button can be decremented, and then looks like: Plan Task (n) or just Plan Task, if n == 0
    this method decrements the 'n', synchronized and also redraws the button thread save.

    :return: void
    """
    with lock:
        # logger.debug('decrement button executed from: {}'.format(threading.current_thread().getName()))# todo remove
        global button_counter
        button_counter -= 1
        if button_counter <= 0:
            button_counter = 0
            plan_sm_button.set_label_widget(create_label_widget_with_icon(RTPP_ICON, _(plan_task_label), tool_tip_text))
        else:
            plan_sm_button.set_label_widget(
                create_label_widget_with_icon(plan_task_fa_icon, _(plan_task_label + ' ({})'.format(button_counter)),
                                              tool_tip_text + '\n' + str(button_counter) + __get_progress_text()))


def __on_button_clicked(button):
    """
    Opens the planning setup form.

    :return: void
    """
    PlanningSetupForm(datastore_from_file(DATASTORE_STORAGE_PATH)).initialize()


def __on_show_menu(button):
    """
    Opens the drop-down menu, with all currently running tasks.

    :return: void
    """
    cancel_task_menu = button.get_menu()
    # first remove all entries (they could be outdated)
    for child in cancel_task_menu.get_children():
        cancel_task_menu.remove(child)
    # now get all registered threads and display them with names, also register a on click for each menu item
    planning_threads = get_planning_threads()
    current_time = time.time()
    # a map containg all threads, with the created label as key
    label_thread = {}  # label:thread
    # ------------------------------------------------------------------------------------------------------------------
    # a call back function for an activated menu item.
    def __on_menu_item_activate(menu_item):
        """
        Handles the abortion of a task.

        :return: void
        """
        # get label
        label = menu_item.get_label()
        # load glade file
        cancel_dialog = Gtk.Dialog("Task Planner Plugin - Cancel Task", None, 0,
                                   (Gtk.STOCK_YES, Gtk.ResponseType.YES, Gtk.STOCK_NO, Gtk.ResponseType.NO))
        cancel_dialog.set_default_size(400, 100)
        cancel_dialog.get_content_area().add(Gtk.Label('Do you really want to cancel Task:\n' + label + '?'))
        main_window = gui_singletons.main_window_controller.view['main_window']
        cancel_dialog.set_transient_for(main_window)
        cancel_dialog.show_all()
        user_response = cancel_dialog.run()
        cancel_dialog.destroy()
        if user_response == Gtk.ResponseType.YES:
            if label_thread[label].is_alive():
                logger.info('Canceling Task: {}'.format(label))
                label_thread[label].interrupt()
                logger.info('Interrupted Task: {}'.format(label))
            else:
                logger.info('Task {} already terminated.'.format(label))
    # ------------------------------------------------------------------------------------------------------------------
    # fill menu:
    for index, key in enumerate(planning_threads.keys()):
        label = planning_threads[key][1] + ' ({0:.4f}s)'.format(current_time - key)
        # to ensure, no labels are duplicates.
        label_counter = 1
        while label in label_thread.keys():
            label = label + '({})'.format(label_counter)
            label_counter += 1
        label_thread[label] = planning_threads[key][0]
        c_menu_item = Gtk.MenuItem.new_with_label(label)
        c_menu_item.set_tooltip_text('Planner: {}'.format(planning_threads[key][2]))
        c_menu_item.connect('activate', __on_menu_item_activate)
        cancel_task_menu.attach(c_menu_item, 0, 1, index, index + 1)
    cancel_task_menu.show_all()


def __get_progress_text():
    """
    This method just decides between plural and singular.

    :return: String: The progress test.
    """
    if button_counter == 1:
        return " Task in progress."
    else:
        return " Tasks in progress."
