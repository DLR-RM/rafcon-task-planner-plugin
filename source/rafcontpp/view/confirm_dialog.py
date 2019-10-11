# Copyright (C) 2019 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the 3-Clause BSD License which accompanies this
# distribution, and is available at
# https://opensource.org/licenses/BSD-3-Clause
#
# Contributors:
# Christoph Suerig <christoph.suerig@dlr.de>

# Don't connect with the Copyright comment above!
# Version 12.07.2019
import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk
import os
from rafcon.utils import log

logger = log.get_logger(__name__)


class ConfirmDialog:
    """
    Just a little confirm dialog.
    """

    def __init__(self, parent, content):
        """

        :param parent: The parent window of the dialog.
        :param content: The text to display in the dialog.
        """
        confirm_dialog_path = os.path.abspath(
            os.path.join(os.path.dirname(os.path.realpath(__file__)), "glade", "confirm_dialog.glade"))
        builder = Gtk.Builder()
        builder.add_from_file(confirm_dialog_path)
        self.__confirm_dialog = builder.get_object('rtpp_confirm_dialog')
        self.__confirm_dialog.set_title('Task Planner Plugin')
        self.__confirm_dialog.set_transient_for(parent)
        self.__confirm_dialog.set_position(Gtk.WindowPosition.CENTER_ALWAYS)
        builder.get_object('rtpp_comfirm_dialog_label').set_text(str(content))
        window_button = builder.get_object('rtpp_planning_confirm_dialog_ok_button')
        window_button.connect('clicked', lambda x: self.__confirm_dialog.destroy())

    def show(self):
        """
        Displays the dialog on screen.

        :return: void
        """
        self.__confirm_dialog.show_all()

    def hide(self):
        """
        Hides the dialog.

        :return: void
        """
        self.__confirm_dialog.hide()

    def destroy(self):
        """
        Destroys the dialog.

        :return: void
        """
        self.__confirm_dialog.destroy()
