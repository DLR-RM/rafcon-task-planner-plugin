# Contributors:
# Christoph Suerig <christoph.suerig@dlr.de>
# Version 31.05.2019
import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk
from gi.repository import Gdk
import os
from rafcon.utils import log
import rafcon.gui.singleton as gui_singletons
logger = log.get_logger(__name__)

class PlanningWaitWindow:
    '''
    Just a little confirm dialog, indicating that planning is in progress, and the user can wait.
    '''

    def __init__(self):

        planning_wait_dialog_path = os.path.abspath(
            os.path.join(os.path.dirname(os.path.realpath(__file__)), "glade", "planning_wait_dialog.glade"))
        builder = Gtk.Builder()
        builder.add_from_file(planning_wait_dialog_path)
        self.__planning_wait_dialog = builder.get_object('rtpp_planning_wait_window')
        self.__planning_wait_dialog.set_title('Task Planner Plugin')
        main_window = gui_singletons.main_window_controller.view['main_window']
        self.__planning_wait_dialog.set_transient_for(main_window)
        self.__planning_wait_dialog.set_position(Gtk.WindowPosition.CENTER_ALWAYS)
        window_button = builder.get_object('rtpp_planning_wait_window_ok_button')
        window_button.connect('clicked', lambda x: planning_wait_dialog.destroy())


    def show(self):
        '''
        shows the planning wait window.
        '''
        self.__planning_wait_dialog.show_all()

    def hide(self):
        '''
        hides the planning_wait_window
        '''
        self.__planning_wait_dialog.hide()


    def destroy(self):
        '''
        destroys the planning_wait_window
        '''
        self.__planning_wait_dialog.destroy()