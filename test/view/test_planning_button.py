# Contributors:
# Christoph Suerig <christoph.suerig@dlr.de>
# Version 08.03.2019
import pytest
import testing_utils
from testing_utils import call_gui_callback
from rafcontpp.view import planning_button


def test_increment_button():
    # arrange
    testing_utils.run_gui(gui_config={'HISTORY_ENABLED': False, 'AUTO_BACKUP_ENABLED': False})
    call_gui_callback(planning_button.initialize)
    # act / assert

    try:
        for index in range(100):
            call_gui_callback(planning_button.increment_button)
            assert index+1 == planning_button.button_counter

    finally:
        testing_utils.close_gui()
        testing_utils.shutdown_environment(caplog=None)

def test_decrement_button():

    #arrange
    testing_utils.run_gui(gui_config={'HISTORY_ENABLED': False, 'AUTO_BACKUP_ENABLED': False})
    call_gui_callback(planning_button.initialize)
    for index in range(100):
        call_gui_callback(planning_button.increment_button)
    try:
        rnge = 150
        for index in range(rnge):
            expected_value = rnge - (index+51) if rnge - (index+51) > 0 else 0
            call_gui_callback(planning_button.decrement_button)
            assert expected_value == planning_button.button_counter

    finally:
        testing_utils.close_gui()
        testing_utils.shutdown_environment(caplog=None)