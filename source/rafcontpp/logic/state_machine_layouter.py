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
# Version 21.02.2020

import math
import time
import rafcon
from gaphas.solver import Variable
from rafcon.gui.models.state_machine import StateMachineModel
from rafcon.gui.singleton import state_machine_manager_model
from rafcon.gui.utils import constants
from rafcon.utils import log

logger = log.get_logger(__name__)


class StateMachineLayouter:
    """
    StateMachineLayouter gets a state machine, and layouts it in a particular way.
    """

    def layout_state_machine(self, state_machine, target_state, fixed_size, state_order):
        """
        This function will format the state machine in a merlon like format.

        :param state_machine: A state machine to layout.
        :param target_state: The "root state" all content in the state will be formated, it needs to be tube like.
        :param fixed_size: True if the size of the root state is fixed.
        :param state_order: The order of the states in the machine.
        :return: void
        """
        start_time = time.time()
        logger.info("Layouting state machine...")
        # the state machine model.
        state_machine_m = None
        if state_machine.state_machine_id in state_machine_manager_model.state_machines:
            state_machine_m = state_machine_manager_model.state_machines[state_machine.state_machine_id]
        else:
            state_machine_m = StateMachineModel(state_machine)
        target_state_m = state_machine_m.get_state_model_by_path(target_state.get_path())
        # number of rows of states.
        num_states = len(target_state_m.states)
        row_count = 0
        column_count = 0
        label_height = 0  # the height of the state label.
        x_gap = 25  # a gap between the state columns
        y_gap = 25  # a gap between the state rows                             _   _
        # the sm will be layouted column by column, in merlon shape. Like: |_| |_| |_|
        # the width of a state in the sm
        state_width = 100.
        # the height of a state in the sm
        state_height = 100.
        # format root state
        canvas_height = 0
        canvas_width = 0
        r_width = 0
        r_height = 0
        border_size = 0
        if fixed_size:
            r_width, r_height = target_state_m.meta['gui']['editor_gaphas']['size']
            border_size = Variable(min(r_width, r_height) / constants.BORDER_WIDTH_STATE_SIZE_FACTOR)
            #get the height of the state name, so that states won't cover target state name.
            graphical_editor_controller = rafcon.gui.singleton.main_window_controller.state_machines_editor_ctrl.get_controller(1)
            target_state_view = graphical_editor_controller.canvas.get_view_for_model(target_state_m)
            if target_state_view:
                target_name_view = target_state_view.name_view
                label_height = target_name_view.height
            else:
                logger.error("Could not get state view of state: {}".format(target_state_m))
                label_height = 0 #Overriding the lable is the best option when not knowing its height, in order to maintain the right layout.
            canvas_height = r_height - label_height - 2 * border_size
            canvas_width = r_width - 2 * border_size
            row_count = self.__get_num_rows(num_states, canvas_width, canvas_height)
            column_count = math.ceil(num_states / row_count)
            state_width, state_height, x_gap, y_gap = self.__get_state_dimensions(canvas_width, canvas_height,
                                                                                  column_count + 1, row_count)
        else:
            label_height = 100  # the height of the state label, it's an assumed default height.
            row_count = self.__get_num_rows(num_states)
            column_count = math.ceil(num_states / row_count)
            canvas_width = (column_count + 1) * (x_gap + state_width) + x_gap
            canvas_height = row_count * (y_gap + state_height) + label_height
            # root state width, height, and root state border size.
            r_width, r_height, border_size = self.__get_target_state_dimensions(canvas_width, canvas_height)
            logger.debug("Root state size: height: {} width: {}".format(r_height, r_width))
            # set root state size
            target_state_m.meta['gui']['editor_gaphas']['size'] = (r_width, r_height)
        # set root state in / out come position
        target_state_m.income.set_meta_data_editor('rel_pos', (0., label_height + border_size + y_gap + state_height / 4. ))
        out_come = [oc for oc in target_state_m.outcomes if oc.outcome.outcome_id == 0].pop()
        out_come.meta['gui']['editor_gaphas']['rel_pos'] = (r_width, label_height + border_size + y_gap + state_height / 4.)
        # positions where an income or an outcome can occure
        up_pos = (state_width / 2., 0.)
        down_pos = (state_width / 2., state_height)
        left_pos = (0., state_height / 4.)
        right_pos = (state_width, state_height / 4.)
        current_row = 0
        current_column = 0
        # increment_row is true if formatting digs down a row, and false if it climbs the next row up again.
        increment_row = True
        # format states
        for c_state_id in state_order:  # state_machine_m.root_state.states.values():
            # gui model of state
            state_m = target_state_m.states[c_state_id]
            # decide position of income and outcome
            income_pos = up_pos if increment_row else down_pos
            outcome_pos = down_pos if increment_row else up_pos
            # special cases e.g. the corners of the merlon structure.
            if current_row == 0 and current_row + 1 >= row_count:  # special case, if row_count = 1
                income_pos = left_pos
                outcome_pos = right_pos
            elif current_row == 0 and increment_row:  # upper left corner
                income_pos = left_pos
                outcome_pos = down_pos
            elif current_row == 0 and not increment_row:  # upper right corner
                income_pos = down_pos
                outcome_pos = right_pos
            elif current_row + 1 >= row_count and increment_row:  # lower left corner
                income_pos = up_pos
                outcome_pos = right_pos
            elif current_row + 1 >= row_count and not increment_row:  # lower right corner
                income_pos = left_pos
                outcome_pos = up_pos
            # set state size
            state_m.meta['gui']['editor_gaphas']['size'] = (state_width, state_height)
            # set position of income and outcome
            state_m.income.set_meta_data_editor('rel_pos', income_pos)
            out_come = [oc for oc in state_m.outcomes if oc.outcome.outcome_id >= 0].pop()
            out_come.meta['gui']['editor_gaphas']['rel_pos'] = outcome_pos
            # set position of state
            current_x = current_column * (x_gap + state_width) + x_gap + border_size
            current_y = current_row * (y_gap + state_height) + y_gap + border_size + label_height
            state_m.meta['gui']['editor_gaphas']['rel_pos'] = (current_x, current_y)
            # logger.debug("x: {} y: {}".format(current_x, current_y))
            # loop trailer, in / decrement row and column counter, decide if to increment row next.
            if current_row <= 0 and not increment_row:
                increment_row = True
                current_column += 1
            elif current_row + 1 >= row_count and increment_row:
                increment_row = False
                current_column += 1
            else:
                current_row = current_row + 1 if increment_row else current_row - 1
        # last state is a special case, its outcome should always be right.
        out_come = [oc for oc in target_state_m.states[state_order[-1]].outcomes if oc.outcome.outcome_id == 0].pop()
        out_come.meta['gui']['editor_gaphas']['rel_pos'] = right_pos
        # store the meta data.
        if state_machine.file_system_path:
            state_machine_m.store_meta_data()  # TODO find solution, if state machine was never saved bevore.
        logger.info("State machine layouting took {0:.4f} seconds.".format(time.time() - start_time))

    def __get_num_rows(self, num_states, width=16., height=9.):
        """
        Get num rows, receives the number of states, a width and a height. it uses the width and the height to calculate
        a ratio, to be able to calculate the number of rows to use the given space optimal.

        :param num_states: The number of the states used.
        :param width: The width of the available space. if this or height <= 0 automatically set to 16.
        :param height: The height of the available space if this or heigt <= 0 automatically set to 9.
        :return: double: The number of rows optimal in the state machine.
        """
        if width <= 0 or height <= 0:
            width = 16.
            height = 9.
        ratio = round(width / height, 2)
        logger.debug('Width / Height Ratio: {}:1'.format(ratio))
        row_count = math.sqrt(
            num_states / ratio)  # claculates the hight for approximatly ratio of 16:9, which is appr. 1.78:1
        row_count = round(row_count) if row_count > 1 else 1
        return row_count

    def __get_target_state_dimensions(self, canvas_width, canvas_height):
        """
        get_target_state_dimensions receives a desired canvas width and height, and returns the overall rootstate size,
        and the border width.

        :param canvas_width: The width of the canvas.
        :param canvas_height: The height of the canvas.
        :return: (double, double, double): (width, height, border_width).
        """
        border_width = Variable(min(canvas_width, canvas_height) / constants.BORDER_WIDTH_STATE_SIZE_FACTOR)
        r_width = canvas_width + 2 * border_width
        r_height = canvas_height + 2 * border_width
        border_width = Variable(min(r_width, r_height) / constants.BORDER_WIDTH_STATE_SIZE_FACTOR)
        while (r_width - 2 * border_width) < canvas_width and (r_height - 2 * border_width) < canvas_height:
            r_width = r_width + 2 * border_width
            r_height = r_height + 2 * border_width
            border_width = Variable(min(r_width, r_height) / constants.BORDER_WIDTH_STATE_SIZE_FACTOR)
        return (r_width, r_height, border_width)

    def __get_state_dimensions(self, canvas_width, canvas_height, col_count, row_count):
        """
        get_state_dimensions reveives a fixed canvas width and height, a col and a row count and calcualtes a possible
        state size. adds an additional x_gap.

        :param canvas_width:
        :param canvas_height:
        :return: (double,double,double,double). (state_width, state_height, x_gap, y_gap)
        """
        num_xgap = 1 + col_count
        num_ygap = 1 + row_count
        state_width = max(canvas_width / (0.25 * num_xgap + col_count), 1)
        state_height = max(canvas_height / (0.25 * num_ygap + row_count), 1)
        state_width = min(state_width, state_height)
        state_height = min(state_width, state_height)
        x_gap = 0.25 * state_width
        y_gap = 0.25 * state_height
        return state_width, state_height, x_gap, y_gap
