# Contributors:
# Christoph Suerig <christoph.suerig@dlr.de>
# Version 21.06.1019

import math
import time
from rafcon.utils import log
from rafcon.gui.models.state_machine import StateMachineModel
logger = log.get_logger(__name__)

class StateMachineLayouter:
    '''
    StateMachineLayouter gets a state machine, and layouts it in a particular way.
    '''



    def layout_state_machine(self, state_machine, state_order):
        '''
         This function will format the state machine in a merlon like format.
        :param state_machine: a state machine to layout
        :param state_order: the order of the states in the machine
        '''
        start_time = time.time()
        #the state machine model.
        state_machine_m = StateMachineModel(state_machine)
        #number of rows of states.
        num_states = len(state_machine_m.root_state.states)
        row_count = self.__get_num_states_per_col(num_states)
        logger.debug("States per column: {}".format(row_count))
        column_count = num_states / row_count
        current_row = 0
        current_column = 0
        x_gap = 25 # a gap between the state columns
        y_gap = 25 # a gap between the state rows                             _   _
        # the sm will be layouted column by column, in merlon shape. Like: |_| |_| |_|
        #increment_row is true if formatting digs down a row, and false if it climbs the next row up again.
        increment_row = True
        #the width of a state in the sm
        state_width = 100.
        #the height of a state in the sm
        state_height = 100.
        #format root state
        #root state width
        r_width = column_count * (x_gap+state_width)+4*x_gap
        #root state height
        r_height = row_count * (y_gap+state_height)+4*y_gap
        logger.debug("Root state size: height: {} width: {}".format(r_height,r_width))
        #set root state size
        state_machine_m.root_state.meta['gui']['editor_opengl']['size'] = (r_width, r_height)
        state_machine_m.root_state.meta['gui']['editor_gaphas']['size'] = (r_width, r_height)
        #set root state in / out come position
        state_machine_m.root_state.income.meta['gui']['editor_gaphas']['rel_pos'] = (0.,y_gap+state_height/4.)
        out_come = [oc for oc in state_machine_m.root_state.outcomes if oc.outcome.outcome_id == 0].pop()
        out_come.meta['gui']['editor_gaphas']['rel_pos'] = (r_width,y_gap+state_height/4.)

        #positions where an income or an outcome can occure
        up_pos = (state_width/2., 0.)
        down_pos = (state_width/2., state_height)
        left_pos = (0., state_height/4.)
        right_pos = (state_width, state_height/4.)
        #format states
        for c_state_id in state_order:#state_machine_m.root_state.states.values():
            #gui model of state
            state_m = state_machine_m.root_state.states[c_state_id]

            #decide position of income and outcome
            income_pos = up_pos if increment_row else down_pos
            out_come_pos = down_pos if increment_row else up_pos
            #special cases e.g. the corners of the merlon structure.
            if current_row == 0 and current_row +1 >= row_count: # special case, if row_count = 1
                income_pos = left_pos
                out_come_pos = right_pos
            elif current_row == 0 and increment_row: # upper left corner
                income_pos = left_pos
                out_come_pos = down_pos
            elif current_row == 0 and not increment_row: # upper right corner
                income_pos = down_pos
                out_come_pos = right_pos
            elif current_row +1 >= row_count and increment_row: # lower left corner
                income_pos = up_pos
                out_come_pos = right_pos
            elif current_row +1 >= row_count and not increment_row: # lower right corner
                income_pos = left_pos
                out_come_pos = up_pos


            #set state size
            state_m.meta['gui']['editor_opengl']['size'] = (state_width, state_height)
            state_m.meta['gui']['editor_gaphas']['size'] = (state_width, state_height)

            #set position of income and outcome
            state_m.income.meta['gui']['editor_gaphas']['rel_pos'] = income_pos
            out_come = [oc for oc in state_m.outcomes if oc.outcome.outcome_id >= 0].pop()
            out_come.meta['gui']['editor_gaphas']['rel_pos'] = out_come_pos

            #set position of state
            current_x = current_column*(x_gap+state_width)+x_gap
            current_y = current_row*(y_gap+state_height)+y_gap
            state_m.meta['gui']['editor_opengl']['rel_pos'] = (current_x, current_y)
            state_m.meta['gui']['editor_gaphas']['rel_pos'] = (current_x, current_y)
            logger.debug("x: {} y: {}".format(current_x, current_y))

            #loop trailer, in / decrement rhow and column counter, decide if to increment row next.
            if current_row <= 0 and not increment_row:
                increment_row = True
                current_column +=1
            elif current_row +1 >= row_count and increment_row:
                increment_row = False
                current_column += 1
            else:
                current_row = current_row + 1 if increment_row else current_row - 1

        # =========================new start==========================
        if row_count == 1:
            for c_state_id in state_order:  # state_machine_m.root_state.states.values():
                # gui model of state
                state_m = state_machine_m.root_state.states[c_state_id]
                # set state size
                state_m.meta['gui']['editor_opengl']['size'] = (state_width, state_height)
                state_m.meta['gui']['editor_gaphas']['size'] = (state_width, state_height)
                # set position of income and outcome
                state_m.income.meta['gui']['editor_gaphas']['rel_pos'] = left_pos
                out_come = [oc for oc in state_m.outcomes if oc.outcome.outcome_id >= 0].pop()
                out_come.meta['gui']['editor_gaphas']['rel_pos'] = right_pos

                # set position of state
                current_x = current_column * (x_gap + state_width) + x_gap
                current_y = y_gap
                state_m.meta['gui']['editor_opengl']['rel_pos'] = (current_x, current_y)
                state_m.meta['gui']['editor_gaphas']['rel_pos'] = (current_x, current_y)
                logger.debug("x: {} y: {}".format(current_x, current_y))

                # loop trailer, in / decrement rhow and column counter, decide if to increment row next.
                if current_row <= 0 and not increment_row:
                    increment_row = True
                    current_column += 1
                elif current_row + 1 >= row_count and increment_row:
                    increment_row = False
                    current_column += 1
                else:
                    current_row = current_row + 1 if increment_row else current_row - 1

        else:

        # =========================new end  ==========================

        #last state is a special case, its outcome should always be right.
        out_come = [oc for oc in state_machine_m.root_state.states[state_order[-1]].outcomes if oc.outcome.outcome_id == 0].pop()
        out_come.meta['gui']['editor_gaphas']['rel_pos'] = right_pos

        #store the meta data.
        state_machine_m.store_meta_data()
        logger.info("State machine layouting took {0:.4f} seconds.".format(time.time()- start_time))




    def __get_num_states_per_col(self, num_states):
        '''
        gets a number of states, and returns a calculated row count. e.g. how many states per column are desired.
        :param num_states: number of states in a state machine.
        :return: the number of states per column.
        '''

        height = math.sqrt(num_states/1.78)#claculates the hight for approximatly ratio of 16:9, which is appr. 1.78:1
        height = round(height) if height > 1 else 1
        return height


