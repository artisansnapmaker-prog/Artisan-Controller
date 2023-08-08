#include "AxisManager.h"
#include "shaper/MoveQueue.h"

#define DEFAULT_IS_FREQ   (50)
#define DEFAULT_IS_DAMP   (0.1)
#define DEFAULT_IS_TYPE   (1)

AxisManager axisManager;

static const char *dbg_name[SHAPER_DBG_MAX] = {
  "EMPTY_MOVES_COUNT",
  "NO_STEPS",
  "NOT_ENOUGH_MOVES_RESC",
  "NOT_ENOUGH_FUNC_LIST_RESC",
  "CALC_STEP_TIMEOUT_COUNT",
  "CALC_STEP_TIME",
  "ABORT_END_BLOCK"
};

void AxisManager::input_shaper_reset() {

  axisManager.axis[X_AXIS].axis_input_shaper->type = (InputShaperType)DEFAULT_IS_TYPE;
  axisManager.axis[X_AXIS].axis_input_shaper->frequency = DEFAULT_IS_FREQ;
  axisManager.axis[X_AXIS].axis_input_shaper->zeta = DEFAULT_IS_DAMP;

  axisManager.axis[Y_AXIS].axis_input_shaper->type = (InputShaperType)DEFAULT_IS_TYPE;
  axisManager.axis[Y_AXIS].axis_input_shaper->frequency = DEFAULT_IS_FREQ;
  axisManager.axis[Y_AXIS].axis_input_shaper->zeta = DEFAULT_IS_DAMP;

}

err_code_t AxisManager::input_shaper_set(int axis, int type, float freq, float dampe)  {
  if (req_abort) {
    SERIAL_ECHO_MSG("cannot set input shaper during aborting moving!\n");
    return E_FAILURE;
  }

  if (axis != X_AXIS && axis != Y_AXIS) return E_PARAM;

  AxisInputShaper* axis_input_shaper = axisManager.axis[axis].axis_input_shaper;
  if (freq != axis_input_shaper->frequency || dampe != axis_input_shaper->zeta || type != (int)axis_input_shaper->type) {
    axis_input_shaper->setConfig(type, freq, dampe);
    planner.synchronize();
    axisManager.initAxisShaper();
    axisManager.abort();
  }
  LOG_I("setting: axis: %d type: %s, frequency: %lf, zeta: %lf\n", axis, input_shaper_type_name[type], freq, dampe);

  return E_SUCCESS;
}

err_code_t AxisManager::input_shaper_get(int axis, int &type, float &freq, float &dampe) {

  if (axis != X_AXIS && axis != Y_AXIS) return E_PARAM;

  AxisInputShaper* axis_input_shaper = axisManager.axis[axis].axis_input_shaper;
  type = (int)axis_input_shaper->type;
  freq = axis_input_shaper->frequency;
  dampe = axis_input_shaper->zeta;
  LOG_I("getting: axis: %d type: %s, frequency: %lf, zeta: %lf\n", axis, input_shaper_type_name[type], freq, dampe);

  return E_SUCCESS;
}


void AxisManager::show_debug_info() {
  LOG_I("debug info for input shaper:\n");
  for (int i = 0; i < SHAPER_DBG_MAX; i++) {
    LOG_I("[%s] = %d\n", dbg_name[i], counts[i]);
  }
}


void AxisManager::reset_debug_info() {
  for (int i = 0; i < SHAPER_DBG_MAX; i++) {
    counts[i] = 0;
  }
}


FORCE_INLINE err_code_t Axis::generateFuncParams(uint8_t block_index, uint8_t move_start, uint8_t move_end) {
    if (block_index == generated_block_index) {
        return E_SUCCESS;
    }
    // is_get_next_step_null = false;

    err_code_t res;
    if (is_shaped) {
        res = axis_input_shaper->generateShapedFuncParams(&func_manager, move_start, move_end);
    #if ENABLED(LIN_ADVANCE)
    } else if (axis == E_AXIS) {
        res = generateEAxisFuncParams(block_index, move_start, move_end);
    #endif
    } else {
      res = generateAxisFuncParams(move_start, move_end);
    }

    // E_NO_RESRC indicates no move to be used to generate function parameters
    if (res == E_SUCCESS || res == E_NO_RESRC) {
        generated_block_index = block_index;
        return E_SUCCESS;
    }
    else
        return res;
}

bool Axis::getNextStep() {
    bool result;
    if (axis == E_AXIS || axis == J_AXIS)
        result = func_manager.getNextPosTimeEextend(1, &dir, mm_to_step, half_step_mm);
    else
        result = func_manager.getNextPosTime(1, &dir, mm_to_step, half_step_mm);
    if (!result) {
      is_get_next_step_null = true;
      return false;
    }

    print_time = func_manager.print_time;
    is_consumed = false;

    return true;
}

#if ENABLED(LIN_ADVANCE)
FORCE_INLINE err_code_t Axis::generateEAxisFuncParams(uint8_t block_index, uint8_t move_start, uint8_t move_end) {
    uint8_t move_index;
    block_t *block = &planner.block_buffer[block_index];
    float e_la = 0;

    // LOG_I("generate E AxisFuncParams!\n");

    if (generated_move_index == -1) {
        move_index = move_start;
    } else {
        move_index = moveQueue.nextMoveIndex(generated_move_index);
    }

    while (move_index != moveQueue.nextMoveIndex(move_end)) {
        Move *move = &moveQueue.moves[move_index];

        if (IS_ZERO(move->t)) {
          move_index = moveQueue.nextMoveIndex(move_index);
          continue;
        }

        // #define K (0.04)
        float K = block->use_advance_lead ? planner.extruder_advance_K[active_extruder] * 1000 : 0;
        float delta_v = IS_ZERO(move->accelerate) ? 0 : K * move->accelerate;
        float eda = delta_v * move->t * move->axis_r[axis];

        // LOG_I("k: %lf, d_v: %lf, eda: %lf\n", K, delta_v, eda);

        double a = 0.5f * move->accelerate * move->axis_r[axis];
        double b, c, x2, y2, dx, dy;

        if (eda < 0 && move->start_v + delta_v > 0 && move->end_v + delta_v < 0) {
            float zero_t = ABS((move->start_v + delta_v) / move->accelerate);
            float zero_pos = ((move->start_v + delta_v) * zero_t + 0.5f * move->accelerate * sq(zero_t)) * move->axis_r[axis];

            y2 = move->start_pos_e + delta_e + zero_pos;
            dy = zero_pos;
            x2 = zero_t;
            dx = zero_t;

            c = move->start_pos_e + delta_e;
            b = dy / dx - a * x2;

            int type;
            if (IS_ZERO(dy)) {
                type = 0;
            } else {
                type = dy > 0 ? 1 : -1;
            }

            time_double_t end_t = move->start_t + zero_t;
            func_manager.addFuncParamsExtend(a, b, c, type, end_t, y2);

            y2 = move->end_pos_e + delta_e + eda;
            dy = move->end_pos_e - move->start_pos_e + eda - zero_pos;
            x2 = move->t - zero_t;
            dx = move->t - zero_t;

            c = move->start_pos_e + delta_e + zero_pos;
            b = dy / dx - a * x2;

            if (IS_ZERO(dy)) {
                type = 0;
            } else {
                type = dy > 0 ? 1 : -1;
            }

            end_t = move->end_t;
            func_manager.addFuncParamsExtend(a, b, c, type, end_t, y2);
        } else {
            y2 = move->end_pos_e + delta_e + eda;
            dy = move->end_pos_e - move->start_pos_e + eda;
            x2 = move->t;
            dx = move->t;

            c = move->start_pos_e + delta_e;
            b = dy / dx - a * x2;

            int type;
            if (IS_ZERO(dy)) {
                type = 0;
            } else {
                type = dy > 0 ? 1 : -1;
            }
            time_double_t end_t = move->end_t;
            func_manager.addFuncParamsExtend(a, b, c, type, end_t, y2);
        }

        delta_e += eda;
        e_la += eda;
        move_index = moveQueue.nextMoveIndex(move_index);
    }

    block->e_stepper_offset += e_la;

    generated_move_index = move_end;
    return E_SUCCESS;
}
#endif


FORCE_INLINE err_code_t Axis::generateAxisFuncParams(uint8_t move_start, uint8_t move_end) {
  uint8_t move_index;

    // LOG_I("generateAxisFuncParams!\n");

  if (generated_move_index == -1) {
      move_index = move_start;
  } else {
      move_index = moveQueue.nextMoveIndex(generated_move_index);
  }

  while (move_index != moveQueue.nextMoveIndex(move_end)) {
    Move *move = &moveQueue.moves[move_index];

    if (IS_ZERO(move->t)) {
      move_index = moveQueue.nextMoveIndex(move_index);
      continue;
    }

    if (axis == J_AXIS)
        generateLineFuncParamsExtend(move);
    else
        generateLineFuncParams(move);

    move_index = moveQueue.nextMoveIndex(move_index);
  }
  generated_move_index = move_end;
  return E_SUCCESS;
}


// FORCE_INLINE void Axis::generateLineFuncParams(Move* move) {
//     float y2 = move->end_pos[axis];
//     float dy = move->end_pos[axis] - move->start_pos[axis];
//     float x2 = move->t;
//     float dx = move->t;

//     float a = 0.5f * move->accelerate * move->axis_r[axis];
//     float c = move->start_pos[axis];
//     float b = dy / dx - a * x2;

//     int type;
//     if (IS_ZERO(dy)) {
//         type = 0;
//     } else {
//         type = dy > 0 ? 1 : -1;
//     }
//     time_double_t end_t = move->end_t;
//     func_manager.addFuncParams(a, b, c, type, end_t, y2);
// }

float AxisManager::getRemainingConsumeTime() {
    return min_last_time - print_time;
}

err_code_t AxisManager::generateAllAxisFuncParams(uint8_t block_index, block_t* block) {
    err_code_t res = E_SUCCESS;

    uint8_t move_start = block->shaper_data.move_start;
    uint8_t move_end = block->shaper_data.move_end;

    for (int i = 0; i < NUM_AXIS; ++i) {
        if (i < 2 && axis[i].func_manager.getFreeSize() < 15) {
            // axisManager.counts[SHAPER_DBG_NOT_ENOUGH_FUNC_LIST_RESC]++;
            // LOG_I("no function array, axis: %d getSize: %d, getFreeSize: %d\n", i, axis[i].func_manager.getSize(), axis[i].func_manager.getFreeSize());
            return E_NO_RESRC;
        } else if (i >= 2 && axis[i].func_manager.getFreeSize() < 4) {
            // axisManager.counts[SHAPER_DBG_NOT_ENOUGH_FUNC_LIST_RESC]++;
            // LOG_I("no function array, axis: %d getSize: %d, getFreeSize: %d\n", i, axis[i].func_manager.getSize(), axis[i].func_manager.getFreeSize());
            return E_NO_RESRC;
        }
    }

    if (isShaped()) {
        if (move_start != moveQueue.move_tail && moveQueue.moves[moveQueue.prevMoveIndex(move_start)].flag == MOVE_FLAG_START) {
            move_start = moveQueue.prevMoveIndex(move_start);
        }

        if (!moveQueue.isBetween(move_start)) {
            move_start = moveQueue.move_tail;
        }

        if (move_end != moveQueue.prevMoveIndex(moveQueue.move_head) && moveQueue.moves[moveQueue.nextMoveIndex(move_end)].flag == MOVE_FLAG_START) {
            move_end = moveQueue.nextMoveIndex(move_end);
        }

        if (!moveQueue.isBetween(move_end)) {
            move_end = moveQueue.prevMoveIndex(moveQueue.move_head);
        }
    }

    // LOG_I("start %d, end %d\n", move_start, move_end);

    for (int i = 0; i < NUM_AXIS; ++i) {
        // if (i < 2 && axis[i].func_manager.getFreeSize() < 15) {
        //     axisManager.counts[SHAPER_DBG_NOT_ENOUGH_FUNC_LIST_RESC]++;
        //     return E_NO_RESRC;
        // } else if (i >= 2 && axis[i].func_manager.getFreeSize() < 4) {
        //     axisManager.counts[SHAPER_DBG_NOT_ENOUGH_FUNC_LIST_RESC]++;
        //     return E_NO_RESRC;
        // }

        res = axis[i].generateFuncParams(block_index, move_start, move_end);
        if (res != E_SUCCESS) {
            LOG_E("failed to generate func for axis[%d], ret[%u]!\n", i, res);
            return res;
        }
    }

    uint8_t new_move_tail = moveQueue.calculateMoveStart(move_start, axisManager.shaped_delta_window);

    moveQueue.updateMoveTail(new_move_tail);

    axisManager.updateMinLastTime();

    return res;
}

// /*
//  Copy next step's information to axis_stepper
//  and mark is_consumed = true;
// */
// bool AxisManager::getCurrentAxisStepper(AxisStepper *axis_stepper) {
//     if (is_consumed) {
//         return false;
//     }
//     axis_stepper->axis = print_axis;
//     axis_stepper->dir = print_dir;
//     axis_stepper->print_time = print_time;

//     if (print_axis != T0_T1_AXIS_INDEX)
//       current_steps[print_axis] += print_dir;

//     is_consumed = true;
//     return true;
// }

/*
 Calcuation all axes's next step's time if need
 And then return the closest time axis if we have
*/
bool AxisManager::calcNextAxisStepper() {
    if (getAxisStepperFreeSize() == 0) {
        return false;
    }

    // If a axis has been consumed, calculate the next step time
    for (int i = 0; i < NUM_AXIS; ++i) {
        if (axis[i].is_consumed) {
            axis[i].getNextStep();
        }
    }

    bool is_consumed = true;
    // Fine the closest time of all the axes
    time_double_t min_print_time = 2000000000;
    for (int i = 0; i < NUM_AXIS; ++i) {
        if (!axis[i].is_consumed) {
            if (axis[i].print_time < min_print_time) {
                min_print_time = axis[i].print_time;
                print_axis = i;
                is_consumed = false;
            }
        }
    }

    // is_consumed == false, means that we has a steps need to output
    if (!is_consumed) {
        axis[print_axis].is_consumed = true;

        print_dir = axis[print_axis].dir;

        AxisStepper* axis_stepper = &axis_steppers[axis_steppper_head];

        axis_stepper->axis = print_axis;
        axis_stepper->dir = print_dir;
        axis_stepper->delta_time = min_print_time - print_time;
        axis_stepper->print_time = min_print_time;

        print_time = min_print_time;

        axis_steppper_head = nextAxisStepper(axis_steppper_head);

        return true;
    } else {
        return false;
    }
}

