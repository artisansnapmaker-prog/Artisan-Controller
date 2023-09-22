#include "MoveQueue.h"
#include "../../../snapmaker/src/common/debug.h"

MoveQueue moveQueue;

static xyze_float_t ZERO_AXIS_R = {0};

void MoveQueue::calculateMoves(block_t* block) {
    float millimeters = block->millimeters;

    // 1mm/s -> 0.001mm/ms
    float entry_speed = block->initial_speed / 1000.0f;
    float leave_speed = block->final_speed / 1000.0f;
    float cruise_speed = block->cruise_speed / 1000.0f;

    if (cruise_speed < EPSILON) {
        // LOG_I("error speed: %lf\n", cruise_speed);
        block->shaper_data.is_zero_speed = true;
        return;
    }

    float i_cruise_speed = 1000.0f / block->cruise_speed;
    // mm/s^2 -> mm/ms^2
    float acceleration = LROUND(block->acceleration) / 1000000.0f;
    float i_acceleration = 1000000.0f / LROUND(block->acceleration);

    float accelDistance = Planner::estimate_acceleration_distance(entry_speed, cruise_speed, acceleration);
    // if (accelDistance > millimeters + EPSILON) {
        // LOG_I("error accelDistance: %lf, %lf\n", accelDistance, millimeters);
    // }
    if (accelDistance < EPSILON) {
        accelDistance = 0;
    }
    // unit: ms
    float accelClocks = (cruise_speed - entry_speed) * i_acceleration;

    float deceleration = acceleration;
    float decelDistance = Planner::estimate_acceleration_distance(cruise_speed, leave_speed, -deceleration);
    // if (decelDistance > millimeters + EPSILON) {
        // LOG_I("error decelDistance: %lf, %lf\n", decelDistance, millimeters);
    // }
    if (decelDistance < EPSILON) {
        decelDistance = 0;
    }
    float decelClocks = (cruise_speed - leave_speed) * i_acceleration;

    float plateau = millimeters - accelDistance - decelDistance;

    if (plateau < 0) {
        float newAccelDistance = Planner::intersection_distance(entry_speed, leave_speed, acceleration, millimeters);
        if (newAccelDistance > millimeters + EPSILON) {
            // LOG_I("error newAccelDistance: %lf, %lf\n", newAccelDistance, millimeters);
        }
        if (newAccelDistance > millimeters) {
            newAccelDistance = millimeters;
        }
        if (newAccelDistance < EPSILON) {
            newAccelDistance = 0;
        }
        if ((millimeters - newAccelDistance) < EPSILON) {
            newAccelDistance = millimeters;
        }
        accelDistance = newAccelDistance;
        cruise_speed = SQRT(2 * acceleration * newAccelDistance + sq(entry_speed));
        i_cruise_speed = 1 / cruise_speed;
        if (cruise_speed < leave_speed) {
            cruise_speed = leave_speed;
        }
        accelClocks = (cruise_speed - entry_speed) * i_acceleration;
        decelDistance = millimeters - accelDistance;
        decelClocks = (cruise_speed - leave_speed) * i_acceleration;
        plateau = 0;
    }

    block->shaper_data.move_start = move_head;

    float plateauClocks = plateau * i_cruise_speed;

    if (plateau == 0) {
        if (accelDistance > 0) {
            addMove(entry_speed, cruise_speed, acceleration, accelDistance, block->axis_r, accelClocks);
        }
        if (decelDistance > 0) {
            addMove(cruise_speed, leave_speed, -deceleration, decelDistance, block->axis_r, decelClocks);
        }
    } else {
        if (accelDistance > 0) {
            addMove(entry_speed, cruise_speed, acceleration, accelDistance, block->axis_r, accelClocks);
        }

        // LOG_I("p: %lf, s: %lf, t: %lf\n", plateau, cruise_speed, plateau / cruise_speed);
        addMove(cruise_speed, cruise_speed, 0, plateau, block->axis_r, plateauClocks);

        if (decelDistance > 0) {
            addMove(cruise_speed, leave_speed, -deceleration, decelDistance, block->axis_r, decelClocks);
        }
    }

#if ENABLED(LASER_POWER_INLINE_TRAPEZOID)
    if (smprinter.get_toolhead_type() == TH_TYPE_LASER) {
        auto &laser = block->laser;
        if (laser.power_pwm > 0) { // No need to care if power == 0
            uint32_t accelerate_steps , decelerate_steps;

            // cruise_speed maybe less than nominal_speed, so need to convert nominal_rate
            block->nominal_rate *= block->cruise_speed / block->nominal_speed;
            laser.power_pwm *= block->cruise_speed / block->nominal_speed;

            // nominal_rate is converted, so use cruise_speed to calculate initial_rate & final_rate
            block->initial_rate = CEIL(block->nominal_rate * block->initial_speed / block->cruise_speed),
            block->final_rate = CEIL(block->nominal_rate * block->final_speed / block->cruise_speed); // (steps per second)

            // calculate the acceleration steps and deceleration steps of major axis
            planner.calculate_major_axis(block, accelerate_steps, decelerate_steps);

            if (accelerate_steps > 0) {
                const uint16_t entry_power = laser.power_pwm * block->initial_speed / block->cruise_speed; // Power on block entry
                // Speedup power
                const uint16_t entry_power_diff = laser.power_pwm - entry_power;
                if (entry_power_diff) {
                    // increase power per [entry_per] steps
                    laser.entry_per = (uint16_t)(accelerate_steps / entry_power_diff + 0.5);
                    laser.power_entry = entry_power;
                    // LOG_I("la: ts: %u, as: %u, ep: %u, pen: %u, tp: %u\r\n", block->step_event_count, accelerate_steps, laser.entry_per, laser.power_entry, laser.power_pwm);
                }
                else {
                    laser.entry_per = 0;
                    laser.power_entry = laser.power_pwm;
                }
            }
            else {
                // no acceleration phase
                laser.entry_per = 0;
                laser.power_entry = laser.power_pwm;
            }

            if (decelerate_steps > 0) {
                // Slowdown power
                // decrease power per [exit_per] steps
                const uint16_t exit_power = laser.power_pwm * block->final_speed / block->cruise_speed, // Power on block entry
                            exit_power_diff = laser.power_pwm - exit_power;
                if (exit_power_diff) {
                    laser.exit_per = (uint16_t)(decelerate_steps / exit_power_diff + 0.5);
                    laser.power_exit = exit_power;
                    // LOG_I("la: ts: %u, da: %u, ep: %u, pex: %u, tp: %u\r\n", block->step_event_count, block->decelerate_after, laser.exit_per, laser.power_exit, laser.power_pwm);
                }
                else {
                    laser.exit_per = 0;
                    laser.power_exit = laser.power_pwm;
                }
            }
            else {
                // // no deceleration phase
                laser.exit_per = 0;
                laser.power_exit = laser.power_pwm;
            }
        }
        else {
            laser.power_entry = laser.power_pwm;
            laser.power_exit = laser.power_pwm;
            laser.exit_per = 0;
            laser.entry_per = 0;
            block->accelerate_until = 0;
            block->decelerate_after = block->step_event_count;
            laser.status.isEnabled = false;
        }
        block->laser.status.isPlanned = true;
    }
#endif

    block->shaper_data.block_time = accelClocks + plateauClocks + decelClocks;

    block->shaper_data.move_end = prevMoveIndex(move_head);

    block->cruise_speed = cruise_speed * 1000;

    Move& end_move = moves[block->shaper_data.move_end];
    for (int i = 0; i < LINEAR_AXES - 1; ++i) {
        float p1 = end_move.end_pos[i];
        end_move.end_pos[i] = LROUND(end_move.end_pos[i]);
        if (ABS(p1 - end_move.end_pos[i]) > 1) {
            LOG_I("error LROUND: %lf, %lf\n", p1, end_move.end_pos[i]);
        }
        // if (i == 0 || i == 1) {
            // LOG_I("%d %lf %lf\n", i, tmp, end_move.end_pos[i]);
        // }
    }
    {
        double p1 = end_move.end_pos_e;
        end_move.end_pos_e = (int64_t)(end_move.end_pos_e + 0.5);
        if (ABS(p1 - end_move.end_pos_e) > 1) {
            LOG_I("error E LROUND: %lf, %lf\n", p1, end_move.end_pos_e);
        }

        p1 = end_move.end_pos_j;
        end_move.end_pos_j = (int64_t)(end_move.end_pos_j + (end_move.end_pos_j >= 0 ? 0.5 : -0.5));
        if (ABS(p1 - end_move.end_pos_j) > 1) {
            LOG_I("error B LROUND: %lf, %lf\n", p1, end_move.end_pos_j);
        }
    }

    block->shaper_data.last_print_time = moves[block->shaper_data.move_end].end_t;
}

void MoveQueue::setMove(uint8_t move_index, float start_v, float end_v, float accelerate, float distance, xyze_float_t& axis_r, float t, uint8_t flag) {
    Move &move = moves[move_index];

    move.start_v = start_v;
    move.end_v = end_v;

    move.accelerate = accelerate;
    move.distance = distance;

    move.t = t;
    move.axis_r[0] = axis_r.x;
    move.axis_r[1] = axis_r.y;
    move.axis_r[2] = axis_r.z;
    move.axis_r[3] = axis_r.i;
    move.axis_r[4] = axis_r.j;
    move.axis_r[5] = axis_r.e;

    Move& last_move = moves[prevMoveIndex(move_index)];
    move.start_t = is_first ? 0 : last_move.end_t;
    move.end_t = move.start_t + move.t;

    // LOG_I("move_index: %d %lf %d %lf\n", move_index, t, flag, move.start_t.toFloat());

    float last_end_v = is_first? 0 : last_move.end_v;
    if (!IS_ZERO(last_end_v - move.start_v)) {
        // LOG_I("error v: %lf, %lf\n", last_end_v,  move.start_v);
    }

    for (int i = 0; i < LINEAR_AXES - 1; ++i) {
        move.start_pos[i] = is_first ? 0 : last_move.end_pos[i];
        move.end_pos[i] = move.start_pos[i] + move.distance * move.axis_r[i];

        // if (IS_ZERO(move.end_pos[i]) && !IS_ZERO(move.start_pos[i])) {
        //     LOG_I("debug1: %lf, %lf\n", move.start_pos[i], move.end_pos[i]);
        // }

        // when quick home xy, distance will be 410 * 15 = 615
        if (i <= 1 && (move.end_pos[i] < -56000 || move.end_pos[i] > 56000)) {
            LOG_I("debug: %d, %lf, %lf\n", i, move.distance, move.end_pos[i]);
        }
    }
    move.start_pos_e = is_first ? 0 : last_move.end_pos_e;
    move.end_pos_e = move.start_pos_e + move.distance * move.axis_r[E_AXIS];

    move.start_pos_j = is_first ? 0 : last_move.end_pos_j;
    move.end_pos_j = move.start_pos_j + move.distance * move.axis_r[J_AXIS];

    is_first = false;

    // LOG_I("v1: %lf, v2: %lf, s_p: %lf, e_p: %lf, t: %lf\n", start_v, end_v, move.start_pos[3], move.end_pos[3], t);

    // LOG_I("%d %lf %lf %lf %lf %lf %lf %lf %lf %lf %d\n", move_head, move.t, move.start_t.toFloat(), move.end_t.toFloat(), start_v, distance, move.start_pos[0], move.end_pos[0], move.start_pos[1], move.end_pos[1], move.flag);

    // LOG_I("move, %lf %lf %lf %lf %lf %lf\n", move.start_t.toDouble(), move.end_t.toDouble(), move.accelerate, move.axis_r[0], move.axis_r[1], move.distance);

//    if (flag != 1) {
//        Move& last_move = moves[prevMoveIndex(move_index)];
//        for (int i = 0; i < NUM_AXIS; ++i) {
//            move.start_pos[i] = last_move.end_pos[i];
//            move.end_pos[i] = move.start_pos[i] + move.distance * move.axis_r[i];
//        }
//    } else {
//        for (int i = 0; i < NUM_AXIS; ++i) {
//            move.start_pos[i] = 0;
//            move.end_pos[i] = move.start_pos[i] + move.distance * move.axis_r[i];
//        }
//    }

    move.flag = flag;

    // printf("move: end_t: %lf, pos: %lf, start_v: %lf, t: %lf, flag: %d\n", move.end_t.toDouble(), move.end_pos[0], move.start_v, move.t, move.flag);
}

uint8_t MoveQueue::addMove(float start_v, float end_v, float accelerate, float distance, xyze_float_t& axis_r, float t, uint8_t flag) {
    setMove(move_head, start_v, end_v, accelerate, distance, axis_r, t, flag);

    uint8_t move_index = move_head;

    move_head = nextMoveIndex(move_head);

//    setMoveEnd();

    return move_index;
}

uint8_t MoveQueue::addEmptyMove(float time) {
    return addMove(0, 0, 0, 0, ZERO_AXIS_R, time, MOVE_FLAG_START);
}

uint8_t MoveQueue::addMoveStart() {
    return addMove(0, 0, 0, 0, ZERO_AXIS_R, EMPTY_TIME, MOVE_FLAG_START);
}

uint8_t MoveQueue::addMoveEnd() {
    return addMove(0, 0, 0, 0, ZERO_AXIS_R, EMPTY_TIME, MOVE_FLAG_END);
}

void MoveQueue::setMoveEnd() {
    setMove(move_head, 0, 0, 0, 0, ZERO_AXIS_R, EMPTY_TIME, MOVE_FLAG_END);
}


uint8_t MoveQueue::calculateMoveStart(uint8_t index, float delta_window) {
    if (index == move_tail) {
        return index;
    }
    uint8_t move_shaped_start = prevMoveIndex(index);
    float t = moves[move_shaped_start].t;

    while (t < delta_window && move_shaped_start != move_tail) {
        move_shaped_start = prevMoveIndex(move_shaped_start);
        t += moves[move_shaped_start].t;
    }
    return move_shaped_start;
};

uint8_t MoveQueue::calculateMoveEnd(uint8_t index, float delta_window) {
    if (index == move_head) {
        return index;
    }
    uint8_t move_shaped_end = nextMoveIndex(index);
    float t = moves[move_shaped_end].t;

    while (t < delta_window && move_shaped_end != move_head) {
        move_shaped_end = nextMoveIndex(move_shaped_end);
        t += moves[move_shaped_end].t;
    }
    return move_shaped_end;
};

void MoveQueue::updateMoveTail(uint8_t index) {
    move_tail = index;
};

void MoveQueue::initMoveTimeAndPos(uint8_t move_shaped_start, uint8_t move_start, uint8_t move_shaped_end) {
    float start_t = 0;
    // float start_pos[NUM_AXIS] = {0};

    uint8_t index = move_start;
    Move *move;
    while (index != nextMoveIndex(move_shaped_end)) {
        move = &moves[index];

        move->start_t = start_t;
        start_t += move->t;
        move->end_t = start_t;

//        for (int i = 0; i < NUM_AXIS; ++i) {
//            move->start_pos[i] = start_pos[i];
//            start_pos[i] += move->distance * move->axis_r[i];
//            move->end_pos[i] = start_pos[i];
//        }

        index = nextMoveIndex(index);
    }

    start_t = 0;

//    for (int i = 0; i < NUM_AXIS; ++i) {
//        start_pos[i] = 0;
//    }

    index = prevMoveIndex(move_start);

    while (index != prevMoveIndex(move_shaped_start)) {
        move = &moves[index];

        move->end_t = start_t;
        start_t -= move->t;
        move->start_t = start_t;

//        for (int i = 0; i < NUM_AXIS; ++i) {
//            move->end_pos[i] = start_pos[i];
//            start_pos[i] -= move->distance * move->axis_r[i];
//            move->start_pos[i] = start_pos[i];
//        }

        index = prevMoveIndex(index);
    }
}

float MoveQueue::getAxisPositionAcrossMoves(int move_index, int axis, time_double_t time, int move_shaped_start, int move_shaped_end) {
    while (time < moves[move_index].start_t && move_index != move_shaped_start) {
        move_index = prevMoveIndex(move_index);
    }
    while (time > moves[move_index].end_t && move_index != move_shaped_end) {
        move_index = nextMoveIndex(move_index);
    }

    return getAxisPosition(move_index, axis, time);
}

float MoveQueue::getAxisPosition(int move_index, int axis, time_double_t time) {
    Move *move = &moves[move_index];
    float axis_r = move->axis_r[axis];
    float start_pos = move->start_pos[axis];

    float delta_time = time - move->start_t;

    float move_dist = (move->start_v + 0.5f * move->accelerate * delta_time) * delta_time;

    return start_pos + axis_r * move_dist;
}