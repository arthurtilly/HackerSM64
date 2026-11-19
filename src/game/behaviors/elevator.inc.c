// elevator.inc.c

static s16 sElevatorHeights[] = {
      -51,    0, 0,
     -461,    0, 0,
     -512,    0, 0,
    -2611,    0, 0,
    -2360,    0, 0,
      214,    0, 0,
      -50, 1945, 1,
};

void elevator_starting_shake(void) {
    cur_obj_play_sound_2(SOUND_GENERAL_QUIET_POUND1);
    cur_obj_shake_screen(SHAKE_POS_SMALL);
}

static f32 elevator_height(const struct Object *rider) {
    return obj_height_difference(o, &rider->oPosVec, &o->oHomeVec);
}

void elevator_act_0(void) {
    o->oVelY = 0;
    if (o->oElevatorType == 2) {
        if (gMarioObject->platform == o) {
            if (elevator_height(o) > o->oElevatorMidY) {
                o->oAction = 2;
            } else {
                o->oAction = 1;
            }
        }
    } else if (elevator_height(gMarioObject) > o->oElevatorMidY || o->oElevatorType == 1) {
        obj_offset_up(o, &o->oPosVec, &o->oHomeVec, o->oElevatorMaxY);
        if (gMarioObject->platform == o) {
            o->oAction = 2;
        }
    } else {
        obj_offset_up(o, &o->oPosVec, &o->oHomeVec, o->oElevatorMinY);
        if (gMarioObject->platform == o) {
            o->oAction = 1;
        }
    }
}

void elevator_act_1(void) {
    cur_obj_play_sound_1(SOUND_ENV_ELEVATOR1);
    if (o->oTimer == 0 && cur_obj_is_mario_on_platform()) {
        elevator_starting_shake();
    }
    approach_f32_signed(&o->oVelY, 10.0f, 2.0f);
    obj_add_up(o, &o->oPosVec, o->oVelY);
    if (elevator_height(o) > o->oElevatorMaxY) {
        obj_offset_up(o, &o->oPosVec, &o->oHomeVec, o->oElevatorMaxY);
        if (o->oElevatorType == 2 || o->oElevatorType == 1) {
            o->oAction = 3;
        } else if (elevator_height(gMarioObject) < o->oElevatorMidY) {
            o->oAction = 2;
        } else {
            o->oAction = 3;
        }
    }
}

void elevator_act_2(void) {
    cur_obj_play_sound_1(SOUND_ENV_ELEVATOR1);
    if (o->oTimer == 0 && cur_obj_is_mario_on_platform()) {
        elevator_starting_shake();
    }
    approach_f32_signed(&o->oVelY, -10.0f, -2.0f);
    obj_add_up(o, &o->oPosVec, o->oVelY);
    if (elevator_height(o) < o->oElevatorMinY) {
        obj_offset_up(o, &o->oPosVec, &o->oHomeVec, o->oElevatorMinY);
        if (o->oElevatorType == 1) {
            o->oAction = 4;
        } else if (o->oElevatorType == 2) {
            o->oAction = 3;
        } else if (elevator_height(gMarioObject) > o->oElevatorMidY) {
            o->oAction = 1;
        } else {
            o->oAction = 3;
        }
    }
}

void elevator_act_4(void) {
    o->oVelY = 0;
    if (o->oTimer == 0) {
        cur_obj_shake_screen(SHAKE_POS_SMALL);
        cur_obj_play_sound_2(SOUND_GENERAL_ELEVATOR_LAND);
    }
    if (!mario_is_in_air_action() && !cur_obj_is_mario_on_platform()) {
        o->oAction = 1;
    }
}

void elevator_act_3(void) {
    o->oVelY = 0;
    if (o->oTimer == 0) {
        cur_obj_shake_screen(SHAKE_POS_SMALL);
        cur_obj_play_sound_2(SOUND_GENERAL_ELEVATOR_LAND);
    }
    if (!mario_is_in_air_action() && !cur_obj_is_mario_on_platform()) {
        o->oAction = 0;
    }
}

void bhv_elevator_init(void) {
    const s16 *heights = &sElevatorHeights[o->oBehParams2ndByte * 3];
    o->oElevatorMinY = heights[0] - o->oHomeY;
    o->oElevatorMaxY = heights[2] ? heights[1] - o->oHomeY : 0.0f;
    o->oElevatorMidY = (o->oElevatorMinY + o->oElevatorMaxY) * 0.5f;
    o->oElevatorType = heights[2] ? 2 : cur_obj_has_behavior(bhvRrElevatorPlatform);
}

ObjActionFunc sElevatorActions[] = {
    elevator_act_0,
    elevator_act_1,
    elevator_act_2,
    elevator_act_3,
    elevator_act_4,
};

void bhv_elevator_loop(void) {
    cur_obj_call_action_function(sElevatorActions);
}
