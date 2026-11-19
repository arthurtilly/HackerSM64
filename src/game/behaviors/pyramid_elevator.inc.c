
/**
 * Behaviors for bhvPyramidElevator and bhvPyramidElevatorTrajectoryMarkerBall.
 *
 * This controls the elevator that descends Shifting Sand Land's pyramid, as
 * well as the small marker balls that demarcate its trajactory.
 */

/**
 * Generate the ten trajectory marker balls that indicate where the elevator
 * moves.
 */
void bhv_pyramid_elevator_init(void) {
    s32 i;

    for (i = 0; i < 10; i++) {
        struct Object *ball = spawn_object(o, MODEL_TRAJECTORY_MARKER_BALL,
                                           bhvPyramidElevatorTrajectoryMarkerBall);
        obj_offset_up(o, &ball->oPosVec, &o->oHomeVec, 4600 - i * 460 - o->oHomeY);
    }
}

void bhv_pyramid_elevator_loop(void) {
    f32 bottom = 128.0f - o->oHomeY;
    switch (o->oAction) {
        /**
         * Do not move until Mario stands on the elevator. When he does,
         * transition to the starting state.
         */
        case PYRAMID_ELEVATOR_IDLE:
            if (gMarioObject->platform == o) {
                o->oAction = PYRAMID_ELEVATOR_START_MOVING;
            }
            break;

        /**
         * Use a sine wave to start the elevator's movement with a small jolt.
         * After a certain amount of time, transition to a constant-velocity state.
         */
        case PYRAMID_ELEVATOR_START_MOVING:
            obj_set_height(o, &o->oPosVec, obj_height(o, &o->oHomeVec) - sins(o->oTimer * 0x1000) * 10.0f);
            if (o->oTimer == 8) {
                o->oAction = PYRAMID_ELEVATOR_CONSTANT_VELOCITY;
            }
            break;

        /**
         * Move downwards with constant velocity. Once at the bottom of the
         * track, transition to the final state.
         */
        case PYRAMID_ELEVATOR_CONSTANT_VELOCITY:
            o->oVelY = -10.0f;
            obj_add_up(o, &o->oPosVec, o->oVelY);
            if (obj_height_difference(o, &o->oPosVec, &o->oHomeVec) < bottom) {
                obj_offset_up(o, &o->oPosVec, &o->oHomeVec, bottom);
                o->oAction = PYRAMID_ELEVATOR_AT_BOTTOM;
            }
            break;

        /**
         * Use a sine wave to stop the elevator's movement with a small jolt.
         * Then, remain at the bottom of the track.
         */
        case PYRAMID_ELEVATOR_AT_BOTTOM:
            if (o->oTimer < 8) bottom += sins(o->oTimer * 0x1000) * 10.0f;
            else o->oVelY = 0;
            obj_offset_up(o, &o->oPosVec, &o->oHomeVec, bottom);
            break;
    }
}

/**
 * Deactivate the trajectory marker balls if the elevator is not moving.
 * Otherwise, set their scale.
 */
void bhv_pyramid_elevator_trajectory_marker_ball_loop(void) {
    struct Object *elevator;

    cur_obj_scale(0.15f);
    elevator = cur_obj_nearest_object_with_behavior(bhvPyramidElevator);

    if (elevator != NULL) {
        if (elevator->oAction != PYRAMID_ELEVATOR_IDLE) {
            o->activeFlags = ACTIVE_FLAG_DEACTIVATED;
        }
    }
}
