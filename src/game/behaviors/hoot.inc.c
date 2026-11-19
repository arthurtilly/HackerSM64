// hoot.inc.c

void bhv_hoot_init(void) {
    cur_obj_init_animation(HOOT_ANIM_DEFAULT);
    obj_offset_local(o, &o->oHomeVec, &o->oPosVec, 800.0f, -150.0f, 300.0f);
    o->header.gfx.node.flags |= GRAPH_RENDER_INVISIBLE;

    cur_obj_become_intangible();
}

f32 hoot_find_next_floor(f32 dist) {
    Vec3f pos;
    obj_offset_forward(o, pos, &o->oPosVec, o->oMoveAngleYaw, dist, 0.0f);
    return find_actor_floor(o, pos, CELL_HEIGHT_LIMIT - obj_height(o, pos), NULL);
}

void hoot_floor_bounce(void) {
    f32 floorY = hoot_find_next_floor(375.0f);

    if (floorY + 75.0f > obj_height(o, &o->oPosVec)) {
        o->oMoveAnglePitch -= DEGREES(20);
    }

    floorY = hoot_find_next_floor(200.0f);
    if (floorY + 125.0f > obj_height(o, &o->oPosVec)) {
        o->oMoveAnglePitch -= DEGREES(40);
    }

    floorY = hoot_find_next_floor(0.0f);
    if (floorY + 125.0f > obj_height(o, &o->oPosVec)) {
        obj_set_height(o, &o->oPosVec, floorY + 125.0f);
    }

    if (o->oMoveAnglePitch < -DEGREES(120)) {
        o->oMoveAnglePitch = -DEGREES(120);
    }
}

static void hoot_fly(s32 speed, f32 flapRate) {
    s16 frame = o->header.gfx.animInfo.animFrame;
    f32 forward = coss(o->oMoveAnglePitch) * speed;
    o->oVelX = sins(o->oMoveAngleYaw) * forward;
    o->oVelZ = coss(o->oMoveAngleYaw) * forward;
    o->oVelY = -sins(o->oMoveAnglePitch) * speed - coss((s32)(frame * flapRate)) * 12.5f;
    cur_obj_move_using_vel();
    if (frame == 0) cur_obj_play_sound_2(SOUND_GENERAL_WING_FLAP);
}

void hoot_player_set_yaw(void) {
#ifdef HOOT_YAW_FIX
    s16 turnSpeed  = gMarioState->intendedMag * 0x20;
    o->oMoveAngleYaw = approach_s16_symmetric(o->oMoveAngleYaw, obj_yaw_in_frame(o, gMarioObject, gMarioState->intendedYaw), turnSpeed);
#else
    s16 stickX = gPlayer1Controller->rawStickX;
    s16 stickY = gPlayer1Controller->rawStickY;
    if (stickX < 10 && stickX > -10) stickX = 0;
    if (stickY < 10 && stickY > -10) stickY = 0;
    o->oMoveAngleYaw -= 5 * stickX;
#endif
}

static void hoot_surface_collision(const Vec3f prev) {
    if (resolve_actor_wall(o, &o->oPosVec, 10.0f, 50.0f, 1.0f, NULL)) {
        gMarioObject->oInteractStatus |= INT_STATUS_MARIO_DROP_FROM_HOOT;
    }

    if (absf(o->oPosX) > 8000.0f || absf(o->oPosZ) > 8000.0f) {
        vec3_copy(&o->oPosVec, prev);
    }
    f32 floorY = find_actor_floor(o, &o->oPosVec, 0.0f, NULL);
    if (floorY + 125.0f > obj_height(o, &o->oPosVec)) {
        obj_set_height(o, &o->oPosVec, floorY + 125.0f);
    }
}

static void hoot_act_ascent(void) {
    Vec3f delta;
    vec3_scale_dest(delta, &o->oPosVec, -1.0f);
    s16 angleToOrigin = obj_yaw_from_direction(o, delta);

    o->oMoveAngleYaw = approach_s16_symmetric(o->oMoveAngleYaw, angleToOrigin, 0x500);
    o->oMoveAnglePitch = DEGREES(290);

    if (o->oTimer >= 29) {
        cur_obj_play_sound_1(SOUND_ENV_WIND2);
        o->header.gfx.animInfo.animFrame = 1;
    }

    if (o->oPosY > 6500.0f) {
        o->oAction = HOOT_ACT_CARRY;
    }

    hoot_fly(60, 6553.6f);
}

void hoot_action_loop(void) {
    Vec3f prev;
    vec3_copy(prev, &o->oPosVec);

    switch (o->oAction) {
        case HOOT_ACT_ASCENT:
            hoot_act_ascent();
            break;

        case HOOT_ACT_CARRY:
            hoot_player_set_yaw();

            o->oMoveAnglePitch = 0x71C;

            if (o->oPosY < 2700.0f) {
                set_time_stop_flags(TIME_STOP_ENABLED | TIME_STOP_MARIO_AND_DOORS);

                if (cutscene_object_with_dialog(CUTSCENE_DIALOG, o, DIALOG_045)) {
                    clear_time_stop_flags(TIME_STOP_ENABLED | TIME_STOP_MARIO_AND_DOORS);
                    o->oAction = HOOT_ACT_TIRED;
                }
            }

            hoot_fly(20, 6553.6f);
            break;

        case HOOT_ACT_TIRED:
            hoot_player_set_yaw();

            o->oMoveAnglePitch = 0;

            hoot_fly(20, 6553.6f);

            if (o->oTimer > 60) {
                gMarioObject->oInteractStatus |= INT_STATUS_MARIO_DROP_FROM_HOOT;
            }
            break;
    }

    hoot_surface_collision(prev);
}

void hoot_turn_to_home(void) {
    Vec3f delta;
    obj_local_delta(o, delta, &o->oHomeVec, &o->oPosVec);
    s16 yawToHome = atan2s(delta[2], delta[0]);
    s16 pitchToHome = atan2s(sqrtf(sqr(delta[0]) + sqr(delta[2])), delta[1]);

    o->oMoveAngleYaw = approach_s16_symmetric(o->oMoveAngleYaw, yawToHome, 0x140);
    o->oMoveAnglePitch = approach_s16_symmetric(o->oMoveAnglePitch, -pitchToHome, 0x140);
}

void hoot_awake_loop(void) {
    if (o->oInteractStatus == TRUE) { //! Note: Not a flag, treated as a TRUE/FALSE statement
        hoot_action_loop();
        cur_obj_init_animation(HOOT_ANIM_HOLDING_MARIO);
    } else {
        cur_obj_init_animation(HOOT_ANIM_DEFAULT);

        hoot_turn_to_home();
        hoot_floor_bounce();
        hoot_fly(10, 3276.8f);

        o->oAction = 0;
        o->oTimer = 0;
    }

    set_object_visibility(o, 2000);
}

void bhv_hoot_loop(void) {
    switch (o->oHootAvailability) {
        case HOOT_AVAIL_ASLEEP_IN_TREE:
            if (is_point_within_radius_of_mario(o->oPosX, o->oPosY, o->oPosZ, 50)) {
                o->header.gfx.node.flags &= ~GRAPH_RENDER_INVISIBLE;
                o->oHootAvailability = HOOT_AVAIL_WANTS_TO_TALK;
            }
#ifdef HOOT_TREE_PARTICLES
            if (random_float() < 0.05f) {
                struct Object *obj   = spawn_object(o, MODEL_LEAVES, bhvTreeLeaf);
                f32 scale            = (random_float() * 3.0f);
                obj_scale(obj, scale);
                obj->oMoveAngleYaw   = random_u16();
                obj->oForwardVel     = ((random_float() *  5.0f) + 5.0f);
                obj->oVelY           =  (random_float() * 15.0f);
                obj->oFaceAnglePitch = random_u16();
                obj->oFaceAngleRoll  = random_u16();
                obj->oFaceAngleYaw   = random_u16();
            }
#endif
            break;

        case HOOT_AVAIL_WANTS_TO_TALK:
            hoot_awake_loop();

            if (set_mario_npc_dialog(MARIO_DIALOG_LOOK_UP) == MARIO_DIALOG_STATUS_SPEAK 
                && cutscene_object_with_dialog(CUTSCENE_DIALOG, o, DIALOG_044)) {
                set_mario_npc_dialog(MARIO_DIALOG_STOP);

                cur_obj_become_tangible();

                o->oHootAvailability = HOOT_AVAIL_READY_TO_FLY;
            }
            break;

        case HOOT_AVAIL_READY_TO_FLY:
            hoot_awake_loop();
            break;
    }
}
