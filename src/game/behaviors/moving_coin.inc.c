// moving_coin.inc.c

static void coin_step(void) {
    cur_obj_move_rolling();
    obj_check_floor_death();
    if (o->oMoveFlags & OBJ_MOVE_BOUNCE) {
        cur_obj_play_sound_2(SOUND_GENERAL_COIN_DROP);
    }
}

void moving_coin_flicker(void) {
    coin_step();
    obj_flicker_and_disappear(o, 0);
}

void coin_collected(void) {
    spawn_object(o, MODEL_SPARKLES, bhvCoinSparklesSpawner);
    o->activeFlags = ACTIVE_FLAG_DEACTIVATED;
}

void bhv_moving_yellow_coin_loop(void) {
    switch (o->oAction) {
        case MOV_YCOIN_ACT_IDLE:
            coin_step();

            if (o->oTimer < 10) {
                cur_obj_become_intangible();
            } else {
                cur_obj_become_tangible();
            }

            if (o->oTimer > 300) {
                o->oAction = MOV_YCOIN_ACT_BLINKING;
            }
            break;

        case MOV_YCOIN_ACT_BLINKING:
            moving_coin_flicker();
            break;

        case OBJ_ACT_LAVA_DEATH:
#ifdef COIN_LAVA_FLICKER
            moving_coin_flicker();
#else
            obj_mark_for_deletion(o);
#endif
            break;

        case OBJ_ACT_DEATH_PLANE_DEATH:
            obj_mark_for_deletion(o);
            break;
    }

    if (o->oInteractStatus & INT_STATUS_INTERACTED) {
        coin_collected();
        o->oInteractStatus = INT_STATUS_NONE;
    }
}

void bhv_moving_blue_coin_loop(void) {
    switch (o->oAction) {
        case MOV_BCOIN_ACT_STILL:
            if (is_point_within_radius_of_mario(o->oPosX, o->oPosY, o->oPosZ, 1500)) {
                o->oAction = MOV_BCOIN_ACT_MOVING;
            }
            break;

        case MOV_BCOIN_ACT_MOVING:
            cur_obj_move_rolling();

            if (o->oMoveFlags & OBJ_MOVE_MASK_ON_GROUND) {
                o->oForwardVel += 25.0f;
                if (o->oMoveFlags & OBJ_MOVE_BOUNCE) {
                    cur_obj_play_sound_2(SOUND_GENERAL_COIN_DROP);
                }
            } else {
                o->oForwardVel *= 0.98f;
            }

            if (o->oForwardVel > 75.0f) {
                o->oForwardVel = 75.0f;
            }

            obj_flicker_and_disappear(o, 600);
            break;
    }

    if (o->oInteractStatus & INT_STATUS_INTERACTED) {
        coin_collected();
        o->oInteractStatus = INT_STATUS_NONE;
    }
}

void blue_coin_sliding_away_from_mario(void) {
    o->oForwardVel = 15.0f;
    o->oMoveAngleYaw = o->oAngleToMario + 0x8000;

    coin_step();
    if (o->oMoveFlags & OBJ_MOVE_BOUNCE) {
        o->oVelY += 18.0f;
    }

    if (o->oMoveFlags & (OBJ_MOVE_HIT_WALL | OBJ_MOVE_HIT_EDGE)) {
        o->oAction = MOV_BCOIN_ACT_STOPPED;
    }

    if (!is_point_within_radius_of_mario(o->oPosX, o->oPosY, o->oPosZ, 1000)) {
        o->oAction = MOV_BCOIN_ACT_SLOWING_DOWN;
    }
}

void blue_coin_sliding_slow_down(void) {
    coin_step();
    o->oForwardVel *= 0.98f;

    if (is_point_within_radius_of_mario(o->oPosX, o->oPosY, o->oPosZ, 500)) {
        o->oAction = MOV_BCOIN_ACT_MOVING;
    }

    if (o->oTimer > 150) {
        o->oAction = MOV_BCOIN_ACT_STOPPED;
    }
}

void bhv_blue_coin_sliding_loop(void) {
    switch (o->oAction) {
        case MOV_BCOIN_ACT_STILL:
            if (is_point_within_radius_of_mario(o->oPosX, o->oPosY, o->oPosZ, 500)) {
                o->oAction = MOV_BCOIN_ACT_MOVING;
            }

            set_object_visibility(o, 3000);
            break;

        case MOV_BCOIN_ACT_MOVING:
            blue_coin_sliding_away_from_mario();
            break;

        case MOV_BCOIN_ACT_SLOWING_DOWN:
            blue_coin_sliding_slow_down();
            set_object_visibility(o, 3000);
            break;

        case MOV_BCOIN_ACT_STOPPED:
            coin_step();
            if (o->oTimer > 60) {
                o->oAction = MOV_BCOIN_ACT_FLICKERING;
            }
            break;

        case MOV_BCOIN_ACT_FLICKERING:
            moving_coin_flicker();
            break;

        case OBJ_ACT_LAVA_DEATH:
#ifdef COIN_LAVA_FLICKER
            o->oAction = MOV_BCOIN_ACT_FLICKERING;
#else
            obj_mark_for_deletion(o);
#endif
            break;

        case OBJ_ACT_DEATH_PLANE_DEATH:
            obj_mark_for_deletion(o);
            break;
    }

    if (o->oInteractStatus & INT_STATUS_INTERACTED) {
        coin_collected();
        o->oInteractStatus = INT_STATUS_NONE;
    }
}

void bhv_blue_coin_jumping_loop(void) {
    switch (o->oAction) {
        case MOV_BCOIN_ACT_STILL:
            if (o->oTimer == 0) {
                cur_obj_become_intangible();
                o->oVelY = 50.0f;
            }

            cur_obj_move_rolling();

            if (o->oTimer == 15) {
                cur_obj_become_tangible();
                o->oAction = MOV_BCOIN_ACT_MOVING;
            }
            break;

        case MOV_BCOIN_ACT_MOVING:
            blue_coin_sliding_away_from_mario();
            break;

        case MOV_BCOIN_ACT_SLOWING_DOWN:
            blue_coin_sliding_slow_down();
            set_object_visibility(o, 3000);
            break;

        case MOV_BCOIN_ACT_STOPPED:
            coin_step();
            if (o->oTimer > 60) {
                o->oAction = MOV_BCOIN_ACT_FLICKERING;
            }
            break;

        case MOV_BCOIN_ACT_FLICKERING:
            moving_coin_flicker();
            break;
    }

    if (o->oInteractStatus & INT_STATUS_INTERACTED) {
        coin_collected();
        o->oInteractStatus = INT_STATUS_NONE;
    }
}
