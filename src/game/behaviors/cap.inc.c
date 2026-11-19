// cap.inc.c

static struct ObjectHitbox sCapHitbox = {
    /* interactType:      */ INTERACT_CAP,
    /* damageOrCoinValue: */ 0,
    /* health:            */ 0,
    /* numLootCoins:      */ 0,
    /* radius:            */ 60,
    /* offset:            */ 25,
    /* hurtboxRadius:     */ 51,
    /* hurtboxOffset:     */ 29,
};

s32 cap_set_hitbox(void) {
    obj_set_hitbox(o, &sCapHitbox);

    if (o->oInteractStatus & INT_STATUS_INTERACTED) {
        o->activeFlags = ACTIVE_FLAG_DEACTIVATED;
        o->oInteractStatus = INT_STATUS_NONE;
        return TRUE;
    }

    return FALSE;
}

void cap_check_quicksand(void) {
    if (o->oFloor == NULL) {
        return;
    }

    switch (o->oFloor->type) {
        case SURFACE_DEATH_PLANE:
            o->activeFlags = ACTIVE_FLAG_DEACTIVATED;
            break;

        case SURFACE_SHALLOW_QUICKSAND:
        case SURFACE_DEEP_QUICKSAND:
        case SURFACE_QUICKSAND:
            o->oAction = CAP_ACT_QUICKSAND;
            o->oForwardVel = 0.0f;
            break;

        case SURFACE_DEEP_MOVING_QUICKSAND:
        case SURFACE_SHALLOW_MOVING_QUICKSAND:
        case SURFACE_MOVING_QUICKSAND:
            o->oAction = CAP_ACT_MOVING_QUICKSAND;
            o->oMoveAngleYaw = (o->oFloor->force & 0xFF) << 8;
            o->oForwardVel = 8 + 2 * (0 - ((o->oFloor->force & 0xFF00) >> 8));
            break;

        case SURFACE_INSTANT_QUICKSAND:
            o->oAction = CAP_ACT_INSTANT_QUICKSAND;
            o->oForwardVel = 0.0f;
            break;

        case SURFACE_INSTANT_MOVING_QUICKSAND:
            o->oAction = CAP_ACT_INSTANT_MOVING_QUICKSAND;
            o->oMoveAngleYaw = (o->oFloor->force & 0xFF) << 8;
            o->oForwardVel = 8 + 2 * (0 - ((o->oFloor->force & 0xFF00) >> 8));
            break;
    }
}

void cap_sink_quicksand(void) {
    switch (o->oAction) {
        case CAP_ACT_QUICKSAND:
            if (o->oTimer < 10) {
                o->oGraphYOffset += -1.0f;
                o->oFaceAnglePitch = 0x2000;
            }
            break;

        case CAP_ACT_MOVING_QUICKSAND:
            if (o->oTimer < 10) {
                o->oGraphYOffset += -3.0f;
            }
            o->oFaceAnglePitch = 0x2000;
            break;

        case CAP_ACT_INSTANT_QUICKSAND:
            o->oGraphYOffset += -1.0f;
            if (o->oTimer > 20) {
                o->activeFlags = ACTIVE_FLAG_DEACTIVATED;
            }
            break;

        case CAP_ACT_INSTANT_MOVING_QUICKSAND:
            o->oGraphYOffset += -6.0f;
            if (o->oTimer > 20) {
                o->activeFlags = ACTIVE_FLAG_DEACTIVATED;
            }
            o->oFaceAnglePitch = 0x2000;
            break;
    }

    cap_check_quicksand();
}

void cap_scale_vertically(void) {
    o->oCapScaleAngle += 0x2000;
    o->header.gfx.scale[1] = coss(o->oCapScaleAngle) * 0.3f + 0.7f;
    if (o->oCapScaleAngle == 0x10000) {
        o->oCapScaleAngle = 0;
        o->oCapDoScaleVertically = FALSE;
    }
}

static void cap_move(s32 squashOnBounce) {
    cur_obj_move_rolling();

    if (o->oAction == CAP_ACT_MOVE) {
        o->oFaceAngleYaw += o->oForwardVel * 128.0f;
        if (o->oMoveFlags & OBJ_MOVE_MASK_ON_ANY_GROUND) {
            cap_check_quicksand();
            if (squashOnBounce && (o->oMoveFlags & OBJ_MOVE_BOUNCE)) {
                o->oCapDoScaleVertically = TRUE;
                o->oVelY = 0.0f;
                o->oFaceAnglePitch = 0;
            }
        }
        if (o->oCapDoScaleVertically) {
            cap_scale_vertically();
        }
    } else {
        cap_sink_quicksand();
    }

    if (cur_obj_is_near_floor()) {
        cur_obj_align_gfx_with_floor();
    }
}

void bhv_powerup_cap_loop(void) {
    cap_move(!cur_obj_has_behavior(bhvMetalCap));
    if (o->oTimer > 20) {
        cur_obj_become_tangible();
    }
    if (o->oTimer > 300) {
        obj_flicker_and_disappear(o, 300);
    }
    cap_set_hitbox();
}

void bhv_normal_cap_init(void) {
    o->oOpacity = 255;

    save_file_set_cap_pos(o->oPosX, o->oPosY, o->oPosZ);
}

void normal_cap_set_save_flags(void) {
    save_file_clear_flags(SAVE_FLAG_CAP_ON_GROUND);

    switch (gCurrCourseNum) {
        case COURSE_SSL:
            save_file_set_flags(SAVE_FLAG_CAP_ON_KLEPTO);
            break;

        case COURSE_SL:
            save_file_set_flags(SAVE_FLAG_CAP_ON_MR_BLIZZARD);
            break;

        case COURSE_TTM:
            save_file_set_flags(SAVE_FLAG_CAP_ON_UKIKI);
            break;

        default:
            save_file_set_flags(SAVE_FLAG_CAP_ON_KLEPTO);
            break;
    }
}

void bhv_normal_cap_loop(void) {
    if (o->oAction == CAP_ACT_MOVE) {
        o->oFaceAnglePitch += o->oForwardVel * 80.0f;
        if (o->oMoveFlags & OBJ_MOVE_MASK_ON_GROUND) {
            o->oForwardVel *= 0.89f;
        }
    }
    cap_move(TRUE);

    if ((s32) o->oForwardVel != 0) {
        save_file_set_cap_pos(o->oPosX, o->oPosY, o->oPosZ);
    }

    if (o->activeFlags == ACTIVE_FLAG_DEACTIVATED) {
        normal_cap_set_save_flags();
    }

    if (cap_set_hitbox() == TRUE) {
        save_file_clear_flags(SAVE_FLAG_CAP_ON_GROUND);
    }
}
