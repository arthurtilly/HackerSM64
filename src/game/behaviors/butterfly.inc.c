// butterfly.inc.c

void bhv_butterfly_init(void) {
    cur_obj_init_animation(BUTTERFLY_ANIM_RESTING);
    o->oButterflyYPhase = random_float() * 100.0f;
    o->header.gfx.animInfo.animFrame = random_float() * 7.0f;
    vec3f_copy(&o->oHomeVec, &o->oPosVec);
}

void butterfly_step(s32 speed) {
    s16 yaw = o->oMoveAngleYaw;
    s16 pitch = o->oMoveAnglePitch;
    s16 yPhase = o->oButterflyYPhase;
    f32 floorY;

    o->oVelX = sins(yaw) * (f32) speed;
    o->oVelY = -sins(pitch) * (f32) speed;
    o->oVelZ = coss(yaw) * (f32) speed;
    if (o->oAction == BUTTERFLY_ACT_FOLLOW_MARIO) o->oVelY -= coss((s32)(yPhase * 655.36f)) * 5.0f;
    cur_obj_move_using_vel();

    floorY = find_actor_floor(o, &o->oPosVec, 0.0f, NULL);

    if (obj_height(o, &o->oPosVec) < floorY + 2.0f) {
        obj_set_height(o, &o->oPosVec, floorY + 2.0f);
    }

    o->oButterflyYPhase++;
    if (o->oButterflyYPhase > 100) {
        o->oButterflyYPhase = 0;
    }
}

void butterfly_calculate_angle(void) {
    Vec3f target;
    obj_local_delta(o, target, &gMarioObject->oPosVec, &o->oPosVec);
    f32 sway = 5 * o->oButterflyYPhase / 4;
    s16 yaw = atan2s(target[2] + sway, target[0] + sway);
    s16 pitch = atan2s(sqrtf(sqr(target[0]) + sqr(target[2])), -target[1] - sway - 64.0f);
    o->oMoveAngleYaw = approach_angle(o->oMoveAngleYaw, yaw, 0x300);
    o->oMoveAnglePitch = approach_angle(o->oMoveAnglePitch, pitch, 0x500);
}

void butterfly_act_rest(void) {
    if (is_point_within_radius_of_mario(o->oPosX, o->oPosY, o->oPosZ, 1000)) {
        cur_obj_init_animation(BUTTERFLY_ANIM_FLYING);

        o->oAction = BUTTERFLY_ACT_FOLLOW_MARIO;
        o->oMoveAngleYaw = obj_yaw_in_frame(o, gMarioObject, gMarioObject->header.gfx.angle[1]);
    }
}

void butterfly_act_follow_mario(void) {
    butterfly_calculate_angle();

    butterfly_step(7);

    if (!is_point_within_radius_of_mario(o->oHomeX, o->oHomeY, o->oHomeZ, 1200)) {
        o->oAction = BUTTERFLY_ACT_RETURN_HOME;
    }
}

void butterfly_act_return_home(void) {
    Vec3f homeDist;
    obj_local_delta(o, homeDist, &o->oHomeVec, &o->oPosVec);
    f32 homeDistXSq  = sqr(homeDist[0]);
    f32 homeDistZSq  = sqr(homeDist[2]);
    s16 hAngleToHome = atan2s(homeDist[2], homeDist[0]);
    s16 vAngleToHome = atan2s(sqrtf(homeDistXSq + homeDistZSq), -homeDist[1]);
    o->oMoveAngleYaw = approach_s16_symmetric(o->oMoveAngleYaw, hAngleToHome, 0x800);
    o->oMoveAnglePitch = approach_s16_symmetric(o->oMoveAnglePitch, vAngleToHome, 0x50);

    butterfly_step(7);

    if ((homeDistXSq + sqr(homeDist[1]) + homeDistZSq) < 144.0f) {
        cur_obj_init_animation(BUTTERFLY_ANIM_RESTING);

        o->oAction = BUTTERFLY_ACT_RESTING;
        vec3f_copy(&o->oPosVec, &o->oHomeVec);
    }
}

void bhv_butterfly_loop(void) {
    switch (o->oAction) {
        case BUTTERFLY_ACT_RESTING:
            butterfly_act_rest();
            break;

        case BUTTERFLY_ACT_FOLLOW_MARIO:
            butterfly_act_follow_mario();
            break;

        case BUTTERFLY_ACT_RETURN_HOME:
            butterfly_act_return_home();
            break;
    }

    set_object_visibility(o, 3000);
}
