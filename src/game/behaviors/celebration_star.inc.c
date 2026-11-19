// celebration_star.inc.c

void bhv_celebration_star_init(void) {
    vec3_copy(&o->oHomeVec, gMarioObject->header.gfx.pos);
    vec3_copy(&o->oPosVec, &o->oHomeVec);
    obj_add_up(o, &o->oPosVec, 30.0f);
    o->oMoveAngleYaw = obj_yaw_in_frame(o, gMarioObject, gMarioObject->header.gfx.angle[1]) + 0x8000;
    o->oCelebStarDiameterOfRotation = 100;
    o->oFaceAnglePitch = 0;
    if (obj_has_model(gMarioState->interactObj, MODEL_BOWSER_KEY)) {
        o->oFaceAngleRoll = 0xC000;
        cur_obj_scale(0.1f);
        o->oCelebStarIsBowserKey = TRUE;
    } else {
        o->oFaceAngleRoll = 0;
        cur_obj_scale(0.4f);
        o->oCelebStarIsBowserKey = FALSE;
    }
}

void celeb_star_act_spin_around_mario(void) {
    f32 up = obj_height_difference(o, &o->oPosVec, &o->oHomeVec) + 5.0f;
    obj_offset_forward(o, &o->oPosVec, &o->oHomeVec, o->oMoveAngleYaw, o->oCelebStarDiameterOfRotation / 2, up);
    o->oFaceAngleYaw += 0x1000;
    o->oMoveAngleYaw += 0x2000;

    if (o->oTimer == 40) {
        o->oAction = CELEB_STAR_ACT_FACE_CAMERA;
    }

    if (o->oTimer < 35) {
        spawn_object(o, MODEL_SPARKLES, bhvCelebrationStarSparkle);
        o->oCelebStarDiameterOfRotation++;
    } else {
        o->oCelebStarDiameterOfRotation -= 20;
    }
}

void celeb_star_act_face_camera(void) {
    if (o->oTimer < 10) {
        if (o->oCelebStarIsBowserKey == FALSE) {
            cur_obj_scale((f32) o->oTimer / 10.0f);
        } else {
            cur_obj_scale((f32) o->oTimer / 30.0f);
        }
        o->oFaceAngleYaw += 0x1000;
    } else {
        o->oFaceAngleYaw = obj_yaw_in_frame(o, gMarioObject, gMarioObject->header.gfx.angle[1]);
    }

    if (o->oTimer == 59) {
        o->activeFlags = ACTIVE_FLAG_DEACTIVATED;
    }
}

void bhv_celebration_star_loop(void) {
    switch (o->oAction) {
        case CELEB_STAR_ACT_SPIN_AROUND_MARIO:
            celeb_star_act_spin_around_mario();
            break;

        case CELEB_STAR_ACT_FACE_CAMERA:
            celeb_star_act_face_camera();
            break;
    }
}

void bhv_celebration_star_sparkle_loop(void) {
    obj_add_up(o, &o->oPosVec, -15.0f);

    if (o->oTimer == 12) {
        o->activeFlags = ACTIVE_FLAG_DEACTIVATED;
    }
}

void bhv_star_key_collection_puff_spawner_loop(void) {
    spawn_mist_particles_variable(0, 10, 30.0f);
    o->activeFlags = ACTIVE_FLAG_DEACTIVATED;
}
