// orange_number.inc.c

void bhv_orange_number_init(void) {
    o->oAnimState = o->oBehParams2ndByte;
    o->oVelY = 26.0f;
    obj_set_height(o, &o->oHomeVec, obj_height(o, &o->oPosVec));
}

void bhv_orange_number_loop(void) {
#ifdef DIALOG_INDICATOR
    if (o->oAnimState <= ORANGE_NUMBER_9) {
#endif
        obj_add_up(o, &o->oPosVec, o->oVelY);
        o->oVelY -= 2.0f;

        if (o->oVelY < -21.0f) {
            o->oVelY = 14.0f;
        }

        s16 rightYaw = obj_yaw_in_frame(o, NULL, gCamera->nextYaw + 0x4000);
        obj_offset_forward(o, &o->oPosVec, &o->oHomeVec, rightYaw, o->oOrangeNumberOffset,
                           obj_height_difference(o, &o->oPosVec, &o->oHomeVec));

        if (o->oTimer == 35) {
            struct Object *sparkleObj = spawn_object(o, MODEL_SPARKLES, bhvCoinSparklesSpawner);
            obj_add_up(sparkleObj, &sparkleObj->oPosVec, -30.0f);
            obj_mark_for_deletion(o);
        }
#ifdef DIALOG_INDICATOR
    } else if (o->oTimer >= 1 || gMarioState->action == ACT_READING_SIGN) {
        obj_mark_for_deletion(o);
    }
#endif
}
