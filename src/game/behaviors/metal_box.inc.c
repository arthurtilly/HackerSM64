// metal_box.inc.c

struct ObjectHitbox sMetalBoxHitbox = {
    /* interactType:      */ INTERACT_NONE,
    /* damageOrCoinValue: */ 0,
    /* health:            */ 1,
    /* numLootCoins:      */ 0,
    /* radius:            */ 195,
    /* offset:            */ 135,
    /* hurtboxRadius:     */ 182,
    /* hurtboxOffset:     */ 135,
};

s32 check_if_moving_over_floor(f32 maxDist, f32 offset) {
    Vec3f pos;
    obj_offset_forward(o, pos, &o->oPosVec, o->oMoveAngleYaw, offset, 0.0f);
    f32 height = find_actor_floor(o, pos, 0.0f, NULL);
    return absf(height - obj_height(o, pos)) < maxDist;
}

void bhv_pushable_loop(void) {
    obj_set_hitbox(o, &sMetalBoxHitbox);
    o->oForwardVel = 0.0f;

    if (obj_check_if_collided_with_object(o, gMarioObject) && gMarioStates[0].flags & MARIO_PUSHING) {
        s16 angleToMario = obj_angle_to_object(o, gMarioObject);
        s16 marioYaw = obj_yaw_in_frame(o, gMarioObject, gMarioObject->oMoveAngleYaw);
        if (abs_angle_diff(angleToMario, marioYaw) > 0x4000) {
            o->oMoveAngleYaw = (s16)((marioYaw + 0x2000) & 0xc000);
            if (check_if_moving_over_floor(8.0f, 150.0f)) {
                o->oForwardVel = 4.0f;
                cur_obj_play_sound_1(SOUND_ENV_METAL_BOX_PUSH);
            }
        }
    }

    cur_obj_move_using_fvel_and_gravity();
}
