
/**
 * Behavior for the falling pillars inside the underwater cave area of
 * Jolly Roger Bay.
 *
 * Also includes behavior for the invisible hitboxes they spawn.
 */

static struct ObjectHitbox sFallingPillarHitbox = {
    /* interactType:      */ INTERACT_DAMAGE,
    /* damageOrCoinValue: */ 3,
    /* health:            */ 0,
    /* numLootCoins:      */ 0,
    /* radius:            */ 145,
    /* offset:            */ 0,
    /* hurtboxRadius:     */ 0,
    /* hurtboxOffset:     */ 0,
};

/**
 * Spawns 4 hitboxes with Y coordinates offset.
 */
void bhv_falling_pillar_spawn_hitboxes(void) {
    s32 i;

    for (i = 0; i < 4; i++) {
        spawn_object_relative(i, 0, i * 400 + 300, 0, o, MODEL_NONE, bhvFallingPillarHitbox);
    }
}

/**
 * Computes the angle from current pillar position to 500 units in front of
 * Mario.
 */
s16 bhv_falling_pillar_calculate_angle_in_front_of_mario(void) {
    Vec3f target;
    obj_offset_forward(gMarioObject, target, gMarioObject->header.gfx.pos, gMarioState->faceAngle[1], 500.0f, 0.0f);
    vec3_sub(target, &o->oPosVec);
    return obj_yaw_from_direction(o, target);
}

/**
 * Falling pillar main logic loop.
 */
void bhv_falling_pillar_loop(void) {
    s16 angleInFrontOfMario;

    if (o->oAction == FALLING_PILLAR_ACT_TURNING || o->oAction == FALLING_PILLAR_ACT_FALLING) {
        cur_obj_update_floor_and_walls();
        cur_obj_move_standard(60);
    }

    switch (o->oAction) {
        case FALLING_PILLAR_ACT_IDLE:
            // When Mario is within 1300 units of distance...
            if (is_point_within_radius_of_mario(o->oPosX, o->oPosY, o->oPosZ, 1300)) {
                // Begin slightly moving towards Mario.
                o->oMoveAngleYaw = o->oAngleToMario;
                o->oForwardVel = 1.0f;

                // Spawn the invisible hitboxes.
                bhv_falling_pillar_spawn_hitboxes();

                // Start turning towards Mario.
                o->oAction = FALLING_PILLAR_ACT_TURNING;

                // Play the detaching sound.
                cur_obj_play_sound_2(SOUND_GENERAL_POUND_ROCK);
            }
            break;

        case FALLING_PILLAR_ACT_TURNING:
            // Calculate angle in front of Mario and turn towards it.
            angleInFrontOfMario = bhv_falling_pillar_calculate_angle_in_front_of_mario();
            o->oFaceAngleYaw = approach_s16_symmetric(o->oFaceAngleYaw, angleInFrontOfMario, 0x400);

            // After 10 ticks, start falling.
            if (o->oTimer > 10) {
                o->oAction = FALLING_PILLAR_ACT_FALLING;
            }
            break;

        case FALLING_PILLAR_ACT_FALLING:
            // Start falling slowly, with increasing acceleration each frame.
            o->oFallingPillarPitchAcceleration += 4.0f;
            o->oAngleVelPitch += o->oFallingPillarPitchAcceleration;
            o->oFaceAnglePitch += o->oAngleVelPitch;

            // Once the pillar has turned nearly 90 degrees (after ~22 frames),
            if (o->oFaceAnglePitch > 0x3900) {
                // Move 500 units in the direction of falling.
                obj_offset_forward(o, &o->oPosVec, &o->oPosVec, o->oFaceAngleYaw, 500.0f, 0.0f);

                // Make the camera shake and spawn dust clouds.
                set_camera_shake_from_point(SHAKE_POS_MEDIUM, o->oPosX, o->oPosY, o->oPosZ);
                spawn_mist_particles_variable(0, 0, 92.0f);

                // Go invisible.
                o->activeFlags = ACTIVE_FLAG_DEACTIVATED;

                // Play the hitting the ground sound.
                create_sound_spawner(SOUND_GENERAL_BIG_POUND);
            }
            break;
    }
}

/**
 * Main loop for the invisible hitboxes.
 */
void bhv_falling_pillar_hitbox_loop(void) {
    // Get the state of the pillar.
    s32 pitch = o->parentObj->oFaceAnglePitch;
    s32 yaw = o->parentObj->oFaceAngleYaw;
    f32 yOffset = o->oBehParams2ndByte * 400 + 300;

    // Update position of hitboxes so they fall with the pillar.
    obj_offset_forward(o->parentObj, &o->oPosVec, &o->parentObj->oPosVec, yaw,
                       sins(pitch) * yOffset, coss(pitch) * yOffset);

    // Give these a hitbox so they can collide with Mario.
    obj_set_hitbox(o, &sFallingPillarHitbox);

    // When the pillar goes inactive, the hitboxes also go inactive.
    if (o->parentObj->activeFlags == ACTIVE_FLAG_DEACTIVATED) {
        o->activeFlags = ACTIVE_FLAG_DEACTIVATED;
    }
}
