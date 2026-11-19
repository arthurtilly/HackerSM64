
/**
 * Behavior for bhvTTCPitBlock. This is the block that moves up and down near the
 * Pit and the Pendulum star.
 */

/**
 * Collision models. The second one is unused.
 */
static Collision const *sTTCPitBlockCollisionModels[] = {
    ttc_seg7_collision_07015754,
    ttc_seg7_collision_070157D8,
};

/**
 * The speed of movement, and the time to wait before moving.
 * If the wait time is negative (before moving down on random setting), wait
 * a random amount of time instead.
 */
struct TTCPitBlockProperties {
    s16 speed;
    s16 waitTime;
};

/**
 * Properties for the pit block on each speed setting when moving up and down,
 * respectively.
 */
static struct TTCPitBlockProperties sTTCPitBlockProperties[][2] = {
    /* TTC_SPEED_SLOW    */ { { 11, 20 }, { -9, 30 } },
    /* TTC_SPEED_FAST    */ { { 18, 15 }, { -11, 15 } },
    /* TTC_SPEED_RANDOM  */ { { 11, 20 }, { -9, -1 } },
    /* TTC_SPEED_STOPPED */ { { 0, 0 }, { 0, 0 } },
};

/**
 * Init function for bhvTTCPitBlock.
 */
void bhv_ttc_pit_block_init(void) {
    o->collisionData = segmented_to_virtual(sTTCPitBlockCollisionModels[o->oBehParams2ndByte]);

    if (gTTCSpeedSetting == TTC_SPEED_STOPPED) {
        obj_add_up(o, &o->oPosVec, 330.0f);
    }
}

/**
 * Update function for bhvTTCPitBlock.
 * Move up and down, pausing before changing direction.
 */
void bhv_ttc_pit_block_update(void) {
    if (o->oTimer > o->oTTCPitBlockWaitTime) {
        f32 offset = obj_height_difference(o, &o->oPosVec, &o->oHomeVec) + o->oVelY;
        s32 reachedEnd = clamp_f32(&offset, 0.0f, 330.0f);
        obj_offset_up(o, &o->oPosVec, &o->oHomeVec, offset);
        if (reachedEnd) {
            o->oTTCPitBlockDir ^= 0x1;

            if ((o->oTTCPitBlockWaitTime = sTTCPitBlockProperties[gTTCSpeedSetting][o->oTTCPitBlockDir & 0x1].waitTime) < 0) {
                o->oTTCPitBlockWaitTime = random_mod_offset(10, 20, 6);
            }

            o->oVelY = sTTCPitBlockProperties[gTTCSpeedSetting][o->oTTCPitBlockDir].speed;
            o->oTimer = 0;
        }
    }
}
