#include <PR/ultratypes.h>

#include "sm64.h"
#include "area.h"
#include "audio/external.h"
#include "behavior_actions.h"
#include "behavior_data.h"
#include "camera.h"
#include "course_table.h"
#include "dialog_ids.h"
#include "engine/behavior_script.h"
#include "engine/math_util.h"
#include "engine/surface_collision.h"
#include "envfx_bubbles.h"
#include "game_init.h"
#include "ingame_menu.h"
#include "interaction.h"
#include "level_misc_macros.h"
#include "level_table.h"
#include "level_update.h"
#include "levels/bob/header.h"
#include "levels/ttm/header.h"
#include "mario.h"
#include "mario_actions_cutscene.h"
#include "mario_misc.h"
#include "memory.h"
#include "obj_behaviors.h"
#include "object_collision.h"
#include "object_helpers.h"
#include "object_list_processor.h"
#include "rendering_graph_node.h"
#include "save_file.h"
#include "spawn_object.h"
#include "spawn_sound.h"
#include "rumble_init.h"

/**
 * @file obj_behaviors.c
 * This file contains a portion of the obj behaviors and many helper functions for those
 * specific behaviors. Few functions besides the bhv_ functions are used elsewhere in the repo.
 */

/**
 * Keeps track of Mario's previous non-zero room.
 * Helps keep track of room when Mario is over an object.
 */
s16 sPrevCheckMarioRoom = 0;

/**
 * Tracks whether or not Yoshi has walked/jumped off the roof.
 */
s8 sYoshiDead = FALSE;

extern void *ccm_seg7_trajectory_snowman;
extern void *inside_castle_seg7_trajectory_mips;

/**
 * Resets yoshi as spawned/despawned upon new file select.
 * Possibly a function with stubbed code.
 */
void set_yoshi_as_not_dead(void) {
    sYoshiDead = FALSE;
}

/**
 * An unused geo function. Bears strong similarity to geo_bits_bowser_coloring, and relates something
 * of the opacity of an object to something else. Perhaps like, giving a parent object the same
 * opacity?
 */
Gfx UNUSED *geo_obj_transparency_something(s32 callContext, struct GraphNode *node, UNUSED Mat4 *mtx) {
    Gfx *gfxHead = NULL;
    Gfx *gfx;

    if (callContext == GEO_CONTEXT_RENDER) {
        struct Object *heldObject = (struct Object *) gCurGraphNodeObject;
        struct Object *obj = (struct Object *) node;


        if (gCurGraphNodeHeldObject != NULL) {
            heldObject = gCurGraphNodeHeldObject->objNode;
        }

        gfxHead = alloc_display_list(3 * sizeof(Gfx));
        gfx = gfxHead;
        SET_GRAPH_NODE_LAYER(obj->header.gfx.node.flags, LAYER_TRANSPARENT);

        gDPSetEnvColor(gfx++, 255, 255, 255, heldObject->oOpacity);

        gSPEndDisplayList(gfx);
    }

    return gfxHead;
}

/**
 * Uses an object's forward velocity and yaw to move its X, Y, and Z positions.
 * This does accept an object as an argument, though it is always called with `o`.
 */
void obj_move_xyz_using_fvel_and_yaw(struct Object *obj) {
    obj->oVelX = obj->oForwardVel * sins(obj->oMoveAngleYaw);
    obj->oVelZ = obj->oForwardVel * coss(obj->oMoveAngleYaw);

    obj_offset_local(obj, &obj->oPosVec, &obj->oPosVec, obj->oVelX, obj->oVelY, obj->oVelZ);
}

/**
 * Checks if a point is within distance from Mario's graphical position. Test is exclusive.
 */
s32 is_point_within_radius_of_mario(f32 x, f32 y, f32 z, s32 dist) {
    f32 dx = x - gMarioObject->header.gfx.pos[0];
    f32 dy = y - gMarioObject->header.gfx.pos[1];
    f32 dz = z - gMarioObject->header.gfx.pos[2];

    return sqr(dx) + sqr(dy) + sqr(dz) < (f32)sqr(dist);
}

/**
 * Checks whether a point is within distance of a given point. Test is exclusive.
 */
s32 is_point_close_to_object(struct Object *obj, f32 x, f32 y, f32 z, s32 dist) {
    f32 dx = x - obj->oPosX;
    f32 dy = y - obj->oPosY;
    f32 dz = z - obj->oPosZ;

    return sqr(dx) + sqr(dy) + sqr(dz) < (f32)sqr(dist);
}

/**
 * Sets an object as visible if within a certain distance of Mario's graphical position.
 */
void set_object_visibility(struct Object *obj, s32 dist) {
    COND_BIT(
        !is_point_within_radius_of_mario(obj->oPosX, obj->oPosY, obj->oPosZ, dist),
        obj->header.gfx.node.flags,
        GRAPH_RENDER_INVISIBLE
    );
}

/**
 * Turns an object towards home if Mario is not near to it.
 */
static s32 cur_obj_return_home_if_safe(s32 dist) {
    if (is_point_within_radius_of_mario(o->oHomeX, o->oHomeY, o->oHomeZ, dist)) return TRUE;
    cur_obj_rotate_yaw_toward(cur_obj_angle_to_home(), 320);
    return FALSE;
}

/**
 * Randomly displaces an objects home if RNG says to, and turns the object towards its home.
 */
static void cur_obj_wander_near_home(f32 baseDisp) {
    if ((s32)(random_float() * 50.0f) == 0) {
        f32 x = random_f32_around_zero(baseDisp * 2);
        f32 z = random_f32_around_zero(baseDisp * 2);
        obj_offset_local(o, &o->oHomeVec, &o->oHomeVec, x, 0.0f, z);
    }
    cur_obj_rotate_yaw_toward(cur_obj_angle_to_home(), 320);
}

/**
 * A series of checks using sin and cos to see if a given angle is facing in the same direction
 * of a given angle, within a certain range.
 */
s32 obj_check_if_facing_toward_angle(u32 base, u32 goal, s16 range) {
    s16 dAngle = (u16) goal - (u16) base;

    if (((f32) sins(-range) < (f32) sins(dAngle)) && ((f32) sins(dAngle) < (f32) sins(range))
        && (coss(dAngle) > 0)) {
        return TRUE;
    }

    return FALSE;
}

/**
 * Finds any wall collisions and returns what the displacement vector would be.
 */
s32 obj_find_wall_displacement(Vec3f dist, f32 x, f32 y, f32 z, f32 radius) {
    Vec3f pos = { x, y, z };
    if (!resolve_actor_wall(o, pos, 10.0f, radius, 1.0f, NULL)) return FALSE;
    vec3_set(dist, pos[0] - x, pos[1] - y, pos[2] - z);
    return TRUE;
}

/**
 * Spawns a number of coins at the location of an object
 * with a random forward velocity, y velocity, and direction.
 */
void obj_spawn_yellow_coins(struct Object *obj, s8 nCoins) {
    struct Object *coin;
    s8 count;

    for (count = 0; count < nCoins; count++) {
        coin = spawn_object(obj, MODEL_YELLOW_COIN, bhvMovingYellowCoin);
        coin->oForwardVel = random_float() * 20;
        coin->oVelY = random_float() * 40 + 20;
        coin->oMoveAngleYaw = random_u16();
    }
}

/**
 * Controls whether certain objects should flicker/when to despawn.
 */
s32 obj_flicker_and_disappear(struct Object *obj, s16 lifeSpan) {
    if (obj->oTimer < lifeSpan) {
        return FALSE;
    }

    if (obj->oTimer < lifeSpan + 40) {
        COND_BIT((obj->oTimer & 0x1), obj->header.gfx.node.flags, GRAPH_RENDER_INVISIBLE);
    } else {
        obj->activeFlags = ACTIVE_FLAG_DEACTIVATED;
        return TRUE;
    }

    return FALSE;
}

/**
 * Checks if a given room is Mario's current room, even if on an object.
 */
s32 current_mario_room_check(RoomData room) {
    s32 result;

    // Since object surfaces have room 0, this tests if the surface is an
    // object first and uses the last room if so.
    if (gMarioCurrentRoom == 0) {
        return room == sPrevCheckMarioRoom;
    } else {
        result = room == gMarioCurrentRoom;

        sPrevCheckMarioRoom = gMarioCurrentRoom;
    }

    return result;
}

/**
 * Triggers dialog when Mario is facing an object and controls it while in the dialog.
 */
s32 trigger_obj_dialog_when_facing(s32 *inDialog, s16 dialogID, f32 dist, s32 actionArg) {
    if ((is_point_within_radius_of_mario(o->oPosX, o->oPosY, o->oPosZ, (s32) dist)
         && obj_check_if_facing_toward_angle(o->oFaceAngleYaw, obj_yaw_in_frame(o, gMarioObject, gMarioObject->header.gfx.angle[1]) + 0x8000, 0x1000)
         && obj_check_if_facing_toward_angle(o->oMoveAngleYaw, o->oAngleToMario, 0x1000))
        || (*inDialog == TRUE)) {
        *inDialog = TRUE;

        if (set_mario_npc_dialog(actionArg) == MARIO_DIALOG_STATUS_SPEAK) { // If Mario is speaking.
            s16 dialogResponse = cutscene_object_with_dialog(CUTSCENE_DIALOG, o, dialogID);
            if (dialogResponse != DIALOG_RESPONSE_NONE) {
                set_mario_npc_dialog(MARIO_DIALOG_STOP);
                *inDialog = FALSE;
                return dialogResponse;
            }
            return DIALOG_RESPONSE_NONE;
        }
    }

    return DIALOG_RESPONSE_NONE;
}

static s32 cur_obj_is_near_floor(void) {
    // Keep contact across short gaps when moving down a slope.
    return o->oFloor != NULL && obj_height(o, &o->oPosVec) - o->oFloorHeight < 37.0f;
}

static void cur_obj_move_and_align_with_floor(s16 steepSlopeAngle) {
    cur_obj_move_standard(steepSlopeAngle);
    if (cur_obj_is_near_floor() && obj_normal_up(o, o->oFloor->normal) < 1.0f) {
        cur_obj_align_gfx_with_floor();
    }
}

static void cur_obj_accelerate_downhill(void) {
    struct Normal normal = obj_local_normal(o, o->oFloor->normal);
    f32 downhill = -2.0f * o->oGravity * (sqr(normal.x) + sqr(normal.z));
    if (downhill != 0.0f) {
        o->oVelX += normal.x * downhill;
        o->oVelZ += normal.z * downhill;
        o->oForwardVel = sqrtf(sqr(o->oVelX) + sqr(o->oVelZ));
        o->oMoveAngleYaw = atan2s(o->oVelZ, o->oVelX);
    }
}

static void cur_obj_move_rolling(void) {
    if (o->activeFlags & (ACTIVE_FLAG_FAR_AWAY | ACTIVE_FLAG_IN_DIFFERENT_ROOM)) return;

    cur_obj_update_floor_and_bounce_off_walls();
    cur_obj_move_standard(60);
    if (cur_obj_is_near_floor()) cur_obj_accelerate_downhill();
}

static void obj_check_floor_death(void) {
    if (!(o->oMoveFlags & OBJ_MOVE_MASK_ON_ANY_GROUND)) return;

    if (o->oMoveFlags & OBJ_MOVE_ABOVE_LAVA) {
        o->oAction = OBJ_ACT_LAVA_DEATH;
    } else if (o->oMoveFlags & OBJ_MOVE_ABOVE_DEATH_BARRIER) {
        o->oAction = OBJ_ACT_DEATH_PLANE_DEATH;
    }
}

/**
 * Controls an object dying in lava by creating smoke, sinking the object, playing
 * audio, and eventually despawning it. Returns TRUE when the obj is dead.
 */
s32 obj_lava_death(void) {
    struct Object *deathSmoke;

    if (o->oTimer > 30) {
        o->activeFlags = ACTIVE_FLAG_DEACTIVATED;
        return TRUE;
    } else {
        // Sinking effect
        obj_add_up(o, &o->oPosVec, -10.0f);
    }

    if ((o->oTimer % 8) == 0) {
        cur_obj_play_sound_2(SOUND_OBJ_BULLY_EXPLODE_LAVA);
        deathSmoke = spawn_object(o, MODEL_SMOKE, bhvBobombBullyDeathSmoke);
        deathSmoke->oPosX += random_float() * 20.0f;
        deathSmoke->oPosY += random_float() * 20.0f;
        deathSmoke->oPosZ += random_float() * 20.0f;
        deathSmoke->oForwardVel = random_float() * 10.0f;
    }

    return FALSE;
}

/**
 * Spawns an orange number object relatively, such as those that count up for secrets.
 */
void spawn_orange_number(s8 behParam, s16 relX, s16 relY, s16 relZ) {
#ifdef DIALOG_INDICATOR
    if (behParam > ORANGE_NUMBER_F) return;
#else
    if (behParam > ORANGE_NUMBER_9) return;
#endif

    struct Object *orangeNumber = spawn_object_relative(behParam, relX, relY, relZ, o, MODEL_NUMBER, bhvOrangeNumber);
    obj_add_up(o, &orangeNumber->oPosVec, 25.0f);
    orangeNumber->oOrangeNumberOffset = relX;
    vec3_copy(&orangeNumber->oHomeVec, &o->oPosVec);
}

/**
 * Unused variables for debug_sequence_tracker.
 */
s8 sDebugSequenceTracker = 0;
s8 sDebugTimer = 0;

/**
 * Unused presumably debug function that tracks for a sequence of inputs.
 */
UNUSED s32 debug_sequence_tracker(s16 debugInputSequence[]) {
    // If end of sequence reached, return true.
    if (debugInputSequence[sDebugSequenceTracker] == 0) {
        sDebugSequenceTracker = 0;
        return TRUE;
    }

    // If the button pressed is next in sequence, reset timer and progress to next value.
    if (debugInputSequence[sDebugSequenceTracker] & gPlayer1Controller->buttonPressed) {
        sDebugSequenceTracker++;
        sDebugTimer = 0;
    // If wrong input or timer reaches 10, reset sequence progress.
    } else if (sDebugTimer == 10 || gPlayer1Controller->buttonPressed != 0) {
        sDebugSequenceTracker = 0;
        sDebugTimer = 0;
        return FALSE;
    }
    sDebugTimer++;

    return FALSE;
}

#include "behaviors/moving_coin.inc.c"
#include "behaviors/seaweed.inc.c"
#include "behaviors/bobomb.inc.c"
#include "behaviors/cannon_door.inc.c"
#include "behaviors/whirlpool.inc.c"
#include "behaviors/amp.inc.c"
#include "behaviors/butterfly.inc.c"
#include "behaviors/hoot.inc.c"
#include "behaviors/beta_holdable_object.inc.c"
#include "behaviors/explosion.inc.c"
#include "behaviors/respawner.inc.c"
#include "behaviors/bully.inc.c"
#include "behaviors/water_ring.inc.c"
#include "behaviors/bowser_bomb.inc.c"
#include "behaviors/celebration_star.inc.c"
#include "behaviors/drawbridge.inc.c"
#include "behaviors/bomp.inc.c"
#include "behaviors/sliding_platform.inc.c"
#include "behaviors/moneybag.inc.c"
#include "behaviors/bowling_ball.inc.c"
#include "behaviors/cruiser.inc.c"
#include "behaviors/spindel.inc.c"
#include "behaviors/pyramid_wall.inc.c"
#include "behaviors/pyramid_elevator.inc.c"
#include "behaviors/pyramid_top.inc.c"
#include "behaviors/sound_waterfall.inc.c"
#include "behaviors/sound_volcano.inc.c"
#include "behaviors/castle_flag.inc.c"
#include "behaviors/sound_birds.inc.c"
#include "behaviors/sound_ambient.inc.c"
#include "behaviors/sound_sand.inc.c"
#include "behaviors/castle_cannon_grate.inc.c"
#include "behaviors/snowman.inc.c"
#include "behaviors/boulder.inc.c"
#include "behaviors/cap.inc.c"
#include "behaviors/koopa_shell.inc.c"
#include "behaviors/spawn_star.inc.c"
#include "behaviors/red_coin.inc.c"
#include "behaviors/hidden_star.inc.c"
#include "behaviors/rolling_log.inc.c"
#include "behaviors/mushroom_1up.inc.c"
#include "behaviors/controllable_platform.inc.c"
#include "behaviors/breakable_box_small.inc.c"
#include "behaviors/snow_mound.inc.c"
#include "behaviors/floating_platform.inc.c"
#include "behaviors/arrow_lift.inc.c"
#include "behaviors/orange_number.inc.c"
#include "behaviors/manta_ray.inc.c"
#include "behaviors/falling_pillar.inc.c"
#include "behaviors/floating_box.inc.c"
#include "behaviors/decorative_pendulum.inc.c"
#include "behaviors/treasure_chest.inc.c"
#include "behaviors/mips.inc.c"
#include "behaviors/yoshi.inc.c"
