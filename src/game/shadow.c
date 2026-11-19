#include <PR/ultratypes.h>
#include <PR/gbi.h>

#include "engine/math_util.h"
#include "engine/surface_collision.h"
#include "behavior_data.h"
#include "level_table.h"
#include "memory.h"
#include "level_update.h"
#include "object_list_processor.h"
#include "segment2.h"
#include "shadow.h"
#include "sm64.h"

/**
 * @file shadow.c
 * This file implements a self-contained subsystem used to draw shadows.
 */

/**
 * An array consisting of all the hardcoded rectangle shadows in the game.
 */
static const Vec2f sShadowRectangles[] = {
    { 7.2f, 4.6f }, // Spindel
    { 4.0f, 3.6f }, // Whomp
};

struct Shadow gCurrShadow;
static struct Shadow *const s = &gCurrShadow;

/**
 * Shrink a shadow when its parent object is further from the floor, given the
 * initial size of the shadow and the current distance.
 */
f32 scale_shadow_with_distance(f32 initial, f32 distFromFloor) {
    if (distFromFloor <= 0.0f) {
        return initial;
    } else if (distFromFloor >= 600.0f) {
        return initial * 0.5f;
    } else {
        return initial * (1.0f - ((distFromFloor * 0.5f) / 600.0f));
    }
}

/**
 * Dim a shadow when its parent object is further from the ground.
 */
s32 dim_shadow_with_distance(u8 solidity, f32 distFromFloor) {
    if (solidity < 121) {
        return solidity;
    } else if (distFromFloor <= 0.0f) {
        return solidity;
    } else if (distFromFloor >= 600.0f) {
        return 120;
    } else {
        return (((120 - solidity) * distFromFloor) / 600.0f) + (f32) solidity;
    }
}

/**
 * Initialize a shadow. Return 0 on success, 1 on failure.
 *
 * @param pos Position of the parent object (not the shadow)
 * @param shadowScale Diameter of the shadow
 * @param overwriteSolidity Flag for whether the existing shadow solidity should
 *                          be dimmed based on its distance to the floor
 */
s32 init_shadow(f32 distToShadow, s16 shadowScale, s8 shadowType, u8 overwriteSolidity) {
    f32 baseScale;

    if (shadowType != SHADOW_SQUARE_PERMANENT) {
        // Set solidity and scale based on distance.

        if (overwriteSolidity) {
            s->solidity = dim_shadow_with_distance(overwriteSolidity, distToShadow);
        }

        baseScale = scale_shadow_with_distance(shadowScale, distToShadow);
    } else {
        s->solidity = overwriteSolidity;
        baseScale = shadowScale;
    }
    vec3f_set(s->scale, baseScale, baseScale, baseScale);

    return !(s->solidity);
}

/**
 * Linearly interpolate a shadow's solidity between zero and finalSolidity
 * depending on curr's relation to start and end.
 */
void linearly_interpolate_solidity_positive(u8 finalSolidity, s16 curr, s16 start,
                                            s16 end) {
    if (curr >= 0 && curr < start) {
        s->solidity = 0;
    } else if (end < curr) {
        s->solidity = finalSolidity;
    } else {
        s->solidity = (f32) finalSolidity * (curr - start) / (end - start);
    }
}

/**
 * Linearly interpolate a shadow's solidity between initialSolidity and zero
 * depending on curr's relation to start and end. Note that if curr < start,
 * the solidity will be zero.
 */
void linearly_interpolate_solidity_negative(u8 initialSolidity, s16 curr, s16 start,
                                            s16 end) {
    // The curr < start case is not handled. Thus, if start != 0, this function
    // will have the surprising behavior of hiding the shadow until start.
    // This is not necessarily a bug, since this function is only used once,
    // with start == 0.
    if (curr >= start && end >= curr) {
        s->solidity = ((f32) initialSolidity * (1.0f - (f32)(curr - start) / (end - start)));
    } else {
        s->solidity = 0;
    }
}

/**
 * Change a shadow's solidity based on the player's current animation frame.
 */
s32 correct_shadow_solidity_for_animations(u8 initialSolidity) {
    s16 animFrame = gMarioObject->header.gfx.animInfo.animFrame;
    switch (gMarioObject->header.gfx.animInfo.animID) {
        case MARIO_ANIM_IDLE_ON_LEDGE:
            return SHADOW_SOLIDITY_NO_SHADOW;
        case MARIO_ANIM_FAST_LEDGE_GRAB:
            linearly_interpolate_solidity_positive(initialSolidity, animFrame,  5, 14);
            return SHADOW_SOILDITY_ALREADY_SET;
        case MARIO_ANIM_SLOW_LEDGE_GRAB:
            linearly_interpolate_solidity_positive(initialSolidity, animFrame, 21, 33);
            return SHADOW_SOILDITY_ALREADY_SET;
        case MARIO_ANIM_CLIMB_DOWN_LEDGE:
            linearly_interpolate_solidity_negative(initialSolidity, animFrame,  0,  5);
            return SHADOW_SOILDITY_ALREADY_SET;
        default:
            return SHADOW_SOLIDITY_NOT_YET_SET;
    }
}

#ifdef ENABLE_VANILLA_LEVEL_SPECIFIC_CHECKS
/**
 * Slightly change the height of a shadow in levels with lava.
 */
void correct_lava_shadow_height(f32 *floorHeight) {
    if (gCurrLevelNum == LEVEL_BITFS) {
        if (*floorHeight < -3000.0f) {
            *floorHeight = -3062.0f;
            s->isDecal = FALSE;
        } else if (*floorHeight > 3400.0f) {
            *floorHeight = 3492.0f;
            s->isDecal = FALSE;
        }
    } else if (gCurrLevelNum == LEVEL_LLL
               && gCurrAreaIndex == 1) {
        *floorHeight = 5.0f;
        s->isDecal = FALSE;
    }
}
#endif

/**
 * Add a shadow to the given display list.
 * shadowType 0 uses a circle texture, the rest use a square texture.
 * Uses environment alpha for shadow solidity.
 */
static void add_shadow_to_display_list(Gfx *displayListHead, s8 shadowType) {
    if (shadowType == SHADOW_CIRCLE) {
        gSPDisplayList(displayListHead++, dl_shadow_circle);
    } else {
        gSPDisplayList(displayListHead++, dl_shadow_square);
    }
    gDPSetEnvColor(displayListHead++, 255, 255, 255, s->solidity);
    gSPDisplayList(displayListHead++, dl_shadow_end);
    gSPEndDisplayList(displayListHead);
}

// A graphics or animation offset can move the caster past the edge of its physics floor.
static s32 shadow_over_actor_floor(const struct Object *obj, const Vec3f pos, const Vec3f feet) {
    if (obj->gravityDir[1] == -1.0f) return pos[0] == feet[0] && pos[2] == feet[2];
    Vec3f delta, lateral;
    vec3_diff(delta, pos, feet);
    vec3_cross(lateral, delta, obj->gravityDir);
    return vec3_sumsq(lateral) < 0.0001f;
}

static struct Surface *find_shadow_floor(const struct Object *obj, const Vec3f pos, f32 range, f32 *distance) {
    s32 mario = obj == gMarioObject;
    struct Surface *floor = mario ? gMarioState->floor : obj->oFloor;
    const f32 *feet = mario ? gMarioState->pos : &obj->oPosVec;
    if (floor != NULL && obj->oHeldState == HELD_FREE && obj_normal_up(obj, floor->normal) > 0.0f
        && shadow_over_actor_floor(obj, pos, feet)) {
        *distance = obj_height(obj, pos) - (mario ? gMarioState->floorHeight : obj->oFloorHeight);
        return floor;
    }

    // Start slightly above the caster so graphics that dip into a slope still cast a shadow.
    Vec3f origin;
    struct SurfaceContact hit;
    obj_offset_up(obj, origin, pos, FIND_FLOOR_BUFFER);
    *distance = collision_raycast(origin, obj->gravityDir, range + FIND_FLOOR_BUFFER, 0.0f,
                                  COLLISION_FLAGS_NONE, &hit) - FIND_FLOOR_BUFFER;
    return hit.surface;
}

static s32 project_shadow(const struct Object *obj, Vec3f pos, f32 *distance) {
    f32 range = obj == gMarioObject ? LEVEL_RAY_LENGTH : 1024.0f;
    struct Surface *floor = find_shadow_floor(obj, pos, range, distance);
    f32 waterRange = floor == NULL ? range : MIN(*distance, range);
    f32 water = raycast_water_surface(pos, obj->gravityDir, waterRange, s->floorNormal);

    s->isDecal = TRUE;
    if (water >= 0.0f) {
        *distance = water;
        s->isDecal = FALSE;
    } else {
        if (floor == NULL) return FALSE;
        vec3_set(s->floorNormal, floor->normal.x, floor->normal.y, floor->normal.z);
        if (floor->type == SURFACE_ICE) {
            s->isDecal = FALSE;
#ifdef ENABLE_VANILLA_LEVEL_SPECIFIC_CHECKS
        } else if (floor->type == SURFACE_BURNING && floor->normal.y == 1.0f && obj->gravityDir[1] < 0.0f) {
            f32 lavaY = pos[1] + obj->gravityDir[1] * *distance;
            correct_lava_shadow_height(&lavaY);
            if (!s->isDecal) *distance = (lavaY - pos[1]) / obj->gravityDir[1];
#endif
        } else if (floor->object != NULL && floor->object->behavior == segmented_to_virtual(bhvPlatformOnTrack)
                   && floor->object->oPlatformOnTrackType == PLATFORM_ON_TRACK_TYPE_CARPET) {
            *distance -= 5.0f;
            s->isDecal = FALSE;
        }
    }
    if (*distance < -80.0f || *distance > range) return FALSE;
    obj_add_up(obj, pos, -*distance);
    return TRUE;
}

Gfx *create_shadow(const struct Object *obj, Vec3f pos, s16 shadowScale, u8 shadowSolidity, s8 shadowType) {
    if (obj == gMarioObject) {
        switch (correct_shadow_solidity_for_animations(shadowSolidity)) {
            case SHADOW_SOLIDITY_NO_SHADOW: return NULL;
            case SHADOW_SOILDITY_ALREADY_SET: shadowSolidity = 0; break;
        }
    }
    f32 distance;
    if (!project_shadow(obj, pos, &distance) || init_shadow(distance, shadowScale, shadowType, shadowSolidity)) {
        return NULL;
    }
    if (shadowType >= SHADOW_RECTANGLE_HARDCODED_OFFSET) {
        const f32 *size = sShadowRectangles[shadowType - SHADOW_RECTANGLE_HARDCODED_OFFSET];
        s->scale[0] *= size[0];
        s->scale[2] *= size[1];
    }

    Gfx *displayList = alloc_display_list(4 * sizeof(Gfx));
    if (displayList != NULL) add_shadow_to_display_list(displayList, shadowType);
    return displayList;
}
