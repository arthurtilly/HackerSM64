#include <PR/ultratypes.h>

#include "engine/math_util.h"
#include "engine/surface_collision.h"
#include "level_update.h"
#include "object_fields.h"
#include "object_helpers.h"
#include "object_list_processor.h"
#include "platform_displacement.h"
#include "types.h"
#include "sm64.h"
#include "behavior_data.h"
#include "game_init.h"

#include "config.h"

static struct PlatformDisplacementInfo sMarioDisplacementInfo;
static Vec3f sMarioAmountDisplaced;

static void remember_platform(struct PlatformDisplacementInfo *info, struct Object *rider,
                              Vec3f pos, s16 yaw, Mat4 transform) {
    Vec3f relative, heading;
    Vec3f inverseRows[3];
    // The parent of an attached platform can scale its axes unequally.
    vec3_cross(inverseRows[0], transform[1], transform[2]);
    f32 determinant = vec3_dot(transform[0], inverseRows[0]);
    if (determinant == 0.0f) {
        info->prevPlatform = NULL;
        return;
    }
    f32 inv = 1.0f / determinant;
    vec3_cross(inverseRows[1], transform[2], transform[0]);
    vec3_cross(inverseRows[2], transform[0], transform[1]);
    obj_yaw_direction(rider, heading, yaw);
    vec3_diff(relative, pos, transform[3]);
    for (s32 i = 0; i < 3; i++) {
        info->prevTransformedPos[i] = vec3_dot(inverseRows[i], relative) * inv;
        info->prevTransformedYawVec[i] = vec3_dot(inverseRows[i], heading) * inv;
    }
    vec3_copy(info->prevPos, pos);
    info->prevYaw = yaw;
    info->prevPlatform = rider->platform;
    info->prevTimer = gGlobalTimer;
}

/**
 * Determine if Mario is standing on a platform object, meaning that he is
 * within 4 units of the floor. Set his referenced platform object accordingly.
 */
void update_mario_platform(void) {
    struct SurfaceContact floor;
    if (gMarioObject == NULL) return;
    f32 height = find_actor_floor(gMarioObject, &gMarioObject->oPosVec, 0.0f, &floor);
    struct Object *platform = NULL;
    if (floor.surface != NULL && (floor.surface->flags & SURFACE_FLAG_DYNAMIC)
        && absf(obj_height(gMarioObject, &gMarioObject->oPosVec) - height) < 4.0f) {
        platform = floor.surface->object;
    }
    gMarioObject->platform = platform;
    if (platform != NULL && (platform != sMarioDisplacementInfo.prevPlatform
                         || gGlobalTimer != sMarioDisplacementInfo.prevTimer)) {
        Mat4 transform;
        obj_build_transform(platform, transform);
        remember_platform(&sMarioDisplacementInfo, gMarioObject, gMarioState->pos, gMarioState->faceAngle[1], transform);
    }
}

/**
 * Apply one frame of platform displacement to Mario or an object using the given
 * platform.
 */
void apply_platform_displacement(struct PlatformDisplacementInfo *displaceInfo, struct Object *rider, Vec3f pos, s16 *yaw) {
    struct Object *platform = rider->platform;
    Vec3f posDifference;
    Vec3f yawVec;
    Vec3f oldPos;
    Mat4 transform;
    // Determine how much Mario turned on his own since last frame
    s16 yawDifference = *yaw - displaceInfo->prevYaw;

    if (platform == NULL || platform->collisionFrame != gGlobalTimer) return;
    obj_build_transform(platform, transform);
    vec3_copy(oldPos, pos);

    // Determine how far Mario moved on his own since last frame
    vec3_diff(posDifference, pos, displaceInfo->prevPos);

    if ((platform == displaceInfo->prevPlatform) && (gGlobalTimer == displaceInfo->prevTimer + 1)) {
        // For certain objects, only use velocity for displacement rather than the transform
        // E.g. TTC treadmills
        if (platform->oFlags & OBJ_FLAG_NO_AUTO_DISPLACEMENT) {
            obj_offset_local(platform, pos, pos, platform->oVelX, platform->oVelY, platform->oVelZ);
        } else {
            // Transform from relative positions to world positions
            linear_mtxf_mul_vec3_and_translate(transform, pos, displaceInfo->prevTransformedPos);

            // Add on how much Mario moved in the previous frame
            vec3_add(pos, posDifference);

            // Calculate new yaw
            linear_mtxf_mul_vec3(transform, yawVec, displaceInfo->prevTransformedYawVec);
            *yaw = obj_yaw_from_direction(rider, yawVec) + yawDifference;
        }
    }

    remember_platform(displaceInfo, rider, pos, *yaw, transform);

    // If the object is Mario, set inertia
    if (pos == gMarioState->pos) {
        vec3_diff(sMarioAmountDisplaced, pos, oldPos);
    }
}

// Doesn't change in the code, set this to FALSE if you don't want inertia
u8 gDoInertia = TRUE;

static u8 sShouldApplyInertia = FALSE;
static u8 sInertiaFirstFrame = FALSE;

/**
 * Apply inertia based on Mario's last platform.
 */
static void apply_mario_inertia(void) {
    f32 up = obj_height(gMarioObject, sMarioAmountDisplaced);
#ifdef MARIO_INERTIA_UPWARD
    // On the first frame of leaving the ground, boost Mario's y velocity
    if (sInertiaFirstFrame) {
        if (up > 0.0f) {
            gMarioState->vel[1] += up;
        }
    }
#endif

#ifdef MARIO_INERTIA_LATERAL
    // Apply sideways inertia
    obj_add_up(gMarioObject, sMarioAmountDisplaced, -up);
    vec3_add(gMarioState->pos, sMarioAmountDisplaced);
    vec3_scale(sMarioAmountDisplaced, 0.97f);
#endif

    // Stop applying inertia once Mario has landed, or when ground pounding
    if (!(gMarioState->action & ACT_FLAG_AIR) || (gMarioState->action == ACT_GROUND_POUND)) {
        sShouldApplyInertia = FALSE;
    }
}

/**
 * Apply platform displacement or inertia if required.
 */
void apply_mario_platform_displacement(void) {
    if (!(gTimeStopState & TIME_STOP_ACTIVE) && gMarioObject != NULL) {
        if (gMarioObject->platform != NULL) {
            apply_platform_displacement(&sMarioDisplacementInfo, gMarioObject, gMarioState->pos, &gMarioState->faceAngle[1]);
            sShouldApplyInertia = TRUE;
            sInertiaFirstFrame = TRUE;
        } else if (sShouldApplyInertia && gDoInertia) {
            apply_mario_inertia();
            sInertiaFirstFrame = FALSE;
        }
    }
}

/**
 * Set Mario's platform to NULL.
 */
void clear_mario_platform(void) {
    if (gMarioObject != NULL) gMarioObject->platform = NULL;
    sMarioDisplacementInfo.prevPlatform = NULL;
}
