#include <PR/ultratypes.h>

#include "sm64.h"
#include "area.h"
#include "behavior_actions.h"
#include "behavior_data.h"
#include "camera.h"
#include "debug.h"
#include "dialog_ids.h"
#include "engine/behavior_script.h"
#include "engine/geo_layout.h"
#include "engine/math_util.h"
#include "engine/surface_collision.h"
#include "game_init.h"
#include "helper_macros.h"
#include "ingame_menu.h"
#include "interaction.h"
#include "level_table.h"
#include "level_update.h"
#include "mario.h"
#include "mario_actions_cutscene.h"
#include "memory.h"
#include "obj_behaviors.h"
#include "object_helpers.h"
#include "object_collision.h"
#include "object_list_processor.h"
#include "rendering_graph_node.h"
#include "spawn_object.h"
#include "spawn_sound.h"

Gfx *geo_update_projectile_pos_from_parent(s32 callContext, UNUSED struct GraphNode *node, Mat4 mtx) {
    if (callContext == GEO_CONTEXT_RENDER) {
        struct Object *projObj = (struct Object *) gCurGraphNodeObject; // HACKERSM64_DO: change global type to Object pointer
        if (projObj->prevObj) {
            obj_update_pos_from_parent_transformation(mtx, projObj->prevObj);
            obj_set_gfx_pos_from_pos(projObj->prevObj);
        }
    }
    return NULL;
}

Gfx *geo_update_layer_transparency(s32 callContext, struct GraphNode *node, UNUSED void *context) {
    Gfx *dlStart = NULL;

    if (callContext == GEO_CONTEXT_RENDER) {
        struct Object *objectGraphNode = (struct Object *) gCurGraphNodeObject; // HACKERSM64_DO: change this to object pointer?
        struct GraphNodeGenerated *currentGraphNode = (struct GraphNodeGenerated *) node;
        s32 parameter = currentGraphNode->parameter;

        if (gCurGraphNodeHeldObject != NULL) {
            objectGraphNode = gCurGraphNodeHeldObject->objNode;
        }

        s32 objectOpacity = objectGraphNode->oOpacity;
        dlStart = alloc_display_list(sizeof(Gfx) * 3);

        Gfx *dlHead = dlStart;

        if (objectOpacity == 0xFF) {
            if (parameter == GEO_TRANSPARENCY_MODE_DECAL) {
                SET_GRAPH_NODE_LAYER(currentGraphNode->fnNode.node.flags, LAYER_TRANSPARENT_DECAL);
            } else {
                SET_GRAPH_NODE_LAYER(currentGraphNode->fnNode.node.flags, LAYER_OPAQUE);
            }

            objectGraphNode->oAnimState = TRANSPARENCY_ANIM_STATE_OPAQUE;
        } else {
            if (parameter == GEO_TRANSPARENCY_MODE_DECAL) {
                SET_GRAPH_NODE_LAYER(currentGraphNode->fnNode.node.flags, LAYER_TRANSPARENT_DECAL);
            } else if (parameter == GEO_TRANSPARENCY_MODE_INTER) {
                SET_GRAPH_NODE_LAYER(currentGraphNode->fnNode.node.flags, LAYER_TRANSPARENT_INTER);
            } else {
                SET_GRAPH_NODE_LAYER(currentGraphNode->fnNode.node.flags, LAYER_TRANSPARENT);
            }

            objectGraphNode->oAnimState = TRANSPARENCY_ANIM_STATE_TRANSPARENT;

            if (objectOpacity == 0x00 && segmented_to_virtual(bhvBowser) == objectGraphNode->behavior) {
                objectGraphNode->oAnimState = BOWSER_ANIM_STATE_INVISIBLE;
            }

            if (parameter != GEO_TRANSPARENCY_MODE_NO_DITHER
                && (objectGraphNode->activeFlags & ACTIVE_FLAG_DITHERED_ALPHA)) {
                gDPSetAlphaCompare(dlHead++, G_AC_DITHER);
            }
        }
        gDPSetEnvColor(dlHead++, 255, 255, 255, objectOpacity);
        gSPEndDisplayList(dlHead);
    }

    return dlStart;
}

Gfx *geo_switch_anim_state(s32 callContext, struct GraphNode *node, UNUSED void *context) {
    if (callContext == GEO_CONTEXT_RENDER) {
        struct Object *obj = gCurGraphNodeObjectNode;

        // move to a local var because GraphNodes are passed in all geo functions.
        // cast the pointer.
        struct GraphNodeSwitchCase *switchCase = (struct GraphNodeSwitchCase *) node;

        if (gCurGraphNodeHeldObject != NULL) {
            obj = gCurGraphNodeHeldObject->objNode;
        }

        // if the case is greater than the number of cases, set to 0 to avoid overflowing
        // the switch.
        if (obj->oAnimState >= switchCase->numCases) {
            obj->oAnimState = 0;
        }

        // assign the case number for execution.
        switchCase->selectedCase = obj->oAnimState;
    }

    return NULL;
}

Gfx *geo_switch_area(s32 callContext, struct GraphNode *node, UNUSED void *context) {
    struct GraphNodeSwitchCase *switchCase = (struct GraphNodeSwitchCase *) node;
    RoomData room;

    if (callContext == GEO_CONTEXT_RENDER && gMarioObject != NULL) {
        room = get_room_at_pos(
            gMarioObject->oPosX,
            gMarioObject->oPosY,
            gMarioObject->oPosZ
        );

        print_debug_top_down_objectinfo("areainfo %d", room);

        if (room > 0) {
            gMarioCurrentRoom = room;
            switchCase->selectedCase = (room - 1);
        }
    } else {
        switchCase->selectedCase = 0;
    }

    return NULL;
}

void obj_update_pos_from_parent_transformation(Mat4 a0, struct Object *a1) {
    f32 spC = a1->oParentRelativePosX;
    f32 sp8 = a1->oParentRelativePosY;
    f32 sp4 = a1->oParentRelativePosZ;

    a1->oPosX = spC * a0[0][0] + sp8 * a0[1][0] + sp4 * a0[2][0] + a0[3][0];
    a1->oPosY = spC * a0[0][1] + sp8 * a0[1][1] + sp4 * a0[2][1] + a0[3][1];
    a1->oPosZ = spC * a0[0][2] + sp8 * a0[1][2] + sp4 * a0[2][2] + a0[3][2];
}


void create_transformation_from_matrices(Mat4 a0, Mat4 a1, Mat4 a2) {
    f32 spC, sp8, sp4;

    spC = a2[3][0] * a2[0][0] + a2[3][1] * a2[0][1] + a2[3][2] * a2[0][2];
    sp8 = a2[3][0] * a2[1][0] + a2[3][1] * a2[1][1] + a2[3][2] * a2[1][2];
    sp4 = a2[3][0] * a2[2][0] + a2[3][1] * a2[2][1] + a2[3][2] * a2[2][2];

    a0[0][0] = a1[0][0] * a2[0][0] + a1[0][1] * a2[0][1] + a1[0][2] * a2[0][2];
    a0[0][1] = a1[0][0] * a2[1][0] + a1[0][1] * a2[1][1] + a1[0][2] * a2[1][2];
    a0[0][2] = a1[0][0] * a2[2][0] + a1[0][1] * a2[2][1] + a1[0][2] * a2[2][2];

    a0[1][0] = a1[1][0] * a2[0][0] + a1[1][1] * a2[0][1] + a1[1][2] * a2[0][2];
    a0[1][1] = a1[1][0] * a2[1][0] + a1[1][1] * a2[1][1] + a1[1][2] * a2[1][2];
    a0[1][2] = a1[1][0] * a2[2][0] + a1[1][1] * a2[2][1] + a1[1][2] * a2[2][2];

    a0[2][0] = a1[2][0] * a2[0][0] + a1[2][1] * a2[0][1] + a1[2][2] * a2[0][2];
    a0[2][1] = a1[2][0] * a2[1][0] + a1[2][1] * a2[1][1] + a1[2][2] * a2[1][2];
    a0[2][2] = a1[2][0] * a2[2][0] + a1[2][1] * a2[2][1] + a1[2][2] * a2[2][2];

    a0[3][0] = a1[3][0] * a2[0][0] + a1[3][1] * a2[0][1] + a1[3][2] * a2[0][2] - spC;
    a0[3][1] = a1[3][0] * a2[1][0] + a1[3][1] * a2[1][1] + a1[3][2] * a2[1][2] - sp8;
    a0[3][2] = a1[3][0] * a2[2][0] + a1[3][1] * a2[2][1] + a1[3][2] * a2[2][2] - sp4;

    a0[0][3] = 0;
    a0[1][3] = 0;
    a0[2][3] = 0;
    a0[3][3] = 1.0f;
}

void obj_set_held_state(struct Object *obj, const BehaviorScript *heldBehavior) {
    obj->parentObj = o;

    if (obj->oFlags & OBJ_FLAG_HOLDABLE) {
        if (heldBehavior == bhvCarrySomethingHeld   ) {
            obj->oHeldState = HELD_HELD;
        }

        if (heldBehavior == bhvCarrySomethingThrown ) {
            obj->oHeldState = HELD_THROWN;
        }

        if (heldBehavior == bhvCarrySomethingDropped) {
            obj->oHeldState = HELD_DROPPED;
        }
    } else {
        obj->curBhvCommand = segmented_to_virtual(heldBehavior);
        obj->bhvStackIndex = 0;
    }
}

static f32 obj_lateral_distance(const struct Object *obj, const Vec3f a, const Vec3f b) {
    Vec3f delta;
    obj_local_delta(obj, delta, a, b);
    return sqrtf(sqr(delta[0]) + sqr(delta[2]));
}

f32 lateral_dist_between_objects(struct Object *obj1, struct Object *obj2) {
    return obj_lateral_distance(obj1, &obj1->oPosVec, &obj2->oPosVec);
}

f32 dist_between_objects(struct Object *obj1, struct Object *obj2) {
    Vec3f d;
    vec3_diff(d, &obj2->oPosVec, &obj1->oPosVec);
    return vec3_mag(d);
}

// Skip sqrtf
f32 dist_between_objects_squared(struct Object *obj1, struct Object *obj2) {
    Vec3f d;
    vec3_diff(d, &obj2->oPosVec, &obj1->oPosVec);
    return vec3_sumsq(d);
}

void cur_obj_forward_vel_approach_upward(f32 target, f32 increment) {
    if (o->oForwardVel >= target) {
        o->oForwardVel = target;
    } else {
        o->oForwardVel += increment;
    }
}

s32 cur_obj_rotate_yaw_toward(s16 target, s16 increment) {
    s16 startYaw;

    startYaw = (s16) o->oMoveAngleYaw;
    o->oMoveAngleYaw = approach_s16_symmetric(o->oMoveAngleYaw, target, increment);

    if ((o->oAngleVelYaw = (s16)((s16) o->oMoveAngleYaw - startYaw)) == 0) {
        return TRUE;
    } else {
        return FALSE;
    }
}

s32 obj_angle_to_object(struct Object *obj1, struct Object *obj2) {
    Vec3f direction;
    vec3_diff(direction, &obj2->oPosVec, &obj1->oPosVec);
    return obj_yaw_from_direction(obj1, direction);
}

s32 obj_turn_toward_object(struct Object *obj, struct Object *target, s16 angleIndex, s16 turnAmount) {
    Vec3f d;
    s16 targetAngle = 0x0;
    s16 startAngle;

    switch (angleIndex) {
        case O_MOVE_ANGLE_PITCH_INDEX:
        case O_FACE_ANGLE_PITCH_INDEX:
            obj_local_delta(obj, d, &target->oPosVec, &obj->oPosVec);
            targetAngle = atan2s(sqrtf(sqr(d[0]) + sqr(d[2])), -d[1]);
            break;

        case O_MOVE_ANGLE_YAW_INDEX:
        case O_FACE_ANGLE_YAW_INDEX:
            targetAngle = obj_angle_to_object(obj, target);
            break;
    }

    startAngle = obj->rawData.asU32[angleIndex];
    obj->rawData.asU32[angleIndex] = approach_s16_symmetric(startAngle, targetAngle, turnAmount);
    return targetAngle;
}

void obj_set_parent_relative_pos(struct Object *obj, s16 relX, s16 relY, s16 relZ) {
    obj->oParentRelativePosX = relX;
    obj->oParentRelativePosY = relY;
    obj->oParentRelativePosZ = relZ;
}

void obj_set_pos(struct Object *obj, s16 x, s16 y, s16 z) {
    obj->oPosX = x;
    obj->oPosY = y;
    obj->oPosZ = z;
}

void obj_set_angle(struct Object *obj, s16 pitch, s16 yaw, s16 roll) {
    obj->oFaceAnglePitch = pitch;
    obj->oFaceAngleYaw = yaw;
    obj->oFaceAngleRoll = roll;

    obj->oMoveAnglePitch = pitch;
    obj->oMoveAngleYaw = yaw;
    obj->oMoveAngleRoll = roll;
}

/*
 * Spawns an object at an absolute location with a specified angle.
 */
struct Object *spawn_object_abs_with_rot(struct Object *parent, s16 uselessArg, ModelID32 model,
                                         const BehaviorScript *behavior,
                                         s16 x, s16 y, s16 z, s16 pitch, s16 yaw, s16 roll) {
    // 'uselessArg' is unused in the function spawn_object_at_origin()
    struct Object *newObj = spawn_object_at_origin(parent, uselessArg, model, behavior);
    obj_set_pos(newObj, x, y, z);
    obj_set_angle(newObj, pitch, yaw, roll);

    return newObj;
}

/*
 * Spawns an object relative to the parent with a specified angle.
 */
struct Object *spawn_object_rel_with_rot(struct Object *parent, ModelID32 model, const BehaviorScript *behavior,
                                         s16 xOff, s16 yOff, s16 zOff, s16 pitch, s16 yaw, s16 roll) {
    struct Object *newObj = spawn_object_at_origin(parent, 0, model, behavior);
    newObj->header.gfx.transformFlags |= GRAPH_OBJ_PARENT;
    obj_set_parent_relative_pos(newObj, xOff, yOff, zOff);
    obj_set_angle(newObj, pitch, yaw, roll);

    return newObj;
}

struct Object *spawn_water_droplet(struct Object *parent, struct WaterDropletParams *params) {
    struct Object *newObj = spawn_object(parent, params->model, params->behavior);

    if (params->flags & WATER_DROPLET_FLAG_RAND_ANGLE) {
        newObj->oMoveAngleYaw = random_u16();
    }

    if (params->flags & WATER_DROPLET_FLAG_RAND_ANGLE_INCR_BACKWARD) {
        newObj->oMoveAngleYaw = (s16)(newObj->oMoveAngleYaw + 0x8000)
                                + (s16) random_f32_around_zero(params->moveAngleRange);
    }

    if (params->flags & WATER_DROPLET_FLAG_RAND_ANGLE_INCR_FORWARD) {
        newObj->oMoveAngleYaw =
            (s16) newObj->oMoveAngleYaw + (s16) random_f32_around_zero(params->moveAngleRange);
    }

    if (params->flags & WATER_DROPLET_FLAG_SET_Y_TO_WATER_LEVEL) {
        newObj->oPosY = find_water_level(newObj->oPosX, newObj->oPosZ);
    }

    if (params->flags & WATER_DROPLET_FLAG_RAND_OFFSET_XZ) {
        obj_translate_xz_random(newObj, params->moveRange);
    }

    if (params->flags & WATER_DROPLET_FLAG_RAND_OFFSET_XYZ) {
        obj_translate_xyz_random(newObj, params->moveRange);
    }

    newObj->oForwardVel = random_float() * params->randForwardVelScale + params->randForwardVelOffset;
    newObj->oVelY = random_float() * params->randYVelScale + params->randYVelOffset;

    f32 randomScale = random_float() * params->randSizeScale + params->randSizeOffset;
    obj_scale(newObj, randomScale);

    return newObj;
}

void obj_inherit_gravity_frame(struct Object *obj, const struct Object *parent) {
    if ((obj->header.gfx.transformFlags | parent->header.gfx.transformFlags) & GRAPH_OBJ_GRAVITY_FRAME) {
        obj->gravityFrame = parent->gravityFrame;
        vec3_copy(obj->gravityDir, parent->gravityDir);
        COND_BIT(parent->header.gfx.transformFlags & GRAPH_OBJ_GRAVITY_FRAME,
                 obj->header.gfx.transformFlags, GRAPH_OBJ_GRAVITY_FRAME);
    }
}

struct Object *spawn_object_at_origin(struct Object *parent, UNUSED s32 unusedArg, ModelID32 model,
                                      const BehaviorScript *behavior) {
    const BehaviorScript *behaviorAddr = segmented_to_virtual(behavior);
    struct Object *obj = create_object(behaviorAddr);

    obj->parentObj = parent;
    obj->header.gfx.areaIndex = parent->header.gfx.areaIndex;
    obj->header.gfx.activeAreaIndex = parent->header.gfx.areaIndex;

    geo_obj_init((struct GraphNodeObject *) &obj->header.gfx, gLoadedGraphNodes[model], gVec3fZero, gVec3sZero);

    obj_inherit_gravity_frame(obj, parent);
    return obj;
}

struct Object *spawn_object(struct Object *parent, ModelID32 model, const BehaviorScript *behavior) {
    struct Object *obj = spawn_object_at_origin(parent, 0, model, behavior);

    obj_copy_pos(obj, parent);
    vec3i_copy(&obj->oMoveAngleVec, &parent->oMoveAngleVec);
    vec3i_copy(&obj->oFaceAngleVec, &parent->oFaceAngleVec);
    return obj;
}

struct Object *try_to_spawn_object(s16 offsetY, f32 scale, struct Object *parent, ModelID32 model,
                                   const BehaviorScript *behavior) {
    if (gFreeObjectList.next != NULL) {
        struct Object *obj = spawn_object(parent, model, behavior);
        obj_add_up(obj, &obj->oPosVec, offsetY);
        obj_scale(obj, scale);
        return obj;
    } else {
        return NULL;
    }
}

struct Object *spawn_object_with_scale(struct Object *parent, ModelID32 model, const BehaviorScript *behavior, f32 scale) {
    struct Object *obj = spawn_object(parent, model, behavior);
    obj_scale(obj, scale);

    return obj;
}

struct Object *spawn_object_relative(s16 behaviorParam, s16 relativePosX, s16 relativePosY, s16 relativePosZ,
                                     struct Object *parent, ModelID32 model, const BehaviorScript *behavior) {
    struct Object *obj = spawn_object(parent, model, behavior);
    obj_set_parent_relative_pos(obj, relativePosX, relativePosY, relativePosZ);
    obj_add_local_offset(obj, &obj->oFaceAngleVec);

    obj->oBehParams2ndByte = behaviorParam;
    SET_BPARAM2(obj->oBehParams, behaviorParam);

    return obj;
}

struct Object *spawn_object_relative_with_scale(s16 behaviorParam, s16 relativePosX, s16 relativePosY,
                                                s16 relativePosZ, f32 scale, struct Object *parent,
                                                ModelID32 model, const BehaviorScript *behavior) {
    struct Object *obj = spawn_object_relative(behaviorParam, relativePosX, relativePosY, relativePosZ,
                                               parent, model, behavior);
    obj_scale(obj, scale);

    return obj;
}

void cur_obj_move_using_vel(void) {
    obj_offset_local(o, &o->oPosVec, &o->oPosVec, o->oVelX, o->oVelY, o->oVelZ);
}

void obj_copy_graph_y_offset(struct Object *dst, struct Object *src) {
    dst->oGraphYOffset = src->oGraphYOffset;
}

void obj_copy_pos_and_angle(struct Object *dst, struct Object *src) {
    obj_copy_pos(dst, src);
    obj_copy_angle(dst, src);
}

void obj_copy_pos(struct Object *dst, struct Object *src) {
    vec3f_copy(&dst->oPosVec, &src->oPosVec);
}

void obj_copy_angle(struct Object *dst, struct Object *src) {
    vec3i_copy(&dst->oMoveAngleVec, &src->oMoveAngleVec);
    vec3i_copy(&dst->oFaceAngleVec, &src->oFaceAngleVec);
    dst->oMoveAngleYaw = obj_yaw_in_frame(dst, src, src->oMoveAngleYaw);
    dst->oFaceAngleYaw = obj_yaw_in_frame(dst, src, src->oFaceAngleYaw);
}

void obj_set_gfx_pos_from_pos(struct Object *obj) {
    vec3f_copy(obj->header.gfx.pos, &obj->oPosVec);
}

void obj_init_animation(struct Object *obj, s32 animIndex) {
    struct Animation **anims = obj->oAnimations;
    geo_obj_init_animation(&obj->header.gfx, &anims[animIndex]);
}

void obj_copy_scale(struct Object *dst, struct Object *src) {
    vec3f_copy(dst->header.gfx.scale, src->header.gfx.scale);
}

void obj_scale_xyz(struct Object *obj, f32 xScale, f32 yScale, f32 zScale) {
    vec3f_set(obj->header.gfx.scale, xScale, yScale, zScale);
}

void obj_scale(struct Object *obj, f32 scale) {
    vec3_same(obj->header.gfx.scale, scale);
}

void cur_obj_scale(f32 scale) {
    vec3_same(o->header.gfx.scale, scale);
}

void cur_obj_init_animation(s32 animIndex) {
    struct Animation **anims = o->oAnimations;
    geo_obj_init_animation(&o->header.gfx, &anims[animIndex]);
}

void cur_obj_init_animation_with_sound(s32 animIndex) {
    struct Animation **anims = o->oAnimations;
    geo_obj_init_animation(&o->header.gfx, &anims[animIndex]);
    o->oSoundStateID = animIndex;
}

void cur_obj_init_animation_with_accel_and_sound(s32 animIndex, f32 accel) {
    struct Animation **anims = o->oAnimations;
    s32 animAccel = (s32)(accel * 65536.0f);
    geo_obj_init_animation_accel(&o->header.gfx, &anims[animIndex], animAccel);
    o->oSoundStateID = animIndex;
}

void obj_init_animation_with_sound(struct Object *obj, const struct Animation * const* animations, s32 animIndex) {
    struct Animation **anims = (struct Animation **)animations;
    obj->oAnimations = (struct Animation **)animations;
    geo_obj_init_animation(&obj->header.gfx, &anims[animIndex]);
    obj->oSoundStateID = animIndex;
}

void cur_obj_enable_rendering_and_become_tangible(struct Object *obj) {
    obj->header.gfx.node.flags |= GRAPH_RENDER_ACTIVE;
    obj->oIntangibleTimer = 0;
}

void cur_obj_enable_rendering(void) {
    o->header.gfx.node.flags |= GRAPH_RENDER_ACTIVE;
}

void cur_obj_disable_rendering_and_become_intangible(struct Object *obj) {
    obj->header.gfx.node.flags &= ~GRAPH_RENDER_ACTIVE;
    obj->oIntangibleTimer = -1;
}

void cur_obj_disable_rendering(void) {
    o->header.gfx.node.flags &= ~GRAPH_RENDER_ACTIVE;
}

void cur_obj_unhide(void) {
    o->header.gfx.node.flags &= ~GRAPH_RENDER_INVISIBLE;
}

void cur_obj_hide(void) {
    o->header.gfx.node.flags |= GRAPH_RENDER_INVISIBLE;
}

void cur_obj_set_pos_relative(struct Object *other, f32 dleft, f32 dy, f32 dforward) {
    f32 facingZ = coss(other->oMoveAngleYaw);
    f32 facingX = sins(other->oMoveAngleYaw);

    f32 dz = dforward * facingZ - dleft * facingX;
    f32 dx = dforward * facingX + dleft * facingZ;

    o->oMoveAngleYaw = obj_yaw_in_frame(o, other, other->oMoveAngleYaw);
    obj_offset_local(other, &o->oPosVec, &other->oPosVec, dx, dy, dz);
}

u32 get_object_list_from_behavior(const BehaviorScript *behavior) {
    u32 objectList;

    // If the first behavior command is "begin", then get the object list header
    // from there
    if ((behavior[0] >> 24) == 0) {
        objectList = (behavior[0] >> 16) & 0xFFFF;
    } else {
        objectList = OBJ_LIST_DEFAULT;
    }

    return objectList;
}

struct Object *cur_obj_nearest_object_with_behavior(const BehaviorScript *behavior) {
    f32 dist;
    return cur_obj_find_nearest_object_with_behavior(behavior, &dist);
}

f32 cur_obj_dist_to_nearest_object_with_behavior(const BehaviorScript *behavior) {
    f32 dist;
    if (cur_obj_find_nearest_object_with_behavior(behavior, &dist) == NULL) dist = 15000.0f;
    return dist;
}

struct Object *cur_obj_find_nearest_object_with_behavior(const BehaviorScript *behavior, f32 *dist) {
    uintptr_t *behaviorAddr = segmented_to_virtual(behavior);
    struct ObjectNode *listHead = &gObjectLists[get_object_list_from_behavior(behaviorAddr)];
    struct Object *obj = (struct Object *) listHead->next;
    struct Object *closestObj = NULL;
    f32 minDist = 0x20000;

    while (obj != (struct Object *) listHead) {
        if (obj->behavior == behaviorAddr
            && obj->activeFlags != ACTIVE_FLAG_DEACTIVATED
            && obj != o
        ) {
            f32 objDist = dist_between_objects(o, obj);
            if (objDist < minDist) {
                closestObj = obj;
                minDist = objDist;
            }
        }

        obj = (struct Object *) obj->header.next;
    }

    *dist = minDist;
    return closestObj;
}

struct Object *find_unimportant_object(void) {
    struct ObjectNode *listHead = &gObjectLists[OBJ_LIST_UNIMPORTANT];
    struct ObjectNode *obj = listHead->next;

    if (listHead == obj) {
        return NULL;
    }

    return (struct Object *) obj;
}

s32 count_unimportant_objects(void) {
    struct ObjectNode *listHead = &gObjectLists[OBJ_LIST_UNIMPORTANT];
    struct ObjectNode *obj = listHead->next;
    s32 count = 0;

    while (listHead != obj) {
        count++;
        obj = obj->next;
    }

    return count;
}

s32 count_objects_with_behavior(const BehaviorScript *behavior) {
    uintptr_t *behaviorAddr = segmented_to_virtual(behavior);
    struct ObjectNode *listHead = &gObjectLists[get_object_list_from_behavior(behaviorAddr)];
    struct ObjectNode *obj = listHead->next;
    s32 count = 0;

    while (listHead != obj) {
        if (((struct Object *) obj)->behavior == behaviorAddr) {
            count++;
        }

        obj = obj->next;
    }

    return count;
}

struct Object *cur_obj_find_nearby_held_actor(const BehaviorScript *behavior, f32 maxDist) {
    const BehaviorScript *behaviorAddr = segmented_to_virtual(behavior);
    struct ObjectNode *listHead = &gObjectLists[OBJ_LIST_GENACTOR];
    struct Object *obj = (struct Object *) listHead->next;
    struct Object *foundObj = NULL;

    while ((struct Object *) listHead != obj) {
        if (
            obj->behavior == behaviorAddr
            && obj->activeFlags != ACTIVE_FLAG_DEACTIVATED
            && obj->oHeldState != HELD_FREE
            && dist_between_objects(o, obj) < maxDist
        ) {
            // This includes the dropped and thrown states. By combining instant
            // release, this allows us to activate mama penguin remotely
            foundObj = obj;
            break;
        }

        obj = (struct Object *) obj->header.next;
    }

    return foundObj;
}

static void cur_obj_reset_timer_and_subaction(void) {
    o->oTimer = 0;
    o->oSubAction = 0;
}

void cur_obj_change_action(s32 action) {
    o->oAction = action;
    o->oPrevAction = action;
    cur_obj_reset_timer_and_subaction();
}

void cur_obj_set_vel_from_mario_vel(f32 min, f32 mul) {
    f32 marioFwdVel = gMarioStates[0].forwardVel;
    f32 minVel = min * mul;

    if (marioFwdVel < minVel) {
        o->oForwardVel = minVel;
    } else {
        o->oForwardVel = marioFwdVel * mul;
    }
}

void cur_obj_reverse_animation(void) {
    if (o->header.gfx.animInfo.animFrame >= 0) {
        o->header.gfx.animInfo.animFrame--;
    }
}

void cur_obj_extend_animation_if_at_end(void) {
    s32 animFrame = o->header.gfx.animInfo.animFrame;
    s32 nearLoopEnd = o->header.gfx.animInfo.curAnim->loopEnd - 2;

    if (animFrame == nearLoopEnd) o->header.gfx.animInfo.animFrame--;
}

s32 cur_obj_check_if_near_animation_end(void) {
    s32 animFrame = o->header.gfx.animInfo.animFrame;
    s32 nearLoopEnd = o->header.gfx.animInfo.curAnim->loopEnd - 2;

    if ((o->header.gfx.animInfo.curAnim->flags & ANIM_FLAG_NOLOOP) && nearLoopEnd + 1 == animFrame) {
        return TRUE;
    }

    return animFrame == nearLoopEnd;
}

s32 cur_obj_check_if_at_animation_end(void) {
    return o->header.gfx.animInfo.animFrame == o->header.gfx.animInfo.curAnim->loopEnd - 1;
}

s32 cur_obj_check_anim_frame(s32 frame) {
    return o->header.gfx.animInfo.animFrame == frame;
}

s32 cur_obj_check_anim_frame_in_range(s32 startFrame, s32 rangeLength) {
    s32 animFrame = o->header.gfx.animInfo.animFrame;

    return animFrame >= startFrame && animFrame < startFrame + rangeLength;
}

s32 cur_obj_check_frame_prior_current_frame(s16 *frame) {
    s16 animFrame = o->header.gfx.animInfo.animFrame;

    while (*frame != -1) {
        if (*frame == animFrame) return TRUE;

        frame++;
    }

    return FALSE;
}

s32 mario_is_in_air_action(void) {
    return gMarioStates[0].action & ACT_FLAG_AIR;
}

s32 mario_is_dive_sliding(void) {
    return gMarioStates[0].action == ACT_DIVE_SLIDE;
}

void cur_obj_set_y_vel_and_animation(f32 yVel, s32 animIndex) {
    o->oVelY = yVel;
    cur_obj_init_animation_with_sound(animIndex);
}

void cur_obj_unrender_set_action_and_anim(s32 animIndex, s32 action) {
    cur_obj_become_intangible();
    cur_obj_disable_rendering();

    // only set animation if non-negative value
    if (animIndex >= 0) {
        cur_obj_init_animation_with_sound(animIndex);
    }

    o->oAction = action;
}

static void set_object_floor(const struct SurfaceContact *floor, f32 height) {
    o->oFloorHeight = height;
    o->oFloor = floor->surface;
    o->oFloorType = floor->surface != NULL ? floor->type : SURFACE_DEFAULT;
    o->oFloorRoom = floor->surface != NULL ? floor->room : 0;
}

static void cur_obj_update_floor(f32 offset) {
    struct SurfaceContact floor;
    f32 height = find_actor_floor(o, &o->oPosVec, offset, &floor);
    set_object_floor(&floor, height);
}

static void cur_obj_move_after_thrown_or_dropped(f32 forwardVel, f32 velY) {
    o->oMoveFlags = 0;
    cur_obj_update_floor(160.0f);
    if (o->oFloor == NULL && is_outside_level_bounds(o->oPosX, o->oPosZ)) {
        obj_copy_pos(o, gMarioObject);
        cur_obj_update_floor(0.0f);
    }
    if (o->oFloor != NULL && o->oFloorHeight > obj_height(o, &o->oPosVec)) {
        obj_set_height(o, &o->oPosVec, o->oFloorHeight);
    }

    cur_obj_set_throw_velocity(forwardVel, velY);

    if (forwardVel != 0) {
        cur_obj_move_y(/*gravity*/ -4.0f, /*bounciness*/ -0.1f, /*buoyancy*/ 2.0f);
    }
}

void cur_obj_get_thrown_or_placed(f32 forwardVel, f32 velY, s32 thrownAction) {
    if (o->behavior == segmented_to_virtual(bhvBowser)) {
        // Interestingly, when bowser is thrown, he is offset slightly to
        // Mario's right
        cur_obj_set_pos_relative(o->parentObj, -41.684f, 85.859f, 321.577f);
    }

    cur_obj_become_tangible();
    cur_obj_enable_rendering();

    o->oHeldState = HELD_FREE;

    if ((o->oInteractionSubtype & INT_SUBTYPE_HOLDABLE_NPC) || forwardVel == 0.0f) {
        cur_obj_move_after_thrown_or_dropped(0.0f, 0.0f);
    } else {
        o->oAction = thrownAction;
        cur_obj_move_after_thrown_or_dropped(forwardVel, velY);
    }
}

void cur_obj_get_dropped(void) {
    cur_obj_become_tangible();
    cur_obj_enable_rendering();

    o->oHeldState = HELD_FREE;
    cur_obj_move_after_thrown_or_dropped(0.0f, 0.0f);
}

void obj_set_model(struct Object *obj, ModelID16 modelID) {
    obj->header.gfx.sharedChild = gLoadedGraphNodes[modelID];
}

void cur_obj_set_model(ModelID16 modelID) {
    o->header.gfx.sharedChild = gLoadedGraphNodes[modelID];
}

s32 obj_has_model(struct Object *obj, ModelID16 modelID) {
    return (obj->header.gfx.sharedChild == gLoadedGraphNodes[modelID]);
}

s32 cur_obj_has_model(ModelID16 modelID) {
    return (o->header.gfx.sharedChild == gLoadedGraphNodes[modelID]);
}

// HackerSM64 function
ModelID32 obj_get_model_id(struct Object *obj) {
    if (obj->header.gfx.sharedChild != NULL) {
        for (s32 i = MODEL_NONE; i < MODEL_ID_COUNT; i++) {
            if (obj->header.gfx.sharedChild == gLoadedGraphNodes[i]) {
                return i;
            }
        }
    }
    return MODEL_NONE;
}

void mario_set_flag(s32 flag) {
    gMarioStates[0].flags |= flag;
}

s32 cur_obj_clear_interact_status_flag(s32 flag) {
    if (o->oInteractStatus & flag) {
        o->oInteractStatus &= (flag ^ 0xFFFFFFFF);
        return TRUE;
    }
    return FALSE;
}

/**
 * Mark an object to be unloaded at the end of the frame.
 */
void obj_mark_for_deletion(struct Object *obj) {
    //! This clears all activeFlags. Since some of these flags disable behavior,
    //  setting it to 0 could potentially enable unexpected behavior. After an
    //  object is marked for deletion, it still updates on that frame (I think),
    //  so this is worth looking into.
    //! NOTE: Changing this can cause reference issues!
    obj->activeFlags = ACTIVE_FLAG_DEACTIVATED;
}

void cur_obj_disable(void) {
    cur_obj_disable_rendering();
    cur_obj_hide();
    cur_obj_become_intangible();
}

void cur_obj_become_intangible(void) {
    // When the timer is negative, the object is intangible and the timer
    // doesn't count down
    o->oIntangibleTimer = -1;
}

void cur_obj_become_tangible(void) {
    o->oIntangibleTimer = 0;
}

void obj_become_tangible(struct Object *obj) {
    obj->oIntangibleTimer = 0;
}

void cur_obj_update_floor_height(void) {
    cur_obj_update_floor(0.0f);
}

struct Surface *cur_obj_update_floor_height_and_get_floor(void) {
    cur_obj_update_floor(0.0f);
    return o->oFloor;
}

static void apply_drag_to_value(f32 *value, f32 dragStrength) {
    f32 decel;

    if (*value != 0) {
        //! Can overshoot if |*value| > 1 / (dragStrength * 0.0001)
        decel = sqr(*value) * (dragStrength * 0.0001f);

        if (*value > 0) {
            *value -= decel;
            if (*value < 0.001f) {
                *value = 0;
            }
        } else {
            *value += decel;
            if (*value > -0.001f) {
                *value = 0;
            }
        }
    }
}

void cur_obj_apply_drag_xz(f32 dragStrength) {
    apply_drag_to_value(&o->oVelX, dragStrength);
    apply_drag_to_value(&o->oVelZ, dragStrength);
}

static void cur_obj_move_xz(f32 steepSlopeNormalY, s32 careAboutEdgesAndSteepSlopes) {
    o->oMoveFlags &= ~OBJ_MOVE_HIT_EDGE;
    if (o->oVelX == 0.0f && o->oVelZ == 0.0f) return;

    Vec3f pos;
    obj_offset_local(o, pos, &o->oPosVec, o->oVelX, 0.0f, o->oVelZ);
    struct SurfaceContact floor;
    f32 height = find_actor_floor(o, pos, 0.0f, &floor);
    if (is_outside_level_bounds(pos[0], pos[2]) || (careAboutEdgesAndSteepSlopes && floor.surface == NULL)) {
        o->oMoveFlags |= OBJ_MOVE_HIT_EDGE;
        return;
    }
    if (floor.surface == NULL) {
        vec3_copy(&o->oPosVec, pos);
        set_object_floor(&floor, height);
        return;
    }
    f32 normalUp = obj_normal_up(o, floor.normal);
    if (o->oRoom != -1 && floor.room != 0 && o->oRoom != floor.room && floor.room != 18) return;

    f32 rise = height - o->oFloorHeight;
    if (rise < 5.0f) {
        if (careAboutEdgesAndSteepSlopes
            && (normalUp <= steepSlopeNormalY || (rise < -50.0f && (o->oMoveFlags & OBJ_MOVE_ON_GROUND)))) {
            o->oMoveFlags |= OBJ_MOVE_HIT_EDGE;
            return;
        }
    } else if (normalUp <= steepSlopeNormalY && obj_height(o, pos) <= height) {
        return;
    }

    vec3_copy(&o->oPosVec, pos);
    set_object_floor(&floor, height);
}

static void cur_obj_land(f32 bounciness, u32 previousFlags) {
    if (o->oFloor != NULL && obj_height(o, &o->oPosVec) <= o->oFloorHeight) {
        obj_set_height(o, &o->oPosVec, o->oFloorHeight);
        o->oMoveFlags |= (previousFlags & OBJ_MOVE_MASK_ON_GROUND) ? OBJ_MOVE_ON_GROUND : OBJ_MOVE_LANDED;
        if (o->oVelY < 0.0f) {
            o->oVelY *= bounciness;
            if (o->oVelY > 5.0f) o->oMoveFlags |= OBJ_MOVE_BOUNCE;
        }
    } else {
        o->oMoveFlags |= OBJ_MOVE_IN_AIR;
        if (previousFlags & OBJ_MOVE_ON_GROUND) o->oMoveFlags |= OBJ_MOVE_LEFT_GROUND;
    }
}

void cur_obj_move_y(f32 gravity, f32 bounciness, f32 buoyancy) {
    u32 previousFlags = o->oMoveFlags;
    f32 waterLevel = (o->activeFlags & ACTIVE_FLAG_IGNORE_ENV_BOXES) ? FLOOR_LOWER_LIMIT
                                                                    : find_water_level(o->oPosX, o->oPosZ);
    // Water boxes are world-horizontal. Compare their surface to the world Y of the support point.
    f32 floorY = obj_world_y_at_height(o, &o->oPosVec, o->oFloorHeight);
    s32 waterAboveFloor = waterLevel > FLOOR_LOWER_LIMIT && (o->oFloor == NULL || waterLevel > floorY || o->gravityDir[1] >= 0.0f);
    s32 inWater = waterAboveFloor && o->oPosY <= waterLevel && (previousFlags & OBJ_MOVE_MASK_IN_WATER);

    o->oMoveFlags &= ~(OBJ_MOVE_MASK_ON_GROUND | OBJ_MOVE_MASK_IN_WATER | OBJ_MOVE_IN_AIR
                     | OBJ_MOVE_LEFT_GROUND | OBJ_MOVE_LEAVING_WATER | OBJ_MOVE_BOUNCE
                     | OBJ_MOVE_ABOVE_LAVA | OBJ_MOVE_ABOVE_DEATH_BARRIER);
    if (inWater && (previousFlags & OBJ_MOVE_AT_WATER_SURFACE) && o->oVelY > 5.0f) {
        inWater = FALSE;
        o->oMoveFlags |= OBJ_MOVE_LEAVING_WATER;
    }

    o->oVelY += gravity + (inWater ? buoyancy : 0.0f);
    if (o->oVelY < -78.0f) o->oVelY = -78.0f;
    obj_add_up(o, &o->oPosVec, o->oVelY);

    if (o->oFloorType == SURFACE_BURNING) {
        o->oMoveFlags |= OBJ_MOVE_ABOVE_LAVA;
    } else if (o->oFloorType == SURFACE_DEATH_PLANE || o->oFloorType == SURFACE_VERTICAL_WIND) {
        o->oMoveFlags |= OBJ_MOVE_ABOVE_DEATH_BARRIER;
    }

    if (!waterAboveFloor || (!inWater && o->oPosY > waterLevel)) {
        cur_obj_land(bounciness, previousFlags);
        return;
    }

    if (!inWater) o->oMoveFlags |= OBJ_MOVE_ENTERED_WATER;
    if (o->oFloor != NULL && obj_height(o, &o->oPosVec) <= o->oFloorHeight) {
        obj_set_height(o, &o->oPosVec, o->oFloorHeight);
        o->oVelY = 0.0f;
        o->oMoveFlags |= OBJ_MOVE_UNDERWATER_ON_GROUND;
    } else if (o->oPosY >= waterLevel && o->gravityDir[1] < -0.001f) {
        obj_add_up(o, &o->oPosVec, (o->oPosY - waterLevel) / o->gravityDir[1]);
        o->oVelY = 0.0f;
        o->oMoveFlags |= OBJ_MOVE_AT_WATER_SURFACE;
    } else {
        o->oVelY *= 1.0f - o->oDragStrength * 0.07f;
        o->oMoveFlags |= OBJ_MOVE_UNDERWATER_OFF_GROUND;
    }
}

void cur_obj_move_xz_using_fvel_and_yaw(void) {
    cur_obj_compute_vel_xz();
    obj_offset_local(o, &o->oPosVec, &o->oPosVec, o->oVelX, 0.0f, o->oVelZ);
}

void cur_obj_move_y_with_terminal_vel(void) {
    if (o->oVelY < -70.0f) {
        o->oVelY = -70.0f;
    }

    obj_add_up(o, &o->oPosVec, o->oVelY);
}

void cur_obj_compute_vel_xz(void) {
    o->oVelX = o->oForwardVel * sins(o->oMoveAngleYaw);
    o->oVelZ = o->oForwardVel * coss(o->oMoveAngleYaw);
}

f32 increment_velocity_toward_range(f32 value, f32 center, f32 zeroThreshold, f32 increment) {
    f32 relative;
    if ((relative = value - center) > 0) {
        if (relative < zeroThreshold) {
            return 0.0f;
        } else {
            return -increment;
        }
    } else {
        if (relative > -zeroThreshold) {
            return 0.0f;
        } else {
            return increment;
        }
    }
}

s32 obj_check_if_collided_with_object(struct Object *obj1, struct Object *obj2) {
    s32 i;
    for (i = 0; i < obj1->numCollidedObjs; i++) {
        if (obj1->collidedObjs[i] == obj2) {
            return TRUE;
        }
    }

    return FALSE;
}

void cur_obj_set_behavior(const BehaviorScript *behavior) {
    o->behavior = segmented_to_virtual(behavior);
}

void obj_set_behavior(struct Object *obj, const BehaviorScript *behavior) {
    obj->behavior = segmented_to_virtual(behavior);
}

s32 cur_obj_has_behavior(const BehaviorScript *behavior) {
    return (o->behavior == segmented_to_virtual(behavior));
}

s32 obj_has_behavior(struct Object *obj, const BehaviorScript *behavior) {
    return (obj->behavior == segmented_to_virtual(behavior));
}

f32 cur_obj_lateral_dist_from_mario_to_home(void) {
    return obj_lateral_distance(o, &o->oHomeVec, &gMarioObject->oPosVec);
}

f32 cur_obj_lateral_dist_to_home(void) {
    return obj_lateral_distance(o, &o->oHomeVec, &o->oPosVec);
}

void cur_obj_set_pos_to_home(void) {
    vec3f_copy(&o->oPosVec, &o->oHomeVec);
}

void cur_obj_set_pos_to_home_and_stop(void) {
    cur_obj_set_pos_to_home();

    o->oForwardVel = 0;
    o->oVelY = 0;
}

void cur_obj_shake_y(f32 amount) {
    //! Technically could cause a bit of drift, but not much
    if ((o->oTimer & 0x1) == 0) {
        obj_add_up(o, &o->oPosVec, amount);
    } else {
        obj_add_up(o, &o->oPosVec, -amount);
    }
}

void cur_obj_start_cam_event(UNUSED struct Object *obj, s32 cameraEvent) {
    gPlayerCameraState->cameraEvent = (s16) cameraEvent;
    gSecondCameraFocus = o;
}

void obj_set_billboard(struct Object *obj) {
    obj->header.gfx.node.flags |= GRAPH_RENDER_BILLBOARD;
}

static void obj_spawn_loot_coins(struct Object *obj, s32 numCoins, f32 baseYVel,
                                    const BehaviorScript *coinBehavior,
                                    s16 posJitter, ModelID16 model) {
    s32 i;
    struct Object *coin;

    f32 spawnHeight = find_actor_floor(obj, &obj->oPosVec, 0.0f, NULL);
    f32 height = obj_height(obj, &obj->oPosVec);
    if (height - spawnHeight > 100.0f) {
        spawnHeight = height;
    }

    for (i = 0; i < numCoins; i++) {
        if (obj->oNumLootCoins <= 0) {
            break;
        }

        obj->oNumLootCoins--;

        coin = spawn_object(obj, model, coinBehavior);
        obj_translate_xz_random(coin, posJitter);
        obj_set_height(coin, &coin->oPosVec, spawnHeight);
        coin->oCoinBaseYVel = baseYVel;
    }
}

void obj_spawn_loot_blue_coins(struct Object *obj, s32 numCoins, f32 baseYVel, s16 posJitter) {
    obj_spawn_loot_coins(obj, numCoins, baseYVel, bhvBlueCoinJumping, posJitter, MODEL_BLUE_COIN);
}

void obj_spawn_loot_yellow_coins(struct Object *obj, s32 numCoins, f32 baseYVel) {
    obj_spawn_loot_coins(obj, numCoins, baseYVel, bhvSingleCoinGetsSpawned, 0, MODEL_YELLOW_COIN);
}

void cur_obj_spawn_loot_coin_at_mario_pos(void) {
    if (o->oNumLootCoins <= 0) {
        return;
    }

    o->oNumLootCoins--;

    struct Object *coin = spawn_object(o, MODEL_YELLOW_COIN, bhvSingleCoinGetsSpawned);
    coin->oVelY = 30.0f;

    obj_copy_pos(coin, gMarioObject);
}

UNUSED f32 cur_obj_abs_y_dist_to_home(void) {
    return absf(obj_height_difference(o, &o->oHomeVec, &o->oPosVec));
}

UNUSED s32 cur_obj_advance_looping_anim(void) {
    s32 animFrame = o->header.gfx.animInfo.animFrame;
    s32 loopEnd = o->header.gfx.animInfo.curAnim->loopEnd;

    if (animFrame < 0) {
        animFrame = 0;
    } else if (loopEnd - 1 == animFrame) {
        animFrame = 0;
    } else {
        animFrame++;
    }

    return (animFrame << 16) / loopEnd;
}

static s32 cur_obj_detect_steep_floor(void) {
    if (o->oForwardVel == 0.0f || (o->oVelX == 0.0f && o->oVelZ == 0.0f)) return FALSE;
    struct SurfaceContact floor;
    Vec3f pos;
    obj_offset_local(o, pos, &o->oPosVec, o->oVelX, 0.0f, o->oVelZ);
    f32 height = find_actor_floor(o, pos, 0.0f, &floor);
    if (floor.surface != NULL && height > o->oFloorHeight && height > obj_height(o, pos) && obj_normal_up(o, floor.normal) < 0.5f) {
        struct Normal n = obj_local_normal(o, floor.normal);
        o->oWallAngle = atan2s(n.z, n.x);
        return TRUE;
    }
    return FALSE;
}

s32 cur_obj_resolve_wall_collisions(void) {
    struct SurfaceContact wall;
    if (o->oWallHitboxRadius <= 0.1f
        || !resolve_actor_wall(o, &o->oPosVec, 10.0f, o->oWallHitboxRadius, 1.0f, &wall)) return FALSE;
    struct Normal n = obj_local_normal(o, wall.normal);
    o->oWallAngle = atan2s(n.z, n.x);
    return abs_angle_diff(o->oWallAngle, o->oMoveAngleYaw) > 0x4000;
}

void cur_obj_update_floor_and_walls(void) {
    if (o->activeFlags & (ACTIVE_FLAG_FAR_AWAY | ACTIVE_FLAG_IN_DIFFERENT_ROOM)) {
        cur_obj_update_floor(0.0f);
        o->oMoveFlags &= ~(OBJ_MOVE_HIT_WALL | OBJ_MOVE_MASK_IN_WATER);
    } else {
        s32 hitWall = cur_obj_resolve_wall_collisions();
        cur_obj_update_floor(0.0f);
        COND_BIT(hitWall || cur_obj_detect_steep_floor(), o->oMoveFlags, OBJ_MOVE_HIT_WALL);
    }
    if (o->oFloor == NULL || obj_height(o, &o->oPosVec) > o->oFloorHeight) o->oMoveFlags |= OBJ_MOVE_IN_AIR;
}

void cur_obj_update_floor_and_bounce_off_walls(void) {
    cur_obj_update_floor_and_walls();
    if (o->oMoveFlags & OBJ_MOVE_HIT_WALL) {
        o->oMoveAngleYaw = cur_obj_reflect_move_angle_off_wall();
    }
}

void cur_obj_move_standard(s16 steepSlopeAngleDegrees) {
    if (o->activeFlags & (ACTIVE_FLAG_FAR_AWAY | ACTIVE_FLAG_IN_DIFFERENT_ROOM)) return;

    s32 avoidEdges = (steepSlopeAngleDegrees < 0);
    if (avoidEdges) steepSlopeAngleDegrees = -steepSlopeAngleDegrees;
    f32 normalY = steepSlopeAngleDegrees == 78 ? COS78 : coss(DEGREES(steepSlopeAngleDegrees));

    cur_obj_compute_vel_xz();
    if (o->oDragStrength != 0.0f && o->oForwardVel != 0.0f) {
        cur_obj_apply_drag_xz(o->oDragStrength);
        f32 speed = sqrtf(sqr(o->oVelX) + sqr(o->oVelZ));
        o->oForwardVel = o->oForwardVel < 0.0f ? -speed : speed;
    }
    cur_obj_move_xz(normalY, avoidEdges);
    cur_obj_move_y(o->oGravity, o->oBounciness, o->oBuoyancy);
}

UNUSED static s32 cur_obj_within_bounds(f32 bounds) {
    if (o->oPosX < -bounds || bounds < o->oPosX) return FALSE;
    if (o->oPosY < -bounds || bounds < o->oPosY) return FALSE;
    if (o->oPosZ < -bounds || bounds < o->oPosZ) return FALSE;
    return TRUE;
}

void cur_obj_move_using_vel_and_gravity(void) {
    o->oVelY += o->oGravity; //! No terminal velocity
    cur_obj_move_using_vel();
}

void cur_obj_move_using_fvel_and_gravity(void) {
    cur_obj_compute_vel_xz();
    cur_obj_move_using_vel_and_gravity(); //! No terminal velocity
}

s32 cur_obj_angle_to_home(void) {
    Vec3f delta;
    vec3_diff(delta, &o->oHomeVec, &o->oPosVec);
    return obj_yaw_from_direction(o, delta);
}

void obj_set_gfx_pos_at_obj_pos(struct Object *obj1, struct Object *obj2) {
    vec3f_copy(obj1->header.gfx.pos, &obj2->oPosVec);
    obj_add_up(obj2, obj1->header.gfx.pos, obj2->oGraphYOffset);

    obj1->header.gfx.angle[0] = obj2->oMoveAnglePitch & 0xFFFF;
    obj1->header.gfx.angle[1] = obj_yaw_in_frame(obj1, obj2, obj2->oMoveAngleYaw);
    obj1->header.gfx.angle[2] = obj2->oMoveAngleRoll & 0xFFFF;
}

void obj_yaw_direction(const struct Object *obj, Vec3f dest, s16 yaw) {
    vec3_set(dest, sins(yaw), 0.0f, coss(yaw));
    if (obj->header.gfx.transformFlags & GRAPH_OBJ_GRAVITY_FRAME) {
        quat_rotate_vec3f(dest, &obj->gravityFrame, dest);
    }
}

s16 obj_yaw_from_direction(const struct Object *obj, const Vec3f direction) {
    Vec3f local;
    obj_world_to_local(obj, local, direction);
    return atan2s(local[2], local[0]);
}

// Project a heading into the receiver's tangent plane. At a pole its right axis still defines yaw.
s16 obj_yaw_in_frame(const struct Object *dest, const struct Object *src, s16 yaw) {
    if (src == dest || (!(src && (src->header.gfx.transformFlags & GRAPH_OBJ_GRAVITY_FRAME))
                     && !(dest && (dest->header.gfx.transformFlags & GRAPH_OBJ_GRAVITY_FRAME)))) return yaw;
    Vec3f dir = { sins(yaw), 0.0f, coss(yaw) };
    if (src != NULL) obj_local_to_world(src, dir, dir);
    if (dest != NULL) obj_world_to_local(dest, dir, dir);
    if (sqr(dir[0]) + sqr(dir[2]) > 0.000001f) return atan2s(dir[2], dir[0]);
    vec3_set(dir, coss(yaw), 0.0f, -sins(yaw));
    if (src != NULL) obj_local_to_world(src, dir, dir);
    if (dest != NULL) obj_world_to_local(dest, dir, dir);
    return atan2s(dir[2], dir[0]) - 0x4000;
}

// A release carries the thrower's entire velocity into the other actor's tangent plane and vertical axis.
s16 obj_convert_launch(const struct Object *dest, const struct Object *src, s16 yaw, f32 *forward, f32 *up) {
    if (src->gravityDir[0] == dest->gravityDir[0] && src->gravityDir[1] == dest->gravityDir[1]
        && src->gravityDir[2] == dest->gravityDir[2]) return obj_yaw_in_frame(dest, src, yaw);
    Vec3f vel = { *forward * sins(yaw), *up, *forward * coss(yaw) };
    obj_local_to_world(src, vel, vel);
    obj_world_to_local(dest, vel, vel);
    f32 lateral = sqrtf(sqr(vel[0]) + sqr(vel[2]));
    s32 backwards = *forward < 0.0f;
    *up = vel[1];
    *forward = backwards ? -lateral : lateral;
    if (lateral < 0.001f) return obj_yaw_in_frame(dest, src, yaw);
    return atan2s(vel[2], vel[0]) + (backwards ? 0x8000 : 0);
}

void cur_obj_set_throw_velocity(f32 forward, f32 up) {
    o->oMoveAngleYaw = obj_convert_launch(o, gMarioObject, gMarioState->faceAngle[1], &forward, &up);
    o->oForwardVel = forward;
    o->oVelY = up;
}

// Carry yaw zero along with gravity. At exactly opposite directions, turn about the actor's heading.
void obj_set_gravity_direction(struct Object *obj, const Vec3f down) {
    Vec3f direction, heading;
    Quat turn;
    if (down[0] == obj->gravityDir[0] && down[1] == obj->gravityDir[1] && down[2] == obj->gravityDir[2]) return;
    f32 length = vec3_mag(down);
    if (length < NEAR_ZERO) return;
    vec3_scale_dest(direction, down, 1.0f / length);
    obj_yaw_direction(obj, heading, obj->oFaceAngleYaw);
    quat_from_to(&turn, obj->gravityDir, direction, heading);
    quat_mul(&obj->gravityFrame, &turn, &obj->gravityFrame);
    quat_normalize(&obj->gravityFrame);
    vec3_copy(obj->gravityDir, direction);
    COND_BIT(obj->gravityFrame.x != 0 || obj->gravityFrame.y != 0 || obj->gravityFrame.z != 0,
             obj->header.gfx.transformFlags, GRAPH_OBJ_GRAVITY_FRAME);
    obj->oFloor = NULL;
    obj->oMoveFlags &= ~(OBJ_MOVE_MASK_ON_GROUND | OBJ_MOVE_HIT_WALL);
    obj->platform = NULL;
    if (obj == gMarioObject && gMarioState != NULL && gMarioState->marioObj == obj) {
        gMarioState->floor = gMarioState->ceil = NULL;
        gMarioState->hasWall = FALSE;
        gMarioState->peakHeight = obj_height(obj, gMarioState->pos);
    }
    if (obj->header.gfx.rotation != NULL) {
        quat_mul(&obj->orientation, &turn, obj->header.gfx.rotation);
        quat_normalize(&obj->orientation);
        obj->header.gfx.rotation = &obj->orientation;
    }
}

void obj_align_to_normal(struct Object *obj, const Vec3f normal, s16 yaw) {
    if (normal[1] == 1.0f && !(obj->header.gfx.transformFlags & GRAPH_OBJ_GRAVITY_FRAME)) {
        quat_from_yaw(&obj->orientation, yaw);
    } else {
        Vec3f forward;
        obj_yaw_direction(obj, forward, yaw);
        quat_from_up_forward(&obj->orientation, normal, forward);
    }
    obj->header.gfx.rotation = &obj->orientation;
}

static void obj_rotation_from_angles(struct Object *obj, Quat *rotation, const Vec3s angles) {
    quat_from_zxy(rotation, angles);
    if (obj->header.gfx.transformFlags & GRAPH_OBJ_GRAVITY_FRAME) {
        quat_mul(rotation, &obj->gravityFrame, rotation);
    }
}

void obj_hitbox_up(struct Object *obj, Vec3f up) {
    struct GraphNodeObject *gfx = &obj->header.gfx;
    if (obj->oFlags & OBJ_FLAG_HITBOX_UP_FROM_GRAVITY) {
        vec3_scale_dest(up, obj->gravityDir, -1.0f);
    } else if (gfx->transformFlags & GRAPH_OBJ_PARENT) {
        Mat4 transform;
        obj_build_transform(obj, transform);
        vec3_copy(up, transform[1]);
        vec3_normalize(up);
        if (gfx->scale[1] < 0.0f) vec3_scale(up, -1.0f);
    } else if (gfx->rotation != NULL) {
        quat_rotate_vec3f(up, gfx->rotation, gVec3fY);
    } else if ((s16) obj->oFaceAnglePitch || (s16) obj->oFaceAngleRoll) {
        Vec3s angles;
        Quat rotation;
        vec3i_to_vec3s(angles, &obj->oFaceAngleVec);
        obj_rotation_from_angles(obj, &rotation, angles);
        quat_rotate_vec3f(up, &rotation, gVec3fY);
    } else {
        vec3_scale_dest(up, obj->gravityDir, -1.0f);
    }
}

void obj_update_gfx_pos_and_angle(struct Object *obj) {
    vec3_copy(obj->header.gfx.pos, &obj->oPosVec);
    if (obj->oGraphYOffset != 0.0f) obj_add_up(obj, obj->header.gfx.pos, obj->oGraphYOffset);
    vec3i_to_vec3s(obj->header.gfx.angle, &obj->oFaceAngleVec);
}

void obj_get_gfx_rotation(struct Object *obj, Quat *rotation) {
    if (obj->header.gfx.rotation != NULL) {
        *rotation = *obj->header.gfx.rotation;
    } else {
        obj_rotation_from_angles(obj, rotation, obj->header.gfx.angle);
    }
}

static void obj_build_local_transform(struct GraphNodeObject *gfx, Mat4 dest, Vec3f pos, Vec3s angles) {
    if (gfx->rotation != NULL) {
        mtxf_from_quat(dest, gfx->rotation, pos);
    } else if ((gfx->transformFlags & (GRAPH_OBJ_PARENT | GRAPH_OBJ_GRAVITY_FRAME)) == GRAPH_OBJ_GRAVITY_FRAME) {
        Quat rotation;
        obj_rotation_from_angles((struct Object *) gfx, &rotation, angles);
        mtxf_from_quat(dest, &rotation, pos);
    } else {
        mtxf_rotate_zxy_and_translate(dest, pos, angles);
    }
    mtxf_scale_vec3f(dest, dest, gfx->scale);
}

static void obj_build_actor_local_transform(struct Object *obj, Mat4 dest) {
    Vec3s angles;
    vec3i_to_vec3s(angles, &obj->oFaceAngleVec);
    f32 *pos = (obj->header.gfx.transformFlags & GRAPH_OBJ_PARENT) ? &obj->oParentRelativePosVec : &obj->oPosVec;
    obj_build_local_transform(&obj->header.gfx, dest, pos, angles);
}

void obj_build_transform(struct Object *obj, Mat4 dest) {
    obj_build_actor_local_transform(obj, dest);
    while (obj->header.gfx.transformFlags & GRAPH_OBJ_PARENT) {
        Mat4 parent;
        obj = obj->parentObj;
        obj_build_actor_local_transform(obj, parent);
        mtxf_mul(dest, dest, parent);
    }
}

void obj_build_gfx_transform(struct GraphNodeObject *gfx, Mat4 dest) {
    if (gfx->transformFlags & GRAPH_OBJ_PARENT) {
        obj_build_transform((struct Object *) gfx, dest);
    } else {
        obj_build_local_transform(gfx, dest, gfx->pos, gfx->angle);
    }
}

void obj_update_relative_pos(struct Object *obj) {
    Mat4 parent;
    obj_build_transform(obj->parentObj, parent);
    linear_mtxf_mul_vec3_and_translate(parent, &obj->oPosVec, &obj->oParentRelativePosVec);
}

static void obj_rotate_local_vector(struct Object *obj, Vec3f dest, const Vec3f vec, const Vec3i angles) {
    Vec3s rot;
    Quat orientation;
    vec3i_to_vec3s(rot, angles);
    obj_rotation_from_angles(obj, &orientation, rot);
    quat_rotate_vec3f(dest, &orientation, vec);
}

void obj_add_local_offset(struct Object *obj, const Vec3i angles) {
    Vec3f offset;
    obj_rotate_local_vector(obj, offset, &obj->oParentRelativePosVec, angles);
    vec3_add(&obj->oPosVec, offset);
}

void cur_obj_rotate_move_angle_using_vel(void) {
    vec3i_add(&o->oMoveAngleVec, &o->oAngleVelVec);
}

void cur_obj_rotate_face_angle_using_vel(void) {
    vec3i_add(&o->oFaceAngleVec, &o->oAngleVelVec);
}

void cur_obj_set_face_angle_to_move_angle(void) {
    vec3i_copy(&o->oFaceAngleVec, &o->oMoveAngleVec);
}

// removed unused arg: define allows for any number of args
s32 cur_obj_follow_path_new(void) {
    if (o->oPathedPrevWaypointFlags == 0) {
        o->oPathedPrevWaypoint = o->oPathedStartWaypoint;
        o->oPathedPrevWaypointFlags = WAYPOINT_FLAGS_INITIALIZED;
    }

    struct Waypoint *startWaypoint = o->oPathedStartWaypoint;
    struct Waypoint *lastWaypoint = o->oPathedPrevWaypoint;
    struct Waypoint *targetWaypoint;
    Vec3f prevToNext, objToNext, local;

    if ((lastWaypoint + 1)->flags != WAYPOINT_FLAGS_END) {
        targetWaypoint = lastWaypoint + 1;
    } else {
        targetWaypoint = startWaypoint;
    }

    o->oPathedPrevWaypointFlags = lastWaypoint->flags | WAYPOINT_FLAGS_INITIALIZED;

    vec3_diff(prevToNext, targetWaypoint->pos, lastWaypoint->pos);
    vec3_diff(objToNext, targetWaypoint->pos, &o->oPosVec);

    obj_world_to_local(o, local, objToNext);
    o->oPathedTargetYaw = atan2s(local[2], local[0]);
    o->oPathedTargetPitch = atan2s(sqrtf(sqr(local[0]) + sqr(local[2])), -local[1]);

    // If dot(prevToNext, objToNext) <= 0 (i.e. reached other side of target waypoint)
    if (vec3_dot(prevToNext, objToNext) <= 0.0f) {
        o->oPathedPrevWaypoint = targetWaypoint;
        if ((targetWaypoint + 1)->flags == WAYPOINT_FLAGS_END) {
            return PATH_REACHED_END;
        } else {
            return PATH_REACHED_WAYPOINT;
        }
    }

    return PATH_NONE;
}

void chain_segment_init(struct ChainSegment *segment) {
    vec3_zero(segment->pos);
    vec3_zero(segment->angle);
}

f32 random_f32_around_zero(f32 diameter) {
    return random_float() * diameter - diameter / 2;
}

void obj_scale_random(struct Object *obj, f32 rangeLength, f32 minScale) {
    f32 scale = random_float() * rangeLength + minScale;
    obj_scale(obj, scale);
}

void obj_translate_xyz_random(struct Object *obj, f32 rangeLength) {
    obj->oPosX += random_f32_around_zero(rangeLength);
    obj->oPosY += random_f32_around_zero(rangeLength);
    obj->oPosZ += random_f32_around_zero(rangeLength);
}

void obj_translate_xz_random(struct Object *obj, f32 rangeLength) {
    f32 x = random_f32_around_zero(rangeLength);
    f32 z = random_f32_around_zero(rangeLength);
    obj_offset_local(obj, &obj->oPosVec, &obj->oPosVec, x, 0.0f, z);
}

void cur_obj_set_pos_via_transform(void) {
    Vec3f vel = { o->oLeftVel, o->oUpVel, o->oForwardVel };
    Vec3s angles;
    Quat rotation;
    vec3i_to_vec3s(angles, &o->oMoveAngleVec);
    quat_from_zxy(&rotation, angles);
    quat_rotate_vec3f(&o->oVelVec, &rotation, vel);
    cur_obj_move_using_vel();
}

s32 cur_obj_reflect_move_angle_off_wall(void) {
    return (s16)(o->oWallAngle - ((s16) o->oMoveAngleYaw - (s16) o->oWallAngle) + 0x8000);
}

void cur_obj_spawn_particles(struct SpawnParticlesInfo *info) {
    struct Object *particle;
    s32 i;
    f32 scale;
    s32 numParticles = info->count;

    // If there are a lot of objects already, limit the number of particles
    if ((gPrevFrameObjectCount > (OBJECT_POOL_CAPACITY - 90)) && numParticles > 10) {
        numParticles = 10;
    }

    // We're close to running out of object slots, so don't spawn particles at
    // all
    if (gPrevFrameObjectCount > (OBJECT_POOL_CAPACITY - 30)) {
        numParticles = 0;
    }

    for (i = 0; i < numParticles; i++) {
        scale = random_float() * (info->sizeRange * 0.1f) + info->sizeBase * 0.1f;

        particle = spawn_object(o, info->model, bhvWhitePuffExplosion);

        particle->oBehParams2ndByte = info->behParam;
        particle->oMoveAngleYaw = random_u16();
        particle->oGravity = info->gravity;
        particle->oDragStrength = info->dragStrength;

        obj_add_up(particle, &particle->oPosVec, info->offsetY);
        particle->oForwardVel = random_float() * info->forwardVelRange + info->forwardVelBase;
        particle->oVelY = random_float() * info->velYRange + info->velYBase;

        obj_scale(particle, scale);
    }
}

void obj_set_hitbox(struct Object *obj, struct ObjectHitbox *hitbox) {
    if (!(obj->oFlags & OBJ_FLAG_HITBOX_WAS_SET)) {
        obj->oFlags |= OBJ_FLAG_HITBOX_WAS_SET;

        obj->oInteractType = hitbox->interactType;
        obj->oDamageOrCoinValue = hitbox->damageOrCoinValue;
        obj->oHealth = hitbox->health;
        obj->oNumLootCoins = hitbox->numLootCoins;

        obj_become_tangible(obj);
    }

    f32 scale = absf(obj->header.gfx.scale[0]);
    obj->hitboxRadius = scale * hitbox->radius;
    obj->interactionHalfHeight = 0.0f;
    obj->hitboxOffset = obj->header.gfx.scale[1] * hitbox->offset;
    obj->hurtboxRadius = scale * hitbox->hurtboxRadius;
    obj->hurtboxOffset = obj->header.gfx.scale[1] * hitbox->hurtboxOffset;
}

s32 cur_obj_wait_then_blink(s32 timeUntilBlinking, s32 numBlinks) {
    if (o->oTimer >= timeUntilBlinking) {
        s32 timeBlinking = o->oTimer - timeUntilBlinking;
        if (timeBlinking & 0x1) {
            o->header.gfx.node.flags |= GRAPH_RENDER_INVISIBLE;
            if (timeBlinking / 2 > numBlinks) {
                return TRUE;
            }
        } else {
            o->header.gfx.node.flags &= ~GRAPH_RENDER_INVISIBLE;
        }
    }

    return FALSE;
}

s32 cur_obj_is_mario_ground_pounding_platform(void) {
    if (gMarioObject->platform == o) {
        if (gMarioStates[0].action == ACT_GROUND_POUND_LAND) {
            return TRUE;
        }
    }

    return FALSE;
}

void spawn_mist_particles(void) {
    spawn_mist_particles_variable(0, 0, 46.0f);
}

void spawn_mist_particles_with_sound(u32 soundMagic) {
    spawn_mist_particles_variable(0, 0, 46.0f);
    create_sound_spawner(soundMagic);
}


void bhv_dust_smoke_loop(void) {
    cur_obj_move_using_vel();

    if (o->oSmokeTimer == 10) {
        obj_mark_for_deletion(o);
    }

    o->oSmokeTimer++;
}

s32 cur_obj_set_direction_table(s8 *pattern) {
    o->oToxBoxMovementPattern = pattern;
    o->oToxBoxMovementStep = 0;

    return *(s8 *) o->oToxBoxMovementPattern;
}

s32 cur_obj_progress_direction_table(void) {
    s8 action;
    s8 *pattern  = o->oToxBoxMovementPattern;
    s32 nextStep = o->oToxBoxMovementStep + 1;

    if (pattern[nextStep] != -1) {
        action = pattern[nextStep];
        o->oToxBoxMovementStep++;
    } else {
        action = pattern[0];
        o->oToxBoxMovementStep = 0;
    }

    return action;
}

void cur_obj_scale_over_time(s32 axis, s32 times, f32 start, f32 end) {
    f32 scale = ((end - start) * ((f32) o->oTimer / times)) + start;

    if (axis & SCALE_AXIS_X) {
        o->header.gfx.scale[0] = scale;
    }

    if (axis & SCALE_AXIS_Y) {
        o->header.gfx.scale[1] = scale;
    }

    if (axis & SCALE_AXIS_Z) {
        o->header.gfx.scale[2] = scale;
    }
}

void cur_obj_set_pos_to_home_with_debug(void) {
    vec3_sum(&o->oPosVec, &o->oHomeVec, gDebugInfo[DEBUG_PAGE_ENEMYINFO]);

    cur_obj_scale(gDebugInfo[DEBUG_PAGE_ENEMYINFO][3] / 100.0f + 1.0f);
}

s32 cur_obj_is_mario_on_platform(void) {
    return gMarioObject->platform == o;
}

s32 cur_obj_shake_y_until(s32 cycles, s32 amount) {
    if (o->oTimer & 0x1) {
        obj_add_up(o, &o->oPosVec, -amount);
        // Return FALSE since o->oTimer can't be equal to (cycles * 2) if it is odd.
        return FALSE;
    } else {
        obj_add_up(o, &o->oPosVec, amount);
    }

    return o->oTimer == cycles * 2;
}

void cur_obj_call_action_function(ObjActionFunc actionFunctions[]) {
    ObjActionFunc actionFunction = actionFunctions[o->oAction];
    actionFunction();
}

s32 cur_obj_mario_far_away(void) {
    Vec3f d;
    vec3f_diff(d, &o->oHomeVec, &gMarioObject->oPosVec);

    return o->oDistanceToMario > 2000.0f && vec3_sumsq(d) > sqr(2000.0f);
}

s32 is_mario_moving_fast_or_in_air(s32 speedThreshold) {
    return (
        gMarioState->forwardVel > speedThreshold
        || (gMarioState->action & ACT_FLAG_AIR)
    );
}

s32 is_item_in_array(s8 item, s8 *array) {
    while (*array != -1) {
        if (*array == item) {
            return TRUE;
        }

        array++;
    }

    return FALSE;
}

void bhv_init_room(void) {
    o->oRoom = get_room_at_pos(o->oPosX, o->oPosY, o->oPosZ);
}

s32 cur_obj_is_mario_in_room(void) {
    if (o->oRoom != -1 && gMarioCurrentRoom != 0) {
        if (gMarioCurrentRoom == o->oRoom // Object is in Mario's room.
            || gDoorAdjacentRooms[gMarioCurrentRoom].forwardRoom  == o->oRoom // Object is in the transition room's forward  room.
            || gDoorAdjacentRooms[gMarioCurrentRoom].backwardRoom == o->oRoom // Object is in the transition room's backward room.
        ) {
            return MARIO_INSIDE_ROOM;
        }

        return MARIO_OUTSIDE_ROOM;
    }

    return MARIO_ROOM_UNDEFINED;
}

void cur_obj_enable_rendering_in_room(void) {
    cur_obj_enable_rendering();
    o->activeFlags &= ~ACTIVE_FLAG_IN_DIFFERENT_ROOM;
    gNumRoomedObjectsInMarioRoom++;
}

void cur_obj_disable_rendering_in_room(void) {
    cur_obj_disable_rendering();
    o->activeFlags |= ACTIVE_FLAG_IN_DIFFERENT_ROOM;
    gNumRoomedObjectsNotInMarioRoom++;
}

s32 cur_obj_set_hitbox_and_die_if_attacked(struct ObjectHitbox *hitbox, s32 deathSound, s32 noLootCoins) {
    s32 interacted = FALSE;

    obj_set_hitbox(o, hitbox);

    if (noLootCoins) {
        o->oNumLootCoins = 0;
    }

    if (o->oInteractStatus & INT_STATUS_INTERACTED) {
        if (o->oInteractStatus & INT_STATUS_WAS_ATTACKED) {
            spawn_mist_particles();
            obj_spawn_loot_yellow_coins(o, o->oNumLootCoins, 20.0f);
            obj_mark_for_deletion(o);
            create_sound_spawner(deathSound);
        } else {
            interacted = TRUE;
        }
    }

    o->oInteractStatus = INT_STATUS_NONE;
    return interacted;
}


void obj_explode_and_spawn_coins(f32 mistSize, s32 coinType) {
    spawn_mist_particles_variable(0, 0, mistSize);
    spawn_triangle_break_particles(30, MODEL_DIRT_ANIMATION, 3.0f, TINY_DIRT_PARTICLE_ANIM_STATE_YELLOW);
    obj_mark_for_deletion(o);

    if (coinType == COIN_TYPE_YELLOW) {
        obj_spawn_loot_yellow_coins(o, o->oNumLootCoins, 20.0f);
    } else if (coinType == COIN_TYPE_BLUE) {
        obj_spawn_loot_blue_coins(o, o->oNumLootCoins, 20.0f, 150);
    }
}

void obj_set_collision_data(struct Object *obj, const void *segAddr) {
    obj->collisionData = segmented_to_virtual(segAddr);
}

s32 cur_obj_hide_if_mario_far_away_y(f32 distY) {
    if (absf(obj_height_difference(o, &gMarioObject->oPosVec, &o->oPosVec)) < distY) {
        cur_obj_unhide();
        return FALSE;
    } else {
        cur_obj_hide();
        return TRUE;
    }
}

Gfx *geo_offset_klepto_held_object(s32 callContext, struct GraphNode *node, UNUSED Mat4 mtx) {
    if (callContext == GEO_CONTEXT_RENDER) {
        vec3s_set(((struct GraphNodeTranslationRotation *) node->next)->translation, 300, 300, 0);
    }

    return NULL;
}

s32 obj_is_hidden(struct Object *obj) {
    if (obj->header.gfx.node.flags & GRAPH_RENDER_INVISIBLE) {
        return TRUE;
    } else {
        return FALSE;
    }
}

void enable_time_stop(void) {
    gTimeStopState |= TIME_STOP_ENABLED;
}

void disable_time_stop(void) {
    gTimeStopState &= ~TIME_STOP_ENABLED;
}

void set_time_stop_flags(s32 flags) {
    gTimeStopState |= flags;
}

void clear_time_stop_flags(s32 flags) {
    gTimeStopState = gTimeStopState & (flags ^ 0xFFFFFFFF);
}

s32 cur_obj_can_mario_activate_textbox(f32 radius, f32 height, UNUSED s32 unused) {
    f32 up = obj_height_difference(o, &gMarioObject->oPosVec, &o->oPosVec);
    return (
        o->oDistanceToMario < 1500.0f && up > -160.0f && up < height
        && !(gMarioStates[0].action & ACT_FLAG_AIR)
        && lateral_dist_between_objects(o, gMarioObject) < radius
        && mario_ready_to_speak()
    );
}

s32 cur_obj_can_mario_activate_textbox_2(f32 radius, f32 height) {
    // The last argument here is unused. When this function is called directly the argument is always set to 0x7FFF.
    return cur_obj_can_mario_activate_textbox(radius, height, 0x1000);
}

static void cur_obj_end_dialog(s32 dialogFlags, s32 dialogResult) {
    o->oDialogResponse = dialogResult;
    o->oDialogState++;

    if (!(dialogFlags & DIALOG_FLAG_TIME_STOP_ENABLED)) {
        set_mario_npc_dialog(MARIO_DIALOG_STOP);
    }
}

s32 cur_obj_update_dialog(s32 actionArg, s32 dialogFlags, s32 dialogID, UNUSED s32 unused) {
    s32 dialogResponse = DIALOG_RESPONSE_NONE;

    switch (o->oDialogState) {
#ifdef BUGFIX_DIALOG_TIME_STOP
        case DIALOG_STATUS_ENABLE_TIME_STOP:
            // Patched :(
            // Wait for Mario to be ready to speak, and then enable time stop
            if (mario_ready_to_speak() || gMarioState->action == ACT_READING_NPC_DIALOG) {
                gTimeStopState |= TIME_STOP_ENABLED;
                o->activeFlags |= ACTIVE_FLAG_INITIATED_TIME_STOP;
                o->oDialogState++;
            } else {
                break;
            }
            // Fall through so that Mario's action is interrupted immediately
            // after time is stopped
            FALL_THROUGH;
#else
        case DIALOG_STATUS_ENABLE_TIME_STOP:
            //! We enable time stop even if Mario is not ready to speak. This
            //  allows us to move during time stop as long as Mario never enters
            //  an action that can be interrupted with text.
            if (gMarioState->health >= 0x100) {
                gTimeStopState |= TIME_STOP_ENABLED;
                o->activeFlags |= ACTIVE_FLAG_INITIATED_TIME_STOP;
                o->oDialogState++;
            }
            break;
#endif
        case DIALOG_STATUS_INTERRUPT:
            // Interrupt until Mario is actually speaking with the NPC
            if (set_mario_npc_dialog(actionArg) == MARIO_DIALOG_STATUS_SPEAK) {
                o->oDialogState++;
            }
            break;

        case DIALOG_STATUS_START_DIALOG:
            // Starts dialog, depending of the flag defined, it calls
            // a default dialog or a dialog with response.
            if (dialogFlags & DIALOG_FLAG_TEXT_RESPONSE) {
                create_dialog_box_with_response(dialogID);
            } else if (dialogFlags & DIALOG_FLAG_TEXT_DEFAULT) {
                create_dialog_box(dialogID);
            }
            o->oDialogState++;
            break;

        case DIALOG_STATUS_STOP_DIALOG:
            // Stops dialog, if the flag dialog response was called
            // then it defines the value to let the object do the rest.
            if (dialogFlags & DIALOG_FLAG_TEXT_RESPONSE) {
                if (gDialogResponse != DIALOG_RESPONSE_NONE) {
                    cur_obj_end_dialog(dialogFlags, gDialogResponse);
                }
            } else if (dialogFlags & DIALOG_FLAG_TEXT_DEFAULT) {
                if (get_dialog_id() == DIALOG_NONE) {
                    cur_obj_end_dialog(dialogFlags, DIALOG_RESPONSE_NOT_DEFINED);
                }
            } else {
                cur_obj_end_dialog(dialogFlags, DIALOG_RESPONSE_NOT_DEFINED);
            }
            break;

        case DIALOG_STATUS_DISABLE_TIME_STOP:
            // We disable time stop for a few seconds when Mario is no longer
            // speaking or the flag is defined, then we enable it again.
            // Usually, an object disables time stop using a separate function
            // after a certain condition is met.
            if (gMarioState->action != ACT_READING_NPC_DIALOG || (dialogFlags & DIALOG_FLAG_TIME_STOP_ENABLED)) {
                gTimeStopState &= ~TIME_STOP_ENABLED;
                o->activeFlags &= ~ACTIVE_FLAG_INITIATED_TIME_STOP;
                dialogResponse = o->oDialogResponse;
                o->oDialogState = DIALOG_STATUS_ENABLE_TIME_STOP;
            }
            break;

        default:
            o->oDialogState = DIALOG_STATUS_ENABLE_TIME_STOP;
            break;
    }

    return dialogResponse;
}

s32 cur_obj_update_dialog_with_cutscene(s32 actionArg, s32 dialogFlags, s32 cutsceneTable, s32 dialogID) {
    s32 dialogResponse = DIALOG_RESPONSE_NONE;
    s32 doneTurning = TRUE;

    switch (o->oDialogState) {
#ifdef BUGFIX_DIALOG_TIME_STOP
        case DIALOG_STATUS_ENABLE_TIME_STOP:
            // Wait for Mario to be ready to speak, and then enable time stop
            if (mario_ready_to_speak() || gMarioState->action == ACT_READING_NPC_DIALOG) {
                gTimeStopState |= TIME_STOP_ENABLED;
                o->activeFlags |= ACTIVE_FLAG_INITIATED_TIME_STOP;
                o->oDialogState++;
                o->oDialogResponse = DIALOG_RESPONSE_NONE;
            } else {
                break;
            }
            // Fall through so that Mario's action is interrupted immediately
            // after time is stopped
            FALL_THROUGH;
#else
        case DIALOG_STATUS_ENABLE_TIME_STOP:
            //! We enable time stop even if Mario is not ready to speak. This
            //  allows us to move during time stop as long as Mario never enters
            //  an action that can be interrupted with text.
            if (gMarioState->health >= 0x0100) {
                gTimeStopState |= TIME_STOP_ENABLED;
                o->activeFlags |= ACTIVE_FLAG_INITIATED_TIME_STOP;
                o->oDialogState++;
                o->oDialogResponse = DIALOG_RESPONSE_NONE;
            }
            break;
#endif
        case DIALOG_STATUS_INTERRUPT:
            // Additional flag that makes the NPC rotate towards to Mario
            if (dialogFlags & DIALOG_FLAG_TURN_TO_MARIO) {
                doneTurning = cur_obj_rotate_yaw_toward(obj_angle_to_object(o, gMarioObject), 0x800);
                // Failsafe just in case it takes more than 33 frames somehow
                if (o->oDialogResponse > DIALOG_RESPONSE_MAXIMUM) {
                    doneTurning = TRUE;
                }
            }
            // Interrupt status until Mario is actually speaking with the NPC and if the
            // object is done turning to Mario
            if (set_mario_npc_dialog(actionArg) == MARIO_DIALOG_STATUS_SPEAK && doneTurning) {
                o->oDialogResponse = DIALOG_RESPONSE_NONE;
                o->oDialogState++;
            } else {
                o->oDialogResponse++; // treated as a timer for the failsafe
            }
            break;

        case DIALOG_STATUS_START_DIALOG:
            // Special check for Cap Switch cutscene since the cutscene itself
            // handles what dialog should use
            if (cutsceneTable == CUTSCENE_CAP_SWITCH_PRESS) {
                if ((o->oDialogResponse = cutscene_object_without_dialog(cutsceneTable, o))) {
                    o->oDialogState++;
                }
            } else {
                // General dialog cutscene function, most of the time
                // the "CUTSCENE_DIALOG" cutscene is called
                if ((o->oDialogResponse = cutscene_object_with_dialog(cutsceneTable, o, dialogID))) {
                    o->oDialogState++;
                }
            }
            break;

        case DIALOG_STATUS_STOP_DIALOG:
            // If flag defined, keep time stop enabled until the object
            // decided to disable it independently
            if (dialogFlags & DIALOG_FLAG_TIME_STOP_ENABLED) {
                dialogResponse = o->oDialogResponse;
                o->oDialogState = DIALOG_STATUS_ENABLE_TIME_STOP;
            } else if (gMarioState->action != ACT_READING_NPC_DIALOG) {
                // Disable time stop, then enable time stop for a frame
                // until the set_mario_npc_dialog function disables it
                gTimeStopState &= ~TIME_STOP_ENABLED;
                o->activeFlags &= ~ACTIVE_FLAG_INITIATED_TIME_STOP;
                dialogResponse = o->oDialogResponse;
                o->oDialogState = DIALOG_STATUS_ENABLE_TIME_STOP;
            } else {
                // And finally stop Mario dialog status
                set_mario_npc_dialog(MARIO_DIALOG_STOP);
            }
            break;
    }

    return dialogResponse;
}

void cur_obj_align_gfx_with_floor(void) {
    if (o->oFloor != NULL) {
        Vec3f normal;
        surface_normal_to_vec3f(normal, o->oFloor);
        obj_align_to_normal(o, normal, o->oFaceAngleYaw);
    }
}

UNUSED s32 mario_is_within_rectangle(s16 minX, s16 maxX, s16 minZ, s16 maxZ) {
    if (gMarioObject->oPosX < minX || maxX < gMarioObject->oPosX) {
        return FALSE;
    }

    if (gMarioObject->oPosZ < minZ || maxZ < gMarioObject->oPosZ) {
        return FALSE;
    }

    return TRUE;
}

void cur_obj_shake_screen(s32 shake) {
    set_camera_shake_from_point(shake, o->oPosX, o->oPosY, o->oPosZ);
}

s32 obj_attack_collided_from_other_object(struct Object *obj) {
    if (obj->numCollidedObjs != 0) {
        struct Object *other = obj->collidedObjs[0];

        if (other != gMarioObject) {
            other->oInteractStatus |= INT_STATUS_TOUCHED_MARIO | INT_STATUS_WAS_ATTACKED | INT_STATUS_INTERACTED
                                      | INT_STATUS_TOUCHED_BOB_OMB;
            return TRUE;
        }
    }

    return FALSE;
}

s32 cur_obj_was_attacked_or_ground_pounded(void) {
    s32 attacked = FALSE;

    if ((o->oInteractStatus & INT_STATUS_INTERACTED)
        && (o->oInteractStatus & INT_STATUS_WAS_ATTACKED)) {
        attacked = TRUE;
    }

    if (cur_obj_is_mario_ground_pounding_platform()) {
        attacked = TRUE;
    }

    o->oInteractStatus = INT_STATUS_NONE;
    return attacked;
}

void obj_copy_behavior_params(struct Object *dst, struct Object *src) {
    dst->oBehParams = src->oBehParams;
    dst->oBehParams2ndByte = src->oBehParams2ndByte;
}

void cur_obj_init_animation_and_anim_frame(s32 animIndex, s32 animFrame) {
    cur_obj_init_animation_with_sound(animIndex);
    o->header.gfx.animInfo.animFrame = animFrame;
}

s32 cur_obj_init_animation_and_check_if_near_end(s32 animIndex) {
    cur_obj_init_animation_with_sound(animIndex);
    return cur_obj_check_if_near_animation_end();
}

void cur_obj_init_animation_and_extend_if_at_end(s32 animIndex) {
    cur_obj_init_animation_with_sound(animIndex);
    cur_obj_extend_animation_if_at_end();
}

s32 cur_obj_check_grabbed_mario(void) {
    if (o->oInteractStatus & INT_STATUS_GRABBED_MARIO) {
        o->oKingBobombHoldingMarioState = HELD_HELD;
        cur_obj_become_intangible();
        return TRUE;
    }

    return FALSE;
}

s32 sPlayerGrabReleaseState;

s32 player_performed_grab_escape_action(void) {
    if (gPlayer1Controller->stickMag < 30.0f) {
        sPlayerGrabReleaseState = FALSE;
    }

    if (!sPlayerGrabReleaseState && (gPlayer1Controller->stickMag > 40.0f)) {
        sPlayerGrabReleaseState = TRUE;
        return TRUE;
    }

    if (gPlayer1Controller->buttonPressed & (A_BUTTON | B_BUTTON | Z_TRIG)) {
        return TRUE;
    }

    return FALSE;
}

void cur_obj_unused_play_footstep_sound(s32 animFrame1, s32 animFrame2, s32 sound) {
    if (cur_obj_check_anim_frame(animFrame1) || cur_obj_check_anim_frame(animFrame2)) {
        cur_obj_play_sound_2(sound);
    }
}

void enable_time_stop_including_mario(void) {
    gTimeStopState |= TIME_STOP_ENABLED | TIME_STOP_MARIO_AND_DOORS;
    o->activeFlags |= ACTIVE_FLAG_INITIATED_TIME_STOP;
}

void disable_time_stop_including_mario(void) {
    gTimeStopState &= ~(TIME_STOP_ENABLED | TIME_STOP_MARIO_AND_DOORS);
    o->activeFlags &= ~ACTIVE_FLAG_INITIATED_TIME_STOP;
}

s32 cur_obj_check_interacted(void) {
    if (o->oInteractStatus & INT_STATUS_INTERACTED) {
        o->oInteractStatus = INT_STATUS_NONE;
        return TRUE;
    } else {
        return FALSE;
    }
}

void cur_obj_spawn_loot_blue_coin(void) {
    if (o->oNumLootCoins >= 5) {
        spawn_object(o, MODEL_BLUE_COIN, bhvMrIBlueCoin);
        o->oNumLootCoins -= 5;
    }
}

void cur_obj_spawn_star_at_y_offset(f32 targetX, f32 targetY, f32 targetZ, f32 offsetY) {
    Vec3f pos;
    vec3_copy(pos, &o->oPosVec);
    obj_add_up(o, &o->oPosVec, offsetY + gDebugInfo[DEBUG_PAGE_ENEMYINFO][0]);
    spawn_default_star(targetX, targetY, targetZ);
    vec3_copy(&o->oPosVec, pos);
}
