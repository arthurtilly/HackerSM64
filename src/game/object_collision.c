#include <PR/ultratypes.h>

#include "sm64.h"
#include "debug.h"
#include "interaction.h"
#include "level_update.h"
#include "object_list_processor.h"
#include "object_collision.h"
#include "object_helpers.h"
#include "spawn_object.h"
#include "engine/math_util.h"

UNUSED struct Object *debug_print_obj_collision(struct Object *a) {
    struct Object *currCollidedObj;
    s32 i;

    for (i = 0; i < a->numCollidedObjs; i++) {
        print_debug_top_down_objectinfo("ON", 0);
        currCollidedObj = a->collidedObjs[i];
        if (currCollidedObj != gMarioObject) {
            return currCollidedObj;
        }
    }
    return NULL;
}

void obj_set_interaction_volume(struct Object *obj, f32 radius, f32 bottom, f32 top) {
    obj->hitboxRadius = radius;
    obj->hitboxOffset = (top + bottom) * 0.5f;
    obj->interactionHalfHeight = (top - bottom) * 0.5f;
}

void obj_sphere_center(struct Object *obj, const Vec3f pos, f32 offset, Vec3f center) {
    Vec3f up;
    obj_hitbox_up(obj, up);
    for (s32 i = 0; i < 3; i++) center[i] = pos[i] + offset * up[i];
}

f32 obj_local_height(struct Object *obj, const Vec3f pos) {
    Vec3f up, delta;
    obj_hitbox_up(obj, up);
    vec3_diff(delta, pos, &obj->oPosVec);
    return vec3_dot(delta, up);
}

// Separate grounded bodies in the actor's tangent plane, using the sphere cross-section at this height.
static s32 sphere_push(struct Object *actor, const Vec3f pos, f32 offset, struct Object *other,
                       f32 otherOffset, f32 radius, Vec3f push) {
    Vec3f a, b, delta;
    obj_sphere_center(actor, pos, offset, a);
    obj_sphere_center(other, &other->oPosVec, otherOffset, b);
    vec3_diff(delta, a, b);
    f32 height = vec3_dot(delta, actor->gravityDir);
    f32 rangeSq = sqr(radius) - sqr(height);
    if (rangeSq <= 0.0f || radius <= 0.0f) return FALSE;
    for (s32 i = 0; i < 3; i++) push[i] = delta[i] - height * actor->gravityDir[i];
    f32 lateralSq = vec3_sumsq(push);
    if (lateralSq >= rangeSq) return FALSE;
    if (lateralSq > NEAR_ZERO) {
        vec3_scale(push, sqrtf(rangeSq / lateralSq) - 1.0f);
    } else {
        obj_yaw_direction(actor, push, actor->oFaceAngleYaw);
        vec3_scale(push, sqrtf(rangeSq));
    }
    return TRUE;
}

s32 obj_sphere_push(struct Object *actor, const Vec3f pos, struct Object *other, f32 padding, s32 useHurtbox, Vec3f push) {
    s32 hurtA = useHurtbox && actor->hurtboxRadius > 0.0f;
    s32 hurtB = useHurtbox && other->hurtboxRadius > 0.0f;
    f32 radius = (hurtA ? actor->hurtboxRadius : actor->hitboxRadius)
               + (hurtB ? other->hurtboxRadius : other->hitboxRadius) + padding;
    return sphere_push(actor, pos, hurtA ? actor->hurtboxOffset : actor->hitboxOffset, other,
                       hurtB ? other->hurtboxOffset : other->hitboxOffset, radius, push);
}

void cur_obj_push_mario_away_from_sphere(f32 radius, f32 offset) {
    Vec3f push;
    struct Object *mario = gMarioObject;
    vec3_diff(push, gMarioState->pos, &o->oPosVec);
    radius += mario->hitboxRadius;
    if (vec3_sumsq(push) < sqr(radius + mario->hitboxOffset + absf(offset))
        && sphere_push(mario, gMarioState->pos, mario->hitboxOffset, o, offset, radius, push)) {
        vec3_scale(push, 0.5f);
        vec3_add(gMarioState->pos, push);
    }
}

// A pole, gate or warp has a finite axial range independent of its lateral reach.
static s32 sphere_touches_volume(const Vec3f delta, const Vec3f up, f32 radius, const struct Object *volume) {
    f32 height = vec3_dot(delta, up);
    f32 beyond = MAX(absf(height) - volume->interactionHalfHeight, 0.0f);
    if (beyond >= radius) return FALSE;
    f32 reach = volume->hitboxRadius + (beyond == 0.0f ? radius : sqrtf(sqr(radius) - sqr(beyond)));
    return vec3_sumsq(delta) - sqr(height) < sqr(reach);
}

static void check_collision_in_list(struct Object *a, const Vec3f center, const Vec3f hurtCenter,
                                    struct Object *b, struct Object *end) {
    for (; b != end; b = (struct Object *) b->header.next) {
        if (a->numCollidedObjs == ARRAY_COUNT(a->collidedObjs)) break;
        if (b->oIntangibleTimer != 0 || b->hitboxRadius <= 0.0f
            || b->numCollidedObjs == ARRAY_COUNT(b->collidedObjs)) continue;

        Vec3f delta, up;
        f32 radius = a->hitboxRadius + b->hitboxRadius;
        vec3_diff(delta, center, &b->oPosVec);
        // Reject distant pairs before looking up the target's orientation.
        if (vec3_sumsq(delta) >= sqr(radius + absf(b->hitboxOffset) + b->interactionHalfHeight)) continue;
        obj_hitbox_up(b, up);
        for (s32 i = 0; i < 3; i++) delta[i] -= up[i] * b->hitboxOffset;
        if (b->interactionHalfHeight > 0.0f) {
            if (!sphere_touches_volume(delta, up, a->hitboxRadius, b)) continue;
        } else if (vec3_sumsq(delta) >= sqr(radius)) {
            continue;
        }

        a->collidedObjs[a->numCollidedObjs++] = b;
        b->collidedObjs[b->numCollidedObjs++] = a;
        a->collidedObjInteractTypes |= b->oInteractType;
        b->collidedObjInteractTypes |= a->oInteractType;

        if (a == gMarioObject && b->hurtboxRadius > 0.0f) {
            for (s32 i = 0; i < 3; i++) {
                delta[i] = hurtCenter[i] - (&b->oPosVec)[i] - up[i] * b->hurtboxOffset;
            }
            COND_BIT(vec3_sumsq(delta) >= sqr(a->hurtboxRadius + b->hurtboxRadius),
                     b->oInteractionSubtype, INT_SUBTYPE_DELAY_INVINCIBILITY);
        }
    }
}

static void clear_object_collision(struct Object *list) {
    for (struct Object *obj = (struct Object *) list->header.next; obj != list; obj = (struct Object *) obj->header.next) {
        obj->numCollidedObjs = 0;
        obj->collidedObjInteractTypes = 0;
        if (obj->oIntangibleTimer > 0) obj->oIntangibleTimer--;
    }
}

static void check_object_list(s32 listIndex, const u8 *otherLists, s32 count) {
    struct Object *list = (struct Object *) &gObjectLists[listIndex];
    for (struct Object *a = (struct Object *) list->header.next; a != list; a = (struct Object *) a->header.next) {
        if (a->oIntangibleTimer != 0 || a->hitboxRadius <= 0.0f) continue;
        if (listIndex == OBJ_LIST_DESTRUCTIVE && (a->oDistanceToMario >= 2000.0f
            || (a->activeFlags & ACTIVE_FLAG_DESTRUCTIVE_OBJ_DONT_DESTROY))) continue;
        Vec3f center, hurtCenter;
        obj_sphere_center(a, &a->oPosVec, a->hitboxOffset, center);
        if (a == gMarioObject) obj_sphere_center(a, &a->oPosVec, a->hurtboxOffset, hurtCenter);
        check_collision_in_list(a, center, hurtCenter, (struct Object *) a->header.next, list);
        for (s32 i = 0; i < count; i++) {
            struct Object *other = (struct Object *) &gObjectLists[otherLists[i]];
            check_collision_in_list(a, center, hurtCenter, (struct Object *) other->header.next, other);
        }
    }
}

void detect_object_collisions(void) {
    clear_object_collision((struct Object *) &gObjectLists[OBJ_LIST_POLELIKE]);
    clear_object_collision((struct Object *) &gObjectLists[OBJ_LIST_PLAYER]);
    clear_object_collision((struct Object *) &gObjectLists[OBJ_LIST_PUSHABLE]);
    clear_object_collision((struct Object *) &gObjectLists[OBJ_LIST_GENACTOR]);
    clear_object_collision((struct Object *) &gObjectLists[OBJ_LIST_LEVEL]);
    clear_object_collision((struct Object *) &gObjectLists[OBJ_LIST_SURFACE]);
    clear_object_collision((struct Object *) &gObjectLists[OBJ_LIST_DESTRUCTIVE]);
    static const u8 playerLists[] = { OBJ_LIST_POLELIKE, OBJ_LIST_LEVEL, OBJ_LIST_GENACTOR,
                                      OBJ_LIST_PUSHABLE, OBJ_LIST_SURFACE, OBJ_LIST_DESTRUCTIVE };
    static const u8 destructiveLists[] = { OBJ_LIST_GENACTOR, OBJ_LIST_PUSHABLE, OBJ_LIST_SURFACE };
    check_object_list(OBJ_LIST_PLAYER, playerLists, ARRAY_COUNT(playerLists));
    check_object_list(OBJ_LIST_DESTRUCTIVE, destructiveLists, ARRAY_COUNT(destructiveLists));
    check_object_list(OBJ_LIST_PUSHABLE, NULL, 0);
}
