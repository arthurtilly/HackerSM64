#ifndef OBJECT_COLLISION_H
#define OBJECT_COLLISION_H

#include "types.h"

void obj_hitbox_up(struct Object *obj, Vec3f up);
void obj_sphere_center(struct Object *obj, const Vec3f pos, f32 offset, Vec3f center);
f32 obj_local_height(struct Object *obj, const Vec3f pos);
void obj_set_interaction_volume(struct Object *obj, f32 radius, f32 bottom, f32 top);
// Limits along local up for climbable poles and other bounded triggers.
static inline f32 obj_trigger_bottom(const struct Object *obj) { return obj->hitboxOffset - obj->interactionHalfHeight; }
static inline f32 obj_trigger_top(const struct Object *obj) { return obj->hitboxOffset + obj->interactionHalfHeight; }
s32 obj_sphere_push(struct Object *actor, const Vec3f pos, struct Object *other, f32 padding, s32 useHurtbox, Vec3f push);
void cur_obj_push_mario_away_from_sphere(f32 radius, f32 offset);
void detect_object_collisions(void);

#endif // OBJECT_COLLISION_H
