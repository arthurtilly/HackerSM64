// metal_box.inc.c

#include "game/rigid_body.h"

struct ObjectHitbox sMetalBoxHitbox = {
    /* interactType:      */ INTERACT_NONE,
    /* downOffset:        */ 0,
    /* damageOrCoinValue: */ 0,
    /* health:            */ 1,
    /* numLootCoins:      */ 0,
    /* radius:            */ 220,
    /* height:            */ 300,
    /* hurtboxRadius:     */ 220,
    /* hurtboxHeight:     */ 300,
};

s32 check_if_moving_over_floor(f32 maxDist, f32 offset) {
    struct Surface *floor;
    f32 xPos = o->oPosX + sins(o->oMoveAngleYaw) * offset;
    f32 zPos = o->oPosZ + coss(o->oMoveAngleYaw) * offset;

    f32 floorHeight = find_floor(xPos, o->oPosY, zPos, &floor);

    return (absf(floorHeight - o->oPosY) < maxDist);
}

Vec3f sPushableBoxSize = { 153.f, 153.f, 153.f };

f32 find_floor_height(f32 x, f32 y, f32 z);

void bhv_pushable_init(void) {
    u32 sleep = FALSE;
    f32 floorHeight = find_floor_height(o->oPosX, o->oPosY, o->oPosZ);
    if (ABS(o->oPosY - floorHeight) < 5.f) sleep = TRUE;
    struct RigidBody *body = allocate_rigid_body_from_object(o, &Cube_Mesh, 6.f, sPushableBoxSize, 0.f, -153.f, 0.f);
    body->asleep = sleep;
}

void bhv_pushable_loop(void) {
    obj_set_hitbox(o, &sMetalBoxHitbox);
    rigid_body_general_object_loop();
}

Vec3f sSignpostSize = { 45.f, 63.f, 16.f };

void bhv_message_panel_init(void) {
    struct RigidBody *body = allocate_rigid_body_from_object(o, &Cube_Mesh, 1.f, sSignpostSize, 0.f, -63.f, 0.f);
    body->asleep = TRUE;
}

void bhv_message_panel_loop(void) {
    rigid_body_general_object_loop();
}
