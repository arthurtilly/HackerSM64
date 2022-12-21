// metal_box.inc.c

#include "game/rigid_body.h"

struct ObjectHitbox sMetalBoxHitbox = {
    /* interactType:      */ INTERACT_NONE,
    /* downOffset:        */ 150,
    /* damageOrCoinValue: */ 0,
    /* health:            */ 1,
    /* numLootCoins:      */ 0,
    /* radius:            */ 250,
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

Vec3f sPushableBoxSize = { 154.f, 154.f, 154.f };

f32 find_floor_height(f32 x, f32 y, f32 z);

void bhv_pushable_init(void) {
    o->oPosY += 154.f;
    struct RigidBody *body = allocate_rigid_body(&Cube_Mesh, 50000.0f, sPushableBoxSize, &o->oPosVec, &gCurrentObject->transform);
    body->obj = o;
    f32 floorHeight = find_floor_height(o->oPosX, o->oPosY, o->oPosZ);
    if (o->oPosY - floorHeight < 5.f) body->asleep = TRUE;
    o->rigidBody = body;
}

void bhv_pushable_loop(void) {
    obj_set_hitbox(o, &sMetalBoxHitbox);

    s16 animID = gMarioObject->header.gfx.animInfo.animID;

    if (obj_check_if_collided_with_object(o, gMarioObject) && 
        (animID == MARIO_ANIM_SIDESTEP_RIGHT || animID == MARIO_ANIM_SIDESTEP_LEFT || animID == MARIO_ANIM_PUSHING)) {
        s16 angleToMario = obj_angle_to_object(o, gMarioObject);
        if (abs_angle_diff(angleToMario, gMarioObject->oMoveAngleYaw) > 0x7000) {
            Vec3f force;
            force[0] = sins(gMarioObject->oMoveAngleYaw) * 5.f;
            force[1] = 0.0f;
            force[2] = coss(gMarioObject->oMoveAngleYaw) * 5.f;
            rigid_body_add_force(o->rigidBody, &gMarioObject->oPosVec, force);
        }
    }
}
