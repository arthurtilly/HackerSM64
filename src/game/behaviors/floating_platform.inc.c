// floating_platform.inc.c

f32 floating_platform_find_home_y(void) {
    struct Surface *floor;
    f32 waterLevel  = find_water_level(o->oPosX, o->oPosZ);
    f32 floorHeight = find_floor(o->oPosX, o->oPosY, o->oPosZ, &floor);

    if (waterLevel > floorHeight + 64.f) {
        o->oFloatingPlatformIsOnFloor = FALSE;
        return (waterLevel + 64.f);
    } else {
        o->oFloatingPlatformIsOnFloor = TRUE;
        return (floorHeight + 64.f);
    }
}

#include "game/rigid_body.h"

Vec3f sFloatingPlatformSizes[2] = {
    { 256.f, 64.f, 256.f }, // Square
    { 256.f, 64.f, 640.f } // Rectangular
};

f32 sFloatingPlatformMasses[2] = {
    8.f,
    15.f
};

void bhv_floating_platform_init(void) {
    o->oPosY = floating_platform_find_home_y();
    struct RigidBody *body = allocate_rigid_body(&Cube_Mesh,
        sFloatingPlatformMasses[o->oBehParams2ndByte],
        sFloatingPlatformSizes[o->oBehParams2ndByte],
        &o->oPosVec, &o->transform);
    rigid_body_set_yaw(body, o->oFaceAngleYaw);
    
    body->obj = o;
    body->asleep = o->oFloatingPlatformIsOnFloor;
    body->floating = !o->oFloatingPlatformIsOnFloor;
    o->rigidBody = body;
}

void bhv_floating_platform_loop(void) {
    rigid_body_general_object_loop();
}
