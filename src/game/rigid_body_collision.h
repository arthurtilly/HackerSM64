#pragma once

#include <PR/ultratypes.h>
#include "macros.h"
#include "engine/math_util.h"

// A specific contact point between two rigid bodies
struct CollisionPoint {
    Vec3f point;
    Vec3f normal;
    f32 penetration;
};

// An array of all contact points between two specific bodies
struct Collision {
    struct RigidBody *body1;
    struct RigidBody *body2;
    struct CollisionPoint points[20];
    u32 numPoints;
};

extern struct Collision gCollisions[20];
extern u32 gNumCollisions;

void rigid_body_check_collisions(struct RigidBody *body);
