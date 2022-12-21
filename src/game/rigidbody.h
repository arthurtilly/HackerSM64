#pragma once

#include <PR/ultratypes.h>
#include "macros.h"
#include "engine/math_util.h"

#define DAMPING 0.99f

#define SLEEP_DETECTION_BIAS 0.96f
#define SLEEP_DETECTION_THRESHOLD 3.f

#define PENETRATION_BIAS 0.1f
#define PENETRATION_MAX_DEPTH 1.f

#define GRAVITY_FORCE -3.f
#define FRICTION 0.5f

#define NUM_RIGID_BODY_STEPS 3
#define NUM_IMPULSE_ITERATIONS 1

typedef Vec4f Quat;

enum RigidBodyTypes {
    RIGID_BODY_TYPE_CUBE,
    RIGID_BODY_TYPE_TRIANGLE,
};

// Controls an instance of a rigid body
struct RigidBody {
    u8 allocated; // Mark if the struct has been allocated
    u8 type; // Shape of the rigid body
    u8 isStatic; // Can the rigid body move
    u8 asleep; // Body goes to sleep if inactive for a while, and collision is not calculated

    f32 mass;
    f32 invMass; // 1/m (for performance)
    f32 size;
    f32 motion; // Average motion over the past few frames, used to determine if the body should sleep
    Vec3f centerOfMass; // Position
    Quat angleQuat; // Orientation

    Vec3f linearVel; // Linear velocity
    Vec3f angularVel; // Angular velocity
    Vec3f netForce; // Total force applied
    Vec3f netTorque; // Total torque applied

    Mat4 *transform; // Pointer to the transformation matrix of the body
    Mat4 invInertia; // Inverse inertia tensor

    struct Object *obj; // Pointer to the object this rigid body is linked to
};

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

void do_rigid_body_step(void);
void bhv_rigid_cube_init(void);
