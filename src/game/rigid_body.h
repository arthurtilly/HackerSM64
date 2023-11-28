#pragma once

#include <PR/ultratypes.h>
#include "macros.h"
#include "engine/math_util.h"

#include "rigid_body_collision.h"

#define DAMPING 0.999f

#define SLEEP_DETECTION_BIAS 0.9f
#define SLEEP_DETECTION_THRESHOLD 5.f

#define PENETRATION_MIN_DEPTH 0.f
#define PENETRATION_MAX_DEPTH 40.f

#define GRAVITY_FORCE -3.f
#define FRICTION 0.5f

#define NUM_RIGID_BODY_STEPS 2

#define MAX_RIGID_BODIES 40

typedef Vec4f Quat;

#ifdef PUPPYPRINT_DEBUG
extern u32 pNumTrisChecked;
extern u32 pNumCols;
extern u32 pNumColsTrunc;
extern u32 pNumVertexChecks;
extern u32 pNumEdgeChecks;
extern u32 pNumFaceChecks;
extern u32 pNumImpulsesApplied;

extern u32 pNumRigidBodies;
extern u32 pNumActiveRigidBodies;

void increment_debug_counter(u32 *counter, s32 amount);
#else

#define increment_debug_counter(_, _)

#endif

/// Add a vector scaled by a constant to another vector.
#define vec3f_add_scaled(dest, src, scale) { \
    dest[0] += src[0] * scale; \
    dest[1] += src[1] * scale; \
    dest[2] += src[2] * scale; \
}

/// Set a vector to another vector scaled by a constant.
#define vec3f_scale(dest, src, scale) { \
    dest[0] = src[0] * scale; \
    dest[1] = src[1] * scale; \
    dest[2] = src[2] * scale; \
}


// Controls an instance of a rigid body
struct RigidBody {
    u8 allocated:1; // Mark if the struct has been allocated
    u8 isStatic:1; // Can the rigid body move
    u8 asleep:1; // Body goes to sleep if inactive for a while, and collision is not calculated
    u8 hasGravity:1; // Apply gravity to the body
    u8 floating:1; // Body is floating on water

    f32 mass;
    f32 invMass; // 1/m (for performance)
    f32 motion; // Average motion over the past few frames, used to determine if the body should sleep
    Vec3f size;
    Vec3f centerOfMass; // Position
    Vec3f minCorner; // Lowest coords of AABB
    Vec3f maxCorner; // Highest coords of AABB
    Quat angleQuat; // Orientation

    Vec3f linearVel; // Linear velocity
    Vec3f angularVel; // Angular velocity
    Vec3f linearDisplacement;
    f32 displacementMagnitude;
    Vec3f netForce; // Total force applied
    Vec3f netTorque; // Total torque applied

    Mat4 transform; // The transformation matrix of the body
    Mat4 invInertia; // Inverse inertia tensor

    struct MeshInfo *mesh; // Pointer to the mesh info of the body
    struct Object *obj; // Pointer to the object this rigid body is linked to
};

extern struct RigidBody gRigidBodies[MAX_RIGID_BODIES];

struct RigidBody *allocate_rigid_body_from_object(struct Object *obj, struct MeshInfo *mesh, f32 mass, Vec3f size, f32 xOffset, f32 yOffset, f32 zOffset);
struct RigidBody *allocate_rigid_body(struct MeshInfo *mesh, f32 mass, Vec3f size, Vec3f pos);
void deallocate_rigid_body(struct RigidBody *body);
void rigid_body_set_yaw(struct RigidBody *body, s16 yaw);
u32 rigid_bodies_near(struct RigidBody *body1, struct RigidBody *body2);
void check_hit_rigid_body_floor_or_ceiling(struct Surface *surf, Vec3f pos, f32 yvel);
void check_hit_rigid_body_wall(struct Surface *wall, Vec3f pos, f32 fvel, f32 yvel, s16 yaw);
void rigid_body_add_force(struct RigidBody *body, Vec3f contactPoint, Vec3f force, u32 wake);

void do_rigid_body_step(void);
void rigid_body_general_object_loop(void);