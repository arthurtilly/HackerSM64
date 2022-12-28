#include <PR/ultratypes.h>

#include "config.h"
#include "sm64.h"
#include "rigid_body.h"
#include "object_list_processor.h"
#include "engine/surface_collision.h"

struct RigidBody gRigidBodies[MAX_RIGID_BODIES];

f32 dt = 1.f/NUM_RIGID_BODY_STEPS;

#ifdef PUPPYPRINT_DEBUG
u32 pNumTrisChecked;
u32 pNumCols;
u32 pNumColsTrunc;
u32 pNumVertexChecks;
u32 pNumEdgeChecks;
u32 pNumFaceChecks;
u32 pNumImpulsesApplied;

u32 pNumRigidBodies = 0;
u32 pNumActiveRigidBodies = 0;

void increment_debug_counter(u32 *counter, s32 amount) {
    *counter += amount;
}
#endif

/// Set a vector to another vector scaled by a constant.
void vec3f_scale(Vec3f dest, Vec3f src, f32 scale) {
    dest[0] = src[0] * scale;
    dest[1] = src[1] * scale;
    dest[2] = src[2] * scale;
}

/// Add a vector scaled by a constant to another vector.
void vec3f_add_scaled(Vec3f dest, Vec3f src, f32 scale) {
    dest[0] += src[0] * scale;
    dest[1] += src[1] * scale;
    dest[2] += src[2] * scale;
}

/// Convert a quaternion to a rotation matrix.
void mtxf_from_quat(Quat q, Mat4 dest) {
    dest[0][0] = -2.f * (q[1] * q[1] + q[2] * q[2]) + 1.f;
    dest[0][1] =  2.f * (q[0] * q[1] - q[3] * q[2]);
    dest[0][2] =  2.f * (q[0] * q[2] + q[3] * q[1]);

    dest[1][0] =  2.f * (q[0] * q[1] + q[3] * q[2]);
    dest[1][1] = -2.f * (q[0] * q[0] + q[2] * q[2]) + 1.f;
    dest[1][2] =  2.f * (q[1] * q[2] - q[3] * q[0]);

    dest[2][0] =  2.f * (q[0] * q[2] - q[3] * q[1]);
    dest[2][1] =  2.f * (q[1] * q[2] + q[3] * q[0]);
    dest[2][2] = -2.f * (q[0] * q[0] + q[1] * q[1]) + 1.f;

    dest[3][0] = dest[3][1] = dest[3][2] = dest[0][3] = dest[1][3] = dest[2][3] = 0.f;
    dest[3][3] = 1.f;
}

/// Copy a quaternion.
void quat_copy(Quat dest, Quat src) {
    dest[0] = src[0];
    dest[1] = src[1];
    dest[2] = src[2];
    dest[3] = src[3];
}

/// Multiply two quaternions.
void quat_mul(Quat dest, Quat a, Quat b) {
    dest[0] = a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1];
    dest[1] = a[3] * b[1] + a[1] * b[3] + a[2] * b[0] - a[0] * b[2];
    dest[2] = a[3] * b[2] + a[2] * b[3] + a[0] * b[1] - a[1] * b[0];
    dest[3] = a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2];
}

/// Normalize a quaternion.
void quat_normalize(Quat quat) {
    f32 invMag = 1.f / sqrtf(quat[0] * quat[0] + quat[1] * quat[1] + quat[2] * quat[2] + quat[3] * quat[3]);
    quat[0] *= invMag;
    quat[1] *= invMag;
    quat[2] *= invMag;
    quat[3] *= invMag;
}

/// Resolve a collision impulse between two rigid bodies.
void rigid_body_collision_impulse(struct RigidBody *body1, struct RigidBody *body2, Vec3f hitPoint, Vec3f normal, f32 penetration) {
    u32 doSecondBody = (!(body2->isStatic));
    f32 body1InvMass = body1->invMass;
    f32 body2InvMass = (doSecondBody ? body2->invMass : 0.f);
    if (body1->isStatic) {
        return;
    }

    if (doSecondBody) {
        body1->asleep = FALSE;
        body2->asleep = FALSE;
    }
    increment_debug_counter(&pNumImpulsesApplied, 1);

    // Determine the relative velocity (dv) of the two bodies at the point of impact.
    Vec3f r0, r1, v0, v1, dv;
    vec3f_copy(r0, hitPoint);
    vec3f_sub(r0, body1->centerOfMass);
    vec3f_cross(v0, r0, body1->angularVel);
    vec3f_add(v0, body1->linearVel);
    vec3f_copy(dv, v0);

    if (doSecondBody) {
        vec3f_copy(r1, hitPoint);
        vec3f_sub(r1, body2->centerOfMass);
        vec3f_cross(v1, r1, body2->angularVel);
        vec3f_add(v1, body2->linearVel);
        vec3f_sub(dv, v1);
    }

    // Normal Impulse Code
    Vec3f temp1_1, temp1_2, temp2_1, temp2_2;
    vec3f_cross(temp1_1, normal, r0);
    linear_mtxf_mul_vec3f(body1->invInertia, temp1_2, temp1_1);
    vec3f_cross(temp1_1, r0, temp1_2);

    if (doSecondBody) {
        vec3f_cross(temp2_1, normal, r1);
        linear_mtxf_mul_vec3f(body2->invInertia, temp2_2, temp2_1);
        vec3f_cross(temp2_1, r1, temp2_2);
        vec3f_add(temp1_1, temp2_1);
    }

    f32 kNormal = body1InvMass + body2InvMass + vec3f_dot(normal, temp1_1);

    f32 vn = vec3f_dot(dv, normal);
    f32 bias = PENETRATION_BIAS * MAX(0.f, penetration - PENETRATION_MIN_DEPTH) * NUM_RIGID_BODY_STEPS;

    f32 dPn = MAX(((-vn + bias) / kNormal), 0.f);

    Vec3f P;
    vec3f_scale(P, normal, dPn);

    // Apply impulse
    vec3f_add_scaled(body1->linearVel, P, body1InvMass);
    vec3f_cross(temp1_1, P, r0);
    linear_mtxf_mul_vec3f(body1->invInertia, temp1_2, temp1_1);
    vec3f_add(body1->angularVel, temp1_2);

    if (doSecondBody) {
        vec3f_add_scaled(body2->linearVel, P, -body2InvMass);
        vec3f_cross(temp2_1, P, r1);
        linear_mtxf_mul_vec3f(body2->invInertia, temp2_2, temp2_1);
        vec3f_sub(body2->angularVel, temp2_2);
    }

    // Tangent Impulse Code (friction)
    {
        Vec3f tangent;
        tangent[0] = dv[0] - normal[0] * vn;
        tangent[1] = dv[1] - normal[1] * vn;
        tangent[2] = dv[2] - normal[2] * vn;
        vec3f_normalize(tangent);
        f32 vt = vec3f_dot(dv, tangent);

        vec3f_cross(temp1_1, tangent, r0);
        linear_mtxf_mul_vec3f(body1->invInertia, temp1_2, temp1_1);
        vec3f_cross(temp1_1, r0, temp1_2);

        if (doSecondBody) {
            vec3f_cross(temp2_1, tangent, r1);
            linear_mtxf_mul_vec3f(body2->invInertia, temp2_2, temp2_1);
            vec3f_cross(temp2_1, r1, temp2_2);

            vec3f_add(temp1_1, temp2_1);
        }

        f32 kTangent = body1InvMass + body2InvMass + vec3f_dot(tangent, temp1_1);

        f32 maxPt = FRICTION * dPn;
        f32 dPt = CLAMP((-vt / kTangent), -maxPt, maxPt);

        vec3f_scale(P, tangent, dPt);

        // Apply impulse
        vec3f_add_scaled(body1->linearVel, P, body1InvMass);
        vec3f_cross(temp1_1, P, r0);
        linear_mtxf_mul_vec3f(body1->invInertia, temp1_2, temp1_1);
        vec3f_add(body1->angularVel, temp1_2);

        if (doSecondBody) {
            vec3f_add_scaled(body2->linearVel, P, -body2InvMass);
            vec3f_cross(temp2_1, P, r1);
            linear_mtxf_mul_vec3f(body2->invInertia, temp2_2, temp2_1);
            vec3f_sub(body2->angularVel, temp2_2);
        }
    }
}

/// Updates the rigid body's transformation matrix and its inertia tensor.
void rigid_body_update_matrix(struct RigidBody *body) {
    mtxf_from_quat(body->angleQuat, *body->transform);

    // Calculate the inverse of the inertia tensor.
    // will need to be modified a ton for rigid bodies that aren't uniform size in all dimensions
	f32 i = 1.f / ((body->size[0] * body->size[0]) * body->mass);
    mtxf_identity(body->invInertia);
    body->invInertia[0][0] = i;
    body->invInertia[1][1] = i;
    body->invInertia[2][2] = i;

    vec3f_copy((*body->transform)[3], body->centerOfMass);
}

/// Allocate a rigid body and return a pointer to it.
struct RigidBody *allocate_rigid_body(struct MeshInfo *mesh, f32 mass, Vec3f size, Vec3f pos, Mat4 *transform) {
    // Search list for deallocated rigid body
    for (u32 i = 0; i < MAX_RIGID_BODIES; i++) {
        struct RigidBody *body = &gRigidBodies[i];
        if (!body->allocated) {
            body->allocated = TRUE;
            body->hasGravity = TRUE;
            body->floating = FALSE;
            body->asleep = FALSE;
            body->mesh = mesh;
            body->mass = mass;
            if (mass != 0.f) {
                body->invMass = 1.0f / mass;
                body->isStatic = FALSE;
            } else {
                body->invMass = 0.0f;
                body->isStatic = TRUE;
            }
            vec3f_copy(body->size, size);
            vec3f_copy(body->centerOfMass, pos);
            body->angleQuat[0] = 0.0f;
            body->angleQuat[1] = 0.0f;
            body->angleQuat[2] = 0.0f;
            body->angleQuat[3] = 1.0f;
            body->motion = 10.0f;
            vec3f_set(body->linearVel, 0.0f, 0.0f, 0.0f);
            vec3f_set(body->angularVel, 0.0f, 0.0f, 0.0f);
            vec3f_set(body->netForce, 0.0f, 0.0f, 0.0f);
            vec3f_set(body->netTorque, 0.0f, 0.0f, 0.0f);

            body->transform = transform;
            body->obj = NULL;
            rigid_body_update_matrix(body);
            return body;
        }
    }
    return NULL;
}

void rigid_body_set_yaw(struct RigidBody *body, s16 yaw) {
    s32 ang = (-yaw) >> 1;
    body->angleQuat[0] = 0.f;
    body->angleQuat[1] = sins(ang);
    body->angleQuat[2] = 0.f;
    body->angleQuat[3] = coss(ang);
    rigid_body_update_matrix(body);
}

/// Deallocates a rigid body.
void deallocate_rigid_body(struct RigidBody *body) {
    body->allocated = FALSE;
}

u32 rigid_bodies_near(struct RigidBody *body1, struct RigidBody *body2) {
    if (body1->maxCorner[0] < body2->minCorner[0]) return FALSE;
    if (body1->minCorner[0] > body2->maxCorner[0]) return FALSE;
    if (body1->maxCorner[1] < body2->minCorner[1]) return FALSE;
    if (body1->minCorner[1] > body2->maxCorner[1]) return FALSE;
    if (body1->maxCorner[2] < body2->minCorner[2]) return FALSE;
    if (body1->minCorner[2] > body2->maxCorner[2]) return FALSE;
    return TRUE;
}

/// Applies a force to a rigid body at a given point.
void rigid_body_add_force(struct RigidBody *body, Vec3f contactPoint, Vec3f force, u32 wake) {
    // Calculate force
    vec3f_add(body->netForce, force);
    
    // Calculate torque
    // τ = r x F
    Vec3f torque, contactOffset;
    vec3f_copy(contactOffset, contactPoint);
    vec3f_sub(contactOffset, body->centerOfMass);
    vec3f_cross(torque, force, contactOffset);
    vec3f_add(body->netTorque, torque);

    if (wake) {
        body->asleep = FALSE;
        body->motion = 10.f;
    }
}

/// Updates the position of a rigid body based on its velocity.
void rigid_body_update_position(struct RigidBody *body) {
    if (body->isStatic) {
        return;
    }

        f32 motion = vec3f_dot(body->linearVel, body->linearVel) + vec3f_dot(body->angularVel, body->angularVel);
    body->motion = SLEEP_DETECTION_BIAS * body->motion + (1 - SLEEP_DETECTION_BIAS) * motion;

    if (body->motion < SLEEP_DETECTION_THRESHOLD) {
        body->asleep = TRUE;
        vec3f_set(body->linearVel, 0.0f, 0.0f, 0.0f);
        vec3f_set(body->angularVel, 0.0f, 0.0f, 0.0f);
    }

    // Apply linear velocity
    // Δx = v * Δt
    vec3f_add_scaled(body->centerOfMass, body->linearVel, dt);
    
    // Apply angular velocity
    // Δθ = ω * Δt
    Quat angleChange;
    f32 norm = sqrtf(body->angularVel[0] * body->angularVel[0] + body->angularVel[1] * body->angularVel[1] + body->angularVel[2] * body->angularVel[2]);
    norm *= dt;

    if (norm > 0.0001f) {
        // Create quaternion from angular velocity
        f32 sinVal = sinf(norm * 0.5f) / norm;
        f32 cosVal = cosf(norm * 0.5f);

        angleChange[0] = body->angularVel[0] * dt * sinVal;
        angleChange[1] = body->angularVel[1] * dt * sinVal;
        angleChange[2] = body->angularVel[2] * dt * sinVal;
        angleChange[3] = cosVal;

        // Apply and normalize quaternion
        Quat curRot;
        quat_copy(curRot, body->angleQuat);
        quat_mul(body->angleQuat, curRot, angleChange);
        quat_normalize(body->angleQuat);
    }

    // Reset forces and torques
    vec3f_set(body->netForce, 0.0f, 0.0f, 0.0f);
    vec3f_set(body->netTorque, 0.0f, 0.0f, 0.0f);

    rigid_body_update_matrix(body);

    if (body->obj != NULL) {
        vec3f_copy(&body->obj->oPosVec, body->centerOfMass);
        body->obj->header.gfx.throwMatrix = body->transform;
    }
}

#include "game_init.h"
#include "game/level_update.h"

void rigid_body_apply_gravity(struct RigidBody *body) {
    Vec3f gravityForce;
    vec3f_set(gravityForce, 0.f, GRAVITY_FORCE * body->mass, 0.f);
    rigid_body_add_force(body, body->centerOfMass, gravityForce, FALSE);
}

void rigid_body_apply_buoyancy(struct RigidBody *body) {
    body->floating = FALSE;
    // Iterate over vertices of a cube
    for (s32 x = -1; x < 2; x+=2) {
        for (s32 y = -1; y < 2; y+=2) {
            for (s32 z = -1; z < 2; z+=2) {
                Vec3f buoyancyPoint, buoyancyPointWorld;
                vec3f_set(buoyancyPoint, x*0.5f, y*0.5f, z*0.5f);
                vec3f_mul(buoyancyPoint, body->size);
                linear_mtxf_mul_vec3f_and_translate(*body->transform, buoyancyPointWorld, buoyancyPoint);
                // Check if underwater
                f32 waterLevel = find_water_level(buoyancyPointWorld[0], buoyancyPointWorld[2]);
                if (buoyancyPointWorld[1] < waterLevel) {
                    Vec3f buoyancyForce;
                    vec3f_set(buoyancyForce, 0.f, -GRAVITY_FORCE * body->mass / 7.f, 0.f);
                    rigid_body_add_force(body, buoyancyPointWorld, buoyancyForce, FALSE);
                    body->linearVel[0] *= 0.99f;
                    body->linearVel[1] *= 0.999f; // dampened less
                    body->linearVel[2] *= 0.99f;
                    body->angularVel[0] *= 0.99f;
                    body->angularVel[1] *= 0.99f;
                    body->angularVel[2] *= 0.99f;
                    body->floating = TRUE;
                }
            }
        }
    }
}

/// Updates the velocity of a rigid body.
void rigid_body_update_velocity(struct RigidBody *body) {
    if (body->isStatic || body->asleep) {
        return;
    }

    // Apply Gravity
    // Fg = m * g
    if (body->hasGravity) {
        rigid_body_apply_gravity(body);
    }
    rigid_body_apply_buoyancy(body);

    // Calculate linear velocity
    // Δv = (F / m) * Δt
    vec3f_add_scaled(body->linearVel, body->netForce, body->invMass * dt);

    // Calculate angular velocity
    // Δω = (τ / I) * Δt
    Vec3f angularAccel;
    linear_mtxf_mul_vec3f(body->invInertia, angularAccel, body->netTorque);
    vec3f_add_scaled(body->angularVel, angularAccel, dt);

    // Damping
    vec3f_scale(body->linearVel, body->linearVel, DAMPING);
    vec3f_scale(body->angularVel, body->angularVel, DAMPING);

    rigid_body_update_matrix(body);
}

/// Apply impulses to rigid bodies to resolve stored collisions.
void apply_impulses(void) {
    for (u32 i = 0; i < gNumCollisions; i++) {
        struct Collision *col = &gCollisions[i];

        for (u32 j = 0; j < col->numPoints; j++) {
            struct CollisionPoint *colPoint = &col->points[j];
            Vec3f normal;
            vec3f_copy(normal, colPoint->normal);
            vec3f_normalize(normal);
            rigid_body_collision_impulse(col->body1, col->body2, colPoint->point, normal, colPoint->penetration);
        }
    }
}

/// Perform one step for the rigid body physics system.
void do_rigid_body_step(void) {
    gNumCollisions = 0;
    
    if (gPlayer1Controller->buttonPressed & D_JPAD) {
        for (u32 i = 0; i < 1; i++) {
            if (gRigidBodies[i].allocated) {
                Vec3f force;
                vec3f_set(force, 40.f * sins(gMarioState->faceAngle[1]), 40.f, 40.f * coss(gMarioState->faceAngle[1]));
                rigid_body_add_force(&gRigidBodies[i], gMarioState->pos, force, TRUE);
            }
        }
    }

    // Check collisions
    for (u32 i = 0; i < MAX_RIGID_BODIES; i++) {
        rigid_body_do_collision(i);
    }

    // Update velocity
    for (u32 i = 0; i < MAX_RIGID_BODIES; i++) {
        if (gRigidBodies[i].allocated) {
            rigid_body_update_velocity(&gRigidBodies[i]);
        }
    }
    // Apply impulses
    for (u32 iter = 0; iter < NUM_IMPULSE_ITERATIONS; iter++) {
        apply_impulses();
    }
    // Update position
    for (u32 i = 0; i < MAX_RIGID_BODIES; i++) {
        if (gRigidBodies[i].allocated) {
            rigid_body_update_position(&gRigidBodies[i]);
        }
    }
}

void check_hit_rigid_body_floor_or_ceiling(struct Surface *surf, Vec3f pos, f32 yvel) {
    if (surf->object != NULL) {
        if (surf->object->rigidBody != NULL) {
            Vec3f force;
            vec3f_set(force, 0.f, yvel, 0.f);
            rigid_body_add_force(surf->object->rigidBody, pos, force, TRUE);
        }
    }
}

void check_hit_rigid_body_wall(struct Surface *wall, Vec3f pos, f32 fvel, f32 yvel, s16 yaw) {
    if (wall->object != NULL) {
        if (wall->object->rigidBody != NULL) {
            Vec3f force;
            force[0] = fvel * sins(yaw);
            force[1] = yvel;
            force[2] = fvel * coss(yaw);
            rigid_body_add_force(wall->object->rigidBody, pos, force, TRUE);
        }
    }
}

void rigid_body_general_object_loop(void) {
    s16 animID = gMarioObject->header.gfx.animInfo.animID;

    if (obj_check_if_collided_with_object(o, gMarioObject) && 
        (animID == MARIO_ANIM_SIDESTEP_RIGHT || animID == MARIO_ANIM_SIDESTEP_LEFT || animID == MARIO_ANIM_PUSHING)) {
        s16 angleToMario = obj_angle_to_object(o, gMarioObject);
        if (abs_angle_diff(angleToMario, gMarioObject->oMoveAngleYaw) > 0x7000) {
            Vec3f force;
            force[0] = sins(gMarioObject->oMoveAngleYaw) * 10.f;
            force[1] = 0.0f;
            force[2] = coss(gMarioObject->oMoveAngleYaw) * 10.f;
            Vec3f loc;
            vec3f_copy(loc, &gMarioObject->oPosVec);
            loc[1] += 100.f;
            rigid_body_add_force(o->rigidBody, loc, force, TRUE);
            o->rigidBody->motion = 10.f;
        }
    }

    if (o->oPosY < -5000.f) {
        mark_obj_for_deletion(o);
    }
}
