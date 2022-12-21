#include <PR/ultratypes.h>

#include "config.h"
#include "sm64.h"
#include "rigid_body.h"
#include "rigid_body_collision.h"

struct Collision gCollisions[20];
u32 gNumCollisions = 0;

struct RigidBody gRigidBodyFloor = {
    TRUE,
    RIGID_BODY_TYPE_TRIANGLE,
    TRUE,
    FALSE,

    0.f,
    0.f,
    0.f,
    0.f,
    {0.f, 0.f, 0.f},
    {0.f, 0.f, 0.f, 1.f},

    {0.f, 0.f, 0.f},
    {0.f, 0.f, 0.f},
    {0.f, 0.f, 0.f},
    {0.f, 0.f, 0.f},

    NULL,
    {0.f, 0.f, 0.f, 0.f,
     0.f, 0.f, 0.f, 0.f,
     0.f, 0.f, 0.f, 0.f,
     0.f, 0.f, 0.f, 0.f},

    NULL
};

/// Initializes a collision struct. 
struct Collision *init_collision(struct RigidBody *body1, struct RigidBody *body2) {
    struct Collision *collision = &gCollisions[gNumCollisions];
    gNumCollisions++;

    collision->body1 = body1;
    collision->body2 = body2;
    collision->numPoints = 0;
    return collision;
}

/// Adds a contact point to the given collision struct.
void add_collision(struct Collision *collision, Vec3f point, Vec3f normal, f32 penetration) {
    struct CollisionPoint *colPoint = &collision->points[collision->numPoints];
    vec3f_copy(colPoint->point, point);
    vec3f_copy(colPoint->normal, normal);
    colPoint->penetration = penetration;
    collision->numPoints++;
}

f32 find_floor(f32 x, f32 y, f32 z, struct Surface **floor);

/// Checks for collisions between the given rigid body and the floor.
void rigid_body_check_collisions(struct RigidBody *body) {
    struct Collision *col = init_collision(body, &gRigidBodyFloor);
    u32 i =0;
    for (s32 x = -1; x < 2; x+=2) {
        for (s32 y = -1; y < 2; y+=2) {
            for (s32 z = -1; z < 2; z+=2) {
                Vec3f vertex;
                vec3f_set(vertex, x * body->size, y * body->size, z * body->size);
                
                Vec3f vertexWorld;
                linear_mtxf_mul_vec3f_and_translate(*body->transform, vertexWorld, vertex);

                Vec3f normal;
                struct Surface *floor;
                vec3f_set(normal, 0.f, 1.f, 0.f);
                f32 floorHeight = find_floor(vertexWorld[0], vertexWorld[1], vertexWorld[2], &floor);
                if (floor == NULL) {
                    vec3f_set(normal, 0.f, 1.f, 0.f);
                } else {
                    vec3f_copy(normal, &floor->normal.x);
                }
                if (vertexWorld[1] < floorHeight) {
                    add_collision(col, vertexWorld, normal, floorHeight - vertexWorld[1]);
                }
                i++;
            }
        }
    }
}
