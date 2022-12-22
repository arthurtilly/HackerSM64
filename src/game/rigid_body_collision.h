#pragma once

#include <PR/ultratypes.h>
#include "macros.h"
#include "engine/math_util.h"

typedef u8 MeshEdge[2];
typedef u8 MeshTri[3];
typedef u8 MeshQuad[4];

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
    struct CollisionPoint points[50];
    u32 numPoints;
};

struct MeshInfo {
    Vec3f *vertices; // will be scaled by rigid body's size
    MeshEdge *edges;
    MeshTri *tris;
    MeshQuad *quads;
    u32 numVertices;
    u32 numEdges;
    u32 numTris;
    u32 numQuads;
};

struct TriangleInfo {
    Vec3f vertices[3];
    Vec3f normal;
};

struct QuadInfo {
    Vec3f vertices[4];
    Vec3f normal;
};

extern struct Collision gCollisions[20];
extern u32 gNumCollisions;

extern struct MeshInfo Cube_Mesh;

void rigid_body_do_collision(u32 bodyIndex);