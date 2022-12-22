#include <PR/ultratypes.h>

#include "config.h"
#include "sm64.h"
#include "rigid_body.h"
#include "engine/surface_load.h"

struct Collision gCollisions[20];
u32 gNumCollisions = 0;

Vec3f Cube_Vertices[8] = {
    {-1.f, -1.f, -1.f},
    { 1.f, -1.f, -1.f},
    { 1.f,  1.f, -1.f},
    {-1.f,  1.f, -1.f},
    {-1.f, -1.f,  1.f},
    { 1.f, -1.f,  1.f},
    { 1.f,  1.f,  1.f},
    {-1.f,  1.f,  1.f}
};

MeshEdge Cube_Edges[12] = {
    {0, 1},
    {1, 2},
    {2, 3},
    {3, 0},
    {4, 5},
    {5, 6},
    {6, 7},
    {7, 4},
    {0, 4},
    {1, 5},
    {2, 6},
    {3, 7}
};

MeshQuad Cube_Quads[6] = {
    {0, 1, 2, 3},
    {4, 5, 6, 7},
    {0, 1, 5, 4},
    {1, 2, 6, 5},
    {2, 3, 7, 6},
    {3, 0, 4, 7}
};

struct MeshInfo Cube_Mesh = {
    Cube_Vertices,
    Cube_Edges,
    NULL,
    Cube_Quads,
    8, // Number of vertices
    12, // Number of edges
    0,
    6 // Number of quads
};

struct RigidBody gRigidBodyFloor = {
    TRUE,
    TRUE,
    FALSE,
    FALSE,

    0.f,
    0.f,
    0.f,
    0.f,
    {0.f, 0.f, 0.f},
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

    NULL,
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
#ifdef PUPPYPRINT_DEBUG
    pNumCols++;
#endif
    // Check if there is a nearby point already in the collision.
    for (s32 i = 0; i < collision->numPoints; i++) {
        Vec3f dist;
        vec3f_sub2(dist, point, collision->points[i].point);
        if (vec3f_dot(dist, dist) < 0.1f) {
            return;
        }
    }
#ifdef PUPPYPRINT_DEBUG
    pNumColsTrunc++;
#endif
    struct CollisionPoint *colPoint = &collision->points[collision->numPoints];
    vec3f_copy(colPoint->point, point);
    vec3f_copy(colPoint->normal, normal);
    colPoint->penetration = penetration;
    collision->numPoints++;
}

void vec3f_sub2(Vec3f dest, Vec3f a, Vec3f b) {
    dest[0] = a[0] - b[0];
    dest[1] = a[1] - b[1];
    dest[2] = a[2] - b[2];
}

/// Check how close a point is to a plane along the plane's normal.
f32 point_in_plane(Vec3f point, Vec3f planePoint, Vec3f normal) {
    Vec3f planeToPointRelative;
    vec3f_sub2(planeToPointRelative, planePoint, point);
    return vec3f_dot(planeToPointRelative, normal);
}

/// Check if a point is inside a triangle, assuming the point is coplanar with the triangle.
s32 point_is_in_tri(Vec3f point, struct TriangleInfo *tri) {
    Vec3f edge1, edge2, edge3;
    Vec3f pointToVertex1, pointToVertex2, pointToVertex3;
    Vec3f cross1, cross2, cross3;
    f32 dot1, dot2, dot3;

    vec3f_sub2(edge1, tri->vertices[1], tri->vertices[0]);
    vec3f_sub2(edge2, tri->vertices[2], tri->vertices[1]);
    vec3f_sub2(edge3, tri->vertices[0], tri->vertices[2]);

    vec3f_sub2(pointToVertex1, point, tri->vertices[0]);
    vec3f_sub2(pointToVertex2, point, tri->vertices[1]);
    vec3f_sub2(pointToVertex3, point, tri->vertices[2]);

    vec3f_cross(cross1, edge1, pointToVertex1);
    vec3f_cross(cross2, edge2, pointToVertex2);
    vec3f_cross(cross3, edge3, pointToVertex3);

    dot1 = vec3f_dot(cross1, tri->normal);
    dot2 = vec3f_dot(cross2, tri->normal);
    dot3 = vec3f_dot(cross3, tri->normal);

    return (dot1 >= 0.f && dot2 >= 0.f && dot3 >= 0.f) || (dot1 <= 0.f && dot2 <= 0.f && dot3 <= 0.f);
}

/// Check if a point is inside a quad.
s32 point_is_in_quad(Vec3f point, struct QuadInfo *quad) {
    Vec3f edge1, edge2, edge3, edge4;
    Vec3f pointToVertex1, pointToVertex2, pointToVertex3, pointToVertex4;
    Vec3f cross1, cross2, cross3, cross4;
    f32 dot1, dot2, dot3, dot4;

    vec3f_sub2(edge1, quad->vertices[1], quad->vertices[0]);
    vec3f_sub2(edge2, quad->vertices[2], quad->vertices[1]);
    vec3f_sub2(edge3, quad->vertices[3], quad->vertices[2]);
    vec3f_sub2(edge4, quad->vertices[0], quad->vertices[3]);

    vec3f_sub2(pointToVertex1, point, quad->vertices[0]);
    vec3f_sub2(pointToVertex2, point, quad->vertices[1]);
    vec3f_sub2(pointToVertex3, point, quad->vertices[2]);
    vec3f_sub2(pointToVertex4, point, quad->vertices[3]);

    vec3f_cross(cross1, edge1, pointToVertex1);
    vec3f_cross(cross2, edge2, pointToVertex2);
    vec3f_cross(cross3, edge3, pointToVertex3);
    vec3f_cross(cross4, edge4, pointToVertex4);

    dot1 = vec3f_dot(cross1, quad->normal);
    dot2 = vec3f_dot(cross2, quad->normal);
    dot3 = vec3f_dot(cross3, quad->normal);
    dot4 = vec3f_dot(cross4, quad->normal);

    return (dot1 >= 0.f && dot2 >= 0.f && dot3 >= 0.f && dot4 >= 0.f) || (dot1 <= 0.f && dot2 <= 0.f && dot3 <= 0.f && dot4 <= 0.f);
}

/// Find the point of intersection between a line and a plane. Returns FALSE if the line doesn't intersect.
s32 edge_intersects_plane(Vec3f intersectionPoint, Vec3f edgePoint1, Vec3f edgePoint2, Vec3f planePoint, Vec3f planeNormal) {
    Vec3f lineDir, relPlane;
    // Find the point of intersection.
    vec3f_sub2(lineDir, edgePoint2, edgePoint1);
    f32 dot = vec3f_dot(planeNormal, lineDir);
    if (dot == 0.f) return FALSE;
    vec3f_sub2(relPlane, planePoint, edgePoint1);
    dot = vec3f_dot(planeNormal, relPlane) / dot;
    if (dot < 0.f || dot > 1.f) return FALSE;
    vec3f_scale(intersectionPoint, lineDir, dot);
    vec3f_add(intersectionPoint, edgePoint1);
    return TRUE;
}

/// Check if a mesh's vertices are intersecting a triangle's face.
void vertices_vs_tri_face(Vec3f vertices[], u32 numVertices, struct TriangleInfo *tri, struct Collision *col) {
    for (u32 i = 0; i < numVertices; i++) {
#ifdef PUPPYPRINT_DEBUG
        pNumVertexChecks++;
#endif
        f32 distance = point_in_plane(vertices[i], tri->vertices[0], tri->normal);
        if (distance <= PENETRATION_MIN_DEPTH || distance >= PENETRATION_MAX_DEPTH) continue;
        if (point_is_in_tri(vertices[i], tri)) {
            add_collision(col, vertices[i], tri->normal, distance);
        }
    }
}

/// Check if a mesh's vertices are intersecting a quad's face.
void vertices_vs_quad_face(Vec3f vertices[], u32 numVertices, struct QuadInfo *quad, struct Collision *col) {
    for (u32 i = 0; i < numVertices; i++) {
#ifdef PUPPYPRINT_DEBUG
        pNumVertexChecks++;
#endif
        f32 distance = point_in_plane(vertices[i], quad->vertices[0], quad->normal);
        if (distance <= PENETRATION_MIN_DEPTH || distance >= PENETRATION_MAX_DEPTH) continue;
        if (point_is_in_quad(vertices[i], quad)) {
            add_collision(col, vertices[i], quad->normal, distance);
        }
    }
}

/// Check if a mesh's edges are intersecting an edge belonging to a face.
void edges_vs_edge(Vec3f vertices[], MeshEdge edges[], u32 numEdges, Vec3f edgePoint1, Vec3f edgePoint2, Vec3f edgeNormal, struct Collision *col) {
    Vec3f temp, edge, closestPointOnEdge, planeNormal, intersectionPoint;
    
    vec3f_sub2(edge, edgePoint2, edgePoint1);
    vec3f_cross(planeNormal, edgeNormal, edge);
    vec3f_normalize(planeNormal);
    for (u32 i = 0; i < numEdges; i++) {
#ifdef PUPPYPRINT_DEBUG
        pNumEdgeChecks++;
#endif
        if (edge_intersects_plane(intersectionPoint, vertices[edges[i][0]], vertices[edges[i][1]], edgePoint1, planeNormal)) {
            // Find distance from intersection point to edge
            vec3f_sub2(temp, edgePoint1, intersectionPoint);
            f32 distance = vec3f_dot(temp, edgeNormal);
            if (distance <= PENETRATION_MIN_DEPTH || distance >= PENETRATION_MAX_DEPTH) continue;

            // Find closest point to intersection point on edge
            vec3f_scale(closestPointOnEdge, edgeNormal, -distance);
            vec3f_add(closestPointOnEdge, intersectionPoint);

            // Check that closest point is on line segment
            vec3f_sub2(temp, closestPointOnEdge, edgePoint1);
            f32 dot = vec3f_dot(temp, edge);
            if (dot < 0.f || dot > vec3f_dot(edge, edge)) continue;

            add_collision(col, intersectionPoint, edgeNormal, distance);
        }
    }
}

void tris_vs_vertex(struct TriangleInfo tris[], u32 numTris, Vec3f point, Vec3f vertexNormal, struct Collision *col) {
    for (u32 i = 0; i < numTris; i++) {
#ifdef PUPPYPRINT_DEBUG
        pNumFaceChecks++;
#endif
        Vec3f edge1;
        vec3f_sub2(edge1, point, tris[i].vertices[0]);
        f32 distance = vec3f_dot(tris[i].normal, vertexNormal);
        if (distance == 0.f) continue;
        distance = vec3f_dot(tris[i].normal, edge1) / distance;
        if (distance <= PENETRATION_MIN_DEPTH || distance >= PENETRATION_MAX_DEPTH) continue;

        if (point_is_in_tri(point, &tris[i])) {
            Vec3f colPoint;
            vec3f_scale(colPoint, vertexNormal, -distance);
            vec3f_add(colPoint, point);
            add_collision(col, colPoint, vertexNormal, distance);
        }
    }
}

void quads_vs_vertex(struct QuadInfo quads[], u32 numQuads, Vec3f point, Vec3f vertexNormal, struct Collision *col) {
    for (u32 i = 0; i < numQuads; i++) {
#ifdef PUPPYPRINT_DEBUG
        pNumFaceChecks++;
#endif
        Vec3f edge1;
        vec3f_sub2(edge1, point, quads[i].vertices[0]);
        f32 distance = vec3f_dot(quads[i].normal, vertexNormal);
        if (distance == 0.f) continue;
        distance = vec3f_dot(quads[i].normal, edge1) / distance;
        if (distance <= PENETRATION_MIN_DEPTH || distance >= PENETRATION_MAX_DEPTH) continue;

        if (point_is_in_quad(point, &quads[i])) {
            Vec3f colPoint;
            vec3f_scale(colPoint, vertexNormal, -distance);
            vec3f_add(colPoint, point);
            add_collision(col, colPoint, vertexNormal, distance);
        }
    }
}

// Buffer to store the locations of each vertex of the current rigid body in world space.
static Vec3f sCurrentVertices[50];
static Vec3f sCurrentVertices2[50];

static struct TriangleInfo sCurrentTris[50];
static struct TriangleInfo sCurrentTris2[50];

static struct QuadInfo sCurrentQuads[50];
static struct QuadInfo sCurrentQuads2[50];

/// Transform all the vertices of the current rigid body.
void calculate_mesh(struct RigidBody *body, Vec3f vertices[], struct TriangleInfo tris[], struct QuadInfo quads[]) {
    // Calculate vertices
    for (u32 i = 0; i < body->mesh->numVertices; i++) {
        Vec3f vertex;
        vec3f_copy(vertex, body->mesh->vertices[i]);
        vec3f_mul(vertex, body->size);
        linear_mtxf_mul_vec3f_and_translate(*body->transform, vertices[i], vertex);
    }
    Vec3f edge1, edge2;
    // Calculate tris
    for (u32 i = 0; i < body->mesh->numTris; i++) {
        vec3f_copy(tris[i].vertices[0], vertices[body->mesh->tris[i][0]]);
        vec3f_copy(tris[i].vertices[1], vertices[body->mesh->tris[i][1]]);
        vec3f_copy(tris[i].vertices[2], vertices[body->mesh->tris[i][2]]);

        vec3f_sub2(edge1, tris[i].vertices[1], tris[i].vertices[0]);
        vec3f_sub2(edge2, tris[i].vertices[2], tris[i].vertices[0]);
        vec3f_cross(tris[i].normal, edge1, edge2);
        vec3f_normalize(tris[i].normal);
    }
    // Calculate quads
    for (u32 i = 0; i < body->mesh->numQuads; i++) {
        vec3f_copy(quads[i].vertices[0], vertices[body->mesh->quads[i][0]]);
        vec3f_copy(quads[i].vertices[1], vertices[body->mesh->quads[i][1]]);
        vec3f_copy(quads[i].vertices[2], vertices[body->mesh->quads[i][2]]);
        vec3f_copy(quads[i].vertices[3], vertices[body->mesh->quads[i][3]]);

        vec3f_sub2(edge1, quads[i].vertices[1], quads[i].vertices[0]);
        vec3f_sub2(edge2, quads[i].vertices[2], quads[i].vertices[0]);
        vec3f_cross(quads[i].normal, edge1, edge2);
        vec3f_normalize(quads[i].normal);
    }
}

/// Determine if a rigid body is near a triangle.
s32 is_body_near_tri(struct RigidBody *body, struct TriangleInfo *tri) {
    f32 distance = -point_in_plane(body->centerOfMass, tri->vertices[0], tri->normal);
    if (distance > body->diagonal || distance < 0.f) return FALSE;
    Vec3f closestPoint;
    vec3f_scale(closestPoint, tri->normal, -distance);
    vec3f_add(closestPoint, body->centerOfMass);
    
    s32 maxTriX = MAX(tri->vertices[0][0], MAX(tri->vertices[1][0], tri->vertices[2][0]));
    s32 minTriX = MIN(tri->vertices[0][0], MIN(tri->vertices[1][0], tri->vertices[2][0]));
    s32 maxTriY = MAX(tri->vertices[0][1], MAX(tri->vertices[1][1], tri->vertices[2][1]));
    s32 minTriY = MIN(tri->vertices[0][1], MIN(tri->vertices[1][1], tri->vertices[2][1]));
    s32 maxTriZ = MAX(tri->vertices[0][2], MAX(tri->vertices[1][2], tri->vertices[2][2]));
    s32 minTriZ = MIN(tri->vertices[0][2], MIN(tri->vertices[1][2], tri->vertices[2][2]));

    if (body->centerOfMass[0] - body->diagonal > maxTriX) return FALSE;
    if (body->centerOfMass[0] + body->diagonal < minTriX) return FALSE;
    if (body->centerOfMass[1] - body->diagonal > maxTriY) return FALSE;
    if (body->centerOfMass[1] + body->diagonal < minTriY) return FALSE;
    if (body->centerOfMass[2] - body->diagonal > maxTriZ) return FALSE;
    if (body->centerOfMass[2] + body->diagonal < minTriZ) return FALSE;
    return TRUE;
}

/// Check for collisions between a rigid body and a static floor triangle.
void body_vs_surface_collision(struct RigidBody *body, struct Surface *tri, struct Collision *col) {
    struct TriangleInfo triInfo;
    vec3s_to_vec3f(triInfo.vertices[0], tri->vertex1);
    vec3s_to_vec3f(triInfo.vertices[1], tri->vertex2);
    vec3s_to_vec3f(triInfo.vertices[2], tri->vertex3);
    vec3f_copy(triInfo.normal, &tri->normal.x);
    if (!is_body_near_tri(body, &triInfo)) return;
#ifdef PUPPYPRINT_DEBUG
    pNumTrisChecked++;
#endif
    struct MeshInfo *mesh = body->mesh;

    u32 prevCollisions = col->numPoints;

    // Collision detection
    vertices_vs_tri_face(sCurrentVertices, mesh->numVertices, &triInfo, col);

    if (col->numPoints - prevCollisions < 4) {
        for (u32 i = 0; i < 3; i++) {
            edges_vs_edge(sCurrentVertices, mesh->edges, mesh->numEdges, triInfo.vertices[i], triInfo.vertices[i == 2 ? 0 : i + 1], triInfo.normal, col);
        }
    }
    
    if (mesh->numTris > 0) {
        for (u32 i = 0; i < 3; i++) {
            tris_vs_vertex(sCurrentTris, mesh->numTris, triInfo.vertices[i], triInfo.normal, col);
        }
    }

    if (mesh->numQuads > 0) {
        for (u32 i = 0; i < 3; i++) {
            quads_vs_vertex(sCurrentQuads, mesh->numQuads, triInfo.vertices[i], triInfo.normal, col);
        }
    }
}

f32 find_floor(f32 x, f32 y, f32 z, struct Surface **floor);

/// Checks for collisions for the current rigid body.
void rigid_body_check_surf_collisions(struct RigidBody *body) {
    if (body->isStatic || body->asleep) {
        return;
    }

    struct Collision *col = init_collision(body, &gRigidBodyFloor);
    s32 minCellX = GET_CELL_COORD(body->centerOfMass[0] - body->diagonal);
    s32 minCellZ = GET_CELL_COORD(body->centerOfMass[2] - body->diagonal);
    s32 maxCellX = GET_CELL_COORD(body->centerOfMass[0] + body->diagonal);
    s32 maxCellZ = GET_CELL_COORD(body->centerOfMass[2] + body->diagonal);
    // Iterate over all triangles
    for (s32 cellZ = minCellZ; cellZ <= maxCellZ; cellZ++) {
        for (s32 cellX = minCellX; cellX <= maxCellX; cellX++) {
            struct SurfaceNode *cell = gStaticSurfacePartition[cellZ][cellX];
            for (u32 i = 0; i < 3; i++) {
                struct SurfaceNode *node = cell[i].next;
                while (node != NULL) {
                    body_vs_surface_collision(body, node->surface, col);
                    node = node->next;
                }
            }
        }
    }
}

/// Checks for collisions between two rigid bodies.
void rigid_body_check_body_collisions(struct RigidBody *body1, struct RigidBody *body2) {
    u32 body1CantMove = (body1->isStatic || body1->asleep);
    u32 body2CantMove = (body2->isStatic || body2->asleep);
    if (body1CantMove && body2CantMove) {
        return;
    }

    if (!rigid_bodies_near(body1, body2)) {
        return;
    }

    struct Collision *col = init_collision(body1, body2);
    calculate_mesh(body2, sCurrentVertices2, sCurrentTris2, sCurrentQuads2);
    
    // Body 1 verts vs body 2 tris
    if (body2->mesh->numTris > 0) {
        for (u32 i = 0; i < body2->mesh->numTris; i++) {
            // Check for collisions
            vertices_vs_tri_face(sCurrentVertices, body1->mesh->numVertices, &sCurrentTris2[i], col);
        }
    }
    // Body 1 verts vs body 2 quads
    if (body2->mesh->numQuads > 0) {
        for (u32 i = 0; i < body2->mesh->numQuads; i++) {
            // Check for collisions
            vertices_vs_quad_face(sCurrentVertices, body1->mesh->numVertices, &sCurrentQuads2[i], col);
        }
    }

    // Body 1 edges vs body 2 edges
    for (u32 i = 0; i < body2->mesh->numEdges; i++) {
        // Check for collisions
        Vec3f normal;
        vec3f_copy(normal, sCurrentVertices2[body2->mesh->edges[i][0]]);
        vec3f_add(normal, sCurrentVertices2[body2->mesh->edges[i][1]]);
        vec3f_scale(normal, normal, 0.5f);
        vec3f_sub(normal, body2->centerOfMass);
        vec3f_normalize(normal);
        edges_vs_edge(sCurrentVertices, body1->mesh->edges, body1->mesh->numEdges,
            sCurrentVertices2[body2->mesh->edges[i][0]], sCurrentVertices2[body2->mesh->edges[i][1]],
            normal, col);
    }

    u32 prevCollisions = col->numPoints;

    // Body 1 tris vs body 2 verts
    if (body1->mesh->numTris > 0) {
        for (u32 i = 0; i < body2->mesh->numTris; i++) {
            // Check for collisions
            vertices_vs_tri_face(sCurrentVertices2, body2->mesh->numVertices, &sCurrentTris[i], col);
        }
    }

    // Body 1 quads vs body 2 verts
    if (body1->mesh->numQuads > 0) {
        for (u32 i = 0; i < body2->mesh->numQuads; i++) {
            // Check for collisions
            vertices_vs_quad_face(sCurrentVertices2, body2->mesh->numVertices, &sCurrentQuads[i], col);
        }
    }

    // Reverse penetration and normal direction of the past collisions
    for (u32 i = prevCollisions; i < col->numPoints; i++) {
        vec3f_scale(col->points[i].normal, col->points[i].normal, -1.0f);
        col->points[i].penetration = -col->points[i].penetration;
    }
}

void rigid_body_do_collision(u32 bodyIndex) {
    struct RigidBody *body = &gRigidBodies[bodyIndex];
    if (!body->allocated) return;

    calculate_mesh(body, sCurrentVertices, sCurrentTris, sCurrentQuads);

    rigid_body_check_surf_collisions(body);
    for (u32 j = bodyIndex + 1; j < MAX_RIGID_BODIES; j++) {
        if (gRigidBodies[j].allocated) {
            rigid_body_check_body_collisions(body, &gRigidBodies[j]);
        }
    }
}