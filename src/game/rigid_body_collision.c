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
    // Check if there is a nearby point already in the collision.
    for (s32 i = 0; i < collision->numPoints; i++) {
        Vec3f dist;
        vec3f_sub2(dist, point, collision->points[i].point);
        if (vec3f_dot(dist, dist) < 0.1f) {
            return;
        }
    }
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
    Vec3f pointToPlaneRelative;
    vec3f_sub2(pointToPlaneRelative, point, planePoint);
    return vec3f_dot(pointToPlaneRelative, normal);
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

/// Check if vertices are intersecting a triangle's face.
void vertices_vs_tri_face(Vec3f vertices[], u32 numVertices, struct TriangleInfo *tri, struct Collision *col) {
    for (u32 i = 0; i < numVertices; i++) {
        f32 distance = point_in_plane(vertices[i], tri->vertices[0], tri->normal);
        if (distance >= 0.f || distance <= -50.f) continue;
        if (point_is_in_tri(vertices[i], tri)) {
            add_collision(col, vertices[i], tri->normal, distance);
        }
    }
}

/// Check if vertices are intersecting a quad's face.
void vertices_vs_quad_face(Vec3f vertices[], u32 numVertices, struct QuadInfo *quad, struct Collision *col) {
    for (u32 i = 0; i < numVertices; i++) {
        f32 distance = point_in_plane(vertices[i], quad->vertices[0], quad->normal);
        if (distance >= 0.f || distance <= -50.f) continue;
        if (point_is_in_quad(vertices[i], quad)) {
            add_collision(col, vertices[i], quad->normal, distance);
        }
    }
}

/// Check if quads are intersecting any of a triangle's edges.
void edges_vs_tri_edges(Vec3f vertices[], MeshEdge edges[], u32 numEdges, struct TriangleInfo *tri, struct Collision *col) {
    // Calculate normal of a plane that contains edge 2 and is perpendicular to edge 2's normal.
    Vec3f edge1, triEdge, planeNormal, intersectionPoint;
    for (u32 j = 0; j < 3; j++) {
        vec3f_sub2(triEdge, tri->vertices[j == 2 ? 0 : j + 1], tri->vertices[j]);
        vec3f_cross(planeNormal, tri->normal, triEdge);
        vec3f_normalize(planeNormal);
        for (u32 i = 0; i < numEdges; i++) {
            if (edge_intersects_plane(intersectionPoint, vertices[edges[i][0]], vertices[edges[i][1]], tri->vertices[j], planeNormal)) {
                // Find distance from intersection point to triangle edge.
                vec3f_sub2(edge1, intersectionPoint, tri->vertices[j]);
                f32 distance = vec3f_dot(edge1, tri->normal);
                if (distance >= 0.f || distance <= -50.f) continue;
                Vec3f edge2Closest;
                vec3f_scale(edge2Closest, tri->normal, distance);
                vec3f_add(edge2Closest, intersectionPoint);

                vec3f_sub2(edge1, edge2Closest, tri->vertices[j]);
                f32 dot = vec3f_dot(edge1, triEdge);
                if (dot < 0.f || dot > vec3f_dot(triEdge, triEdge)) continue;
                add_collision(col, intersectionPoint, tri->normal, distance);
            }
        }
    }
}

/// Check if edges are intersecting any of a quad's edges.
void edges_vs_quad_edges(Vec3f vertices[], MeshEdge edges[], u32 numEdges, struct QuadInfo *quad, struct Collision *col) {

}

/// Check if tris are intersecting any of a triangle's vertices.
void tris_vs_tri_vertices(Vec3f vertices[], MeshTri tris[], u32 numTris, struct TriangleInfo *tri, struct Collision *col) {

}

/// Check if tris are intersecting any of a quad's vertices.
void tris_vs_quad_vertices(Vec3f vertices[], MeshTri tris[], u32 numTris, struct QuadInfo *quad, struct Collision *col) {

}

/// Check if quads are intersecting any of a triangle's vertices.
void quads_vs_tri_vertices(Vec3f vertices[], MeshQuad quads[], u32 numQuads, struct TriangleInfo *tri, struct Collision *col) {

}

/// Check if quads are intersecting any of a quad's vertices.
void quads_vs_quad_vertices(Vec3f vertices[], MeshQuad quads[], u32 numQuads, struct QuadInfo *quad, struct Collision *col) {

}

// Buffer to store the locations of each vertex of the current rigid body in world space.
static Vec3f sCurrentVertices[50];

/// Transform all the vertices of the current rigid body.
void calculate_vertices(struct RigidBody *body) {
    for (u32 i = 0; i < body->mesh->numVertices; i++) {
        Vec3f vertex;
        vec3f_copy(vertex, body->mesh->vertices[i]);
        vec3f_mul(vertex, body->size);
        linear_mtxf_mul_vec3f_and_translate(*body->transform, sCurrentVertices[i], vertex);
    }
}

/// Determine if a rigid body is near a triangle.
s32 is_body_near_tri(struct RigidBody *body, struct Surface *tri) {
    s32 maxTriX = MAX(tri->vertex1[0], MAX(tri->vertex2[0], tri->vertex3[0]));
    s32 minTriX = MIN(tri->vertex1[0], MIN(tri->vertex2[0], tri->vertex3[0]));
    s32 maxTriY = MAX(tri->vertex1[1], MAX(tri->vertex2[1], tri->vertex3[1]));
    s32 minTriY = MIN(tri->vertex1[1], MIN(tri->vertex2[1], tri->vertex3[1]));
    s32 maxTriZ = MAX(tri->vertex1[2], MAX(tri->vertex2[2], tri->vertex3[2]));
    s32 minTriZ = MIN(tri->vertex1[2], MIN(tri->vertex2[2], tri->vertex3[2]));

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
    if (!is_body_near_tri(body, tri)) return;
    struct MeshInfo *mesh = body->mesh;
    struct TriangleInfo triInfo;
    vec3s_to_vec3f(triInfo.vertices[0], tri->vertex1);
    vec3s_to_vec3f(triInfo.vertices[1], tri->vertex2);
    vec3s_to_vec3f(triInfo.vertices[2], tri->vertex3);
    vec3f_copy(triInfo.normal, &tri->normal.x);
    
    // Transform all the vertices of the current rigid body.
    calculate_vertices(body);

    u32 prevCollisions = col->numPoints;

    // Collision detection
    vertices_vs_tri_face(sCurrentVertices, mesh->numVertices, &triInfo, col);
    if (col->numPoints - prevCollisions < 4) edges_vs_tri_edges(sCurrentVertices, mesh->edges, mesh->numEdges, &triInfo, col);
    //if (mesh->numTris > 0) tris_vs_tri_vertices(sCurrentVertices, mesh->tris, mesh->numTris, &triInfo, col);
    //if (mesh->numQuads > 0) quads_vs_tri_vertices(sCurrentVertices, mesh->quads, mesh->numQuads, &triInfo, col);
}

f32 find_floor(f32 x, f32 y, f32 z, struct Surface **floor);

/// Checks for collisions for the current rigid body.
void rigid_body_check_collisions(struct RigidBody *body) {
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
