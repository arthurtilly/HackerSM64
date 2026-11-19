#include <PR/ultratypes.h>

#include "sm64.h"
#include "game/ingame_menu.h"
#include "graph_node.h"
#include "behavior_script.h"
#include "behavior_data.h"
#include "debugger/assert.h"
#include "game/memory.h"
#include "game/object_helpers.h"
#include "game/game_init.h"
#include "game/macro_special_objects.h"
#include "surface_collision.h"
#include "math_util.h"
#include "game/mario.h"
#include "game/object_list_processor.h"
#include "surface_load.h"
#include "game/puppyprint.h"
#include "game/debug.h"

#include "config.h"

static u8 *sDynamicSurfacePool;

struct SurfaceBatch {
    struct Surface *surfaces;
    u32 count, capacity;
};

#define SURFACES_PER_LEAF 8

struct SurfaceIndex gStaticSurfaceIndex, gDynamicSurfaceIndex;

static ALWAYS_INLINE s32 centroid_sum(struct Surface *s, s32 axis) {
    return s->vertex1[axis] + s->vertex2[axis] + s->vertex3[axis];
}

// Partition the references at the median without moving any Surface objects.
static void split_surfaces(struct SurfaceTree *tree, u16 *indices, s32 first, s32 end, s32 mid) {
    Vec3i min, max;
    for (s32 axis = 0; axis < 3; axis++) {
        min[axis] = max[axis] = centroid_sum(&tree->surfaces[indices[first]], axis);
    }
    for (s32 i = first + 1; i < end; i++) {
        struct Surface *s = &tree->surfaces[indices[i]];
        for (s32 axis = 0; axis < 3; axis++) {
            s32 center = centroid_sum(s, axis);
            min[axis] = MIN(min[axis], center);
            max[axis] = MAX(max[axis], center);
        }
    }
    s32 axis = 0;
    for (s32 i = 1; i < 3; i++) {
        if (max[i] - min[i] > max[axis] - min[axis]) axis = i;
    }
    while (end - first > 1) {
        s32 pivot = centroid_sum(&tree->surfaces[indices[(first + end) / 2]], axis);
        s32 lo = first, hi = end - 1;
        while (lo <= hi) {
            while (centroid_sum(&tree->surfaces[indices[lo]], axis) < pivot) lo++;
            while (centroid_sum(&tree->surfaces[indices[hi]], axis) > pivot) hi--;
            if (lo <= hi) {
                u16 tmp = indices[lo];
                indices[lo++] = indices[hi];
                indices[hi--] = tmp;
            }
        }
        if (mid <= hi) end = hi + 1;
        else if (mid >= lo) first = lo;
        else break;
    }
}

static u32 build_node(struct SurfaceTree *tree, u16 *indices, u32 nodeIndex, s32 first, s32 end) {
    struct SurfaceBvhNode *node = &tree->nodes[nodeIndex];
    s32 count = end - first;
    if (count <= SURFACES_PER_LEAF) {
        struct Surface *s = &tree->surfaces[indices[first]];
        vec3_copy(node->min, s->vertex1);
        vec3_copy(node->max, s->vertex1);
        for (s32 i = first; i < end; i++) {
            s = &tree->surfaces[indices[i]];
            for (s32 axis = 0; axis < 3; axis++) {
                node->min[axis] = MIN(node->min[axis], min_3(s->vertex1[axis], s->vertex2[axis], s->vertex3[axis]));
                node->max[axis] = MAX(node->max[axis], max_3(s->vertex1[axis], s->vertex2[axis], s->vertex3[axis]));
            }
        }
        node->offset = first;
        node->count = count;
        return nodeIndex + 1;
    }

    s32 leaves = (count + SURFACES_PER_LEAF - 1) / SURFACES_PER_LEAF;
    s32 mid = first + (leaves / 2) * SURFACES_PER_LEAF;
    split_surfaces(tree, indices, first, end, mid);
    u32 left = nodeIndex + 1;
    u32 right = build_node(tree, indices, left, first, mid);
    u32 next = build_node(tree, indices, right, mid, end);
    for (s32 i = 0; i < 3; i++) {
        node->min[i] = MIN(tree->nodes[left].min[i], tree->nodes[right].min[i]);
        node->max[i] = MAX(tree->nodes[left].max[i], tree->nodes[right].max[i]);
    }
    node->offset = next;
    node->count = 0;
    return next;
}

// Pack each leaf's triangles together. The permutation is scratch space reclaimed
// after construction, and queries can walk a leaf with a single pointer.
static void reorder_surfaces(struct Surface *surfaces, u16 *indices, u32 count) {
    for (u32 i = 0; i < count; i++) {
        if (indices[i] == i) continue;
        struct Surface saved = surfaces[i];
        u32 dst = i;
        while (indices[dst] != i) {
            u32 src = indices[dst];
            surfaces[dst] = surfaces[src];
            indices[dst] = dst;
            dst = src;
        }
        surfaces[dst] = saved;
        indices[dst] = dst;
    }
}

static void build_surface_tree(struct SurfaceIndex *index, struct SurfaceBatch *batch) {
    u32 used = batch->count * sizeof(struct Surface);
    for (u32 first = 0; first < batch->count;) {
        u32 count = MIN(batch->count - first, 0x10000u);
        u32 leaves = (count + SURFACES_PER_LEAF - 1) / SURFACES_PER_LEAF;
        u32 nodes = 2 * leaves - 1;
        struct SurfaceTree *tree = (struct SurfaceTree *)((u8 *)batch->surfaces + used);
        u16 *indices = (u16 *)(tree->nodes + nodes);
        used = ALIGN8(used + sizeof(*tree) + nodes * sizeof(*tree->nodes));
        aggressf(used + count * sizeof(*indices) <= batch->capacity, "Collision tree exceeds surface pool");

        tree->surfaces = batch->surfaces + first;
        tree->numNodes = nodes;
        for (u32 i = 0; i < count; i++) indices[i] = i;
        build_node(tree, indices, 0, 0, count);
        if (leaves > 1) reorder_surfaces(tree->surfaces, indices, count);

        tree->next = index->first;
        index->first = tree;
        index->numNodes += nodes;
        first += count;
    }
    index->bytes += used;
    index->numSurfaces += batch->count;
}

static struct SurfaceBatch start_static_batch(void) {
    u32 capacity = main_pool_available() - 0x10;
    struct SurfaceBatch batch = { main_pool_alloc(capacity, MEMORY_POOL_LEFT), 0, capacity };
    return batch;
}

static void finish_static_batch(struct SurfaceBatch *batch) {
    u32 previousBytes = gStaticSurfaceIndex.bytes;
    build_surface_tree(&gStaticSurfaceIndex, batch);
    main_pool_realloc(batch->surfaces, gStaticSurfaceIndex.bytes - previousBytes);
}

static struct Surface *read_surface_data(struct SurfaceBatch *batch, TerrainData *vertexData, TerrainData *indices) {
    TerrainData *a = vertexData + 3 * indices[0];
    TerrainData *b = vertexData + 3 * indices[1];
    TerrainData *c = vertexData + 3 * indices[2];
    Vec3f ab, bc, normal;
    vec3_diff(ab, b, a);
    vec3_diff(bc, c, b);
    vec3_cross(normal, ab, bc);
    f32 lengthSq = vec3_sumsq(normal);
    if (lengthSq == 0.0f) return NULL;
    f32 invLength = 1.0f / sqrtf(lengthSq);
    vec3_scale(normal, invLength);

    struct Surface *surface = &batch->surfaces[batch->count++];
    vec3_copy(surface->vertex1, a);
    vec3_copy(surface->vertex2, b);
    vec3_copy(surface->vertex3, c);
    surface->normal.x = normal[0];
    surface->normal.y = normal[1];
    surface->normal.z = normal[2];
    surface->originOffset = -vec3_dot(normal, a);
    surface->lowerY = min_3(a[1], b[1], c[1]);
    surface->upperY = max_3(a[1], b[1], c[1]);
    return surface;
}

#ifndef ALL_SURFACES_HAVE_FORCE
/**
 * Returns whether a surface has exertion/moves Mario
 * based on the surface type.
 */
static s32 surface_has_force(s32 surfaceType) {
    s32 hasForce = FALSE;

    switch (surfaceType) {
        case SURFACE_0004: // Unused
        case SURFACE_FLOWING_WATER:
        case SURFACE_DEEP_MOVING_QUICKSAND:
        case SURFACE_SHALLOW_MOVING_QUICKSAND:
        case SURFACE_MOVING_QUICKSAND:
        case SURFACE_HORIZONTAL_WIND:
        case SURFACE_INSTANT_MOVING_QUICKSAND:
            hasForce = TRUE;
            break;

        default:
            break;
    }
    return hasForce;
}
#endif

/**
 * Returns whether a surface should have the
 * SURFACE_FLAG_NO_CAM_COLLISION flag.
 */
static s32 surf_has_no_cam_collision(s32 surfaceType) {
    s32 flags = SURFACE_FLAGS_NONE;

    switch (surfaceType) {
        case SURFACE_NO_CAM_COLLISION:
        case SURFACE_NO_CAM_COLLISION_77: // Unused
        case SURFACE_NO_CAM_COL_VERY_SLIPPERY:
        case SURFACE_SWITCH:
            flags = SURFACE_FLAG_NO_CAM_COLLISION;
            break;

        default:
            break;
    }

    return flags;
}

/**
 * Load in the surfaces for a given surface type. This includes setting the flags,
 * exertion, and room.
 */
static void load_surfaces(struct SurfaceBatch *batch, TerrainData **data, TerrainData *vertexData,
                          s32 surfaceType, RoomData **rooms, s32 flags) {
    s32 count = *(*data)++;
#ifdef ALL_SURFACES_HAVE_FORCE
    s32 hasForce = TRUE;
#else
    s32 hasForce = surface_has_force(surfaceType);
#endif
    struct Object *object = rooms == NULL ? o : NULL;
    // The DDD warp moves after loading and needs its destination room.
    RoomData room = object && object->behavior == segmented_to_virtual(bhvDddWarp) ? 5 : 0;
    flags |= surf_has_no_cam_collision(surfaceType);
    if (SURFACE_IS_NEW_WATER(surfaceType) && !(flags & SURFACE_FLAG_DYNAMIC)) {
        gStaticSurfaceIndex.hasWater = TRUE;
    }
    aggressf((batch->count + count) * sizeof(struct Surface) <= batch->capacity, "Collision surfaces exceed pool");
    for (s32 i = 0; i < count; i++) {
        struct Surface *surface = read_surface_data(batch, vertexData, *data);
        if (rooms && *rooms) room = *(*rooms)++;
        if (surface != NULL) {
            surface->room = room;
            surface->type = surfaceType;
            surface->flags = flags;
            surface->object = object;
            surface->force = hasForce ? (*data)[3] : 0;
        }
        *data += 3 + hasForce;
    }
}

/**
 * Read the data for vertices for reference by triangles.
 */
static TerrainData *read_vertex_data(TerrainData **data) {
    s32 numVertices = *(*data)++;

    TerrainData *vertexData = *data;
    *data += 3 * numVertices;

    return vertexData;
}

/**
 * Loads in special environmental regions, such as water, poison gas, and JRB fog.
 */
static void load_environmental_regions(TerrainData **data) {
    s32 i;

    gEnvironmentRegions = *data;
    s32 numRegions = *(*data)++;

    for (i = 0; i < numRegions; i++) {
        *data += 5;
        gEnvironmentLevels[i] = *(*data)++;
    }
}

/**
 * Allocate the dynamic surface pool for object collision.
 */
void alloc_surface_pools(void) {
    sDynamicSurfacePool = main_pool_alloc(DYNAMIC_SURFACE_POOL_SIZE, MEMORY_POOL_LEFT);

    gCCMEnteredSlide = FALSE;
    reset_red_coins_collected();
}

#ifdef NO_SEGMENTED_MEMORY
/**
 * Get the size of the terrain data, to get the correct size when copying later.
 */
u32 get_area_terrain_size(TerrainData *data) {
    TerrainData *startPos = data;
    s32 end = FALSE;
    TerrainData terrainLoadType;
    s32 numVertices;
    s32 numRegions;
    s32 numSurfaces;
#ifndef ALL_SURFACES_HAVE_FORCE
    TerrainData hasForce;
#endif

    while (!end) {
        terrainLoadType = *data++;

        switch (terrainLoadType) {
            case TERRAIN_LOAD_VERTICES:
                numVertices = *data++;
                data += 3 * numVertices;
                break;

            case TERRAIN_LOAD_OBJECTS:
                data += get_special_objects_size(data);
                break;

            case TERRAIN_LOAD_ENVIRONMENT:
                numRegions = *data++;
                data += 6 * numRegions;
                break;

            case TERRAIN_LOAD_CONTINUE:
                continue;

            case TERRAIN_LOAD_END:
                end = TRUE;
                break;

            default:
                numSurfaces = *data++;
#ifdef ALL_SURFACES_HAVE_FORCE
                data += 4 * numSurfaces;
#else
                hasForce = surface_has_force(terrainLoadType);
                data += (3 + hasForce) * numSurfaces;
#endif
                break;
        }
    }

    return data - startPos;
}
#endif


/**
 * Process the level file, loading in vertices, surfaces, some objects, and environmental
 * boxes (water, gas, JRB fog).
 */
void load_area_terrain(s32 index, TerrainData *data, RoomData *surfaceRooms, s16 *macroObjects) {
    PUPPYPRINT_GET_SNAPSHOT();
    s32 terrainLoadType;
    TerrainData *vertexData = NULL;
    gEnvironmentRegions = NULL;
    gStaticSurfaceIndex = (struct SurfaceIndex){ 0 };
    gDynamicSurfaceIndex = (struct SurfaceIndex){ 0 };
    if (data == NULL) return;

    struct SurfaceBatch batch = start_static_batch();

    // A while loop iterating through each section of the level data. Sections of data
    // are prefixed by a terrain "type." This type is reused for surfaces as the surface
    // type.
    while (TRUE) {
        terrainLoadType = *data++;

        if (TERRAIN_LOAD_IS_SURFACE_TYPE_LOW(terrainLoadType) || TERRAIN_LOAD_IS_SURFACE_TYPE_HIGH(terrainLoadType)) {
            load_surfaces(&batch, &data, vertexData, terrainLoadType, &surfaceRooms, 0);
        } else if (terrainLoadType == TERRAIN_LOAD_VERTICES) {
            vertexData = read_vertex_data(&data);
        } else if (terrainLoadType == TERRAIN_LOAD_OBJECTS) {
            spawn_special_objects(index, &data);
        } else if (terrainLoadType == TERRAIN_LOAD_ENVIRONMENT) {
            load_environmental_regions(&data);
        } else if (terrainLoadType == TERRAIN_LOAD_END) {
            break;
        }
    }
    finish_static_batch(&batch);

    if (macroObjects != NULL && *macroObjects != -1) {
        // If the first macro object presetID is within the range [0, 29].
        // Generally an early spawning method, every object is in BBH (the first level).
        if (0 <= *macroObjects && *macroObjects < 30) {
            spawn_macro_objects_hardcoded(index, macroObjects);
        }
        // A more general version that can spawn more objects.
        else {
            spawn_macro_objects(index, macroObjects);
        }
    }

    profiler_collision_update(first);
}

void clear_dynamic_surfaces(void) {
    PUPPYPRINT_GET_SNAPSHOT();
    if (!(gTimeStopState & TIME_STOP_ACTIVE)) {
        clear_dynamic_surface_references();
        gDynamicSurfaceIndex = (struct SurfaceIndex){ 0 };
    }
    profiler_collision_update(first);
}

/**
 * Applies an object's transformation to the object's vertices.
 */
void transform_object_vertices(TerrainData **data, TerrainData *vertexData) {
    Mat4 transform;
    s32 numVertices = *(*data)++;
    TerrainData *vertices = *data;
    obj_build_transform(o, transform);

    // Go through all vertices, rotating and translating them to transform the object.
    Vec3f pos;
    while (numVertices--) {
        vec3s_to_vec3f(pos, vertices);
        vertices += 3;

        //! No bounds check on vertex data
        linear_mtxf_mul_vec3_and_translate(transform, vertexData, pos);

        vertexData += 3;
    }

    *data = vertices;
}

#ifdef AUTO_COLLISION_DISTANCE
static void get_optimal_coll_dist(struct Object *obj) {
    register f32 thisVertDist, maxDist = 0.0f;
    Vec3f v;
    TerrainData *collisionData = o->collisionData;
    obj->oFlags |= OBJ_FLAG_DONT_CALC_COLL_DIST;
    collisionData++;
    register u32 vertsLeft = *(collisionData)++;
    while (vertsLeft) {
        vec3_prod(v, collisionData, obj->header.gfx.scale);
        thisVertDist = vec3_sumsq(v);
        if (thisVertDist > maxDist) maxDist = thisVertDist;
        collisionData += 3;
        vertsLeft--;
    }
    obj->oCollisionDistance = (sqrtf(maxDist) + 100.0f);
}
#endif

static void load_object_tree(struct SurfaceBatch *batch, s32 flags) {
    static TerrainData vertexData[600];
    TerrainData *data = o->collisionData + 1;
    transform_object_vertices(&data, vertexData);
    while (*data != TERRAIN_LOAD_CONTINUE) {
        s32 type = *data++;
        load_surfaces(batch, &data, vertexData, type, NULL, flags);
    }
}

/**
 * Transform an object's vertices, reload them, and render the object.
 */
void load_object_collision_model(void) {
    PUPPYPRINT_GET_SNAPSHOT();
    Vec3f dist;
    vec3_diff(dist, &o->oPosVec, &gMarioObject->oPosVec);
    f32 sqrLateralDist = sqr(dist[0]) + sqr(dist[2]);

    f32 verticalMarioDiff = gMarioObject->oPosY - o->oPosY;

#ifdef AUTO_COLLISION_DISTANCE
    if (!(o->oFlags & OBJ_FLAG_DONT_CALC_COLL_DIST)) {
        get_optimal_coll_dist(o);
    }
#endif

    // If the object collision is supposed to be loaded more than the
    // drawing distance, extend the drawing range.
    if (o->oCollisionDistance > o->oDrawingDistance) {
        o->oDrawingDistance = o->oCollisionDistance;
    }
    
    s32 inColRadius = (
           (sqrLateralDist < sqr(o->oCollisionDistance))
        && (verticalMarioDiff > 0 || verticalMarioDiff > -o->oCollisionDistance)
        && (verticalMarioDiff < 0 || verticalMarioDiff < o->oCollisionDistance + 2000.f)
    );

    // Update if no Time Stop, in range, and in the current room.
    if (
        !(gTimeStopState & TIME_STOP_ACTIVE)
        && inColRadius
        && !(o->activeFlags & ACTIVE_FLAG_IN_DIFFERENT_ROOM)
    ) {
        struct SurfaceBatch batch = {
            (struct Surface *)(sDynamicSurfacePool + gDynamicSurfaceIndex.bytes), 0,
            DYNAMIC_SURFACE_POOL_SIZE - gDynamicSurfaceIndex.bytes
        };
        load_object_tree(&batch, SURFACE_FLAG_DYNAMIC);
        build_surface_tree(&gDynamicSurfaceIndex, &batch);
        o->collisionFrame = gGlobalTimer;
    }

    f32 marioDist = o->oDistanceToMario;

    // On an object's first frame, the distance is set to 19000.0f.
    // If the distance hasn't been updated, update it now.
    if (marioDist == 19000.0f) {
        marioDist = dist_between_objects(o, gMarioObject);
    }

    COND_BIT((marioDist < o->oDrawingDistance), o->header.gfx.node.flags, GRAPH_RENDER_ACTIVE);
    profiler_collision_update(first);
}

/**
 * Transform an object's vertices and add them to the static surface pool.
 */
void load_object_static_model(void) {
    PUPPYPRINT_GET_SNAPSHOT();
    struct SurfaceBatch batch = start_static_batch();
    load_object_tree(&batch, 0);
    finish_static_batch(&batch);
    profiler_collision_update(first);
}
