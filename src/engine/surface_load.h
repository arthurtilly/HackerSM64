#ifndef SURFACE_LOAD_H
#define SURFACE_LOAD_H

#include <PR/ultratypes.h>

#include "surface_collision.h"
#include "types.h"

/**
 * The size of the dynamic surface pool, in bytes.
 */
#define DYNAMIC_SURFACE_POOL_SIZE 0x8000

// Nodes are in depth-first order. An interior node skips its entire subtree
// on a miss. A leaf indexes a run of triangles in the same block.
struct SurfaceBvhNode {
    Vec3t min, max;
    u16 offset;
    u16 count;
};

struct SurfaceTree {
    struct SurfaceTree *next;
    struct Surface *surfaces;
    u32 numNodes;
    struct SurfaceBvhNode nodes[];
};

struct SurfaceIndex {
    struct SurfaceTree *first;
    u32 bytes, numSurfaces, numNodes;
    s32 hasWater;
};

struct SurfaceBounds {
    Vec3i min, max;
};

struct SurfaceIterator {
    struct SurfaceTree *tree;
    u32 node;
    struct Surface *surface, *end;
};

extern struct SurfaceIndex gStaticSurfaceIndex, gDynamicSurfaceIndex;

static ALWAYS_INLINE s32 next_surface_leaf(struct SurfaceIterator *it, const struct SurfaceBounds *bounds) {
    while (it->tree != NULL) {
        while (it->node < it->tree->numNodes) {
            struct SurfaceBvhNode *node = &it->tree->nodes[it->node++];
            if (node->max[0] < bounds->min[0] || node->min[0] > bounds->max[0]
             || node->max[2] < bounds->min[2] || node->min[2] > bounds->max[2]
             || node->max[1] < bounds->min[1] || node->min[1] > bounds->max[1]) {
                if (node->count == 0) it->node = node->offset;
            } else if (node->count != 0) {
                it->surface = it->tree->surfaces + node->offset;
                it->end = it->surface + node->count;
                return TRUE;
            }
        }
        it->tree = it->tree->next;
        it->node = 0;
    }
    return FALSE;
}

static ALWAYS_INLINE struct Surface *next_surface(struct SurfaceIterator *it, const struct SurfaceBounds *bounds) {
    if (it->surface == it->end && !next_surface_leaf(it, bounds)) {
        return NULL;
    }
    return it->surface++;
}

void alloc_surface_pools(void);
#ifdef NO_SEGMENTED_MEMORY
u32 get_area_terrain_size(TerrainData *data);
#endif
void load_area_terrain(s32 index, TerrainData *data, RoomData *surfaceRooms, MacroObject *macroObjects);
void clear_dynamic_surfaces(void);
void load_object_collision_model(void);
void load_object_static_model(void);

#endif // SURFACE_LOAD_H
