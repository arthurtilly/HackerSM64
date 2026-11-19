#include <PR/ultratypes.h>

#include "sm64.h"
#include "game/debug.h"
#include "game/level_update.h"
#include "game/mario.h"
#include "game/object_list_processor.h"
#include "math_util.h"
#include "surface_collision.h"
#include "surface_load.h"
#include "game/puppyprint.h"

static ALWAYS_INLINE s32 accepts_surface(const struct Surface *s, s32 flags) {
    if (flags & COLLISION_FLAG_WATER_ONLY) return s->type == SURFACE_NEW_WATER;
    if (SURFACE_IS_NEW_WATER(s->type)) return FALSE;
    if (s->type == SURFACE_INTANGIBLE && !(flags & COLLISION_FLAG_INCLUDE_INTANGIBLE)) return FALSE;
    return (flags & COLLISION_FLAG_CAMERA) ? !(s->flags & SURFACE_FLAG_NO_CAM_COLLISION)
                                          : s->type != SURFACE_CAMERA_BOUNDARY;
}

static ALWAYS_INLINE void set_contact(struct SurfaceContact *hit, struct Surface *s) {
    if (hit != NULL) {
        hit->surface = s;
        if (s != NULL) {
            hit->normal = s->normal;
            hit->type = s->type;
            hit->room = s->room;
        }
    }
}

static ALWAYS_INLINE f32 normal_dot(const struct Normal *n, const Vec3f v) {
    return n->x * v[0] + n->y * v[1] + n->z * v[2];
}

static ALWAYS_INLINE s32 inside_edge(const TerrainData *a, const TerrainData *b, s32 i, s32 j, f32 x, f32 y) {
    return (b[i] - a[i]) * (y - a[j]) >= (b[j] - a[j]) * (x - a[i]);
}

static ALWAYS_INLINE s32 inside_projection(const struct Surface *s, s32 i, s32 j, f32 x, f32 y, f32 facing) {
    const TerrainData *a = s->vertex1, *b = s->vertex2, *c = s->vertex3;
    if (facing < 0.0f) {
        b = s->vertex3;
        c = s->vertex2;
    }
    return inside_edge(a, b, i, j, x, y) && inside_edge(b, c, i, j, x, y) && inside_edge(c, a, i, j, x, y);
}

static s32 inside_triangle_3d(const struct Surface *s, const Vec3f p) {
    const struct Normal *n = &s->normal;
    if (absf(n->x) > absf(n->y) && absf(n->x) > absf(n->z)) {
        return inside_projection(s, 1, 2, p[1], p[2], n->x);
    }
    if (absf(n->z) > absf(n->y)) {
        return inside_projection(s, 0, 1, p[0], p[1], n->z);
    }
    return inside_projection(s, 2, 0, p[2], p[0], n->y);
}

// A vertical ray only needs an XZ point-in-triangle test. Flat faces also avoid a divide.
static ALWAYS_INLINE f32 vertical_ray(f32 x, f32 y, f32 z, f32 end, s32 upward, f32 minFacing,
                                      s32 flags, struct Surface **source) {
    struct SurfaceBounds bounds = { { x, MIN(y, end), z }, { x, MAX(y, end), z } };
    f32 best = end;
    *source = NULL;
    for (s32 dynamic = !(flags & COLLISION_FLAG_EXCLUDE_DYNAMIC); dynamic >= 0; dynamic--) {
        struct SurfaceIterator it = { .tree = dynamic ? gDynamicSurfaceIndex.first : gStaticSurfaceIndex.first };
        while (next_surface_leaf(&it, &bounds)) {
            for (struct Surface *s = it.surface; s < it.end; s++) {
                if (s->lowerY > bounds.max[1] || s->upperY < bounds.min[1]) continue;
                if ((upward ? -s->normal.y : s->normal.y) <= minFacing || !accepts_surface(s, flags)) continue;
                if (!inside_projection(s, 2, 0, z, x, upward ? -1.0f : 1.0f)) continue;
                f32 h = s->lowerY == s->upperY ? s->lowerY : get_surface_height_at_location(x, z, s);
                if (upward ? (h < y || h > best) : (h > y || h < best)) continue;
                if (*source != NULL && h == best) continue;
                *source = s;
                best = h;
                if (upward) bounds.max[1] = h;
                else bounds.min[1] = h;
            }
        }
    }
    return best;
}

static s32 ray_intersects_node(const struct SurfaceBvhNode *node, const Vec3f origin, const Vec3f invDir, f32 end) {
    f32 start = 0.0f;
    for (s32 i = 0; i < 3; i++) {
        if (invDir[i] == 0.0f) {
            if (origin[i] < node->min[i] || origin[i] > node->max[i]) return FALSE;
        } else {
            f32 a = (node->min[i] - origin[i]) * invDir[i];
            f32 b = (node->max[i] - origin[i]) * invDir[i];
            start = MAX(start, MIN(a, b));
            end = MIN(end, MAX(a, b));
            if (start > end) return FALSE;
        }
    }
    return TRUE;
}

f32 collision_raycast(const Vec3f origin, const Vec3f dir, f32 length, f32 minFacing,
                      s32 flags, struct SurfaceContact *hit) {
    PUPPYPRINT_ADD_COUNTER(gPuppyCallCounter.collision_raycast);
    PUPPYPRINT_GET_SNAPSHOT();
    struct Surface *nearest = NULL;
    if (length < 0.0f) goto done;
    if (dir[0] == 0.0f && dir[2] == 0.0f) {
        if (dir[1] == 0.0f) goto done;
        f32 h = vertical_ray(origin[0], origin[1], origin[2], origin[1] + dir[1] * length,
                             dir[1] > 0.0f, minFacing, flags, &nearest);
        length = absf(h - origin[1]);
    } else {
        Vec3f invDir;
        for (s32 i = 0; i < 3; i++) invDir[i] = dir[i] == 0.0f ? 0.0f : 1.0f / dir[i];
        for (s32 dynamic = !(flags & COLLISION_FLAG_EXCLUDE_DYNAMIC); dynamic >= 0; dynamic--) {
            struct SurfaceTree *tree = dynamic ? gDynamicSurfaceIndex.first : gStaticSurfaceIndex.first;
            for (; tree != NULL; tree = tree->next) {
                for (u32 i = 0; i < tree->numNodes;) {
                    struct SurfaceBvhNode *node = &tree->nodes[i++];
                    if (!ray_intersects_node(node, origin, invDir, length)) {
                        if (!node->count) i = node->offset;
                        continue;
                    }
                    for (u32 j = 0; j < node->count; j++) {
                        struct Surface *s = &tree->surfaces[node->offset + j];
                        f32 facing = -normal_dot(&s->normal, dir);
                        if (facing <= minFacing || !accepts_surface(s, flags)) continue;
                        f32 d = normal_dot(&s->normal, origin) + s->originOffset;
                        if (d < 0.0f || d > facing * length) continue;
                        d /= facing;
                        Vec3f p = { origin[0] + dir[0] * d, origin[1] + dir[1] * d, origin[2] + dir[2] * d };
                        if (!inside_triangle_3d(s, p)) continue;
                        length = d;
                        nearest = s;
                    }
                }
            }
        }
    }
done:
    set_contact(hit, nearest);
    profiler_collision_update(first);
    return nearest != NULL ? length : -1.0f;
}

static f32 cast_floor(f32 x, f32 y, f32 z, s32 flags, struct Surface **source) {
    PUPPYPRINT_ADD_COUNTER(gPuppyCallCounter.collision_floor);
    PUPPYPRINT_GET_SNAPSHOT();
    f32 h = FLOOR_LOWER_LIMIT;
    *source = NULL;
    y += FIND_FLOOR_BUFFER;
    if (!is_outside_level_bounds(x, z) && y >= h) {
        // A steep face can receive a floor ray and still push a wall sphere sideways.
        h = vertical_ray(x, y, z, h, FALSE, 0.0f, flags, source);
        if (*source == NULL) gNumFindFloorMisses++;
    }
#ifdef VANILLA_DEBUG
    gNumCalls.floor++;
#endif
    profiler_collision_update(first);
    return h;
}

f32 find_floor_height(f32 x, f32 y, f32 z) {
    struct Surface *source;
    return cast_floor(x, y, z, gCollisionFlags, &source);
}

f32 find_floor(f32 x, f32 y, f32 z, struct Surface **floor) {
    return cast_floor(x, y, z, gCollisionFlags, floor);
}

f32 find_actor_floor(const struct Object *obj, const Vec3f pos, f32 offset, struct SurfaceContact *hit) {
    if (obj->gravityDir[1] != -1.0f) {
        Vec3f origin;
        vec3_copy(origin, pos);
        obj_add_up(obj, origin, offset + FIND_FLOOR_BUFFER);
        f32 distance = collision_raycast(origin, obj->gravityDir, LEVEL_RAY_LENGTH, 0.0f, gCollisionFlags, hit);
        return distance < 0.0f ? FLOOR_LOWER_LIMIT : obj_height(obj, origin) - distance;
    }
    struct Surface *source;
    f32 height = cast_floor(pos[0], pos[1] + offset, pos[2], gCollisionFlags, &source);
    set_contact(hit, source);
    return height;
}

static f32 cast_ceil(f32 x, f32 y, f32 z, f32 end, struct Surface **source) {
    PUPPYPRINT_ADD_COUNTER(gPuppyCallCounter.collision_ceil);
    PUPPYPRINT_GET_SNAPSHOT();
    s32 flags = gCollisionFlags;
    f32 h = CELL_HEIGHT_LIMIT;
    *source = NULL;
    if (!is_outside_level_bounds(x, z) && end >= y) {
        h = vertical_ray(x, y, z, end, TRUE, WALL_MAX_UP, flags, source);
    }
#ifdef VANILLA_DEBUG
    gNumCalls.ceil++;
#endif
    profiler_collision_update(first);
    return *source != NULL ? h : CELL_HEIGHT_LIMIT;
}

f32 find_ceil(f32 x, f32 y, f32 z, struct Surface **ceil) {
    return cast_ceil(x, y, z, CELL_HEIGHT_LIMIT, ceil);
}

f32 find_actor_ceil(const struct Object *obj, const Vec3f pos, f32 floorHeight, f32 clearance, struct SurfaceContact *hit) {
    f32 heightHere = obj_height(obj, pos);
    f32 base = floorHeight == FLOOR_LOWER_LIMIT ? heightHere : MAX(heightHere, floorHeight);
    if (obj->gravityDir[1] != -1.0f) {
        Vec3f origin, up;
        vec3_copy(origin, pos);
        obj_add_up(obj, origin, base + 3.0f - heightHere);
        vec3_scale_dest(up, obj->gravityDir, -1.0f);
        f32 distance = collision_raycast(origin, up, clearance - 3.0f, WALL_MAX_UP, gCollisionFlags, hit);
        return distance < 0.0f ? CELL_HEIGHT_LIMIT : base + 3.0f + distance;
    }
    struct Surface *source;
    f32 height = cast_ceil(pos[0], base + 3.0f, pos[2], base + clearance, &source);
    set_contact(hit, source);
    return height;
}

// Return center minus the closest point on an edge or vertex. An interior
// projection uses the cached face plane and needs no barycentric division.
static s32 sphere_edge_delta(const struct Surface *s, const Vec3f center, Vec3f delta) {
    Vec3f ab, ac, ap;
    vec3_diff(ab, s->vertex2, s->vertex1);
    vec3_diff(ac, s->vertex3, s->vertex1);
    vec3_diff(ap, center, s->vertex1);
    f32 d1 = vec3_dot(ab, ap), d2 = vec3_dot(ac, ap);
    if (d1 <= 0.0f && d2 <= 0.0f) {
        vec3_copy(delta, ap);
        return TRUE;
    }

    Vec3f bp;
    vec3_diff(bp, center, s->vertex2);
    f32 d3 = vec3_dot(ab, bp), d4 = vec3_dot(ac, bp);
    if (d3 >= 0.0f && d4 <= d3) {
        vec3_copy(delta, bp);
        return TRUE;
    }
    if (d1 >= 0.0f && d3 <= 0.0f && d1 * d4 < d3 * d2) {
        f32 t = d1 / (d1 - d3);
        for (s32 i = 0; i < 3; i++) delta[i] = ap[i] - t * ab[i];
        return TRUE;
    }

    Vec3f cp;
    vec3_diff(cp, center, s->vertex3);
    f32 d5 = vec3_dot(ab, cp), d6 = vec3_dot(ac, cp);
    if (d6 >= 0.0f && d5 <= d6) {
        vec3_copy(delta, cp);
        return TRUE;
    }
    if (d2 >= 0.0f && d6 <= 0.0f && d5 * d2 < d1 * d6) {
        f32 t = d2 / (d2 - d6);
        for (s32 i = 0; i < 3; i++) delta[i] = ap[i] - t * ac[i];
        return TRUE;
    }
    if (d4 >= d3 && d5 >= d6 && d3 * d6 < d5 * d4) {
        f32 t = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        for (s32 i = 0; i < 3; i++) delta[i] = bp[i] - t * (ac[i] - ab[i]);
        return TRUE;
    }
    return FALSE;
}

static s32 can_pass_grates(s32 flags) {
    return !(flags & COLLISION_FLAG_CAMERA) && o != NULL
        && ((o->activeFlags & ACTIVE_FLAG_MOVE_THROUGH_GRATE) || (o == gMarioObject && (gMarioState->flags & MARIO_VANISH_CAP)));
}

// The face plane gives the earliest possible impact. Outside the triangle,
// advance to the tangent plane of its nearest edge or vertex and try again.
static void sweep_wall_sphere(struct Surface *s, const Vec3f center, const Vec3f move, f32 radius,
                              f32 distance, f32 approach, struct SweepHit *hit) {
    if (distance < 0.0f || distance - MAX(approach, 0.0f) * hit->time > radius) return;
    f32 t = distance > radius ? (distance - radius) / approach : 0.0f;
    for (s32 i = 0; i < 8 && t <= hit->time; i++) {
        Vec3f p, normal;
        for (s32 axis = 0; axis < 3; axis++) p[axis] = center[axis] + move[axis] * t;
        f32 gap = distance - approach * t;
        if (gap < -0.01f) return;
        f32 closing = approach;
        if (sphere_edge_delta(s, p, normal) && vec3_sumsq(normal) > 0.000001f) {
            gap = vec3_mag(normal);
            vec3_scale(normal, 1.0f / gap);
            closing = -vec3_dot(normal, move);
        } else {
            vec3_set(normal, s->normal.x, s->normal.y, s->normal.z);
        }
        gap -= radius;
        if (closing <= 0.0001f && gap >= -0.01f) return;
        if (gap <= 0.01f || i == 7) {
            hit->face.surface = s;
            vec3_copy(hit->normal, normal);
            hit->depth = MAX(-gap, 0.0f);
            hit->time = t;
            return;
        }
        t += gap / closing;
    }
}

s32 sweep_actor(const Vec3f pos, const Vec3f move, const Vec3f up, const struct ActorShape *shape,
                struct SweepHit *hit) {
    PUPPYPRINT_ADD_COUNTER(gPuppyCallCounter.collision_wall);
    PUPPYPRINT_GET_SNAPSHOT();
    f32 radius = shape->radius;
    Vec3f lower, upper, head;
    struct SurfaceBounds bounds;
    for (s32 i = 0; i < 3; i++) {
        lower[i] = pos[i] + up[i] * shape->lowerOffset;
        upper[i] = pos[i] + up[i] * shape->upperOffset;
        head[i] = pos[i] + up[i] * shape->height;
        bounds.min[i] = MIN(MIN(lower[i], upper[i]) - radius, MIN(pos[i], head[i])) + MIN(move[i], 0.0f);
        bounds.max[i] = MAX(MAX(lower[i], upper[i]) + radius, MAX(pos[i], head[i])) + MAX(move[i], 0.0f);
    }

    hit->time = 1.0f;
    hit->face.surface = NULL;
    s32 flags = gCollisionFlags;
    s32 passGrates = can_pass_grates(flags);
    s32 worldUp = up[1] == 1.0f;
    for (s32 dynamic = !(flags & COLLISION_FLAG_EXCLUDE_DYNAMIC); dynamic >= 0; dynamic--) {
        struct SurfaceIterator it = { .tree = dynamic ? gDynamicSurfaceIndex.first : gStaticSurfaceIndex.first };
        while (next_surface_leaf(&it, &bounds)) {
            for (struct Surface *s = it.surface; s < it.end; s++) {
                if (s->lowerY > bounds.max[1] || s->upperY < bounds.min[1] || !accepts_surface(s, flags)) continue;
                f32 normalUp = worldUp ? s->normal.y : normal_dot(&s->normal, up);
                f32 approach = -normal_dot(&s->normal, move);
                if (absf(normalUp) <= WALL_MAX_UP) {
                    if (passGrates && s->type == SURFACE_VANISH_CAP_WALLS) continue;
                    f32 d = normal_dot(&s->normal, pos) + s->originOffset;
                    sweep_wall_sphere(s, lower, move, radius, d + shape->lowerOffset * normalUp, approach, hit);
                    if (shape->upperOffset != shape->lowerOffset) {
                        sweep_wall_sphere(s, upper, move, radius, d + shape->upperOffset * normalUp, approach, hit);
                    }
                } else if (approach > 0.0001f) {
                    const f32 *origin = normalUp > 0.0f ? pos : head;
                    f32 d = normal_dot(&s->normal, origin) + s->originOffset;
                    if (d < -0.01f || d > approach * hit->time) continue;
                    f32 t = MAX(d, 0.0f) / approach;
                    Vec3f p;
                    for (s32 i = 0; i < 3; i++) p[i] = origin[i] + move[i] * t;
                    if (!inside_triangle_3d(s, p)) continue;
                    hit->face.surface = s;
                    vec3_set(hit->normal, s->normal.x, s->normal.y, s->normal.z);
                    hit->time = t;
                    hit->depth = 0.0f;
                }
            }
        }
    }
    set_contact(&hit->face, hit->face.surface);
    profiler_collision_update(first);
    return hit->face.surface != NULL;
}

enum SphereContact { SPHERE_MISS, SPHERE_PUSH, SPHERE_CONTACT };

static s32 push_sphere(Vec3f pos, const Vec3f up, const struct WallSphere *sphere,
                       const struct Surface *s, f32 normalUp, f32 distance, f32 maxContactUp) {
    f32 radius = sphere->radius;
    if (distance <= -radius || distance >= radius) return SPHERE_MISS;
    Vec3f center, lateral;
    for (s32 i = 0; i < 3; i++) center[i] = pos[i] + up[i] * sphere->offset;
    if (center[1] + radius < s->lowerY || center[1] - radius > s->upperY) return SPHERE_MISS;

    s32 report = absf(normalUp) <= maxContactUp;
    f32 scale;
    if (sphere_edge_delta(s, center, lateral) && vec3_sumsq(lateral) > 0.000001f) {
        f32 d2 = vec3_sumsq(lateral);
        if (distance < 0.0f || d2 >= radius * radius) return SPHERE_MISS;
        f32 height = vec3_dot(lateral, up);
        if (height * height > maxContactUp * maxContactUp * d2) report = FALSE;
        for (s32 i = 0; i < 3; i++) lateral[i] -= height * up[i];
        f32 lateralSq = vec3_sumsq(lateral);
        if (lateralSq < 0.0001f) return SPHERE_MISS;
        scale = sqrtf((radius * radius - height * height) / lateralSq) - 1.0f;
    } else {
        lateral[0] = s->normal.x - up[0] * normalUp;
        lateral[1] = s->normal.y - up[1] * normalUp;
        lateral[2] = s->normal.z - up[2] * normalUp;
        scale = (radius - distance) / (1.0f - normalUp * normalUp);
    }
    for (s32 i = 0; i < 3; i++) pos[i] += lateral[i] * scale;
    return report ? SPHERE_CONTACT : SPHERE_PUSH;
}

static ALWAYS_INLINE void sphere_bounds(struct SurfaceBounds *bounds, const Vec3f pos, const Vec3f lo, const Vec3f hi) {
    vec3_sum(bounds->min, pos, lo);
    vec3_sum(bounds->max, pos, hi);
}

s32 resolve_wall_spheres(Vec3f pos, const Vec3f up, const struct WallSphere *spheres, s32 count,
                         f32 maxContactUp, struct SurfaceContact *hits) {
    PUPPYPRINT_ADD_COUNTER(gPuppyCallCounter.collision_wall);
    PUPPYPRINT_GET_SNAPSHOT();
    s32 flags = gCollisionFlags;
    s32 reported = FALSE;
    s32 worldUp = up[0] == 0.0f && up[1] == 1.0f && up[2] == 0.0f;
    Vec3f lo, hi;
    for (s32 j = 0; j < count; j++) {
        if (hits != NULL) hits[j].surface = NULL;
        for (s32 axis = 0; axis < 3; axis++) {
            f32 offset = up[axis] * spheres[j].offset;
            f32 a = offset - spheres[j].radius, b = offset + spheres[j].radius;
            if (j == 0 || a < lo[axis]) lo[axis] = a;
            if (j == 0 || b > hi[axis]) hi[axis] = b;
        }
    }
    if (count <= 0) goto done;
    struct SurfaceBounds bounds;
    sphere_bounds(&bounds, pos, lo, hi);
    s32 passGrates = can_pass_grates(flags);
    for (s32 dynamic = !(flags & COLLISION_FLAG_EXCLUDE_DYNAMIC); dynamic >= 0; dynamic--) {
        struct SurfaceIterator it = { .tree = dynamic ? gDynamicSurfaceIndex.first : gStaticSurfaceIndex.first };
        while (next_surface_leaf(&it, &bounds)) {
            for (struct Surface *s = it.surface; s < it.end; s++) {
                if (s->lowerY > bounds.max[1] || s->upperY < bounds.min[1]) continue;
                f32 normalUp = worldUp ? s->normal.y : normal_dot(&s->normal, up);
                if (absf(normalUp) > WALL_MAX_UP || !accepts_surface(s, flags)) continue;
                if (passGrates && s->type == SURFACE_VANISH_CAP_WALLS) continue;
                f32 distance = normal_dot(&s->normal, pos) + s->originOffset;
                for (s32 j = 0; j < count; j++) {
                    s32 contact = push_sphere(pos, up, &spheres[j], s, normalUp, distance + spheres[j].offset * normalUp,
                                              maxContactUp);
                    if (contact == SPHERE_MISS) continue;
                    sphere_bounds(&bounds, pos, lo, hi);
                    if (contact == SPHERE_CONTACT) {
                        reported = TRUE;
                        if (hits != NULL && (hits[j].surface == NULL || s->type == SURFACE_BURNING)) hits[j].surface = s;
                    }
                    if (j + 1 < count) distance = normal_dot(&s->normal, pos) + s->originOffset;
                }
            }
        }
    }
done:
    if (hits != NULL) {
        for (s32 j = 0; j < count; j++) set_contact(&hits[j], hits[j].surface);
    }
#ifdef VANILLA_DEBUG
    gNumCalls.wall++;
#endif
    profiler_collision_update(first);
    return reported;
}

s32 f32_find_wall_collision(f32 *x, f32 *y, f32 *z, f32 offset, f32 radius) {
    Vec3f pos = { *x, *y, *z };
    s32 hit = resolve_wall_sphere(pos, gVec3fY, offset, radius, 1.0f, NULL);
    *x = pos[0];
    *y = pos[1];
    *z = pos[2];
    return hit;
}

f32 find_room_floor(f32 x, f32 y, f32 z, struct Surface **pfloor) {
    return cast_floor(x, y, z, gCollisionFlags | COLLISION_FLAG_EXCLUDE_DYNAMIC | COLLISION_FLAG_INCLUDE_INTANGIBLE, pfloor);
}

/**
 * Get the room index at a given position.
 */
s32 get_room_at_pos(f32 x, f32 y, f32 z) {
    if (gCurrentArea->surfaceRooms != NULL) {
        struct Surface *floor;

        find_room_floor(x, y, z, &floor);

        if (floor != NULL) {
            return floor->room;
        }
    }

    return -1;
}

/**
 * Find the highest water floor under a given position and return the height.
 */
static f32 find_water_floor(s32 x, s32 y, s32 z) {
    if (!gStaticSurfaceIndex.hasWater || is_outside_level_bounds(x, z)) return FLOOR_LOWER_LIMIT;

    struct SurfaceBounds bounds = { { x, y + FIND_FLOOR_BUFFER, z }, { x, S32_MAX, z } };
    struct SurfaceIterator it = { .tree = gStaticSurfaceIndex.first };
    struct Surface *surf;
    f32 limit = (f32)S32_MAX;
    while ((surf = next_surface(&it, &bounds)) != NULL) {
        if (surf->type != SURFACE_NEW_WATER_BOTTOM || absf(surf->normal.y) < 0.01f) continue;
        if (!inside_projection(surf, 2, 0, z, x, surf->normal.y)) continue;
        f32 height = get_surface_height_at_location(x, z, surf);
        if (height < bounds.min[1] || height >= limit) continue;
        limit = height;
        bounds.max[1] = height;
    }

    f32 top = FLOOR_LOWER_LIMIT;
    bounds.min[1] = FLOOR_LOWER_LIMIT;
    it = (struct SurfaceIterator){ .tree = gStaticSurfaceIndex.first };
    while ((surf = next_surface(&it, &bounds)) != NULL) {
        if (surf->type != SURFACE_NEW_WATER || absf(surf->normal.y) < 0.01f) continue;
        if (!inside_projection(surf, 2, 0, z, x, surf->normal.y)) continue;
        f32 height = get_surface_height_at_location(x, z, surf);
        if (height > limit || height <= top) continue;
        top = height;
        bounds.min[1] = top;
    }
#ifdef VANILLA_DEBUG
    gNumCalls.floor++;
#endif
    return top;
}

/**************************************************
 *               ENVIRONMENTAL BOXES              *
 **************************************************/

// Return the distance to the nearest water surface facing the ray, or -1 on a miss.
f32 raycast_water_surface(const Vec3f origin, const Vec3f dir, f32 length, Vec3f normal) {
    if (length < 0.0f) return -1.0f;
    PUPPYPRINT_ADD_COUNTER(gPuppyCallCounter.collision_water);
    PUPPYPRINT_GET_SNAPSHOT();
    f32 best = -1.0f;
    if (gStaticSurfaceIndex.hasWater) {
        struct SurfaceContact hit;
        best = collision_raycast(origin, dir, length, 0.0f,
                                 COLLISION_FLAG_WATER_ONLY | COLLISION_FLAG_EXCLUDE_DYNAMIC, &hit);
        if (hit.surface != NULL) {
            vec3_set(normal, hit.normal.x, hit.normal.y, hit.normal.z);
            length = best;
        }
    }
    if (gEnvironmentRegions != NULL && dir[1] < 0.0f) {
        const TerrainData *box = gEnvironmentRegions + 1;
        f32 invY = -1.0f / dir[1];
        for (s32 i = 0; i < gEnvironmentRegions[0]; i++, box += 6) {
            if (box[0] >= 50) continue;
            f32 distance = (origin[1] - box[5]) * invY;
            if (distance < 0.0f || distance > length) continue;
            f32 x = origin[0] + dir[0] * distance;
            f32 z = origin[2] + dir[2] * distance;
            if (x < box[1] || x > box[3] || z < box[2] || z > box[4]) continue;
            best = length = distance;
            vec3_copy(normal, gVec3fY);
        }
    }
    profiler_collision_update(first);
    return best;
}

/**
 * Finds the height of water at a given location.
 */
s32 find_water_level(s32 x, s32 z) { // HACKERSM64_DO: Allow y pos
    s32 val;
    s32 loX, hiX, loZ, hiZ;
    TerrainData *p = gEnvironmentRegions;
    PUPPYPRINT_ADD_COUNTER(gPuppyCallCounter.collision_water);
    PUPPYPRINT_GET_SNAPSHOT();
    s32 waterLevel = find_water_floor(x, (gCollisionFlags & COLLISION_FLAG_CAMERA) ? gLakituState.pos[1] : gMarioState->pos[1], z);

    if ((p != NULL) && (waterLevel == FLOOR_LOWER_LIMIT)) {
        s32 numRegions = *p++;

        for (s32 i = 0; i < numRegions; i++) {
            val = *p++;
            loX = *p++;
            loZ = *p++;
            hiX = *p++;
            hiZ = *p++;

            // If the location is within a water box and it is a water box.
            // Water is less than 50 val only, while above is gas and such.
            if (loX <= x && x <= hiX && loZ <= z && z <= hiZ && val < 50) {
                // Set the water height. Since this breaks, only return the first height.
                waterLevel = *p;
                break;
            }
            p++;
        }
    }

    profiler_collision_update(first);

    return waterLevel;
}

/**
 * Finds the height of the poison gas (used only in HMC) at a given location.
 */
s32 find_poison_gas_level(s32 x, s32 z) {
    s32 val;
    s32 loX, hiX, loZ, hiZ;
    s32 gasLevel = FLOOR_LOWER_LIMIT;
    TerrainData *p = gEnvironmentRegions;
    PUPPYPRINT_ADD_COUNTER(gPuppyCallCounter.collision_water);
    PUPPYPRINT_GET_SNAPSHOT();

    if (p != NULL) {
        s32 numRegions = *p++;

        for (s32 i = 0; i < numRegions; i++) {
            val = *p;

            if (val >= 50) {
                loX = p[1];
                loZ = p[2];
                hiX = p[3];
                hiZ = p[4];

                // If the location is within a gas's box and it is a gas box.
                // Gas has a value of 50, 60, etc.
                if (loX < x && x < hiX && loZ < z && z < hiZ && val % 10 == 0) {
                    // Set the gas height. Since this breaks, only return the first height.
                    gasLevel = p[5];
                    break;
                }
            }

            p += 6;
        }
    }
    
    profiler_collision_update(first);
    return gasLevel;
}

/**************************************************
 *                      DEBUG                     *
 **************************************************/

#ifdef VANILLA_DEBUG
void debug_surface_info(void) {
    print_debug_top_down_mapinfo("bvh nodes %d", gStaticSurfaceIndex.numNodes + gDynamicSurfaceIndex.numNodes);

    set_text_array_x_y(80, -3);

    print_debug_top_down_mapinfo("%d", gNumCalls.floor);
    print_debug_top_down_mapinfo("%d", gNumCalls.wall);
    print_debug_top_down_mapinfo("%d", gNumCalls.ceil);

    set_text_array_x_y(-80, 0);

    print_debug_top_down_mapinfo("statbg %d", gStaticSurfaceIndex.numSurfaces);
    print_debug_top_down_mapinfo("movebg %d", gDynamicSurfaceIndex.numSurfaces);

    gNumCalls.floor = 0;
    gNumCalls.ceil = 0;
    gNumCalls.wall = 0;
}
#endif
