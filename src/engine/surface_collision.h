#ifndef SURFACE_COLLISION_H
#define SURFACE_COLLISION_H

#include <PR/ultratypes.h>

#include "types.h"

#include "config/config_world.h"

// The y coord is moved upward by this amount when finding floors.
// Vanilla value is 78.
#define FIND_FLOOR_BUFFER 78

#define CELL_HEIGHT_LIMIT            20000
#define FLOOR_LOWER_LIMIT           -11000
#define FLOOR_LOWER_LIMIT_MISC      (FLOOR_LOWER_LIMIT + 1000)
#define LEVEL_RAY_LENGTH            (4.0f * LEVEL_BOUNDARY_MAX + 2.0f * CELL_HEIGHT_LIMIT)

#define is_outside_level_bounds(xPos, zPos) \
    (((xPos) <= -LEVEL_BOUNDARY_MAX) ||     \
     ((xPos) >=  LEVEL_BOUNDARY_MAX) ||     \
     ((zPos) <= -LEVEL_BOUNDARY_MAX) ||     \
     ((zPos) >=  LEVEL_BOUNDARY_MAX))

#define get_surface_height_at_location(xPos, zPos, surf) (-(((xPos) * (surf)->normal.x) + ((zPos) * (surf)->normal.z) + (surf)->originOffset) / (surf)->normal.y)

#define SURFACE_YAW(s) (atan2s(((s)->normal.z), ((s)->normal.x)))

enum CollisionFlags {
    COLLISION_FLAGS_NONE              = (0 << 0),
    COLLISION_FLAG_CAMERA             = (1 << 2),
    COLLISION_FLAG_INCLUDE_INTANGIBLE = (1 << 3),
    COLLISION_FLAG_EXCLUDE_DYNAMIC    = (1 << 4),
    COLLISION_FLAG_WATER_ONLY         = (1 << 5),
};

// On a miss surface is NULL. The remaining fields describe a successful hit.
// The source triangle gives platform and camera code access to ownership and geometry.
struct SurfaceContact {
    struct Normal normal;
    TerrainData type;
    RoomData room;
    struct Surface *surface;
};

struct WallSphere {
    f32 offset; // Along the actor's unit up vector, measured from its position.
    f32 radius;
};

// Limits on the absolute dot product of the face normal and the actor's up vector.
#define WALL_MAX_UP      0.30f
#define WALL_KICK_MAX_UP 0.01f

struct ActorShape {
    f32 radius, lowerOffset, upperOffset; // Wall spheres, measured along up from the feet.
    f32 height;
};

struct SweepHit {
    struct SurfaceContact face;
    Vec3f normal; // At an edge this points from the edge toward the sphere.
    f32 time;     // Fraction of the displacement before impact.
    f32 depth;    // Initial overlap.
};

// Cast the wall spheres and the foot and head points along move. up must be a unit vector.
s32 sweep_actor(const Vec3f pos, const Vec3f move, const Vec3f up, const struct ActorShape *shape,
                struct SweepHit *hit);

// Returns travel along the unit direction, including both endpoints, or -1 on a miss.
// minFacing is the minimum dot product between the face normal and the reverse ray.
f32 collision_raycast(const Vec3f origin, const Vec3f direction, f32 length, f32 minFacing,
                      s32 flags, struct SurfaceContact *hit);
f32 find_actor_floor(const struct Object *obj, const Vec3f pos, f32 offset, struct SurfaceContact *hit);
f32 find_actor_ceil(const struct Object *obj, const Vec3f pos, f32 floorHeight, f32 clearance, struct SurfaceContact *hit);

// Steep faces push the actor. maxContactUp tests both the face and the direction
// of edge contact. Use 1 to report every push or WALL_KICK_MAX_UP for a vertical wall.
// hits may be NULL or an array of count contacts. Corrections stay perpendicular to up.
s32 resolve_wall_spheres(Vec3f pos, const Vec3f up, const struct WallSphere *spheres, s32 count,
                         f32 maxContactUp, struct SurfaceContact *hits);
static ALWAYS_INLINE s32 resolve_wall_sphere(Vec3f pos, const Vec3f up, f32 offset, f32 radius,
                                             f32 maxContactUp, struct SurfaceContact *hit) {
    const struct WallSphere sphere = { offset, radius };
    return resolve_wall_spheres(pos, up, &sphere, 1, maxContactUp, hit);
}

static ALWAYS_INLINE s32 resolve_actor_wall(const struct Object *obj, Vec3f pos, f32 offset, f32 radius,
                                           f32 maxContactUp, struct SurfaceContact *hit) {
    Vec3f up = { -obj->gravityDir[0], -obj->gravityDir[1], -obj->gravityDir[2] };
    return resolve_wall_sphere(pos, up, offset, radius, maxContactUp, hit);
}

s32 f32_find_wall_collision(f32 *xPtr, f32 *yPtr, f32 *zPtr, f32 offsetY, f32 radius);
f32 find_ceil(f32 posX, f32 posY, f32 posZ, struct Surface **pceil);

f32 find_floor_height(f32 x, f32 y, f32 z);
f32 find_floor(f32 xPos, f32 yPos, f32 zPos, struct Surface **pfloor);
f32 find_room_floor(f32 x, f32 y, f32 z, struct Surface **pfloor);
s32 get_room_at_pos(f32 x, f32 y, f32 z);
f32 raycast_water_surface(const Vec3f origin, const Vec3f dir, f32 length, Vec3f normal);
s32 find_water_level(s32 x, s32 z);
s32 find_poison_gas_level(s32 x, s32 z);
#ifdef VANILLA_DEBUG
void debug_surface_info(void);
#endif

#endif // SURFACE_COLLISION_H
