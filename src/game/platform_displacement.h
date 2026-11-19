#ifndef PLATFORM_DISPLACEMENT_H
#define PLATFORM_DISPLACEMENT_H

#include <PR/ultratypes.h>

#include "types.h"

#include "config.h"

struct PlatformDisplacementInfo {
    Vec3f prevPos;
    Vec3f prevTransformedPos;
    Vec3f prevTransformedYawVec;
    s16 prevYaw;
    struct Object *prevPlatform;
    u32 prevTimer;
};

void update_mario_platform(void);
void apply_platform_displacement(struct PlatformDisplacementInfo *displaceInfo, struct Object *rider, Vec3f pos, s16 *yaw);
void apply_mario_platform_displacement(void);
void clear_mario_platform(void);

#endif // PLATFORM_DISPLACEMENT_H
