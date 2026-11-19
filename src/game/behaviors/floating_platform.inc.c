// floating_platform.inc.c

static f32 floating_platform_height(void) {
    struct SurfaceContact floor;
    f32 height = find_actor_floor(o, &o->oPosVec, 0.0f, &floor);
    f32 water = find_water_level(o->oPosX, o->oPosZ);
    f32 upY = -o->gravityDir[1];
    if (water > FLOOR_LOWER_LIMIT && upY > WALL_MAX_UP) {
        f32 waterHeight = obj_height(o, &o->oPosVec) + (water - o->oPosY) / upY;
        if (floor.surface == NULL || waterHeight > height + o->oFloatingPlatformHeightOffset) {
            o->oFloatingPlatformIsOnFloor = FALSE;
            return waterHeight + o->oFloatingPlatformHeightOffset;
        }
    }
    o->oFloatingPlatformIsOnFloor = TRUE;
    return floor.surface != NULL ? height + o->oFloatingPlatformHeightOffset : obj_height(o, &o->oHomeVec);
}

static void floating_platform_act_move_to_home(f32 height) {
    if (gMarioObject->platform == o) {
        Vec3f rider;
        obj_local_delta(o, rider, gMarioObject->header.gfx.pos, &o->oPosVec);
        f32 cy = coss(-o->oMoveAngleYaw);
        f32 sy = sins(-o->oMoveAngleYaw);
        o->oFaceAnglePitch = ((rider[2] * cy) - (rider[0] * sy)) * 2;
        o->oFaceAngleRoll = -((rider[0] * cy) + (rider[2] * sy)) * 2;
        o->oVelY -= 1.0f;
        if (o->oVelY < 0.0f) {
            o->oVelY = 0.0f;
        }

        o->oFloatingPlatformMarioWeightWobbleOffset += o->oVelY;
        if (o->oFloatingPlatformMarioWeightWobbleOffset > 90.0f) {
            o->oFloatingPlatformMarioWeightWobbleOffset = 90.0f;
        }
    } else {
        o->oFaceAnglePitch /= 2;
        o->oFaceAngleRoll /= 2;
        o->oFloatingPlatformMarioWeightWobbleOffset -= 5.0f;

        o->oVelY = 10.0f;

        if (o->oFloatingPlatformMarioWeightWobbleOffset < 0.0f) {
            o->oFloatingPlatformMarioWeightWobbleOffset = 0.0f;
        }
    }

    obj_set_height(o, &o->oPosVec, height - 64.0f - o->oFloatingPlatformMarioWeightWobbleOffset
                                 + sins(o->oFloatingPlatformWaterSurfaceWobbleOffset * 0x800) * 10.0f);

    o->oFloatingPlatformWaterSurfaceWobbleOffset++;

    if (o->oFloatingPlatformWaterSurfaceWobbleOffset == 32) {
        o->oFloatingPlatformWaterSurfaceWobbleOffset = 0;
    }
}

void bhv_floating_platform_loop(void) {
    f32 height = floating_platform_height();

    if (o->oFloatingPlatformIsOnFloor) {
        obj_set_height(o, &o->oPosVec, height);
    } else {
        floating_platform_act_move_to_home(height);
    }
}
