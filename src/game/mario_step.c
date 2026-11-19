#include <ultra64.h>

#include "sm64.h"
#include "engine/math_util.h"
#include "engine/surface_collision.h"
#include "mario.h"
#include "audio/external.h"
#include "game_init.h"
#include "interaction.h"
#include "object_helpers.h"
#include "mario_step.h"

#include "config.h"

static s16 sMovingSandSpeeds[] = { 12, 8, 4, 0 };

f32 mario_floor_with_water(struct MarioState *m, const Vec3f pos, f32 waterLevel, f32 height, struct SurfaceContact *floor) {
    f32 upY = -m->marioObj->gravityDir[1];
    if (!(m->action & ACT_FLAG_RIDING_SHELL) || waterLevel <= FLOOR_LOWER_LIMIT || upY <= WALL_MAX_UP) return height;
    f32 waterHeight = obj_height(m->marioObj, pos) + (waterLevel - pos[1]) / upY;
    if (floor->surface != NULL && waterHeight <= height) return height;
    static struct Surface water = { .type = SURFACE_VERY_SLIPPERY, .normal = { 0.0f, 1.0f, 0.0f } };
    water.originOffset = -waterLevel;
    *floor = (struct SurfaceContact){ .normal = water.normal, .type = water.type, .surface = &water };
    return waterHeight;
}

/**
 * Always returns zero. This may have been intended
 * to be used for the beta trampoline. Its return value
 * is used by set_mario_y_vel_based_on_fspeed as a constant
 * addition to Mario's Y velocity. Given the closeness of
 * this function to stub_mario_step_2, it is probable that this
 * was intended to check whether a trampoline had made itself
 * known through stub_mario_step_2 and whether Mario was on it,
 * and if so return a higher value than 0.
 */
f32 get_additive_y_vel_for_jumps(void) {
    return 0.0f;
}

/**
 * Does nothing, but takes in a MarioState. This is only ever
 * called by update_mario_inputs, which is called as part of Mario's
 * update routine. Due to its proximity to stub_mario_step_2, an
 * incomplete trampoline function, and get_additive_y_vel_for_jumps,
 * a potentially trampoline-related function, it is plausible that
 * this could be used for checking if Mario was on the trampoline.
 * It could, for example, make him bounce.
 */
void stub_mario_step_1(UNUSED struct MarioState *x) {
}

/**
 * Does nothing. This is only called by the beta trampoline.
 * Due to its proximity to get_additive_y_vel_for_jumps, another
 * currently-pointless function, it is probable that this was used
 * by the trampoline to make itself known to get_additive_y_vel_for_jumps,
 * or to set a variable with its intended additive Y vel.
 */
void stub_mario_step_2(void) {
}

void mario_bonk_reflection(struct MarioState *m, u32 negateSpeed) {
    if (m->hasWall) {
        s16 wallAngle = m->wallYaw;
        m->faceAngle[1] = wallAngle - (s16)(m->faceAngle[1] - wallAngle);

        play_sound((m->flags & MARIO_METAL_CAP) ? SOUND_ACTION_METAL_BONK : SOUND_ACTION_BONK,
                   m->marioObj->header.gfx.cameraToObject);
    } else {
        play_sound(SOUND_ACTION_HIT, m->marioObj->header.gfx.cameraToObject);
    }

    if (negateSpeed) {
        mario_set_forward_vel(m, -m->forwardVel);
    } else {
        m->faceAngle[1] += 0x8000;
    }
}

u32 mario_update_quicksand(struct MarioState *m, f32 sinkingSpeed) {
    if (m->action & ACT_FLAG_RIDING_SHELL) {
        m->quicksandDepth = 0.0f;
    } else {
        if (m->quicksandDepth < 1.1f) {
            m->quicksandDepth = 1.1f;
        }

        switch (m->floorType) {
            case SURFACE_SHALLOW_QUICKSAND:
                if ((m->quicksandDepth += sinkingSpeed) >= 10.0f) {
                    m->quicksandDepth = 10.0f;
                }
                break;

            case SURFACE_SHALLOW_MOVING_QUICKSAND:
                if ((m->quicksandDepth += sinkingSpeed) >= 25.0f) {
                    m->quicksandDepth = 25.0f;
                }
                break;

            case SURFACE_QUICKSAND:
            case SURFACE_MOVING_QUICKSAND:
                if ((m->quicksandDepth += sinkingSpeed) >= 60.0f) {
                    m->quicksandDepth = 60.0f;
                }
                break;

            case SURFACE_DEEP_QUICKSAND:
            case SURFACE_DEEP_MOVING_QUICKSAND:
                if ((m->quicksandDepth += sinkingSpeed) >= 160.0f) {
                    update_mario_sound_and_camera(m);
                    return drop_and_set_mario_action(m, ACT_QUICKSAND_DEATH, 0);
                }
                break;

            case SURFACE_INSTANT_QUICKSAND:
            case SURFACE_INSTANT_MOVING_QUICKSAND:
                update_mario_sound_and_camera(m);
                return drop_and_set_mario_action(m, ACT_QUICKSAND_DEATH, 0);
                break;

            default:
                m->quicksandDepth = 0.0f;
                break;
        }
    }

    return FALSE;
}

u32 mario_push_off_steep_floor(struct MarioState *m, u32 action, u32 actionArg) {
    s16 floorDYaw = m->floorYaw - m->faceAngle[1];

    if (floorDYaw > -0x4000 && floorDYaw < 0x4000) {
        m->forwardVel = 16.0f;
        m->faceAngle[1] = m->floorYaw;
    } else {
        m->forwardVel = -16.0f;
        m->faceAngle[1] = m->floorYaw + 0x8000;
    }

    return set_mario_action(m, action, actionArg);
}

u32 mario_update_moving_sand(struct MarioState *m) {
    s32 floorType = m->floorType;

    if (floorType == SURFACE_DEEP_MOVING_QUICKSAND || floorType == SURFACE_SHALLOW_MOVING_QUICKSAND
        || floorType == SURFACE_MOVING_QUICKSAND || floorType == SURFACE_INSTANT_MOVING_QUICKSAND) {
        s16 pushAngle = obj_yaw_in_frame(m->marioObj, NULL, m->floor->force << 8);
        f32 pushSpeed = sMovingSandSpeeds[m->floor->force >> 8];

        m->vel[0] += pushSpeed * sins(pushAngle);
        m->vel[2] += pushSpeed * coss(pushAngle);

        return TRUE;
    }

    return FALSE;
}

u32 mario_update_windy_ground(struct MarioState *m) {
    if (m->floorType == SURFACE_HORIZONTAL_WIND) {
        f32 pushSpeed;
        s16 pushAngle = obj_yaw_in_frame(m->marioObj, NULL, m->floor->force << 8);

        if (m->action & ACT_FLAG_MOVING) {
            s16 pushDYaw = m->faceAngle[1] - pushAngle;

            pushSpeed = m->forwardVel > 0.0f ? -m->forwardVel * 0.5f : -8.0f;

            if (pushDYaw > -0x4000 && pushDYaw < 0x4000) {
                pushSpeed *= -1.0f;
            }

            pushSpeed *= coss(pushDYaw);
        } else {
            pushSpeed = 3.2f + (gGlobalTimer % 4);
        }

        m->vel[0] += pushSpeed * sins(pushAngle);
        m->vel[2] += pushSpeed * coss(pushAngle);

        return TRUE;
    }

    return FALSE;
}

void stop_and_set_height_to_floor(struct MarioState *m) {
    struct Object *marioObj = m->marioObj;

    mario_set_forward_vel(m, 0.0f);
    m->vel[1] = 0.0f;

    // HackerSM64 2.1: This check fixes the ledgegrab downwarp after being pushed off a ledge.
    if (m->floor != NULL && obj_height(m->marioObj, m->pos) <= m->floorHeight + 160.0f) {
        obj_set_height(m->marioObj, m->pos, m->floorHeight);
    }

    vec3f_copy(marioObj->header.gfx.pos, m->pos);
    vec3s_set(marioObj->header.gfx.angle, 0, m->faceAngle[1], 0);
}

s32 stationary_ground_step(struct MarioState *m) {
    struct Object *marioObj = m->marioObj;
    u32 stepResult = GROUND_STEP_NONE;

    mario_set_forward_vel(m, 0.0f);

    u32 takeStep = (mario_update_moving_sand(m) | mario_update_windy_ground(m));
    if (takeStep) {
        stepResult = perform_ground_step(m);
    } else {
        // HackerSM64 2.1: This check prevents the downwarps that plagued stationary actions.
        if (m->floor != NULL && obj_height(m->marioObj, m->pos) <= m->floorHeight + 160.0f) {
            obj_set_height(m->marioObj, m->pos, m->floorHeight);
        }

        vec3f_copy(marioObj->header.gfx.pos, m->pos);
        vec3s_set(marioObj->header.gfx.angle, 0, m->faceAngle[1], 0);
    }

    return stepResult;
}

enum MarioMovement { MOVE_GROUND, MOVE_AIR, MOVE_SWIM };

static const struct ActorShape sMarioShapes[] = {
    [MOVE_GROUND] = { 50.0f, 100.0f, 100.0f, 160.0f },
    [MOVE_AIR]    = { 50.0f,  50.0f, 110.0f, 160.0f },
    [MOVE_SWIM]   = { 110.0f, 80.0f,  80.0f, 160.0f },
};

static const struct ActorShape *mario_shape(struct MarioState *m, enum MarioMovement mode) {
    // Both ground probes clear the same 50-unit riser. The short one fits beneath a low lintel.
    static const struct ActorShape crawling = { 23.0f, 73.0f, 73.0f, 2 * MARIO_CRAWL_RADIUS };
    if (mode == MOVE_GROUND && (m->action & ACT_FLAG_SHORT_HITBOX)) return &crawling;
    return &sMarioShapes[mode];
}

void resolve_mario_ground_walls(struct MarioState *m) {
    const struct ActorShape *shape = mario_shape(m, MOVE_GROUND);
    resolve_actor_wall(m->marioObj, m->pos, shape->lowerOffset, shape->radius, 1.0f, NULL);
}

struct MarioSweep {
    Vec3f pos;
    struct SurfaceContact wall, floor, ceil;
    s32 atBoundary;
};

static s32 clamp_to_level(Vec3f pos) {
    if (!is_outside_level_bounds(pos[0], pos[2])) return FALSE;
    f32 limit = LEVEL_BOUNDARY_MAX - 1.0f;
    pos[0] = CLAMP(pos[0], -limit, limit);
    pos[2] = CLAMP(pos[2], -limit, limit);
    return TRUE;
}

static void sweep_mario(struct MarioState *m, enum MarioMovement mode, const Vec3f displacement, struct MarioSweep *result) {
    const struct ActorShape *shape = mario_shape(m, mode);
    Vec3f move, end, up;
    vec3_scale_dest(up, m->marioObj->gravityDir, -1.0f);
    vec3_copy(move, displacement);
    vec3_copy(result->pos, m->pos);
    result->floor.surface = result->ceil.surface = result->wall.surface = NULL;
    result->atBoundary = FALSE;

    // Only cast again when a contact leaves motion to slide along another face.
    for (s32 i = 0; i < 3 && vec3_sumsq(move) > 0.000001f; i++) {
        struct SweepHit hit;
        vec3_sum(end, result->pos, move);
        if (clamp_to_level(end)) {
            result->atBoundary = TRUE;
            vec3_diff(move, end, result->pos);
        }
        if (!sweep_actor(result->pos, move, up, shape, &hit)) {
            vec3_copy(result->pos, end);
            break;
        }
        for (s32 axis = 0; axis < 3; axis++) result->pos[axis] += move[axis] * hit.time;
        vec3_scale(move, 1.0f - hit.time);
        f32 inward = MIN(vec3_dot(move, hit.normal), 0.0f);

        f32 faceUp = obj_normal_up(m->marioObj, hit.face.normal);
        f32 contactUp = vec3_dot(hit.normal, up);
        s32 wall = absf(faceUp) <= WALL_MAX_UP;
        if (!wall) {
            if (faceUp > 0.0f) result->floor = hit.face;
            else result->ceil = hit.face;
            if (mode == MOVE_AIR) break;
        } else if (mode == MOVE_SWIM || (absf(faceUp) <= WALL_KICK_MAX_UP
                                      && absf(contactUp) <= WALL_KICK_MAX_UP)) {
            result->wall = hit.face;
        }

        f32 lateralSq = 1.0f - sqr(contactUp);
        // Walk along upward-facing contacts, including the rounded lip of an obstacle.
        // Against a wall, keep the vertical component of the movement.
        if ((!wall && absf(contactUp) > 0.0001f) || (mode == MOVE_GROUND && contactUp > WALL_MAX_UP)) {
            vec3_scale_dest(hit.normal, up, 1.0f / contactUp);
        } else if (lateralSq > 0.25f) {
            for (s32 axis = 0; axis < 3; axis++) hit.normal[axis] -= up[axis] * contactUp;
            vec3_scale(hit.normal, 1.0f / lateralSq);
        }
        f32 push = wall ? hit.depth + 0.02f : 0.0f;
        for (s32 axis = 0; axis < 3; axis++) {
            result->pos[axis] += hit.normal[axis] * push;
            move[axis] -= hit.normal[axis] * inward;
        }
        result->atBoundary |= clamp_to_level(result->pos);
    }
}

static f32 find_step_floor(struct MarioState *m, Vec3f pos, struct SurfaceContact *floor) {
    f32 height = find_actor_floor(m->marioObj, pos, 0.0f, floor);
    if (m->action & ACT_FLAG_RIDING_SHELL) {
        height = mario_floor_with_water(m, pos, find_water_level(pos[0], pos[2]), height, floor);
    }
    return height;
}

s32 perform_ground_step(struct MarioState *m) {
    struct MarioSweep step;
    const struct Normal *n = &m->floorNormal;
    Vec3f move = { n->y * m->vel[0], -(n->x * m->vel[0] + n->z * m->vel[2]), n->y * m->vel[2] };
    obj_local_to_world(m->marioObj, move, move);
    sweep_mario(m, MOVE_GROUND, move, &step);
    set_mario_wall(m, &step.wall);

    struct SurfaceContact floor, ceil;
    f32 floorHeight = find_step_floor(m, step.pos, &floor);
    f32 height = obj_height(m->marioObj, step.pos);
    s32 leftGround = floor.surface == NULL || height > floorHeight + 100.0f;
    if (!leftGround) height = floorHeight;
    f32 clearance = mario_shape(m, MOVE_GROUND)->height;
    f32 ceilHeight = find_actor_ceil(m->marioObj, step.pos, floorHeight, clearance, &ceil);
    s32 result = GROUND_STEP_NONE;
    if (ceil.surface != NULL && height + clearance > ceilHeight) {
        result = GROUND_STEP_HIT_WALL;
    } else {
        obj_set_height(m->marioObj, step.pos, height);
        vec3_copy(m->pos, step.pos);
        set_mario_floor(m, &floor, floorHeight);
        if (leftGround) {
            result = GROUND_STEP_LEFT_GROUND;
        } else if (step.atBoundary || step.ceil.surface != NULL) {
            result = GROUND_STEP_HIT_WALL;
        } else if (m->hasWall) {
            s16 difference = abs_angle_diff(m->wallYaw, m->faceAngle[1]);
            if (difference < DEGREES(60) || difference > DEGREES(120)) result = GROUND_STEP_HIT_WALL;
        }
    }
    m->terrainSoundAddend = mario_get_terrain_sound_addend(m);
    vec3f_copy(m->marioObj->header.gfx.pos, m->pos);
    vec3s_set(m->marioObj->header.gfx.angle, 0, m->faceAngle[1], 0);
    return result;
}

static s32 try_ledge_grab(struct MarioState *m, const struct SurfaceContact *wall) {
    struct Normal n = obj_local_normal(m->marioObj, wall->normal);
    if (m->vel[1] > 0.0f || n.x * m->vel[0] + n.z * m->vel[2] > 0.0f) return FALSE;
    Vec3f ledge, hands, reach = { -n.x, 0.0f, -n.z };
    obj_offset_local(m->marioObj, ledge, m->pos, reach[0] * 60.0f, sMarioShapes[MOVE_AIR].height, reach[2] * 60.0f);
    obj_local_to_world(m->marioObj, reach, reach);
    if (is_outside_level_bounds(ledge[0], ledge[2])) return FALSE;
    struct SurfaceContact floor;
    f32 distance = collision_raycast(ledge, m->marioObj->gravityDir, 60.0f, WALL_MAX_UP, 0, &floor);
    if (distance < 0.0f) return FALSE;
#ifdef DONT_LEDGE_GRAB_STEEP_SLOPES
    if (obj_normal_up(m->marioObj, floor.normal) < COS25) return FALSE;
#endif
    obj_add_up(m->marioObj, ledge, -distance);
    f32 height = obj_height(m->marioObj, ledge);
    vec3_copy(hands, m->pos);
    obj_set_height(m->marioObj, hands, height + 3.0f);
    if (collision_raycast(hands, reach, 60.0f, 0.0f, 0, NULL) >= 0.0f) return FALSE;
    vec3_copy(m->pos, ledge);
    set_mario_floor(m, &floor, height);
    m->faceAngle[0] = 0;
    m->faceAngle[1] = atan2s(n.z, n.x) + 0x8000;
    return TRUE;
}

static s32 move_mario_in_air(struct MarioState *m, u32 stepArg) {
    struct MarioSweep step;
    Vec3f move;
    obj_local_to_world(m->marioObj, move, m->vel);
    sweep_mario(m, MOVE_AIR, move, &step);
    vec3_copy(m->pos, step.pos);
    set_mario_wall(m, &step.wall);
    if (step.floor.surface != NULL) {
        set_mario_floor(m, &step.floor, obj_height(m->marioObj, m->pos));
        return AIR_STEP_LANDED;
    }

    struct SurfaceContact floor;
    f32 floorHeight = find_step_floor(m, m->pos, &floor);
    set_mario_floor(m, &floor, floorHeight);
    if (floor.surface != NULL && m->vel[1] <= 0.0f && obj_height(m->marioObj, m->pos) <= floorHeight) {
        obj_set_height(m->marioObj, m->pos, floorHeight);
        return AIR_STEP_LANDED;
    }
    if (step.ceil.surface != NULL) {
        set_mario_ceil(m, step.ceil.surface, obj_height(m->marioObj, m->pos) + sMarioShapes[MOVE_AIR].height);
        if (m->vel[1] < 0.0f) return AIR_STEP_HIT_CEILING;
        m->vel[1] = 0.0f;
#ifdef HANGING_FIX
        if (!(m->prevAction & ACT_FLAG_HANGING) && step.ceil.type == SURFACE_HANGABLE) return AIR_STEP_GRABBED_CEILING;
#else
        if ((stepArg & AIR_STEP_CHECK_HANG) && step.ceil.type == SURFACE_HANGABLE) return AIR_STEP_GRABBED_CEILING;
#endif
    }
    if (m->hasWall) {
        if ((stepArg & AIR_STEP_CHECK_LEDGE_GRAB) && try_ledge_grab(m, &step.wall)) return AIR_STEP_GRABBED_LEDGE;
        if (step.wall.type == SURFACE_BURNING) return AIR_STEP_HIT_LAVA_WALL;
        if (abs_angle_diff(m->wallYaw, m->faceAngle[1]) > DEGREES(180 - WALL_KICK_DEGREES)) {
            m->flags |= MARIO_AIR_HIT_WALL;
            return AIR_STEP_HIT_WALL;
        }
    }
    return AIR_STEP_NONE;
}

s32 move_mario_in_water(struct MarioState *m, const Vec3f move) {
    struct MarioSweep step;
    struct SurfaceContact floor, ceil;
    f32 height = sMarioShapes[MOVE_SWIM].height;
    sweep_mario(m, MOVE_SWIM, move, &step);
    f32 floorHeight = find_step_floor(m, step.pos, &floor);
    f32 ceilHeight = find_actor_ceil(m->marioObj, step.pos, floorHeight, height, &ceil);
    if (floor.surface != NULL && ceil.surface != NULL && ceilHeight - floorHeight < height) return WATER_STEP_CANCELLED;

    s32 result = WATER_STEP_NONE;
    if (floor.surface != NULL && obj_height(m->marioObj, step.pos) < floorHeight) {
        obj_set_height(m->marioObj, step.pos, floorHeight);
        result = WATER_STEP_HIT_FLOOR;
    } else if (ceil.surface != NULL && obj_height(m->marioObj, step.pos) + height > ceilHeight) {
        obj_set_height(m->marioObj, step.pos, ceilHeight - height);
        result = WATER_STEP_HIT_CEILING;
    } else if (step.floor.surface != NULL) {
        result = WATER_STEP_HIT_FLOOR;
    } else if (step.ceil.surface != NULL) {
        result = WATER_STEP_HIT_CEILING;
    } else if (step.wall.surface != NULL) {
        result = WATER_STEP_HIT_WALL;
    } else if (step.atBoundary) {
        result = WATER_STEP_CANCELLED;
    }
    set_mario_floor(m, &floor, floorHeight);
    vec3_copy(m->pos, step.pos);
    return result;
}

void apply_twirl_gravity(struct MarioState *m) {
#ifdef Z_TWIRL
    f32 Zmodifier = m->input & INPUT_Z_DOWN ? 4.0f : 1.0f;
#endif
    f32 heaviness = 1.0f;

    if (m->angleVel[1] > 1024) {
        heaviness = 1024.0f / m->angleVel[1];
    }

#ifdef Z_TWIRL
    f32 terminalVelocity = -75.0f * heaviness * Zmodifier;
    m->vel[1] -= 4.0f * heaviness * Zmodifier;
#else
    f32 terminalVelocity = -75.0f * heaviness;

    m->vel[1] -= 4.0f * heaviness;
#endif
    if (m->vel[1] < terminalVelocity) {
        m->vel[1] = terminalVelocity;
    }
}

u32 should_strengthen_gravity_for_jump_ascent(struct MarioState *m) {
    if (!(m->flags & MARIO_JUMPING)) {
        return FALSE;
    }

    if (m->action & (ACT_FLAG_INTANGIBLE | ACT_FLAG_INVULNERABLE)) {
        return FALSE;
    }

    if (!(m->input & INPUT_A_DOWN) && m->vel[1] > 20.0f) {
        return (m->action & ACT_FLAG_CONTROL_JUMP_HEIGHT) != 0;
    }

    return FALSE;
}

void apply_gravity(struct MarioState *m) {
    if (m->action == ACT_TWIRLING && m->vel[1] < 0.0f) {
        apply_twirl_gravity(m);
    } else if (m->action == ACT_SHOT_FROM_CANNON) {
        m->vel[1] -= 1.0f;
        if (m->vel[1] < -75.0f) {
            m->vel[1] = -75.0f;
        }
    } else if (m->action == ACT_LONG_JUMP || m->action == ACT_SLIDE_KICK
               || m->action == ACT_BBH_ENTER_SPIN) {
        m->vel[1] -= 2.0f;
        if (m->vel[1] < -75.0f) {
            m->vel[1] = -75.0f;
        }
    } else if (m->action == ACT_LAVA_BOOST || m->action == ACT_FALL_AFTER_STAR_GRAB) {
        m->vel[1] -= 3.2f;
        if (m->vel[1] < -65.0f) {
            m->vel[1] = -65.0f;
        }
    } else if (m->action == ACT_GETTING_BLOWN) {
        m->vel[1] -= m->windGravity;
        if (m->vel[1] < -75.0f) {
            m->vel[1] = -75.0f;
        }
    } else if (should_strengthen_gravity_for_jump_ascent(m)) {
        m->vel[1] /= 4.0f;
    } else if (m->action & ACT_FLAG_METAL_WATER) {
        m->vel[1] -= 1.6f;
        if (m->vel[1] < -16.0f) {
            m->vel[1] = -16.0f;
        }
    } else if ((m->flags & MARIO_WING_CAP) && m->vel[1] < 0.0f && (m->input & INPUT_A_DOWN)) {
        m->marioBodyState->wingFlutter = TRUE;

        m->vel[1] -= 2.0f;
        if (m->vel[1] < -37.5f) {
            if ((m->vel[1] += 4.0f) > -37.5f) {
                m->vel[1] = -37.5f;
            }
        }
    } else {
        m->vel[1] -= 4.0f;
        if (m->vel[1] < -75.0f) {
            m->vel[1] = -75.0f;
        }
    }
}

void apply_vertical_wind(struct MarioState *m) {
    f32 maxVelY;

    if (m->action != ACT_GROUND_POUND) {
        f32 offsetY = m->pos[1] - -1500.0f;

        if (m->floorType == SURFACE_VERTICAL_WIND && -3000.0f < offsetY && offsetY < 2000.0f) {
            if (offsetY >= 0.0f) {
                maxVelY = 10000.0f / (offsetY + 200.0f);
            } else {
                maxVelY = 50.0f;
            }

            if (m->vel[1] < maxVelY) {
                if ((m->vel[1] += maxVelY / 8.0f) > maxVelY) {
                    m->vel[1] = maxVelY;
                }
            }
        }
    }
}

s32 perform_air_step(struct MarioState *m, u32 stepArg) {
    s32 stepResult = move_mario_in_air(m, stepArg);

    if (m->vel[1] >= 0.0f) {
        m->peakHeight = obj_height(m->marioObj, m->pos);
    }

    m->terrainSoundAddend = mario_get_terrain_sound_addend(m);

    if (m->action != ACT_FLYING) {
        apply_gravity(m);
    }
    apply_vertical_wind(m);

    vec3f_copy(m->marioObj->header.gfx.pos, m->pos);
    vec3s_set(m->marioObj->header.gfx.angle, 0, m->faceAngle[1], 0);

    return stepResult;
}

// They had these functions the whole time and never used them? Lol

void set_vel_from_pitch_and_yaw(struct MarioState *m) {
    m->vel[0] = m->forwardVel * coss(m->faceAngle[0]) * sins(m->faceAngle[1]);
    m->vel[1] = m->forwardVel * sins(m->faceAngle[0]);
    m->vel[2] = m->forwardVel * coss(m->faceAngle[0]) * coss(m->faceAngle[1]);
}

void set_vel_from_yaw(struct MarioState *m) {
    m->vel[0] = m->slideVelX = m->forwardVel * sins(m->faceAngle[1]);
    m->vel[1] = 0.0f;
    m->vel[2] = m->slideVelZ = m->forwardVel * coss(m->faceAngle[1]);
}
