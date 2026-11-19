#ifndef MARIO_STEP_H
#define MARIO_STEP_H

#include <PR/ultratypes.h>

#include "types.h"

f32 get_additive_y_vel_for_jumps(void);
void stub_mario_step_1(struct MarioState *m);
void stub_mario_step_2(void);

void mario_bonk_reflection(struct MarioState *m, u32 negateSpeed);
u32 mario_update_quicksand(struct MarioState *m, f32 sinkingSpeed);
u32 mario_push_off_steep_floor(struct MarioState *m, u32 action, u32 actionArg);
u32 mario_update_moving_sand(struct MarioState *m);
u32 mario_update_windy_ground(struct MarioState *m);
void stop_and_set_height_to_floor(struct MarioState *m);
s32 stationary_ground_step(struct MarioState *m);
struct SurfaceContact;
f32 mario_floor_with_water(struct MarioState *m, const Vec3f pos, f32 waterLevel, f32 height, struct SurfaceContact *floor);
void resolve_mario_ground_walls(struct MarioState *m);
s32 move_mario_in_water(struct MarioState *m, const Vec3f move);
s32 perform_ground_step(struct MarioState *m);
s32 perform_air_step(struct MarioState *m, u32 stepArg);

#endif // MARIO_STEP_H
