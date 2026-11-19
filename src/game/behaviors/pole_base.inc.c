// pole_base.inc.c

void bhv_pole_base_loop(void) {
    if (o->oTimer <= 10 || (gMarioState->action & ACT_FLAG_ON_POLE)) return;
    Vec3f delta, up;
    obj_hitbox_up(o, up);
    vec3_diff(delta, gMarioState->pos, &o->oPosVec);
    f32 height = vec3_dot(delta, up);
    if (height < obj_trigger_bottom(o) - 10.0f || height > obj_trigger_top(o) + 30.0f) return;
    for (s32 i = 0; i < 3; i++) delta[i] -= height * up[i];
    f32 distSq = vec3_sumsq(delta);
    if (distSq < sqr(70.0f)) {
        vec3_scale(delta, 1.0f - sqrtf(distSq) / 70.0f);
        vec3_add(gMarioState->pos, delta);
    }
}
