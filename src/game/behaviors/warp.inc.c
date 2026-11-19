// The radius parameter describes a patch of floor, including level-wide warps.
static void set_warp_trigger(f32 defaultRadius) {
    if (o->oTimer == 0) {
        u32 param = GET_BPARAM1(o->oBehParams);
        f32 radius = param == 0 ? defaultRadius : param == 0xFF ? 10000.0f : param * 10.0f;
        obj_set_interaction_volume(o, MAX(1.0f, radius - 28.0f), 0.0f, 50.0f);
    }
    o->oInteractStatus = INT_STATUS_NONE;
}

void bhv_warp_loop(void) {
    set_warp_trigger(50.0f);
}

void bhv_fading_warp_loop(void) {
    set_warp_trigger(85.0f);
}
