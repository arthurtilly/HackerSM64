
/**
 * Behaviors for bhvPoleGrabbing and bhvGiantPole.
 *
 * bhvPoleGrabbing is a generic behavior used almost anywhere the player can
 * grab onto and climb up another object.
 *
 * bhvGiantPole controls the large pole in Whomp's Fortress and differs from
 * other poles because it spawns a yellow ball that rests at its top.
 */

/**
 * The first two behavior parameter bytes give the shaft length in tens of units.
 */
void bhv_pole_init(void) {
    obj_set_interaction_volume(o, o->hitboxRadius, 0, GET_BPARAM12(o->oBehParams) * 10);
}

/**
 * Main loop for bhvGiantPole. It is equivalent to bhv_pole_base_loop() except
 * for the fact that it spawns a yellow sphere at the top of the pole on the
 * first frame.
 */
void bhv_giant_pole_loop(void) {
    if (o->oTimer == 0) {
        struct Object *topBall = spawn_object(o, MODEL_YELLOW_SPHERE, bhvYellowBall);
        obj_sphere_center(o, &o->oPosVec, obj_trigger_top(o) + 50.0f, &topBall->oPosVec);
    }
    bhv_pole_base_loop();
}
