
/**
 * Behavior for bhvBetaHoldableObject.
 * This is a simple implementation of a holdable object, probably used
 * for testing. This was previously assumed to be a beta shell, as there
 * are unused shell models left in the game; however, there is no evidence
 * to support this theory.
 */

/**
 * Drop the object.
 */
static void beta_holdable_object_drop(void) {
    // Re-enable rendering
    cur_obj_enable_rendering();

    cur_obj_get_dropped();

    o->oHeldState = HELD_FREE;

    o->oForwardVel = 0.0f;
    o->oVelY = 0.0f;
}

/**
 * Throw the object.
 */
static void beta_holdable_object_throw(void) {
    cur_obj_enable_rendering();

    o->oHeldState = HELD_FREE;

    // This flag is never set, why is it cleared?
    o->oFlags &= ~OBJ_FLAG_SET_FACE_YAW_TO_MOVE_YAW;

    // Set initial velocity
    o->oForwardVel = 40.0f;
    o->oVelY = 20.0f;
}

/**
 * Update function for bhvBetaHoldableObject.
 * Apply standard physics to the object if not held;
 * otherwise, handle holding logic.
 */
void bhv_beta_holdable_object_loop(void) {
    switch (o->oHeldState) {
        case HELD_FREE:
            cur_obj_update_floor_and_bounce_off_walls();
            cur_obj_move_standard(60);
            break;

        case HELD_HELD:
            // Disable rendering to hide the object while it's held
            cur_obj_disable_rendering();
            break;

        case HELD_THROWN:
            beta_holdable_object_throw();
            break;

        case HELD_DROPPED:
            beta_holdable_object_drop();
            break;
    }
}
