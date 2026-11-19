// lll_sinking_rock_block.inc.c

void bhv_lll_sinking_rock_block_loop(void) {
    lll_octagonal_mesh_find_y_offset(&o->oSinkWhenSteppedOnTimer, &o->oSinkWhenSteppedOnYOffsetFromHome, 124, -110);
    o->oGraphYOffset = 0.0f;
    obj_set_height(o, &o->oPosVec, obj_height(o, &o->oHomeVec) + o->oSinkWhenSteppedOnYOffsetFromHome);
}
