// floating_box.inc.c

void bhv_jrb_floating_box_loop(void) {
    obj_set_height(o, &o->oPosVec, obj_height(o, &o->oHomeVec) + sins(o->oTimer * 0x400) * 10.0f);
}
