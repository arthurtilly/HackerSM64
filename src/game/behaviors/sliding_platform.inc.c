// sliding_platform.inc.c

void bhv_wf_sliding_platform_init(void) {
    o->oFaceAngleYaw -= 0x4000;
    obj_offset_local(o, &o->oPosVec, &o->oPosVec, 2.0f, 0.0f, 0.0f);
    vec3_copy(&o->oHomeVec, &o->oPosVec);

    switch (o->oBehParams2ndByte) {
        case WF_SLID_BRICK_PTFM_BP_MOV_VEL_10:
            o->oWFSlidBrickPtfmMovVel = 10.0f;
            break;

        case WF_SLID_BRICK_PTFM_BP_MOV_VEL_15:
            o->oWFSlidBrickPtfmMovVel = 15.0f;
            break;

        case WF_SLID_BRICK_PTFM_BP_MOV_VEL_20:
            o->oWFSlidBrickPtfmMovVel = 20.0f;
            break;
    }

    o->oTimer = random_float() * 100.0f;
}

void bhv_wf_sliding_platform_loop(void) {
    switch (o->oAction) {
        case WF_SLID_BRICK_PTFM_ACT_WAIT:
            if (o->oTimer > 100) {
                o->oAction = WF_SLID_BRICK_PTFM_ACT_EXTEND;
                o->oForwardVel = o->oWFSlidBrickPtfmMovVel;
            }
            break;

        case WF_SLID_BRICK_PTFM_ACT_EXTEND:
            if (o->oTimer >= 500.0f / o->oWFSlidBrickPtfmMovVel) {
                o->oForwardVel = 0.0f;
                obj_offset_local(o, &o->oPosVec, &o->oHomeVec, 510.0f, 0.0f, 0.0f);
            }

            if (o->oTimer == 60) {
                o->oAction = WF_SLID_BRICK_PTFM_ACT_RETRACT;
                o->oForwardVel = o->oWFSlidBrickPtfmMovVel;
                o->oMoveAngleYaw -= 0x8000;
            }
            break;

        case WF_SLID_BRICK_PTFM_ACT_RETRACT:
            if (o->oTimer >= 500.0f / o->oWFSlidBrickPtfmMovVel) {
                o->oForwardVel = 0.0f;
                cur_obj_set_pos_to_home();
            }

            if (o->oTimer == 90) {
                o->oAction = WF_SLID_BRICK_PTFM_ACT_EXTEND;
                o->oForwardVel = o->oWFSlidBrickPtfmMovVel;
                o->oMoveAngleYaw -= 0x8000;
            }
            break;
    }
}
