#ifndef SHADOW_H
#define SHADOW_H

#include <PR/ultratypes.h>
#include <PR/gbi.h>

#include "types.h"

/**
 * Shadow types. Shadows are circles, squares, or hardcoded rectangles, and
 * can be composed of either 4 or 9 vertices.
 */
enum ShadowType {
#ifdef LEGACY_SHADOW_IDS
    LEGACY_SHADOW_CIRCLE_9_VERTS             =  0,
    LEGACY_SHADOW_CIRCLE_4_VERTS             =  1,
    LEGACY_SHADOW_CIRCLE_4_VERTS_FLAT_UNUSED =  2,
    LEGACY_SHADOW_SQUARE_PERMANENT           = 10,
    LEGACY_SHADOW_SQUARE_SCALABLE            = 11,
    LEGACY_SHADOW_SQUARE_TOGGLABLE           = 12,
    LEGACY_SHADOW_CIRCLE_PLAYER              = 99,

    SHADOW_CIRCLE           = 5,
    SHADOW_CIRCLE_PLAYER    = SHADOW_CIRCLE,
    SHADOW_CIRCLE_4_VERTS   = SHADOW_CIRCLE,
    SHADOW_CIRCLE_9_VERTS   = SHADOW_CIRCLE,
    SHADOW_SQUARE           = 6,
    SHADOW_SQUARE_PERMANENT = 7,
    SHADOW_SQUARE_SCALABLE  = SHADOW_SQUARE,
    SHADOW_SQUARE_TOGGLABLE = SHADOW_SQUARE,
#else
    SHADOW_CIRCLE           = 0,
    SHADOW_CIRCLE_PLAYER    = SHADOW_CIRCLE,
    SHADOW_CIRCLE_4_VERTS   = SHADOW_CIRCLE,
    SHADOW_CIRCLE_9_VERTS   = SHADOW_CIRCLE,
    SHADOW_SQUARE           = 1,
    SHADOW_SQUARE_PERMANENT = 2,
    SHADOW_SQUARE_SCALABLE  = SHADOW_SQUARE,
    SHADOW_SQUARE_TOGGLABLE = SHADOW_SQUARE,
#endif
    /**
     * This defines an offset after which rectangular shadows with custom
     * widths and heights can be defined.
     */
    SHADOW_RECTANGLE_HARDCODED_OFFSET = 50,
    SHADOW_RECTANGLE_SPINDEL          = (0 + SHADOW_RECTANGLE_HARDCODED_OFFSET),
    SHADOW_RECTANGLE_WHOMP            = (1 + SHADOW_RECTANGLE_HARDCODED_OFFSET),
};

/**
 * Encapsulation of information about a shadow.
 */
struct Shadow {
    /* Normal vector of the floor. */
    Vec3f floorNormal;
    /* Size of the shadow. */
    Vec3f scale;
    /* Initial solidity of the shadow, from 0 to 255 (just an alpha value). */
    Alpha solidity;
    /*
     * Whether the shadow is a decal or not.
     * It should not be a decal when it's on a transparent surface (water, lava, ice, flying carpet, etc.).
     */
    s8 isDecal;
};

enum ShadowSolidity {
    /**
     * Constant to indicate that a shadow should not be drawn.
     * This is used to disable shadows during specific frames of Mario's
     * animations.
     */
    SHADOW_SOLIDITY_NO_SHADOW,
    /**
     * Constant to indicate that a shadow's solidity has been pre-set by a previous
     * function and should not be overwritten.
     */
    SHADOW_SOILDITY_ALREADY_SET,
    /**
     * Constant to indicate that a shadow's solidity has not yet been set.
     */
    SHADOW_SOLIDITY_NOT_YET_SET,
};

extern struct Shadow gCurrShadow;

// Project pos along the object's gravity and create the display list for the shadow there.
Gfx *create_shadow(const struct Object *obj, Vec3f pos, s16 shadowScale, u8 shadowSolidity, s8 shadowType);

#endif // SHADOW_H
