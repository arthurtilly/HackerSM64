#pragma once

// For the static assert macro
#include "macros.h"

/**
 * World scale value. This allows you to scale down geometry by the given amount, which allows for larger levels
 * without the distortion you would otherwise get. Larger world scale comes at a cost of precision, which
 * can increase Z-fighting. Values above 4 should not be necessary.
 *
 * Uncomment this out to specifically set it, otherwise world scale will be based off of your extended bounds mode.
 */
// #define WORLD_SCALE 1

/**
 *  0, 2: Regular bounds (-8192 to 8191)
 *  1: 2x extended bounds (-16384 to 16383)
 *  3: 4x extended bounds (-32768 to 32767)
 */

/**
 * Set this to the extended bounds mode you want, then do "make clean".
 */
#define EXTENDED_BOUNDS_MODE 1

/**************************************************************************
 * Don't touch the stuff past this point unless you know what you're doing!
 **************************************************************************/

#if EXTENDED_BOUNDS_MODE == 0 || EXTENDED_BOUNDS_MODE == 2
    #define LEVEL_BOUNDARY_MAX 0x2000L
    #ifndef WORLD_SCALE
        #define WORLD_SCALE 1
    #endif
#elif EXTENDED_BOUNDS_MODE == 1
    #define LEVEL_BOUNDARY_MAX 0x4000L
    #ifndef WORLD_SCALE
        #define WORLD_SCALE 2
    #endif
#elif EXTENDED_BOUNDS_MODE == 3
    #define LEVEL_BOUNDARY_MAX 0x8000L
    #ifndef WORLD_SCALE
        #define WORLD_SCALE 4
    #endif
#endif

STATIC_ASSERT(((EXTENDED_BOUNDS_MODE >= 0) && (EXTENDED_BOUNDS_MODE <= 3)), "You must set a valid extended bounds mode!");
