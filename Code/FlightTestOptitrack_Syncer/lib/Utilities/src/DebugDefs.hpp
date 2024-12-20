/**
 * @file DebugDefs.hpp
 * @brief macros used for debugging
 */
#ifndef DEBUGDEFS_HPP__
#define DEBUGDEFS_HPP__

#define DEBUG_ENABLED 0

// wrap DEBUG_C around all debug code.
#if DEBUG_ENABLED
#define DEBUG_C(c) c
#else
#define DEBUG_C(c) 0
#endif

#endif