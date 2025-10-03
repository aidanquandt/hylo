#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#ifndef arr_len
#define arr_len(a) ((int)(sizeof(a)/sizeof((a)[0])))
#endif

typedef struct
{
    double x, y, z;
} vec3_S;

static inline double vec3_dist(const vec3_S* a, const vec3_S* b)
{
    double dx = a->x - b->x, dy = a->y - b->y, dz = a->z - b->z;
    return sqrt(dx*dx + dy*dy + dz*dz);
}

// Speed of light in m/s (DW1000 convention)
#define c_mps 299702547.0

// Time helpers
static inline double ns_to_s(uint64_t ns)
{
    return (double)ns * 1e-9;
}
static inline uint64_t s_to_ns(double s)
{
    return (uint64_t)(s * 1e9);
}
