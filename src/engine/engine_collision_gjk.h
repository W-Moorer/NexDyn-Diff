// Copyright 2024 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SIMCORE_SRC_ENGINE_ENGINE_COLLISION_GJK_H_
#define SIMCORE_SRC_ENGINE_ENGINE_COLLISION_GJK_H_

#include <float.h>
#include <stddef.h>

#include <simcore/SIM_export.h>
#include <simcore/SIM_model.h>
#include <simcore/SIM_tnum.h>

#include "engine/engine_collision_convex.h"

#ifdef __cplusplus
extern "C" {
#endif

// numerical max limit
#ifndef SIM_USESINGLE
  #define SIM_MAX_LIMIT DBL_MAX
#else
  #define SIM_MAX_LIMIT FLT_MAX
#endif

// max number of EPA iterations
#define SIM_MAX_EPA_ITERATIONS 170

// tolerance for normal alignment of two faces (cosine of 1.6e-3)
#define SIM_FACE_TOL 0.99999872

// tolerance for edge-face alignment (sine of 1.6e-3)
#define SIM_EDGE_TOL 0.00159999931

// max number of supported vertices in a polygon face of a mesh
#define SIM_MAX_POLYVERT 150

// Status of an EPA run
typedef enum {
  SIM_EPA_NOCONTACT           = -1,
  SIM_EPA_SUCCESS             = 0,
  SIM_EPA_P2_INVALID_FACES,
  SIM_EPA_P2_NONCONVEX,
  SIM_EPA_P2_ORIGIN_ON_FACE,
  SIM_EPA_P3_BAD_NORMAL,
  SIM_EPA_P3_INVALID_V4,
  SIM_EPA_P3_INVALID_V5,
  SIM_EPA_P3_MISSING_ORIGIN,
  SIM_EPA_P3_ORIGIN_ON_FACE,
  SIM_EPA_P4_MISSING_ORIGIN,
} SIM_EPAStatus;

// vertex in a polytope
typedef struct {
  sim_scalar_t vert[3];   // v1 - v2; vertex in Minkowski sum making up polytope
  sim_scalar_t vert1[3];  // vertex of polytope in obj1
  sim_scalar_t vert2[3];  // vertex of polytope in obj2
  int index1;       // vertex index in mesh 1
  int index2;       // vertex index in mesh 2
} Vertex;

// configuration for convex collision detection
typedef struct {
  int max_iterations;  // the maximum number of iterations for GJK and EPA
  sim_scalar_t tolerance;    // tolerance used by GJK and EPA
  int max_contacts;    // set to max number of contact points to recover
  sim_scalar_t dist_cutoff;  // set to max geom distance to recover
  void* context;       // opaque data pointer passed to callbacks

  // callback to allocate memory for polytope (only needed for penetration recovery)
  void*(*alloc)(void* context, size_t nbytes);

  // callback to free memory from alloc callback
  void(*free)(void* context, void* buffer);
} SIM_CCDConfig;

// data produced from running GJK and EPA
typedef struct {
  // geom distance information
  sim_scalar_t dist;                  // distance between geoms
  sim_scalar_t x1[3 * SIM_MAXCONPAIR];  // witness points for geom 1
  sim_scalar_t x2[3 * SIM_MAXCONPAIR];  // witness points for geom 2
  int nx;                       // number of witness points

  // configurations used
  int max_iterations;           // the maximum number of iterations for GJK and EPA
  sim_scalar_t tolerance;             // tolerance used by GJK and EPA
  int max_contacts;             // set to max number of contact points to recover
  sim_scalar_t dist_cutoff;           // set to max geom distance to recover

  // statistics for debugging purposes
  int gjk_iterations;           // number of iterations that GJK ran
  int epa_iterations;           // number of iterations that EPA ran (zero if EPA did not run)
  SIM_EPAStatus epa_status;       // status of the EPA run
  Vertex simplex[4];
  int nsimplex;
} SIM_CCDStatus;

// run general convex collision detection, returns positive for distance, negative for penetration
SIM_API sim_scalar_t SIM_c_ccd(const SIM_CCDConfig* config, SIM_CCDStatus* status, sim_collision_ccd_object_t* obj1, sim_collision_ccd_object_t* obj2);
#ifdef __cplusplus
}
#endif

#endif  // SIMCORE_SRC_ENGINE_ENGINE_COLLISION_GJK_H_
