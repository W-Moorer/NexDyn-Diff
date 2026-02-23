// Copyright 2021 DeepMind Technologies Limited
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

#ifndef SIMCORE_SRC_ENGINE_ENGINE_COLLISION_DRIVER_H_
#define SIMCORE_SRC_ENGINE_ENGINE_COLLISION_DRIVER_H_

#include <simcore/SIM_data.h>
#include <simcore/SIM_export.h>
#include <simcore/SIM_model.h>

#ifdef __cplusplus
extern "C" {
#endif

// collision function pointers and max contact pairs
SIM_API extern SIM_fCollision SIM_COLLISIONFUNC[SIM_NGEOMTYPES][SIM_NGEOMTYPES];

// collision detection entry point
SIM_API void sim_collision(const sim_model_t* m, sim_data_t* d);

// applies Separating Axis Theorem for rotated AABBs
SIM_API int sim_collideOBB(const sim_scalar_t aabb1[6], const sim_scalar_t aabb2[6],
                        const sim_scalar_t xpos1[3], const sim_scalar_t xmat1[9],
                        const sim_scalar_t xpos2[3], const sim_scalar_t xmat2[9], sim_scalar_t margin,
                        sim_scalar_t product[36], sim_scalar_t offset[12], sim_byte_t* initialize);

// is element active (for collisions)
SIM_API int sim_isElemActive(const sim_model_t* m, int f, int e);

// checks if pair is already present in pair_geom and calls narrow phase
void sim_collideGeomPair(const sim_model_t* m, sim_data_t* d, int g1, int g2, int merged,
                        int startadr, int pairadr);

// binary search between two bodyflex trees
void sim_collideTree(const sim_model_t* m, sim_data_t* d, int bf1, int bf2,
                    int merged, int startadr, int pairadr);

// broad phase collision detection; return list of bodyflex pairs
int sim_broadphase(const sim_model_t* m, sim_data_t* d, int* bfpair, int maxpair);

// test two geoms for collision, apply filters, add to contact list
void sim_collideGeoms(const sim_model_t* m, sim_data_t* d, int g1, int g2);

// test a plane geom and a flex for collision, add to contact list
void sim_collidePlaneFlex(const sim_model_t* m, sim_data_t* d, int g, int f);

// test for internal flex collisions, add to contact list
void sim_collideFlexInternal(const sim_model_t* m, sim_data_t* d, int f);

// test active element self-collisions with SAP
void sim_collideFlexSAP(const sim_model_t* m, sim_data_t* d, int f);

// test a geom and an elem for collision, add to contact list
void sim_collideGeomElem(const sim_model_t* m, sim_data_t* d, int g, int f, int e);

// test two elems for collision, add to contact list
void sim_collideElems(const sim_model_t* m, sim_data_t* d, int f1, int e1, int f2, int e2);

// test element and vertex for collision, add to contact list
void sim_collideElemVert(const sim_model_t* m, sim_data_t* d, int f, int e, int v);


#ifdef __cplusplus
}
#endif

#endif  // SIMCORE_SRC_ENGINE_ENGINE_COLLISION_DRIVER_H_
