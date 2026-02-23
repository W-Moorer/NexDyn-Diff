// Copyright 2025 DeepMind Technologies Limited
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

#ifndef SIMCORE_SRC_ENGINE_ENGINE_CORE_UTIL_H_
#define SIMCORE_SRC_ENGINE_ENGINE_CORE_UTIL_H_

#include <simcore/SIM_data.h>
#include <simcore/SIM_export.h>
#include <simcore/SIM_model.h>

#ifdef __cplusplus
extern "C" {
#endif


//-------------------------- model properties ------------------------------------------------------

// determine type of friction cone
SIM_API int sim_isPyramidal(const sim_model_t* m);

// determine type of constraint Jacobian
SIM_API int sim_isSparse(const sim_model_t* m);


//-------------------------- sparse chains ---------------------------------------------------------

// merge dof chains for two bodies
int sim_mergeChain(const sim_model_t* m, int* chain, int b1, int b2);

// merge dof chains for two simple bodies
int sim_mergeChainSimple(const sim_model_t* m, int* chain, int b1, int b2);

// get body chain
int sim_bodyChain(const sim_model_t* m, int body, int* chain);


//-------------------------- Jacobians -------------------------------------------------------------

// compute 3/6-by-nv Jacobian of global point attached to given body
SIM_API void sim_jac(const sim_model_t* m, const sim_data_t* d,
                  sim_scalar_t* jacp, sim_scalar_t* jacr, const sim_scalar_t point[3], int body);

// compute body frame Jacobian
SIM_API void sim_jacBody(const sim_model_t* m, const sim_data_t* d,
                      sim_scalar_t* jacp, sim_scalar_t* jacr, int body);

// compute body center-of-mass Jacobian
SIM_API void sim_jacBodyCom(const sim_model_t* m, const sim_data_t* d,
                         sim_scalar_t* jacp, sim_scalar_t* jacr, int body);

// compute subtree center-of-mass Jacobian
SIM_API void sim_jacSubtreeCom(const sim_model_t* m, sim_data_t* d, sim_scalar_t* jacp, int body);

// compute geom Jacobian
SIM_API void sim_jacGeom(const sim_model_t* m, const sim_data_t* d,
                      sim_scalar_t* jacp, sim_scalar_t* jacr, int geom);

// compute site Jacobian
SIM_API void sim_jacSite(const sim_model_t* m, const sim_data_t* d,
                      sim_scalar_t* jacp, sim_scalar_t* jacr, int site);

// compute translation Jacobian of point, and rotation Jacobian of axis
SIM_API void sim_jacPointAxis(const sim_model_t* m, sim_data_t* d,
                           sim_scalar_t* jacPoint, sim_scalar_t* jacAxis,
                           const sim_scalar_t point[3], const sim_scalar_t axis[3], int body);

// compute 3/6-by-nv sparse Jacobian of global point attached to given body
void sim_jacSparse(const sim_model_t* m, const sim_data_t* d,
                  sim_scalar_t* jacp, sim_scalar_t* jacr, const sim_scalar_t* point, int body,
                  int NV, const int* chain);

// sparse Jacobian difference for simple body contacts
void sim_jacSparseSimple(const sim_model_t* m, const sim_data_t* d,
                        sim_scalar_t* jacdifp, sim_scalar_t* jacdifr, const sim_scalar_t* point,
                        int body, int flg_second, int NV, int start);

// dense or sparse Jacobian difference for two body points: pos2 - pos1, global
SIM_API int sim_jacDifPair(const sim_model_t* m, const sim_data_t* d, int* chain,
                        int b1, int b2, const sim_scalar_t pos1[3], const sim_scalar_t pos2[3],
                        sim_scalar_t* jac1p, sim_scalar_t* jac2p, sim_scalar_t* jacdifp,
                        sim_scalar_t* jac1r, sim_scalar_t* jac2r, sim_scalar_t* jacdifr, int issparse);

// dense or sparse weighted sum of multiple body Jacobians at same point
int sim_jacSum(const sim_model_t* m, sim_data_t* d, int* chain,
              int n, const int* body, const sim_scalar_t* weight,
              const sim_scalar_t point[3], sim_scalar_t* jac, int flg_rot);

// compute 3/6-by-nv Jacobian time derivative of global point attached to given body
SIM_API void sim_jacDot(const sim_model_t* m, const sim_data_t* d,
                     sim_scalar_t* jacp, sim_scalar_t* jacr, const sim_scalar_t point[3], int body);

// compute subtree angular momentum matrix
SIM_API void sim_angmomMat(const sim_model_t* m, sim_data_t* d, sim_scalar_t* mat, int body);


//-------------------------- coordinate transformation ---------------------------------------------

// compute object 6D velocity in object-centered frame, world/local orientation
SIM_API void sim_objectVelocity(const sim_model_t* m, const sim_data_t* d,
                             int objtype, int objid, sim_scalar_t res[6], int flg_local);

// compute object 6D acceleration in object-centered frame, world/local orientation
SIM_API void sim_objectAcceleration(const sim_model_t* m, const sim_data_t* d,
                                 int objtype, int objid, sim_scalar_t res[6], int flg_local);

// map from body local to global Cartesian coordinates
SIM_API void sim_local2Global(sim_data_t* d, sim_scalar_t xpos[3], sim_scalar_t xmat[9],
                           const sim_scalar_t pos[3], const sim_scalar_t quat[4],
                           int body, sim_byte_t sameframe);


//-------------------------- miscellaneous ---------------------------------------------------------

// extract 6D force:torque for one contact, in contact frame
SIM_API void sim_contactForce(const sim_model_t* m, const sim_data_t* d, int id, sim_scalar_t result[6]);

// count the number of length limit violations for tendon i (0, 1 or 2)
int tendonLimit(const sim_model_t* m, const sim_scalar_t* ten_length, int i);

// high-level warning function: count warnings in sim_data_t, print only the first time
SIM_API void sim_runtime_warning(sim_data_t* d, int warning, int info);


#ifdef __cplusplus
}
#endif

#endif  // SIMCORE_SRC_ENGINE_ENGINE_CORE_UTIL_H_

