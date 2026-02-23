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

#ifndef SIMCORE_SRC_ENGINE_ENGINE_CORE_SMOOTH_H_
#define SIMCORE_SRC_ENGINE_ENGINE_CORE_SMOOTH_H_

#include <simcore/SIM_data.h>
#include <simcore/SIM_export.h>
#include <simcore/SIM_model.h>

#ifdef __cplusplus
extern "C" {
#endif
//-------------------------- position --------------------------------------------------------------

// forward kinematics part 1: bodies
void sim_kinematics1(const sim_model_t* m, sim_data_t* d);

// forward kinematics part 2: body inertias, geoms and sites
void sim_kinematics2(const sim_model_t* m, sim_data_t* d);

// forward kinematics
SIM_API void sim_kinematics(const sim_model_t* m, sim_data_t* d);

// map inertias and motion dofs to global frame centered at CoM
SIM_API void sim_comPos(const sim_model_t* m, sim_data_t* d);

// compute camera and light positions and orientations
SIM_API void sim_camlight(const sim_model_t* m, sim_data_t* d);

// compute flex-related quantities
SIM_API void sim_flex(const sim_model_t* m, sim_data_t* d);

// compute tendon lengths, velocities and moment arms
SIM_API void sim_tendon(const sim_model_t* m, sim_data_t* d);

// compute time derivative of dense tendon Jacobian for one tendon
SIM_API void sim_tendonDot(const sim_model_t* m, sim_data_t* d, int id, sim_scalar_t* Jdot);

// compute actuator transmission lengths and moments
SIM_API void sim_transmission(const sim_model_t* m, sim_data_t* d);


//-------------------------- inertia ---------------------------------------------------------------

// composite rigid body inertia algorithm
SIM_API void sim_crb(const sim_model_t* m, sim_data_t* d);

// add tendon armature to M
SIM_API void sim_tendonArmature(const sim_model_t* m, sim_data_t* d);

// make inertia matrix
SIM_API void sim_makeM(const sim_model_t* m, sim_data_t* d);

// sparse L'*D*L factorizaton of inertia-like matrix M, assumed spd  (legacy implementation)
SIM_API void sim_factorI_legacy(const sim_model_t* m, sim_data_t* d, const sim_scalar_t* M,
                             sim_scalar_t* qLD, sim_scalar_t* qLDiagInv);

// sparse L'*D*L factorizaton of inertia-like matrix (only dofs in index, if given)
SIM_API void sim_factorI(sim_scalar_t* mat, sim_scalar_t* diaginv, int nv,
                      const int* rownnz, const int* rowadr, const int* colind, const int* index);

// sparse L'*D*L factorizaton of the inertia matrix M, assumed spd
SIM_API void sim_factorM(const sim_model_t* m, sim_data_t* d);

// sparse backsubstitution:  x = inv(L'*D*L)*x  (legacy implementation)
SIM_API void sim_solveLD_legacy(const sim_model_t* m, sim_scalar_t* x, int n,
                             const sim_scalar_t* qLD, const sim_scalar_t* qLDiagInv);

// in-place sparse backsubstitution (only dofs in index, if given):  x = inv(L'*D*L)*x
//  handle n vectors at once
SIM_API void sim_solveLD(sim_scalar_t* x, const sim_scalar_t* qLD, const sim_scalar_t* qLDiagInv, int nv, int n,
                      const int* rownnz, const int* rowadr, const int* colind, const int* index);

// sparse backsubstitution:  x = inv(L'*D*L)*y, use factorization in d
SIM_API void sim_solveM(const sim_model_t* m, sim_data_t* d, sim_scalar_t* x, const sim_scalar_t* y, int n);

// half of sparse backsubstitution:  x = sqrt(inv(D))*inv(L')*y
SIM_API void sim_solveM2(const sim_model_t* m, sim_data_t* d, sim_scalar_t* x, const sim_scalar_t* y,
                      const sim_scalar_t* sqrtInvD, int n);


//-------------------------- velocity --------------------------------------------------------------

// compute cvel, cdof_dot
SIM_API void sim_comVel(const sim_model_t* m, sim_data_t* d);

// subtree linear velocity and angular momentum
SIM_API void sim_subtreeVel(const sim_model_t* m, sim_data_t* d);


//-------------------------- RNE -------------------------------------------------------------------

// RNE: compute M(qpos)*qacc + C(qpos,qvel); flg_acc=0 removes inertial term
SIM_API void sim_rne(const sim_model_t* m, sim_data_t* d, int flg_acc, sim_scalar_t* result);

// RNE with complete data: compute cacc, cfrc_ext, cfrc_int
SIM_API void sim_rnePostConstraint(const sim_model_t* m, sim_data_t* d);


//-------------------------- tendon bias -----------------------------------------------------------

// add bias force due to tendon armature
SIM_API void sim_tendonBias(const sim_model_t* m, sim_data_t* d, sim_scalar_t* qfrc);

#ifdef __cplusplus
}
#endif

#endif  // SIMCORE_SRC_ENGINE_ENGINE_CORE_SMOOTH_H_
