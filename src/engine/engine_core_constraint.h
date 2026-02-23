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

#ifndef SIMCORE_SRC_ENGINE_ENGINE_CORE_CONSTRAINT_H_
#define SIMCORE_SRC_ENGINE_ENGINE_CORE_CONSTRAINT_H_

#include <simcore/SIM_data.h>
#include <simcore/SIM_export.h>
#include <simcore/SIM_model.h>

#ifdef __cplusplus
extern "C" {
#endif


//-------------------------- Jacobian-related ------------------------------------------------------

// determine type of solver
SIM_API int sim_isDual(const sim_model_t* m);

// multiply Jacobian by vector
SIM_API void sim_mulJacVec(const sim_model_t* m, const sim_data_t* d, sim_scalar_t* res, const sim_scalar_t* vec);

// multiply JacobianT by vector
SIM_API void sim_mulJacTVec(const sim_model_t* m, const sim_data_t* d, sim_scalar_t* res, const sim_scalar_t* vec);


//-------------------------- utility functions -----------------------------------------------------

// assign/override solver reference parameters
void sim_assignRef(const sim_model_t* m, sim_scalar_t* target, const sim_scalar_t* source);

// assign/override solver impedance parameters
void sim_assignImp(const sim_model_t* m, sim_scalar_t* target, const sim_scalar_t* source);

// assign/clamp contact friction parameters
void sim_assignFriction(const sim_model_t* m, sim_scalar_t* target, const sim_scalar_t* source);

// assign/override geom margin
sim_scalar_t sim_assignMargin(const sim_model_t* m, sim_scalar_t source);

// add contact to d->contact list; return 0 if success; 1 if buffer full
SIM_API int sim_addContact(const sim_model_t* m, sim_data_t* d, const sim_contact_t* con);

//-------------------------- constraint instantiation ----------------------------------------------

// equality constraints
void sim_instantiateEquality(const sim_model_t* m, sim_data_t* d);

// frictional dofs and tendons
void sim_instantiateFriction(const sim_model_t* m, sim_data_t* d);

// joint and tendon limits
void sim_instantiateLimit(const sim_model_t* m, sim_data_t* d);

// frictionless and frictional contacts
void sim_instantiateContact(const sim_model_t* m, sim_data_t* d);

// compute Jacobian for contact, return number of DOFs affected
int sim_contactJacobian(const sim_model_t* m, sim_data_t* d, const sim_contact_t* con, int dim,
                       sim_scalar_t* jac, sim_scalar_t* jacdif, sim_scalar_t* jacdifp,
                       sim_scalar_t* jacdifr, sim_scalar_t* jac1p, sim_scalar_t* jac2p,
                       sim_scalar_t* jac1r, sim_scalar_t* jac2r, int* chain);


//------------------------ parameter computation/extraction ----------------------------------------

// compute efc_diagApprox
void sim_diagApprox(const sim_model_t* m, sim_data_t* d);

// compute efc_R, efc_D, efc_KDIP, adjust diagApprox
void sim_makeImpedance(const sim_model_t* m, sim_data_t* d);


//---------------------------- top-level API for constraint construction ---------------------------

// main driver: call all functions above
SIM_API void sim_makeConstraint(const sim_model_t* m, sim_data_t* d);

// compute efc_AR
SIM_API void sim_projectConstraint(const sim_model_t* m, sim_data_t* d);

// compute efc_vel, efc_aref
SIM_API void sim_referenceConstraint(const sim_model_t* m, sim_data_t* d);

// compute efc_state, efc_force
//  optional: cost(qacc) = s_hat(jar); cone Hessians
SIM_API void sim_constraintUpdate_impl(int ne, int nf, int nefc,
                                    const sim_scalar_t* D, const sim_scalar_t* R, const sim_scalar_t* floss,
                                    const sim_scalar_t* jar, const int* type, const int* id,
                                    sim_contact_t* contact, int* state, sim_scalar_t* force, sim_scalar_t cost[1],
                                    int flg_coneHessian);

// compute efc_state, efc_force, qfrc_constraint
// optional: cost(qacc) = s_hat(jar) where jar = Jac*qacc-aref; cone Hessians
SIM_API void sim_constraintUpdate(const sim_model_t* m, sim_data_t* d, const sim_scalar_t* jar,
                               sim_scalar_t cost[1], int flg_coneHessian);


#ifdef __cplusplus
}
#endif

#endif  // SIMCORE_SRC_ENGINE_ENGINE_CORE_CONSTRAINT_H_
