// Copyright 2022 DeepMind Technologies Limited
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

#ifndef SIMCORE_SRC_ENGINE_ENGINE_DERIVATIVE_FD_H_
#define SIMCORE_SRC_ENGINE_ENGINE_DERIVATIVE_FD_H_

#include <simcore/SIM_data.h>
#include <simcore/SIM_export.h>
#include <simcore/SIM_model.h>

#ifdef __cplusplus
extern "C" {
#endif

// centered finite difference approximation to SIM_d_smooth_vel
SIM_API void SIM_d_smooth_velFD(const sim_model_t* m, sim_data_t* d, sim_scalar_t eps);

// add forward finite difference approximation of (d qfrc_passive / d qvel) to qDeriv
SIM_API void SIM_d_passive_velFD(const sim_model_t* m, sim_data_t* d, sim_scalar_t eps);

// advance simulation using control callback, skipstage is SIM_tStage
SIM_API void sim_stepSkip(const sim_model_t* m, sim_data_t* d, int skipstage, int skipsensor);

// finite differenced transition matrices (control theory notation)
SIM_API void SIM_d_transitionFD(const sim_model_t* m, sim_data_t* d, sim_scalar_t eps, sim_byte_t centered,
                            sim_scalar_t* A, sim_scalar_t* B, sim_scalar_t* C, sim_scalar_t* D);

// finite differenced Jacobian of  (force, sensors) = sim_inverse(state, acceleration)
SIM_API void SIM_d_inverseFD(const sim_model_t* m, sim_data_t* d, sim_scalar_t eps, sim_byte_t flg_actuation,
                         sim_scalar_t *DfDq, sim_scalar_t *DfDv, sim_scalar_t *DfDa,
                         sim_scalar_t *DsDq, sim_scalar_t *DsDv, sim_scalar_t *DsDa,
                         sim_scalar_t *DmDq);

#ifdef __cplusplus
}
#endif

#endif  // SIMCORE_SRC_ENGINE_ENGINE_DERIVATIVE_FD_H_
