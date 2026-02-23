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

#ifndef SIMCORE_SRC_ENGINE_ENGINE_INVERSE_H_
#define SIMCORE_SRC_ENGINE_ENGINE_INVERSE_H_

#include <simcore/SIM_data.h>
#include <simcore/SIM_export.h>
#include <simcore/SIM_model.h>

#ifdef __cplusplus
extern "C" {
#endif

// inverse dynamics
SIM_API void sim_inverse(const sim_model_t* m, sim_data_t* d);

// Inverse dynamics with skip; skipstage is SIM_tStage.
SIM_API void sim_inverseSkip(const sim_model_t* m, sim_data_t* d,
                          int skipstage, int skipsensor);

// position-dependent computations
SIM_API void sim_invPosition(const sim_model_t* m, sim_data_t* d);

// velocity-dependent computations
SIM_API void sim_invVelocity(const sim_model_t* m, sim_data_t* d);

// inverse constraint solver
SIM_API void sim_invConstraint(const sim_model_t* m, sim_data_t* d);

// compare forward and inverse dynamics, without changing results of forward dynamics
SIM_API void sim_compareFwdInv(const sim_model_t* m, sim_data_t* d);

#ifdef __cplusplus
}
#endif

#endif  // SIMCORE_SRC_ENGINE_ENGINE_INVERSE_H_
