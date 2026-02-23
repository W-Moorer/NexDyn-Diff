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

#ifndef SIMCORE_SRC_ENGINE_ENGINE_SETCONST_H_
#define SIMCORE_SRC_ENGINE_ENGINE_SETCONST_H_

#include <simcore/SIM_data.h>
#include <simcore/SIM_export.h>
#include <simcore/SIM_model.h>

#ifdef __cplusplus
extern "C" {
#endif

// Set constant fields of sim_model_t, corresponding to qpos0 configuration.
SIM_API void sim_setConst(sim_model_t* m, sim_data_t* d);

// Set actuator_lengthrange for specified actuator; return 1 if ok, 0 if error.
SIM_API int sim_setLengthRange(sim_model_t* m, sim_data_t* d, int index,
                            const SIM_LROpt* opt, char* error, int error_sz);

#ifdef __cplusplus
}
#endif

#endif  // SIMCORE_SRC_ENGINE_ENGINE_SETCONST_H_
