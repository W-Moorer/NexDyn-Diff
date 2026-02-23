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

#ifndef SIMCORE_SRC_ENGINE_ENGINE_COLLISION_SDF_H_
#define SIMCORE_SRC_ENGINE_ENGINE_COLLISION_SDF_H_

#include <simcore/SIM_data.h>
#include <simcore/SIM_export.h>
#include <simcore/SIM_model.h>
#include <simcore/SIM_plugin.h>
#include <simcore/SIM_tnum.h>

#ifdef __cplusplus
extern "C" {
#endif

// get sdf from geom id
SIM_API const SIM_pPlugin* SIM_c_getSDF(const sim_model_t* m, int id);

// signed distance function
SIM_API sim_scalar_t SIM_c_distance(const sim_model_t* m, const sim_data_t* d, const SIM_SDF* s, const sim_scalar_t x[3]);

// gradient of sdf
SIM_API void SIM_c_gradient(const sim_model_t* m, const sim_data_t* d, const SIM_SDF* s, sim_scalar_t gradient[3],
                        const sim_scalar_t x[3]);

// collision between a height field and a signed distance field
int SIM_c_HFieldSDF(const sim_model_t* m, const sim_data_t* d, sim_contact_t* con, int g1, int g2, sim_scalar_t margin);

// collision between a mesh and a signed distance field
int SIM_c_MeshSDF(const sim_model_t* m, const sim_data_t* d, sim_contact_t* con, int g1, int g2, sim_scalar_t margin);

// collision between two signed distance fields
int SIM_c_SDF(const sim_model_t* m, const sim_data_t* d, sim_contact_t* con, int g1, int g2, sim_scalar_t margin);

#ifdef __cplusplus
}
#endif
#endif  // SIMCORE_SRC_ENGINE_ENGINE_COLLISION_SDF_H_
