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

#ifndef SIMCORE_SRC_ENGINE_ENGINE_SENSOR_H_
#define SIMCORE_SRC_ENGINE_ENGINE_SENSOR_H_

#include <simcore/SIM_data.h>
#include <simcore/SIM_export.h>
#include <simcore/SIM_model.h>

#ifdef __cplusplus
extern "C" {
#endif

//-------------------------------- sensors ---------------------------------------------------------

// compute value for one sensor, write to sensordata, apply cutoff
void sim_computeSensor(const sim_model_t* m, sim_data_t* d, int i, sim_scalar_t* sensordata);

// position-dependent sensors
SIM_API void sim_sensorPos(const sim_model_t* m, sim_data_t* d);

// velocity-dependent sensors
SIM_API void sim_sensorVel(const sim_model_t* m, sim_data_t* d);

// acceleration/force-dependent sensors
SIM_API void sim_sensorAcc(const sim_model_t* m, sim_data_t* d);


//-------------------------------- energy ----------------------------------------------------------

// position-dependent energy (potential)
SIM_API void sim_energyPos(const sim_model_t* m, sim_data_t* d);

// velocity-dependent energy (kinetic)
SIM_API void sim_energyVel(const sim_model_t* m, sim_data_t* d);

#ifdef __cplusplus
}
#endif

#endif  // SIMCORE_SRC_ENGINE_ENGINE_SENSOR_H_
