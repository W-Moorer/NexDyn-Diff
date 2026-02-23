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

#ifndef SIMCORE_SRC_ENGINE_ENGINE_INIT_H_
#define SIMCORE_SRC_ENGINE_ENGINE_INIT_H_

#include <simcore/core_api.h>

// set default options for length range computation
void sim_defaultLROpt(SIM_LROpt* opt);

// set model options to default values
void sim_defaultOption(SIM_Option* opt);

// set statistics to default values
void sim_defaultStatistic(SIM_Statistic* stat);

// set solver parameters to default values
void sim_defaultSolRefImp(sim_scalar_t* solref, sim_scalar_t* solimp);

#endif  // SIMCORE_SRC_ENGINE_ENGINE_INIT_H_
