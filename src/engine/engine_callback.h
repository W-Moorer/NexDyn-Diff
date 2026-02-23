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

#ifndef SIMCORE_SRC_ENGINE_ENGINE_CALLBACK_H_
#define SIMCORE_SRC_ENGINE_ENGINE_CALLBACK_H_

#include <simcore/SIM_data.h>
#include <simcore/SIM_export.h>

#ifdef __cplusplus
extern "C" {
#endif

// global callback function pointers
SIM_API extern SIM_fGeneric  SIM_cb_passive;
SIM_API extern SIM_fGeneric  SIM_cb_control;
SIM_API extern SIM_fConFilt  SIM_cb_contactfilter;
SIM_API extern SIM_fSensor   SIM_cb_sensor;
SIM_API extern SIM_fTime     SIM_cb_time;
SIM_API extern SIM_fAct      SIM_cb_act_bias;
SIM_API extern SIM_fAct      SIM_cb_act_gain;
SIM_API extern SIM_fAct      SIM_cb_act_dyn;


// reset callbacks to defaults
SIM_API void sim_resetCallbacks(void);

#ifdef __cplusplus
}
#endif
#endif  // SIMCORE_SRC_ENGINE_ENGINE_CALLBACK_H_
