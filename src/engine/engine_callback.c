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

#include "engine/engine_callback.h"

#include <simcore/SIM_data.h>

//------------------------- global callback pointers -----------------------------------------------

SIM_fGeneric SIM_cb_passive  = 0;
SIM_fGeneric SIM_cb_control  = 0;
SIM_fConFilt SIM_cb_contactfilter = 0;
SIM_fSensor SIM_cb_sensor    = 0;
SIM_fTime SIM_cb_time        = 0;
SIM_fAct SIM_cb_act_bias     = 0;
SIM_fAct SIM_cb_act_gain     = 0;
SIM_fAct SIM_cb_act_dyn      = 0;



// reset callbacks to defaults
void sim_resetCallbacks(void) {
  SIM_cb_passive  = 0;
  SIM_cb_control  = 0;
  SIM_cb_contactfilter = 0;
  SIM_cb_sensor   = 0;
  SIM_cb_time     = 0;
  SIM_cb_act_bias = 0;
  SIM_cb_act_gain = 0;
  SIM_cb_act_dyn  = 0;
}
