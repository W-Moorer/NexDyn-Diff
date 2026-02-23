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

#ifndef SIMCORE_SRC_ENGINE_ENGINE_SLEEP_H_
#define SIMCORE_SRC_ENGINE_ENGINE_SLEEP_H_

#include <simcore/SIM_data.h>
#include <simcore/SIM_export.h>
#include <simcore/SIM_model.h>

#ifdef __cplusplus
extern "C" {
#endif

// compute sleeping arrays from tree_asleep, if flg_staticawake is set, treat static bodies as awake
void sim_updateSleepInit(const sim_model_t* m, sim_data_t* d, int flg_staticawake);

// compute {ntree,nbody,nv}_awake, {tree,body}_awake, {body,dof}_awake_ind from tree_asleep
SIM_API void sim_updateSleep(const sim_model_t* m, sim_data_t* d);

// return the first tree in the sleep cycle that starts at i, -1 if error
int sim_sleepCycle(const int* tree_asleep, int ntree, int i);

// return the first tree in the sleep cycle that starts at i, -1 if error
int sim_sleepCycle(const int* tree_asleep, int ntree, int i);

// wake tree i and its related island cycle, return number of woke trees
SIM_API int sim_wakeTree(int* tree_asleep, int ntree, int i, int wakeval);

// wake trees with nonzero velocity or external forces, return number of woke trees
int sim_wake(const sim_model_t* m, sim_data_t* d);

// wake sleeping trees that touch awake trees, return number of woke trees
int sim_wakeCollision(const sim_model_t* m, sim_data_t* d);

// wake sleeping trees with a constrained tendon to a waking tree, return number of woke trees
int sim_wakeTendon(const sim_model_t* m, sim_data_t* d);

// wake sleeping trees with an equality to a waking tree, return number of woke trees
int sim_wakeEquality(const sim_model_t* m, sim_data_t* d);

// put trees to sleep according to tolerance, return number of slept trees
int sim_sleep(const sim_model_t* m, sim_data_t* d);

// return sleep state of object i
SIM_API SIM_tSleepState sim_sleepState(const sim_model_t* m, const sim_data_t* d, sim_obj_t type, int i);

#ifdef __cplusplus
}
#endif

#endif  // SIMCORE_SRC_ENGINE_ENGINE_SLEEP_H_
