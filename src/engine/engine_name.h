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

#ifndef SIMCORE_SRC_ENGINE_ENGINE_NAME_H_
#define SIMCORE_SRC_ENGINE_ENGINE_NAME_H_

#include <stdint.h>

#include <simcore/SIM_export.h>
#include <simcore/SIM_model.h>

#ifdef __cplusplus
extern "C" {
#endif

//-------------------------- name functions --------------------------------------------------------

// get string hash, see http://www.cse.yorku.ca/~oz/hash.html
uint64_t sim_hashString(const char* s, uint64_t n);

// get id of object with the specified sim_obj_t type and name, returns -1 if id not found
SIM_API int sim_name2id(const sim_model_t* m, int type, const char* name);

// get name of object with the specified sim_obj_t type and id, returns NULL if name not found
SIM_API const char* sim_id2name(const sim_model_t* m, int type, int id);
#ifdef __cplusplus
}
#endif

#endif  // SIMCORE_SRC_ENGINE_ENGINE_NAME_H_
