// Copyright 2023 DeepMind Technologies Limited
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

#ifndef SIMCORE_SRC_ENGINE_ENGINE_ISLAND_H_
#define SIMCORE_SRC_ENGINE_ENGINE_ISLAND_H_

#include <simcore/SIM_data.h>
#include <simcore/SIM_export.h>
#include <simcore/SIM_model.h>

#ifdef __cplusplus
extern "C" {
#endif


// find disjoint subgraphs ("islands") given sparse symmetric adjacency matrix
SIM_API int sim_floodFill(int* island, int nr, const int* rownnz, const int* rowadr, const int* colind,
                       int* stack);

//-------------------------- top-level API for island construction ---------------------------------

// discover islands:
//   nisland, island_dofadr, dof_island, dof_islandnext, island_efcadr, efc_island, efc_islandnext
SIM_API void sim_island(const sim_model_t* m, sim_data_t* d);

#ifdef __cplusplus
}
#endif

#endif  // SIMCORE_SRC_ENGINE_ENGINE_ISLAND_H_
