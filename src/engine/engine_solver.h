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

#ifndef SIMCORE_SRC_ENGINE_ENGINE_SOLVER_H_
#define SIMCORE_SRC_ENGINE_ENGINE_SOLVER_H_

#include <simcore/SIM_data.h>
#include <simcore/SIM_model.h>

//------------------------------ monolithic solvers ------------------------------------------------

// PGS solver
void sim_solPGS(const sim_model_t* m, sim_data_t* d, int maxiter);

// No Slip solver (modified PGS)
void sim_solNoSlip(const sim_model_t* m, sim_data_t* d, int maxiter);

// CG solver
void sim_solCG(const sim_model_t* m, sim_data_t* d, int maxiter);

// Newton solver
void sim_solNewton(const sim_model_t* m, sim_data_t* d, int maxiter);


//------------------------------ per-island solvers ------------------------------------------------

// CG solver
void sim_solCG_island(const sim_model_t* m, sim_data_t* d, int island, int maxiter);

// Newton entry point
void sim_solNewton_island(const sim_model_t* m, sim_data_t* d, int island, int maxiter);

#endif  // SIMCORE_SRC_ENGINE_ENGINE_SOLVER_H_
