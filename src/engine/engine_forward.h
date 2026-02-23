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

#ifndef SIMCORE_SRC_ENGINE_ENGINE_FORWARD_H_
#define SIMCORE_SRC_ENGINE_ENGINE_FORWARD_H_

#include <simcore/SIM_data.h>
#include <simcore/SIM_export.h>
#include <simcore/SIM_model.h>

#ifdef __cplusplus
extern "C" {
#endif
// check positions, velocities, accelerations; reset if bad
SIM_API void sim_checkPos(const sim_model_t* m, sim_data_t* d);
SIM_API void sim_checkVel(const sim_model_t* m, sim_data_t* d);
SIM_API void sim_checkAcc(const sim_model_t* m, sim_data_t* d);


//-------------------------------- top-level API ---------------------------------------------------

// advance simulation: use control callback, no external force, RK4 available
SIM_API void sim_step(const sim_model_t* m, sim_data_t* d);

// advance simulation in two steps: before external force/control is set by user
SIM_API void sim_step1(const sim_model_t* m, sim_data_t* d);

// advance simulation in two steps: after external force/control is set by user
SIM_API void sim_step2(const sim_model_t* m, sim_data_t* d);

// forward dynamics
SIM_API void sim_forward(const sim_model_t* m, sim_data_t* d);

// forward dynamics with skip; skipstage is SIM_tStage
SIM_API void sim_forwardSkip(const sim_model_t* m, sim_data_t* d, int skipstage, int skipsensor);



//-------------------------------- integrators -----------------------------------------------------

// Runge Kutta explicit order-N integrator
SIM_API void sim_RungeKutta(const sim_model_t* m, sim_data_t* d, int N);

// Euler integrator, semi-implicit in velocity
SIM_API void sim_Euler(const sim_model_t* m, sim_data_t* d);

// Euler integrator, semi-implicit in velocity, possibly skipping factorisation
SIM_API void sim_EulerSkip(const sim_model_t* m, sim_data_t* d, int skipfactor);

// fully implicit in velocity
SIM_API void sim_implicit(const sim_model_t *m, sim_data_t *d);

// fully implicit in velocity, possibly skipping factorization
SIM_API void sim_implicitSkip(const sim_model_t *m, sim_data_t *d, int skipfactor);


//-------------------------------- solver components -----------------------------------------------

// all kinematics-like computations
SIM_API void sim_fwdKinematics(const sim_model_t* m, sim_data_t* d);

// computations that depend only on qpos
SIM_API void sim_fwdPosition(const sim_model_t* m, sim_data_t* d);

// computations that depend only on qpos and qvel
SIM_API void sim_fwdVelocity(const sim_model_t* m, sim_data_t* d);

// compute actuator force
SIM_API void sim_fwdActuation(const sim_model_t* m, sim_data_t* d);

// add up all non-constraint forces, compute qacc_smooth
SIM_API void sim_fwdAcceleration(const sim_model_t* m, sim_data_t* d);

// forward constraint
SIM_API void sim_fwdConstraint(const sim_model_t* m, sim_data_t* d);

#ifdef __cplusplus
}
#endif

#endif  // SIMCORE_SRC_ENGINE_ENGINE_FORWARD_H_
