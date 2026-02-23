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

#ifndef SIMCORE_SRC_ENGINE_ENGINE_PASSIVE_H_
#define SIMCORE_SRC_ENGINE_ENGINE_PASSIVE_H_

#include <simcore/SIM_data.h>
#include <simcore/SIM_export.h>
#include <simcore/SIM_model.h>

#ifdef __cplusplus
extern "C" {
#endif

//------------------------- passive forces ---------------------------------------------------------

// all passive forces
SIM_API void sim_passive(const sim_model_t* m, sim_data_t* d);

// compute interpolated flex state: xpos, vel, quat
SIM_API void sim_flexInterpState(const sim_model_t* m, sim_data_t* d, int f,
                              sim_scalar_t* xpos, sim_scalar_t* vel, sim_scalar_t* quat);


//------------------------- fluid models -----------------------------------------------------------

// fluid forces based on inertia-box approximation
void sim_inertiaBoxFluidModel(const sim_model_t* m, sim_data_t* d, int i);

// fluid forces based on ellipsoid approximation
void sim_ellipsoidFluidModel(const sim_model_t* m, sim_data_t* d, int bodyid);

// compute forces due to added mass (potential flow)
void sim_addedMassForces(
    const sim_scalar_t local_vels[6], const sim_scalar_t local_accels[6],
    sim_scalar_t fluid_density, const sim_scalar_t virtual_mass[3],
    const sim_scalar_t virtual_inertia[3], sim_scalar_t local_force[6]);

// compute forces due to viscous effects
void sim_viscousForces(
    const sim_scalar_t local_vels[6], sim_scalar_t fluid_density,
    sim_scalar_t fluid_viscosity, const sim_scalar_t size[3],
    sim_scalar_t magnus_lift_coef, sim_scalar_t kutta_lift_coef,
    sim_scalar_t blunt_drag_coef, sim_scalar_t slender_drag_coef,
    sim_scalar_t ang_drag_coef, sim_scalar_t local_force[6]);

void readFluidGeomInteraction(const sim_scalar_t* geom_fluid_coefs,
                              sim_scalar_t* geom_fluid_coef,
                              sim_scalar_t* blunt_drag_coef,
                              sim_scalar_t* slender_drag_coef,
                              sim_scalar_t* ang_drag_coef,
                              sim_scalar_t* kutta_lift_coef,
                              sim_scalar_t* magnus_lift_coef,
                              sim_scalar_t virtual_mass[3],
                              sim_scalar_t virtual_inertia[3]);

void writeFluidGeomInteraction (sim_scalar_t* geom_fluid_coefs,
                                const sim_scalar_t* geom_fluid_coef,
                                const sim_scalar_t* blunt_drag_coef,
                                const sim_scalar_t* slender_drag_coef,
                                const sim_scalar_t* ang_drag_coef,
                                const sim_scalar_t* kutta_lift_coef,
                                const sim_scalar_t* magnus_lift_coef,
                                const sim_scalar_t virtual_mass[3],
                                const sim_scalar_t virtual_inertia[3]);


#ifdef __cplusplus
}
#endif

#endif  // SIMCORE_SRC_ENGINE_ENGINE_PASSIVE_H_
