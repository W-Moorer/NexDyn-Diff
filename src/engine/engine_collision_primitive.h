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

#ifndef SIMCORE_SRC_ENGINE_ENGINE_COLLISION_PRIMITIVE_H_
#define SIMCORE_SRC_ENGINE_ENGINE_COLLISION_PRIMITIVE_H_

#include <simcore/SIM_data.h>
#include <simcore/SIM_export.h>
#include <simcore/SIM_model.h>

// define and extract geom info
#define SIM_GETINFO \
    const sim_scalar_t* pos1  = d->geom_xpos + 3*g1; \
    const sim_scalar_t* mat1  = d->geom_xmat + 9*g1; \
    const sim_scalar_t* size1 = m->geom_size + 3*g1; \
    const sim_scalar_t* pos2  = d->geom_xpos + 3*g2; \
    const sim_scalar_t* mat2  = d->geom_xmat + 9*g2; \
    const sim_scalar_t* size2 = m->geom_size + 3*g2; \
    (void) size1; (void) size2;  // size1 and size2 are sometimes unused

#ifdef __cplusplus
extern "C" {
#endif

// raw collision functions (called by SIM_c_XXX)
int SIM_raw_SphereCapsule (sim_contact_t* con, sim_scalar_t margin,
                         const sim_scalar_t* pos1, const sim_scalar_t* mat1, const sim_scalar_t* size1,
                         const sim_scalar_t* pos2, const sim_scalar_t* mat2, const sim_scalar_t* size2);
int SIM_raw_CapsuleCapsule(sim_contact_t* con, sim_scalar_t margin,
                         const sim_scalar_t* pos1, const sim_scalar_t* mat1, const sim_scalar_t* size1,
                         const sim_scalar_t* pos2, const sim_scalar_t* mat2, const sim_scalar_t* size2);
int SIM_raw_CapsuleBox    (sim_contact_t* con, sim_scalar_t margin,
                         const sim_scalar_t* pos1, const sim_scalar_t* mat1, const sim_scalar_t* size1,
                         const sim_scalar_t* pos2, const sim_scalar_t* mat2, const sim_scalar_t* size2);
int SIM_raw_SphereTriangle(sim_contact_t* con, sim_scalar_t margin,
                         const sim_scalar_t* s, sim_scalar_t rs,
                         const sim_scalar_t* t1, const sim_scalar_t* t2, const sim_scalar_t* t3, sim_scalar_t rt);
int SIM_raw_BoxTriangle(sim_contact_t* con, sim_scalar_t margin, const sim_scalar_t* pos,
                      const sim_scalar_t* mat, const sim_scalar_t* size, const sim_scalar_t* t1,
                      const sim_scalar_t* t2, const sim_scalar_t* t3, sim_scalar_t rt);
int SIM_raw_CapsuleTriangle(sim_contact_t* con, sim_scalar_t margin, const sim_scalar_t* pos,
                          const sim_scalar_t* mat, const sim_scalar_t* size,
                          const sim_scalar_t* t1, const sim_scalar_t* t2, const sim_scalar_t* t3,
                          sim_scalar_t rt);

// plane collisions
SIM_API int SIM_c_PlaneSphere     (const sim_model_t* m, const sim_data_t* d,
                               sim_contact_t* con, int g1, int g2, sim_scalar_t margin);
SIM_API int SIM_c_PlaneCapsule    (const sim_model_t* m, const sim_data_t* d,
                               sim_contact_t* con, int g1, int g2, sim_scalar_t margin);
SIM_API int SIM_c_PlaneCylinder   (const sim_model_t* m, const sim_data_t* d,
                               sim_contact_t* con, int g1, int g2, sim_scalar_t margin);
SIM_API int SIM_c_PlaneBox        (const sim_model_t* m, const sim_data_t* d,
                               sim_contact_t* con, int g1, int g2, sim_scalar_t margin);

// sphere and capsule collisions
SIM_API int SIM_c_SphereSphere    (const sim_model_t* m, const sim_data_t* d,
                               sim_contact_t* con, int g1, int g2, sim_scalar_t margin);
SIM_API int SIM_c_SphereCapsule   (const sim_model_t* m, const sim_data_t* d,
                               sim_contact_t* con, int g1, int g2, sim_scalar_t margin);
SIM_API int SIM_c_SphereCylinder  (const sim_model_t* m, const sim_data_t* d,
                               sim_contact_t* con, int g1, int g2, sim_scalar_t margin);
SIM_API int SIM_c_CapsuleCapsule  (const sim_model_t* m, const sim_data_t* d,
                               sim_contact_t* con, int g1, int g2, sim_scalar_t margin);

// box collisions: from engine_collision_box.c
SIM_API int SIM_c_CapsuleBox      (const sim_model_t* m, const sim_data_t* d,
                               sim_contact_t* con, int g1, int g2, sim_scalar_t margin);
SIM_API int SIM_c_SphereBox       (const sim_model_t* m, const sim_data_t* d,
                               sim_contact_t* con, int g1, int g2, sim_scalar_t margin);
SIM_API int SIM_c_BoxBox          (const sim_model_t* m, const sim_data_t* d,
                               sim_contact_t* con, int g1, int g2, sim_scalar_t margin);

#ifdef __cplusplus
}
#endif
#endif  // SIMCORE_SRC_ENGINE_ENGINE_COLLISION_PRIMITIVE_H_
