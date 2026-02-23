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

#ifndef SIMCORE_SRC_ENGINE_ENGINE_COLLISION_CONVEX_H_
#define SIMCORE_SRC_ENGINE_ENGINE_COLLISION_CONVEX_H_

// libCCD has an unconditional `#define _CRT_SECURE_NO_WARNINGS` on Windows.
// TODO(stunya): Remove once https://github.com/danfis/libccd/pull/77 is merged
#ifdef _CRT_SECURE_NO_WARNINGS
#undef _CRT_SECURE_NO_WARNINGS
#endif

#include <ccd/vec3.h>

#include <simcore/SIM_export.h>
#include <simcore/SIM_data.h>
#include <simcore/SIM_model.h>
#include <simcore/SIM_tnum.h>

// minimum number of vertices to use hill-climbing in mesh support
#define SIM_MESH_HILLCLIMB_MIN 10

#ifdef __cplusplus
extern "C" {
#endif

// internal object type for convex collision detection
struct _mjCCDObj {
  const sim_model_t* model;
  const sim_data_t* data;
  int geom;
  int geom_type;
  int vertindex;
  int meshindex;
  int flex;
  int elem;
  int vert;
  sim_scalar_t margin;
  sim_scalar_t rotate[4];
  void (*center)(sim_scalar_t res[3], const struct _mjCCDObj* obj);
  void (*support)(sim_scalar_t res[3], struct _mjCCDObj* obj, const sim_scalar_t dir[3]);

  // for hfield
  sim_scalar_t prism[6][3];
  const sim_scalar_t* size;
  const float* hfield_data;
  int hfield_nrow;
  int hfield_ncol;
};
typedef struct _mjCCDObj sim_collision_ccd_object_t;

// initialize a CCD object
SIM_API void SIM_c_initCCDObj(sim_collision_ccd_object_t* obj, const sim_model_t* m, const sim_data_t* d, int g, sim_scalar_t margin);

// center function for convex collision algorithms
SIM_API void SIM_c_center(sim_scalar_t res[3], const sim_collision_ccd_object_t *obj);

// libccd center function
SIM_API void SIM_ccd_center(const void *obj, ccd_vec3_t *center);

// libccd support function
SIM_API void SIM_ccd_support(const void *obj, const ccd_vec3_t *dir, ccd_vec3_t *vec);

// support function for point
void SIM_c_pointSupport(sim_scalar_t res[3], sim_collision_ccd_object_t* obj, const sim_scalar_t dir[3]);

// support function for line (capsule)
void SIM_c_lineSupport(sim_scalar_t res[3], sim_collision_ccd_object_t* obj, const sim_scalar_t dir[3]);

// pairwise geom collision functions using ccd
int SIM_c_PlaneConvex(const sim_model_t* m, const sim_data_t* d,
                    sim_contact_t* con, int g1, int g2, sim_scalar_t margin);
int SIM_c_ConvexHField(const sim_model_t* m, const sim_data_t* d,
                     sim_contact_t* con, int g1, int g2, sim_scalar_t margin);
SIM_API int SIM_c_Convex(const sim_model_t* m, const sim_data_t* d,
                     sim_contact_t* con, int g1, int g2, sim_scalar_t margin);

// geom-elem or elem-elem or vert-elem collision function using ccd
int SIM_c_ConvexElem    (const sim_model_t* m, const sim_data_t* d, sim_contact_t* con,
                       int g1, int f1, int e1, int v1, int f2, int e2, sim_scalar_t margin);

// heightfield-elem collision function using ccd
int SIM_c_HFieldElem    (const sim_model_t* m, const sim_data_t* d, sim_contact_t* con,
                       int g, int f, int e, sim_scalar_t margin);

// fix contact frame normal
void SIM_c_fixNormal(const sim_model_t* m, const sim_data_t* d, sim_contact_t* con, int g1, int g2);

#ifdef __cplusplus
}
#endif
#endif  // SIMCORE_SRC_ENGINE_ENGINE_COLLISION_CONVEX_H_
