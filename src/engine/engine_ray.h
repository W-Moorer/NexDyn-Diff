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

#ifndef SIMCORE_SRC_ENGINE_ENGINE_RAY_H_
#define SIMCORE_SRC_ENGINE_ENGINE_RAY_H_

#include <simcore/SIM_data.h>
#include <simcore/SIM_export.h>
#include <simcore/SIM_model.h>

#ifdef __cplusplus
extern "C" {
#endif

SIM_API void sim_math_multiRayPrepare(const sim_model_t* m, const sim_data_t* d,
                               const sim_scalar_t pnt[3], const sim_scalar_t ray_xmat[9],
                               const sim_byte_t* geomgroup, sim_byte_t flg_static,
                               int bodyexclude, sim_scalar_t cutoff, sim_scalar_t* geom_ba,
                               int* geom_eliminate);

// intersect multiple rays emanating from a single source, compute normals if given
//  similar semantics to sim_ray, but vec, normal and dist are arrays
SIM_API void sim_multiRay(const sim_model_t* m, sim_data_t* d, const sim_scalar_t pnt[3], const sim_scalar_t* vec,
                       const sim_byte_t* geomgroup, sim_byte_t flg_static, int bodyexclude,
                       int* geomid, sim_scalar_t* dist, sim_scalar_t* normal, int nray, sim_scalar_t cutoff);


// intersect ray (pnt+x*vec, x>=0) with visible geoms, except geoms on bodyexclude
//  return geomid and distance (x) to nearest surface, or -1 if no intersection
//  geomgroup and flg_static control group/static filtering; geomgroup==NULL skips group exclusion
SIM_API sim_scalar_t sim_ray(const sim_model_t* m, const sim_data_t* d, const sim_scalar_t pnt[3], const sim_scalar_t vec[3],
                    const sim_byte_t* geomgroup, sim_byte_t flg_static, int bodyexclude,
                    int geomid[1], sim_scalar_t normal[3]);

// intersect ray with hfield, compute normal if given
SIM_API sim_scalar_t sim_rayHfield(const sim_model_t* m, const sim_data_t* d, int geomid,
                          const sim_scalar_t pnt[3], const sim_scalar_t vec[3], sim_scalar_t normal[3]);

// intersect ray with triangle
SIM_API sim_scalar_t ray_triangle(sim_scalar_t v[][3], const sim_scalar_t lpnt[3], const sim_scalar_t lvec[3],
                          const sim_scalar_t b0[3], const sim_scalar_t b1[3], sim_scalar_t normal[3]);

// intersect ray with mesh, compute normal if given
SIM_API sim_scalar_t sim_rayMesh(const sim_model_t* m, const sim_data_t* d, int geomid,
                        const sim_scalar_t pnt[3], const sim_scalar_t vec[3], sim_scalar_t normal[3]);

// intersect ray with primitive geom, no meshes or hfields, compute normal if given
SIM_API sim_scalar_t sim_math_rayGeom(const sim_scalar_t pos[3], const sim_scalar_t mat[9], const sim_scalar_t size[3],
                         const sim_scalar_t pnt[3], const sim_scalar_t vec[3], int geomtype,
                         sim_scalar_t normal[3]);

// intersect ray with flex, return nearest vertex id, compute normal if given
SIM_API sim_scalar_t sim_rayFlex(const sim_model_t* m, const sim_data_t* d, int flex_layer,
                        sim_byte_t flg_vert, sim_byte_t flg_edge, sim_byte_t flg_face,
                        sim_byte_t flg_skin, int flexid, const sim_scalar_t pnt[3],
                        const sim_scalar_t vec[3], int vertid[1], sim_scalar_t normal[3]);

// intersect ray with skin, return nearest vertex id
SIM_API sim_scalar_t sim_math_raySkin(int nface, int nvert, const int* face, const float* vert,
                         const sim_scalar_t pnt[3], const sim_scalar_t vec[3], int vertid[1]);

#ifdef __cplusplus
}
#endif

#endif  // SIMCORE_SRC_ENGINE_ENGINE_RAY_H_
