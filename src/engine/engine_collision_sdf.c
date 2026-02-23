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

#include "engine/engine_collision_sdf.h"

#include <math.h>
#include <stdio.h>

#include <simcore/SIM_data.h>
#include <simcore/SIM_model.h>
#include <simcore/SIM_san.h>  // IWYU pragma: keep
#include <simcore/SIM_tnum.h>
#include "engine/engine_collision_primitive.h"
#include "engine/engine_plugin.h"
#include "engine/engine_ray.h"
#include "engine/engine_util_blas.h"
#include "engine/engine_util_errmem.h"
#include "engine/engine_util_misc.h"
#include "engine/engine_util_spatial.h"


//---------------------------- interpolated sdf -------------------------------------------

sim_scalar_t boxProjection(sim_scalar_t point[3], const sim_scalar_t box[6]) {
  sim_scalar_t r[3] = {point[0] - box[0], point[1] - box[1], point[2] - box[2]};
  sim_scalar_t q[3] = {sim_math_abs(r[0]) - box[3], sim_math_abs(r[1]) - box[4],
                 sim_math_abs(r[2]) - box[5]};
  sim_scalar_t dist_sqr = 0;
  sim_scalar_t eps = 1e-6;

  // skip the projection if inside
  if (q[0] <= 0 && q[1] <= 0 && q[2] <= 0) {
    return sim_math_max(q[0], sim_math_max(q[1], q[2]));
  }

  // in-place projection inside the box if outside
  if ( q[0] >= 0 ) {
    dist_sqr += q[0] * q[0];
    point[0] -= r[0] > 0 ? (q[0]+eps) : -(q[0]+eps);
  }
  if ( q[1] >= 0 ) {
    dist_sqr += q[1] * q[1];
    point[1] -= r[1] > 0 ? (q[1]+eps) : -(q[1]+eps);
  }
  if ( q[2] >= 0 ) {
    dist_sqr += q[2] * q[2];
    point[2] -= r[2] > 0 ? (q[2]+eps) : -(q[2]+eps);
  }

  return sim_math_sqrt(dist_sqr);
}


// find the octree leaf containing the point p, return the index of the leaf and
// populate the weights of the interpolated function (if w is not null) and of
// its gradient (if dw is not null) using the vertices as degrees of freedom for
// trilinear interpolation.
static int findOct(sim_scalar_t w[8], sim_scalar_t dw[8][3], const sim_scalar_t* oct_aabb,
                   const int* oct_child, const sim_scalar_t p[3]) {
  int stack = 0;
  sim_scalar_t eps = 1e-8;
  int niter = 100;

  while (niter-- > 0) {
    int node = stack;
    sim_scalar_t vmin[3], vmax[3];

    if (node == -1) {  // SHOULD NOT OCCUR
      sim_error("Invalid node number");
      return -1;
    }

    for (int j = 0; j < 3; j++) {
      vmin[j] = oct_aabb[6*node+j] - oct_aabb[6*node+3+j];
      vmax[j] = oct_aabb[6*node+j] + oct_aabb[6*node+3+j];
    }

    // check if the point is inside the aabb of the octree node
    if (p[0] + eps < vmin[0] || p[0] - eps > vmax[0] ||
        p[1] + eps < vmin[1] || p[1] - eps > vmax[1] ||
        p[2] + eps < vmin[2] || p[2] - eps > vmax[2]) {
      continue;
    }

    sim_scalar_t coord[3] = {(p[0] - vmin[0]) / (vmax[0] - vmin[0]),
                       (p[1] - vmin[1]) / (vmax[1] - vmin[1]),
                       (p[2] - vmin[2]) / (vmax[2] - vmin[2])};

    // check if the node is a leaf
    if (oct_child[8*node+0] == -1 && oct_child[8*node+1] == -1 &&
        oct_child[8*node+2] == -1 && oct_child[8*node+3] == -1 &&
        oct_child[8*node+4] == -1 && oct_child[8*node+5] == -1 &&
        oct_child[8*node+6] == -1 && oct_child[8*node+7] == -1) {
      for (int j = 0; j < 8; j++) {
        if (w) {
          w[j] = (j & 1 ? coord[0] : 1 - coord[0]) *
                 (j & 2 ? coord[1] : 1 - coord[1]) *
                 (j & 4 ? coord[2] : 1 - coord[2]);
        }
        if (dw) {
          dw[j][0] = (j & 1 ? 1 : -1) *
                     (j & 2 ? coord[1] : 1 - coord[1]) *
                     (j & 4 ? coord[2] : 1 - coord[2]);
          dw[j][1] = (j & 1 ? coord[0] : 1 - coord[0]) *
                     (j & 2 ? 1 : -1) *
                     (j & 4 ? coord[2] : 1 - coord[2]);
          dw[j][2] = (j & 1 ? coord[0] : 1 - coord[0]) *
                     (j & 2 ? coord[1] : 1 - coord[1]) *
                     (j & 4 ? 1 : -1);
        }
      }
      return node;
    }

    // compute which of 8 children to visit next
    int x = coord[0] < .5 ? 0 : 1;
    int y = coord[1] < .5 ? 0 : 1;
    int z = coord[2] < .5 ? 0 : 1;
    stack = oct_child[8 * node + 4*z + 2*y + x];
  }

  sim_error("Node not found");  // SHOULD NOT OCCUR
  return -1;
}

// sdf
sim_scalar_t oct_distance(const sim_model_t* m, const sim_scalar_t p[3], int meshid) {
  int octadr = m->mesh_octadr[meshid];
  int* oct_child = m->oct_child + 8*octadr;
  sim_scalar_t* oct_aabb = m->oct_aabb + 6*octadr;
  sim_scalar_t* oct_coeff = m->oct_coeff + 8*octadr;

  if (octadr == -1) {
    SIM_ERROR("Octree not found in mesh %d", meshid);
    return 0;
  }

  sim_scalar_t w[8];
  sim_scalar_t sdf = 0;
  sim_scalar_t point[3] = {p[0], p[1], p[2]};
  sim_scalar_t boxDist = boxProjection(point, oct_aabb);
  int node = findOct(w, NULL, oct_aabb, oct_child, point);
  for (int i = 0; i < 8; ++i) {
    sdf += w[i] * oct_coeff[8*node + i];
  }
  return boxDist > 0 ? sdf + boxDist : sdf;
}


// gradient of sdf
void oct_gradient(const sim_model_t* m, sim_scalar_t grad[3], const sim_scalar_t point[3], int meshid) {
  sim_math_zero_3(grad);
  sim_scalar_t p[3] = {point[0], point[1], point[2]};

  int octadr = m->mesh_octadr[meshid];
  int* oct_child = m->oct_child + 8*octadr;
  sim_scalar_t* oct_aabb = m->oct_aabb + 6*octadr;
  sim_scalar_t* oct_coeff = m->oct_coeff + 8*octadr;

  if (octadr == -1) {
    SIM_ERROR("Octree not found in mesh %d", meshid);
  }

  // analytic in the interior
  if (boxProjection(p, oct_aabb) <= 0) {
    sim_scalar_t dw[8][3];
    int node = findOct(NULL, dw, oct_aabb, oct_child, p);
    for (int i = 0; i < 8; ++i) {
      grad[0] += dw[i][0] * oct_coeff[8*node + i];
      grad[1] += dw[i][1] * oct_coeff[8*node + i];
      grad[2] += dw[i][2] * oct_coeff[8*node + i];
    }
    return;
  }

  // finite difference in the exterior
  sim_scalar_t eps = 1e-8;
  sim_scalar_t dist0 = oct_distance(m, point, meshid);
  sim_scalar_t pointX[3] = {point[0]+eps, point[1], point[2]};
  sim_scalar_t distX = oct_distance(m, pointX, meshid);
  sim_scalar_t pointY[3] = {point[0], point[1]+eps, point[2]};
  sim_scalar_t distY = oct_distance(m, pointY, meshid);
  sim_scalar_t pointZ[3] = {point[0], point[1], point[2]+eps};
  sim_scalar_t distZ = oct_distance(m, pointZ, meshid);

  grad[0] = (distX - dist0) / eps;
  grad[1] = (distY - dist0) / eps;
  grad[2] = (distZ - dist0) / eps;
}


//---------------------------- primitives sdf ---------------------------------------------

static void radialField3d(sim_scalar_t field[3], const sim_scalar_t a[3], const sim_scalar_t x[3],
                          const sim_scalar_t size[3]) {
  field[0] = -size[0] / a[0];
  field[1] = -size[1] / a[1];
  field[2] = -size[2] / a[2];
  sim_math_normalize_3(field);

  // flip sign if necessary
  if (x[0] < 0) field[0] = -field[0];
  if (x[1] < 0) field[1] = -field[1];
  if (x[2] < 0) field[2] = -field[2];
}

static sim_scalar_t geomDistance(const sim_model_t* m, const sim_data_t* d, const SIM_pPlugin* p,
                           int i, const sim_scalar_t x[3], SIM_tGeom type) {
  sim_scalar_t a[3], b[3];
  const sim_scalar_t* size = m->geom_size+3*i;

  // see https://iquilezles.org/articles/distfunctions/
  switch (type) {
  case SIM_GEOM_PLANE:
    return x[2];

  case SIM_GEOM_SPHERE:
    return sim_math_norm3(x) - size[0];

  case SIM_GEOM_BOX:
    // compute shortest distance to box surface if outside, otherwise
    // intersect with a unit gradient that linearly rotates from radial to the face normals
    a[0] = sim_math_abs(x[0]) - size[0];
    a[1] = sim_math_abs(x[1]) - size[1];
    a[2] = sim_math_abs(x[2]) - size[2];
    if (a[0] >= 0 || a[1] >= 0 || a[2] >= 0) {
      b[0] = sim_math_max(a[0], 0);
      b[1] = sim_math_max(a[1], 0);
      b[2] = sim_math_max(a[2], 0);
      return sim_math_norm3(b) + sim_math_min(sim_math_max(a[0], sim_math_max(a[1], a[2])), 0);
    }
    radialField3d(b, a, x, size);
    sim_scalar_t t[3];
    t[0] = -a[0] / sim_math_abs(b[0]);
    t[1] = -a[1] / sim_math_abs(b[1]);
    t[2] = -a[2] / sim_math_abs(b[2]);
    return -sim_math_min(t[0], sim_math_min(t[1], t[2])) * sim_math_norm3(b);

  case SIM_GEOM_CAPSULE:
    a[0] = x[0];
    a[1] = x[1];
    a[2] = x[2] - sim_math_clip(x[2], -size[1], size[1]);
    return sim_math_norm3(a) - size[0];

  case SIM_GEOM_ELLIPSOID:
    a[0] = x[0] / size[0];
    a[1] = x[1] / size[1];
    a[2] = x[2] / size[2];
    b[0] = a[0] / size[0];
    b[1] = a[1] / size[1];
    b[2] = a[2] / size[2];
    sim_scalar_t k0 = sim_math_norm3(a);
    sim_scalar_t k1 = sim_math_norm3(b);
    return k0 * (k0 - 1.0) / k1;

  case SIM_GEOM_CYLINDER:
    a[0] = sim_math_sqrt(x[0]*x[0]+x[1]*x[1]) - size[0];
    a[1] = sim_math_abs(x[2]) - size[1];
    b[0] = sim_math_max(a[0], 0);
    b[1] = sim_math_max(a[1], 0);
    return sim_math_min(sim_math_max(a[0], a[1]), 0) + sim_math_norm(b, 2);

  case SIM_GEOM_SDF:
    if (p) {
      return p->sdf_distance(x, d, i);
    } else {
      return oct_distance(m, x, i);
    }

  case SIM_GEOM_MESH:
    if (m->mesh_octnum[i]) {
      return oct_distance(m, x, i);
    } else {
      sim_math_mul_mat_vec_3(a, d->geom_xmat + 9 * i, x);
      sim_math_add_to_3(a, d->geom_xpos + 3 * i);
      sim_scalar_t dir[3] = {-a[0], -a[1], -a[2]};
      sim_scalar_t r = sim_math_norm3(dir);
      sim_scalar_t dist = sim_rayMesh(m, d, i, a, dir, NULL);
      if (dist > r) {
        sim_math_scale_3(dir, dir, -1);
        return -sim_rayMesh(m, d, i, a, dir, NULL);
      }
      return dist;
    }

  default:
    SIM_ERROR("sdf collisions not available for geom type %d", type);
    return 0;
  }
}


static void geomGradient(sim_scalar_t gradient[3], const sim_model_t* m, const sim_data_t* d,
                         const SIM_pPlugin* p, int i, const sim_scalar_t x[3],
                         SIM_tGeom type) {
  sim_scalar_t a[3], b[3], c, e;
  const sim_scalar_t* size = m->geom_size+3*i;

  // see https://iquilezles.org/articles/distfunctions/
  switch (type) {
  case SIM_GEOM_PLANE:
    sim_math_zero_3(gradient);
    gradient[2] = 1;
    break;

  case SIM_GEOM_SPHERE:
    sim_math_copy_3(gradient, x);
    c = sim_math_norm3(x);
    gradient[0] *= 1. / c;
    gradient[1] *= 1. / c;
    gradient[2] *= 1. / c;
    break;

  case SIM_GEOM_BOX:
    sim_math_zero_3(gradient);
    a[0] = sim_math_abs(x[0]) - size[0];
    a[1] = sim_math_abs(x[1]) - size[1];
    a[2] = sim_math_abs(x[2]) - size[2];
    int k = a[0] > a[1] ? 0 : 1;
    int l = a[2] > a[k] ? 2 : k;
    if (a[l] < 0) {
      radialField3d(gradient, a, x, size);
    } else {
      b[0] = sim_math_max(a[0], 0);
      b[1] = sim_math_max(a[1], 0);
      b[2] = sim_math_max(a[2], 0);
      c = sim_math_norm3(b);
      gradient[0] = a[0] > 0 ? b[0] / c * x[0] / sim_math_abs(x[0]) : 0;
      gradient[1] = a[1] > 0 ? b[1] / c * x[1] / sim_math_abs(x[1]) : 0;
      gradient[2] = a[2] > 0 ? b[2] / c * x[2] / sim_math_abs(x[2]) : 0;
    }
    break;

  case SIM_GEOM_CAPSULE:
    a[0] = x[0];
    a[1] = x[1];
    a[2] = x[2] - sim_math_clip(x[2], -size[1], size[1]);
    c = sim_math_norm3(a);
    gradient[0] = a[0] / c;
    gradient[1] = a[1] / c;
    gradient[2] = a[2] / c;
    break;

  case SIM_GEOM_ELLIPSOID:
    a[0] = x[0] / size[0];
    a[1] = x[1] / size[1];
    a[2] = x[2] / size[2];
    b[0] = a[0] / size[0];
    b[1] = a[1] / size[1];
    b[2] = a[2] / size[2];
    sim_scalar_t k0 = sim_math_norm3(a);
    sim_scalar_t k1 = sim_math_norm3(b);
    sim_scalar_t invK0 = 1. / k0;
    sim_scalar_t invK1 = 1. / k1;
    sim_scalar_t gk0[3] = {b[0]*invK0, b[1]*invK0, b[2]*invK0};
    sim_scalar_t gk1[3] = {b[0]*invK1/(size[0]*size[0]),
                     b[1]*invK1/(size[1]*size[1]),
                     b[2]*invK1/(size[2]*size[2])};
    sim_scalar_t df_dk0 = (2.*k0 - 1.) * invK1;
    sim_scalar_t df_dk1 = k0*(k0 - 1.) * invK1 * invK1;
    gradient[0] = gk0[0]*df_dk0 - gk1[0]*df_dk1;
    gradient[1] = gk0[1]*df_dk0 - gk1[1]*df_dk1;
    gradient[2] = gk0[2]*df_dk0 - gk1[2]*df_dk1;
    sim_math_normalize_3(gradient);
    break;

  case SIM_GEOM_CYLINDER:
    c = sim_math_sqrt(x[0]*x[0]+x[1]*x[1]);
    e = sim_math_abs(x[2]);
    a[0] = c - size[0];
    a[1] = e - size[1];
    sim_scalar_t grada[3] = {x[0] / sim_math_max(c, 1. / SIM_MAXVAL),
                       x[1] / sim_math_max(c, 1. / SIM_MAXVAL),
                       x[2] / sim_math_max(e, 1. / SIM_MAXVAL)};
    int j = a[0] > a[1] ? 0 : 1;
    if (a[j] < 0) {
      gradient[0] = j == 0 ? grada[0] : 0;
      gradient[1] = j == 0 ? grada[1] : 0;
      gradient[2] = j == 1 ? grada[2] : 0;
    } else {
      b[0] = sim_math_max(a[0], 0);
      b[1] = sim_math_max(a[1], 0);
      sim_scalar_t bnorm = sim_math_max(sim_math_norm(b, 2), 1./SIM_MAXVAL);
      gradient[0] = grada[0] * b[0] / bnorm;
      gradient[1] = grada[1] * b[0] / bnorm;
      gradient[2] = grada[2] * b[1] / bnorm;
    }
    break;

  case SIM_GEOM_SDF:
    if (p) {
      p->sdf_gradient(gradient, x, d, i);
    } else {
      oct_gradient(m, gradient, x, i);
    }
    break;

  case SIM_GEOM_MESH:
    if (m->mesh_octnum[i]) {
      oct_gradient(m, gradient, x, i);
    } else {
      sim_math_mul_mat_vec_3(a, d->geom_xmat+9*i, x);
      sim_math_add_to_3(a, d->geom_xpos+3*i);
      sim_scalar_t dir[3] = {-a[0], -a[1], -a[2]};
      sim_scalar_t r = sim_math_norm3(dir);
      sim_scalar_t dist = sim_rayMesh(m, d, i, a, dir, NULL);
      gradient[0] = dist > r ? 1 : -1;
      gradient[1] = dist > r ? 1 : -1;
      gradient[2] = dist > r ? 1 : -1;
    }
    break;

  default:
    SIM_ERROR("sdf collisions not available for geom type %d", type);
  }
}


//---------------------------- helper functions -------------------------------------------

// signed distance function
sim_scalar_t SIM_c_distance(const sim_model_t* m, const sim_data_t* d, const SIM_SDF* s, const sim_scalar_t x[3]) {
  sim_scalar_t y[3];

  switch (s->type) {
  case SIM_SDFTYPE_SINGLE:
    return geomDistance(m, d, s->plugin[0], s->id[0], x, s->geomtype[0]);

  case SIM_SDFTYPE_INTERSECTION:
    sim_math_mul_mat_vec_3(y, s->relmat, x);
    sim_math_add_to_3(y, s->relpos);
    return sim_math_max(geomDistance(m, d, s->plugin[0], s->id[0], x, s->geomtype[0]),
                   geomDistance(m, d, s->plugin[1], s->id[1], y, s->geomtype[1]));

  case SIM_SDFTYPE_MIDSURFACE:
    sim_math_mul_mat_vec_3(y, s->relmat, x);
    sim_math_add_to_3(y, s->relpos);
    return geomDistance(m, d, s->plugin[0], s->id[0], x, s->geomtype[0]) -
           geomDistance(m, d, s->plugin[1], s->id[1], y, s->geomtype[1]);

  case SIM_SDFTYPE_COLLISION:
    sim_math_mul_mat_vec_3(y, s->relmat, x);
    sim_math_add_to_3(y, s->relpos);
    sim_scalar_t A = geomDistance(m, d, s->plugin[0], s->id[0], x, s->geomtype[0]);
    sim_scalar_t B = geomDistance(m, d, s->plugin[1], s->id[1], y, s->geomtype[1]);
    return A + B + sim_math_abs(sim_math_max(A, B));

  default:
    SIM_ERROR("SDF type not available");
    return 0;
  }
}


// gradient of sdf
void SIM_c_gradient(const sim_model_t* m, const sim_data_t* d, const SIM_SDF* s,
                  sim_scalar_t gradient[3], const sim_scalar_t x[3]) {
  sim_scalar_t y[3];
  const sim_scalar_t* point[2] = {x, y};
  sim_scalar_t grad1[3], grad2[3];

  switch (s->type) {
  case SIM_SDFTYPE_INTERSECTION:
    sim_math_mul_mat_vec_3(y, s->relmat, x);
    sim_math_add_to_3(y, s->relpos);
    int i = geomDistance(m, d, s->plugin[0], s->id[0], x, s->geomtype[0]) >
            geomDistance(m, d, s->plugin[1], s->id[1], y, s->geomtype[1]) ? 0 : 1;
    geomGradient(gradient, m, d, s->plugin[i], s->id[i], point[i], s->geomtype[i]);
    if (i == 1) {
      sim_math_mulMatTVec3(gradient, s->relmat, gradient);
    }
    break;

  case SIM_SDFTYPE_MIDSURFACE:
    sim_math_mul_mat_vec_3(y, s->relmat, x);
    sim_math_add_to_3(y, s->relpos);
    geomGradient(grad1, m, d, s->plugin[0], s->id[0], x, s->geomtype[0]);
    sim_math_normalize_3(grad1);
    geomGradient(grad2, m, d, s->plugin[1], s->id[1], y, s->geomtype[1]);
    sim_math_mulMatTVec3(grad2, s->relmat, grad2);
    sim_math_normalize_3(grad2);
    sim_math_sub_3(gradient, grad1, grad2);
    sim_math_normalize_3(gradient);
    break;

  case SIM_SDFTYPE_COLLISION:
    sim_math_mul_mat_vec_3(y, s->relmat, x);
    sim_math_add_to_3(y, s->relpos);
    sim_scalar_t A = geomDistance(m, d, s->plugin[0], s->id[0], x, s->geomtype[0]);
    sim_scalar_t B = geomDistance(m, d, s->plugin[1], s->id[1], y, s->geomtype[1]);
    geomGradient(grad1, m, d, s->plugin[0], s->id[0], x, s->geomtype[0]);
    geomGradient(grad2, m, d, s->plugin[1], s->id[1], y, s->geomtype[1]);
    sim_math_mulMatTVec3(grad2, s->relmat, grad2);
    gradient[0] = grad1[0] + grad2[0];
    gradient[1] = grad1[1] + grad2[1];
    gradient[2] = grad1[2] + grad2[2];
    sim_math_add_to_scale_3(gradient, A > B ? grad1 : grad2, sim_math_max(A, B) > 0 ? 1 : -1);
    break;

  case SIM_SDFTYPE_SINGLE:
    geomGradient(gradient, m, d, s->plugin[0], s->id[0], point[0], s->geomtype[0]);
    break;
  default:
    SIM_ERROR("SDF type not available");
  }
}


// get sdf from geom id
const SIM_pPlugin* SIM_c_getSDF(const sim_model_t* m, int id) {
  int instance = m->geom_plugin[id];
  const int nslot = sim_plugin_pluginCount();
  const int slot = m->plugin[instance];
  const SIM_pPlugin* sdf = sim_plugin_getPluginAtSlotUnsafe(slot, nslot);
  if (!sdf) SIM_ERROR("invalid plugin slot: %d", slot);
  if (!(sdf->capabilityflags & SIM_PLUGIN_SDF)) {
    SIM_ERROR("Plugin is not a signed distance field at slot %d", slot);
  }
  return sdf;
}


// map (pos12, mat12) as (xpos2, xmat2)^-1 \circ (xpos1, xmat1)
static void mapPose(const sim_scalar_t xpos1[3], const sim_scalar_t xquat1[4],
                    const sim_scalar_t xpos2[3], const sim_scalar_t xquat2[4],
                    sim_scalar_t pos12[3], sim_scalar_t mat12[9]) {
  sim_scalar_t negpos[3], negquat[4], quat12[4];
  sim_math_negPose(negpos, negquat, xpos2, xquat2);
  sim_math_mulPose(pos12, quat12, negpos, negquat, xpos1, xquat1);
  sim_math_quat2Mat(mat12, quat12);
}


//---------------------------- narrow phase -----------------------------------------------

// check if the collision point already exists
static int isknown(const sim_scalar_t* points, const sim_scalar_t x[3], int cnt) {
  for (int i = 0; i < cnt; i++) {
    if (sim_math_dist3(x, points + 3*i) < SIM_MINVAL) {
      return 1;
    }
  }
  return 0;
}


// adds candidate point to result
static int addContact(sim_scalar_t* points, sim_contact_t* con, const sim_scalar_t x[3],
                      const sim_scalar_t pos2[3], const sim_scalar_t quat2[4], sim_scalar_t dist,
                      int cnt, const sim_model_t* m, const SIM_SDF* s, sim_data_t* d) {
  // check if there is a collision
  if (dist > 0 || isknown(points, x, cnt)) {
    return cnt;
  } else {
    sim_math_copy_3(points+3*cnt, x);
  }

  // compute normal in local coordinates
  sim_scalar_t norm[3], vec[3];
  SIM_c_gradient(m, d, s, norm, x);
  sim_math_scale_3(norm, norm, -1);

  // construct contact
  con[cnt].dist = dist;
  sim_math_rotVecQuat(con[cnt].frame, norm, quat2);
  sim_math_zero_3(con[cnt].frame+3);
  sim_math_makeFrame(con[cnt].frame);
  sim_math_scale_3(vec, con[cnt].frame, -con[cnt].dist/2);
  sim_math_rotVecQuat(con[cnt].pos, x, quat2);
  sim_math_add_to_3(con[cnt].pos, pos2);
  sim_math_add_to_3(con[cnt].pos, vec);

  return cnt+1;
}


// finds minimum using gradient descent
static sim_scalar_t stepGradient(sim_scalar_t x[3], const sim_model_t* m, const SIM_SDF* s,
                           sim_data_t* d, int niter) {
  const sim_scalar_t c = .1;       // reduction factor for the target decrease in the objective function
  const sim_scalar_t rho = .5;     // reduction factor for the gradient scaling (alpha)
  const sim_scalar_t amin = 1e-4;  // minimum value for alpha
  sim_scalar_t dist = SIM_MAXVAL;

  for (int step=0; step < niter; step++) {
    sim_scalar_t grad[3];
    sim_scalar_t alpha = 2.;  // initial line search factor scaling the gradient
                        // the units of the gradient depend on s->type

    // evaluate gradient
    SIM_c_gradient(m, d, s, grad, x);

    // sanity check
    if (isnan(grad[0]) || grad[0] > SIM_MAXVAL || grad[0] < -SIM_MAXVAL ||
        isnan(grad[1]) || grad[1] > SIM_MAXVAL || grad[1] < -SIM_MAXVAL ||
        isnan(grad[2]) || grad[2] > SIM_MAXVAL || grad[2] < -SIM_MAXVAL) {
      return SIM_MAXVAL;
    }

    // save current solution
    sim_scalar_t x0[] = {x[0], x[1], x[2]};

    // evaluate distance
    sim_scalar_t dist0 = SIM_c_distance(m, d, s, x0);
    sim_scalar_t wolfe = - c * alpha * sim_math_dot_3(grad, grad);

    // backtracking line search
    do {
      alpha *= rho;
      wolfe *= rho;
      sim_math_addScl3(x, x0, grad, -alpha);
      dist = SIM_c_distance(m, d, s, x);
    } while (alpha > amin && dist - dist0 > wolfe);

    // if no improvement, early stop
    if (dist0 < dist) {
      return dist;
    }
  }

  // the distance will be used for the contact creation
  return dist;
}


//------------------------------ collision functions -----------------------------------------------

// collision between a height field and a signed distance field
int SIM_c_HFieldSDF(const sim_model_t* m, const sim_data_t* d, sim_contact_t* con, int g1, int g2, sim_scalar_t margin) {
  sim_warning("HField vs SDF collision not yet supported!");
  return 0;
}


// collision between a mesh and a signed distance field
int SIM_c_MeshSDF(const sim_model_t* m, const sim_data_t* d, sim_contact_t* con, int g1, int g2, sim_scalar_t margin) {
  return SIM_c_SDF(m, d, con, g1, g2, margin);
}


// collision between two SDFs
int SIM_c_SDF(const sim_model_t* m, const sim_data_t* d, sim_contact_t* con, int g1, int g2, sim_scalar_t margin) {
  SIM_GETINFO;
  size1 = m->geom_aabb + 6*g1;
  size2 = m->geom_aabb + 6*g2;

  int cnt = 0;
  sim_scalar_t x[3], y[3], dist, vec1[3], vec2[3];
  sim_scalar_t aabb1[6] = {SIM_MAXVAL, SIM_MAXVAL, SIM_MAXVAL, -SIM_MAXVAL, -SIM_MAXVAL, -SIM_MAXVAL};
  sim_scalar_t aabb2[6] = {SIM_MAXVAL, SIM_MAXVAL, SIM_MAXVAL, -SIM_MAXVAL, -SIM_MAXVAL, -SIM_MAXVAL};
  sim_scalar_t aabb[6]  = {SIM_MAXVAL, SIM_MAXVAL, SIM_MAXVAL, -SIM_MAXVAL, -SIM_MAXVAL, -SIM_MAXVAL};

  // second geom must be an SDF
  if (m->geom_type[g2] != SIM_GEOM_SDF) {
    SIM_ERROR("geom is not an SDF");
  }

  // compute transformations from/to g1 to/from g2
  sim_scalar_t quat1[4], quat2[4];
  sim_scalar_t offset21[3], rotation21[9], rotation12[9];
  sim_scalar_t offset12[3], offset2[3], rotation2[9];
  sim_math_mat2Quat(quat1, mat1);
  sim_math_mat2Quat(quat2, mat2);
  mapPose(pos1, quat1, pos1, quat1, offset2, rotation2);
  mapPose(pos2, quat2, pos1, quat1, offset21, rotation21);
  mapPose(pos1, quat1, pos2, quat2, offset12, rotation12);

  // axis-aligned bounding boxes in g1 frame
  for (int i=0; i < 8; i++) {
    vec1[0] = (i&1 ? size1[0]+size1[3] : size1[0]-size1[3]);
    vec1[1] = (i&2 ? size1[1]+size1[4] : size1[1]-size1[4]);
    vec1[2] = (i&4 ? size1[2]+size1[5] : size1[2]-size1[5]);

    vec2[0] = (i&1 ? size2[0]+size2[3] : size2[0]-size2[3]);
    vec2[1] = (i&2 ? size2[1]+size2[4] : size2[1]-size2[4]);
    vec2[2] = (i&4 ? size2[2]+size2[5] : size2[2]-size2[5]);

    sim_math_mul_mat_vec_3(vec2, rotation21, vec2);
    sim_math_add_to_3(vec2, offset21);

    for (int k=0; k < 3; k++) {
      aabb1[0+k] = sim_math_min(aabb1[0+k], vec1[k]);
      aabb1[3+k] = sim_math_max(aabb1[3+k], vec1[k]);
      aabb2[0+k] = sim_math_min(aabb2[0+k], vec2[k]);
      aabb2[3+k] = sim_math_max(aabb2[3+k], vec2[k]);
    }
  }

  // intersection of aabbs
  for (int k=0; k < 3; k++) {
    aabb[0+k] = sim_math_max(aabb1[0+k], aabb2[0+k]);
    aabb[3+k] = sim_math_min(aabb1[3+k], aabb2[3+k]);
  }

  // no intersection if max < min
  if (aabb[3] < aabb[0] || aabb[4] < aabb[1] || aabb[5] < aabb[2]) {
    return cnt;
  }

  // create sdf pointers
  int instance[2];
  const SIM_pPlugin* sdf_ptr[2];
  SIM_tGeom geomtypes[2] = {m->geom_type[g2], m->geom_type[g1]};

  instance[0] = m->geom_plugin[g2];
  sdf_ptr[0] = instance[0] == -1 ? NULL : SIM_c_getSDF(m, g2);

  // get sdf plugins
  if (m->geom_type[g1] == SIM_GEOM_SDF) {
    instance[1] = m->geom_plugin[g1];
    sdf_ptr[1] = instance[1] == -1 ? NULL : SIM_c_getSDF(m, g1);
  } else {
    instance[1] = g1;
    sdf_ptr[1] = NULL;
  }

  // reset visualization count
  if (sdf_ptr[0]) {
    sdf_ptr[0]->reset(m, NULL, (void*)(d->plugin_data[instance[0]]), instance[0]);
  }

  // copy into sdf
  SIM_SDF sdf;
  instance[0] = instance[0] == -1 ? m->geom_dataid[g2] : instance[0];
  instance[1] = instance[1] == -1 ? m->geom_dataid[g1] : instance[1];
  sdf.id = instance;
  sdf.relpos = offset21;
  sdf.relmat = rotation21;
  sdf.plugin = sdf_ptr;
  sdf.geomtype = geomtypes;

  // minimize sdf intersection
  sim_scalar_t contacts[3*SIM_MAXCONPAIR];

  int i = 0, j = 0;
  while (i < m->opt.sdf_initpoints) {
    x[0] = aabb[0] + (aabb[3]-aabb[0]) * sim_math_Halton(j, 2);
    x[1] = aabb[1] + (aabb[4]-aabb[1]) * sim_math_Halton(j, 3);
    x[2] = aabb[2] + (aabb[5]-aabb[2]) * sim_math_Halton(j, 5);

    sim_math_mul_mat_vec_3(y, rotation2, x);
    sim_math_add_to_3(y, offset2);

    sim_math_mul_mat_vec_3(x, rotation12, y);
    sim_math_add_to_3(x, offset12);

    j++;

    // here a criterion for rejecting points could be inserted

    i++;

    // start counters
    if (sdf_ptr[0]) {
      sdf_ptr[0]->compute(m, (sim_data_t*)d, instance[0], SIM_PLUGIN_SDF);
    }

    // gradient descent - we use a special function of the two SDF as objective
    sdf.type = SIM_SDFTYPE_COLLISION;
    dist = stepGradient(x, m, &sdf, (sim_data_t*)d, m->opt.sdf_iterations);

    // inexact SDFs can yield spurious collisions, filter them by projecting on the midsurface
    sdf.type = SIM_SDFTYPE_INTERSECTION;
    dist = stepGradient(x, m, &sdf, (sim_data_t*)d, 1);

    // contact point and normal - we use the midsurface where SDF1=SDF2 as zero level set
    sdf.type = SIM_SDFTYPE_MIDSURFACE;
    cnt = addContact(contacts, con, x, pos2, quat2, dist, cnt, m, &sdf, (sim_data_t*)d);

    // SHOULD NOT OCCUR
    if (cnt > SIM_MAXCONPAIR) {
      SIM_ERROR("too many contact points");
    }
  }

  return cnt;
}
