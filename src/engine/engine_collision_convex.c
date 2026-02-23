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

#include "engine/engine_collision_convex.h"

#include <float.h>
#include <stddef.h>

#include <ccd/ccd.h>
#include <ccd/vec3.h>

#include <simcore/SIM_data.h>
#include <simcore/SIM_macro.h>
#include <simcore/SIM_model.h>
#include "engine/engine_collision_gjk.h"
#include "engine/engine_collision_primitive.h"
#include "engine/engine_memory.h"
#include "engine/engine_inline.h"
#include "engine/engine_util_blas.h"
#include "engine/engine_util_errmem.h"
#include "engine/engine_util_misc.h"
#include "engine/engine_util_spatial.h"

#define SIM_MINVAL2 (SIM_MINVAL * SIM_MINVAL)

// allocate callback for EPA in nativeccd
static void* ccd_allocate(void* data, size_t nbytes) {
  sim_markStack((sim_data_t*)data);
  return sim_stackAllocByte((sim_data_t*)data, nbytes, sizeof(sim_scalar_t));
}

// free callback for EPA in nativeccd
static void ccd_free(void* data, void* buffer) {
  sim_freeStack((sim_data_t*)data);
}

// call libccd or nativeccd to recover penetration info
static int SIM_c_penetration(const sim_model_t* m, sim_collision_ccd_object_t* obj1, sim_collision_ccd_object_t* obj2,
                           const ccd_t* ccd, ccd_real_t* depth, ccd_vec3_t* dir, ccd_vec3_t* pos) {
  // fallback to MPR
  if (SIM_DISABLED(SIM_DSBL_NATIVECCD)) {
    return ccdMPRPenetration(obj1, obj2, ccd, depth, dir, pos);
  }

  SIM_CCDConfig config;
  SIM_CCDStatus status;

  // set config
  config.max_iterations = ccd->max_iterations;
  config.tolerance = ccd->mpr_tolerance;
  config.max_contacts = 1;
  config.dist_cutoff = 0;  // no geom distances needed
  config.context = (void*)obj1->data;
  config.alloc = ccd_allocate;
  config.free = ccd_free;

  sim_scalar_t dist = SIM_c_ccd(&config, &status, obj1, obj2);
  if (dist < 0) {
    if (depth) *depth = -dist;
    if (dir) {
      sim_math_sub_3(dir->v, status.x1, status.x2);
      sim_math_normalize_3(dir->v);
    }
    if (pos) {
      pos->v[0] = 0.5 * (status.x1[0] + status.x2[0]);
      pos->v[1] = 0.5 * (status.x1[1] + status.x2[1]);
      pos->v[2] = 0.5 * (status.x1[2] + status.x2[2]);
    }
    return 0;
  }
  if (depth) *depth = 0;
  if (dir) sim_math_zero_3(dir->v);
  if (pos) sim_math_zero_3(dir->v);
  return 1;
}


// ccd center function
void SIM_ccd_center(const void *obj, ccd_vec3_t *center) {
  SIM_c_center(center->v, (const sim_collision_ccd_object_t*) obj);
}

// center function for convex collision algorithms
void SIM_c_center(sim_scalar_t res[3], const sim_collision_ccd_object_t *obj) {
  int g = obj->geom;
  int f = obj->flex;
  int e = obj->elem;
  int v = obj->vert;

  // return geom position
  if (g >= 0) {
    sim_math_internal_copy_3(res, obj->data->geom_xpos + 3*g);
  }

  // return flex element position
  else if (e >= 0) {
    sim_math_internal_copy_3(res, obj->data->flexelem_aabb + 6*(obj->model->flex_elemadr[f]+e));
  }

  // return flex vertex position
  else {
    sim_math_internal_copy_3(res, obj->data->flexvert_xpos + 3*(obj->model->flex_vertadr[f]+v));
  }
}


// prism center function
static void SIM_c_prism_center(sim_scalar_t res[3], const sim_collision_ccd_object_t* obj) {
  // compute mean
  sim_math_zero_3(res);
  for (int i=0; i < 6; i++) {
    sim_math_internal_addTo3(res, obj->prism[i]);
  }
  sim_math_scale_3(res, res, 1.0/6.0);
}


// ccd prism center function
static void SIM_ccd_prism_center(const void *obj, ccd_vec3_t *center) {
  SIM_c_prism_center(center->v, (const sim_collision_ccd_object_t*) obj);
}

// ------------------------------------ Support functions -----------------------------------------

// transform a vector from global to local frame
static inline void mulMatTVec3(sim_scalar_t res[3], const sim_scalar_t mat[9], const sim_scalar_t dir[3]) {
  // perform matT * dir
  res[0] = mat[0]*dir[0] + mat[3]*dir[1] + mat[6]*dir[2];
  res[1] = mat[1]*dir[0] + mat[4]*dir[1] + mat[7]*dir[2];
  res[2] = mat[2]*dir[0] + mat[5]*dir[1] + mat[8]*dir[2];
}


// transform a vector from local to global frame
static inline void localToGlobal(sim_scalar_t res[3], const sim_scalar_t mat[9], const sim_scalar_t dir[3],
                                 const sim_scalar_t pos[3]) {
  // perform mat * dir + pos
  res[0] = mat[0]*dir[0] + mat[1]*dir[1] + mat[2]*dir[2];
  res[1] = mat[3]*dir[0] + mat[4]*dir[1] + mat[5]*dir[2];
  res[2] = mat[6]*dir[0] + mat[7]*dir[1] + mat[8]*dir[2];
  res[0] += pos[0];
  res[1] += pos[1];
  res[2] += pos[2];
}


// point support function
void SIM_c_pointSupport(sim_scalar_t res[3], sim_collision_ccd_object_t* obj, const sim_scalar_t dir[3]) {
  const sim_scalar_t* pos = obj->data->geom_xpos + 3*obj->geom;
  sim_math_internal_copy_3(res, pos);
}


// sphere support function
static void SIM_c_sphereSupport(sim_scalar_t res[3], sim_collision_ccd_object_t* obj, const sim_scalar_t dir[3]) {
  const sim_model_t* m = obj->model;
  const sim_data_t* d = obj->data;

  // sphere data
  const sim_scalar_t* pos = d->geom_xpos + 3*obj->geom;
  sim_scalar_t radius = m->geom_size[3*obj->geom];

  res[0] = radius*dir[0] + pos[0];
  res[1] = radius*dir[1] + pos[1];
  res[2] = radius*dir[2] + pos[2];
}


// line support function (capsule)
void SIM_c_lineSupport(sim_scalar_t res[3], sim_collision_ccd_object_t* obj, const sim_scalar_t dir[3]) {
  const sim_model_t* m = obj->model;
  const sim_data_t* d = obj->data;

  // capsule data
  int i = 3*obj->geom;
  const sim_scalar_t* mat = d->geom_xmat + 3*i;
  const sim_scalar_t* pos = d->geom_xpos + i;
  sim_scalar_t length = m->geom_size[i+1];

  sim_scalar_t dot = mat[2]*dir[0] + mat[5]*dir[1] + mat[8]*dir[2];
  sim_scalar_t scl = dot >= 0 ? length : -length;

  // transform result to global frame
  res[0] = mat[2]*scl + pos[0];
  res[1] = mat[5]*scl + pos[1];
  res[2] = mat[8]*scl + pos[2];
}


// capsule support function
static void SIM_c_capsuleSupport(sim_scalar_t res[3], sim_collision_ccd_object_t* obj, const sim_scalar_t dir[3]) {
  const sim_model_t* m = obj->model;
  const sim_data_t* d = obj->data;

  // capsule data
  int i = 3*obj->geom;
  const sim_scalar_t* mat = d->geom_xmat + 3*i;
  const sim_scalar_t* pos = d->geom_xpos + i;
  sim_scalar_t radius = m->geom_size[i];
  sim_scalar_t length = m->geom_size[i+1];

  // rotate dir to geom local frame
  sim_scalar_t local_dir[3], local_supp[3];
  mulMatTVec3(local_dir, mat, dir);

  // start with sphere
  local_supp[0] = local_dir[0] * radius;
  local_supp[1] = local_dir[1] * radius;
  local_supp[2] = local_dir[2] * radius;

  // add cylinder contribution
  local_supp[2] += (local_dir[2] >= 0 ? length : -length);

  // transform result to global frame
  localToGlobal(res, mat, local_supp, pos);
}


// ellipsoid support function
static void SIM_c_ellipsoidSupport(sim_scalar_t res[3], sim_collision_ccd_object_t* obj, const sim_scalar_t dir[3]) {
  const sim_model_t* m = obj->model;
  const sim_data_t* d = obj->data;

  // ellipsoid data
  int i = 3*obj->geom;
  const sim_scalar_t* mat = d->geom_xmat + 3*i;
  const sim_scalar_t* pos = d->geom_xpos + i;
  const sim_scalar_t* size = m->geom_size + i;

  // rotate dir to geom local frame
  sim_scalar_t local_dir[3], local_supp[3];
  mulMatTVec3(local_dir, mat, dir);

  // find support point on unit sphere: scale dir by ellipsoid sizes
  local_supp[0] = local_dir[0] * size[0];
  local_supp[1] = local_dir[1] * size[1];
  local_supp[2] = local_dir[2] * size[2];

  sim_scalar_t norm2 = local_supp[0]*local_supp[0] + local_supp[1]*local_supp[1] + local_supp[2]*local_supp[2];

  // too small to normalize
  if (norm2 < SIM_MINVAL2) {
    res[0] = mat[0]*size[0] + pos[0];
    res[1] = mat[3]*size[0] + pos[1];
    res[2] = mat[6]*size[0] + pos[2];
    return;
  }

  // normalize and transform to ellipsoid
  sim_scalar_t norm_inv = 1/sim_math_sqrt(norm2);
  local_supp[0] *= norm_inv * size[0];
  local_supp[1] *= norm_inv * size[1];
  local_supp[2] *= norm_inv * size[2];

  // transform result to global frame
  localToGlobal(res, mat, local_supp, pos);
}


// cylinder support function
static void SIM_c_cylinderSupport(sim_scalar_t res[3], sim_collision_ccd_object_t* obj, const sim_scalar_t dir[3]) {
  const sim_model_t* m = obj->model;
  const sim_data_t* d = obj->data;

  // cylinder data
  int i = 3*obj->geom;
  const sim_scalar_t* mat = d->geom_xmat + 3*i;
  const sim_scalar_t* pos = d->geom_xpos + i;
  const sim_scalar_t* size = m->geom_size + i;

  // rotate dir to geom local frame
  sim_scalar_t local_dir[3], local_supp[3];
  mulMatTVec3(local_dir, mat, dir);

  sim_scalar_t n2 = local_dir[0]*local_dir[0] + local_dir[1]*local_dir[1];
  sim_scalar_t scl = n2 >= SIM_MINVAL2 ? size[0] / sim_math_sqrt(n2) : 0;
  local_supp[0] = scl * local_dir[0];
  local_supp[1] = scl * local_dir[1];

  // set result in Z direction
  local_supp[2] = local_dir[2] >= 0 ? size[1] : -size[1];

  // transform result to global frame
  localToGlobal(res, mat, local_supp, pos);
}


// box support function
static void SIM_c_boxSupport(sim_scalar_t res[3], sim_collision_ccd_object_t* obj, const sim_scalar_t dir[3]) {
  const sim_model_t* m = obj->model;
  const sim_data_t* d = obj->data;

  // box data
  int i = 3*obj->geom;
  const sim_scalar_t* mat = d->geom_xmat + 3*i;
  const sim_scalar_t* pos = d->geom_xpos + i;
  const sim_scalar_t* size = m->geom_size + i;

  // rotate dir to geom local frame
  sim_scalar_t local_dir[3], local_supp[3];
  mulMatTVec3(local_dir, mat, dir);

  // find support point in local frame
  local_supp[0] = local_dir[0] >= 0 ? size[0] : -size[0];
  local_supp[1] = local_dir[1] >= 0 ? size[1] : -size[1];
  local_supp[2] = local_dir[2] >= 0 ? size[2] : -size[2];

  // mark the index of the corner of the box for fast lookup
  obj->vertindex  = (local_supp[0] > 0) ? 1 : 0;
  obj->vertindex |= (local_supp[1] > 0) ? 2 : 0;
  obj->vertindex |= (local_supp[2] > 0) ? 4 : 0;

  // transform support point to global frame
  localToGlobal(res, mat, local_supp, pos);
}


// dot product between sim_scalar_t and float
static inline sim_scalar_t dot3f(const sim_scalar_t a[3], const float b[3]) {
  return a[0]*(sim_scalar_t)b[0] + a[1]*(sim_scalar_t)b[1] + a[2]*(sim_scalar_t)b[2];
}


// mesh support function via exhaustive search
static void SIM_c_meshSupport(sim_scalar_t res[3], sim_collision_ccd_object_t* obj, const sim_scalar_t dir[3]) {
  const sim_model_t* m = obj->model;
  const sim_data_t* d = obj->data;

  // mesh data
  int g = obj->geom;
  const sim_scalar_t* mat = d->geom_xmat+9*g;
  const sim_scalar_t* pos = d->geom_xpos+3*g;
  float* verts = m->mesh_vert + 3*m->mesh_vertadr[m->geom_dataid[g]];
  int nverts = m->mesh_vertnum[m->geom_dataid[g]];

  sim_scalar_t local_dir[3];
  mulMatTVec3(local_dir, mat, dir);

  sim_scalar_t max = -FLT_MAX;
  int imax = 0;

  // used cached results from previous search
  if (obj->vertindex >= 0) {
    imax = obj->vertindex;
    max = dot3f(local_dir, verts + 3*imax);
  }

  // search all vertices, find maximum dot product
  for (int i=0; i < nverts; i++) {
    sim_scalar_t vdot = dot3f(local_dir, verts + 3*i);

    // update max
    if (vdot > max) {
      max = vdot;
      imax = i;
    }
  }

  // record vertex index of maximum
  obj->vertindex = imax;

  local_dir[0] = (sim_scalar_t)verts[3*imax + 0];
  local_dir[1] = (sim_scalar_t)verts[3*imax + 1];
  local_dir[2] = (sim_scalar_t)verts[3*imax + 2];

  // transform result to global frame
  localToGlobal(res, mat, local_dir, pos);
}


// mesh support function via hill climbing
static void SIM_c_hillclimbSupport(sim_scalar_t res[3], sim_collision_ccd_object_t* obj, const sim_scalar_t dir[3]) {
  const sim_model_t* m = obj->model;
  const sim_data_t* d = obj->data;

  // get mesh info
  int g = obj->geom;
  int graphadr = m->mesh_graphadr[m->geom_dataid[g]];
  int numvert = m->mesh_graph[graphadr];
  int* vert_edgeadr = m->mesh_graph + graphadr + 2;
  int* vert_globalid = m->mesh_graph + graphadr + 2 + numvert;
  int* edge_localid = m->mesh_graph + graphadr + 2 + 2*numvert;
  float* verts = m->mesh_vert + 3*m->mesh_vertadr[m->geom_dataid[g]];
  const sim_scalar_t* pos = d->geom_xpos + 3*g;
  const sim_scalar_t* mat = d->geom_xmat + 9*g;

  // rotate dir to geom local frame
  sim_scalar_t local_dir[3];
  mulMatTVec3(local_dir, mat, dir);

  sim_scalar_t max = -FLT_MAX;
  int prev = -1;
  int imax = obj->meshindex >= 0 ? obj->meshindex : 0;

  // hillclimb until no change
  while (imax != prev) {
    prev = imax;
    int subidx;
    for (int i = vert_edgeadr[imax]; (subidx = edge_localid[i]) >= 0; i++) {
      sim_scalar_t vdot = dot3f(local_dir, verts + 3*vert_globalid[subidx]);
      if (vdot > max) {
        max = vdot;
        imax = subidx;  // update maximum vertex index
      }
    }
  }

  // record vertex index of maximum (local id)
  obj->meshindex = imax;

  // get resulting support vertex
  obj->vertindex = imax = vert_globalid[imax];
  local_dir[0] = (sim_scalar_t)verts[3*imax + 0];
  local_dir[1] = (sim_scalar_t)verts[3*imax + 1];
  local_dir[2] = (sim_scalar_t)verts[3*imax + 2];

  // transform result to global frame
  localToGlobal(res, mat, local_dir, pos);
}


// prism support function
static void SIM_c_prism_support(sim_scalar_t res[3], sim_collision_ccd_object_t* obj, const sim_scalar_t dir[3]) {
  int istart, ibest;
  sim_scalar_t best, tmp;

  // find best vertex in halfspace determined by dir.z
  istart = dir[2] < 0 ? 0 : 3;
  ibest = istart;
  best = sim_math_dot_3(obj->prism[istart], dir);
  for (int i=istart+1; i < istart+3; i++) {
    if ((tmp = sim_math_dot_3(obj->prism[i], dir)) > best) {
      ibest = i;
      best = tmp;
    }
  }

  // copy best point
  sim_math_internal_copy_3(res, obj->prism[ibest]);
}


// flex support function
static void SIM_c_flexSupport(sim_scalar_t res[3], sim_collision_ccd_object_t* obj, const sim_scalar_t dir[3]) {
  const sim_model_t* m = obj->model;
  const sim_data_t* d = obj->data;
  int f = obj->flex;
  int dim = m->flex_dim[f];

  // flex element
  if (obj->elem >= 0) {
    int e = obj->elem;
    const int* edata = m->flex_elem + m->flex_elemdataadr[f] + e*(dim+1);
    const sim_scalar_t* vert = d->flexvert_xpos + 3*m->flex_vertadr[f];

    // find element vertex with largest projection along dir
    sim_math_internal_copy_3(res, vert+3*edata[0]);
    sim_scalar_t best = sim_math_dot_3(res, dir);
    for (int i=1; i <= dim; i++) {
      sim_scalar_t dot = sim_math_dot_3(vert+3*edata[i], dir);

      // better vertex found: assign
      if (dot > best) {
        best = dot;
        sim_math_internal_copy_3(res, vert+3*edata[i]);
      }
    }

    // add radius and margin/2
    sim_math_internal_addToScl3(res, dir, m->flex_radius[f] + 0.5*obj->margin);
    return;
  }

  // flex vertex
  else {
    const sim_scalar_t* vert = d->flexvert_xpos + 3*(m->flex_vertadr[f] + obj->vert);
    sim_math_internal_addScl3(res, vert, dir, m->flex_radius[f] + 0.5*obj->margin);
    return;
  }
}


// libccd support function
void SIM_ccd_support(const void *_obj, const ccd_vec3_t *_dir, ccd_vec3_t *vec) {
  sim_collision_ccd_object_t *obj = (sim_collision_ccd_object_t *)_obj;
  sim_scalar_t *res = vec->v;
  const sim_scalar_t *dir = _dir->v;
  const sim_model_t* m = obj->model;
  const sim_data_t* d = obj->data;
  int g = obj->geom;

  if (g < 0) {
    int f = obj->flex;
    int dim = m->flex_dim[f];

    // flex element
    if (obj->elem >= 0) {
      int e = obj->elem;
      const int* edata = m->flex_elem + m->flex_elemdataadr[f] + e*(dim+1);
      const sim_scalar_t* vert = d->flexvert_xpos + 3*m->flex_vertadr[f];

      // find element vertex with largest projection along dir
      sim_math_internal_copy_3(res, vert+3*edata[0]);
      sim_scalar_t best = sim_math_dot_3(res, dir);
      for (int i=1; i <= dim; i++) {
        sim_scalar_t dot = sim_math_dot_3(vert+3*edata[i], dir);

        // better vertex found: assign
        if (dot > best) {
          best = dot;
          sim_math_internal_copy_3(res, vert+3*edata[i]);
        }
      }

      // add radius and margin/2
      sim_math_internal_addToScl3(res, dir, m->flex_radius[f] + 0.5*obj->margin);
      return;
    }

    // flex vertex
    else {
      const sim_scalar_t* vert = d->flexvert_xpos + 3*(m->flex_vertadr[f] + obj->vert);
      sim_math_internal_addScl3(res, vert, dir, m->flex_radius[f] + 0.5*obj->margin);
      return;
    }
  }

  float* vertdata;
  int ibest, graphadr, numvert, change, locid;
  int *vert_edgeadr, *vert_globalid, *edge_localid;
  sim_scalar_t tmp, vdot;

  const sim_scalar_t* size = m->geom_size+3*g;  // geom sizes
  sim_scalar_t local_dir[3];                    // direction in geom local frame

  // rotate dir to geom local frame
  sim_math_mulMatTVec3(local_dir, d->geom_xmat+9*g, dir);

  // compute result according to geom type
  switch ((SIM_tGeom) obj->geom_type) {
  case SIM_GEOM_SPHERE:
    sim_math_internal_scl3(res, local_dir, size[0]);
    break;

  case SIM_GEOM_CAPSULE:
    // start with sphere
    sim_math_internal_scl3(res, local_dir, size[0]);

    // add cylinder contribution
    res[2] += sim_math_sign(local_dir[2]) * size[1];
    break;

  case SIM_GEOM_ELLIPSOID:
    // find support point on unit sphere: scale dir by ellipsoid sizes and renormalize
    for (int i=0; i < 3; i++) {
      res[i] = local_dir[i] * size[i];
    }
    sim_math_normalize_3(res);

    // transform to ellipsoid
    for (int i=0; i < 3; i++) {
      res[i] *= size[i];
    }
    break;

  case SIM_GEOM_CYLINDER:
    // set result in XY plane: support on circle
    tmp = sim_math_sqrt(local_dir[0]*local_dir[0] + local_dir[1]*local_dir[1]);
    if (tmp > SIM_MINVAL) {
      res[0] = local_dir[0]/tmp*size[0];
      res[1] = local_dir[1]/tmp*size[0];
    } else {
      res[0] = res[1] = 0;
    }

    // set result in Z direction
    res[2] = sim_math_sign(local_dir[2]) * size[1];
    break;

  case SIM_GEOM_BOX:
    for (int i=0; i < 3; i++) {
      res[i] = sim_math_sign(local_dir[i]) * size[i];
    }
    break;

  case SIM_GEOM_MESH:
  case SIM_GEOM_SDF:
    // init search
    vertdata = m->mesh_vert + 3*m->mesh_vertadr[m->geom_dataid[g]];
    tmp = -1E+10;
    ibest = -1;

    // no graph data: exhaustive search
    if (m->mesh_graphadr[m->geom_dataid[g]] < 0) {
      // search all vertices, find best
      for (int i=0; i < m->mesh_vertnum[m->geom_dataid[g]]; i++) {
        // vdot = dot(vertex, dir)
        vdot = local_dir[0] * (sim_scalar_t)vertdata[3*i] +
               local_dir[1] * (sim_scalar_t)vertdata[3*i+1] +
               local_dir[2] * (sim_scalar_t)vertdata[3*i+2];

        // update best
        if (vdot > tmp) {
          tmp = vdot;
          ibest = i;
        }
      }

      // record best vertex index, in globalid format
      obj->meshindex = ibest;
    }

    // hill-climb using graph data
    else {
      // get info
      graphadr = m->mesh_graphadr[m->geom_dataid[g]];
      numvert = m->mesh_graph[graphadr];
      vert_edgeadr = m->mesh_graph + graphadr + 2;
      vert_globalid = m->mesh_graph + graphadr + 2 + numvert;
      edge_localid = m->mesh_graph + graphadr + 2 + 2*numvert;

      // init with first vertex in convex hull or warmstart
      ibest = obj->meshindex < 0 ? 0 : obj->meshindex;
      tmp = local_dir[0] * (sim_scalar_t)vertdata[3*vert_globalid[ibest]+0] +
            local_dir[1] * (sim_scalar_t)vertdata[3*vert_globalid[ibest]+1] +
            local_dir[2] * (sim_scalar_t)vertdata[3*vert_globalid[ibest]+2];

      // hill-climb until no change
      change = 1;
      while (change) {
        // look for improvement in ibest neighborhood
        change = 0;
        int i = vert_edgeadr[ibest];
        while ((locid=edge_localid[i]) >= 0) {
          // vdot = dot(vertex, local_dir)
          vdot = local_dir[0] * (sim_scalar_t)vertdata[3*vert_globalid[locid]] +
                 local_dir[1] * (sim_scalar_t)vertdata[3*vert_globalid[locid]+1] +
                 local_dir[2] * (sim_scalar_t)vertdata[3*vert_globalid[locid]+2];

          // update best
          if (vdot > tmp) {
            tmp = vdot;
            ibest = locid;
            change = 1;
          }

          // advance to next edge
          i++;
        }
      }

      // record best vertex index, in locid format
      obj->meshindex = ibest;

      // map best index to globalid
      ibest = vert_globalid[ibest];
    }

    // sanity check, SHOULD NOT OCCUR
    if (ibest < 0) {
      sim_warning("mesh_support could not find support vertex");
      sim_math_zero_3(res);
    }

    // copy best vertex
    else {
      for (int i=0; i < 3; i++) {
        res[i] = (sim_scalar_t)vertdata[3*ibest + i];
      }
    }
    break;

  default:
    SIM_ERROR("ccd support function is undefined for geom type %d", m->geom_type[g]);
  }

  // add local_dir*margin/2 to result
  for (int i=0; i < 3; i++) {
    res[i] += local_dir[i] * obj->margin/2;
  }

  // rotate result to global frame
  sim_math_mul_mat_vec_3(res, d->geom_xmat+9*g, res);

  // add geom position
  sim_math_internal_addTo3(res, d->geom_xpos+3*g);
}


// libccd prism support function
static void SIM_ccd_prism_support(const void *obj, const ccd_vec3_t *dir, ccd_vec3_t *vec) {
  SIM_c_prism_support(vec->v, (sim_collision_ccd_object_t*) obj, dir->v);
}

// ------------------------------------------------------------------------------------------------

// initialize a CCD object
void SIM_c_initCCDObj(sim_collision_ccd_object_t* obj, const sim_model_t* m, const sim_data_t* d, int g, sim_scalar_t margin) {
  obj->model = m;
  obj->data = d;
  obj->geom = g;
  obj->margin = margin;
  obj->center = SIM_c_center;
  obj->vertindex = -1;
  obj->meshindex = -1;
  obj->flex = -1;
  obj->elem = -1;
  obj->vert = -1;
  sim_math_zero4(obj->rotate);
  obj->rotate[0] = 1;
  if (g >= 0) {
    obj->geom_type = m->geom_type[g];
    switch ((SIM_tGeom) obj->geom_type) {
    case SIM_GEOM_ELLIPSOID:
      obj->support = SIM_c_ellipsoidSupport;
      break;
    case SIM_GEOM_MESH:
    case SIM_GEOM_SDF:
      if (m->mesh_graphadr[m->geom_dataid[g]] < 0 ||
          m->mesh_vertnum[m->geom_dataid[g]] < SIM_MESH_HILLCLIMB_MIN) {
        obj->support = SIM_c_meshSupport;
      } else {
        obj->support = SIM_c_hillclimbSupport;
      }
      break;
    case SIM_GEOM_SPHERE:
      obj->support = SIM_c_sphereSupport;
      break;
    case SIM_GEOM_CAPSULE:
      obj->support = SIM_c_capsuleSupport;
      break;
    case SIM_GEOM_CYLINDER:
      obj->support = SIM_c_cylinderSupport;
      break;
    case SIM_GEOM_BOX:
      obj->support = SIM_c_boxSupport;
      break;
    case SIM_GEOM_HFIELD:
      obj->center = SIM_c_prism_center;
      obj->support = SIM_c_prism_support;

      int hid = m->geom_dataid[g];
      obj->hfield_nrow = m->hfield_nrow[hid];
      obj->hfield_ncol = m->hfield_ncol[hid];
      obj->size = m->hfield_size + 4*hid;
      obj->hfield_data = m->hfield_data + m->hfield_adr[hid];
      break;
    default:
      obj->support = NULL;
      break;
    }
  } else {
    obj->geom_type = SIM_GEOM_FLEX;
    obj->support = SIM_c_flexSupport;
  }
}


// set flex data for CCD object
static void SIM_c_setCCDObjFlex(sim_collision_ccd_object_t* obj, int flex, int elem, int vert) {
  obj->flex = flex;
  obj->elem = elem;
  obj->vert = vert;
}


// initialize CCD structure
static void SIM_c_initCCD(ccd_t* ccd, const sim_model_t* m) {
  CCD_INIT(ccd);
  ccd->mpr_tolerance = m->opt.ccd_tolerance;
  ccd->epa_tolerance = m->opt.ccd_tolerance;  // use MPR tolerance for EPA
  ccd->max_iterations = m->opt.ccd_iterations;
}


// find convex-convex collision
static int SIM_c_CCDIteration(const sim_model_t* m, const sim_data_t* d, sim_collision_ccd_object_t* obj1, sim_collision_ccd_object_t* obj2,
                            sim_contact_t* con, int max_contacts, sim_scalar_t margin) {
  if (!SIM_DISABLED(SIM_DSBL_NATIVECCD)) {
    SIM_CCDConfig config;
    SIM_CCDStatus status;

    // set config
    config.max_iterations = m->opt.ccd_iterations;
    config.tolerance = m->opt.ccd_tolerance;
    config.max_contacts = max_contacts;
    config.dist_cutoff = 0;  // no geom distances needed
    config.context = (void*)d;
    config.alloc = ccd_allocate;
    config.free = ccd_free;

    sim_scalar_t dist = SIM_c_ccd(&config, &status, obj1, obj2);
    if (dist < 0) {
      for (int i = 0; i < status.nx; i++) {
        sim_contact_t* c = con++;
        c->dist = margin + dist;
        sim_math_sub_3(c->frame, status.x1 + 3*i, status.x2 + 3*i);
        sim_math_normalize_3(c->frame);
        c->pos[0] = 0.5 * (status.x1[0 + 3*i] + status.x2[0 + 3*i]);
        c->pos[1] = 0.5 * (status.x1[1 + 3*i] + status.x2[1 + 3*i]);
        c->pos[2] = 0.5 * (status.x1[2 + 3*i] + status.x2[2 + 3*i]);
        sim_math_zero_3(c->frame+3);
      }
      return status.nx;
    }
    return 0;
  }

  // init libccd structure
  ccd_t ccd;
  SIM_c_initCCD(&ccd, m);
  ccd.first_dir = ccdFirstDirDefault;
  ccd.center1 = SIM_ccd_center;
  ccd.center2 = SIM_ccd_center;
  ccd.support1 = SIM_ccd_support;
  ccd.support2 = SIM_ccd_support;

  ccd_vec3_t dir, pos;
  ccd_real_t depth;

  // call MPR from libccd
  if (ccdMPRPenetration(obj1, obj2, &ccd, &depth, &dir, &pos) == 0) {
    // contact is found but normal is undefined
    if (ccdVec3Eq(&dir, ccd_vec3_origin)) {
      return 0;
    }

    // fill in contact data
    con->dist = margin-depth;
    sim_math_internal_copy_3(con->frame, dir.v);
    sim_math_internal_copy_3(con->pos, pos.v);
    sim_math_zero_3(con->frame+3);

    // both geoms: fix contact frame normal
    if (obj1->geom >= 0 && obj2->geom >= 0) {
      SIM_c_fixNormal(m, d, con, obj1->geom, obj2->geom);
    }

    return 1;
  }
  return 0;
}


// compare new contact to previous contacts, return 1 if it is far from all of them
static int SIM_c_isDistinctContact(sim_contact_t* con, int ncon, sim_scalar_t tolerance) {
  for (int i=0; i < ncon-1; i++) {
    if (sim_math_dist3(con[i].pos, con[ncon - 1].pos) <= tolerance) {
      return 0;
    }
  }
  return 1;
}


// in-place rotation of spatial frame around given point of origin
static void sim_math_rotateFrame(const sim_scalar_t origin[3], const sim_scalar_t rot[9],
                            sim_scalar_t xmat[9], sim_scalar_t xpos[3]) {
  sim_scalar_t mat[9], vec[3], rel[3];

  // rotate frame: xmat = rot*xmat
  sim_math_mulMatMat3(mat, rot, xmat);
  sim_math_copy(xmat, mat, 9);

  // vector to rotation origin: rel = origin - xpos
  sim_math_internal_sub3(rel, origin, xpos);

  // displacement of origin due to rotation: vec = rot*rel - rel
  sim_math_mul_mat_vec_3(vec, rot, rel);
  sim_math_subFrom3(vec, rel);

  // correct xpos by subtracting displacement: xpos = xpos - vec
  sim_math_internal_subFrom3(xpos, vec);
}


// return number of contacts supported by a single pass of narrowphase
static int maxContacts(const sim_collision_ccd_object_t* obj1, const sim_collision_ccd_object_t* obj2) {
  const sim_model_t* m = obj1->model;

  // single pass not supported for margins
  if (obj1->margin > 0 || obj2->margin > 0) {
    return 1;
  }

  // can return 8 contacts for box-box collision in one pass
  int type1 = m->geom_type[obj1->geom];
  int type2 = m->geom_type[obj2->geom];
  if (type1 == SIM_GEOM_BOX && type2 == SIM_GEOM_BOX) {
    return 8;
  }

  // reduce mesh collisions to 4 contacts max
  if (type1 == SIM_GEOM_BOX || type1 == SIM_GEOM_MESH) {
    if (type2 == SIM_GEOM_BOX || type2 == SIM_GEOM_MESH) {
      return SIM_ENABLED(SIM_ENBL_MULTICCD) ? 4 : 1;
    }
  }

  // not supported for other geom types
  return 1;
}


// multi-point convex-convex collision, using libccd
int SIM_c_Convex(const sim_model_t* m, const sim_data_t* d,
               sim_contact_t* con, int g1, int g2, sim_scalar_t margin) {
  // init ccd objects
  sim_collision_ccd_object_t obj1, obj2;
  SIM_c_initCCDObj(&obj1, m, d, g1, margin);
  SIM_c_initCCDObj(&obj2, m, d, g2, margin);
  int max_contacts = maxContacts(&obj1, &obj2);

  // find initial contact
  int ncon = SIM_c_CCDIteration(m, d, &obj1, &obj2, con, max_contacts, margin);


  // no additional contacts needed
  if (!SIM_DISABLED(SIM_DSBL_NATIVECCD) && max_contacts > 1) {
    return ncon;
  }

  // look for additional contacts
  if (ncon == 1 && SIM_ENABLED(SIM_ENBL_MULTICCD)  // TODO(tassa) leave as bitflag or make geom attribute (?)
      && m->geom_type[g1] != SIM_GEOM_ELLIPSOID && m->geom_type[g1] != SIM_GEOM_SPHERE
      && m->geom_type[g2] != SIM_GEOM_ELLIPSOID && m->geom_type[g2] != SIM_GEOM_SPHERE) {
    // multiCCD parameters
    const sim_scalar_t relative_tolerance = 1e-3;
    const sim_scalar_t perturbation_angle = 1e-3;

    // save positions and orientations of g1 and g2
    sim_scalar_t xpos1[3], xmat1[9], xpos2[3], xmat2[9];
    sim_math_internal_copy_3(xpos1, d->geom_xpos+3*g1);
    sim_math_internal_copy9(xmat1, d->geom_xmat+9*g1);
    sim_math_internal_copy_3(xpos2, d->geom_xpos+3*g2);
    sim_math_internal_copy9(xmat2, d->geom_xmat+9*g2);

    // complete frame of initial contact
    sim_scalar_t frame[9];
    sim_math_internal_copy9(frame, con[0].frame);
    sim_math_makeFrame(frame);

    // tolerance for determining if newly found contacts are distinct
    const sim_scalar_t tolerance = relative_tolerance * sim_math_min(m->geom_rbound[g1], m->geom_rbound[g2]);

    // axes and rotation angles for perturbation test
    sim_scalar_t* axes[2] = {frame+3, frame+6};
    sim_scalar_t angles[2] = {-perturbation_angle, perturbation_angle};

    // rotate both geoms, search for new contacts
    for (int axis_id = 0; axis_id < 2; ++axis_id) {
      for (int angle_id = 0; angle_id < 2; ++angle_id) {
        sim_scalar_t* axis = axes[axis_id];
        sim_scalar_t angle = angles[angle_id];

        // make rotation matrix rot
        sim_scalar_t quat[4], rot[9];
        sim_math_internal_axisAngle2Quat(quat, axis, angle);
        sim_math_quat2Mat(rot, quat);

        // rotate g1 around initial contact point
        sim_math_rotateFrame(con[0].pos, rot, d->geom_xmat+9*g1, d->geom_xpos+3*g1);

        // inversely rotate g2 around initial contact point
        sim_scalar_t invrot[9];
        sim_math_transpose(invrot, rot, 3, 3);
        sim_math_rotateFrame(con[0].pos, invrot, d->geom_xmat+9*g2, d->geom_xpos+3*g2);

        // search for new contact
        int new_contact = SIM_c_CCDIteration(m, d, &obj1, &obj2, con+ncon, 1, margin);

        // check new contact
        if (new_contact && SIM_c_isDistinctContact(con, ncon + 1, tolerance)) {
          // set penetration of new point to equal that of initial point
          con[ncon].dist = con[0].dist;
          // add new point
          ncon += 1;
        }

        // reset positions and orientations of g1 and g2
        sim_math_internal_copy_3(d->geom_xpos+3*g1, xpos1);
        sim_math_internal_copy9(d->geom_xmat+9*g1, xmat1);
        sim_math_internal_copy_3(d->geom_xpos+3*g2, xpos2);
        sim_math_internal_copy9(d->geom_xmat+9*g2, xmat2);
      }
    }
  }
  return ncon;
}


// parameters for plane-mesh extra contacts
const int maxplanemesh = 3;
const sim_scalar_t tolplanemesh = 0.3;

// add one plane-mesh contact
static int addplanemesh(sim_contact_t* con, const float vertex[3],
                        const sim_scalar_t pos1[3], const sim_scalar_t normal1[3],
                        const sim_scalar_t pos2[3], const sim_scalar_t mat2[9],
                        const sim_scalar_t first[3], sim_scalar_t rbound) {
  // compute point in global coordinates
  sim_scalar_t pnt[3], v[3] = {vertex[0], vertex[1], vertex[2]};
  sim_math_mul_mat_vec_3(pnt, mat2, v);
  sim_math_add_to_3(pnt, pos2);

  // skip if too close to first contact
  if (sim_math_dist3(pnt, first) < tolplanemesh*rbound) {
    return 0;
  }

  // pnt-pos difference vector
  sim_scalar_t dif[3];
  sim_math_internal_sub3(dif, pnt, pos1);

  // set distance
  con->dist = sim_math_dot_3(normal1, dif);

  // set position
  sim_math_internal_copy_3(con->pos, pnt);
  sim_math_internal_addToScl3(con->pos, normal1, -0.5*con->dist);

  // set frame
  sim_math_internal_copy_3(con->frame, normal1);
  sim_math_zero_3(con->frame+3);

  return 1;
}


// plane-convex collision, using libccd
int SIM_c_PlaneConvex(const sim_model_t* m, const sim_data_t* d,
                    sim_contact_t* con, int g1, int g2, sim_scalar_t margin) {
  SIM_GETINFO
  sim_scalar_t dist, dif[3], normal[3] = {mat1[2], mat1[5], mat1[8]};
  ccd_vec3_t dir, vec;
  sim_collision_ccd_object_t obj;
  SIM_c_initCCDObj(&obj, m, d, g2, 0);
  // get support point in -normal direction
  ccdVec3Set(&dir, -mat1[2], -mat1[5], -mat1[8]);
  SIM_ccd_support(&obj, &dir, &vec);

  // compute normal distance, return if too far
  sim_math_internal_sub3(dif, vec.v, pos1);
  dist = sim_math_dot_3(normal, dif);
  if (dist > margin) {
    return 0;
  }

  // fill in contact data
  con->dist = dist;
  sim_math_internal_copy_3(con->pos, vec.v);
  sim_math_internal_addToScl3(con->pos, normal, -0.5*dist);
  sim_math_internal_copy_3(con->frame, normal);
  sim_math_zero_3(con->frame+3);

  //--------------- add all/connected vertices below margin
  float* vertdata;
  int graphadr, numvert, locid;
  int *vert_edgeadr, *vert_globalid, *edge_localid;
  sim_scalar_t vdot;
  int count = 1, g = g2;

  // g is an ellipsoid: no need for further mesh-specific processing
  if (m->geom_dataid[g] == -1) {
    return count;
  }

  // init
  vertdata = m->mesh_vert + 3*m->mesh_vertadr[m->geom_dataid[g]];

  // express dir in geom local frame
  sim_scalar_t locdir[3];
  sim_math_mulMatTVec3(locdir, d->geom_xmat+9*g, dir.v);

  // inclusion threshold along locdir, relative to geom2 center
  sim_math_internal_sub3(dif, pos2, pos1);
  sim_scalar_t threshold = sim_math_dot_3(normal, dif) - margin;

  // no graph data: exhaustive search
  if (m->mesh_graphadr[m->geom_dataid[g]] < 0) {
    // search all vertices, find best
    for (int i=0; i < m->mesh_vertnum[m->geom_dataid[g]] && count < maxplanemesh; i++) {
      // vdot = dot(vertex, dir)
      vdot = locdir[0] * (sim_scalar_t)vertdata[3*i] +
             locdir[1] * (sim_scalar_t)vertdata[3*i+1] +
             locdir[2] * (sim_scalar_t)vertdata[3*i+2];

      // detect contact, skip best
      if (vdot > threshold && i != obj.meshindex) {
        count += addplanemesh(con+count, vertdata+3*i,
                              pos1, normal, pos2, mat2,
                              con->pos, m->geom_rbound[g2]);
      }
    }
  }

  // use graph data
  else if (obj.meshindex >= 0) {
    // get info
    graphadr = m->mesh_graphadr[m->geom_dataid[g]];
    numvert = m->mesh_graph[graphadr];
    vert_edgeadr = m->mesh_graph + graphadr + 2;
    vert_globalid = m->mesh_graph + graphadr + 2 + numvert;
    edge_localid = m->mesh_graph + graphadr + 2 + 2*numvert;

    // look for contacts in ibest neighborhood
    int i = vert_edgeadr[obj.meshindex];
    while ((locid=edge_localid[i]) >= 0 && count < maxplanemesh) {
      // vdot = dot(vertex, dir)
      vdot = locdir[0] * (sim_scalar_t)vertdata[3*vert_globalid[locid]] +
             locdir[1] * (sim_scalar_t)vertdata[3*vert_globalid[locid]+1] +
             locdir[2] * (sim_scalar_t)vertdata[3*vert_globalid[locid]+2];

      // detect contact
      if (vdot > threshold) {
        count += addplanemesh(con+count, vertdata+3*vert_globalid[locid],
                              pos1, normal, pos2, mat2,
                              con->pos, m->geom_rbound[g2]);
      }

      // advance to next edge
      i++;
    }
  }

  return count;
}


//----------------------------  heightfield collisions ---------------------------------------------

// ccd prism first dir
static void prism_firstdir(const void* o1, const void* o2, ccd_vec3_t *vec) {
  ccdVec3Set(vec, 0, 0, 1);
}


// add vertex to prism
static inline void addVert(sim_collision_ccd_object_t* obj, sim_scalar_t x, sim_scalar_t y, sim_scalar_t z) {
  // move old data
  sim_math_internal_copy_3(obj->prism[0], obj->prism[1]);
  sim_math_internal_copy_3(obj->prism[1], obj->prism[2]);
  sim_math_internal_copy_3(obj->prism[3], obj->prism[4]);
  sim_math_internal_copy_3(obj->prism[4], obj->prism[5]);

  // add new vertex at last position
  obj->prism[2][0] = obj->prism[5][0] = x;
  obj->prism[2][1] = obj->prism[5][1] = y;
  obj->prism[5][2] = z;
}


// add vertex to prism
static inline void addPrismVert(sim_collision_ccd_object_t* obj, int r, int c, int i, sim_scalar_t dx, sim_scalar_t dy, sim_scalar_t margin) {
  // move old data
  sim_math_internal_copy_3(obj->prism[0], obj->prism[1]);
  sim_math_internal_copy_3(obj->prism[1], obj->prism[2]);
  sim_math_internal_copy_3(obj->prism[3], obj->prism[4]);
  sim_math_internal_copy_3(obj->prism[4], obj->prism[5]);

  int dr = 1 - i;

  // add new vertex at last position
  obj->prism[2][0] = obj->prism[5][0] = dx*c - obj->size[0];
  obj->prism[2][1] = obj->prism[5][1] = dy*(r + dr) - obj->size[1];
  obj->prism[5][2] = obj->hfield_data[(r + dr)*obj->hfield_ncol + c]*obj->size[2];

  // factor in margin
  obj->prism[5][2] += margin;
}


// entry point for heightfield collisions
int SIM_c_ConvexHField(const sim_model_t* m, const sim_data_t* d,
                     sim_contact_t* con, int g1, int g2, sim_scalar_t margin) {
  // hfield frame
  const sim_scalar_t* pos1 = d->geom_xpos + 3*g1;
  const sim_scalar_t* mat1 = d->geom_xmat + 9*g1;

  // geom2 frame
  sim_scalar_t* pos2 = d->geom_xpos + 3*g2;
  sim_scalar_t* mat2 = d->geom_xmat + 9*g2;

  // hfield data
  int hid = m->geom_dataid[g1];
  int nrow = m->hfield_nrow[hid];
  int ncol = m->hfield_ncol[hid];
  sim_scalar_t size0 = m->hfield_size[4*hid + 0], size1 = m->hfield_size[4*hid + 1];
  sim_scalar_t size2 = m->hfield_size[4*hid + 2], size3 = m->hfield_size[4*hid + 3];

  // try early return using box-sphere test

  // express geom2 pos in hfield frame
  sim_scalar_t pos[3] = {pos2[0] - pos1[0], pos2[1] - pos1[1], pos2[2] - pos1[2]};
  sim_math_mulMatTVec3(pos, mat1, pos);

  // sphere radius is geom2 rbound + margin
  sim_scalar_t radius = m->geom_rbound[g2] + margin;

  // box-sphere test
  if ((size0 < pos[0] - radius) || (-size0 > pos[0] + radius) ||
      (size1 < pos[1] - radius) || (-size1 > pos[1] + radius) ||
      (size2 < pos[2] - radius) || (-size3 > pos[2] + radius)) {
    return 0;
  }

  // ccd set up
  sim_collision_ccd_object_t obj1, obj2;
  SIM_c_initCCDObj(&obj1, m, d, g1, 0);
  SIM_c_initCCDObj(&obj2, m, d, g2, 0);


  // try early return using AABB box-box test

  // express geom2 mat in hfield frame
  sim_scalar_t mat[9];
  sim_math_internal_mulMatTMat3(mat, mat1, mat2);

  // save mat2 and pos2, replace with relative frame
  sim_scalar_t savemat2[9], savepos2[3];
  sim_math_internal_copy9(savemat2, mat2);
  sim_math_internal_copy_3(savepos2, pos2);
  sim_math_internal_copy9(mat2, mat);
  sim_math_internal_copy_3(pos2, pos);

  sim_scalar_t dir[3] = {0, 0, 0}, res[3];

  // get support point in +X
  dir[0] = 1;
  obj2.support(res, &obj2, dir);
  sim_scalar_t xmax = res[0];

  // get support point in -X
  dir[0] = -1;
  obj2.support(res, &obj2, dir);
  sim_scalar_t xmin = res[0];

  dir[0] = 0;

  // get support point in +Y
  dir[1] = 1;
  obj2.support(res, &obj2, dir);
  sim_scalar_t ymax = res[1];

  // get support point in -Y
  dir[1] = -1;
  obj2.support(res, &obj2, dir);
  sim_scalar_t ymin = res[1];

  dir[1] = 0;

  // get support point in +Z
  dir[2] = 1;
  obj2.support(res, &obj2, dir);
  sim_scalar_t zmax = res[2];

  // get support point in -Z
  dir[2] = -1;
  obj2.support(res, &obj2, dir);
  sim_scalar_t zmin = res[2];

  // AABB box-box test
  if ((xmin - margin > size0) || (xmax + margin < -size0) ||
      (ymin - margin > size1) || (ymax + margin < -size1) ||
      (zmin - margin > size2) || (zmax + margin < -size3)) {
    sim_math_internal_copy9(mat2, savemat2);
    sim_math_internal_copy_3(pos2, savepos2);
    return 0;
  }

  // compute sub-grid bounds
  int cmin = (int) sim_math_floor((xmin + size0) / (2.0*size0) * (ncol-1));
  int cmax = (int) sim_math_ceil ((xmax + size0) / (2.0*size0) * (ncol-1));
  int rmin = (int) sim_math_floor((ymin + size1) / (2.0*size1) * (nrow-1));
  int rmax = (int) sim_math_ceil ((ymax + size1) / (2.0*size1) * (nrow-1));
  cmin = SIM_MAX(0, cmin);
  cmax = SIM_MIN(ncol-1, cmax);
  rmin = SIM_MAX(0, rmin);
  rmax = SIM_MIN(nrow-1, rmax);

  // CCD collision testing

  ccd_t ccd;
  SIM_c_initCCD(&ccd, m);
  ccd.first_dir = prism_firstdir;
  ccd.center1 = SIM_ccd_prism_center;
  ccd.center2 = SIM_ccd_center;
  ccd.support1 = SIM_ccd_prism_support;
  ccd.support2 = SIM_ccd_support;

  // geom margin needed for actual collision test
  obj2.margin = margin;

  // compute real-valued grid step
  sim_scalar_t dx = (2.0*size0) / (ncol-1);
  sim_scalar_t dy = (2.0*size1) / (nrow-1);

  // set zbottom value using base size
  obj1.prism[0][2] = obj1.prism[1][2] = obj1.prism[2][2] = -size3;

  // process all prisms in subgrid
  int ncon = 0;
  for (int r=rmin; r < rmax; r++) {
    addPrismVert(&obj1, r, cmin, 0, dx, dy, margin);
    addPrismVert(&obj1, r, cmin, 1, dx, dy, margin);
    for (int c=cmin + 1; c <= cmax; c++) {
      for (int i=0; i < 2; i++) {
        // send vertex to prism constructor
        addPrismVert(&obj1, r, c, i, dx, dy, margin);

        // prism height test
        if (obj1.prism[3][2] < zmin && obj1.prism[4][2] < zmin && obj1.prism[5][2] < zmin) {
          continue;
        }

        // run penetration function, save contact
        ccd_vec3_t dirccd, vecccd;
        ccd_real_t depth;
        if (SIM_c_penetration(m, &obj1, &obj2, &ccd, &depth, &dirccd, &vecccd) == 0
            && !ccdVec3Eq(&dirccd, ccd_vec3_origin)) {
          // fill in contact data, transform to global coordinates
          con[ncon].dist = -depth;
          sim_math_internal_mulMatVec3(con[ncon].frame, mat1, dirccd.v);
          sim_math_internal_mulMatVec3(con[ncon].pos, mat1, vecccd.v);
          sim_math_internal_addTo3(con[ncon].pos, pos1);
          sim_math_zero_3(con[ncon].frame+3);

          // force out of all loops if max contacts reached
          if (++ncon >= SIM_MAXCONPAIR) {
            r = rmax+1;
            c = cmax+1;
            i = 3;
            break;
          }
        }
      }
    }
  }

  // restore mat2 and pos2
  sim_math_internal_copy9(mat2, savemat2);
  sim_math_internal_copy_3(pos2, savepos2);

  if (SIM_DISABLED(SIM_DSBL_NATIVECCD)) {
    // fix contact normals
    for (int i=0; i < ncon; i++) {
      SIM_c_fixNormal(m, d, con+i, g1, g2);
    }
  }

  return ncon;
}


//--------------------------- fix contact frame normal ---------------------------------------------

// compute normal for point outside ellipsoid, using ray-projection SQP
static int SIM_c_ellipsoidInside(sim_scalar_t nrm[3], const sim_scalar_t pos[3], const sim_scalar_t size[3]) {
  // algorithm constants
  const int maxiter = 30;
  const sim_scalar_t tolerance = 1e-6;

  // precompute quantities
  sim_scalar_t S2inv[3] = {1/(size[0]*size[0]), 1/(size[1]*size[1]), 1/(size[2]*size[2])};
  sim_scalar_t C = pos[0]*pos[0]*S2inv[0] + pos[1]*pos[1]*S2inv[1] + pos[2]*pos[2]*S2inv[2] - 1;
  if (C > 0) {
    return 0;
  }

  // normalize initial normal (just in case)
  sim_math_normalize_3(nrm);

  // main iteration
  int iter;
  for (iter=0; iter < maxiter; iter++) {
    // coefficients and determinant of quadratic
    sim_scalar_t A = nrm[0]*nrm[0]*S2inv[0] + nrm[1]*nrm[1]*S2inv[1] + nrm[2]*nrm[2]*S2inv[2];
    sim_scalar_t B = pos[0]*nrm[0]*S2inv[0] + pos[1]*nrm[1]*S2inv[1] + pos[2]*nrm[2]*S2inv[2];
    sim_scalar_t det = B*B - A*C;
    if (det < SIM_MINVAL || A < SIM_MINVAL) {
      return (iter > 0);
    }

    // ray intersection with ellipse: pos + x*nrm, x>=0
    sim_scalar_t x = (-B + sim_math_sqrt(det))/A;
    if (x < 0) {
      return (iter > 0);
    }

    // new point on ellipsoid
    sim_scalar_t pnt[3];
    sim_math_internal_addScl3(pnt, pos, nrm, x);

    // normal at new point
    sim_scalar_t newnrm[3] = {pnt[0]*S2inv[0], pnt[1]*S2inv[1], pnt[2]*S2inv[2]};
    sim_math_normalize_3(newnrm);

    // save change and assign
    sim_scalar_t change = sim_math_dist3(nrm, newnrm);
    sim_math_internal_copy_3(nrm, newnrm);

    // terminate if converged
    if (change < tolerance) {
      break;
    }
  }

  return 1;
}


// compute normal for point inside ellipsoid, using diagonal QCQP
static int SIM_c_ellipsoidOutside(sim_scalar_t nrm[3], const sim_scalar_t pos[3], const sim_scalar_t size[3]) {
  // algorithm constants
  const int maxiter = 30;
  const sim_scalar_t tolerance = 1e-6;

  // precompute quantities
  sim_scalar_t S2[3] = {size[0]*size[0], size[1]*size[1], size[2]*size[2]};
  sim_scalar_t PS2[3] = {pos[0]*pos[0]*S2[0], pos[1]*pos[1]*S2[1], pos[2]*pos[2]*S2[2]};

  // main iteration
  sim_scalar_t la = 0;
  int iter;
  for (iter=0; iter < maxiter; iter++) {
    // precompute 1/(s^2+la)
    sim_scalar_t R[3] = {1/(S2[0]+la), 1/(S2[1]+la), 1/(S2[2]+la)};

    // value
    sim_scalar_t val = PS2[0]*R[0]*R[0] + PS2[1]*R[1]*R[1] + PS2[2]*R[2]*R[2] - 1;
    if (val < tolerance) {
      break;
    }

    // derivative
    sim_scalar_t deriv = -2*(PS2[0]*R[0]*R[0]*R[0] + PS2[1]*R[1]*R[1]*R[1] + PS2[2]*R[2]*R[2]*R[2]);
    if (deriv > -SIM_MINVAL) {
      break;
    }

    // delta
    sim_scalar_t delta = -val/deriv;
    if (delta < tolerance) {
      break;
    }

    // update
    la += delta;
  }

  // compute normal given lambda
  nrm[0] = pos[0]/(S2[0]+la);
  nrm[1] = pos[1]/(S2[1]+la);
  nrm[2] = pos[2]/(S2[2]+la);
  sim_math_normalize_3(nrm);

  return 1;
}


// fix normals if required
void SIM_c_fixNormal(const sim_model_t* m, const sim_data_t* d, sim_contact_t* con, int g1, int g2) {
  sim_scalar_t dst1, dst2;

  // get geom ids and types
  int gid[2] = {g1, g2};
  SIM_tGeom type[2];
  for (int i=0; i < 2; i++) {
    if (gid[i] < 0) {
      type[i] = SIM_GEOM_NONE;
    } else {
      type[i] = m->geom_type[gid[i]];
    }

    // set to SIM_GEOM_NONE if type cannot be processed
    if (type[i] != SIM_GEOM_SPHERE && type[i] != SIM_GEOM_CAPSULE &&
        type[i] != SIM_GEOM_ELLIPSOID && type[i] != SIM_GEOM_CYLINDER) {
      type[i] = SIM_GEOM_NONE;
    }
  }

  // neither type can be processed: nothing to do
  if (type[0] == SIM_GEOM_NONE && type[1] == SIM_GEOM_NONE) {
    return;
  }

  // init normals
  sim_scalar_t normal[2][3] = {
    {con->frame[0], con->frame[1], con->frame[2]},
    {-con->frame[0], -con->frame[1], -con->frame[2]}
  };

  // process geoms in type range
  int processed[2] = {0, 0};
  for (int i=0; i < 2; i++) {
    if (type[i] != SIM_GEOM_NONE) {
      // get geom mat and size
      sim_scalar_t* mat = d->geom_xmat + 9*gid[i];
      sim_scalar_t* size = m->geom_size + 3*gid[i];

      // map contact point and normal to local frame
      sim_scalar_t dif[3], pos[3], nrm[3];
      sim_math_sub_3(dif, con->pos, d->geom_xpos+3*gid[i]);
      sim_math_mulMatTVec3(pos, mat, dif);
      sim_math_mulMatTVec3(nrm, mat, normal[i]);

      // process according to type
      switch (type[i]) {
      case SIM_GEOM_SPHERE:
        sim_math_internal_copy_3(nrm, pos);
        processed[i] = 1;
        break;

      case SIM_GEOM_CAPSULE:
        // Z: bottom cap
        if (pos[2] < -size[1]) {
          nrm[2] = pos[2]+size[1];
        }

        // Z: top cap
        else if (pos[2] > size[1]) {
          nrm[2] = pos[2]-size[1];
        }

        // Z: cylinder
        else {
          nrm[2] = 0;
        }

        // copy XY
        nrm[0] = pos[0];
        nrm[1] = pos[1];
        processed[i] = 1;
        break;

      case SIM_GEOM_ELLIPSOID:
        // guard against invalid ellipsoid size (just in case)
        if (size[0] < SIM_MINVAL || size[1] < SIM_MINVAL || size[2] < SIM_MINVAL) {
          break;
        }

        // compute elliptic distance^2
        dst1 = pos[0]*pos[0]/(size[0]*size[0]) +
               pos[1]*pos[1]/(size[1]*size[1]) +
               pos[2]*pos[2]/(size[2]*size[2]);

        // dispatch to inside or outside solver
        if (dst1 <= 1) {
          processed[i] = SIM_c_ellipsoidInside(nrm, pos, size);
        } else {
          processed[i] = SIM_c_ellipsoidOutside(nrm, pos, size);
        }
        break;

      case SIM_GEOM_CYLINDER:
        // skip if within 5% length of flat wall
        if (sim_math_abs(pos[2]) > 0.95*size[1]) {
          break;
        }

        // compute distances to flat and round wall
        dst1 = sim_math_abs(size[1]-sim_math_abs(pos[2]));
        dst2 = sim_math_abs(size[0]-sim_math_norm(pos, 2));

        // require 4x closer to round than flat wall
        if (dst1 < 0.25*dst2) {
          break;
        }

        // set normal for round wall
        nrm[0] = pos[0];
        nrm[1] = pos[1];
        nrm[2] = 0;
        processed[i] = 1;
        break;

      default:
        // do nothing: only sphere, capsule, ellipsoid and cylinder are processed
        break;
      }

      // normalize and map normal to global frame
      if (processed[i]) {
        sim_math_normalize_3(nrm);
        sim_math_internal_mulMatVec3(normal[i], mat, nrm);
      }
    }
  }

  // both processed: average
  if (processed[0] && processed[1]) {
    sim_math_internal_sub3(con->frame, normal[0], normal[1]);
    sim_math_normalize_3(con->frame);
  }

  // first processed: copy
  else if (processed[0]) {
    sim_math_internal_copy_3(con->frame, normal[0]);
  }

  // second processed: copy reverse
  else if (processed[1]) {
    sim_math_internal_scl3(con->frame, normal[1], -1);
  }

  // clear second frame axis if processed, just in case
  if (processed[0] || processed[1]) {
    sim_math_zero_3(con->frame+3);
  }
}


//----------------------------  flex collisions ---------------------------------------------

// geom-elem or elem-elem or vert-elem convex collision using ccd
int SIM_c_ConvexElem(const sim_model_t* m, const sim_data_t* d, sim_contact_t* con,
                   int g1, int f1, int e1, int v1, int f2, int e2, sim_scalar_t margin) {
  sim_collision_ccd_object_t obj1, obj2;
  SIM_c_initCCDObj(&obj1, m, d, g1, margin);
  SIM_c_initCCDObj(&obj2, m, d, -1, margin);
  SIM_c_setCCDObjFlex(&obj1, f1, e1, v1);
  SIM_c_setCCDObjFlex(&obj2, f2, e2, -1);

  // find contacts
  int ncon = SIM_c_CCDIteration(m, d, &obj1, &obj2, con, 1, margin);

  // fix normals for 2D flex
  if (ncon && !SIM_DISABLED(SIM_DSBL_NATIVECCD)) {
    // check if either object is a 2D flex
    int isflex2d = 0;
    if (f1 >= 0 && m->flex_dim[f1] == 2) isflex2d = 1;
    if (f2 >= 0 && m->flex_dim[f2] == 2) isflex2d = 1;

    if (isflex2d) {
      for (int i = 0; i < ncon; i++) {
        SIM_c_fixNormal(m, d, con + i, g1, -1);
      }
    }
  }

  return ncon;
}


// test a height field and a flex element for collision
int SIM_c_HFieldElem(const sim_model_t* m, const sim_data_t* d, sim_contact_t* con,
                   int g, int f, int e, sim_scalar_t margin) {
  sim_scalar_t vec[3], dx, dy;
  sim_scalar_t xmin, xmax, ymin, ymax, zmin, zmax;
  int dr[2], cnt, rmin, rmax, cmin, cmax;
  sim_collision_ccd_object_t obj1;
  obj1.center = SIM_c_prism_center;
  obj1.support = SIM_c_prism_support;

  // get hfield info
  int hid = m->geom_dataid[g];
  int nrow = m->hfield_nrow[hid];
  int ncol = m->hfield_ncol[hid];
  sim_scalar_t* hpos = d->geom_xpos + 3*g;
  sim_scalar_t* hmat = d->geom_xmat + 9*g;
  sim_scalar_t* hsize = m->hfield_size + 4*hid;
  const float* hdata = m->hfield_data + m->hfield_adr[hid];

  // get elem indo
  int dim = m->flex_dim[f];
  const int* edata = m->flex_elem + m->flex_elemdataadr[f] + e*(dim+1);
  sim_scalar_t* evert[4] = {NULL, NULL, NULL, NULL};
  for (int i=0; i <= dim; i++) {
    evert[i] = d->flexvert_xpos + 3*(m->flex_vertadr[f] + edata[i]);
  }
  sim_scalar_t* ecenter = d->flexelem_aabb + 6*(m->flex_elemadr[f]+e);

  // ccd-related
  ccd_vec3_t dirccd, vecccd;
  ccd_real_t depth;
  sim_collision_ccd_object_t obj2;
  SIM_c_initCCDObj(&obj2, m, d, -1, margin);
  SIM_c_setCCDObjFlex(&obj2, f, e, -1);
  ccd_t ccd;

  //------------------------------------- AABB computation, box-box test

  // save elem vertices, transform to hfield frame
  sim_scalar_t savevert[4][3];
  for (int i=0; i <= dim; i++) {
    sim_math_internal_copy_3(savevert[i], evert[i]);
    sim_math_internal_sub3(vec, evert[i], hpos);
    sim_math_internal_mulMatTVec3(evert[i], hmat, vec);
  }

  // save elem center, transform to hfield frame
  sim_scalar_t savecenter[3];
  sim_math_internal_copy_3(savecenter, ecenter);
  sim_math_internal_sub3(vec, ecenter, hpos);
  sim_math_internal_mulMatTVec3(ecenter, hmat, vec);

  // compute elem bounding box (in hfield frame)
  xmin = xmax = evert[0][0];
  ymin = ymax = evert[0][1];
  zmin = zmax = evert[0][2];
  for (int i=1; i <= dim; i++) {
    xmin = sim_math_min(xmin, evert[i][0]);
    xmax = sim_math_max(xmax, evert[i][0]);
    ymin = sim_math_min(ymin, evert[i][1]);
    ymax = sim_math_max(ymax, evert[i][1]);
    zmin = sim_math_min(zmin, evert[i][2]);
    zmax = sim_math_max(zmax, evert[i][2]);
  }

  // box-box test
  if ((xmin-margin > hsize[0]) || (xmax+margin < -hsize[0]) ||
      (ymin-margin > hsize[1]) || (ymax+margin < -hsize[1]) ||
      (zmin-margin > hsize[2]) || (zmax+margin < -hsize[3])) {
    // restore vertices and center
    for (int i=0; i <= dim; i++) {
      sim_math_internal_copy_3(evert[i], savevert[i]);
    }
    sim_math_internal_copy_3(ecenter, savecenter);

    return 0;
  }

  // compute sub-grid bounds
  cmin = (int) sim_math_floor((xmin + hsize[0]) / (2*hsize[0]) * (ncol-1));
  cmax = (int) sim_math_ceil ((xmax + hsize[0]) / (2*hsize[0]) * (ncol-1));
  rmin = (int) sim_math_floor((ymin + hsize[1]) / (2*hsize[1]) * (nrow-1));
  rmax = (int) sim_math_ceil ((ymax + hsize[1]) / (2*hsize[1]) * (nrow-1));
  cmin = SIM_MAX(0, cmin);
  cmax = SIM_MIN(ncol-1, cmax);
  rmin = SIM_MAX(0, rmin);
  rmax = SIM_MIN(nrow-1, rmax);

  //------------------------------------- collision testing

  // init ccd structure
  CCD_INIT(&ccd);
  ccd.first_dir = prism_firstdir;
  ccd.center1 = SIM_ccd_prism_center;
  ccd.center2 = SIM_ccd_center;
  ccd.support1 = SIM_ccd_prism_support;
  ccd.support2 = SIM_ccd_support;

  // set ccd parameters
  ccd.max_iterations = m->opt.ccd_iterations;
  ccd.mpr_tolerance = m->opt.ccd_tolerance;

  // compute real-valued grid step, and triangulation direction
  dx = (2.0*hsize[0]) / (ncol-1);
  dy = (2.0*hsize[1]) / (nrow-1);
  dr[0] = 1;
  dr[1] = 0;

  // set zbottom value using base size
  obj1.prism[0][2] = obj1.prism[1][2] = obj1.prism[2][2] = -hsize[3];

  // process all prisms in sub-grid
  cnt = 0;
  for (int r=rmin; r < rmax; r++) {
    int nvert = 0;
    for (int c=cmin; c <= cmax; c++) {
      for (int k=0; k < 2; k++) {
        // send vertex to prism constructor
        addVert(&obj1, dx*c-hsize[0], dy*(r+dr[k])-hsize[1],
                hdata[(r+dr[k])*ncol+c]*hsize[2]+margin);

        // check for enough vertices
        if (++nvert > 2) {
          // prism height test
          if (obj1.prism[3][2] < zmin && obj1.prism[4][2] < zmin && obj1.prism[5][2] < zmin) {
            continue;
          }

          // run ccd, save contact
          if (SIM_c_penetration(m, &obj1, &obj2, &ccd, &depth, &dirccd, &vecccd) == 0) {
            if (!ccdVec3Eq(&dirccd, ccd_vec3_origin)) {
              // fill in contact data, transform to global coordinates
              con[cnt].dist = -depth;
              sim_math_internal_mulMatVec3(con[cnt].frame, hmat, dirccd.v);
              sim_math_internal_mulMatVec3(con[cnt].pos, hmat, vecccd.v);
              sim_math_internal_addTo3(con[cnt].pos, hpos);
              sim_math_zero_3(con[cnt].frame+3);

              // count, stop if max number reached
              cnt++;
              if (cnt >= SIM_MAXCONPAIR) {
                r = rmax+1;
                c = cmax+1;
                k = 3;
                break;
              }
            }
          }
        }
      }
    }
  }

  // restore elem vertices and center
  for (int i=0; i <= dim; i++) {
    sim_math_internal_copy_3(evert[i], savevert[i]);
  }
  sim_math_internal_copy_3(ecenter, savecenter);

  return cnt;
}
