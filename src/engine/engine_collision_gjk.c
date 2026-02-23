// Copyright 2024 DeepMind Technologies Limited
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

#include "engine/engine_collision_gjk.h"

#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <simcore/SIM_tnum.h>
#include <simcore/SIM_model.h>
#include "engine/engine_collision_convex.h"
#include "engine/engine_macro.h"
#include "engine/engine_util_blas.h"
#include "engine/engine_util_errmem.h"

#define SIM_MINVAL2 (SIM_MINVAL * SIM_MINVAL)
#define SIM_MAXVAL2 (SIM_MAXVAL * SIM_MAXVAL)

// subdistance algorithm for GJK that computes the barycentric coordinates of the point in a
// simplex closest to the origin
// implementation adapted from Montanari et al, ToG 2017
static void subdistance(sim_scalar_t lambda[4], int n, const Vertex simplex[4]);

// compute the barycentric coordinates of the closest point to the origin in the n-simplex,
// for n = 3, 2, 1 respectively
static void S3D(sim_scalar_t lambda[4], const sim_scalar_t s1[3], const sim_scalar_t s2[3], const sim_scalar_t s3[3],
                const sim_scalar_t s4[3]);
static void S2D(sim_scalar_t lambda[3], const sim_scalar_t s1[3], const sim_scalar_t s2[3], const sim_scalar_t s3[3]);
static void S1D(sim_scalar_t lambda[2], const sim_scalar_t s1[3], const sim_scalar_t s2[3]);

// compute the support point for GJK
static void gjkSupport(Vertex* v, sim_collision_ccd_object_t* obj1, sim_collision_ccd_object_t* obj2,
                       const sim_scalar_t x_k[3], sim_scalar_t x_norm);

// compute the linear combination of 1 - 4 3D vectors
static inline void lincomb(sim_scalar_t res[3], const sim_scalar_t* coef, int n, const sim_scalar_t v1[3],
                           const sim_scalar_t v2[3], const sim_scalar_t v3[3], const sim_scalar_t v4[3]);

// one face in a polytope
typedef struct {
  int verts[3];      // indices of the three vertices of the face in the polytope
  int adj[3];        // adjacent faces, one for each edge: [v1,v2], [v2,v3], [v3,v1]
  sim_scalar_t v[3];       // projection of the origin on face, can be used as face normal
  sim_scalar_t dist2;      // squared norm of v; negative if deleted
  int index;         // index in map; -1: not in map, -2: deleted from polytope
} Face;

// polytope used in the Expanding Polytope Algorithm (EPA)
typedef struct {
  Vertex* verts;      // list of vertices that make up the polytope
  int nverts;         // number of vertices
  Face* faces;        // list of faces that make up the polytope
  int nfaces;         // number of faces
  int maxfaces;       // max number of faces that can be stored in polytope
  Face** map;         // linear map storing faces
  int nmap;           // number of faces in map
  struct Horizon {    // polytope boundary edges that can be seen from w
    int* indices;     // indices of faces on horizon
    int* edges;       // corresponding edge of each face on the horizon
    int nedges;       // number of edges in horizon
    const sim_scalar_t* w;  // point where horizon is created
  } horizon;
} Polytope;

// compute the support point for EPA
static int epaSupport(Polytope* pt, sim_collision_ccd_object_t* obj1, sim_collision_ccd_object_t* obj2,
                      const sim_scalar_t d[3], sim_scalar_t dnorm);

// make copy of vertex in polytope and return its index
static int insertVertex(Polytope* pt, const Vertex* v);

// attach a face to the polytope with the given vertex indices; return squared distance to origin
static sim_scalar_t attachFace(Polytope* pt, int v1, int v2, int v3, int adj1, int adj2, int adj3);

// return 1 if objects are in contact; 0 if not; -1 if inconclusive
// status must have initial tetrahedrons
static int gjkIntersect(SIM_CCDStatus* status, sim_collision_ccd_object_t* obj1, sim_collision_ccd_object_t* obj2);

// create initial polytope for EPA for a 2, 3, or 4-simplex
static int polytope2(Polytope* pt, SIM_CCDStatus* status, sim_collision_ccd_object_t* obj1, sim_collision_ccd_object_t* obj2);
static int polytope3(Polytope* pt, SIM_CCDStatus* status, sim_collision_ccd_object_t* obj1, sim_collision_ccd_object_t* obj2);
static int polytope4(Polytope* pt, SIM_CCDStatus* status, sim_collision_ccd_object_t* obj1, sim_collision_ccd_object_t* obj2);

// return a face of the expanded polytope that best approximates the pentration depth
// witness points are in status->{x1, x2}
static Face* epa(SIM_CCDStatus* status, Polytope* pt, sim_collision_ccd_object_t* obj1, sim_collision_ccd_object_t* obj2);

// -------------------------------- inlined  3D vector utils --------------------------------------

// v1 == v2 up to 1e-15
static inline int equal3(const sim_scalar_t v1[3], const sim_scalar_t v2[3]) {
  return sim_math_abs(v1[0] - v2[0]) < SIM_MINVAL &&
         sim_math_abs(v1[1] - v2[1]) < SIM_MINVAL &&
         sim_math_abs(v1[2] - v2[2]) < SIM_MINVAL;
}

// res = v1 + v2
static inline void add3(sim_scalar_t res[3], const sim_scalar_t v1[3], const sim_scalar_t v2[3]) {
  res[0] = v1[0] + v2[0], res[1] = v1[1] + v2[1], res[2] = v1[2] + v2[2];
}

// res = v1 - v2
static inline void sub3(sim_scalar_t res[3], const sim_scalar_t v1[3], const sim_scalar_t v2[3]) {
  res[0] = v1[0] - v2[0], res[1] = v1[1] - v2[1], res[2] = v1[2] - v2[2];
}

// dot product
static inline sim_scalar_t dot3(const sim_scalar_t v1[3], const sim_scalar_t v2[3]) {
  return v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2];
}

// norm of v
static inline sim_scalar_t norm3(const sim_scalar_t v[3]) {
  return sim_math_sqrt(dot3(v, v));
}

// res = v
static inline void copy3(sim_scalar_t res[3], const sim_scalar_t v[3]) {
  res[0] = v[0], res[1] = v[1], res[2] = v[2];
}

// scalar product: res = s*v
static inline void scl3(sim_scalar_t res[3], const sim_scalar_t v[3], sim_scalar_t s) {
  res[0] = s*v[0], res[1] = s*v[1], res[2] = s*v[2];
}

// cross product: res = v1 x v2
static inline void cross3(sim_scalar_t res[3], const sim_scalar_t v1[3], const sim_scalar_t v2[3]) {
  res[0] = v1[1]*v2[2] - v1[2]*v2[1];
  res[1] = v1[2]*v2[0] - v1[0]*v2[2];
  res[2] = v1[0]*v2[1] - v1[1]*v2[0];
}

// return determinant of the 3x3 matrix with columns v1, v2, v3
static inline sim_scalar_t det3(const sim_scalar_t v1[3], const sim_scalar_t v2[3], const sim_scalar_t v3[3]) {
  // v1 * (v2 x v3)
  return v1[0]*(v2[1]*v3[2] - v2[2]*v3[1])
       + v1[1]*(v2[2]*v3[0] - v2[0]*v3[2])
       + v1[2]*(v2[0]*v3[1] - v2[1]*v3[0]);
}

// ---------------------------------------- GJK ---------------------------------------------------


// return true if both geoms are discrete shapes (i.e. meshes or boxes with no margin)
static int discreteGeoms(sim_collision_ccd_object_t* obj1, sim_collision_ccd_object_t* obj2) {
  // non-zero margin makes geoms smooth
  if (obj1->margin != 0 || obj2->margin != 0) return 0;

  int g1 = obj1->geom_type;
  int g2 = obj2->geom_type;
  return (g1 == SIM_GEOM_MESH || g1 == SIM_GEOM_BOX || g1 == SIM_GEOM_HFIELD) &&
         (g2 == SIM_GEOM_MESH || g2 == SIM_GEOM_BOX || g2 == SIM_GEOM_HFIELD);
}


// GJK algorithm
static void gjk(SIM_CCDStatus* status, sim_collision_ccd_object_t* obj1, sim_collision_ccd_object_t* obj2) {
  int get_dist = status->dist_cutoff > 0;  // need to recover geom distances if not in contact
  int backup_gjk = !get_dist;              // use gjkIntersect if no geom distances needed
  Vertex* simplex = status->simplex;
  int n = 0;                               // number of vertices in the simplex
  int k = 0;                               // current iteration
  int kmax = status->max_iterations;       // max number of iterations
  sim_scalar_t* x1_k = status->x1;               // the kth approximation point for obj1
  sim_scalar_t* x2_k = status->x2;               // the kth approximation point for obj2
  sim_scalar_t x_k[3];                           // the kth approximation point in Minkowski difference
  sim_scalar_t lambda[4] = {1, 0, 0, 0};         // barycentric coordinates for x_k
  sim_scalar_t cutoff2 = status->dist_cutoff * status->dist_cutoff;

  // if both geoms are discrete, finite convergence is guaranteed; set tolerance to 0
  sim_scalar_t epsilon = discreteGeoms(obj1, obj2) ? 0 : 0.5 * status->tolerance * status->tolerance;
  sim_scalar_t x_norm;

  // set initial guess
  sub3(x_k, x1_k, x2_k);

  for (; k < kmax; k++) {
    // compute the kth support point
    x_norm = dot3(x_k, x_k);
    if (x_norm < SIM_MINVAL2) {
      break;
    }
    x_norm = sim_math_sqrt(x_norm);
    gjkSupport(simplex + n, obj1, obj2, x_k, x_norm);
    sim_scalar_t *s_k = simplex[n].vert;

    // stopping criteria using the Frank-Wolfe duality gap given by
    //  |f(x_k) - f(x_min)|^2 <= < grad f(x_k), (x_k - s_k) >
    sim_scalar_t diff[3];
    sub3(diff, x_k, s_k);
    if (dot3(x_k, diff) < epsilon) {
      if (!k) n = 1;
      break;
    }

    // if the hyperplane separates the Minkowski difference and origin, the objects don't collide
    // if geom distance isn't requested, return early
    if (!get_dist) {
      if (dot3(x_k, s_k) > 0) {
        status->gjk_iterations = k;
        status->nsimplex = 0;
        status->nx = 0;
        status->dist = SIM_MAX_LIMIT;
        return;
      }
    } else if (status->dist_cutoff < SIM_MAX_LIMIT) {
      sim_scalar_t vs = dot3(x_k, s_k), vv = dot3(x_k, x_k);
      if (dot3(x_k, s_k) > 0 && (vs*vs / vv) >= cutoff2) {
        status->gjk_iterations = k;
        status->nsimplex = 0;
        status->nx = 0;
        status->dist = SIM_MAX_LIMIT;
        return;
      }
    }

    // tetrahedron is generated and only need contact info; fallback to gjkIntersect to
    // determine contact
    if (n == 3 && backup_gjk) {
      status->gjk_iterations = k;
      int ret = gjkIntersect(status, obj1, obj2);
      if (ret != -1) {
        status->nx = 0;
        status->dist = ret > 0 ? 0 : SIM_MAX_LIMIT;
        return;
      }
      k = status->gjk_iterations;
      backup_gjk = 0;
    }

    // run the distance subalgorithm to compute the barycentric coordinates
    // of the closest point to the origin in the simplex
    subdistance(lambda, n + 1, simplex);

    // remove vertices from the simplex no longer needed
    n = 0;
    for (int i = 0; i < 4; i++) {
      if (!lambda[i]) continue;
      simplex[n] = simplex[i];
      lambda[n++] = lambda[i];
    }

    // SHOULD NOT OCCUR
    if (n < 1) {
      status->gjk_iterations = k;
      status->nsimplex = 0;
      status->nx = 0;
      status->dist = SIM_MAX_LIMIT;
      return;
    }

    // get the next iteration of x_k
    sim_scalar_t x_next[3];
    lincomb(x_next, lambda, n, simplex[0].vert, simplex[1].vert,
            simplex[2].vert, simplex[3].vert);

    // x_k has converged to minimum
    if (equal3(x_next, x_k)) {
      break;
    }

    // copy next iteration into x_k
    copy3(x_k, x_next);

    // we have a tetrahedron containing the origin so return early
    if (n == 4) {
      x_norm = 0;
      break;
    }
  }

  // compute the approximate witness points
  lincomb(x1_k, lambda, n, simplex[0].vert1, simplex[1].vert1, simplex[2].vert1,
          simplex[3].vert1);
  lincomb(x2_k, lambda, n, simplex[0].vert2, simplex[1].vert2, simplex[2].vert2,
          simplex[3].vert2);

  status->nx = 1;
  status->gjk_iterations = k;
  status->nsimplex = n;
  status->dist = x_norm;
}


// compute the support point in obj1 and obj2 for Minkowski difference
static inline void support(Vertex* v, sim_collision_ccd_object_t* obj1, sim_collision_ccd_object_t* obj2,
                           const sim_scalar_t dir[3], const sim_scalar_t dir_neg[3]) {
  // obj1
  obj1->support(v->vert1, obj1, dir);
  if (obj1->margin > 0 && obj1->geom >= 0) {
    sim_scalar_t margin = 0.5 * obj1->margin;
    v->vert1[0] += dir[0] * margin;
    v->vert1[1] += dir[1] * margin;
    v->vert1[2] += dir[2] * margin;
  }

  // obj2
  obj2->support(v->vert2, obj2, dir_neg);
  if (obj2->margin > 0 && obj2->geom >= 0) {
    sim_scalar_t margin = 0.5 * obj2->margin;
    v->vert2[0] += dir_neg[0] * margin;
    v->vert2[1] += dir_neg[1] * margin;
    v->vert2[2] += dir_neg[2] * margin;
  }

  // compute S_{A-B}(dir) = S_A(dir) - S_B(-dir)
  sub3(v->vert, v->vert1, v->vert2);

  // copy vertex indices of discrete geoms
  v->index1 = obj1->vertindex;
  v->index2 = obj2->vertindex;
}


// compute the support points in obj1 and obj2 for the kth approximation point
static inline void gjkSupport(Vertex* v, sim_collision_ccd_object_t* obj1, sim_collision_ccd_object_t* obj2,
                              const sim_scalar_t x_k[3], sim_scalar_t x_norm) {
  sim_scalar_t dir[3], dir_neg[3];

  // SIM_c_support requires a normalized direction
  scl3(dir_neg, x_k, 1 / x_norm);
  scl3(dir, dir_neg, -1);
  support(v, obj1, obj2, dir, dir_neg);
}


// compute support points in Minkowski difference, return index of new vertex in polytope
static int epaSupport(Polytope* pt, sim_collision_ccd_object_t* obj1, sim_collision_ccd_object_t* obj2,
                     const sim_scalar_t d[3], sim_scalar_t dnorm) {
  sim_scalar_t dir[3] = {1, 0, 0}, dir_neg[3] = {-1, 0, 0};

  // SIM_c_support assumes a normalized direction
  if (dnorm > SIM_MINVAL) {
    dir[0] = d[0] / dnorm;
    dir[1] = d[1] / dnorm;
    dir[2] = d[2] / dnorm;
    scl3(dir_neg, dir, -1);
  }

  int n = pt->nverts++;
  Vertex* v = pt->verts + n;
  support(v, obj1, obj2, dir, dir_neg);
  return n;
}


// compute the support point in the Minkowski difference for gjkIntersect (without normalization)
static void gjkIntersectSupport(Vertex* v, sim_collision_ccd_object_t* obj1, sim_collision_ccd_object_t* obj2,
                                const sim_scalar_t dir[3]) {
  sim_scalar_t dir_neg[3] = {-dir[0], -dir[1], -dir[2]};
  support(v, obj1, obj2, dir, dir_neg);
}


// compute the signed distance of a face along with the normal
static inline sim_scalar_t signedDistance(sim_scalar_t normal[3], const Vertex* v1, const Vertex* v2,
                                    const Vertex* v3) {
  sim_scalar_t diff1[3], diff2[3];
  sub3(diff1, v3->vert, v1->vert);
  sub3(diff2, v2->vert, v1->vert);
  cross3(normal, diff1, diff2);
  sim_scalar_t norm2 = dot3(normal, normal);
  if (norm2 > SIM_MINVAL2 && norm2 < SIM_MAXVAL2) {
    scl3(normal, normal, 1 / sim_math_sqrt(norm2));
    return dot3(normal, v1->vert);
  }
  return SIM_MAX_LIMIT;  // cannot recover normal (ignore face)
}


// return 1 if objects are in contact; 0 if not; -1 if inconclusive
static int gjkIntersect(SIM_CCDStatus* status, sim_collision_ccd_object_t* obj1, sim_collision_ccd_object_t* obj2) {
  Vertex simplex[4] = {status->simplex[0], status->simplex[1],
                       status->simplex[2], status->simplex[3]};
  int s[4] = {0, 1, 2, 3};

  int k = status->gjk_iterations, kmax = status->max_iterations;
  for (; k < kmax; k++) {
    // compute the signed distance to each face in the simplex along with normals
    sim_scalar_t dist[4], normals[12];
    dist[0] = signedDistance(&normals[0], simplex + s[2], simplex + s[1], simplex + s[3]);
    dist[1] = signedDistance(&normals[3], simplex + s[0], simplex + s[2], simplex + s[3]);
    dist[2] = signedDistance(&normals[6], simplex + s[1], simplex + s[0], simplex + s[3]);
    dist[3] = signedDistance(&normals[9], simplex + s[0], simplex + s[1], simplex + s[2]);

    // if origin is on any affine hull, convergence will fail
    if (!dist[3] || !dist[2] || !dist[1] || !dist[0]) {
      status->gjk_iterations = k;
      return -1;
    }

    // find the face with the smallest distance to the origin
    int i = (dist[0] < dist[1]) ? 0 : 1;
    int j = (dist[2] < dist[3]) ? 2 : 3;
    int index = (dist[i] < dist[j]) ? i : j;

    // origin inside of simplex (run EPA for contact information)
    if (dist[index] > 0) {
      status->nsimplex = 4;
      status->simplex[0] = simplex[s[0]];
      status->simplex[1] = simplex[s[1]];
      status->simplex[2] = simplex[s[2]];
      status->simplex[3] = simplex[s[3]];
      status->gjk_iterations = k;
      return 1;
    }

    // replace worst vertex (farthest from origin) with new candidate
    gjkIntersectSupport(simplex + s[index], obj1, obj2, normals + 3*index);

    // found origin outside the Minkowski difference (return no collision)
    if (dot3(&normals[3*index], simplex[s[index]].vert) < 0) {
      status->nsimplex = 0;
      status->gjk_iterations = k;
      return 0;
    }

    // swap vertices in the simplex to retain orientation
    i = (index + 1) & 3;
    j = (index + 2) & 3;
    int swap = s[i];
    s[i] = s[j];
    s[j] = swap;
  }
  status->gjk_iterations = k;
  return -1;  // never found origin
}


// linear combination of n 3D vectors
static inline void lincomb(sim_scalar_t res[3], const sim_scalar_t* coef, int n, const sim_scalar_t v1[3],
                           const sim_scalar_t v2[3], const sim_scalar_t v3[3], const sim_scalar_t v4[3]) {
  switch (n) {
    case 1:
      res[0] = coef[0]*v1[0];
      res[1] = coef[0]*v1[1];
      res[2] = coef[0]*v1[2];
      break;
    case 2:
      res[0] = coef[0]*v1[0] + coef[1]*v2[0];
      res[1] = coef[0]*v1[1] + coef[1]*v2[1];
      res[2] = coef[0]*v1[2] + coef[1]*v2[2];
      break;
    case 3:
      res[0] = coef[0]*v1[0] + coef[1]*v2[0] + coef[2]*v3[0];
      res[1] = coef[0]*v1[1] + coef[1]*v2[1] + coef[2]*v3[1];
      res[2] = coef[0]*v1[2] + coef[1]*v2[2] + coef[2]*v3[2];
      break;
    case 4:
      res[0] = coef[0]*v1[0] + coef[1]*v2[0] + coef[2]*v3[0] + coef[3]*v4[0];
      res[1] = coef[0]*v1[1] + coef[1]*v2[1] + coef[2]*v3[1] + coef[3]*v4[1];
      res[2] = coef[0]*v1[2] + coef[1]*v2[2] + coef[2]*v3[2] + coef[3]*v4[2];
      break;
  }
}


// res = origin projected onto plane defined by v1, v2, v3
static int projectOriginPlane(sim_scalar_t res[3], const sim_scalar_t v1[3], const sim_scalar_t v2[3],
                              const sim_scalar_t v3[3]) {
  sim_scalar_t diff21[3], diff31[3], diff32[3], n[3], nv, nn;
  sub3(diff21, v2, v1);
  sub3(diff31, v3, v1);
  sub3(diff32, v3, v2);

  // n = (v1 - v2) x (v3 - v2)
  cross3(n, diff32, diff21);
  nv = dot3(n, v2);
  nn = dot3(n, n);
  if (nn == 0) return 1;
  if (nv != 0 && nn > SIM_MINVAL) {
    scl3(res, n, nv / nn);
    return 0;
  }

  // n = (v2 - v1) x (v3 - v1)
  cross3(n, diff21, diff31);
  nv = dot3(n, v1);
  nn = dot3(n, n);
  if (nn == 0) return 1;
  if (nv != 0 && nn > SIM_MINVAL) {
    scl3(res, n, nv / nn);
    return 0;
  }

  // n = (v1 - v3) x (v2 - v3)
  cross3(n, diff31, diff32);
  nv = dot3(n, v3);
  nn = dot3(n, n);
  scl3(res, n, nv / nn);
  return 0;
}


// res = origin projected onto line defined by v1, v2
static inline void projectOriginLine(sim_scalar_t res[3], const sim_scalar_t v1[3], const sim_scalar_t v2[3]) {
  // res = v2 - <v2, v2 - v1> / <v2 - v1, v2 - v1> * (v2 - v1)
  sim_scalar_t diff[3];
  sub3(diff, v2, v1);
  sim_scalar_t scl = -(dot3(v2, diff) / dot3(diff, diff));
  res[0] = v2[0] + scl*diff[0];
  res[1] = v2[1] + scl*diff[1];
  res[2] = v2[2] + scl*diff[2];
}


// return 1 if both numbers are positive, -1 if both negative and 0 otherwise
static inline int sameSign2(sim_scalar_t a, sim_scalar_t b) {
  if (a > 0 && b > 0) return 1;
  if (a < 0 && b < 0) return -1;
  return 0;
}


// subdistance algorithm for GJK that computes the barycentric coordinates of the point in a
// simplex closest to the origin
// implementation adapted from Montanari et al, ToG 2017
static inline void subdistance(sim_scalar_t lambda[4], int n, const Vertex simplex[4]) {
  memset(lambda, 0, 4 * sizeof(sim_scalar_t));
  const sim_scalar_t* s1 = simplex[0].vert;
  const sim_scalar_t* s2 = simplex[1].vert;
  const sim_scalar_t* s3 = simplex[2].vert;
  const sim_scalar_t* s4 = simplex[3].vert;

  switch (n) {
  case 4:
     S3D(lambda, s1, s2, s3, s4);
     break;
  case 3:
     S2D(lambda, s1, s2, s3);
      break;
  case 2:
     S1D(lambda, s1, s2);
     break;
  default:
    lambda[0] = 1;
    break;
  }
}


static void S3D(sim_scalar_t lambda[4], const sim_scalar_t s1[3], const sim_scalar_t s2[3],
                const sim_scalar_t s3[3], const sim_scalar_t s4[3]) {
  // the matrix M is given by
  //  [[ s1_x, s2_x, s3_x, s4_x ],
  //   [ s1_y, s2_y, s3_y, s4_y ],
  //   [ s1_z, s2_z, s3_z, s4_z ],
  //   [ 1,    1,    1,    1    ]]
  // we want to solve M*lambda = P, where P = [p_x, p_y, p_z, 1] with [p_x, p_y, p_z] is the
  // origin projected onto the simplex

  // compute cofactors to find det(M)
  sim_scalar_t C41 = -det3(s2, s3, s4);
  sim_scalar_t C42 =  det3(s1, s3, s4);
  sim_scalar_t C43 = -det3(s1, s2, s4);
  sim_scalar_t C44 =  det3(s1, s2, s3);

  // note that m_det = 6*SignVol(simplex) with C4i corresponding to the volume of the 3-simplex
  // with vertices {s1, s2, s3, 0} - si
  sim_scalar_t m_det = C41 + C42 + C43 + C44;

  int comp1 = sameSign2(m_det, C41),
      comp2 = sameSign2(m_det, C42),
      comp3 = sameSign2(m_det, C43),
      comp4 = sameSign2(m_det, C44);

  // if all signs are the same then the origin is inside the simplex
  if (comp1 && comp2 && comp3 && comp4) {
    lambda[0] = C41 / m_det;
    lambda[1] = C42 / m_det;
    lambda[2] = C43 / m_det;
    lambda[3] = C44 / m_det;
    return;
  }

  // find the smallest distance, and use the corresponding barycentric coordinates
  sim_scalar_t dmin = SIM_MAX_LIMIT;

  if (!comp1) {
    sim_scalar_t lambda_2d[3], x[3];
    S2D(lambda_2d, s2, s3, s4);
    lincomb(x, lambda_2d, 3, s2, s3, s4, NULL);
    sim_scalar_t d = dot3(x, x);
    lambda[0] = 0;
    lambda[1] = lambda_2d[0];
    lambda[2] = lambda_2d[1];
    lambda[3] = lambda_2d[2];
    dmin = d;
  }

  if (!comp2) {
    sim_scalar_t lambda_2d[3], x[3];
    S2D(lambda_2d, s1, s3, s4);
    lincomb(x, lambda_2d, 3, s1, s3, s4, NULL);
    sim_scalar_t d = dot3(x, x);
    if (d < dmin) {
      lambda[0] = lambda_2d[0];
      lambda[1] = 0;
      lambda[2] = lambda_2d[1];
      lambda[3] = lambda_2d[2];
      dmin = d;
    }
  }

  if (!comp3) {
    sim_scalar_t lambda_2d[3], x[3];
    S2D(lambda_2d, s1, s2, s4);
    lincomb(x, lambda_2d, 3, s1, s2, s4, NULL);
    sim_scalar_t d = dot3(x, x);
    if (d < dmin) {
      lambda[0] = lambda_2d[0];
      lambda[1] = lambda_2d[1];
      lambda[2] = 0;
      lambda[3] = lambda_2d[2];
      dmin = d;
    }
  }

  if (!comp4) {
    sim_scalar_t lambda_2d[3], x[3];
    S2D(lambda_2d, s1, s2, s3);
    lincomb(x, lambda_2d, 3, s1, s2, s3, NULL);
    sim_scalar_t d = dot3(x, x);
    if (d < dmin) {
      lambda[0] = lambda_2d[0];
      lambda[1] = lambda_2d[1];
      lambda[2] = lambda_2d[2];
    }
  }
}


static void S2D(sim_scalar_t lambda[3], const sim_scalar_t s1[3], const sim_scalar_t s2[3], const sim_scalar_t s3[3]) {
  // project origin onto affine hull of the simplex
  sim_scalar_t p_o[3];
  if (projectOriginPlane(p_o, s1, s2, s3)) {
    S1D(lambda, s1, s2);
    lambda[2] = 0;
    return;
  }

  // Below are the minors M_i4 of the matrix M given by
  // [[ s1_x, s2_x, s3_x, s4_x ],
  //  [ s1_y, s2_y, s3_y, s4_y ],
  //  [ s1_z, s2_z, s3_z, s4_z ],
  //  [ 1,    1,    1,    1    ]]
  sim_scalar_t M_14 = s2[1]*s3[2] - s2[2]*s3[1] - s1[1]*s3[2] + s1[2]*s3[1] + s1[1]*s2[2] - s1[2]*s2[1];
  sim_scalar_t M_24 = s2[0]*s3[2] - s2[2]*s3[0] - s1[0]*s3[2] + s1[2]*s3[0] + s1[0]*s2[2] - s1[2]*s2[0];
  sim_scalar_t M_34 = s2[0]*s3[1] - s2[1]*s3[0] - s1[0]*s3[1] + s1[1]*s3[0] + s1[0]*s2[1] - s1[1]*s2[0];

  // exclude the axis with the largest projection of the simplex using the computed minors
  sim_scalar_t M_max = 0;
  sim_scalar_t s1_2D[2], s2_2D[2], s3_2D[2], p_o_2D[2];
  sim_scalar_t mu1 = sim_math_abs(M_14), mu2 = sim_math_abs(M_24), mu3 = sim_math_abs(M_34);
  if (mu1 >= mu2 && mu1 >= mu3) {
    M_max = M_14;
    s1_2D[0] = s1[1];
    s1_2D[1] = s1[2];

    s2_2D[0] = s2[1];
    s2_2D[1] = s2[2];

    s3_2D[0] = s3[1];
    s3_2D[1] = s3[2];

    p_o_2D[0] = p_o[1];
    p_o_2D[1] = p_o[2];
  } else if (mu2 >= mu3) {
    M_max = M_24;
    s1_2D[0] = s1[0];
    s1_2D[1] = s1[2];

    s2_2D[0] = s2[0];
    s2_2D[1] = s2[2];

    s3_2D[0] = s3[0];
    s3_2D[1] = s3[2];

    p_o_2D[0] = p_o[0];
    p_o_2D[1] = p_o[2];
  } else {
    M_max = M_34;
    s1_2D[0] = s1[0];
    s1_2D[1] = s1[1];

    s2_2D[0] = s2[0];
    s2_2D[1] = s2[1];

    s3_2D[0] = s3[0];
    s3_2D[1] = s3[1];

    p_o_2D[0] = p_o[0];
    p_o_2D[1] = p_o[1];
  }

  // compute the cofactors C3i of the following matrix:
  // [[ s1_2D[0] - p_o_2D[0], s2_2D[0] - p_o_2D[0], s3_2D[0] - p_o_2D[0] ],
  //  [ s1_2D[1] - p_o_2D[1], s2_2D[1] - p_o_2D[1], s3_2D[1] - p_o_2D[1] ],
  //  [ 1,                    1,                    1                    ]]

  // C31 corresponds to the signed area of 2-simplex: (p_o_2D, s2_2D, s3_2D)
  sim_scalar_t C31 = p_o_2D[0]*s2_2D[1] + p_o_2D[1]*s3_2D[0] + s2_2D[0]*s3_2D[1]
             - p_o_2D[0]*s3_2D[1] - p_o_2D[1]*s2_2D[0] - s3_2D[0]*s2_2D[1];

  // C32 corresponds to the signed area of 2-simplex: (_po_2D, s1_2D, s3_2D)
  sim_scalar_t C32 = p_o_2D[0]*s3_2D[1] + p_o_2D[1]*s1_2D[0] + s3_2D[0]*s1_2D[1]
             - p_o_2D[0]*s1_2D[1] - p_o_2D[1]*s3_2D[0] - s1_2D[0]*s3_2D[1];

  // C33 corresponds to the signed area of 2-simplex: (p_o_2D, s1_2D, s2_2D)
  sim_scalar_t C33 = p_o_2D[0]*s1_2D[1] + p_o_2D[1]*s2_2D[0] + s1_2D[0]*s2_2D[1]
             - p_o_2D[0]*s2_2D[1] - p_o_2D[1]*s1_2D[0] - s2_2D[0]*s1_2D[1];

  int comp1 = sameSign2(M_max, C31),
      comp2 = sameSign2(M_max, C32),
      comp3 = sameSign2(M_max, C33);

  // all the same sign, p_o is inside the 2-simplex
  if (comp1 && comp2 && comp3) {
    lambda[0] = C31 / M_max;
    lambda[1] = C32 / M_max;
    lambda[2] = C33 / M_max;
    return;
  }

  // find the smallest distance, and use the corresponding barycentric coordinates
  sim_scalar_t dmin = SIM_MAX_LIMIT;

  if (!comp1) {
    sim_scalar_t lambda_1d[2], x[3];
    S1D(lambda_1d, s2, s3);
    lincomb(x, lambda_1d, 2, s2, s3, NULL, NULL);
    sim_scalar_t d = dot3(x, x);
    lambda[0] = 0;
    lambda[1] = lambda_1d[0];
    lambda[2] = lambda_1d[1];
    dmin = d;
  }

  if (!comp2) {
    sim_scalar_t lambda_1d[2], x[3];
    S1D(lambda_1d, s1, s3);
    lincomb(x, lambda_1d, 2, s1, s3, NULL, NULL);
    sim_scalar_t d = dot3(x, x);
    if (d < dmin) {
      lambda[0] = lambda_1d[0];
      lambda[1] = 0;
      lambda[2] = lambda_1d[1];
      dmin = d;
    }
  }

  if (!comp3) {
    sim_scalar_t lambda_1d[2], x[3];
    S1D(lambda_1d, s1, s2);
    lincomb(x, lambda_1d, 2, s1, s2, NULL, NULL);
    sim_scalar_t d = dot3(x, x);
    if (d < dmin) {
      lambda[0] = lambda_1d[0];
      lambda[1] = lambda_1d[1];
      lambda[2] = 0;
    }
  }
}


static void S1D(sim_scalar_t lambda[2], const sim_scalar_t s1[3], const sim_scalar_t s2[3]) {
  // find projection of origin onto the 1-simplex:
  sim_scalar_t p_o[3];
  projectOriginLine(p_o, s1, s2);

  // find the axis with the largest projection "shadow" of the simplex
  sim_scalar_t mu = s1[0] - s2[0];
  sim_scalar_t mu_max = mu;
  int index = 0;

  mu = s1[1] - s2[1];
  if (sim_math_abs(mu) >= sim_math_abs(mu_max)) {
    mu_max = mu;
    index = 1;
  }

  mu = s1[2] - s2[2];
  if (sim_math_abs(mu) >= sim_math_abs(mu_max)) {
    mu_max = mu;
    index = 2;
  }

  sim_scalar_t C1 = p_o[index] - s2[index];
  sim_scalar_t C2 = s1[index] - p_o[index];

  // determine if projection of origin lies inside 1-simplex
  int same = sameSign2(mu_max, C1) && sameSign2(mu_max, C2);
  lambda[0] = same ? C1 / mu_max : 0;
  lambda[1] = same ? C2 / mu_max : 1;
}


// ---------------------------------------- EPA ---------------------------------------------------

// replace a 3-simplex with one of its faces
static inline void replaceSimplex3(Polytope* pt, SIM_CCDStatus* status, int v1, int v2, int v3) {
  // reset status simplex
  status->nsimplex = 3;
  status->simplex[0] = pt->verts[v1];
  status->simplex[1] = pt->verts[v2];
  status->simplex[2] = pt->verts[v3];

  // reset polytope
  pt->nfaces = 0;
  pt->nverts = 0;
  pt->nmap = 0;
}


// return 1 if the origin and p3 are on the same side of the plane defined by p0, p1, p2
static int sameSide(const sim_scalar_t p0[3], const sim_scalar_t p1[3],
                    const sim_scalar_t p2[3], const sim_scalar_t p3[3]) {
    sim_scalar_t diff1[3], diff2[3], diff3[3], diff4[3], n[3];
    sub3(diff1, p1, p0);
    sub3(diff2, p2, p0);
    cross3(n, diff1, diff2);

    sub3(diff3, p3, p0);
    sim_scalar_t dot1 = dot3(n, diff3);

    scl3(diff4, p0, -1);
    sim_scalar_t dot2 = dot3(n, diff4);
    if (dot1 > 0 && dot2 > 0) return 1;
    if (dot1 < 0 && dot2 < 0) return 1;
    return 0;
}


// return 1 if the origin is contained in the tetrahedron, 0 otherwise
static int testTetra(const sim_scalar_t p0[3], const sim_scalar_t p1[3],
                     const sim_scalar_t p2[3], const sim_scalar_t p3[3]) {
  return sameSide(p0, p1, p2, p3)
      && sameSide(p1, p2, p3, p0)
      && sameSide(p2, p3, p0, p1)
      && sameSide(p3, p0, p1, p2);
}


// matrix for 120 degrees rotation around given axis
static void rotmat(sim_scalar_t R[9], const sim_scalar_t axis[3]) {
  sim_scalar_t n = norm3(axis);
  sim_scalar_t u1 = axis[0] / n, u2 = axis[1] / n, u3 = axis[2] / n;
  const sim_scalar_t sin = 0.86602540378;  // sin(120 deg)
  const sim_scalar_t cos = -0.5;           // cos(120 deg)
  R[0] = cos + u1*u1*(1 - cos);
  R[1] = u1*u2*(1 - cos) - u3*sin;
  R[2] = u1*u3*(1 - cos) + u2*sin;
  R[3] = u2*u1*(1 - cos) + u3*sin;
  R[4] = cos + u2*u2*(1 - cos);
  R[5] = u2*u3*(1 - cos) - u1*sin;
  R[6] = u1*u3*(1 - cos) - u2*sin;
  R[7] = u2*u3*(1 - cos) + u1*sin;
  R[8] = cos + u3*u3*(1 - cos);
}


// return nonzero if the ray v1v2 intersects the triangle v3v4v5
static inline int rayTriangle(const sim_scalar_t v1[3], const sim_scalar_t v2[3], const sim_scalar_t v3[3],
                              const sim_scalar_t v4[3], const sim_scalar_t v5[3]) {
  sim_scalar_t diff12[3], diff13[3], diff14[3], diff15[3];
  sub3(diff12, v2, v1);
  sub3(diff13, v3, v1);
  sub3(diff14, v4, v1);
  sub3(diff15, v5, v1);

  sim_scalar_t vol1 = det3(diff13, diff14, diff12);
  sim_scalar_t vol2 = det3(diff14, diff15, diff12);
  sim_scalar_t vol3 = det3(diff15, diff13, diff12);

  if (vol1 >= 0 && vol2 >= 0 && vol3 >= 0) return 1;
  if (vol1 <= 0 && vol2 <= 0 && vol3 <= 0) return -1;
  return 0;
}


// create a polytope from a 1-simplex (returns 0 on success)
static int polytope2(Polytope* pt, SIM_CCDStatus* status, sim_collision_ccd_object_t* obj1, sim_collision_ccd_object_t* obj2) {
  sim_scalar_t *v1 = status->simplex[0].vert, *v2 = status->simplex[1].vert;

  sim_scalar_t diff[3];
  sub3(diff, v2, v1);

  // find component with smallest magnitude (so cross product is largest)
  sim_scalar_t value = SIM_MAX_LIMIT;
  int index = 0;
  for (int i = 0; i < 3; i++) {
    if (sim_math_abs(diff[i]) < value) {
      value = sim_math_abs(diff[i]);
      index = i;
    }
  }

  // cross product with best coordinate axis
  sim_scalar_t e[3] = {0, 0, 0};
  e[index] = 1;
  sim_scalar_t d1[3], d2[3], d3[3];
  cross3(d1, e, diff);

  // rotate around the line segment to get three more points spaced 120 degrees apart
  sim_scalar_t R[9];
  rotmat(R, diff);

  sim_math_mul_mat_vec_3(d2, R, d1);
  sim_math_mul_mat_vec_3(d3, R, d2);

  // save vertices and get indices for each one
  int v1i = insertVertex(pt, status->simplex + 0);
  int v2i = insertVertex(pt, status->simplex + 1);
  int v3i = epaSupport(pt, obj1, obj2, d1, norm3(d1));
  int v4i = epaSupport(pt, obj1, obj2, d2, norm3(d2));
  int v5i = epaSupport(pt, obj1, obj2, d3, norm3(d3));

  sim_scalar_t* v3 = pt->verts[v3i].vert;
  sim_scalar_t* v4 = pt->verts[v4i].vert;
  sim_scalar_t* v5 = pt->verts[v5i].vert;

  // build hexahedron
  if (attachFace(pt, v1i, v3i, v4i, 1, 3, 2) < SIM_MINVAL2) {
    replaceSimplex3(pt, status, v1i, v3i, v4i);
    return polytope3(pt, status, obj1, obj2);
  }
  if (attachFace(pt, v1i, v5i, v3i, 2, 4, 0) < SIM_MINVAL2) {
    replaceSimplex3(pt, status, v1i, v5i, v3i);
    return polytope3(pt, status, obj1, obj2);
  }
  if (attachFace(pt, v1i, v4i, v5i, 0, 5, 1) < SIM_MINVAL2) {
    replaceSimplex3(pt, status, v1i, v4i, v5i);
    return polytope3(pt, status, obj1, obj2);
  }
  if (attachFace(pt, v2i, v4i, v3i, 5, 0, 4) < SIM_MINVAL2) {
    replaceSimplex3(pt, status, v2i, v4i, v3i);
    return polytope3(pt, status, obj1, obj2);
  }
  if (attachFace(pt, v2i, v3i, v5i, 3, 1, 5) < SIM_MINVAL2) {
    replaceSimplex3(pt, status, v2i, v3i, v5i);
    return polytope3(pt, status, obj1, obj2);
  }
  if (attachFace(pt, v2i, v5i, v4i, 4, 2, 3) < SIM_MINVAL2) {
    replaceSimplex3(pt, status, v2i, v5i, v4i);
    return polytope3(pt, status, obj1, obj2);
  }

  // check hexahedron is convex
  if (!rayTriangle(v1, v2, v3, v4, v5)) {
    return SIM_EPA_P2_NONCONVEX;
  }

  for (int i = 0; i < 6; i++) {
    pt->map[i] = pt->faces + i;
    pt->faces[i].index = i;
  }
  pt->nmap = 6;

  // valid hexahedron for EPA
  return 0;
}


// compute the affine coordinates of p on the triangle v1v2v3
static void triAffineCoord(sim_scalar_t lambda[3], const sim_scalar_t v1[3], const sim_scalar_t v2[3],
                           const sim_scalar_t v3[3], const sim_scalar_t p[3]) {
  // compute minors as in S2D
  sim_scalar_t M_14 = v2[1]*v3[2] - v2[2]*v3[1] - v1[1]*v3[2] + v1[2]*v3[1] + v1[1]*v2[2] - v1[2]*v2[1];
  sim_scalar_t M_24 = v2[0]*v3[2] - v2[2]*v3[0] - v1[0]*v3[2] + v1[2]*v3[0] + v1[0]*v2[2] - v1[2]*v2[0];
  sim_scalar_t M_34 = v2[0]*v3[1] - v2[1]*v3[0] - v1[0]*v3[1] + v1[1]*v3[0] + v1[0]*v2[1] - v1[1]*v2[0];

  // exclude one of the axes with the largest projection of the simplex using the computed minors
  sim_scalar_t M_max = 0;
  int x, y;
  sim_scalar_t mu1 = sim_math_abs(M_14), mu2 = sim_math_abs(M_24), mu3 = sim_math_abs(M_34);
  if (mu1 >= mu2 && mu1 >= mu3) {
    M_max = M_14;
    x = 1;
    y = 2;
  } else if (mu2 >= mu3) {
    M_max = M_24;
    x = 0;
    y = 2;
  } else {
    M_max = M_34;
    x = 0;
    y = 1;
  }

  // C31 corresponds to the signed area of 2-simplex: (v, s2, s3)
  sim_scalar_t C31 = p[x]*v2[y] + p[y]*v3[x] + v2[x]*v3[y]
             - p[x]*v3[y] - p[y]*v2[x] - v3[x]*v2[y];

  // C32 corresponds to the signed area of 2-simplex: (v, s1, s3)
  sim_scalar_t C32 = p[x]*v3[y] + p[y]*v1[x] + v3[x]*v1[y]
             - p[x]*v1[y] - p[y]*v3[x] - v1[x]*v3[y];

  // C33 corresponds to the signed area of 2-simplex: (v, s1, s2)
  sim_scalar_t C33 = p[x]*v1[y] + p[y]*v2[x] + v1[x]*v2[y]
             - p[x]*v2[y] - p[y]*v1[x] - v2[x]*v1[y];

  // compute affine coordinates
  lambda[0] = C31 / M_max;
  lambda[1] = C32 / M_max;
  lambda[2] = C33 / M_max;
}


// return true if point p and triangle v1v2v3 intersect
static int triPointIntersect(const sim_scalar_t v1[3], const sim_scalar_t v2[3], const sim_scalar_t v3[3],
                             const sim_scalar_t p[3]) {
  sim_scalar_t lambda[3];
  triAffineCoord(lambda, v1, v2, v3, p);
  if (lambda[0] < 0 || lambda[1] < 0 || lambda[2] < 0) {
    return 0;
  }
  sim_scalar_t pr[3], diff[3];
  pr[0] = v1[0]*lambda[0] + v2[0]*lambda[1] + v3[0]*lambda[2];
  pr[1] = v1[1]*lambda[0] + v2[1]*lambda[1] + v3[1]*lambda[2];
  pr[2] = v1[2]*lambda[0] + v2[2]*lambda[1] + v3[2]*lambda[2];
  sub3(diff, pr, p);
  return norm3(diff) < SIM_MINVAL;
}


// create a polytope from a 2-simplex (returns 0 on success)
static int polytope3(Polytope* pt, SIM_CCDStatus* status, sim_collision_ccd_object_t* obj1, sim_collision_ccd_object_t* obj2) {
  // get vertices of simplex from GJK
  const sim_scalar_t *v1 = status->simplex[0].vert,
               *v2 = status->simplex[1].vert,
               *v3 = status->simplex[2].vert;

  // get normals in both directions
  sim_scalar_t diff1[3], diff2[3], n[3], n_neg[3];
  sub3(diff1, v2, v1);
  sub3(diff2, v3, v1);
  cross3(n, diff1, diff2);
  sim_scalar_t n_norm = norm3(n);
  if (n_norm < SIM_MINVAL) {
    return SIM_EPA_P3_BAD_NORMAL;
  }

  // negative of triangle normal n
  scl3(n_neg, n, -1);

  // save vertices and get indices for each one
  int v1i = insertVertex(pt, status->simplex + 0);
  int v2i = insertVertex(pt, status->simplex + 1);
  int v3i = insertVertex(pt, status->simplex + 2);
  int v5i = epaSupport(pt, obj1, obj2, n_neg, n_norm);
  int v4i = epaSupport(pt, obj1, obj2, n, n_norm);
  sim_scalar_t* v4 = pt->verts[v4i].vert;
  sim_scalar_t* v5 = pt->verts[v5i].vert;

  // check that v4 is not contained in the 2-simplex
  if (triPointIntersect(v1, v2, v3, v4)) {
    return SIM_EPA_P3_INVALID_V4;
  }

  // check that v5 is not contained in the 2-simplex
  if (triPointIntersect(v1, v2, v3, v5)) {
    return SIM_EPA_P3_INVALID_V5;
  }

  // if origin does not lie on simplex then we need to check that the hexahedron contains the
  // origin
  //
  // TODO(kylebayes): It's possible for GJK to return a 2-simplex with the origin not contained in
  // it but within tolerance from it. In that case the hexahedron could possibly be constructed
  // that doesn't contain the origin, but nonetheless there is penetration depth.
  if (status->dist > 10*SIM_MINVAL && !testTetra(v1, v2, v3, v4) && !testTetra(v1, v2, v3, v5)) {
    return SIM_EPA_P3_MISSING_ORIGIN;
  }

  // create hexahedron for EPA
  if (attachFace(pt, v4i, v1i, v2i, 1, 3, 2) < SIM_MINVAL2) {
    return SIM_EPA_P3_ORIGIN_ON_FACE;
  }
  if (attachFace(pt, v4i, v3i, v1i, 2, 4, 0) < SIM_MINVAL2) {
    return SIM_EPA_P3_ORIGIN_ON_FACE;
  }
  if (attachFace(pt, v4i, v2i, v3i, 0, 5, 1) < SIM_MINVAL2) {
    return SIM_EPA_P3_ORIGIN_ON_FACE;
  }
  if (attachFace(pt, v5i, v2i, v1i, 5, 0, 4) < SIM_MINVAL2) {
    return SIM_EPA_P3_ORIGIN_ON_FACE;
  }
  if (attachFace(pt, v5i, v1i, v3i, 3, 1, 5) < SIM_MINVAL2) {
    return SIM_EPA_P3_ORIGIN_ON_FACE;
  }
  if (attachFace(pt, v5i, v3i, v2i, 4, 2, 3) < SIM_MINVAL2) {
    return SIM_EPA_P3_ORIGIN_ON_FACE;
  }

  // populate face map
  for (int i = 0; i < 6; i++) {
    pt->map[i] = pt->faces + i;
    pt->faces[i].index = i;
  }
  pt->nmap = 6;
  return 0;
}


// create a polytope from a 3-simplex (returns 0 on success)
static int polytope4(Polytope* pt, SIM_CCDStatus* status, sim_collision_ccd_object_t* obj1, sim_collision_ccd_object_t* obj2) {
  int v1 = insertVertex(pt, status->simplex + 0);
  int v2 = insertVertex(pt, status->simplex + 1);
  int v3 = insertVertex(pt, status->simplex + 2);
  int v4 = insertVertex(pt, status->simplex + 3);

  // if the origin is on a face, replace the 3-simplex with a 2-simplex
  if (attachFace(pt, v1, v2, v3, 1, 3, 2) < SIM_MINVAL2) {
    replaceSimplex3(pt, status, v1, v2, v3);
    return polytope3(pt, status, obj1, obj2);
  }
  if (attachFace(pt, v1, v4, v2, 2, 3, 0) < SIM_MINVAL2) {
    replaceSimplex3(pt, status, v1, v4, v2);
    return polytope3(pt, status, obj1, obj2);
  }
  if (attachFace(pt, v1, v3, v4, 0, 3, 1) < SIM_MINVAL2) {
    replaceSimplex3(pt, status, v1, v3, v4);
    return polytope3(pt, status, obj1, obj2);
  }
  if (attachFace(pt, v4, v3, v2, 2, 0, 1) < SIM_MINVAL2) {
    replaceSimplex3(pt, status, v4, v3, v2);
    return polytope3(pt, status, obj1, obj2);
  }

  if (!testTetra(pt->verts[v1].vert, pt->verts[v2].vert, pt->verts[v3].vert, pt->verts[v4].vert)) {
    return SIM_EPA_P4_MISSING_ORIGIN;
  }

  // populate face map
  for (int i = 0; i < 4; i++) {
    pt->map[i] = pt->faces + i;
    pt->faces[i].index = i;
  }
  pt->nmap = 4;
  return 0;
}


// make a copy of vertex in polytope and return its index
static inline int insertVertex(Polytope* pt, const Vertex* v) {
  int n = pt->nverts++;
  pt->verts[n] = *v;
  return n;
}


// delete face from map (return non-zero on error)
static void deleteFace(Polytope* pt, Face* face) {
  if (face->index >= 0) {
    pt->map[face->index] = pt->map[--pt->nmap];
    pt->map[face->index]->index = face->index;
  }
  face->index = -2;  // mark face as deleted from map and polytope
}


// return max number of faces that can be stored in polytope
static inline int maxFaces(Polytope* pt) {
  return pt->maxfaces - pt->nfaces;
}


// attach a face to the polytope with the given vertex indices; return squared distance to origin
static inline sim_scalar_t attachFace(Polytope* pt, int v1, int v2, int v3,
                                int adj1, int adj2, int adj3) {
  Face* face = &pt->faces[pt->nfaces++];
  face->verts[0] = v1;
  face->verts[1] = v2;
  face->verts[2] = v3;

  // adjacent faces
  face->adj[0] = adj1;
  face->adj[1] = adj2;
  face->adj[2] = adj3;

  // compute witness point v
  int ret = projectOriginPlane(face->v, pt->verts[v3].vert, pt->verts[v2].vert, pt->verts[v1].vert);
  if (ret) {
    return 0;
  }
  face->dist2 = dot3(face->v, face->v);
  face->index = -1;

  return face->dist2;
}


// add an edge to the horizon
static inline void addEdge(Polytope* pt, int index, int edge) {
  pt->horizon.edges[pt->horizon.nedges] = edge;
  pt->horizon.indices[pt->horizon.nedges++] = index;
}


// get edge index where vertex lies
static inline int getEdge(Face* face, int vertex) {
  if (face->verts[0] == vertex) return 0;
  if (face->verts[1] == vertex) return 1;
  return 2;
}


// recursive call to build horizon; return 1 if face is visible from w otherwise 0
static int horizonRec(Polytope* pt, Face* face, int e) {
    // v is visible from w so it is deleted and adjacent faces are checked
    if (dot3(face->v, pt->horizon.w) - face->dist2 > SIM_MINVAL) {
      deleteFace(pt, face);

      // recursively search the adjacent faces on the next two edges
      for (int k = 1; k < 3; k++) {
        int i = (e + k) % 3;
        Face* adjFace = &pt->faces[face->adj[i]];
        if (adjFace->index > -2) {
          int adjEdge = getEdge(adjFace, face->verts[(i + 1) % 3]);
          if (!horizonRec(pt, adjFace, adjEdge)) {
            addEdge(pt, face->adj[i], adjEdge);
          }
        }
      }
      return 1;
    }
  return 0;
}


// create horizon given the face as starting point
static void horizon(Polytope* pt, Face* face) {
  deleteFace(pt, face);

  // first edge
  Face* adjFace = &pt->faces[face->adj[0]];
  int adjEdge = getEdge(adjFace, face->verts[1]);
  if (!horizonRec(pt, adjFace, adjEdge)) {
    addEdge(pt, face->adj[0], adjEdge);
  }

  // second edge
  adjFace = &pt->faces[face->adj[1]];
  adjEdge = getEdge(adjFace, face->verts[2]);
  if (adjFace->index > -2 && !horizonRec(pt, adjFace, adjEdge)) {
    addEdge(pt, face->adj[1], adjEdge);
  }

  // third edge
  adjFace = &pt->faces[face->adj[2]];
  adjEdge = getEdge(adjFace, face->verts[0]);
  if (adjFace->index > -2 && !horizonRec(pt, adjFace, adjEdge)) {
    addEdge(pt, face->adj[2], adjEdge);
  }
}


// recover witness points from EPA polytope, return signed distance between witness points
static sim_scalar_t epaWitness(const Polytope* pt, const Face* face, sim_scalar_t x1[3], sim_scalar_t x2[3]) {
  // compute affine coordinates for witness points on plane defined by face
  sim_scalar_t lambda[3];
  Vertex* v1 = pt->verts + face->verts[0];
  Vertex* v2 = pt->verts + face->verts[1];
  Vertex* v3 = pt->verts + face->verts[2];
  triAffineCoord(lambda, v1->vert, v2->vert, v3->vert, face->v);

  // witness point on geom 1
  lincomb(x1, lambda, 3, v1->vert1, v2->vert1, v3->vert1, NULL);

  // witness point on geom 2
  lincomb(x2, lambda, 3, v1->vert2, v2->vert2, v3->vert2, NULL);

  return -sim_math_sqrt(face->dist2);
}


// return a face of the expanded polytope that best approximates the pentration depth
// witness points are in status->{x1, x2}
static Face* epa(SIM_CCDStatus* status, Polytope* pt, sim_collision_ccd_object_t* obj1, sim_collision_ccd_object_t* obj2) {
  sim_scalar_t upper = SIM_MAX_LIMIT, upper2 = SIM_MAX_LIMIT, lower2;
  Face* face = NULL, *pface = NULL;  // face closest to origin
  sim_scalar_t tolerance = status->tolerance;
  int discrete = discreteGeoms(obj1, obj2);

  // tolerance is not used for discrete geoms
  if (discrete && sizeof(sim_scalar_t) == sizeof(double)) {
    tolerance = SIM_MINVAL;
  }

  int k, kmax = status->max_iterations;
  for (k = 0; k < kmax; k++) {
    pface = face;

    // find the face closest to the origin (lower bound for penetration depth)
    lower2 = SIM_MAX_LIMIT;
    for (int i = 0; i < pt->nmap; i++) {
      if (pt->map[i]->dist2 < lower2) {
        face = pt->map[i];
        lower2 = face->dist2;
      }
    }

    // face not valid, return previous face
    if (lower2 > upper2 || !face) {
      face = pface;
      break;
    }

    // check if lower bound is 0
    if (lower2 <= 0) {
      sim_warning("EPA: origin lies on affine hull of face");
      break;
    }

    // compute support point w from the closest face's normal
    sim_scalar_t lower = sim_math_sqrt(lower2);
    int wi = epaSupport(pt, obj1, obj2, face->v, lower);
    const Vertex* w = pt->verts + wi;
    sim_scalar_t upper_k = dot3(face->v, w->vert) / lower;  // upper bound for kth iteration
    if (upper_k < upper) {
      upper = upper_k;
      upper2 = upper * upper;
    }
    if (upper - lower < tolerance) {
      break;
    }

    // check if vertex w is a repeated support point
    if (discrete) {
      int i = 0, nverts = pt->nverts - 1;
      for (; i < nverts; i++) {
        if (w->index1 == pt->verts[i].index1 && w->index2 == pt->verts[i].index2) {
          break;
        }
      }
      if (i != nverts) {
        break;
      }
    }

    pt->horizon.w = w->vert;
    horizon(pt, face);

    // unrecoverable numerical issue; at least one face was deleted so nedges is 3 or more
    if (pt->horizon.nedges < 3) {
      face = NULL;
      break;
    }

    // insert w as new vertex and attach faces along the horizon
    int nfaces = pt->nfaces, nedges = pt->horizon.nedges;

    // check if there's enough memory to store new faces
    if (nedges > maxFaces(pt)) {
      sim_warning("EPA: out of memory for faces on expanding polytope");
      break;
    }

    // attach first face
    int horIndex = pt->horizon.indices[0], horEdge = pt->horizon.edges[0];
    Face* horFace = &pt->faces[horIndex];
    int v1 = horFace->verts[horEdge],
        v2 = horFace->verts[(horEdge + 1) % 3];
    horFace->adj[horEdge] = nfaces;
    sim_scalar_t dist2 = attachFace(pt, wi, v2, v1, nfaces + nedges - 1, horIndex, nfaces + 1);

    // unrecoverable numerical issue
    if (dist2 == 0) {
      face = NULL;
      break;
    }

    // store face in map
    if (dist2 >= lower2 && dist2 <= upper2) {
      int i = pt->nmap++;
      pt->map[i] = &pt->faces[pt->nfaces - 1];
      pt->map[i]->index = i;
    }

    // attach remaining faces
    for (int i = 1; i < nedges; i++) {
      int cur = nfaces + i;                  // index of attached face
      int next = nfaces + (i + 1) % nedges;  // index of next face

      horIndex = pt->horizon.indices[i], horEdge = pt->horizon.edges[i];
      horFace = &pt->faces[horIndex];
      v1 = horFace->verts[horEdge];
      v2 = horFace->verts[(horEdge + 1) % 3];
      horFace->adj[horEdge] = cur;
      dist2 = attachFace(pt, wi, v2, v1, cur - 1, horIndex, next);

      // unrecoverable numerical issue
      if (dist2 == 0) {
        face = NULL;
        break;
      }

      // store face in map
      if (dist2 >= lower2 && dist2 <= upper2) {
        int idx = pt->nmap++;
        pt->map[idx] = &pt->faces[pt->nfaces - 1];
        pt->map[idx]->index = idx;
      }
    }
    pt->horizon.nedges = 0;  // clear horizon

    // no face candidates left
    if (!pt->nmap || !face) {
      break;
    }
  }

  status->epa_iterations = k;
  if (face) {
    status->dist = epaWitness(pt, face, status->x1, status->x2);
    status->nx = 1;
  } else {
    status->nx = 0;
    status->dist = 0;
  }
  return face;
}


// ------------------------------------- MultiCCD -------------------------------------------------

// compute area of a quadrilateral
static inline sim_scalar_t area4(const sim_scalar_t a[3], const sim_scalar_t b[3],
                           const sim_scalar_t c[3], const sim_scalar_t d[3]) {
  sim_scalar_t ad[3] = {d[0] - a[0], d[1] - a[1], d[2] - a[2]};
  sim_scalar_t db[3] = {b[0] - d[0], b[1] - d[1], b[2] - d[2]};
  sim_scalar_t bc[3] = {c[0] - b[0], c[1] - b[1], c[2] - b[2]};
  sim_scalar_t ca[3] = {a[0] - c[0], a[1] - c[1], a[2] - c[2]};
  sim_scalar_t e[3], f[3], g[3];
  cross3(e, ad, db);
  cross3(f, bc, ca);
  add3(g, e, f);
  return 0.5 * norm3(g);
}


// return pointer to next vertex in a polygon
static inline sim_scalar_t* next(sim_scalar_t* polygon, int nvert, sim_scalar_t* curr) {
  if (curr == polygon + 3*(nvert - 1)) {
    return polygon;
  }
  return curr + 3;
}


// prune a polygon to a maximum area convex quadrilateral
static inline void polygonQuad(sim_scalar_t* res[4], sim_scalar_t* polygon, int nvert) {
  sim_scalar_t* a = polygon, *b = polygon + 3, *c = polygon + 6, *d = polygon + 9;
  res[0] = a, res[1] = b, res[2] = c, res[3] = d;
  sim_scalar_t m = area4(a, b, c, d), m_next;
  sim_scalar_t* end = polygon + 3 * nvert;
  for (; a < end; a += 3) {
    while (1) {
      m_next = area4(a, b, c, next(polygon, nvert, d));
      if (m_next <= m) {
        break;
      }
      m = m_next;
      d = next(polygon, nvert, d);
      res[0] = a, res[1] = b, res[2] = c, res[3] = d;
      while (1) {
        m_next = area4(a, b, next(polygon, nvert, c), d);
        if (m_next <= m) {
          break;
        }
        m = m_next;
        c = next(polygon, nvert, c);
        res[0] = a, res[1] = b, res[2] = c, res[3] = d;
      }
      while (1) {
        m_next = area4(a, next(polygon, nvert, b), c, d);
        if (m_next <= m) {
          break;
        }
        m = m_next;
        b = next(polygon, nvert, b);
        res[0] = a, res[1] = b, res[2] = c, res[3] = d;
      }
    }
    if (b == a) {
      b = next(polygon, nvert, b);
      if (c == b) {
        c = next(polygon, nvert, c);
        if (d == c) {
          d = next(polygon, nvert, d);
        }
      }
    }
  }
}


// find the normal of a plane perpendicular to the face (given by its normal n) and intersecting the
// face edge (v1, v2)
static sim_scalar_t planeNormal(sim_scalar_t res[3], const sim_scalar_t v1[3], const sim_scalar_t v2[3],
                          const sim_scalar_t n[3]) {
  sim_scalar_t v3[3], diff1[3], diff2[3];
  add3(v3, v1, n);
  sub3(diff1, v2, v1);
  sub3(diff2, v3, v1);
  cross3(res, diff1, diff2);
  return dot3(res, v1);
}


// find what side of a plane a point p lies
static int halfspace(const sim_scalar_t a[3], const sim_scalar_t n[3], const sim_scalar_t p[3]) {
  sim_scalar_t diff[3] = {p[0] - a[0], p[1] - a[1], p[2] - a[2]};
  return dot3(diff, n) > -SIM_MINVAL;
}


// compute the intersection of a plane with a line segment (a, b)
static sim_scalar_t planeIntersect(sim_scalar_t res[3], const sim_scalar_t pn[3], sim_scalar_t pd,
                             const sim_scalar_t a[3], const sim_scalar_t b[3]) {
  sim_scalar_t ab[3];
  sub3(ab, b, a);
  sim_scalar_t temp = dot3(pn, ab);
  if (temp == 0.0) return SIM_MAX_LIMIT;  // parallel; no intersection
  sim_scalar_t t = (pd - dot3(pn, a)) / temp;
  if (t >= 0.0 && t <= 1.0) {
    res[0] = a[0] + t*ab[0];
    res[1] = a[1] + t*ab[1];
    res[2] = a[2] + t*ab[2];
  }
  return t;
}


// clip a polygon against another polygon
static void polygonClip(SIM_CCDStatus* status, const sim_scalar_t* face1, int nface1,
                        const sim_scalar_t* face2, int nface2, const sim_scalar_t n[3],
                       const sim_scalar_t dir[3]) {
  // clipping face needs to be at least a triangle
  if (nface1 < 3) {
    return;
  }

  // compute plane normal and distance to plane for each vertex
  sim_scalar_t pn[3 * SIM_MAX_POLYVERT], pd[SIM_MAX_POLYVERT];
  for (int i = 0; i < nface1 - 1; i++) {
    pd[i] = planeNormal(&pn[3*i], &face1[3*i], &face1[3*i + 3], n);
  }
  pd[nface1 - 1] = planeNormal(&pn[3*(nface1 - 1)], &face1[3*(nface1 - 1)], &face1[0], n);

  // reserve 2 * max_sides as max sides for a clipped polygon
  sim_scalar_t polygon1[6 * SIM_MAX_POLYVERT], polygon2[6 * SIM_MAX_POLYVERT], *polygon, *clipped;
  int npolygon = nface2, nclipped = 0;
  polygon = polygon1;
  clipped = polygon2;

  for (int i = 0; i < nface2; i++) {
    copy3(polygon + 3*i, face2 + 3*i);
  }

  // clip the polygon by one edge e at a time
  for (int e = 0; e < (3 * nface1); e += 3) {
    for (int i = 0; i < npolygon; i++) {
      // get edge PQ of the polygon
      sim_scalar_t *P = polygon + 3*i;
      sim_scalar_t *Q = (i < npolygon - 1) ? polygon + 3*(i+1) : polygon;

      // determine if P and Q are in the halfspace of the clipping edge
      int inside1 = halfspace(face1 + e, pn + e, P);
      int inside2 = halfspace(face1 + e, pn + e, Q);

      // PQ entirely outside the clipping edge, skip
      if (!inside1 && !inside2) {
        continue;
      }

      // edge PQ is inside the clipping edge, add Q
      if (inside1 && inside2) {
        copy3(clipped + 3*nclipped++, Q);
        continue;
      }

      // add new vertex to clipped polygon where PQ intersects the clipping edge
      sim_scalar_t t = planeIntersect(clipped + 3*nclipped++, pn + e, pd[e/3], P, Q);
      if (t < 0.0 || t > 1.0) {
        nclipped--;  // no intersection in PQ
      }

      // add Q as PQ is now back inside the clipping edge
      if (inside2) {
        copy3(clipped + 3*nclipped++, Q);
      }
    }

    // swap clipped and polygon
    sim_scalar_t* tmp = polygon;
    polygon = clipped;
    clipped = tmp;
    npolygon = nclipped;
    nclipped = 0;
  }

  if (npolygon < 1) {
    return;
  }

  // copy final clipped polygon to status
  if (status->max_contacts < 5 && npolygon > 4) {
    status->nx = 4;
    sim_scalar_t* rect[4];
    polygonQuad(rect, polygon, npolygon);
    for (int i = 0; i < 4; i++) {
      copy3(status->x2 + 3*i, rect[i]);
      sub3(status->x1 + 3*i, status->x2 + 3*i, dir);
    }
    return;
  }

  // TODO(kylebayes): Consider using a heristic to prune the polygon.
  if (npolygon > SIM_MAXCONPAIR) {
    status->nx = SIM_MAXCONPAIR;
    for (int i = 0; i < 3*SIM_MAXCONPAIR; i += 3) {
      copy3(status->x2 + i, polygon + i);
      sub3(status->x1 + i, status->x2 + i, dir);
    }
    return;
  }

  // no pruning needed
  int k = 0;
  for (int i = 0; i < 3*npolygon; i += 3) {
    int skip = 0;

    // find possible duplicate vertices
    for (int j = 0; j < k; j += 3) {
      if (equal3(status->x2 + j, polygon + i)) {
        skip = 1;
        break;
      }
    }
    if (skip) continue;
    copy3(status->x2 + k, polygon + i);
    sub3(status->x1 + k, status->x2 + k, dir);
    k += 3;
  }
  status->nx = k/3;
}


// compute global coordinates of a local point (l1, l2, l3)
static inline void globalcoord(sim_scalar_t res[3], const sim_scalar_t mat[9], const sim_scalar_t pos[3],
                              sim_scalar_t l1, sim_scalar_t l2, sim_scalar_t l3) {
  // perform mat * (l1, l2, l3) + pos
  res[0] = mat[0]*l1 + mat[1]*l2 + mat[2]*l3;
  res[1] = mat[3]*l1 + mat[4]*l2 + mat[5]*l3;
  res[2] = mat[6]*l1 + mat[7]*l2 + mat[8]*l3;
  if (pos) {
    res[0] += pos[0];
    res[1] += pos[1];
    res[2] += pos[2];
  }
}


// find up to n <= 2 common integers of two arrays, return n
static int intersect(int res[2], const int* arr1, const int* arr2, int n, int m) {
  int count = 0;
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < m; j++) {
      if (arr1[i] == arr2[j]) {
        res[count++] = arr1[i];
        if (count == 2) return 2;
      }
    }
  }
  return count;
}


// compute possible polygon normals of a mesh given up to 3 vertices
static int meshNormals(sim_scalar_t* res, int resind[3], int dim, sim_collision_ccd_object_t* obj,
                      int v1, int v2, int v3) {
  const sim_model_t* m = obj->model;
  const sim_data_t* d = obj->data;
  int g = obj->geom;
  int polyadr = m->mesh_polyadr[m->geom_dataid[g]];
  int vertadr = m->mesh_vertadr[m->geom_dataid[g]];
  const sim_scalar_t* mat = d->geom_xmat + 9*g;

  if (dim == 3) {
    int v1_adr = m->mesh_polymapadr[vertadr + v1];
    int v1_num = m->mesh_polymapnum[vertadr + v1];

    int v2_adr = m->mesh_polymapadr[vertadr + v2];
    int v2_num = m->mesh_polymapnum[vertadr + v2];

    int v3_adr = m->mesh_polymapadr[vertadr + v3];
    int v3_num = m->mesh_polymapnum[vertadr + v3];

    int edgeset[2], faceset[2];
    int n = intersect(edgeset, m->mesh_polymap + v1_adr, m->mesh_polymap + v2_adr, v1_num, v2_num);
    if (n == 0) return 0;
    n = intersect(faceset, edgeset, m->mesh_polymap + v3_adr, n, v3_num);
    if (n == 0) return 0;

    // three vertices on mesh define a unique face
    sim_scalar_t* normal = m->mesh_polynormal + 3*(polyadr + faceset[0]);
    globalcoord(res, mat, NULL, normal[0], normal[1], normal[2]);
    resind[0] = faceset[0];
    return 1;
  }

  if (dim == 2) {
    int v1_adr = m->mesh_polymapadr[vertadr + v1];
    int v1_num = m->mesh_polymapnum[vertadr + v1];

    int v2_adr = m->mesh_polymapadr[vertadr + v2];
    int v2_num = m->mesh_polymapnum[vertadr + v2];

    // up to two faces as vertices on mesh define an edge
    int edgeset[2];
    int n = intersect(edgeset, m->mesh_polymap + v1_adr, m->mesh_polymap + v2_adr, v1_num, v2_num);
    if (n == 0) return 0;
    for (int i = 0; i < n; i++) {
      sim_scalar_t* normal = m->mesh_polynormal + 3*(polyadr + edgeset[i]);
      globalcoord(res + 3*i, mat, NULL, normal[0], normal[1], normal[2]);
      resind[i] = edgeset[i];
    }
    return n;
  }

  if (dim == 1) {
    int v1_adr = m->mesh_polymapadr[vertadr + v1];
    int v1_num = m->mesh_polymapnum[vertadr + v1];

    // cap number of possible faces intersecting at a vertex
    if (v1_num > SIM_MAX_POLYVERT) v1_num = SIM_MAX_POLYVERT;
    for (int i = 0; i < v1_num; i++) {
      int index = m->mesh_polymap[v1_adr + i];
      sim_scalar_t* normal = m->mesh_polynormal + 3*(polyadr + index);
      globalcoord(res + 3*i, mat, NULL, normal[0], normal[1], normal[2]);
      resind[i] = index;
    }
    return v1_num;
  }
  return 0;
}


// compute normal directional vectors along possible edges given by up to two vertices
static int meshEdgeNormals(sim_scalar_t* res, sim_scalar_t* endverts, int dim, sim_collision_ccd_object_t* obj,
                           const sim_scalar_t v1[3], const sim_scalar_t v2[3], int v1i, int v2i) {
  // mesh data
  int g = obj->geom;
  const sim_scalar_t* mat = obj->data->geom_xmat + 9*g;
  const sim_scalar_t* pos = obj->data->geom_xpos + 3*g;

  // only one edge
  if (dim == 2) {
    copy3(endverts, v2);
    sub3(res, v2, v1);
    sim_math_normalize_3(res);
    return 1;
  }

  if (dim == 1) {
    const sim_model_t* m = obj->model;
    int polyadr = m->mesh_polyadr[m->geom_dataid[g]];
    int vertadr = m->mesh_vertadr[m->geom_dataid[g]];

    int v1_adr = m->mesh_polymapadr[vertadr + v1i];
    int v1_num = m->mesh_polymapnum[vertadr + v1i];
    if (v1_num > SIM_MAX_POLYVERT) v1_num = SIM_MAX_POLYVERT;

    const int* polymap = m->mesh_polymap + v1_adr;
    const int* polyvert = m->mesh_polyvert;
    const int* polyvertadr = m->mesh_polyvertadr + polyadr;
    const int* polyvertnum = m->mesh_polyvertnum + polyadr;
    const float* vert = m->mesh_vert + 3*vertadr;

    // loop through all faces with vertex v1
    for (int i = 0; i < v1_num; i++) {
      int idx = polymap[i];
      int adr = polyvertadr[idx];
      int nvert = polyvertnum[idx];
      // find previous vertex in polygon to form edge
      for (int j = 0; j < nvert; j++) {
        if (polyvert[adr + j] == v1i) {
          int k = (j == 0) ? nvert - 1 : j - 1;
          const float* v = vert + 3*polyvert[adr + k];
          globalcoord(endverts + 3*i, mat, pos, v[0], v[1], v[2]);
          sub3(res + 3*i, endverts + 3*i, v1);
          sim_math_normalize_3(res + 3*i);
          break;
        }
      }
    }
    return v1_num;
  }
  return 0;
}


// try recovering box normal from collision normal
static int boxNormals2(sim_scalar_t res[9], int resind[3], const sim_scalar_t mat[9], const sim_scalar_t n[3]) {
  // list of box face normals
  sim_scalar_t normals[18] = {1, 0, 0,    -1,  0,  0,
                        0, 1, 0,     0, -1,  0,
                        0, 0, 1,     0,  0, -1};

  // get local coordinates of the normal
  sim_scalar_t local_n[3];
  local_n[0] = mat[0]*n[0] + mat[3]*n[1] + mat[6]*n[2];
  local_n[1] = mat[1]*n[0] + mat[4]*n[1] + mat[7]*n[2];
  local_n[2] = mat[2]*n[0] + mat[5]*n[1] + mat[8]*n[2];
  scl3(local_n, local_n, 1/sim_math_sqrt(dot3(local_n, local_n)));

  // determine if there is a side close to the normal
  for (int i = 0; i < 6; i++) {
    if (dot3(local_n, normals + 3*i) > SIM_FACE_TOL) {
      globalcoord(res, mat, NULL, normals[3*i], normals[3*i + 1], normals[3*i + 2]);
      resind[0] = i;
      return 1;
    }
  }
  return 0;
}


// compute possible face normals of a box given up to 3 vertices
static int boxNormals(sim_scalar_t res[9], int resind[3], int dim, sim_collision_ccd_object_t* obj,
                      int v1, int v2, int v3, const sim_scalar_t dir[3]) {
  const sim_scalar_t* mat = obj->data->geom_xmat + 9*obj->geom;
  if (dim == 3) {
    int c = 0;
    int x = ((v1 & 1) && (v2 & 1) && (v3 & 1)) - (!(v1 & 1) && !(v2 & 1) && !(v3 & 1));
    int y = ((v1 & 2) && (v2 & 2) && (v3 & 2)) - (!(v1 & 2) && !(v2 & 2) && !(v3 & 2));
    int z = ((v1 & 4) && (v2 & 4) && (v3 & 4)) - (!(v1 & 4) && !(v2 & 4) && !(v3 & 4));
    globalcoord(res, mat, NULL, x, y, z);
    int sgn = x + y + z;
    if (x) resind[c++] = 0;
    if (y) resind[c++] = 2;
    if (z) resind[c++] = 4;
    if (sgn == -1) resind[0]++;
    return c == 1 ? 1 : boxNormals2(res, resind, mat, dir);
  }

  if (dim == 2) {
    int c = 0;
    int x = ((v1 & 1) && (v2 & 1)) - (!(v1 & 1) && !(v2 & 1));
    int y = ((v1 & 2) && (v2 & 2)) - (!(v1 & 2) && !(v2 & 2));
    int z = ((v1 & 4) && (v2 & 4)) - (!(v1 & 4) && !(v2 & 4));
    if (x) {
      globalcoord(res, mat, NULL, x, 0, 0);
      resind[c++] = (x > 0) ? 0 : 1;
    }
    if (y) {
      globalcoord(res + 3*c, mat, NULL, 0, y, 0);
      resind[c++] = (y > 0) ? 2 : 3;
    }
    if (z) {
      globalcoord(res + 3, mat, NULL, 0, 0, z);
      resind[c++] = (z > 0) ? 4 : 5;
    }
    return c == 2 ? 2 : boxNormals2(res, resind, mat, dir);
  }

  if (dim == 1) {
    sim_scalar_t x = (v1 & 1) ? 1 : -1;
    sim_scalar_t y = (v1 & 2) ? 1 : -1;
    sim_scalar_t z = (v1 & 4) ? 1 : -1;
    globalcoord(res + 0, mat, NULL, x, 0, 0);
    globalcoord(res + 3, mat, NULL, 0, y, 0);
    globalcoord(res + 6, mat, NULL, 0, 0, z);
    resind[0] = (x > 0) ? 0 : 1;
    resind[1] = (y > 0) ? 2 : 3;
    resind[2] = (z > 0) ? 4 : 5;
    return 3;
  }
  return 0;
}


// compute possible edge normals for box for edge collisions
static int boxEdgeNormals(sim_scalar_t res[9], sim_scalar_t endverts[9], int dim, sim_collision_ccd_object_t* obj,
                          const sim_scalar_t v1[3], const sim_scalar_t v2[3], int v1i, int v2i) {
  // box data
  int g = 3*obj->geom;
  const sim_scalar_t* mat = obj->data->geom_xmat + 3*g;
  const sim_scalar_t* pos = obj->data->geom_xpos + g;
  const sim_scalar_t* size = obj->model->geom_size + g;

  if (dim == 2) {
    copy3(endverts, v2);
    sub3(res, v2, v1);
    sim_math_normalize_3(res);
    return 1;
  }

  // return 3 adjacent vertices
  if (dim == 1) {
    sim_scalar_t x = (v1i & 1) ? size[0] : -size[0];
    sim_scalar_t y = (v1i & 2) ? size[1] : -size[1];
    sim_scalar_t z = (v1i & 4) ? size[2] : -size[2];

    globalcoord(endverts, mat, pos, -x, y, z);
    sub3(res, endverts, v1);
    sim_math_normalize_3(res);

    globalcoord(endverts + 3, mat, pos, x, -y, z);
    sub3(res + 3, endverts + 3, v1);
    sim_math_normalize_3(res + 3);

    globalcoord(endverts + 6, mat, pos, x, y, -z);
    sub3(res + 6, endverts + 6, v1);
    sim_math_normalize_3(res + 6);
    return 3;
  }
  return 0;
}


// recover face of a box from its index
static int boxFace(sim_scalar_t res[12], sim_collision_ccd_object_t* obj, int idx) {
  // box data
  int g = 3*obj->geom;
  const sim_scalar_t* mat = obj->data->geom_xmat + 3*g;
  const sim_scalar_t* pos = obj->data->geom_xpos + g;
  const sim_scalar_t* size = obj->model->geom_size + g;

  // compute global coordinates of the box face and face normal
  switch (idx) {
    case 0:  // right
      globalcoord(res + 0, mat, pos,  size[0],  size[1],  size[2]);
      globalcoord(res + 3, mat, pos,  size[0],  size[1], -size[2]);
      globalcoord(res + 6, mat, pos,  size[0], -size[1], -size[2]);
      globalcoord(res + 9, mat, pos,  size[0], -size[1],  size[2]);
      return 4;
     case 1:  // left
      globalcoord(res + 0, mat, pos, -size[0],  size[1], -size[2]);
      globalcoord(res + 3, mat, pos, -size[0],  size[1],  size[2]);
      globalcoord(res + 6, mat, pos, -size[0], -size[1],  size[2]);
      globalcoord(res + 9, mat, pos, -size[0], -size[1], -size[2]);
      return 4;
    case 2:  // top
      globalcoord(res + 0, mat, pos, -size[0],  size[1], -size[2]);
      globalcoord(res + 3, mat, pos,  size[0],  size[1], -size[2]);
      globalcoord(res + 6, mat, pos,  size[0],  size[1],  size[2]);
      globalcoord(res + 9, mat, pos, -size[0],  size[1],  size[2]);
      return 4;
    case 3:  // bottom
      globalcoord(res + 0, mat, pos, -size[0], -size[1],  size[2]);
      globalcoord(res + 3, mat, pos,  size[0], -size[1],  size[2]);
      globalcoord(res + 6, mat, pos,  size[0], -size[1], -size[2]);
      globalcoord(res + 9, mat, pos, -size[0], -size[1], -size[2]);
      return 4;
    case 4:  // front
      globalcoord(res + 0, mat, pos, -size[0],  size[1],  size[2]);
      globalcoord(res + 3, mat, pos,  size[0],  size[1],  size[2]);
      globalcoord(res + 6, mat, pos,  size[0], -size[1],  size[2]);
      globalcoord(res + 9, mat, pos, -size[0], -size[1],  size[2]);
      return 4;
    case 5:  // back
      globalcoord(res + 0, mat, pos,  size[0],  size[1], -size[2]);
      globalcoord(res + 3, mat, pos, -size[0],  size[1], -size[2]);
      globalcoord(res + 6, mat, pos, -size[0], -size[1], -size[2]);
      globalcoord(res + 9, mat, pos,  size[0], -size[1], -size[2]);
      return 4;
  }
  return 0;
}


// recover mesh polygon from its index, return number of edges
static int meshFace(sim_scalar_t* res, sim_collision_ccd_object_t* obj, int idx) {
  const sim_model_t* m = obj->model;

  // mesh data
  int g = 3*obj->geom;
  const sim_scalar_t* mat = obj->data->geom_xmat + 3*g;
  const sim_scalar_t* pos = obj->data->geom_xpos + g;
  int polyadr = m->mesh_polyadr[m->geom_dataid[obj->geom]];
  int vertadr = m->mesh_vertadr[m->geom_dataid[obj->geom]];
  const float* vert = m->mesh_vert + 3*vertadr;
  const int* polyvert = m->mesh_polyvert;

  int adr = m->mesh_polyvertadr[polyadr + idx], j = 0;
  int nvert =  m->mesh_polyvertnum[polyadr + idx];
  if (nvert > SIM_MAX_POLYVERT) nvert = SIM_MAX_POLYVERT;
  for (int i = nvert - 1; i >= 0; i--) {
    const float* v = vert + 3*polyvert[adr + i];
    globalcoord(res + 3*j++, mat, pos, v[0], v[1], v[2]);
  }
  return nvert;
}


// find two normals that are facing each other within a tolerance, return 1 if found
static inline int alignedFaces(int res[2], const sim_scalar_t* v, int nv,
                                 const sim_scalar_t* w, int nw) {
  for (int i = 0; i < nv; i++) {
    for (int j = 0; j < nw; j++) {
      if (dot3(v + 3*i, w + 3*j) < -SIM_FACE_TOL) {
        res[0] = i;
        res[1] = j;
        return 1;
      }
    }
  }
  return 0;
}


// find two normals that are perpendicular to each other within a tolerance, return 1 if found
static inline int alignedFaceEdge(int res[2], const sim_scalar_t* edge, int nedge,
                                  const sim_scalar_t* face, int nface) {
  for (int i = 0; i < nface; i++) {
    for (int j = 0; j < nedge; j++) {
      if (sim_math_abs(dot3(edge + 3*j, face + 3*i)) < SIM_EDGE_TOL) {
        res[0] = j;
        res[1] = i;
        return 1;
      }
    }
  }
  return 0;
}


// return number (1, 2 or 3) of dimensions of a simplex; reorder vertices if necessary
static inline int simplexDim(int* v1i, int* v2i, int* v3i, sim_scalar_t** v1, sim_scalar_t** v2, sim_scalar_t** v3) {
  int val1 = *v1i;
  int val2 = *v2i;
  int val3 = *v3i;

  if (val1 != val2) {
    return (val3 == val1 || val3 == val2) ? 2 : 3;
  }
  if (val1 != val3) {
    *v2i = *v3i;
    *v2 = *v3;
    return 2;
  }
  return 1;
}


// recover multiple contacts from EPA polytope
static void multicontact(Polytope* pt, Face* face, SIM_CCDStatus* status,
                         sim_collision_ccd_object_t* obj1, sim_collision_ccd_object_t* obj2) {
  sim_scalar_t face1[SIM_MAX_POLYVERT * 3], face2[SIM_MAX_POLYVERT * 3], endverts[SIM_MAX_POLYVERT * 3];
  // get vertices of faces from EPA
  int v11i = pt->verts[face->verts[0]].index1;
  int v12i = pt->verts[face->verts[1]].index1;
  int v13i = pt->verts[face->verts[2]].index1;
  int v21i = pt->verts[face->verts[0]].index2;
  int v22i = pt->verts[face->verts[1]].index2;
  int v23i = pt->verts[face->verts[2]].index2;
  sim_scalar_t* v11 = pt->verts[face->verts[0]].vert1;
  sim_scalar_t* v12 = pt->verts[face->verts[1]].vert1;
  sim_scalar_t* v13 = pt->verts[face->verts[2]].vert1;
  sim_scalar_t* v21 = pt->verts[face->verts[0]].vert2;
  sim_scalar_t* v22 = pt->verts[face->verts[1]].vert2;
  sim_scalar_t* v23 = pt->verts[face->verts[2]].vert2;

  // get dimensions of features of geoms 1 and 2
  int nface1 = simplexDim(&v11i, &v12i, &v13i, &v11, &v12, &v13);
  int nface2 = simplexDim(&v21i, &v22i, &v23i, &v21, &v22, &v23);
  int nnorms1 = 0, nnorms2 = 0;
  sim_scalar_t n1[3 * SIM_MAX_POLYVERT], n2[3 * SIM_MAX_POLYVERT];  // normals of possible face collisions
  int idx1[SIM_MAX_POLYVERT], idx2[SIM_MAX_POLYVERT];         // indices of faces

  sim_scalar_t dir[3], dir_neg[3];
  sub3(dir, status->x2, status->x1);
  sub3(dir_neg, status->x1, status->x2);

  // get all possible face normals for each geom
  if (obj1->geom_type == SIM_GEOM_BOX) {
    nnorms1 = boxNormals(n1, idx1, nface1, obj1, v11i, v12i, v13i, dir_neg);
  } else if (obj1->geom_type == SIM_GEOM_MESH) {
    nnorms1 = meshNormals(n1, idx1, nface1, obj1, v11i, v12i, v13i);
  }
  if (obj2->geom_type == SIM_GEOM_BOX) {
    nnorms2 = boxNormals(n2, idx2, nface2, obj2, v21i, v22i, v23i, dir);
  } else if (obj2->geom_type == SIM_GEOM_MESH) {
    nnorms2 = meshNormals(n2, idx2, nface2, obj2, v21i, v22i, v23i);
  }

  // determine if any two face normals match
  int res[2], edgecon1 = 0, edgecon2 = 0;
  if (!alignedFaces(res, n1, nnorms1, n2, nnorms2)) {
    // check if edge-face collision
    if (nface1 < 3 && nface1 <= nface2) {
      nnorms1 = 0;
      if (obj1->geom_type == SIM_GEOM_BOX) {
        nnorms1 = boxEdgeNormals(n1, endverts, nface1, obj1, v11, v12, v11i, v12i);
      } else if (obj1->geom_type == SIM_GEOM_MESH) {
        nnorms1 = meshEdgeNormals(n1, endverts, nface1, obj1, v11, v12, v11i, v12i);
      }
      if (!alignedFaceEdge(res, n1, nnorms1, n2, nnorms2)) return;
      edgecon1 = 1;

    // check if face-edge collision
    } else if (nface2 < 3) {
      nnorms2 = 0;
      if (obj2->geom_type == SIM_GEOM_BOX) {
        nnorms2 = boxEdgeNormals(n2, endverts, nface2, obj2, v21, v22, v21i, v22i);
      } else if (obj2->geom_type == SIM_GEOM_MESH) {
        nnorms2 = meshEdgeNormals(n2, endverts, nface2, obj2, v21, v22, v21i, v22i);
      }
      if (!alignedFaceEdge(res, n2, nnorms2, n1, nnorms1)) return;
      edgecon2 = 1;
    } else {
      // no multi-contact
      return;
    }
  }
  int i = res[0], j = res[1];

  // recover geom1 matching edge or face
  if (edgecon1) {
    copy3(face1, pt->verts[face->verts[0]].vert1);
    copy3(face1 + 3, endverts + 3*i);
    nface1 = 2;
  } else {
    if (obj1->geom_type == SIM_GEOM_BOX) {
      int ind = (edgecon2 ? idx1[j] : idx1[i]);
      nface1 = boxFace(face1, obj1, ind);
    } else if (obj1->geom_type == SIM_GEOM_MESH) {
      int ind = (edgecon2 ? idx1[j] : idx1[i]);
      nface1 = meshFace(face1, obj1, ind);
    }
  }

  // recover geom2 matching edge or face
  if (edgecon2) {
    copy3(face2, pt->verts[face->verts[0]].vert2);
    copy3(face2 + 3, endverts + 3*i);
    nface2 = 2;
  } else {
    if (obj2->geom_type == SIM_GEOM_BOX) {
      nface2 = boxFace(face2, obj2, idx2[j]);
    } else if (obj2->geom_type == SIM_GEOM_MESH) {
      nface2 = meshFace(face2, obj2, idx2[j]);
    }
  }

  // TODO(kylebayes): this approximates the contact direction, by scaling the face normal by the
  // single contact direction's magnitude. This is effective, but polygonClip should compute
  // this for each contact point.
  sim_scalar_t approx_dir[3];

  // face1 is an edge; clip face1 against face2
  if (edgecon1) {
    scl3(approx_dir, n2 + 3*j, norm3(dir));
    polygonClip(status, face2, nface2, face1, nface1, n2 + 3*j, approx_dir);
    return;
  }

  // face2 is an edge; clip face2 against face1
  if (edgecon2) {
    scl3(approx_dir, n1 + 3*j, -norm3(dir));
    polygonClip(status, face1, nface1, face2, nface2, n1 + 3*j, approx_dir);
    return;
  }

  // face-face collision
  scl3(approx_dir, n2 + 3*j, norm3(dir));
  polygonClip(status, face1, nface1, face2, nface2, n1 + 3*i, approx_dir);
}



// inflate a contact by margin
static inline void inflate(SIM_CCDStatus* status, sim_scalar_t margin1, sim_scalar_t margin2) {
  sim_scalar_t n[3];
  sub3(n, status->x2, status->x1);
  sim_math_normalize_3(n);
  if (margin1) {
    status->x1[0] += margin1 * n[0];
    status->x1[1] += margin1 * n[1];
    status->x1[2] += margin1 * n[2];
  }
  if (margin2) {
    status->x2[0] -= margin2 * n[0];
    status->x2[1] -= margin2 * n[1];
    status->x2[2] -= margin2 * n[2];
  }
  status->dist -= (margin1 + margin2);
}


// general convex collision detection
sim_scalar_t SIM_c_ccd(const SIM_CCDConfig* config, SIM_CCDStatus* status, sim_collision_ccd_object_t* obj1, sim_collision_ccd_object_t* obj2) {
  // pre-allocate static memory for low iterations
  void* buffer = NULL;
  static SIM_THREADLOCAL Vertex vert_data[5 + SIM_MAX_EPA_ITERATIONS];
  static SIM_THREADLOCAL Face face_data[6 * SIM_MAX_EPA_ITERATIONS];
  static SIM_THREADLOCAL Face* map_data[6 * SIM_MAX_EPA_ITERATIONS];
  static SIM_THREADLOCAL int index_data[6 + SIM_MAX_EPA_ITERATIONS];
  static SIM_THREADLOCAL int edge_data[6 + SIM_MAX_EPA_ITERATIONS];

  // setup
  obj1->center(status->x1, obj1);
  obj2->center(status->x2, obj2);
  status->gjk_iterations = 0;
  status->epa_iterations = 0;
  status->epa_status = SIM_EPA_NOCONTACT;
  status->tolerance = config->tolerance;
  status->max_iterations = config->max_iterations;
  status->max_contacts = config->max_contacts;
  status->dist_cutoff = config->dist_cutoff;

  // special handling for sphere and capsule (shrink to point and line respectively)
  if (obj1->geom_type == SIM_GEOM_SPHERE || obj2->geom_type == SIM_GEOM_SPHERE ||
      obj1->geom_type == SIM_GEOM_CAPSULE || obj2->geom_type == SIM_GEOM_CAPSULE) {
    void (*support1)(sim_scalar_t*, struct _mjCCDObj*, const sim_scalar_t*) = obj1->support;
    void (*support2)(sim_scalar_t*, struct _mjCCDObj*, const sim_scalar_t*) = obj2->support;
    sim_scalar_t full_margin1 = 0, full_margin2 = 0;
    sim_scalar_t margin1 = obj1->margin, margin2 = obj2->margin;

    if (obj1->geom_type == SIM_GEOM_SPHERE) {
      const sim_model_t* m = obj1->model;
      full_margin1 = m->geom_size[3*obj1->geom] + 0.5*margin1;
      obj1->support = SIM_c_pointSupport;
      obj1->margin = 0;
    } else if (obj1->geom_type == SIM_GEOM_CAPSULE) {
      const sim_model_t* m = obj1->model;
      full_margin1 = m->geom_size[3*obj1->geom] + 0.5*margin1;
      obj1->support = SIM_c_lineSupport;
      obj1->margin = 0;
    }

    if (obj2->geom_type == SIM_GEOM_SPHERE) {
      const sim_model_t* m = obj2->model;
      full_margin2 = m->geom_size[3*obj2->geom] + 0.5*margin2;
      obj2->support = SIM_c_pointSupport;
      obj2->margin = 0;
    } else if (obj2->geom_type == SIM_GEOM_CAPSULE) {
      const sim_model_t* m = obj2->model;
      full_margin2 = m->geom_size[3*obj2->geom] + 0.5*margin2;
      obj2->support = SIM_c_lineSupport;
      obj2->margin = 0;
    }

    status->dist_cutoff += full_margin1 + full_margin2;
    gjk(status, obj1, obj2);
    status->dist_cutoff = config->dist_cutoff;

    // restore original margin and support
    obj1->margin = margin1;
    obj2->margin = margin2;
    obj1->support = support1;
    obj2->support = support2;

    // shallow penetration, inflate contact
    if (status->dist > status->tolerance) {
      inflate(status, full_margin1, full_margin2);
      if (status->dist > status->dist_cutoff) {
        status->dist = SIM_MAX_LIMIT;
      }
      return status->dist;
    }

    // contact not needed
    if (!config->max_contacts) {
      status->nx = 0;
      status->dist = 0;
      return 0;
    }

    // deep penetration, reset initial conditions and rerun GJK + EPA
    status->gjk_iterations = 0;
    obj1->center(status->x1, obj1);
    obj2->center(status->x2, obj2);
  }
  gjk(status, obj1, obj2);

  // penetration recovery for contacts not needed
  if (!config->max_contacts) {
    return status->dist;
  }

  if (status->dist <= config->tolerance && status->nsimplex > 1) {
    status->dist = 0;  // assume touching
    Polytope pt;
    pt.nfaces = pt.nmap = pt.nverts = pt.horizon.nedges = 0;

    // allocate memory via static thread-local storage
    int N = config->max_iterations;
    if (N <= SIM_MAX_EPA_ITERATIONS) {
      pt.maxfaces = 6 * SIM_MAX_EPA_ITERATIONS;
      pt.verts = vert_data;
      pt.faces = face_data;
      pt.map = map_data;
      pt.horizon.indices = index_data;
      pt.horizon.edges = edge_data;
    }

    // static storage insufficient, allocate with callback
    else {
      size_t nbytes = (sizeof(Face) * 6 * N)      // faces in polytope
                    + (sizeof(Face*) * 6 * N)     // map in polytope
                    + (sizeof(Vertex) * (5 + N))  // vertices in polytope
                    + 2*(sizeof(int) * (6 + N));  // horizon data

      pt.maxfaces = 6 * N;
      buffer = config->alloc(config->context, nbytes);
      uint8_t* bbuffer = (uint8_t*)buffer;
      pt.verts = (Vertex*)bbuffer;
      bbuffer += sizeof(Vertex) * (5 + N);
      pt.faces = (Face*)bbuffer;
      bbuffer += sizeof(Face) * (6 * N);
      pt.map = (Face**)bbuffer;
      bbuffer += sizeof(Face*) * (6 * N);
      pt.horizon.indices = (int*)bbuffer;
      bbuffer += sizeof(int) * (6 + N);
      pt.horizon.edges = (int*)bbuffer;
    }

    int ret;
    if (status->nsimplex == 2) {
      ret = polytope2(&pt, status, obj1, obj2);
    } else if (status->nsimplex == 3) {
      ret = polytope3(&pt, status, obj1, obj2);
    } else {
      ret = polytope4(&pt, status, obj1, obj2);
    }
    status->epa_status = ret;

    // simplex not on boundary (objects are penetrating)
    if (!ret) {
      Face* face = epa(status, &pt, obj1, obj2);
      if (config->max_contacts > 1 && face) {
        multicontact(&pt, face, status, obj1, obj2);
      }
    }
  }
  if (buffer) {
    config->free(config->context, buffer);
  }
  return status->dist;
}
