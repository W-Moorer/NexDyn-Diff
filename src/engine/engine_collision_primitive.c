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

#include "engine/engine_collision_primitive.h"

#include <simcore/SIM_data.h>
#include <simcore/SIM_model.h>
#include "engine/engine_inline.h"
#include "engine/engine_util_blas.h"
#include "engine/engine_util_misc.h"
#include "engine/engine_util_spatial.h"


//--------------------------- plane collisions -----------------------------------------------------

// raw plane : sphere
static int SIM_raw_PlaneSphere(sim_contact_t* con, sim_scalar_t margin,
                             const sim_scalar_t* pos1, const sim_scalar_t* mat1, const sim_scalar_t* size1,
                             const sim_scalar_t* pos2, const sim_scalar_t* mat2, const sim_scalar_t* size2) {
  // set normal
  con[0].frame[0] = mat1[2];
  con[0].frame[1] = mat1[5];
  con[0].frame[2] = mat1[8];

  // compute distance, return if too large
  sim_scalar_t tmp[3] = {pos2[0] - pos1[0], pos2[1] - pos1[1], pos2[2] - pos1[2]};
  sim_scalar_t cdist = sim_math_dot_3(tmp, con[0].frame);
  if (cdist > margin + size2[0]) {
    return 0;
  }

  // depth and position
  con[0].dist = cdist - size2[0];
  sim_math_internal_scl3(tmp, con[0].frame, -con[0].dist / 2 - size2[0]);
  sim_math_internal_add3(con[0].pos, pos2, tmp);

  sim_math_zero_3(con[0].frame+3);
  return 1;
}


// plane : sphere
int SIM_c_PlaneSphere(const sim_model_t* m, const sim_data_t* d,
                    sim_contact_t* con, int g1, int g2, sim_scalar_t margin) {
  SIM_GETINFO
  return SIM_raw_PlaneSphere(con, margin, pos1, mat1, size1, pos2, mat2, size2);
}


// plane : capsule
int SIM_c_PlaneCapsule(const sim_model_t* m, const sim_data_t* d,
                     sim_contact_t* con, int g1, int g2, sim_scalar_t margin) {
  SIM_GETINFO

  // get capsule axis, segment = scaled axis
  sim_scalar_t axis[3] = {mat2[2], mat2[5], mat2[8]};
  sim_scalar_t segment[3] = {size2[1]*axis[0], size2[1]*axis[1], size2[1]*axis[2]};

  // get point 1, do sphere-plane test
  sim_scalar_t pos[3];
  sim_math_add3(pos, pos2, segment);
  int n1 = SIM_raw_PlaneSphere(con, margin, pos1, mat1, size1, pos, mat2, size2);

  // get point 2, do sphere-plane test
  sim_math_sub_3(pos, pos2, segment);
  int n2 = SIM_raw_PlaneSphere(con+n1, margin, pos1, mat1, size1, pos, mat2, size2);

  // align contact frames with capsule axis
  if (n1) {
    sim_math_internal_copy_3(con->frame + 3, axis);
  }
  if (n2) {
    sim_math_internal_copy_3((con + n1)->frame + 3, axis);
  }

  return n1+n2;
}


// plane : cylinder
int SIM_c_PlaneCylinder(const sim_model_t* m, const sim_data_t* d,
                      sim_contact_t* con, int g1, int g2, sim_scalar_t margin) {
  SIM_GETINFO
  sim_scalar_t normal[3] = {mat1[2], mat1[5], mat1[8]};
  sim_scalar_t axis[3] = {mat2[2], mat2[5], mat2[8]};

  // project, make sure axis points towards plane
  sim_scalar_t prjaxis = sim_math_dot_3(normal, axis);
  if (prjaxis > 0) {
    sim_math_scale_3(axis, axis, -1);
    prjaxis = -prjaxis;
  }

  // compute normal distance to cylinder center
  sim_scalar_t vec[3] = {pos2[0] - pos1[0], pos2[1] - pos1[1], pos2[2] - pos1[2]};
  sim_scalar_t dist0 = sim_math_dot_3(vec, normal);

  // remove component of -normal along axis, compute length
  sim_math_scale_3(vec, axis, prjaxis);
  sim_math_subFrom3(vec, normal);
  sim_scalar_t len_sqr = sim_math_dot_3(vec, vec);

  // general configuration: normalize vector, scale by radius
  if (len_sqr >= SIM_MINVAL*SIM_MINVAL) {
    sim_scalar_t scl = size2[0]/sim_math_sqrt(len_sqr);
    vec[0] *= scl;
    vec[1] *= scl;
    vec[2] *= scl;
  }

  // disk parallel to plane: pick x-axis of cylinder, scale by radius
  else {
    vec[0] = mat2[0]*size2[0];
    vec[1] = mat2[3]*size2[0];
    vec[2] = mat2[6]*size2[0];
  }

  // project vector on normal
  sim_scalar_t prjvec = sim_math_dot_3(vec, normal);

  // scale axis by half-length
  sim_math_scale_3(axis, axis, size2[1]);
  prjaxis *= size2[1];

  // check first point, construct contact
  int cnt = 0;
  if (dist0 + prjaxis + prjvec <= margin) {
    con[cnt].dist = dist0 + prjaxis + prjvec;
    sim_math_internal_add3(con[cnt].pos, pos2, vec);
    sim_math_internal_addTo3(con[cnt].pos, axis);
    sim_math_internal_addToScl3(con[cnt].pos, normal, -con[cnt].dist * 0.5);
    sim_math_internal_copy_3(con[cnt].frame, normal);
    sim_math_zero_3(con[cnt].frame+3);
    cnt++;
  } else {
    return 0;  // nearest point is above margin: no contacts
  }

  // check second point, construct contact
  if (dist0 - prjaxis + prjvec <= margin) {
    con[cnt].dist = dist0 - prjaxis + prjvec;
    sim_math_internal_add3(con[cnt].pos, pos2, vec);
    sim_math_internal_subFrom3(con[cnt].pos, axis);
    sim_math_internal_addToScl3(con[cnt].pos, normal, -con[cnt].dist * 0.5);
    sim_math_internal_copy_3(con[cnt].frame, normal);
    sim_math_zero_3(con[cnt].frame+3);
    cnt++;
  }

  // try to add triangle points on side closer to plane
  sim_scalar_t prjvec1 = -prjvec*0.5;
  if (dist0 + prjaxis + prjvec1 <= margin) {
    // compute sideways vector: vec1
    sim_scalar_t vec1[3];
    sim_math_internal_cross(vec1, vec, axis);
    sim_math_normalize_3(vec1);
    sim_math_scale_3(vec1, vec1, size2[0] * sim_math_sqrt(3.0) / 2);

    // add point A
    con[cnt].dist = dist0 + prjaxis + prjvec1;
    sim_math_internal_add3(con[cnt].pos, pos2, vec1);
    sim_math_internal_addTo3(con[cnt].pos, axis);
    sim_math_internal_addToScl3(con[cnt].pos, vec, -0.5);
    sim_math_internal_addToScl3(con[cnt].pos, normal, -con[cnt].dist * 0.5);
    sim_math_internal_copy_3(con[cnt].frame, normal);
    sim_math_zero_3(con[cnt].frame+3);
    cnt++;

    // add point B
    con[cnt].dist = dist0 + prjaxis + prjvec1;
    sim_math_internal_sub3(con[cnt].pos, pos2, vec1);
    sim_math_internal_addTo3(con[cnt].pos, axis);
    sim_math_internal_addToScl3(con[cnt].pos, vec, -0.5);
    sim_math_internal_addToScl3(con[cnt].pos, normal, -con[cnt].dist * 0.5);
    sim_math_internal_copy_3(con[cnt].frame, normal);
    sim_math_zero_3(con[cnt].frame+3);
    cnt++;
  }

  return cnt;
}


// plane : box
int SIM_c_PlaneBox(const sim_model_t* m, const sim_data_t* d,
                 sim_contact_t* con, int g1, int g2, sim_scalar_t margin) {
  SIM_GETINFO

  // get normal, difference between centers, normal distance
  sim_scalar_t norm[3] = {mat1[2], mat1[5], mat1[8]};
  sim_scalar_t dif[3] = {pos2[0] - pos1[0], pos2[1] - pos1[1], pos2[2] - pos1[2]};
  sim_scalar_t dist = sim_math_dot_3(dif, norm);

  // test all corners, pick bottom 4
  int cnt = 0;
  for (int i=0; i < 8; i++) {
    // get corner in local coordinates
    sim_scalar_t vec[3];
    vec[0] = (i&1 ? size2[0] : -size2[0]);
    vec[1] = (i&2 ? size2[1] : -size2[1]);
    vec[2] = (i&4 ? size2[2] : -size2[2]);

    // get corner in global coordinates relative to box center
    sim_scalar_t corner[3];
    sim_math_mul_mat_vec_3(corner, mat2, vec);

    // compute distance to plane, skip if too far or pointing up
    sim_scalar_t ldist = sim_math_dot_3(norm, corner);
    if (dist + ldist > margin || ldist > 0) {
      continue;
    }

    // construct contact
    con[cnt].dist = dist + ldist;
    sim_math_internal_copy_3(con[cnt].frame, norm);
    sim_math_zero_3(con[cnt].frame+3);
    sim_math_internal_addTo3(corner, pos2);
    sim_math_internal_scl3(vec, norm, -con[cnt].dist / 2);
    sim_math_internal_add3(con[cnt].pos, corner, vec);

    // count; max is 4
    if (++cnt >= 4) {
      return 4;
    }
  }

  return cnt;
}


//--------------------------- sphere and capsule collisions ----------------------------------------

// sphere : sphere (actual implementation, can be called with modified parameters)
static int SIM_raw_SphereSphere(sim_contact_t* con, sim_scalar_t margin,
                              const sim_scalar_t* pos1, const sim_scalar_t* mat1, const sim_scalar_t* size1,
                              const sim_scalar_t* pos2, const sim_scalar_t* mat2, const sim_scalar_t* size2) {
  // check bounding spheres (this is called from other functions)
  sim_scalar_t dif[3] = {pos1[0] - pos2[0], pos1[1] - pos2[1], pos1[2] - pos2[2]};
  sim_scalar_t cdist_sqr = sim_math_dot_3(dif, dif);
  sim_scalar_t min_dist = margin + size1[0] + size2[0];
  if (cdist_sqr > min_dist*min_dist) {
    return 0;
  }

  // depth and normal
  con[0].dist = sim_math_sqrt(cdist_sqr) - size1[0] - size2[0];
  sim_math_internal_sub3(con[0].frame, pos2, pos1);
  sim_scalar_t len = sim_math_normalize_3(con[0].frame);

  // if centers are the same, norm = cross-product of z axes
  //  if z axes are parallel, norm = [1;0;0]
  if (len < SIM_MINVAL) {
    sim_scalar_t axis1[3] = {mat1[2], mat1[5], mat1[8]};
    sim_scalar_t axis2[3] = {mat2[2], mat2[5], mat2[8]};
    sim_math_internal_cross(con[0].frame, axis1, axis2);
    sim_math_normalize_3(con[0].frame);
  }

  // position
  sim_math_internal_scl3(con[0].pos, con[0].frame, size1[0] + con[0].dist / 2);
  sim_math_internal_addTo3(con[0].pos, pos1);

  sim_math_zero_3(con[0].frame+3);
  return 1;
}


// sphere : sphere
int SIM_c_SphereSphere(const sim_model_t* m, const sim_data_t* d,
                     sim_contact_t* con, int g1, int g2, sim_scalar_t margin) {
  SIM_GETINFO
  return SIM_raw_SphereSphere(con, margin, pos1, mat1, size1, pos2, mat2, size2);
}


// raw sphere : capsule
int SIM_raw_SphereCapsule(sim_contact_t* con, sim_scalar_t margin,
                        const sim_scalar_t* pos1, const sim_scalar_t* mat1, const sim_scalar_t* size1,
                        const sim_scalar_t* pos2, const sim_scalar_t* mat2, const sim_scalar_t* size2) {
  // get capsule length and axis
  sim_scalar_t len = size2[1];
  sim_scalar_t axis[3] = {mat2[2], mat2[5], mat2[8]};

  // find projection, clip to segment
  sim_scalar_t vec[3] = {pos1[0] - pos2[0], pos1[1] - pos2[1], pos1[2] - pos2[2]};
  sim_scalar_t x = sim_math_clip(sim_math_dot_3(axis, vec), -len, len);

  // find nearest point on segment, do sphere-sphere test
  sim_math_internal_scl3(vec, axis, x);
  sim_math_internal_addTo3(vec, pos2);
  return SIM_raw_SphereSphere(con, margin, pos1, mat1, size1, vec, mat2, size2);
}


// sphere : capsule
int SIM_c_SphereCapsule(const sim_model_t* m, const sim_data_t* d,
                      sim_contact_t* con, int g1, int g2, sim_scalar_t margin) {
  SIM_GETINFO
  return SIM_raw_SphereCapsule(con, margin, pos1, mat1, size1, pos2, mat2, size2);
}


// sphere : cylinder
int SIM_c_SphereCylinder(const sim_model_t* m, const sim_data_t* d,
                       sim_contact_t* con, int g1, int g2, sim_scalar_t margin) {
  SIM_GETINFO

  // get cylinder sizes and axis
  sim_scalar_t radius = size2[0];
  sim_scalar_t height = size2[1];
  sim_scalar_t axis[3] = {mat2[2], mat2[5], mat2[8]};

  // find sphere projection onto cylinder axis and plane
  sim_scalar_t vec[3] = {pos1[0] - pos2[0], pos1[1] - pos2[1], pos1[2] - pos2[2]};
  sim_scalar_t x = sim_math_dot_3(axis, vec);
  sim_scalar_t a_proj[3], p_proj[3];
  sim_math_internal_scl3(a_proj, axis, x);
  sim_math_internal_sub3(p_proj, vec, a_proj);
  sim_scalar_t p_proj_sqr = sim_math_dot_3(p_proj, p_proj);

  // get collision type
  int collide_side = sim_math_abs(x) < height;
  int collide_cap = p_proj_sqr < radius*radius;
  if (collide_side && collide_cap) {  // deep penetration (sphere origin inside cylinder)
    sim_scalar_t dist_cap = height - sim_math_abs(x);
    sim_scalar_t dist_radius = radius - sim_math_sqrt(p_proj_sqr);
    if (dist_cap < dist_radius) {  // disable one collision type
      collide_side = 0;
    } else {
      collide_cap = 0;
    }
  }

  // side collision: use sphere-sphere
  if (collide_side) {
    sim_math_internal_addTo3(a_proj, pos2);
    return SIM_raw_SphereSphere(con, margin, pos1, mat1, size1, a_proj, mat2, size2);
  }

  // cap collision: use plane-sphere
  if (collide_cap) {
    const sim_scalar_t flipmat[9] = {
      -mat2[0], mat2[1], -mat2[2],
      -mat2[3], mat2[4], -mat2[5],
      -mat2[6], mat2[7], -mat2[8]
    };
    const sim_scalar_t* mat_cap;
    sim_scalar_t pos_cap[3];
    if (x > 0) {  // top cap
      sim_math_addScl3(pos_cap, pos2, axis, height);
      mat_cap = mat2;
    } else {      // bottom cap
      sim_math_addScl3(pos_cap, pos2, axis, -height);
      mat_cap = flipmat;
    }
    int ncon = SIM_raw_PlaneSphere(con, margin, pos_cap, mat_cap, size2, pos1, mat1, size1);
    if (ncon) {
      // flip frame normal (because SIM_GEOM_PLANE < SIM_GEOM_SPHERE < SIM_GEOM_CYLINDER)
      sim_math_scale_3(con->frame, con->frame, -1);
    }
    return ncon;
  }

  // otherwise corner collision: use sphere-sphere
  sim_math_scale_3(p_proj, p_proj,
           size2[0] / sim_math_sqrt(p_proj_sqr));  // denominator cannot be 0
  sim_math_internal_scl3(vec, axis, x > 0 ? height : -height);
  sim_math_internal_addTo3(vec, p_proj);
  sim_math_internal_addTo3(vec, pos2);

  // sphere-sphere with point sphere at the corner
  sim_scalar_t size_zero[1] = {0};
  return SIM_raw_SphereSphere(con, margin, pos1, mat1, size1, vec, mat2, size_zero);
}


// raw capsule : capsule
int SIM_raw_CapsuleCapsule(sim_contact_t* con, sim_scalar_t margin,
                         const sim_scalar_t* pos1, const sim_scalar_t* mat1, const sim_scalar_t* size1,
                         const sim_scalar_t* pos2, const sim_scalar_t* mat2, const sim_scalar_t* size2) {
  // get capsule axes (scaled) and center difference
  sim_scalar_t axis1[3] = {mat1[2] * size1[1], mat1[5] * size1[1], mat1[8] * size1[1]};
  sim_scalar_t axis2[3] = {mat2[2] * size2[1], mat2[5] * size2[1], mat2[8] * size2[1]};
  sim_scalar_t dif[3] = {pos1[0] - pos2[0], pos1[1] - pos2[1], pos1[2] - pos2[2]};

  // compute matrix coefficients and determinant
  sim_scalar_t ma =  sim_math_dot_3(axis1, axis1);
  sim_scalar_t mb = -sim_math_dot_3(axis1, axis2);
  sim_scalar_t mc =  sim_math_dot_3(axis2, axis2);
  sim_scalar_t u  = -sim_math_dot_3(axis1, dif);
  sim_scalar_t v  =  sim_math_dot_3(axis2, dif);
  sim_scalar_t det = ma*mc - mb*mb;

  // general configuration (non-parallel axes)
  if (sim_math_abs(det) >= SIM_MINVAL) {
    // find projections, clip to segments
    sim_scalar_t x1 = (mc*u - mb*v) / det;
    sim_scalar_t x2 = (ma*v - mb*u) / det;

    if (x1 > 1) {
      x1 = 1;
      x2 = (v - mb) / mc;
    } else if (x1 < -1) {
      x1 = -1;
      x2 = (v + mb) / mc;
    }
    if (x2 > 1) {
      x2 = 1;
      x1 = sim_math_clip((u - mb) / ma, -1, 1);
    } else if (x2 < -1) {
      x2 = -1;
      x1 = sim_math_clip((u + mb) / ma, -1, 1);
    }

    // find nearest points, do sphere-sphere test
    sim_scalar_t vec1[3], vec2[3];
    sim_math_internal_scl3(vec1, axis1, x1);
    sim_math_internal_addTo3(vec1, pos1);
    sim_math_internal_scl3(vec2, axis2, x2);
    sim_math_internal_addTo3(vec2, pos2);

    return SIM_raw_SphereSphere(con, margin, vec1, mat1, size1, vec2, mat2, size2);
  }

  // parallel axes
  else {
    // x1 = 1
    sim_scalar_t vec1[3];
    sim_math_internal_add3(vec1, pos1, axis1);
    sim_scalar_t x2 = sim_math_clip((v - mb) / mc, -1, 1);

    sim_scalar_t vec2[3];
    sim_math_internal_scl3(vec2, axis2, x2);
    sim_math_internal_addTo3(vec2, pos2);
    int n1 = SIM_raw_SphereSphere(con, margin, vec1, mat1, size1, vec2, mat2, size2);

    // x1 = -1
    sim_math_internal_sub3(vec1, pos1, axis1);
    x2 = sim_math_clip((v + mb) / mc, -1, 1);
    sim_math_internal_scl3(vec2, axis2, x2);
    sim_math_internal_addTo3(vec2, pos2);
    int n2 = SIM_raw_SphereSphere(con+n1, margin, vec1, mat1, size1, vec2, mat2, size2);

    // return if two contacts already found
    if (n1+n2 >= 2) {
      return n1+n2;
    }

    // x2 = 1
    sim_math_internal_add3(vec2, pos2, axis2);
    sim_scalar_t x1 = sim_math_clip((u - mb) / ma, -1, 1);
    sim_math_internal_scl3(vec1, axis1, x1);
    sim_math_internal_addTo3(vec1, pos1);
    int n3 = SIM_raw_SphereSphere(con+n1+n2, margin, vec1, mat1, size1, vec2, mat2, size2);

    // return if two contacts already found
    if (n1+n2+n3 >= 2) {
      return n1+n2+n3;
    }

    // x2 = -1
    sim_math_internal_sub3(vec2, pos2, axis2);
    x1 = sim_math_clip((u + mb) / ma, -1, 1);
    sim_math_internal_scl3(vec1, axis1, x1);
    sim_math_internal_addTo3(vec1, pos1);
    int n4 = SIM_raw_SphereSphere(con+n1+n2+n3, margin, vec1, mat1, size1, vec2, mat2, size2);

    return n1+n2+n3+n4;
  }
}


// capsule : capsule
int SIM_c_CapsuleCapsule(const sim_model_t* m, const sim_data_t* d,
                       sim_contact_t* con, int g1, int g2, sim_scalar_t margin) {
  SIM_GETINFO
  return SIM_raw_CapsuleCapsule(con, margin, pos1, mat1, size1, pos2, mat2, size2);
}


// sign of (signed) area of planar triangle
static sim_scalar_t areaSign(const sim_scalar_t p1[2], const sim_scalar_t p2[2], const sim_scalar_t p3[2]) {
  return sim_math_sign((p1[0]-p3[0])*(p2[1]-p3[1]) - (p2[0]-p3[0])*(p1[1]-p3[1]));
}


// find nearest point to p within line segment (u,v); return distance to p
static sim_scalar_t pointSegment(sim_scalar_t res[2], const sim_scalar_t p[2],
                           const sim_scalar_t u[2], const sim_scalar_t v[2]) {
  // make u the origin
  sim_scalar_t uv[2] = {v[0]-u[0], v[1]-u[1]};
  sim_scalar_t up[2] = {p[0]-u[0], p[1]-u[1]};

  // project: find a s.t. uv is orthogonal to (up-a*uv)
  sim_scalar_t a = sim_math_dot(uv, up, 2) / sim_math_max(SIM_MINVAL, sim_math_dot(uv, uv, 2));

  // find nearest point to p, clamp to u or v if a is not in (0,1)
  if (a <= 0) {
    res[0] = u[0];
    res[1] = u[1];
  } else if (a >= 1) {
    res[0] = v[0];
    res[1] = v[1];
  } else {
    sim_math_addScl(res, u, uv, a, 2);
  }

  // compute distance
  return sim_math_sqrt((res[0]-p[0])*(res[0]-p[0]) + (res[1]-p[1])*(res[1]-p[1]));
}


// sphere : triangle with radius
int SIM_raw_SphereTriangle(sim_contact_t* con, sim_scalar_t margin,
                         const sim_scalar_t* s, sim_scalar_t rs,
                         const sim_scalar_t* t1, const sim_scalar_t* t2, const sim_scalar_t* t3, sim_scalar_t rt) {
  sim_scalar_t rbound = margin + rs + rt;
  sim_scalar_t X[3];

  // make t1 the origin: triangle is (O,A,B); sphere center is S
  sim_scalar_t S[3] = { s[0]-t1[0],  s[1]-t1[1],  s[2]-t1[2]};
  sim_scalar_t A[3] = {t2[0]-t1[0], t2[1]-t1[1], t2[2]-t1[2]};
  sim_scalar_t B[3] = {t3[0]-t1[0], t3[1]-t1[1], t3[2]-t1[2]};

  // N is normal to triangle plane
  sim_scalar_t N[3];
  sim_math_internal_cross(N, A, B);
  sim_math_normalize_3(N);

  // dstS is signed distance from S to plane; exit if too large
  sim_scalar_t dstS = sim_math_dot_3(N, S);
  if (sim_math_abs(dstS) > rbound) {
    return 0;
  }

  // P is projection of S in triangle plane
  sim_scalar_t P[3];
  sim_math_internal_addScl3(P, S, N, -dstS);

  // construct orthogonal axes (V1~A, V2) of triangle plane
  sim_scalar_t V1[3], V2[3];
  sim_math_internal_copy_3(V1, A);
  sim_scalar_t lenA = sim_math_normalize_3(V1);
  sim_math_internal_cross(V2, N, A);
  sim_math_normalize_3(V2);

  // triangle is (o,a,b), sphere center is p
  sim_scalar_t o[2] = {0, 0};
  sim_scalar_t a[2] = {lenA, 0};  // equals {sim_math_dot_3(V1, A), sim_math_dot_3(V2, A)}
  sim_scalar_t b[2] = {sim_math_dot_3(V1, B), sim_math_dot_3(V2, B)};
  sim_scalar_t p[2] = {sim_math_dot_3(V1, P), sim_math_dot_3(V2, P)};

  // computed signs of areas of (p,o,a), (p,a,b), (p,b,o)
  sim_scalar_t sign1 = areaSign(p, o, a);
  sim_scalar_t sign2 = areaSign(p, a, b);
  sim_scalar_t sign3 = areaSign(p, b, o);

  // p is inside triangle
  if (sign1 == sign2 && sign2 == sign3) {
    // P is nearest point to S within triangle
    sim_math_internal_copy_3(X, P);
  }

  // p is not inside triangle
  else {
    // find nearest point to p on triangle edges (o,a), (a,b), (b,o)
    sim_scalar_t x[3][2], dstx[3];
    dstx[0] = pointSegment(x[0], p, o, a);
    dstx[1] = pointSegment(x[1], p, a, b);
    dstx[2] = pointSegment(x[2], p, b, o);

    // select minimum
    int best = (dstx[0] < dstx[1] && dstx[0] < dstx[2]) ? 0 : (dstx[1] < dstx[2] ? 1 : 2);

    // convert x[best] to 3D
    sim_math_internal_scl3(X, V1, x[best][0]);
    sim_math_internal_addToScl3(X, V2, x[best][1]);
  }

  // X is now the nearest point to S within the 3D triangle (O,A,B)
  // compute contact normal and distance
  sim_scalar_t nrm[3] = {X[0]-S[0], X[1]-S[1], X[2]-S[2]};
  sim_scalar_t dst = sim_math_normalize_3(nrm);

  // exit if too far
  if (dst > rbound) {
    return 0;
  }

  // construct contact
  con[0].dist = dst - rs - rt;
  sim_math_internal_addScl3(con[0].pos, s, nrm, rs + con[0].dist / 2);
  sim_math_internal_copy_3(con[0].frame, nrm);
  sim_math_zero_3(con[0].frame+3);

  return 1;
}

// box : triangle with radius
int SIM_raw_BoxTriangle(sim_contact_t* con, sim_scalar_t margin, const sim_scalar_t* pos,
                      const sim_scalar_t* mat, const sim_scalar_t* size, const sim_scalar_t* t1,
                      const sim_scalar_t* t2, const sim_scalar_t* t3, sim_scalar_t rt) {
  int cnt = 0;
  const sim_scalar_t* vert[3] = {t1, t2, t3};

  for (int i = 0; i < 3; i++) {
    // map vertex to box local frame
    sim_scalar_t diff[3], local[3];
    sim_math_sub_3(diff, vert[i], pos);
    sim_math_mulMatTVec3(local, mat, diff);

    // find max penetration / closest face
    int maxaxis = 0;
    sim_scalar_t maxval = sim_math_abs(local[0]) - size[0];
    for (int j = 1; j < 3; j++) {
      sim_scalar_t val = sim_math_abs(local[j]) - size[j];
      if (val > maxval) {
        maxval = val;
        maxaxis = j;
      }
    }

    // contact distance: dist = maxval - rt
    // strictly, we only care if dist < margin
    if (maxval - rt > margin) {
      continue;
    }

    // check if within other dimensions (with margin/radius)
    int inside = 1;
    for (int j = 0; j < 3; j++) {
      if (sim_math_abs(local[j]) > size[j] + margin + rt) {
        inside = 0;
        break;
      }
    }
    if (!inside) {
      continue;
    }

    // create contact
    if (cnt < SIM_MAXCONPAIR) {
      // normal in local frame
      sim_scalar_t nrm_local[3] = {0, 0, 0};
      nrm_local[maxaxis] = (local[maxaxis] > 0 ? 1 : -1);

      // normal in global frame (from Box to Triangle)
      sim_math_mul_mat_vec_3(con[cnt].frame, mat, nrm_local);

      // distance
      con[cnt].dist = maxval - rt;

      // position: v - nrm * (rt + dist/2)
      sim_scalar_t offset = rt + con[cnt].dist * 0.5;
      sim_math_internal_addScl3(con[cnt].pos, vert[i], con[cnt].frame, -offset);

      // frame details
      sim_math_zero_3(con[cnt].frame + 3);

      cnt++;
    }
  }

  // check box corners against triangle
  for (int i = 0; i < 8; i++) {
    if (cnt >= SIM_MAXCONPAIR) {
      break;
    }

    // get corner in local coordinates
    sim_scalar_t vec[3];
    vec[0] = (i & 1 ? size[0] : -size[0]);
    vec[1] = (i & 2 ? size[1] : -size[1]);
    vec[2] = (i & 4 ? size[2] : -size[2]);

    // get corner in global coordinates relative to box center
    sim_scalar_t corner[3];
    sim_math_mul_mat_vec_3(corner, mat, vec);
    sim_math_add_to_3(corner, pos);

    // check collision with triangle (radius 0 for corner)
    if (SIM_raw_SphereTriangle(con + cnt, margin, corner, 0, t1, t2, t3, rt)) {
      // SIM_raw_SphereTriangle normal points from Sphere (Corner) to Triangle.
      cnt++;
    }
  }

  return cnt;
}

// capsule : triangle with radius
int SIM_raw_CapsuleTriangle(sim_contact_t* con, sim_scalar_t margin, const sim_scalar_t* pos,
                          const sim_scalar_t* mat, const sim_scalar_t* size,
                          const sim_scalar_t* t1, const sim_scalar_t* t2, const sim_scalar_t* t3,
                          sim_scalar_t rt) {
  int cnt = 0;
  sim_scalar_t radius = size[0];
  sim_scalar_t len = size[1];
  sim_scalar_t axis[3] = {mat[2], mat[5], mat[8]};
  sim_scalar_t p1[3], p2[3];

  // capsule endpoints
  sim_math_addScl3(p1, pos, axis, -len);
  sim_math_addScl3(p2, pos, axis, len);

  // Check endpoints against triangle
  cnt += SIM_raw_SphereTriangle(con + cnt, margin, p1, radius, t1, t2, t3, rt);
  if (cnt >= SIM_MAXCONPAIR) return cnt;
  cnt += SIM_raw_SphereTriangle(con + cnt, margin, p2, radius, t1, t2, t3, rt);
  if (cnt >= SIM_MAXCONPAIR) return cnt;

  // Check triangle vertices against capsule axis (Point-Segment)
  const sim_scalar_t* vert[3] = {t1, t2, t3};
  for (int i = 0; i < 3; i++) {
    // point-segment distance
    sim_scalar_t vec[3], ab[3];
    sim_math_sub_3(vec, vert[i], p1);
    sim_math_sub_3(ab, p2, p1);
    sim_scalar_t t = sim_math_dot_3(vec, ab) / (4 * len * len);  // ab length is 2*len

    // clamp t to [0, 1] segment (only process interior)
    if (t <= SIM_MINVAL || t >= 1 - SIM_MINVAL) {
      continue;
    }

    // closest point on segment
    sim_scalar_t closest[3];
    sim_math_internal_addScl3(closest, p1, ab, t);

    // distance vector
    sim_math_sub_3(vec, vert[i], closest);
    sim_scalar_t dist = sim_math_normalize_3(vec);

    if (dist > radius + rt + margin) {
      continue;
    }

    // con->dist
    con[cnt].dist = dist - radius - rt;

    // Frame: normal from Capsule to Triangle. 'vec' points Closest->Vert.
    sim_math_internal_copy_3(con[cnt].frame, vec);
    sim_math_zero_3(con[cnt].frame + 3);

    // Position: midway between surfaces
    sim_math_internal_add3(con[cnt].pos, closest, vert[i]);
    sim_math_internal_addToScl3(con[cnt].pos, vec, radius - rt);
    sim_math_scale_3(con[cnt].pos, con[cnt].pos, 0.5);

    cnt++;
    if (cnt >= SIM_MAXCONPAIR) return cnt;
  }

  return cnt;
}
