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

#include "engine/engine_sensor.h"

#include <stddef.h>

#include <simcore/SIM_data.h>
#include <simcore/SIM_model.h>
#include <simcore/SIM_plugin.h>
#include <simcore/SIM_san.h>  // IWYU pragma: keep
#include "engine/engine_callback.h"
#include "engine/engine_collision_sdf.h"
#include "engine/engine_core_smooth.h"
#include "engine/engine_core_util.h"
#include "engine/engine_crossplatform.h"
#include "engine/engine_memory.h"
#include "engine/engine_plugin.h"
#include "engine/engine_ray.h"
#include "engine/engine_sleep.h"
#include "engine/engine_sort.h"
#include "engine/engine_support.h"
#include "engine/engine_util_blas.h"
#include "engine/engine_util_errmem.h"
#include "engine/engine_util_misc.h"
#include "engine/engine_util_spatial.h"



//-------------------------------- utility ---------------------------------------------------------


typedef struct {
  sim_scalar_t criterion;  // criterion for partial sort
  int id;            // index in d->contact
  int flip;          // 0: don't flip the normal, 1: flip the normal
} ContactInfo;

// define ContactSelect: find the k smallest elements of a ContactInfo array
static int ContactInfoCompare(const ContactInfo* a, const ContactInfo* b, void* context) {
  if (a->criterion < b->criterion) return -1;
  if (a->criterion > b->criterion) return 1;

  if (a->id < b->id) return -1;
  if (a->id > b->id) return 1;

  return 0;
}
SIM_PARTIAL_SORT(ContactSelect, ContactInfo, ContactInfoCompare);


// apply cutoff to sensor i, clamping values in data buffer
static void apply_cutoff(const sim_model_t* m, int i, sim_scalar_t* data) {
  sim_scalar_t cutoff = m->sensor_cutoff[i];
  if (cutoff <= 0) {
    return;
  }

  // cutoff ignored for contact and fromto sensors (but used by fromto sensors in a different way)
  SIM_tSensor type = (SIM_tSensor)m->sensor_type[i];
  if (type == SIM_SENS_CONTACT || type == SIM_SENS_GEOMFROMTO) {
    return;
  }

  int dim = m->sensor_dim[i];

  for (int j=0; j < dim; j++) {
    // real: apply on both sides
    if (m->sensor_datatype[i] == SIM_DATATYPE_REAL) {
      data[j] = sim_math_clip(data[j], -cutoff, cutoff);
    }

    // positive: apply on positive side only
    else if (m->sensor_datatype[i] == SIM_DATATYPE_POSITIVE) {
      data[j] = sim_math_min(cutoff, data[j]);
    }
  }
}


// get xpos and xmat pointers to an object in sim_data_t
static void get_xpos_xmat(const sim_data_t* d, sim_obj_t type, int id, int sensor_id,
                          sim_scalar_t **xpos, sim_scalar_t **xmat) {
  switch (type) {
  case SIM_OBJ_XBODY:
    *xpos = d->xpos + 3*id;
    *xmat = d->xmat + 9*id;
    break;
  case SIM_OBJ_BODY:
    *xpos = d->xipos + 3*id;
    *xmat = d->ximat + 9*id;
    break;
  case SIM_OBJ_GEOM:
    *xpos = d->geom_xpos + 3*id;
    *xmat = d->geom_xmat + 9*id;
    break;
  case SIM_OBJ_SITE:
    *xpos = d->site_xpos + 3*id;
    *xmat = d->site_xmat + 9*id;
    break;
  case SIM_OBJ_CAMERA:
    *xpos = d->cam_xpos + 3*id;
    *xmat = d->cam_xmat + 9*id;
    break;
  default:
    SIM_ERROR("invalid object type in sensor %d", sensor_id);
  }
}


// get global quaternion of an object in sim_data_t
static void get_xquat(const sim_model_t* m, const sim_data_t* d, sim_obj_t type, int id, int sensor_id,
                      sim_scalar_t *quat) {
  switch (type) {
  case SIM_OBJ_XBODY:
    sim_math_copy4(quat, d->xquat+4*id);
    break;
  case SIM_OBJ_BODY:
    sim_math_mulQuat(quat, d->xquat+4*id, m->body_iquat+4*id);
    break;
  case SIM_OBJ_GEOM:
    sim_math_mulQuat(quat, d->xquat+4*m->geom_bodyid[id], m->geom_quat+4*id);
    break;
  case SIM_OBJ_SITE:
    sim_math_mulQuat(quat, d->xquat+4*m->site_bodyid[id], m->site_quat+4*id);
    break;
  case SIM_OBJ_CAMERA:
    sim_math_mulQuat(quat, d->xquat+4*m->cam_bodyid[id], m->cam_quat+4*id);
    break;
  default:
    SIM_ERROR("invalid object type in sensor %d", sensor_id);
  }
}


static void cam_project(sim_scalar_t sensordata[2], const sim_scalar_t target_xpos[3],
                        const sim_scalar_t cam_xpos[3], const sim_scalar_t cam_xmat[9],
                        const int cam_res[2], sim_scalar_t cam_fovy,
                        const float cam_intrinsic[4], const float cam_sensorsize[2]) {
  sim_scalar_t fx, fy;

  // translation matrix (4x4)
  sim_scalar_t translation[4][4] = {0};
  translation[0][0] = 1;
  translation[1][1] = 1;
  translation[2][2] = 1;
  translation[3][3] = 1;
  translation[0][3] = -cam_xpos[0];
  translation[1][3] = -cam_xpos[1];
  translation[2][3] = -cam_xpos[2];

  // rotation matrix (4x4)
  sim_scalar_t rotation[4][4] = {0};
  rotation[0][0] = 1;
  rotation[1][1] = 1;
  rotation[2][2] = 1;
  rotation[3][3] = 1;
  for (int i=0; i < 3; i++) {
    for (int j=0; j < 3; j++) {
      rotation[i][j] = cam_xmat[j*3+i];
    }
  }

  // focal transformation matrix (3x4)
  if (cam_sensorsize[0] && cam_sensorsize[1]) {
    fx = cam_intrinsic[0] / cam_sensorsize[0] * cam_res[0];
    fy = cam_intrinsic[1] / cam_sensorsize[1] * cam_res[1];
  } else {
    fx = fy = .5 / sim_math_tan(cam_fovy * SIM_PI / 360.) * cam_res[1];
  }

  sim_scalar_t focal[3][4] = {0};
  focal[0][0] = -fx;
  focal[1][1] =  fy;
  focal[2][2] = 1.0;

  // image matrix (3x3)
  sim_scalar_t image[3][3] = {0};
  image[0][0] = 1;
  image[1][1] = 1;
  image[2][2] = 1;
  image[0][2] = (sim_scalar_t)cam_res[0] / 2.0;
  image[1][2] = (sim_scalar_t)cam_res[1] / 2.0;

  // projection matrix (3x4): product of all 4 matrices
  sim_scalar_t proj[3][4] = {0};
  for (int i=0; i < 3; i++) {
    for (int j=0; j < 3; j++) {
      for (int k=0; k < 4; k++) {
        for (int l=0; l < 4; l++) {
          for (int n=0; n < 4; n++) {
            proj[i][n] += image[i][j] * focal[j][k] * rotation[k][l] * translation[l][n];
          }
        }
      }
    }
  }

  // projection matrix multiplies homogenous [x, y, z, 1] vectors
  sim_scalar_t pos_hom[4] = {0, 0, 0, 1};
  sim_math_copy_3(pos_hom, target_xpos);

  // project world coordinates into pixel space, see:
  // https://en.wikipedia.org/wiki/3D_projection#Mathematical_formula
  sim_scalar_t pixel_coord_hom[3] = {0};
  for (int i=0; i < 3; i++) {
    for (int j=0; j < 4; j++) {
      pixel_coord_hom[i] += proj[i][j] * pos_hom[j];
    }
  }

  // avoid dividing by tiny numbers
  sim_scalar_t denom = pixel_coord_hom[2];
  if (sim_math_abs(denom) < SIM_MINVAL) {
    if (denom < 0) {
      denom = sim_math_min(denom, -SIM_MINVAL);
    } else {
      denom = sim_math_max(denom, SIM_MINVAL);
    }
  }

  // compute projection
  sensordata[0] = pixel_coord_hom[0] / denom;
  sensordata[1] = pixel_coord_hom[1] / denom;
}


// check if a contact body/geom matches a sensor spec (type, id)
static int checkMatch(const sim_model_t* m, int body, int geom, sim_obj_t type, int id) {
  if (type == SIM_OBJ_UNKNOWN) return 1;
  if (type == SIM_OBJ_SITE)    return 1;  // already passed site filter test
  if (type == SIM_OBJ_GEOM)    return id == geom;
  if (type == SIM_OBJ_BODY)    return id == body;
  if (type == SIM_OBJ_XBODY) {
    // traverse up the tree from body, return true if we land on id
    while (body > id) {
      body = m->body_parentid[body];
    }
    return body == id;
  }

  return 0;
}

//   0: no match
//   1: match, use contact normal
//  -1: match, flip contact normal
static int matchContact(const sim_model_t* m, const sim_data_t* d, int conid,
                        sim_obj_t type1, int id1, sim_obj_t type2, int id2) {
  // no criterion: quick match
  if (type1 == SIM_OBJ_UNKNOWN && type2 == SIM_OBJ_UNKNOWN) {
    return 1;
  }

  // site filter
  if (type1 == SIM_OBJ_SITE) {
    if (!sim_math_insideGeom(d->site_xpos + 3 * id1, d->site_xmat + 9 * id1,
                        m->site_size + 3 * id1, m->site_type[id1], d->contact[conid].pos)) {
      return 0;
    }
  }

  // get geom, body ids
  int geom1 = d->contact[conid].geom[0];
  int geom2 = d->contact[conid].geom[1];
  int body1 = geom1 >= 0 ? m->geom_bodyid[geom1] : -1;
  int body2 = geom2 >= 0 ? m->geom_bodyid[geom2] : -1;

  // check match of sensor objects with contact objects
  int match11 = checkMatch(m, body1, geom1, type1, id1);
  int match12 = checkMatch(m, body2, geom2, type1, id1);
  int match21 = checkMatch(m, body1, geom1, type2, id2);
  int match22 = checkMatch(m, body2, geom2, type2, id2);

  // if a sensor object is specified, it must be involved in the contact
  if (!match11 && !match12) return 0;
  if (!match21 && !match22) return 0;

  // determine direction
  if (type1 != SIM_OBJ_UNKNOWN && type2 != SIM_OBJ_UNKNOWN) {
    // both obj1 and obj2 specified: direction depends on order
    int order_regular = match11 && match22;
    int order_reverse = match12 && match21;
    if (order_regular && !order_reverse) return 1;
    if (order_reverse && !order_regular) return -1;
    if (order_regular && order_reverse)  return 1;  // ambiguous, return 1
  } else if (type1 != SIM_OBJ_UNKNOWN) {
    // only obj1 specified: normal points away from obj1
    return match11 ? 1 : -1;
  } else if (type2 != SIM_OBJ_UNKNOWN) {
    // only obj2 specified: normal points towards obj2
    return match22 ? 1 : -1;
  }

  // should not occur, all conditions are covered above
  return 0;
}


// fill in output data for contact sensor for all fields
//   if flg_flip > 0, normal/tangent rotate 180 about frame[2]
//   force/torque flip-z s.t. force is equal-and-opposite in new contact frame
static void copySensorData(const sim_model_t* m, const sim_data_t* d,
                           sim_scalar_t* data[SIM_NCONDATA], int id, int flg_flip, int nfound) {
  // found flag
  if (data[SIM_CONDATA_FOUND]) *data[SIM_CONDATA_FOUND] = nfound;

  // contact force and torque
  if (data[SIM_CONDATA_FORCE] || data[SIM_CONDATA_TORQUE]) {
    sim_scalar_t forcetorque[6];
    sim_contactForce(m, d, id, forcetorque);
    if (data[SIM_CONDATA_FORCE]) {
      sim_math_copy_3(data[SIM_CONDATA_FORCE], forcetorque);
      if (flg_flip) data[SIM_CONDATA_FORCE][2] *= -1;
    }
    if (data[SIM_CONDATA_TORQUE]) {
      sim_math_copy_3(data[SIM_CONDATA_TORQUE], forcetorque+3);
      if (flg_flip) data[SIM_CONDATA_TORQUE][2] *= -1;
    }
  }

  // contact penetration distance
  if (data[SIM_CONDATA_DIST]) {
    *data[SIM_CONDATA_DIST] = d->contact[id].dist;
  }

  // contact position
  if (data[SIM_CONDATA_POS]) {
    sim_math_copy_3(data[SIM_CONDATA_POS], d->contact[id].pos);
  }

  // contact normal
  if (data[SIM_CONDATA_NORMAL]) {
    sim_math_copy_3(data[SIM_CONDATA_NORMAL], d->contact[id].frame);
    if (flg_flip) sim_math_scale_3(data[SIM_CONDATA_NORMAL], data[SIM_CONDATA_NORMAL], -1);
  }

  // contact first tangent
  if (data[SIM_CONDATA_TANGENT]) {
    sim_math_copy_3(data[SIM_CONDATA_TANGENT], d->contact[id].frame+3);
    if (flg_flip) sim_math_scale_3(data[SIM_CONDATA_TANGENT], data[SIM_CONDATA_TANGENT], -1);
  }
}


// compute total wrench about one point, in the global frame
static void total_wrench(sim_scalar_t force[3], sim_scalar_t torque[3], const sim_scalar_t point[3], int n,
                        const sim_scalar_t *wrench, const sim_scalar_t *pos, const sim_scalar_t *frame) {
  sim_math_zero_3(force);
  sim_math_zero_3(torque);

  for (int j = 0; j < n; ++j) {
    // rotate force, torque from contact frame to global frame
    sim_scalar_t force_j[3], torque_j[3];
    sim_math_mulMatTVec3(force_j, frame + 9*j, wrench + 6*j);
    sim_math_mulMatTVec3(torque_j, frame + 9*j, wrench + 6*j + 3);

    // add to total force, torque
    sim_math_add_to_3(force, force_j);
    sim_math_add_to_3(torque, torque_j);

    // add induced moment:  torque += (pos - point) x force
    sim_scalar_t diff[3];
    sim_math_sub_3(diff, pos + 3*j, point);
    sim_scalar_t induced_torque[3];
    sim_math_cross(induced_torque, diff, force_j);
    sim_math_add_to_3(torque, induced_torque);
  }
}


//-------------------------------- sensor ----------------------------------------------------------

// fill one pixel's worth of rangefinder data, advance ptr
static sim_scalar_t* fill_raydata(sim_scalar_t* ptr, int dataspec, sim_scalar_t dist,
                            const sim_scalar_t origin[3], const sim_scalar_t direction[3],
                            const sim_scalar_t normal[3], const sim_scalar_t cam_xpos[3],
                            const sim_scalar_t cam_z[3]) {
  int hit = (dist >= 0);

  if (dataspec & (1 << SIM_RAYDATA_DIST)) {
    *ptr++ = dist;
  }
  if (dataspec & (1 << SIM_RAYDATA_DIR)) {
    if (hit) sim_math_copy_3(ptr, direction);
    else     sim_math_zero_3(ptr);
    ptr += 3;
  }
  if (dataspec & (1 << SIM_RAYDATA_ORIGIN)) {
    sim_math_copy_3(ptr, origin);
    ptr += 3;
  }

  // compute point if needed for POINT or DEPTH fields
  sim_scalar_t point[3] = {0, 0, 0};
  if ((dataspec & (1 << SIM_RAYDATA_POINT)) || (dataspec & (1 << SIM_RAYDATA_DEPTH))) {
    if (hit) sim_math_addScl3(point, origin, direction, dist);
  }

  if (dataspec & (1 << SIM_RAYDATA_POINT)) {
    sim_math_copy_3(ptr, point);
    ptr += 3;
  }
  if (dataspec & (1 << SIM_RAYDATA_NORMAL)) {
    if (hit) sim_math_copy_3(ptr, normal);
    else     sim_math_zero_3(ptr);
    ptr += 3;
  }
  if (dataspec & (1 << SIM_RAYDATA_DEPTH)) {
    if (hit) {
      if (cam_z) {
        // camera depth: project onto camera z-axis
        sim_scalar_t delta[3];
        sim_math_sub_3(delta, point, cam_xpos);
        *ptr++ = -sim_math_dot_3(delta, cam_z);
      } else {
        // site sensor: depth = dist
        *ptr++ = dist;
      }
    } else {
      *ptr++ = -1;
    }
  }

  return ptr;
}


// compute position-stage sensor value, write to data buffer
static void sim_computeSensorPos(const sim_model_t* m, sim_data_t* d, int i, sim_scalar_t* sensordata) {
  int ne = d->ne, nf = d->nf, nefc = d->nefc;
  SIM_tSensor type = (SIM_tSensor)m->sensor_type[i];
  int objtype = m->sensor_objtype[i];
  int objid = m->sensor_objid[i];
  int refid = m->sensor_refid[i];
  int reftype = m->sensor_reftype[i];

  sim_scalar_t rvec[3], *xpos, *xmat, *xpos_ref, *xmat_ref;

  // process according to type
  switch (type) {
  case SIM_SENS_MAGNETOMETER:                           // magnetometer
    sim_math_mulMatTVec(sensordata, d->site_xmat+9*objid, m->opt.magnetic, 3, 3);
    break;

  case SIM_SENS_CAMPROJECTION:                          // camera projection
    cam_project(sensordata, d->site_xpos+3*objid, d->cam_xpos+3*refid,
                d->cam_xmat+9*refid, m->cam_resolution+2*refid, m->cam_fovy[refid],
                m->cam_intrinsic+4*refid, m->cam_sensorsize+2*refid);
    break;

  case SIM_SENS_RANGEFINDER:                            // rangefinder
    {
      // get dataspec
      int dataspec = m->sensor_intprm[i*SIM_NSENS];

      if (objtype == SIM_OBJ_SITE) {
        // site-attached rangefinder: single ray
        rvec[0] = d->site_xmat[9*objid+2];
        rvec[1] = d->site_xmat[9*objid+5];
        rvec[2] = d->site_xmat[9*objid+8];
        const sim_scalar_t* origin = d->site_xpos + 3*objid;

        int geomid;
        sim_scalar_t normal[3];
        sim_scalar_t* p_normal = (dataspec & (1 << SIM_RAYDATA_NORMAL)) ? normal : NULL;
        sim_scalar_t dist = sim_ray(m, d, origin, rvec, NULL, 1,
                             m->site_bodyid[objid], &geomid, p_normal);

        // for site sensor: pass NULL for cam_z so depth = dist
        fill_raydata(sensordata, dataspec, dist, origin, rvec, normal, NULL, NULL);

      } else {
        // camera-attached rangefinder: depth image
        const int width = m->cam_resolution[2*objid];
        const int height = m->cam_resolution[2*objid+1];
        const int bodyexclude = m->cam_bodyid[objid];
        const sim_scalar_t* cam_xpos = d->cam_xpos + 3*objid;
        const sim_scalar_t* cam_xmat = d->cam_xmat + 9*objid;
        const int projection = m->cam_projection[objid];

        // camera z-axis (pointing into scene, negative of optical axis)
        sim_scalar_t cam_z[3] = {cam_xmat[2], cam_xmat[5], cam_xmat[8]};

        // compute focal length in pixels using helper
        sim_scalar_t fx, fy, cx, cy, ortho_extent;
        sim_math_camIntrinsics(m, objid, &fx, &fy, &cx, &cy, &ortho_extent);

        if (projection == SIM_PROJ_PERSPECTIVE) {
          // perspective: all rays share origin, different directions
          const int npixel = width * height;
          sim_markStack(d);
          sim_scalar_t* vec = SIM_STACK_ALLOC(d, 3*npixel, sim_scalar_t);
          int* geomid = SIM_STACK_ALLOC(d, npixel, int);
          sim_scalar_t* dist = SIM_STACK_ALLOC(d, npixel, sim_scalar_t);
          sim_scalar_t* normals = NULL;
          if (dataspec & (1 << SIM_RAYDATA_NORMAL)) {
            normals = SIM_STACK_ALLOC(d, 3*npixel, sim_scalar_t);
          }

          // compute ray directions using helper (normalized)
          for (int row = 0; row < height; row++) {
            for (int col = 0; col < width; col++) {
              int idx = row*width + col;
              sim_scalar_t origin[3];
              sim_math_camPixelRay(origin, vec + 3*idx, cam_xpos, cam_xmat,
                              col, row, fx, fy, cx, cy, projection, ortho_extent);
            }
          }

          // cast all rays with normals if needed
          sim_multiRay(m, d, cam_xpos, vec, NULL, 1, bodyexclude,
                      geomid, dist, normals, npixel, SIM_MAXVAL);

          // fill in output for each pixel
          for (int row = 0; row < height; row++) {
            for (int col = 0; col < width; col++) {
              int idx = row*width + col;
              sim_scalar_t* normal_ptr = normals ? normals + 3*idx : NULL;
              sensordata = fill_raydata(sensordata, dataspec, dist[idx], cam_xpos,
                                        vec + 3*idx, normal_ptr, cam_xpos, cam_z);
            }
          }

          sim_freeStack(d);
        } else {
          // orthographic: parallel rays, different origins
          for (int row = 0; row < height; row++) {
            for (int col = 0; col < width; col++) {
              sim_scalar_t origin[3], direction[3];
              sim_math_camPixelRay(origin, direction, cam_xpos, cam_xmat,
                              col, row, fx, fy, cx, cy, projection, ortho_extent);

              int geomid;
              sim_scalar_t normal[3];
              sim_scalar_t dist = sim_ray(m, d, origin, direction, NULL, 1,
                                   bodyexclude, &geomid, normal);

              sensordata = fill_raydata(sensordata, dataspec, dist, origin, direction,
                                 normal, cam_xpos, cam_z);
            }
          }
        }
      }
    }

    break;

  case SIM_SENS_JOINTPOS:                               // joint position
    sensordata[0] = d->qpos[m->jnt_qposadr[objid]];
    break;

  case SIM_SENS_TENDONPOS:                              // tendon position
    sensordata[0] = d->ten_length[objid];
    break;

  case SIM_SENS_ACTUATORPOS:                            // actuator position
    sensordata[0] = d->actuator_length[objid];
    break;

  case SIM_SENS_BALLQUAT:                               // ball joint quaternion
    sim_math_copy4(sensordata, d->qpos+m->jnt_qposadr[objid]);
    sim_math_normalize4(sensordata);
    break;

  case SIM_SENS_JOINTLIMITPOS:                          // joint limit distance-margin
    sensordata[0] = 0;
    for (int j=ne+nf; j < nefc; j++) {
      if (d->efc_type[j] == SIM_CNSTR_LIMIT_JOINT && d->efc_id[j] == objid) {
        sensordata[0] = d->efc_pos[j] - d->efc_margin[j];
        break;
      }
    }
    break;

  case SIM_SENS_TENDONLIMITPOS:                         // tendon limit distance-margin
    sensordata[0] = 0;
    for (int j=ne+nf; j < nefc; j++) {
      if (d->efc_type[j] == SIM_CNSTR_LIMIT_TENDON && d->efc_id[j] == objid) {
        sensordata[0] = d->efc_pos[j] - d->efc_margin[j];
        break;
      }
    }
    break;

  case SIM_SENS_FRAMEPOS:                               // 3D position
  case SIM_SENS_FRAMEXAXIS:                             // x-axis of object's frame
  case SIM_SENS_FRAMEYAXIS:                             // y-axis of object's frame
  case SIM_SENS_FRAMEZAXIS:                             // z-axis of object's frame
    // get xpos and xmat pointers for object frame
    get_xpos_xmat(d, objtype, objid, i, &xpos, &xmat);

    // reference frame unspecified: global frame
    if (refid == -1) {
      if (type == SIM_SENS_FRAMEPOS) {
        sim_math_copy_3(sensordata, xpos);
      } else {
        // offset = (0 or 1 or 2) for (x or y or z)-axis sensors, respectively
        int offset = type - SIM_SENS_FRAMEXAXIS;
        sensordata[0] = xmat[offset+0];
        sensordata[1] = xmat[offset+3];
        sensordata[2] = xmat[offset+6];
      }
    }

    // reference frame specified
    else {
      get_xpos_xmat(d, reftype, refid, i, &xpos_ref, &xmat_ref);
      if (type == SIM_SENS_FRAMEPOS) {
        sim_math_sub_3(rvec, xpos, xpos_ref);
        sim_math_mulMatTVec3(sensordata, xmat_ref, rvec);
      } else {
        // offset = (0 or 1 or 2) for (x or y or z)-axis sensors, respectively
        int offset = type - SIM_SENS_FRAMEXAXIS;
        sim_scalar_t axis[3] = {xmat[offset], xmat[offset+3], xmat[offset+6]};
        sim_math_mulMatTVec3(sensordata, xmat_ref, axis);
      }
    }
    break;

  case SIM_SENS_FRAMEQUAT:                              // orientation quaternion
  {
    // get global object quaternion
    sim_scalar_t objquat[4];
    get_xquat(m, d, objtype, objid, i, objquat);

    // reference frame unspecified: copy object quaternion
    if (refid == -1) {
      sim_math_copy4(sensordata, objquat);
    } else {
      // reference frame specified, get global reference quaternion
      sim_scalar_t refquat[4];
      get_xquat(m, d, reftype, refid, i, refquat);

      // relative quaternion
      sim_math_negQuat(refquat, refquat);
      sim_math_mulQuat(sensordata, refquat, objquat);
    }
  }
  break;

  case SIM_SENS_SUBTREECOM:                             // subtree center of mass
    sim_math_copy_3(sensordata, d->subtree_com+3*objid);
    break;

  case SIM_SENS_INSIDESITE:                             // 1 if object is inside site
    get_xpos_xmat(d, objtype, objid, i, &xpos, &xmat);
    sensordata[0] = sim_math_insideGeom(d->site_xpos + 3*refid,
                                   d->site_xmat + 9*refid,
                                   m->site_size + 3*refid,
                                   m->site_type[refid],
                                   xpos);
    break;

  case SIM_SENS_GEOMDIST:                               // signed distance between two geoms
  case SIM_SENS_GEOMNORMAL:                             // normal direction between two geoms
  case SIM_SENS_GEOMFROMTO:                             // segment between two geoms
    {
      sim_scalar_t cutoff = m->sensor_cutoff[i];

      // initialize outputs
      sim_scalar_t dist = cutoff;    // collision distance
      sim_scalar_t fromto[6] = {0};  // segment between geoms

      // get lists of geoms to collide
      int n1, id1;
      if (objtype == SIM_OBJ_BODY) {
        n1 = m->body_geomnum[objid];
        id1 = m->body_geomadr[objid];
      } else {
        n1 = 1;
        id1 = objid;
      }
      int n2, id2;
      if (reftype == SIM_OBJ_BODY) {
        n2 = m->body_geomnum[refid];
        id2 = m->body_geomadr[refid];
      } else {
        n2 = 1;
        id2 = refid;
      }

      // collide all pairs
      for (int geom1=id1; geom1 < id1+n1; geom1++) {
        for (int geom2=id2; geom2 < id2+n2; geom2++) {
          sim_scalar_t fromto_new[6] = {0};
          sim_scalar_t dist_new = sim_geomDistance(m, d, geom1, geom2, cutoff, fromto_new);
          if (dist_new < dist) {
            dist = dist_new;
            sim_math_copy(fromto, fromto_new, 6);
          }
        }
      }

      // write data
      if (type == SIM_SENS_GEOMDIST) {
        sensordata[0] = dist;
      } else if (type == SIM_SENS_GEOMNORMAL) {
        sim_scalar_t normal[3] = {fromto[3]-fromto[0], fromto[4]-fromto[1], fromto[5]-fromto[2]};
        if (normal[0] || normal[1] || normal[2]) {
          sim_math_normalize_3(normal);
        }
        sim_math_copy_3(sensordata, normal);
      } else {  // SIM_SENS_GEOMFROMTO
        sim_math_copy(sensordata, fromto, 6);
      }
    }
    break;

  case SIM_SENS_E_POTENTIAL:                            // potential energy
    if (!d->flg_energypos) {
      sim_energyPos(m, d);
    }
    sensordata[0] = d->energy[0];
    break;

  case SIM_SENS_E_KINETIC:                              // kinetic energy
    if (!d->flg_energyvel) {
      sim_energyVel(m, d);
    }
    sensordata[0] = d->energy[1];
    break;

  case SIM_SENS_CLOCK:                                  // simulation time
    sensordata[0] = d->time;
    break;

  default:
    SIM_ERROR("invalid sensor type in POS stage, sensor %d", i);
  }
}


// compute velocity-stage sensor value, write to data buffer
static void sim_computeSensorVel(const sim_model_t* m, sim_data_t* d, int i, sim_scalar_t* sensordata) {
  int ne = d->ne, nf = d->nf, nefc = d->nefc;
  SIM_tSensor type = (SIM_tSensor)m->sensor_type[i];
  int objtype = m->sensor_objtype[i];
  int objid = m->sensor_objid[i];
  int refid = m->sensor_refid[i];
  int reftype = m->sensor_reftype[i];

  sim_scalar_t xvel[6];

  // call sim_subtreeVel for sensors that need it (unless already computed)
  if (!d->flg_subtreevel &&
      (type == SIM_SENS_SUBTREELINVEL || type == SIM_SENS_SUBTREEANGMOM)) {
    sim_subtreeVel(m, d);
  }

  // process according to type
  switch (type) {
  case SIM_SENS_VELOCIMETER:                            // velocimeter
    // xvel = site velocity, in site frame
    sim_objectVelocity(m, d, SIM_OBJ_SITE, objid, xvel, 1);

    // assign linear velocity
    sim_math_copy_3(sensordata, xvel+3);
    break;

  case SIM_SENS_GYRO:                                   // gyro
    // xvel = site velocity, in site frame
    sim_objectVelocity(m, d, SIM_OBJ_SITE, objid, xvel, 1);

    // assign angular velocity
    sim_math_copy_3(sensordata, xvel);
    break;

  case SIM_SENS_JOINTVEL:                               // joint velocity
    sensordata[0] = d->qvel[m->jnt_dofadr[objid]];
    break;

  case SIM_SENS_TENDONVEL:                              // tendon velocity
    sensordata[0] = d->ten_velocity[objid];
    break;

  case SIM_SENS_ACTUATORVEL:                            // actuator velocity
    sensordata[0] = d->actuator_velocity[objid];
    break;

  case SIM_SENS_BALLANGVEL:                             // ball joint angular velocity
    sim_math_copy_3(sensordata, d->qvel+m->jnt_dofadr[objid]);
    break;

  case SIM_SENS_JOINTLIMITVEL:                          // joint limit velocity
    sensordata[0] = 0;
    for (int j=ne+nf; j < nefc; j++) {
      if (d->efc_type[j] == SIM_CNSTR_LIMIT_JOINT && d->efc_id[j] == objid) {
        sensordata[0] = d->efc_vel[j];
        break;
      }
    }
    break;

  case SIM_SENS_TENDONLIMITVEL:                         // tendon limit velocity
    sensordata[0] = 0;
    for (int j=ne+nf; j < nefc; j++) {
      if (d->efc_type[j] == SIM_CNSTR_LIMIT_TENDON && d->efc_id[j] == objid) {
        sensordata[0] = d->efc_vel[j];
        break;
      }
    }
    break;

  case SIM_SENS_FRAMELINVEL:                            // 3D linear velocity
  case SIM_SENS_FRAMEANGVEL:                            // 3D angular velocity
    // xvel = 6D object velocity, in global frame
    sim_objectVelocity(m, d, objtype, objid, xvel, 0);

    if (refid > -1) {  // reference frame specified
      sim_scalar_t *xpos, *xmat, *xpos_ref, *xmat_ref, xvel_ref[6], rel_vel[6], cross[3], rvec[3];

      // in global frame: object and reference position, reference orientation and velocity
      get_xpos_xmat(d, objtype, objid, i, &xpos, &xmat);
      get_xpos_xmat(d, reftype, refid, i, &xpos_ref, &xmat_ref);
      sim_objectVelocity(m, d, reftype, refid, xvel_ref, 0);

      // subtract velocities
      sim_math_sub(rel_vel, xvel, xvel_ref, 6);

      // linear velocity: add correction due to rotating reference frame
      sim_math_sub_3(rvec, xpos, xpos_ref);
      sim_math_cross(cross, rvec, xvel_ref);
      sim_math_add_to_3(rel_vel+3, cross);

      // project into reference frame
      sim_math_mulMatTVec3(xvel, xmat_ref, rel_vel);
      sim_math_mulMatTVec3(xvel+3, xmat_ref, rel_vel+3);
    }

    // copy linear or angular component
    if (type == SIM_SENS_FRAMELINVEL) {
      sim_math_copy_3(sensordata, xvel+3);
    } else {
      sim_math_copy_3(sensordata, xvel);
    }
    break;

  case SIM_SENS_SUBTREELINVEL:                          // subtree linear velocity
    sim_math_copy_3(sensordata, d->subtree_linvel+3*objid);
    break;

  case SIM_SENS_SUBTREEANGMOM:                          // subtree angular momentum
    sim_math_copy_3(sensordata, d->subtree_angmom+3*objid);
    break;

  default:
    SIM_ERROR("invalid type in VEL stage, sensor %d", i);
  }
}


// compute acceleration-stage sensor value, write to data buffer
static void sim_computeSensorAcc(const sim_model_t* m, sim_data_t* d, int i, sim_scalar_t* sensordata) {
  int ne = d->ne, nf = d->nf, nefc = d->nefc, nu = m->nu;
  SIM_tSensor type = (SIM_tSensor)m->sensor_type[i];
  int objtype = m->sensor_objtype[i];
  int objid = m->sensor_objid[i];

  sim_scalar_t tmp[6], conforce[6], conray[3], frc;
  const sim_contact_t* con;
  int rootid, bodyid;

  // call sim_rnePostConstraint for sensors that need it (unless already computed)
  if (!d->flg_rnepost &&
      (type == SIM_SENS_ACCELEROMETER ||
       type == SIM_SENS_FORCE         ||
       type == SIM_SENS_TORQUE        ||
       type == SIM_SENS_FRAMELINACC   ||
       type == SIM_SENS_FRAMEANGACC)) {
    sim_rnePostConstraint(m, d);
  }

  // process according to type
  switch (type) {
  case SIM_SENS_TOUCH:                                  // touch
    // extract body data
    bodyid = m->site_bodyid[objid];

    // clear result
    sensordata[0] = 0;

    // find contacts in sensor zone, add normal forces
    for (int j=0; j < d->ncon; j++) {
      // contact pointer, contacting bodies  (-1 for flex)
      con = d->contact + j;
      int conbody[2];
      for (int k=0; k < 2; k++) {
        conbody[k] = (con->geom[k] >= 0) ? m->geom_bodyid[con->geom[k]] : -1;
      }

      // select contacts involving sensorized body
      if (con->efc_address >= 0 && (bodyid == conbody[0] || bodyid == conbody[1])) {
        // get contact force:torque in contact frame
        sim_contactForce(m, d, j, conforce);

        // nothing to do if normal is zero
        if (conforce[0] <= 0) {
          continue;
        }

        // convert contact normal force to global frame, normalize
        sim_math_scale_3(conray, con->frame, conforce[0]);
        sim_math_normalize_3(conray);

        // flip ray direction if sensor is on body2
        if (bodyid == conbody[1]) {
          sim_math_scale_3(conray, conray, -1);
        }

        // add if ray-zone intersection (always true when con->pos inside zone)
        if (sim_math_rayGeom(d->site_xpos+3*objid, d->site_xmat+9*objid,
                        m->site_size+3*objid, con->pos, conray,
                        m->site_type[objid], NULL) >= 0) {
          sensordata[0] += conforce[0];
        }
      }
    }
    break;

  case SIM_SENS_CONTACT:                                // contact
    {
      // local reduce enum for readability
      enum {
        REDUCE_NONE     = 0,
        REDUCE_MINDIST  = 1,
        REDUCE_MAXFORCE = 2,
        REDUCE_NETFORCE = 3,
      };

      // prepare sizes and indices
      int dataspec = m->sensor_intprm[i*SIM_NSENS];
      int size = sim_math_condataSize(dataspec);  // size of each slot
      int dim = m->sensor_dim[i];            // total sensor array dimension
      int num = dim / size;                  // number of slots
      int reftype = m->sensor_reftype[i];
      int refid = m->sensor_refid[i];
      int reduce = m->sensor_intprm[i*SIM_NSENS+1];

      // clear all outputs, prepare data pointers
      sim_math_zero(sensordata, dim);
      sim_scalar_t* data[SIM_NCONDATA] = {NULL};
      for (int j=0; j < SIM_NCONDATA; j++) {
        if (dataspec & (1 << j)) {
          data[j] = sensordata;
          sensordata += SIM_CONDATA_SIZE[j];
        }
      }

      // prepare for matching loop
      int nmatch = 0;
      sim_markStack(d);
      ContactInfo *match = SIM_STACK_ALLOC(d, d->ncon, ContactInfo);

      // find matching contacts
      for (int j=0; j < d->ncon; j++) {
        // check match condition
        int match_j = matchContact(m, d, j, objtype, objid, reftype, refid);
        if (!match_j) {
          continue;
        }

        // save id and flip flag
        match[nmatch].id = j;
        match[nmatch].flip = match_j < 0;

        // save sorting criterion, if required
        if (reduce == REDUCE_MINDIST) {
          match[nmatch].criterion = d->contact[j].dist;
        } else if (reduce == REDUCE_MAXFORCE) {
          sim_scalar_t forcetorque[6];
          sim_contactForce(m, d, j, forcetorque);
          match[nmatch].criterion = -sim_math_dot_3(forcetorque, forcetorque);
        }

        // increment number of matching contacts
        nmatch++;
      }

      // number of slots to be filled
      int nslot = SIM_MIN(num, nmatch);

      // partial sort to get bottom nslot contacts if sorted reduction
      if (reduce == REDUCE_MINDIST || reduce == REDUCE_MAXFORCE) {
        ContactInfo *heap = SIM_STACK_ALLOC(d, nslot, ContactInfo);
        ContactSelect(match, heap, nmatch, nslot, NULL);
      }

      // netforce reduction
      else if (reduce == REDUCE_NETFORCE) {
        sim_scalar_t *wrench = SIM_STACK_ALLOC(d, nmatch * 6, sim_scalar_t);
        sim_scalar_t *pos = SIM_STACK_ALLOC(d, nmatch * 3, sim_scalar_t);
        sim_scalar_t *frame = SIM_STACK_ALLOC(d, nmatch * 9, sim_scalar_t);

        // precompute wrenches, positions, and frames, maybe flip wrench
        for (int j=0; j < nmatch; j++) {
          int conid = match[j].id;
          sim_contactForce(m, d, conid, wrench + 6*j);
          sim_math_copy_3(pos + 3*j, d->contact[conid].pos);
          sim_math_copy9(frame + 9*j, d->contact[conid].frame);
          if (match[j].flip) {
            sim_math_scl(wrench + 6*j , wrench + 6*j, -1, 6);
          }
        }

        // compute point: force-weighted centroid of contact positions
        sim_scalar_t point[3] = {0};
        sim_scalar_t total_force = 0;
        for (int j=0; j < nmatch; j++) {
          sim_scalar_t weight = sim_math_norm3(wrench + 6*j);
          sim_math_add_to_scale_3(point, pos + 3*j, weight);
          total_force += weight;
        }
        sim_math_scale_3(point, point, 1.0 / SIM_MAX(total_force, SIM_MINVAL));

        // compute total wrench about point, in the global frame
        sim_scalar_t force[3], torque[3];
        total_wrench(force, torque, point, nmatch, wrench, pos, frame);

        // write data to slot 0
        if (data[SIM_CONDATA_FOUND])   *data[SIM_CONDATA_FOUND] = nmatch;
        if (data[SIM_CONDATA_FORCE])   sim_math_copy_3(data[SIM_CONDATA_FORCE], force);
        if (data[SIM_CONDATA_TORQUE])  sim_math_copy_3(data[SIM_CONDATA_TORQUE], torque);
        if (data[SIM_CONDATA_DIST])    *data[SIM_CONDATA_DIST] = 0;
        if (data[SIM_CONDATA_POS])     sim_math_copy_3(data[SIM_CONDATA_POS], point);
        if (data[SIM_CONDATA_NORMAL])  data[SIM_CONDATA_NORMAL][0] = 1;
        if (data[SIM_CONDATA_TANGENT]) data[SIM_CONDATA_TANGENT][1] = 1;

        // done with this sensor
        sim_freeStack(d);
        break;
      }

      // copy data into slots, increment pointers
      for (int j=0; j < nslot; j++) {
        copySensorData(m, d, data, match[j].id, match[j].flip, nmatch);
        for (int k=0; k < SIM_NCONDATA; k++) {
          if (data[k]) data[k] += size;
        }
      }

      sim_freeStack(d);
    }
    break;

  case SIM_SENS_TACTILE:                                // tactile
    {
      sim_markStack(d);

      // get parent weld id
      int mesh_id = m->sensor_objid[i];
      int geom_id = m->sensor_refid[i];
      int parent_body = m->geom_bodyid[geom_id];
      int parent_weld = m->body_weldid[parent_body];
      int nchannel = m->sensor_dim[i] / m->mesh_vertnum[mesh_id];

      // clear sensordata and distance matrix
      sim_math_zero(sensordata, m->sensor_dim[i]);

      // count contacts and get contact geom ids
      // TODO: use a more efficient C version of unordered_set
      int* contact_geom_ids = sim_stackAllocInt(d, d->ncon);
      int ncontact = 0;
      for (int k = 0; k < d->ncon; k++) {
        int body1 = m->body_weldid[m->geom_bodyid[d->contact[k].geom1]];
        int body2 = m->body_weldid[m->geom_bodyid[d->contact[k].geom2]];
        if (body1 == parent_weld) {
          int add = 1;
          for (int j = 0; j < ncontact; j++) {
            if (contact_geom_ids[j] == d->contact[k].geom2) {
              add = 0;
              break;
            }
          }
          if (add) {
            contact_geom_ids[ncontact] = d->contact[k].geom2;
            ncontact++;
          }
        }
        if (body2 == parent_weld) {
          int add = 1;
          for (int j = 0; j < ncontact; j++) {
            if (contact_geom_ids[j] == d->contact[k].geom1) {
              add = 0;
              break;
            }
          }
          if (add) {
            contact_geom_ids[ncontact] = d->contact[k].geom1;
            ncontact++;
          }
        }
      }

      // no contacts, return
      if (ncontact == 0) {
        sim_freeStack(d);
        break;
      }

      // all of the quadrature points are contact points
      int ncon = m->mesh_vertnum[mesh_id];

      // get site frame
      sim_scalar_t* geom_pos = d->geom_xpos + 3*geom_id;
      sim_scalar_t* geom_mat = d->geom_xmat + 9*geom_id;

      // allocate contact forces and positions
      sim_scalar_t* forcesT = sim_stackAllocNum(d, ncon*3);
      sim_math_zero(forcesT, ncon*3);

      // iterate over colliding geoms
      for (int g = 0; g < ncontact; g++) {
        int geom = contact_geom_ids[g];
        int body = m->geom_bodyid[geom];

        // get sdf plugin of the geoms
        int sdf_instance[2] = {-1, -1};
        SIM_tGeom geomtype[2] = {SIM_GEOM_SDF, SIM_GEOM_SPHERE};
        const SIM_pPlugin* sdf_ptr[2] = {NULL, NULL};
        if (m->geom_type[geom] == SIM_GEOM_SDF) {
          sdf_instance[0] = m->geom_plugin[geom];
          sdf_ptr[0] = SIM_c_getSDF(m, geom);
        } else if (m->geom_type[geom] == SIM_GEOM_MESH) {
          sdf_instance[0] = m->geom_dataid[geom];
          geomtype[0] = (SIM_tGeom)m->geom_type[geom];
        } else {
          sdf_instance[0] = geom;
          geomtype[0] = (SIM_tGeom)m->geom_type[geom];
        }

        // skip mesh geoms not having an octree
        if (geomtype[0] == SIM_GEOM_MESH &&
            m->mesh_octadr[m->geom_dataid[geom]] == -1) {
          continue;
        }

        // set SDF parameters
        SIM_SDF geom_sdf;
        geom_sdf.id = &sdf_instance[0];
        geom_sdf.type = SIM_SDFTYPE_SINGLE;
        geom_sdf.plugin = &sdf_ptr[0];
        geom_sdf.geomtype = &geomtype[0];

        // get forces in mesh coordinates
        int node = 0;
        float* mesh_vert = m->mesh_vert + 3*m->mesh_vertadr[mesh_id];
        float* mesh_normal = m->mesh_normal + 3*m->mesh_normaladr[mesh_id];
        for (int j = 0; j < ncon; j++) {
          // position in site frame
          sim_scalar_t pos[3] = {mesh_vert[3*j + 0], mesh_vert[3*j + 1], mesh_vert[3*j + 2]};

          // position in global frame
          sim_scalar_t xpos[3];
          sim_math_mul_mat_vec_3(xpos, geom_mat, pos);
          sim_math_add_to_3(xpos, geom_pos);

          // position in other geom frame
          sim_scalar_t lpos[3];
          sim_math_sub_3(tmp, xpos, d->geom_xpos + 3*geom);
          sim_math_mulMatTVec3(lpos, d->geom_xmat + 9*geom, tmp);

          // SDF plugins are in the original mesh frame
          if (sdf_ptr[0] != NULL) {
            sim_scalar_t mesh_mat[9];
            sim_math_quat2Mat(mesh_mat, m->mesh_quat + 4 * m->geom_dataid[geom]);
            sim_math_mul_mat_vec_3(lpos, mesh_mat, lpos);
            sim_math_add_to_3(lpos, m->mesh_pos + 3 * m->geom_dataid[geom]);
          }

          // compute distance
          sim_scalar_t depth = sim_math_min(SIM_c_distance(m, d, &geom_sdf, lpos), 0);
          if (depth == 0) {
            node++;
            continue;
          }

          // get velocity in global frame
          sim_scalar_t vel_sensor[6], vel_other[6], vel_rel[3];
          sim_math_transformSpatial(
              vel_sensor, d->cvel + 6 * parent_weld, 0, xpos,
              d->subtree_com + 3 * m->body_rootid[parent_weld], NULL);
          sim_math_transformSpatial(
              vel_other, d->cvel + 6 * body, 0, d->geom_xpos + 3 * geom,
              d->subtree_com + 3 * m->body_rootid[body], NULL);
          sim_math_sub_3(vel_rel, vel_sensor+3, vel_other+3);

          sim_scalar_t normal[3] = {mesh_normal[9*j + 0], mesh_normal[9*j + 1], mesh_normal[9*j + 2]};
          sim_scalar_t tang1[3] =  {mesh_normal[9*j + 3], mesh_normal[9*j + 4], mesh_normal[9*j + 5]};
          sim_scalar_t tang2[3] =  {mesh_normal[9*j + 6], mesh_normal[9*j + 7], mesh_normal[9*j + 8]};

          // get contact force/torque, rotate into node frame
          sim_math_rotVecQuat(normal, normal, m->mesh_quat + 4 * mesh_id);
          sim_math_rotVecQuat(tang1, tang1, m->mesh_quat + 4 * mesh_id);
          sim_math_rotVecQuat(tang2, tang2, m->mesh_quat + 4 * mesh_id);
          sim_scalar_t force[3];
          sim_scalar_t kMaxDepth = 0.05;
          sim_scalar_t pressure = depth / sim_math_max(kMaxDepth - depth, SIM_MINVAL);
          sim_math_scale_3(force, normal, pressure);

          // one row of mat^T * force
          forcesT[0*ncon + node] = sim_math_dot_3(force, normal);
          forcesT[1*ncon + node] = sim_math_abs(sim_math_dot_3(vel_rel, tang1));
          forcesT[2*ncon + node] = sim_math_abs(sim_math_dot_3(vel_rel, tang2));
          node++;
        }
      }

      // compute sensor output
      for (int c = 0; c < nchannel; c++) {
        if (!sim_math_isZero(forcesT + c*ncon, ncon)) {
          sim_math_addTo(sensordata + c*ncon, forcesT + c*ncon, ncon);
        }
      }

      sim_freeStack(d);
    }
    break;

  case SIM_SENS_ACCELEROMETER:                          // accelerometer
    // tmp = site acceleration, in site frame
    sim_objectAcceleration(m, d, SIM_OBJ_SITE, objid, tmp, 1);

    // assign linear acceleration
    sim_math_copy_3(sensordata, tmp+3);
    break;

  case SIM_SENS_FORCE:                                  // force
    // extract body data
    bodyid = m->site_bodyid[objid];
    rootid = m->body_rootid[bodyid];

    // tmp = interaction force between body and parent, in site frame
    sim_math_transformSpatial(tmp, d->cfrc_int+6*bodyid, 1,
                         d->site_xpos+3*objid, d->subtree_com+3*rootid, d->site_xmat+9*objid);

    // assign force
    sim_math_copy_3(sensordata, tmp+3);
    break;

  case SIM_SENS_TORQUE:                                 // torque
    // extract body data
    bodyid = m->site_bodyid[objid];
    rootid = m->body_rootid[bodyid];

    // tmp = interaction force between body and parent, in site frame
    sim_math_transformSpatial(tmp, d->cfrc_int+6*bodyid, 1,
                         d->site_xpos+3*objid, d->subtree_com+3*rootid, d->site_xmat+9*objid);

    // assign torque
    sim_math_copy_3(sensordata, tmp);
    break;

  case SIM_SENS_ACTUATORFRC:                            // actuator force
    sensordata[0] = d->actuator_force[objid];
    break;

  case SIM_SENS_JOINTACTFRC:                            // actuator force at joint
    sensordata[0] = d->qfrc_actuator[m->jnt_dofadr[objid]];
    break;

  case SIM_SENS_TENDONACTFRC:                           // actuator force at tendon
    frc = 0.0;
    for (int j=0; j < nu; j++) {
      if (m->actuator_trntype[j] == SIM_TRN_TENDON && m->actuator_trnid[2*j] == objid) {
        frc += d->actuator_force[j];
      }
    }
    sensordata[0] = frc;
    break;

  case SIM_SENS_JOINTLIMITFRC:                          // joint limit force
    sensordata[0] = 0;
    for (int j=ne+nf; j < nefc; j++) {
      if (d->efc_type[j] == SIM_CNSTR_LIMIT_JOINT && d->efc_id[j] == objid) {
        sensordata[0] = d->efc_force[j];
        break;
      }
    }
    break;

  case SIM_SENS_TENDONLIMITFRC:                         // tendon limit force
    sensordata[0] = 0;
    for (int j=ne+nf; j < nefc; j++) {
      if (d->efc_type[j] == SIM_CNSTR_LIMIT_TENDON && d->efc_id[j] == objid) {
        sensordata[0] = d->efc_force[j];
        break;
      }
    }
    break;

  case SIM_SENS_FRAMELINACC:                            // 3D linear acceleration
  case SIM_SENS_FRAMEANGACC:                            // 3D angular acceleration
    // get 6D object acceleration, in global frame
    sim_objectAcceleration(m, d, objtype, objid, tmp, 0);

    // copy linear or angular component
    if (type == SIM_SENS_FRAMELINACC) {
      sim_math_copy_3(sensordata, tmp+3);
    } else {
      sim_math_copy_3(sensordata, tmp);
    }
    break;

  default:
    SIM_ERROR("invalid type in ACC stage, sensor %d", i);
  }
}


// compute value for one sensor, write to sensordata, apply cutoff
void sim_computeSensor(const sim_model_t* m, sim_data_t* d, int i, sim_scalar_t* sensordata) {
  switch (m->sensor_needstage[i]) {
  case SIM_STAGE_POS:
    sim_computeSensorPos(m, d, i, sensordata);
    break;

  case SIM_STAGE_VEL:
    sim_computeSensorVel(m, d, i, sensordata);
    break;

  case SIM_STAGE_ACC:
    sim_computeSensorAcc(m, d, i, sensordata);
    break;

  default:
    SIM_ERROR("invalid sensor stage for sensor %d", i);
  }

  // apply cutoff
  apply_cutoff(m, i, sensordata);
}


// compute sensor or read from history buffer (handles delay and interval logic)
static void compute_or_read_sensor(const sim_model_t* m, sim_data_t* d, int i, sim_scalar_t* sensordata) {
  int nsample = m->sensor_history[2*i];

  // no history: compute directly
  if (nsample <= 0) {
    sim_computeSensor(m, d, i, sensordata);
    return;
  }

  sim_scalar_t delay = m->sensor_delay[i];
  int dim = m->sensor_dim[i];

  // delay > 0: read delayed value from buffer
  if (delay > 0) {
    int interp = m->sensor_history[2*i+1];
    const sim_scalar_t* ptr = sim_readSensor(m, d, i, d->time, sensordata, interp);
    if (ptr) sim_math_copy(sensordata, ptr, dim);
    return;
  }

  // interval > 0: compute if interval condition satisfied, else read from buffer
  sim_scalar_t interval = m->sensor_interval[2*i];
  if (interval > 0) {
    int historyadr = m->sensor_historyadr[i];
    sim_scalar_t* buf = d->history + historyadr;
    sim_scalar_t time_prev = buf[0];  // first slot stores time_prev

    if (time_prev + interval <= d->time) {
      // interval condition satisfied: compute new sensor value
      sim_computeSensor(m, d, i, sensordata);
    } else {
      // interval condition not satisfied: read from buffer
      int interp = m->sensor_history[2*i+1];
      const sim_scalar_t* ptr = sim_readSensor(m, d, i, d->time, sensordata, interp);
      if (ptr) sim_math_copy(sensordata, ptr, dim);
    }
    return;
  }

  // history only, no delay or interval: compute directly
  sim_computeSensor(m, d, i, sensordata);
}


// compute user sensors: call user callback and apply cutoff
static void compute_user_sensors(const sim_model_t* m, sim_data_t* d, SIM_tStage stage) {
  if (SIM_cb_sensor) {
    SIM_cb_sensor(m, d, stage);
  }

  // apply cutoff to user sensors
  for (int i=0; i < m->nsensor; i++) {
    if (m->sensor_type[i] == SIM_SENS_USER && m->sensor_needstage[i] == stage) {
      apply_cutoff(m, i, d->sensordata + m->sensor_adr[i]);
    }
  }
}


// compute plugin sensors: call plugin compute and apply cutoff
static void compute_plugin_sensors(const sim_model_t* m, sim_data_t* d, SIM_tStage stage) {
  if (!m->nplugin) {
    return;
  }

  const int nslot = sim_plugin_pluginCount();
  for (int i=0; i < m->nplugin; i++) {
    const int slot = m->plugin[i];
    const SIM_pPlugin* plugin = sim_plugin_getPluginAtSlotUnsafe(slot, nslot);
    if (!plugin) {
      SIM_ERROR("invalid plugin slot: %d", slot);
    }

    // check if plugin is a sensor plugin matching this stage
    if (!(plugin->capabilityflags & SIM_PLUGIN_SENSOR)) {
      continue;
    }
    // match if needstage equals stage, OR stage is POS and needstage is NONE
    int matches_stage = (plugin->needstage == stage) ||
                        (stage == SIM_STAGE_POS && plugin->needstage == SIM_STAGE_NONE);
    if (!matches_stage) {
      continue;
    }

    if (!plugin->compute) {
      SIM_ERROR("`compute` is a null function pointer for plugin at slot %d", slot);
    }

    // call stage-specific preparation if needed
    // TODO(b/247107630): add a flag to allow plugin to specify whether it actually needs this
    if (stage == SIM_STAGE_VEL && !d->flg_subtreevel) {
      sim_subtreeVel(m, d);
    } else if (stage == SIM_STAGE_ACC && !d->flg_rnepost) {
      sim_rnePostConstraint(m, d);
    }

    plugin->compute(m, d, i, SIM_PLUGIN_SENSOR);

    // apply cutoff to all sensors attached to this plugin
    for (int j=0; j < m->nsensor; j++) {
      if (m->sensor_type[j] == SIM_SENS_PLUGIN &&
          m->sensor_plugin[j] == i &&
          m->sensor_needstage[j] == stage) {
        apply_cutoff(m, j, d->sensordata + m->sensor_adr[j]);
      }
    }
  }
}


// position-dependent sensors
void sim_sensorPos(const sim_model_t* m, sim_data_t* d) {
  int nsensor = m->nsensor;
  int nusersensor = 0;

  // disabled sensors: return
  if (SIM_DISABLED(SIM_DSBL_SENSOR)) {
    return;
  }

  // sleep filtering
  int sleep_filter = SIM_ENABLED(SIM_ENBL_SLEEP) && d->nbody_awake < m->nbody;

  // process sensors matching stage
  for (int i=0; i < nsensor; i++) {
    SIM_tSensor type = (SIM_tSensor) m->sensor_type[i];

    // skip sleeping sensor
    if (sleep_filter && sim_sleepState(m, d, SIM_OBJ_SENSOR, i) == sim_spec_ASLEEP) {
      continue;
    }

    // skip sensor plugins -- these are handled after builtin sensor types
    if (type == SIM_SENS_PLUGIN) {
      continue;
    }

    if (m->sensor_needstage[i] == SIM_STAGE_POS) {
      int adr = m->sensor_adr[i];
      sim_scalar_t* sensordata = d->sensordata + adr;

      if (type == SIM_SENS_USER) {
        // clear result, compute later
        sim_math_zero(sensordata, m->sensor_dim[i]);
        nusersensor++;
      } else {
        compute_or_read_sensor(m, d, i, sensordata);
      }
    }
  }

  // fill in user sensors if detected
  if (nusersensor) {
    compute_user_sensors(m, d, SIM_STAGE_POS);
  }

  // compute plugin sensor values
  compute_plugin_sensors(m, d, SIM_STAGE_POS);
}


// velocity-dependent sensors
void sim_sensorVel(const sim_model_t* m, sim_data_t* d) {
  int nusersensor = 0;

  // disabled sensors: return
  if (SIM_DISABLED(SIM_DSBL_SENSOR)) {
    return;
  }

  // sleep filtering
  int sleep_filter = SIM_ENABLED(SIM_ENBL_SLEEP) && d->nbody_awake < m->nbody;

  // process sensors matching stage
  for (int i=0; i < m->nsensor; i++) {
    // skip sensor plugins -- these are handled after builtin sensor types
    if (m->sensor_type[i] == SIM_SENS_PLUGIN) {
      continue;
    }

    // skip sleeping sensor
    if (sleep_filter && sim_sleepState(m, d, SIM_OBJ_SENSOR, i) == sim_spec_ASLEEP) {
      continue;
    }

    if (m->sensor_needstage[i] == SIM_STAGE_VEL) {
      SIM_tSensor type = m->sensor_type[i];
      int adr = m->sensor_adr[i];
      sim_scalar_t* sensordata = d->sensordata + adr;

      if (type == SIM_SENS_USER) {
        // call sim_subtreeVel for user sensors
        if (!d->flg_subtreevel) {
          sim_subtreeVel(m, d);
        }

        // clear result, compute later
        sim_math_zero(sensordata, m->sensor_dim[i]);
        nusersensor++;
      } else {
        compute_or_read_sensor(m, d, i, sensordata);
      }
    }
  }

  // fill in user sensors if detected
  if (nusersensor) {
    compute_user_sensors(m, d, SIM_STAGE_VEL);
  }

  // trigger computation of plugins
  compute_plugin_sensors(m, d, SIM_STAGE_VEL);
}


// acceleration/force-dependent sensors
void sim_sensorAcc(const sim_model_t* m, sim_data_t* d) {
  int nusersensor = 0;

  // disabled sensors: return
  if (SIM_DISABLED(SIM_DSBL_SENSOR)) {
    return;
  }

  // sleep filtering
  int sleep_filter = SIM_ENABLED(SIM_ENBL_SLEEP) && d->nbody_awake < m->nbody;

  // process sensors matching stage
  for (int i=0; i < m->nsensor; i++) {
    // skip sleeping sensor
    if (sleep_filter && sim_sleepState(m, d, SIM_OBJ_SENSOR, i) == sim_spec_ASLEEP) {
      continue;
    }

    // skip sensor plugins -- these are handled after builtin sensor types
    if (m->sensor_type[i] == SIM_SENS_PLUGIN) {
      continue;
    }

    if (m->sensor_needstage[i] == SIM_STAGE_ACC) {
      SIM_tSensor type = m->sensor_type[i];
      int adr = m->sensor_adr[i];
      sim_scalar_t* sensordata = d->sensordata + adr;

      if (type == SIM_SENS_USER) {
        // call sim_rnePostConstraint for user sensors
        if (!d->flg_rnepost) {
          sim_rnePostConstraint(m, d);
        }

        // clear result, compute later
        sim_math_zero(sensordata, m->sensor_dim[i]);
        nusersensor++;
      } else {
        compute_or_read_sensor(m, d, i, sensordata);
      }
    }
  }

  // fill in user sensors if detected
  if (nusersensor) {
    compute_user_sensors(m, d, SIM_STAGE_ACC);
  }

  // trigger computation of plugins
  compute_plugin_sensors(m, d, SIM_STAGE_ACC);
}


//-------------------------------- energy ----------------------------------------------------------

// position-dependent energy (potential)
void sim_energyPos(const sim_model_t* m, sim_data_t* d) {
  int padr;
  sim_scalar_t dif[3], quat[4], stiffness;

  // init potential energy:  -sum_i body(i).mass * sim_math_dot(body(i).pos, gravity)
  d->energy[0] = 0;
  if (!SIM_DISABLED(SIM_DSBL_GRAVITY)) {
    for (int i=1; i < m->nbody; i++) {
      d->energy[0] -= m->body_mass[i] * sim_math_dot_3(m->opt.gravity, d->xipos+3*i);
    }
  }

  int sleep_filter = SIM_ENABLED(SIM_ENBL_SLEEP) && d->nbody_awake < m->nbody;

  // add joint-level springs
  if (!SIM_DISABLED(SIM_DSBL_SPRING)) {
    int nbody = m->nbody;
    for (int b=1; b < nbody; b++) {
      if (sleep_filter && d->body_awake[b] != sim_spec_AWAKE) continue;

      int jnt_start = m->body_jntadr[b];
      int jnt_end = jnt_start + m->body_jntnum[b];
      for (int j=jnt_start; j < jnt_end; j++) {
        stiffness = m->jnt_stiffness[j];
        if (stiffness == 0) {
          continue;
        }
        padr = m->jnt_qposadr[j];

        switch ((SIM_tJoint) m->jnt_type[j]) {
        case SIM_JNT_FREE:
          sim_math_sub_3(dif, d->qpos+padr, m->qpos_spring+padr);
          d->energy[0] += 0.5 * stiffness * sim_math_dot_3(dif, dif);

          // continue with rotations
          padr += 3;
          SIM_FALLTHROUGH;

        case SIM_JNT_BALL:
          // convert quaternion difference into angular "velocity"
          sim_math_copy4(quat, d->qpos+padr);
          sim_math_normalize4(quat);
          sim_math_subQuat(dif, d->qpos + padr, m->qpos_spring + padr);
          d->energy[0] += 0.5 * stiffness * sim_math_dot_3(dif, dif);
          break;

        case SIM_JNT_SLIDE:
        case SIM_JNT_HINGE:
          d->energy[0] += 0.5 * stiffness *
                          (d->qpos[padr] - m->qpos_spring[padr]) *
                          (d->qpos[padr] - m->qpos_spring[padr]);
          break;
        }
      }
    }
  }

  // add tendon-level springs
  if (!SIM_DISABLED(SIM_DSBL_SPRING)) {
    for (int i=0; i < m->ntendon; i++) {
    // skip sleeping or static tendon
    if (sleep_filter && sim_sleepState(m, d, SIM_OBJ_TENDON, i) != sim_spec_AWAKE) {
      continue;
    }

      stiffness = m->tendon_stiffness[i];
      sim_scalar_t length = d->ten_length[i];
      sim_scalar_t displacement = 0;

      // compute spring displacement
      sim_scalar_t lower = m->tendon_lengthspring[2*i];
      sim_scalar_t upper = m->tendon_lengthspring[2*i+1];
      if (length > upper) {
        displacement = upper - length;
      } else if (length < lower) {
        displacement = lower - length;
      }

      d->energy[0] += 0.5*stiffness*displacement*displacement;
    }
  }

  // add flex-level springs for dim=1
  if (!SIM_DISABLED(SIM_DSBL_SPRING)) {
    for (int i=0; i < m->nflex; i++) {
      stiffness = m->flex_edgestiffness[i];
      if (m->flex_rigid[i] || stiffness == 0 || m->flex_dim[i] > 1) {
        continue;
      }

      // process non-rigid edges of this flex
      int flex_edgeadr = m->flex_edgeadr[i];
      int flex_edgenum = m->flex_edgenum[i];
      for (int e=flex_edgeadr; e < flex_edgeadr+flex_edgenum; e++) {
        if (!m->flexedge_rigid[e]) {
          sim_scalar_t displacement = m->flexedge_length0[e] - d->flexedge_length[e];
          d->energy[0] += 0.5*stiffness*displacement*displacement;
        };
      }
    }
  }

  // mark as computed
  d->flg_energypos = 1;
}


// velocity-dependent energy (kinetic)
void sim_energyVel(const sim_model_t* m, sim_data_t* d) {
  sim_markStack(d);
  sim_scalar_t *vec = SIM_STACK_ALLOC(d, m->nv, sim_scalar_t);

  // kinetic energy:  0.5 * qvel' * M * qvel
  sim_mulM(m, d, vec, d->qvel);
  d->energy[1] = 0.5*sim_math_dot(vec, d->qvel, m->nv);

  sim_freeStack(d);

  // mark as computed
  d->flg_energyvel = 1;
}
