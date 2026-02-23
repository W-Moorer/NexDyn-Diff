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

#include "engine/engine_core_smooth.h"

#include <stddef.h>

#include <simcore/SIM_data.h>
#include <simcore/SIM_macro.h>
#include <simcore/SIM_model.h>
#include <simcore/SIM_san.h>  // IWYU pragma: keep
#include "engine/engine_core_constraint.h"
#include "engine/engine_core_util.h"
#include "engine/engine_crossplatform.h"
#include "engine/engine_inline.h"
#include "engine/engine_macro.h"
#include "engine/engine_memory.h"
#include "engine/engine_sleep.h"
#include "engine/engine_util_blas.h"
#include "engine/engine_util_errmem.h"
#include "engine/engine_util_misc.h"
#include "engine/engine_util_sparse.h"
#include "engine/engine_util_spatial.h"


//--------------------------- position -------------------------------------------------------------

// forward kinematics part 1: bodies
void sim_kinematics1(const sim_model_t* m, sim_data_t* d) {
  int nbody = m->nbody;

  // set world position and orientation
  sim_math_zero_3(d->xpos);
  sim_math_unit4(d->xquat);
  sim_math_zero_3(d->xipos);
  sim_math_zero(d->xmat, 9);
  sim_math_zero(d->ximat, 9);
  d->xmat[0] = d->xmat[4] = d->xmat[8] = 1;
  d->ximat[0] = d->ximat[4] = d->ximat[8] = 1;

  int sleep_filter = SIM_ENABLED(SIM_ENBL_SLEEP);

  // compute global cartesian positions and orientations of all bodies
  for (int i=1; i < nbody; i++) {
    // skip static bodies
    if (sleep_filter) {
      if (d->body_awake[i] == sim_spec_STATIC) continue;
    }

    sim_scalar_t xpos[3], xquat[4];
    int jntadr = m->body_jntadr[i];
    int jntnum = m->body_jntnum[i];

    // free joint
    if (jntnum == 1 && m->jnt_type[jntadr] == SIM_JNT_FREE) {
      // get qpos address
      int qadr = m->jnt_qposadr[jntadr];

      // copy pos and quat from qpos
      sim_math_internal_copy_3(xpos, d->qpos+qadr);
      sim_math_internal_copy4(xquat, d->qpos+qadr+3);
      sim_math_normalize4(xquat);

      // assign xanchor and xaxis
      sim_math_internal_copy_3(d->xanchor+3*jntadr, xpos);
      sim_math_internal_copy_3(d->xaxis+3*jntadr, m->jnt_axis+3*jntadr);
    }

    // regular or no joint
    else {
      int pid = m->body_parentid[i];

      // get body pos and quat: from model or mocap
      sim_scalar_t *bodypos, *bodyquat, quat[4];
      if (m->body_mocapid[i] >= 0) {
        bodypos = d->mocap_pos + 3*m->body_mocapid[i];
        sim_math_internal_copy4(quat, d->mocap_quat + 4*m->body_mocapid[i]);
        sim_math_normalize4(quat);
        bodyquat = quat;
      } else {
        bodypos = m->body_pos+3*i;
        bodyquat = m->body_quat+4*i;
      }

      // apply fixed translation and rotation relative to parent
      if (pid) {
        sim_math_internal_mulMatVec3(xpos, d->xmat+9*pid, bodypos);
        sim_math_internal_addTo3(xpos, d->xpos+3*pid);
        sim_math_internal_mulQuat(xquat, d->xquat+4*pid, bodyquat);
      } else {
        // parent is the world
        sim_math_internal_copy_3(xpos, bodypos);
        sim_math_internal_copy4(xquat, bodyquat);
      }

      // accumulate joints, compute xpos and xquat for this body
      sim_scalar_t xanchor[3], xaxis[3];
      for (int j=0; j < jntnum; j++) {
        // get joint id, qpos address, joint type
        int jid = jntadr + j;
        int qadr = m->jnt_qposadr[jid];
        SIM_tJoint jtype = m->jnt_type[jid];

        // compute axis in global frame; ball jnt_axis is (0,0,1), set by compiler
        sim_math_internal_rotVecQuat(xaxis, m->jnt_axis+3*jid, xquat);

        // compute anchor in global frame
        sim_math_internal_rotVecQuat(xanchor, m->jnt_pos+3*jid, xquat);
        sim_math_internal_addTo3(xanchor, xpos);

        // apply joint transformation
        switch (jtype) {
        case SIM_JNT_SLIDE:
          sim_math_internal_addToScl3(xpos, xaxis, d->qpos[qadr] - m->qpos0[qadr]);
          break;

        case SIM_JNT_BALL:
        case SIM_JNT_HINGE:
          {
            // compute local quaternion rotation
            sim_scalar_t qloc[4];
            if (jtype == SIM_JNT_BALL) {
              sim_math_internal_copy4(qloc, d->qpos+qadr);
              sim_math_normalize4(qloc);
            } else {
              sim_math_internal_axisAngle2Quat(qloc, m->jnt_axis+3*jid, d->qpos[qadr] - m->qpos0[qadr]);
            }

            // apply rotation
            sim_math_mulQuat(xquat, xquat, qloc);

            // correct for off-center rotation
            sim_scalar_t vec[3];
            sim_math_internal_rotVecQuat(vec, m->jnt_pos+3*jid, xquat);
            sim_math_internal_sub3(xpos, xanchor, vec);
          }
          break;

        default:
          SIM_ERROR("unknown joint type %d", jtype);  // SHOULD NOT OCCUR
        }

        // assign xanchor and xaxis
        sim_math_internal_copy_3(d->xanchor+3*jid, xanchor);
        sim_math_internal_copy_3(d->xaxis+3*jid, xaxis);
      }
    }

    // normalize quaternion
    sim_math_normalize4(xquat);

    // sleeping body, check for mismatch
    if (sleep_filter && jntnum && d->body_awake[i] == sim_spec_ASLEEP) {
      // compare new and existing xpos and xquat
      const sim_scalar_t* pos = d->xpos+3*i;
      const sim_scalar_t* xq = d->xquat+4*i;
      int match = xpos[0] == pos[0] && xpos[1] == pos[1] && xpos[2] == pos[2] &&
                  xquat[0] == xq[0] && xquat[1] == xq[1] && xquat[2] == xq[2] && xquat[3] == xq[3];

      // match: continue to next body
      if (match) {
        continue;
      }

      // mismatch: mark the tree for waking later (in sim_wake)
      else {
        d->tree_awake[m->body_treeid[i]] = 1;
      }
    }

    // assign xquat and xpos, construct xmat
    sim_math_internal_copy4(d->xquat+4*i, xquat);
    sim_math_internal_copy_3(d->xpos+3*i, xpos);
    sim_math_quat2Mat(d->xmat+9*i, xquat);
  }
}


// forward kinematics part 2: body inertias, geoms and sites
void sim_kinematics2(const sim_model_t* m, sim_data_t* d) {
  int sleep_filter = SIM_ENABLED(SIM_ENBL_SLEEP) && d->nbody_awake < m->nbody;
  int nbody = sleep_filter ? d->nbody_awake : m->nbody;

  // compute/copy Cartesian positions and orientations of body inertial frames
  for (int b=1; b < nbody; b++) {
    int i = sleep_filter ? d->body_awake_ind[b] : b;

    sim_local2Global(d, d->xipos+3*i, d->ximat+9*i,
                    m->body_ipos+3*i, m->body_iquat+4*i,
                    i, m->body_sameframe[i]);
  }

  // compute/copy Cartesian positions and orientations of geoms
  for (int b=0; b < nbody; b++) {
    int i = sleep_filter ? d->body_awake_ind[b] : b;

    // skip geom in sleeping or static body
    if (sleep_filter && d->body_awake[i] != sim_spec_AWAKE) continue;

    int start = m->body_geomadr[i];
    int end = start + m->body_geomnum[i];
    for (int g=start; g < end; g++) {
      sim_local2Global(d, d->geom_xpos+3*g, d->geom_xmat+9*g,
                      m->geom_pos+3*g, m->geom_quat+4*g,
                      m->geom_bodyid[g], m->geom_sameframe[g]);
    }
  }

  // compute/copy Cartesian positions and orientations of sites
  int nsite = m->nsite;
  for (int i=0; i < nsite; i++) {
    int bodyid = m->site_bodyid[i];

    // skip site in sleeping or static body
    if (sleep_filter && d->body_awake[bodyid] != sim_spec_AWAKE) continue;

    sim_local2Global(d, d->site_xpos+3*i, d->site_xmat+9*i,
                    m->site_pos+3*i, m->site_quat+4*i,
                    bodyid, m->site_sameframe[i]);
  }
}


// forward kinematics
void sim_kinematics(const sim_model_t* m, sim_data_t* d) {
  sim_kinematics1(m, d);
  if (sim_wake(m, d)) {
    sim_updateSleep(m, d);
  }
  sim_kinematics2(m, d);
}


// map inertias and motion dofs to global frame centered at subtree-CoM
void sim_comPos(const sim_model_t* m, sim_data_t* d) {
  int sleep_filter = SIM_ENABLED(SIM_ENBL_SLEEP) && d->nbody_awake < m->nbody;
  int nbody = sleep_filter ? d->nbody_awake : m->nbody;
  int nparent = sleep_filter ? d->nparent_awake : m->nbody;

  // subtree_com: initialize with body moment
  for (int b=0; b < nbody; b++) {
    int i = sleep_filter ? d->body_awake_ind[b] : b;

    sim_math_internal_scl3(d->subtree_com+3*i, d->xipos+3*i, m->body_mass[i]);
  }

  // subtree_com: accumulate to parent in backward pass
  for (int b=nparent-1; b >= 0; b--) {
    int i = sleep_filter ? d->parent_awake_ind[b] : b;
    if (!i) continue;

    // accumulate moment to parent, rescale if sleeping
    int parent = m->body_parentid[i];
    if (sleep_filter && d->body_awake[i] == sim_spec_ASLEEP) {
      sim_scalar_t child_moment[3];
      sim_math_internal_scl3(child_moment, d->subtree_com+3*i, m->body_subtreemass[i]);
      sim_math_internal_addTo3(d->subtree_com+3*parent, child_moment);
    } else {
      sim_math_internal_addTo3(d->subtree_com+3*parent, d->subtree_com+3*i);
    }
  }

  // subtree_com: normalize
  for (int b=0; b < nbody; b++) {
    int i = sleep_filter ? d->body_awake_ind[b] : b;

    if (m->body_subtreemass[i] < SIM_MINVAL) {
      sim_math_internal_copy_3(d->subtree_com+3*i, d->xipos+3*i);
    } else {
      sim_math_scale_3(d->subtree_com + 3 * i, d->subtree_com + 3 * i,
               1.0 / m->body_subtreemass[i]);
    }
  }

  // zero out CoM frame inertia for the world body
  sim_math_zero(d->cinert, 10);

  // map inertias to frame centered at subtree_com
  for (int b=1; b < nbody; b++) {
    int i = sleep_filter ? d->body_awake_ind[b] : b;

    sim_scalar_t offset[3];
    sim_math_internal_sub3(offset, d->xipos+3*i, d->subtree_com+3*m->body_rootid[i]);
    sim_math_inertCom(d->cinert+10*i, m->body_inertia+3*i, d->ximat+9*i, offset, m->body_mass[i]);
  }

  // map motion dofs to global frame centered at subtree_com
  for (int b=1; b < nbody; b++) {
    int i = sleep_filter ? d->body_awake_ind[b] : b;

    int jntnum = m->body_jntnum[i];
    if (!jntnum) continue;

    int start = m->body_jntadr[i];
    int end = start + jntnum;
    for (int j=start; j < end; j++) {
      // get cdof address
      int da = 6*m->jnt_dofadr[j];

      // compute com-anchor vector
      sim_scalar_t offset[3], axis[3];
      sim_math_internal_sub3(offset, d->subtree_com+3*m->body_rootid[i], d->xanchor+3*j);

      // create motion dof
      int skip = 0;
      switch ((SIM_tJoint) m->jnt_type[j]) {
      case SIM_JNT_FREE:
        // translation components: x, y, z in global frame
        sim_math_zero(d->cdof+da, 18);
        d->cdof[da+3+7*0] = 1;
        d->cdof[da+3+7*1] = 1;
        d->cdof[da+3+7*2] = 1;

        // rotation components: same as ball
        skip = 18;
        SIM_FALLTHROUGH;

      case SIM_JNT_BALL:
        for (int k=0; k < 3; k++) {
          // I_3 rotation in child frame (assume no subsequent rotations)
          axis[0] = d->xmat[9*i + k + 0];
          axis[1] = d->xmat[9*i + k + 3];
          axis[2] = d->xmat[9*i + k + 6];

          sim_math_dofCom(d->cdof+da+skip+6*k, axis, offset);
        }
        break;

      case SIM_JNT_SLIDE:
        sim_math_dofCom(d->cdof+da, d->xaxis+3*j, 0);
        break;

      case SIM_JNT_HINGE:
        sim_math_dofCom(d->cdof+da, d->xaxis+3*j, offset);
        break;
      }
    }
  }
}


// compute camera and light positions and orientations
void sim_camlight(const sim_model_t* m, sim_data_t* d) {
  int ncam = m->ncam, nlight = m->nlight;
  int sleep_filter = SIM_ENABLED(SIM_ENBL_SLEEP) && d->nbody_awake < m->nbody;

  // compute Cartesian positions and orientations of cameras
  for (int i=0; i < ncam; i++) {
    // get camera body id and target body id
    int id = m->cam_bodyid[i];
    int id1 = m->cam_targetbodyid[i];

    // skip camera if both body and target body are asleep or static
    if (sleep_filter && d->body_awake[id] != sim_spec_AWAKE) {
      if (id1 < 0 || d->body_awake[id1] != sim_spec_AWAKE) {
        continue;
      }
    }

    // default processing for fixed mode
    sim_local2Global(d, d->cam_xpos+3*i, d->cam_xmat+9*i,
                    m->cam_pos+3*i, m->cam_quat+4*i, id, 0);

    // adjust for mode
    switch ((SIM_tCamLight) m->cam_mode[i]) {
    case SIM_CAMLIGHT_FIXED:
      break;
    case SIM_CAMLIGHT_TRACK:
    case SIM_CAMLIGHT_TRACKCOM:
      // fixed global orientation
      sim_math_internal_copy9(d->cam_xmat+9*i, m->cam_mat0+9*i);

      // position: track camera body
      if (m->cam_mode[i] == SIM_CAMLIGHT_TRACK) {
        sim_math_internal_add3(d->cam_xpos+3*i, d->xpos+3*id, m->cam_pos0+3*i);
      }

      // position: track subtree com
      else {
        sim_math_internal_add3(d->cam_xpos+3*i, d->subtree_com+3*id, m->cam_poscom0+3*i);
      }
      break;

    case SIM_CAMLIGHT_TARGETBODY:
    case SIM_CAMLIGHT_TARGETBODYCOM:
      // only if target body is specified
      if (id1 >= 0) {
        sim_scalar_t pos[3];
        // get position to look at
        if (m->cam_mode[i] == SIM_CAMLIGHT_TARGETBODY) {
          sim_math_internal_copy_3(pos, d->xpos+3*id1);
        } else {
          sim_math_internal_copy_3(pos, d->subtree_com+3*id1);
        }

        // zaxis = -desired camera direction, in global frame
        sim_scalar_t matT[9];
        sim_math_internal_sub3(matT+6, d->cam_xpos+3*i, pos);
        sim_math_normalize_3(matT+6);

        // xaxis: orthogonal to zaxis and to (0,0,1)
        matT[3] = 0;
        matT[4] = 0;
        matT[5] = 1;
        sim_math_internal_cross(matT, matT+3, matT+6);
        sim_math_normalize_3(matT);

        // yaxis: orthogonal to xaxis and zaxis
        sim_math_internal_cross(matT+3, matT+6, matT);
        sim_math_normalize_3(matT+3);

        // set camera frame
        sim_math_transpose(d->cam_xmat+9*i, matT, 3, 3);
      }
    }
  }

  // compute Cartesian positions and directions of lights
  for (int i=0; i < nlight; i++) {
    // get light body id and target body id
    int id = m->light_bodyid[i];
    int id1 = m->light_targetbodyid[i];

    // skip light if both body and target body are asleep or static
    if (sleep_filter && d->body_awake[id] != sim_spec_AWAKE) {
      if (id1 < 0 || d->body_awake[id1] != sim_spec_AWAKE) {
        continue;
      }
    }

    // default processing for fixed mode
    sim_local2Global(d, d->light_xpos+3*i, 0, m->light_pos+3*i, 0, id, 0);
    sim_math_internal_rotVecQuat(d->light_xdir+3*i, m->light_dir+3*i, d->xquat+4*id);

    // adjust for mode
    switch ((SIM_tCamLight) m->light_mode[i]) {
    case SIM_CAMLIGHT_FIXED:
      break;
    case SIM_CAMLIGHT_TRACK:
    case SIM_CAMLIGHT_TRACKCOM:
      // fixed global orientation
      sim_math_internal_copy_3(d->light_xdir+3*i, m->light_dir0+3*i);

      // position: track light body
      if (m->light_mode[i] == SIM_CAMLIGHT_TRACK) {
        sim_math_internal_add3(d->light_xpos+3*i, d->xpos+3*id, m->light_pos0+3*i);
      }

      // position: track subtree com
      else {
        sim_math_internal_add3(d->light_xpos+3*i, d->subtree_com+3*id, m->light_poscom0+3*i);
      }
      break;

    case SIM_CAMLIGHT_TARGETBODY:
    case SIM_CAMLIGHT_TARGETBODYCOM:
      // only if target body is specified
      if (id1 >= 0) {
        // get position to look at
        sim_scalar_t lookat[3];
        if (m->light_mode[i] == SIM_CAMLIGHT_TARGETBODY) {
          sim_math_internal_copy_3(lookat, d->xpos+3*id1);
        } else {
          sim_math_internal_copy_3(lookat, d->subtree_com+3*id1);
        }

        // set dir
        sim_math_internal_sub3(d->light_xdir+3*i, lookat, d->light_xpos+3*i);
      }
    }

    // normalize dir
    sim_math_normalize_3(d->light_xdir+3*i);
  }
}


// update dynamic BVH; leaf aabbs must be updated before call
void sim_updateDynamicBVH(const sim_model_t* m, sim_data_t* d, int bvhadr, int bvhnum) {
  sim_markStack(d);
  int* modified = SIM_STACK_ALLOC(d, bvhnum, int);
  sim_math_zeroInt(modified, bvhnum);

  // mark leafs as modified
  for (int i=0; i < bvhnum; i++) {
    if (m->bvh_nodeid[bvhadr+i] >= 0) {
      modified[i] = 1;
    }
  }

  // update non-leafs in backward pass (parents come before children)
  for (int i=bvhnum-1; i >= 0; i--) {
    if (m->bvh_nodeid[bvhadr+i] < 0) {
      int child1 = m->bvh_child[2*(bvhadr+i)];
      int child2 = m->bvh_child[2*(bvhadr+i)+1];

      // update if either child is modified
      if (modified[child1] || modified[child2]) {
        sim_scalar_t* aabb = d->bvh_aabb_dyn + 6*(bvhadr - m->nbvhstatic + i);
        const sim_scalar_t* aabb1 = d->bvh_aabb_dyn + 6*(bvhadr - m->nbvhstatic + child1);
        const sim_scalar_t* aabb2 = d->bvh_aabb_dyn + 6*(bvhadr - m->nbvhstatic + child2);

        // compute new (min, max)
        sim_scalar_t xmin[3], xmax[3];
        for (int k=0; k < 3; k++) {
          xmin[k] = sim_math_min(aabb1[k] - aabb1[k+3], aabb2[k] - aabb2[k+3]);
          xmax[k] = sim_math_max(aabb1[k] + aabb1[k+3], aabb2[k] + aabb2[k+3]);
        }

        // convert to (center, size)
        for (int k=0; k < 3; k++) {
          aabb[k]   = 0.5*(xmax[k]+xmin[k]);
          aabb[k+3] = 0.5*(xmax[k]-xmin[k]);
        }

        modified[i] = 1;
      }
    }
  }

  sim_freeStack(d);
}


// C(3x2) = A(3x2) * B(2x2)
static inline void sim_math_mulMatMat322(sim_scalar_t* C, const sim_scalar_t* A, const sim_scalar_t* B) {
  C[0] = A[0]*B[0] + A[1]*B[2];
  C[1] = A[0]*B[1] + A[1]*B[3];
  C[2] = A[2]*B[0] + A[3]*B[2];
  C[3] = A[2]*B[1] + A[3]*B[3];
  C[4] = A[4]*B[0] + A[5]*B[2];
  C[5] = A[4]*B[1] + A[5]*B[3];
}


// compute flex-related quantities
void sim_flex(const sim_model_t* m, sim_data_t* d) {
  int nv = m->nv;
  int* rowadr = m->flexedge_J_rowadr;
  int* vrowadr = m->flexvert_J_rowadr, *vrownnz = m->flexvert_J_rownnz;

  // skip if no flexes
  if (!m->nflex) {
    return;
  }

  // compute Cartesian positions of flex vertices
  for (int f=0; f < m->nflex; f++) {
    int vstart = m->flex_vertadr[f];
    int vend = m->flex_vertadr[f] + m->flex_vertnum[f];
    int nstart = m->flex_nodeadr[f];
    int nend = m->flex_nodeadr[f] + m->flex_nodenum[f];

    // 0: vertices are the mesh vertices, 1: vertices are interpolated from nodal dofs
    if (m->flex_interp[f] == 0) {
      // centered: copy body position
      if (m->flex_centered[f]) {
        for (int i=vstart; i < vend; i++) {
          sim_math_internal_copy_3(d->flexvert_xpos+3*i, d->xpos+3*m->flex_vertbodyid[i]);
        }
      }

      // non-centered: map from local to global
      else {
        for (int i=vstart; i < vend; i++) {
          sim_math_internal_mulMatVec3(d->flexvert_xpos+3*i, d->xmat+9*m->flex_vertbodyid[i], m->flex_vert+3*i);
          sim_math_internal_addTo3(d->flexvert_xpos+3*i, d->xpos+3*m->flex_vertbodyid[i]);
        }
      }
    }

    // trilinear interpolation
    else {
      sim_scalar_t nodexpos[3*SIM_MAXFLEXNODES];
      if (m->flex_centered[f]) {
        for (int i=nstart; i < nend; i++) {
          sim_math_internal_copy_3(nodexpos + 3*(i-nstart), d->xpos + 3*m->flex_nodebodyid[i]);
        }
      } else {
        for (int i=nstart; i < nend; i++) {
          int j = i - nstart;
          sim_math_internal_mulMatVec3(nodexpos + 3*j, d->xmat + 9*m->flex_nodebodyid[i], m->flex_node + 3*i);
          sim_math_internal_addTo3(nodexpos + 3*j, d->xpos + 3*m->flex_nodebodyid[i]);
        }
      }

      int order = m->flex_interp[f];
      if (nend - nstart != (order + 1) * (order + 1) * (order + 1)) {
        SIM_ERROR("flex_interp_order mismatch");
      }

      for (int i=vstart; i < vend; i++) {
        sim_math_zero_3(d->flexvert_xpos+3*i);
        sim_math_interpolate3D(d->flexvert_xpos+3*i, m->flex_vert0 + 3*i, nodexpos, order);
      }
    }
  }

  // compute flex element aabb
  for (int f=0; f < m->nflex; f++) {
    int dim = m->flex_dim[f];

    // process elements of this flex
    int elemnum = m->flex_elemnum[f];
    for (int e=0; e < elemnum; e++) {
      const int* edata = m->flex_elem + m->flex_elemdataadr[f] + e*(dim+1);
      const sim_scalar_t* vert = d->flexvert_xpos + 3*m->flex_vertadr[f];

      // compute min and max along each global axis
      sim_scalar_t xmin[3], xmax[3];
      sim_math_internal_copy_3(xmin, vert+3*edata[0]);
      sim_math_internal_copy_3(xmax, vert+3*edata[0]);
      for (int i=1; i <= dim; i++) {
        for (int j=0; j < 3; j++) {
          sim_scalar_t value = vert[3*edata[i]+j];
          xmin[j] = sim_math_min(xmin[j], value);
          xmax[j] = sim_math_max(xmax[j], value);
        }
      }

      // compute aabb (center, size)
      int base = m->flex_elemadr[f] + e;
      d->flexelem_aabb[6*base+0] = 0.5*(xmax[0]+xmin[0]);
      d->flexelem_aabb[6*base+1] = 0.5*(xmax[1]+xmin[1]);
      d->flexelem_aabb[6*base+2] = 0.5*(xmax[2]+xmin[2]);
      d->flexelem_aabb[6*base+3] = 0.5*(xmax[0]-xmin[0]) + m->flex_radius[f];
      d->flexelem_aabb[6*base+4] = 0.5*(xmax[1]-xmin[1]) + m->flex_radius[f];
      d->flexelem_aabb[6*base+5] = 0.5*(xmax[2]-xmin[2]) + m->flex_radius[f];
    }
  }

  // update flex bhv_aabb_dyn if needed
  if (!SIM_DISABLED(SIM_DSBL_MIDPHASE)) {
    for (int f=0; f < m->nflex; f++) {
      if (m->flex_bvhadr[f] >= 0) {
        int flex_bvhadr = m->flex_bvhadr[f];
        int flex_bvhnum = m->flex_bvhnum[f];

        // copy element aabbs to bhv leaf aabbs
        for (int i=flex_bvhadr; i < flex_bvhadr+flex_bvhnum; i++) {
          if (m->bvh_nodeid[i] >= 0) {
            sim_math_internal_copy6(d->bvh_aabb_dyn + 6*(i - m->nbvhstatic),
                      d->flexelem_aabb + 6*(m->flex_elemadr[f] + m->bvh_nodeid[i]));
          }
        }

        // update dynamic BVH
        sim_updateDynamicBVH(m, d, m->flex_bvhadr[f], m->flex_bvhnum[f]);
      }
    }
  }

  // allocate space
  sim_markStack(d);
  sim_scalar_t* jac1 = SIM_STACK_ALLOC(d, 3*nv, sim_scalar_t);
  sim_scalar_t* jac2 = SIM_STACK_ALLOC(d, 3*nv, sim_scalar_t);
  sim_scalar_t* jacdif = SIM_STACK_ALLOC(d, 3*nv, sim_scalar_t);
  int* chain = SIM_STACK_ALLOC(d, nv, int);

  // clear Jacobian
  sim_math_zero(d->flexvert_J, 2*m->nJfv);
  sim_math_zero(d->flexedge_J, m->nJfe);

  // compute lengths and Jacobians of edges
  for (int f=0; f < m->nflex; f++) {
    // skip if edges cannot generate forces
    if (m->flex_rigid[f] || m->flex_interp[f]) {
      continue;
    }

    // skip edge Jacobian if no built-in passive force is needed
    int skipjacobian = m->flex_edgeequality[f] != 1 &&
                       !m->flex_edgedamping[f] &&
                       !m->flex_edgestiffness[f] &&
                       !m->flex_damping[f];

    // process edges of this flex
    int vbase = m->flex_vertadr[f];
    int ebase = m->flex_edgeadr[f];
    int edgenum = m->flex_edgenum[f];
    for (int e=0; e < edgenum; e++) {
      int v1 = m->flex_edge[2*(ebase+e)];
      int v2 = m->flex_edge[2*(ebase+e)+1];
      int b1 = m->flex_vertbodyid[vbase+v1];
      int b2 = m->flex_vertbodyid[vbase+v2];
      sim_scalar_t* pos1 = d->flexvert_xpos + 3*(vbase+v1);
      sim_scalar_t* pos2 = d->flexvert_xpos + 3*(vbase+v2);

      // vec = unit vector from v1 to v2, compute edge length
      sim_scalar_t vec[3];
      sim_math_internal_sub3(vec, pos2, pos1);
      d->flexedge_length[ebase+e] = sim_math_normalize_3(vec);

      // skip Jacobian if not needed
      if (skipjacobian) {
        continue;
      }

      // get endpoint Jacobians, subtract
      int NV = sim_jacDifPair(m, d, chain, b1, b2, pos1, pos2,
                              jac1, jac2, jacdif, NULL, NULL, NULL, /*issparse=*/1);

      // no dofs: skip
      if (!NV) {
        continue;
      }

      // apply chain rule to compute edge Jacobian
      sim_math_mulMatTVec(d->flexedge_J + rowadr[ebase+e], jacdif, vec, 3, NV);
    }

    // if dim=2 and constraints are active we use the vertex-based constraint defined in
    // Chen, Kry, and Vouga, "Locking-free Simulation of Isometric Thin Plates", 2019.
    if (m->flex_dim[f] == 2 && m->flex_edgeequality[f] == 2) {
      int nvert = m->flex_vertnum[f];

      // use global vertex adjacency list
      int* v_edge_cnt = m->flex_vertedgenum + vbase;
      int* v_edge_adr = m->flex_vertedgeadr + vbase;
      int* adj_edges = m->flex_vertedge;

      sim_markStack(d);

      // clear Jacobian and assemble vertex by vertex
      int* chain1 = SIM_STACK_ALLOC(d, nv, int);
      int* chain2 = SIM_STACK_ALLOC(d, nv, int);
      sim_scalar_t* J0_dense = SIM_STACK_ALLOC(d, nv, sim_scalar_t);
      sim_scalar_t* J1_dense = SIM_STACK_ALLOC(d, nv, sim_scalar_t);
      sim_math_zero(J0_dense, nv);
      sim_math_zero(J1_dense, nv);

      // temporary buffer for Jacobian accumulation
      sim_scalar_t* J_local = SIM_STACK_ALLOC(d, nv, sim_scalar_t);

      for (int v = 0; v < nvert; v++) {
        sim_scalar_t A[6] = {0};
        int vadr = vbase + v;
        sim_scalar_t* metric = m->flex_vertmetric + 4 * vadr;

        for (int k = 0; k < v_edge_cnt[v]; k++) {
          int e = adj_edges[v_edge_adr[v] + k];

          // compute rest configuration edge vector
          sim_scalar_t dx[3];
          int v1 = m->flex_edge[2 * (ebase + e)];
          int v2 = m->flex_edge[2 * (ebase + e) + 1];
          sim_math_sub_3(dx, m->flex_vert0 + 3 * (vbase + v2), m->flex_vert0 + 3 * (vbase + v1));

          // apply scaling since they are half sizes
          dx[0] *= 2 * m->flex_size[3 * f + 0];
          dx[1] *= 2 * m->flex_size[3 * f + 1];
          dx[2] *= 2 * m->flex_size[3 * f + 2];

          sim_scalar_t dy[3];
          sim_math_sub_3(dy, d->flexvert_xpos + 3 * (vbase + v2), d->flexvert_xpos + 3 * (vbase + v1));

          // get mass of neighbor vertex
          sim_scalar_t weight = 1.0;
          int neighbor_v = (v == v1) ? v2 : v1;
          int b_neighbor = m->flex_vertbodyid[vbase + neighbor_v];
          if (b_neighbor >= 0) {
            weight = m->body_mass[b_neighbor];
            if (weight < SIM_MINVAL) weight = SIM_MINVAL;
          }

          // accumulate A += w * dy * dx'
          A[0] += weight * dy[0] * dx[0];
          A[1] += weight * dy[0] * dx[1];
          A[2] += weight * dy[1] * dx[0];
          A[3] += weight * dy[1] * dx[1];
          A[4] += weight * dy[2] * dx[0];
          A[5] += weight * dy[2] * dx[1];
        }

        sim_scalar_t F[6];
        sim_math_mulMatMat322(F, A, metric);

        // compute Cauchy strain tensor F^T F
        sim_scalar_t cauchy[4];
        cauchy[0] = F[0] * F[0] + F[2] * F[2] + F[4] * F[4];  // c00
        cauchy[1] = F[0] * F[1] + F[2] * F[3] + F[4] * F[5];  // c01
        cauchy[2] = F[1] * F[0] + F[3] * F[2] + F[5] * F[4];  // c10
        cauchy[3] = F[1] * F[1] + F[3] * F[3] + F[5] * F[5];  // c11

        // mass scaling: scale constraint by sqrt(mass) to improve condition number
        // note: departure from original algorithm in Chen, Kry, and Vouga 2019
        sim_scalar_t scale = 1.0;
        int b = m->flex_vertbodyid[vadr];
        if (b >= 0) {
          sim_scalar_t mass = m->body_mass[b];
          if (mass > SIM_MINVAL) {
            scale = sim_math_sqrt(mass);
          }
        }

        // compute tensor invariants
        d->flexvert_length[2 * vadr + 0] = (cauchy[0] + cauchy[3] - 2) * scale;
        d->flexvert_length[2 * vadr + 1] =
            (cauchy[0] * cauchy[3] - cauchy[1] * cauchy[2] - 1) * scale;

        // Jacobian computation
        sim_scalar_t FB[6], adj[4], Fadj[6], FadjBinv[6];
        sim_math_mulMatMat322(FB, F, metric);

        adj[0] = cauchy[3];
        adj[1] = -cauchy[1];
        adj[2] = -cauchy[2];
        adj[3] = cauchy[0];
        sim_math_mulMatMat322(Fadj, F, adj);
        sim_math_mulMatMat322(FadjBinv, Fadj, metric);

        for (int k = 0; k < v_edge_cnt[v]; ++k) {
          int e = adj_edges[v_edge_adr[v] + k];
          sim_scalar_t dx[3];  // rest edge vector
          int v1 = m->flex_edge[2 * (ebase + e)];
          int v2 = m->flex_edge[2 * (ebase + e) + 1];
          sim_math_sub_3(dx, m->flex_vert0 + 3 * (vbase + v2), m->flex_vert0 + 3 * (vbase + v1));
          dx[0] *= 2 * m->flex_size[3 * f + 0];
          dx[1] *= 2 * m->flex_size[3 * f + 1];
          dx[2] *= 2 * m->flex_size[3 * f + 2];
          sim_scalar_t weight = 1.0;
          int neighbor_v = (v == v1) ? v2 : v1;
          int b_neighbor = m->flex_vertbodyid[vbase + neighbor_v];
          if (b_neighbor >= 0) {
            weight = m->body_mass[b_neighbor];
            if (weight < SIM_MINVAL) weight = SIM_MINVAL;
          }

          sim_scalar_t dI1dy1[3], dI1dy2[3], dI2dy[3], dI2dy1[3], dI2dy2[3];

          // dI1/dy1, dI1/dy2 (scaled by weight)
          sim_math_mulMatVec(dI1dy1, FB, dx, 3, 2);
          sim_math_scale_3(dI1dy1, dI1dy1, -2 * weight);
          sim_math_scale_3(dI1dy2, dI1dy1, -1);

          // dI2/dy1, dI2/dy2 (scaled by weight)
          sim_math_mulMatVec(dI2dy, FadjBinv, dx, 3, 2);
          sim_math_scale_3(dI2dy1, dI2dy, -2 * weight);
          sim_math_scale_3(dI2dy2, dI2dy1, -1);

          // get endpoint Jacobians
          int b1 = m->flex_vertbodyid[vbase+v1];
          int b2 = m->flex_vertbodyid[vbase+v2];
          int NV1 = sim_bodyChain(m, b1, chain1);
          sim_jacSparse(m, d, jac1, NULL, d->flexvert_xpos + 3*(vbase+v1), b1, NV1, chain1);
          int NV2 = sim_bodyChain(m, b2, chain2);
          sim_jacSparse(m, d, jac2, NULL, d->flexvert_xpos + 3*(vbase+v2), b2, NV2, chain2);

          // accumulate dense Jacobians for vertex v
          sim_math_mulMatTVec(J_local, jac1, dI1dy1, 3, NV1);
          for (int j=0; j<NV1; j++) {
            J0_dense[chain1[j]] += J_local[j];
          }
          sim_math_mulMatTVec(J_local, jac2, dI1dy2, 3, NV2);
          for (int j=0; j<NV2; j++) {
            J0_dense[chain2[j]] += J_local[j];
          }

          sim_math_mulMatTVec(J_local, jac1, dI2dy1, 3, NV1);
          for (int j=0; j<NV1; j++) {
            J1_dense[chain1[j]] += J_local[j];
          }
          sim_math_mulMatTVec(J_local, jac2, dI2dy2, 3, NV2);
          for (int j=0; j<NV2; j++) {
            J1_dense[chain2[j]] += J_local[j];
          }
        }

        // copy to sparse flexvert_J
        int row0 = 2 * vadr;
        int nnz0 = vrownnz[row0];
        for (int j = 0; j < nnz0; j++) {
          int col = m->flexvert_J_colind[vrowadr[row0] + j];
          d->flexvert_J[vrowadr[row0] + j] += J0_dense[col] * scale;
          J0_dense[col] = 0;
        }
        int row1 = 2 * vadr + 1;
        int nnz1 = vrownnz[row1];
        for (int j = 0; j < nnz1; j++) {
          int col = m->flexvert_J_colind[vrowadr[row1] + j];
          d->flexvert_J[vrowadr[row1] + j] += J1_dense[col] * scale;
          J1_dense[col] = 0;
        }
      }

      sim_freeStack(d);
    }
  }

  sim_freeStack(d);
}


// compute tendon lengths and moments
void sim_tendon(const sim_model_t* m, sim_data_t* d) {
  int nv = m->nv, nten = m->ntendon;
  int *rownnz = d->ten_J_rownnz, *rowadr = d->ten_J_rowadr, *colind = d->ten_J_colind;
  sim_scalar_t *L = d->ten_length, *J = d->ten_J;

  if (!nten) {
    return;
  }

  // allocate stack arrays
  int *chain, *buf_ind;
  sim_scalar_t *jac1, *jac2, *jacdif, *tmp, *sparse_buf;
  sim_markStack(d);
  jac1 = SIM_STACK_ALLOC(d, 3*nv, sim_scalar_t);
  jac2 = SIM_STACK_ALLOC(d, 3*nv, sim_scalar_t);
  jacdif = SIM_STACK_ALLOC(d, 3*nv, sim_scalar_t);
  tmp = SIM_STACK_ALLOC(d, nv, sim_scalar_t);
  chain = SIM_STACK_ALLOC(d, nv, int);
  buf_ind = SIM_STACK_ALLOC(d, nv, int);
  sparse_buf = SIM_STACK_ALLOC(d, nv, sim_scalar_t);

  // clear results
  sim_math_zero(L, nten);

  // clear Jacobian
  sim_math_zeroInt(rownnz, nten);

  // sleep filtering
  int sleep_filter = SIM_ENABLED(SIM_ENBL_SLEEP) && d->ntree_awake < m->ntree;

  // loop over tendons
  int wrapcount = 0;
  for (int i=0; i < nten; i++) {
    // skip sleeping tendon
    if (sleep_filter && sim_sleepState(m, d, SIM_OBJ_TENDON, i) == sim_spec_ASLEEP) {
      continue;
    }

    // initialize tendon path
    int adr = m->tendon_adr[i];
    d->ten_wrapadr[i] = wrapcount;
    d->ten_wrapnum[i] = 0;
    int tendon_num = m->tendon_num[i];

    // sparse Jacobian row init
    rowadr[i] = (i > 0 ? rowadr[i-1] + rownnz[i-1] : 0);

    // process fixed tendon
    if (m->wrap_type[adr] == SIM_WRAP_JOINT) {
      // process all defined joints
      for (int j=0; j < tendon_num; j++) {
        // get joint id
        int k = m->wrap_objid[adr+j];

        // add to length
        L[i] += m->wrap_prm[adr+j] * d->qpos[m->jnt_qposadr[k]];

        // add to moment
        rownnz[i] = sim_math_combineSparse(J+rowadr[i], &m->wrap_prm[adr+j], 1, 1,
                                      rownnz[i], 1,
                                      colind+rowadr[i], &m->jnt_dofadr[k],
                                      sparse_buf, buf_ind);
      }

      continue;
    }

    // process spatial tendon
    sim_scalar_t divisor = 1;
    int wraptype, j = 0;
    while (j < tendon_num-1) {
      // get 1st and 2nd object
      int type0 = m->wrap_type[adr+j+0];
      int type1 = m->wrap_type[adr+j+1];
      int id0 = m->wrap_objid[adr+j+0];
      int id1 = m->wrap_objid[adr+j+1];

      // pulley
      if (type0 == SIM_WRAP_PULLEY || type1 == SIM_WRAP_PULLEY) {
        // get divisor, insert obj=-2
        if (type0 == SIM_WRAP_PULLEY) {
          divisor = m->wrap_prm[adr+j];
          sim_math_zero_3(d->wrap_xpos+wrapcount*3);
          d->wrap_obj[wrapcount] = -2;
          d->ten_wrapnum[i]++;
          wrapcount++;
        }

        // move to next
        j++;
        continue;
      }

      // init sequence; assume it starts with site
      sim_scalar_t wlen = -1;
      int wrapid = -1;
      sim_scalar_t wpnt[12];
      sim_math_internal_copy_3(wpnt, d->site_xpos+3*id0);
      int wbody[4];
      wbody[0] = m->site_bodyid[id0];

      // second object is geom: process site-geom-site
      if (type1 == SIM_WRAP_SPHERE || type1 == SIM_WRAP_CYLINDER) {
        // reassign, get 2nd site info
        wraptype = type1;
        wrapid = id1;
        type1 = m->wrap_type[adr+j+2];
        id1 = m->wrap_objid[adr+j+2];

        // do wrapping, possibly get 2 extra points (wlen>=0)
        int sideid = sim_math_round(m->wrap_prm[adr+j+1]);
        if (sideid < -1 || sideid >= m->nsite) {
          SIM_ERROR("invalid sideid %d in wrap_prm", sideid);  // SHOULD NOT OCCUR
        }

        wlen = sim_math_wrap(wpnt+3, d->site_xpos+3*id0, d->site_xpos+3*id1,
                        d->geom_xpos+3*wrapid, d->geom_xmat+9*wrapid, m->geom_size[3*wrapid],
                        wraptype, (sideid >= 0 ? d->site_xpos+3*sideid : 0));
      } else {
        wraptype = SIM_WRAP_NONE;
      }

      // complete sequence, accumulate lengths
      if (wlen < 0) {
        sim_math_internal_copy_3(wpnt+3, d->site_xpos+3*id1);
        wbody[1] = m->site_bodyid[id1];
        L[i] += sim_math_dist3(wpnt, wpnt+3) / divisor;
      } else {
        sim_math_internal_copy_3(wpnt+9, d->site_xpos+3*id1);
        wbody[1] = wbody[2] = m->geom_bodyid[wrapid];
        wbody[3] = m->site_bodyid[id1];
        L[i] += (sim_math_dist3(wpnt, wpnt+3) + wlen + sim_math_dist3(wpnt+6, wpnt+9)) / divisor;
      }

      // accumulate moments if consecutive points are in different bodies
      for (int k=0; k < (wlen < 0 ? 1 : 3); k++) {
        if (wbody[k] != wbody[k+1]) {
          // get 3D position difference, normalize
          sim_scalar_t dif[3];
          sim_math_internal_sub3(dif, wpnt+3*k+3, wpnt+3*k);
          sim_math_normalize_3(dif);

          // get endpoint Jacobians, subtract
          int NV = sim_jacDifPair(m, d, chain,
                                 wbody[k], wbody[k+1], wpnt+3*k, wpnt+3*k+3,
                                 jac1, jac2, jacdif, NULL, NULL, NULL, /*issparse=*/1);

          // no dofs: skip
          if (!NV) {
            continue;
          }

          // apply chain rule to compute tendon Jacobian
          sim_math_mulMatTVec(tmp, jacdif, dif, 3, NV);

          // add to existing
          rownnz[i] = sim_math_combineSparse(J+rowadr[i], tmp, 1, 1/divisor,
                                        rownnz[i], NV, colind+rowadr[i],
                                        chain, sparse_buf, buf_ind);
        }
      }

      // assign to wrap
      if (wlen < 0) {
        sim_math_internal_copy_3(d->wrap_xpos+wrapcount*3, wpnt);
      } else {
        sim_math_internal_copy9(d->wrap_xpos+wrapcount*3, wpnt);
      }
      d->wrap_obj[wrapcount] = -1;
      if (wlen >= 0) {
        d->wrap_obj[wrapcount+1] = d->wrap_obj[wrapcount+2] = wrapid;
      }
      d->ten_wrapnum[i] += (wlen < 0 ? 1 : 3);
      wrapcount += (wlen < 0 ? 1 : 3);

      // advance
      j += (wraptype != SIM_WRAP_NONE ? 2 : 1);

      // assign last site before pulley or tendon end
      if (j == tendon_num-1 || m->wrap_type[adr+j+1] == SIM_WRAP_PULLEY) {
        sim_math_internal_copy_3(d->wrap_xpos+wrapcount*3, d->site_xpos+3*id1);
        d->wrap_obj[wrapcount] = -1;
        d->ten_wrapnum[i]++;
        wrapcount++;
      }
    }
  }

  sim_freeStack(d);
}


// compute time derivative of dense tendon Jacobian for one tendon
void sim_tendonDot(const sim_model_t* m, sim_data_t* d, int id, sim_scalar_t* Jdot) {
  int nv = m->nv;

  // tendon id is invalid: return
  if (id < 0 || id >= m->ntendon) {
    return;
  }

  // clear output
  sim_math_zero(Jdot, nv);

  // fixed tendon has zero Jdot: return
  int adr = m->tendon_adr[id];
  if (m->wrap_type[adr] == SIM_WRAP_JOINT) {
    return;
  }

  // allocate stack arrays
  sim_markStack(d);
  sim_scalar_t* jac1 = SIM_STACK_ALLOC(d, 3*nv, sim_scalar_t);
  sim_scalar_t* jac2 = SIM_STACK_ALLOC(d, 3*nv, sim_scalar_t);
  sim_scalar_t* jacdif = SIM_STACK_ALLOC(d, 3*nv, sim_scalar_t);
  sim_scalar_t* tmp = SIM_STACK_ALLOC(d, nv, sim_scalar_t);

  // process spatial tendon
  sim_scalar_t divisor = 1;
  int wraptype, j = 0;
  int num = m->tendon_num[id];
  while (j < num-1) {
    // get 1st and 2nd object
    int type0 = m->wrap_type[adr+j+0];
    int type1 = m->wrap_type[adr+j+1];
    int id0 = m->wrap_objid[adr+j+0];
    int id1 = m->wrap_objid[adr+j+1];

    // pulley
    if (type0 == SIM_WRAP_PULLEY || type1 == SIM_WRAP_PULLEY) {
      // get divisor, insert obj=-2
      if (type0 == SIM_WRAP_PULLEY) {
        divisor = m->wrap_prm[adr+j];
      }

      // move to next
      j++;
      continue;
    }

    // init sequence; assume it starts with site
    sim_scalar_t wpnt[6];
    sim_math_internal_copy_3(wpnt, d->site_xpos+3*id0);
    sim_scalar_t vel[6];
    sim_objectVelocity(m, d, SIM_OBJ_SITE, id0, vel, /*flg_local=*/0);
    sim_scalar_t wvel[6] = {vel[3], vel[4], vel[5], 0, 0, 0};
    int wbody[2];
    wbody[0] = m->site_bodyid[id0];

    // second object is geom: process site-geom-site
    if (type1 == SIM_WRAP_SPHERE || type1 == SIM_WRAP_CYLINDER) {
      // TODO(tassa) support geom wrapping (requires derivatives of sim_math_wrap)
      SIM_ERROR("geom wrapping not supported");
    } else {
      wraptype = SIM_WRAP_NONE;
    }

    // complete sequence
    wbody[1] = m->site_bodyid[id1];
    sim_math_internal_copy_3(wpnt+3, d->site_xpos+3*id1);
    sim_objectVelocity(m, d, SIM_OBJ_SITE, id1, vel, /*flg_local=*/0);
    sim_math_internal_copy_3(wvel+3, vel+3);

    // accumulate moments if consecutive points are in different bodies
    if (wbody[0] != wbody[1]) {
      // dpnt = 3D position difference, normalize
      sim_scalar_t dpnt[3];
      sim_math_sub_3(dpnt, wpnt+3, wpnt);
      sim_scalar_t norm = sim_math_normalize_3(dpnt);

      // dvel = d / dt (dpnt)
      sim_scalar_t dvel[3];
      sim_math_sub_3(dvel, wvel+3, wvel);
      sim_scalar_t dot = sim_math_dot_3(dpnt, dvel);
      sim_math_add_to_scale_3(dvel, dpnt, -dot);
      sim_math_scale_3(dvel, dvel, norm > SIM_MINVAL ? 1/norm : 0);

      // TODO(tassa ) write sparse branch, requires sim_jacDotSparse
      // if (sim_isSparse(m)) { ... }

      // get endpoint JacobianDots, subtract
      sim_jacDot(m, d, jac1, 0, wpnt, wbody[0]);
      sim_jacDot(m, d, jac2, 0, wpnt+3, wbody[1]);
      sim_math_sub(jacdif, jac2, jac1, 3*nv);

      // chain rule, first term: Jdot += d/dt(jac2 - jac1) * dpnt
      sim_math_mulMatTVec(tmp, jacdif, dpnt, 3, nv);

      // add to existing
      sim_math_addToScl(Jdot, tmp, 1/divisor, nv);

      // get endpoint Jacobians, subtract
      sim_jac(m, d, jac1, 0, wpnt, wbody[0]);
      sim_jac(m, d, jac2, 0, wpnt+3, wbody[1]);
      sim_math_sub(jacdif, jac2, jac1, 3*nv);

      // chain rule, second term: Jdot += (jac2 - jac1) * d/dt(dpnt)
      sim_math_mulMatTVec(tmp, jacdif, dvel, 3, nv);

      // add to existing
      sim_math_addToScl(Jdot, tmp, 1/divisor, nv);
    }

    // advance
    j += (wraptype != SIM_WRAP_NONE ? 2 : 1);
  }

  sim_freeStack(d);
}


// compute actuator/transmission lengths and moments
void sim_transmission(const sim_model_t* m, sim_data_t* d) {
  int nv = m->nv, nu = m->nu;

  // nothing to do
  if (!nu) {
    return;
  }

  // outputs
  sim_scalar_t* length = d->actuator_length;
  sim_scalar_t* moment = d->actuator_moment;
  int *rownnz = d->moment_rownnz;
  int *rowadr = d->moment_rowadr;
  int *colind = d->moment_colind;

  // allocate Jacbians
  sim_markStack(d);
  sim_scalar_t* jac  = SIM_STACK_ALLOC(d, 3*nv, sim_scalar_t);
  sim_scalar_t* jacA = SIM_STACK_ALLOC(d, 3*nv, sim_scalar_t);
  sim_scalar_t* jacS = SIM_STACK_ALLOC(d, 3*nv, sim_scalar_t);

  // define stack variables required for body transmission, don't allocate
  int issparse = sim_isSparse(m);
  sim_scalar_t* efc_force = NULL;  // used as marker for allocation requirement
  sim_scalar_t *moment_exclude, *jacdifp, *jac1p, *jac2p;
  int *chain;

  // define stack variables required for site transmission, don't allocate
  sim_scalar_t *jacref = NULL, *moment_tmp = NULL;

  int sleep_filter = SIM_ENABLED(SIM_ENBL_SLEEP) && d->nv_awake < nv;

  // compute lengths and moments
  for (int i=0; i < nu; i++) {
    rowadr[i] = i == 0 ? 0 : rowadr[i-1] + rownnz[i-1];
    int nnz, adr = rowadr[i];

    // skip sleeping actuator
    if (sleep_filter && sim_sleepState(m, d, SIM_OBJ_ACTUATOR, i) == sim_spec_ASLEEP) {
      rownnz[i] = 0;
      continue;
    }

    // extract info
    int id = m->actuator_trnid[2*i];
    sim_scalar_t* gear = m->actuator_gear+6*i;

    // process according to transmission type
    switch ((SIM_tTrn) m->actuator_trntype[i]) {
    case SIM_TRN_JOINT:                   // joint
    case SIM_TRN_JOINTINPARENT:           // joint, force in parent frame
      // slide and hinge joint: scalar gear
      if (m->jnt_type[id] == SIM_JNT_SLIDE || m->jnt_type[id] == SIM_JNT_HINGE) {
        // sparsity
        rownnz[i] = 1;
        colind[adr] = m->jnt_dofadr[id];

        length[i] = d->qpos[m->jnt_qposadr[id]]*gear[0];
        moment[adr] = gear[0];
      }

      // ball joint: 3D wrench gear
      else if (m->jnt_type[id] == SIM_JNT_BALL) {
        // axis: expmap representation of quaternion
        sim_scalar_t axis[3], quat[4];
        sim_math_internal_copy4(quat, d->qpos+m->jnt_qposadr[id]);
        sim_math_normalize4(quat);
        sim_math_internal_quat2Vel(axis, quat, 1);

        // gearAxis: rotate to parent frame if necessary
        sim_scalar_t gearAxis[3];
        if (m->actuator_trntype[i] == SIM_TRN_JOINT) {
          sim_math_internal_copy_3(gearAxis, gear);
        } else {
          sim_math_negQuat(quat, quat);
          sim_math_internal_rotVecQuat(gearAxis, gear, quat);
        }

        // length: axis*gearAxis
        length[i] = sim_math_dot_3(axis, gearAxis);

        // dof start address
        int jnt_dofadr = m->jnt_dofadr[id];

        // sparsity
        for (int j = 0; j < 3; j++) {
          colind[adr+j] = jnt_dofadr + j;
        }
        rownnz[i] = 3;

        // moment: gearAxis
        sim_math_internal_copy_3(moment+adr, gearAxis);
      }

      // free joint: 6D wrench gear
      else {
        // cannot compute meaningful length, set to 0
        length[i] = 0;

        // gearAxis: rotate to world frame if necessary
        sim_scalar_t gearAxis[3];
        if (m->actuator_trntype[i] == SIM_TRN_JOINT) {
          sim_math_internal_copy_3(gearAxis, gear+3);
        } else {
          sim_scalar_t quat[4];
          sim_math_internal_copy4(quat, d->qpos+m->jnt_qposadr[id]+3);
          sim_math_normalize4(quat);
          sim_math_negQuat(quat, quat);
          sim_math_internal_rotVecQuat(gearAxis, gear+3, quat);
        }

        // dof start address
        int jnt_dofadr = m->jnt_dofadr[id];

        // sparsity
        for (int j = 0; j < 6; j++) {
          colind[adr+j] = jnt_dofadr + j;
        }
        rownnz[i] = 6;

        // moment: gear(tran), gearAxis
        sim_math_internal_copy_3(moment+adr, gear);
        sim_math_internal_copy_3(moment+adr+3, gearAxis);
      }
      break;

    case SIM_TRN_SLIDERCRANK:             // slider-crank
      {
        // get data
        int idslider = m->actuator_trnid[2*i+1];
        sim_scalar_t rod = m->actuator_cranklength[i];
        sim_scalar_t axis[3] = {d->site_xmat[9 * idslider + 2],
                          d->site_xmat[9 * idslider + 5],
                          d->site_xmat[9 * idslider + 8]};
        sim_scalar_t vec[3];
        sim_math_sub_3(vec, d->site_xpos+3*id, d->site_xpos+3*idslider);

        // compute length and determinant
        //  length = a'*v - sqrt(det);  det = (a'*v)^2 + r^2 - v'*v)
        sim_scalar_t av = sim_math_dot_3(vec, axis);
        sim_scalar_t sdet, det = av*av + rod*rod - sim_math_dot_3(vec, vec);
        int ok = 1;
        if (det <= 0) {
          ok = 0;
          sdet = 0;
          length[i] = av;
        } else {
          sdet = sim_math_sqrt(det);
          length[i] = av - sdet;
        }

        // compute derivatives of length w.r.t. vec and axis
        sim_scalar_t dlda[3], dldv[3];
        if (ok) {
          sim_math_scale_3(dldv, axis, 1-av/sdet);
          sim_math_scale_3(dlda, vec, 1/sdet);        // use dlda as temp
          sim_math_internal_addTo3(dldv, dlda);

          sim_math_scale_3(dlda, vec, 1-av/sdet);
        } else {
          sim_math_internal_copy_3(dlda, vec);
          sim_math_internal_copy_3(dldv, axis);
        }

        // get Jacobians of axis(jacA) and vec(jac)
        sim_jacPointAxis(m, d, jacS, jacA, d->site_xpos+3*idslider,
                        axis, m->site_bodyid[idslider]);
        sim_jacSite(m, d, jac, 0, id);
        sim_math_subFrom(jac, jacS, 3*nv);

        // clear moment
        sim_math_zero(moment + adr, nv);

        // apply chain rule
        for (int j=0; j < nv; j++) {
          for (int k=0; k < 3; k++) {
            moment[adr+j] += dlda[k]*jacA[k*nv+j] + dldv[k]*jac[k*nv+j];
          }
        }

        // scale by gear ratio
        length[i] *= gear[0];
        for (int j = 0; j < nv; j++) {
          moment[adr+j] *= gear[0];
        }

        // sparsity (compress)
        nnz = 0;
        for (int j = 0; j < nv; j++) {
          if (moment[adr+j]) {
            moment[adr+nnz] = moment[adr+j];
            colind[adr+nnz] = j;
            nnz++;
          }
        }
        rownnz[i] = nnz;
      }
      break;

    case SIM_TRN_TENDON:                  // tendon
      length[i] = d->ten_length[id]*gear[0];

      // moment
      {
        int ten_J_rownnz = d->ten_J_rownnz[id];
        int ten_J_rowadr = d->ten_J_rowadr[id];
        rownnz[i] = ten_J_rownnz;
        sim_math_copyInt(colind + adr, d->ten_J_colind + ten_J_rowadr, ten_J_rownnz);

        sim_math_scl(moment + adr, d->ten_J + ten_J_rowadr, gear[0], ten_J_rownnz);
      }
      break;

    case SIM_TRN_SITE:                    // site
      // get site translation (jac) and rotation (jacS) Jacobians in global frame
      sim_jacSite(m, d, jac, jacS, id);

      // clear length
      length[i] = 0;

      // reference site undefined
      if (m->actuator_trnid[2*i+1] == -1) {
        // wrench: gear expressed in global frame
        sim_scalar_t wrench[6];
        sim_math_internal_mulMatVec3(wrench, d->site_xmat+9*id, gear);      // translation
        sim_math_internal_mulMatVec3(wrench+3, d->site_xmat+9*id, gear+3);  // rotation

        // moment: global Jacobian projected on wrench
        sim_math_mulMatTVec(moment+adr, jac, wrench, 3, nv);       // translation
        sim_math_mulMatTVec(jac, jacS, wrench+3, 3, nv);           // rotation
        sim_math_addTo(moment+adr, jac, nv);                       // add the two
      }

      // reference site defined
      else {
        int refid = m->actuator_trnid[2*i+1];
        if (!jacref) jacref = SIM_STACK_ALLOC(d, 3*nv, sim_scalar_t);

        // initialize last dof address for each body
        int b0 = m->body_weldid[m->site_bodyid[id]];
        int b1 = m->body_weldid[m->site_bodyid[refid]];
        int dofadr0 = m->body_dofadr[b0] + m->body_dofnum[b0] - 1;
        int dofadr1 = m->body_dofadr[b1] + m->body_dofnum[b1] - 1;

        // find common ancestral dof, if any
        int dofadr_common = -1;
        if (dofadr0 >= 0 && dofadr1 >= 0) {
          // traverse up the tree until common ancestral dof is found
          while (dofadr0 != dofadr1) {
            if (dofadr0 < dofadr1) {
              dofadr1 = m->dof_parentid[dofadr1];
            } else {
              dofadr0 = m->dof_parentid[dofadr0];
            }
            if (dofadr0 == -1 || dofadr1 == -1) {
              // reached tree root, no common ancestral dof
              break;
            }
          }

          // found common ancestral dof
          if (dofadr0 == dofadr1) {
            dofadr_common = dofadr0;
          }
        }

        // clear moment
        sim_math_zero(moment+adr, nv);

        // translational transmission
        if (!sim_math_isZero(gear, 3)) {
          // vec: site position in reference site frame
          sim_scalar_t vec[3];
          sim_math_sub_3(vec, d->site_xpos+3*id, d->site_xpos+3*refid);
          sim_math_mulMatTVec3(vec, d->site_xmat+9*refid, vec);

          // length: dot product with gear
          length[i] += sim_math_dot_3(vec, gear);

          // jacref: global Jacobian of reference site
          sim_jacSite(m, d, jacref, NULL, refid);

          // subtract jacref from jac
          sim_math_subFrom(jac, jacref, 3*nv);

          // if common ancestral dof was found, clear the columns of its parental chain
          int da = dofadr_common;
          while (da >= 0) {
            jac[nv*0 + da] = 0;
            jac[nv*1 + da] = 0;
            jac[nv*2 + da] = 0;
            da = m->dof_parentid[da];
          }

          // wrench: translational gear expressed in global frame
          sim_scalar_t wrench[6];
          sim_math_internal_mulMatVec3(wrench, d->site_xmat+9*refid, gear);

          // moment: global Jacobian projected on wrench
          sim_math_mulMatTVec(moment+adr, jac, wrench, 3, nv);
        }

        // rotational transmission
        if (!sim_math_isZero(gear+3, 3)) {
          sim_scalar_t refquat[4];

          // get site and refsite quats from parent bodies (avoiding sim_math_mat2Quat)
          sim_scalar_t quat[4];
          sim_math_internal_mulQuat(quat, m->site_quat+4*id, d->xquat+4*m->site_bodyid[id]);
          sim_math_internal_mulQuat(refquat, m->site_quat+4*refid, d->xquat+4*m->site_bodyid[refid]);

          // convert difference to expmap (axis-angle)
          sim_scalar_t vec[3];
          sim_math_internal_subQuat(vec, quat, refquat);

          // add length: dot product with gear
          length[i] += sim_math_dot_3(vec, gear+3);

          // jacref: global rotational Jacobian of reference site
          sim_jacSite(m, d, NULL, jacref, refid);

          // subtract jacref from jacS
          sim_math_subFrom(jacS, jacref, 3*nv);

          // if common ancestral dof was found, clear the columns of its parental chain
          int da = dofadr_common;
          while (da >= 0) {
            jacS[nv*0 + da] = 0;
            jacS[nv*1 + da] = 0;
            jacS[nv*2 + da] = 0;
            da = m->dof_parentid[da];
          }

          // wrench: rotational gear expressed in global frame
          sim_scalar_t wrench[6];
          sim_math_internal_mulMatVec3(wrench, d->site_xmat+9*refid, gear+3);

          // moment_tmp: global Jacobian projected on wrench, add to moment
          if (!moment_tmp) moment_tmp = SIM_STACK_ALLOC(d, nv, sim_scalar_t);
          sim_math_mulMatTVec(moment_tmp, jacS, wrench, 3, nv);
          sim_math_addTo(moment+adr, moment_tmp, nv);
        }
      }

      // sparsity (compress)
      nnz = 0;
      for (int j = 0; j < nv; j++) {
        if (moment[adr+j]) {
          moment[adr+nnz] = moment[adr+j];
          colind[adr+nnz] = j;
          nnz++;
        }
      }
      rownnz[i] = nnz;

      break;

    case SIM_TRN_BODY:                  // body (adhesive contacts)
      // cannot compute meaningful length, set to 0
      length[i] = 0;

      // clear moment
      sim_math_zero(moment+adr, nv);

      // moment is average of all contact normal Jacobians
      {
        // allocate stack variables for the first SIM_TRN_BODY
        if (!efc_force) {
          efc_force = SIM_STACK_ALLOC(d, d->nefc, sim_scalar_t);
          moment_exclude = SIM_STACK_ALLOC(d, nv, sim_scalar_t);
          jacdifp = SIM_STACK_ALLOC(d, 3*nv, sim_scalar_t);
          jac1p = SIM_STACK_ALLOC(d, 3*nv, sim_scalar_t);
          jac2p = SIM_STACK_ALLOC(d, 3*nv, sim_scalar_t);
          chain = issparse ? SIM_STACK_ALLOC(d, nv, int) : NULL;
        }

        // clear efc_force and moment_exclude
        sim_math_zero(efc_force, d->nefc);
        sim_math_zero(moment_exclude, nv);

        // count all relevant contacts, accumulate Jacobians
        int counter = 0, ncon = d->ncon;
        for (int j=0; j < ncon; j++) {
          const sim_contact_t* con = d->contact+j;

          // get geom ids
          int g1 = con->geom[0];
          int g2 = con->geom[1];

          // contact involving flex, continue
          if (g1 < 0 || g2 < 0) {
            continue;
          }

          // get body ids
          int b1 = m->geom_bodyid[g1];
          int b2 = m->geom_bodyid[g2];

          // irrelevant contact, continue
          if (b1 != id && b2 != id) {
            continue;
          }

          // mark contact normals in efc_force
          if (!con->exclude) {
            counter++;

            // condim 1 or elliptic cones: normal is in the first row
            if (con->dim == 1 || m->opt.cone == SIM_CONE_ELLIPTIC) {
              efc_force[con->efc_address] = 1;
            }

            // pyramidal cones: average all pyramid directions
            else {
              int npyramid = con->dim-1;  // number of frictional directions
              for (int k=0; k < 2*npyramid; k++) {
                efc_force[con->efc_address+k] = 0.5/npyramid;
              }
            }
          }

          // excluded contact in gap: get sparse or dense Jacobian, accumulate
          else if (con->exclude == 1) {
            counter++;

            // get Jacobian difference
            int NV = sim_jacDifPair(m, d, chain, b1, b2, con->pos, con->pos,
                                   jac1p, jac2p, jacdifp, NULL, NULL, NULL, issparse);

            // project Jacobian along the normal of the contact frame
            sim_math_mulMatMat(jac, con->frame, jacdifp, 1, 3, NV);

            // accumulate in moment_exclude
            if (issparse) {
              for (int k=0; k < NV; k++) {
                moment_exclude[chain[k]] += jac[k];
              }
            } else {
              sim_math_addTo(moment_exclude, jac, nv);
            }
          }
        }

        // moment is average over contact normal Jacobians, make negative for adhesion
        if (counter) {
          // accumulate active contact Jacobians into moment
          sim_mulJacTVec(m, d, moment+adr, efc_force);

          // add Jacobians from excluded contacts
          sim_math_addTo(moment+adr, moment_exclude, nv);

          // normalize by total contacts, flip sign
          sim_math_scl(moment+adr, moment+adr, -1.0/counter, nv);
        }
      }

      // sparsity (compress)
      nnz = 0;
      for (int j = 0; j < nv; j++) {
        if (moment[adr+j]) {
          moment[adr+nnz] = moment[adr+j];
          colind[adr+nnz] = j;
          nnz++;
        }
      }
      rownnz[i] = nnz;

      break;

    default:
      SIM_ERROR("unknown transmission type %d", m->actuator_trntype[i]);  // SHOULD NOT OCCUR
    }
  }

  sim_freeStack(d);
}


//-------------------------- inertia ---------------------------------------------------------------

// add tendon armature to M
void sim_tendonArmature(const sim_model_t* m, sim_data_t* d) {
  int nv = m->nv, ntendon = m->ntendon;
  const int* M_rownnz = m->M_rownnz;
  const int* M_rowadr = m->M_rowadr;
  const int* M_colind = m->M_colind;

  // sleep filtering
  int sleep_filter = SIM_ENABLED(SIM_ENBL_SLEEP) && d->nv_awake < nv;

  for (int k=0; k < ntendon; k++) {
    // skip sleeping tendon
    if (sleep_filter && sim_sleepState(m, d, SIM_OBJ_TENDON, k) == sim_spec_ASLEEP) {
      continue;
    }

    sim_scalar_t armature = m->tendon_armature[k];
    if (!armature) {
      continue;
    }

    // get sparse info for tendon k
    int J_rowadr = d->ten_J_rowadr[k];
    int J_rownnz = d->ten_J_rownnz[k];
    const int* J_colind = d->ten_J_colind + J_rowadr;
    sim_scalar_t* ten_J = d->ten_J + J_rowadr;

    // M += armature * ten_J' * ten_J
    for (int j=0; j < J_rownnz; j++) {
      sim_scalar_t ten_J_i = ten_J[j];
      if (!ten_J_i) {
        continue;
      }

      // M[i,:] += armature * ten_J[i] * ten_J
      int i = J_colind[j];
      int M_adr = M_rowadr[i];
      sim_math_addToSclSparseInc(d->M + M_adr, ten_J,
                            M_rownnz[i], M_colind + M_adr,
                            J_rownnz, J_colind, armature * ten_J_i);
    }
  }
}


// composite rigid body inertia algorithm
void sim_crb(const sim_model_t* m, sim_data_t* d) {
  // outputs
  sim_scalar_t* crb = d->crb;
  sim_scalar_t* M   = d->M;

  // inputs
  const sim_scalar_t* cinert        = d->cinert;
  const sim_scalar_t* cdof          = d->cdof;
  const sim_scalar_t* dof_M0        = m->dof_M0;
  const sim_scalar_t* dof_armature  = m->dof_armature;
  const int* body_awake_ind   = d->body_awake_ind;
  const int* parent_awake_ind = d->parent_awake_ind;
  const int* dof_awake_ind    = d->dof_awake_ind;
  const int* rownnz           = m->M_rownnz;
  const int* rowadr           = m->M_rowadr;
  const int* body_parentid    = m->body_parentid;
  const int* dof_parentid     = m->dof_parentid;
  const int* dof_simplenum    = m->dof_simplenum;
  const int* dof_bodyid       = m->dof_bodyid;

  // sleep filtering
  int sleep_filter = SIM_ENABLED(SIM_ENBL_SLEEP) && d->nv_awake < m->nv;
  int nbody = sleep_filter ? d->nbody_awake : m->nbody;
  int nparent = sleep_filter ? d->nparent_awake : m->nbody;
  int nv = sleep_filter ? d->nv_awake : m->nv;

  // crb = cinert
  if (!sleep_filter) {
    sim_math_copy(crb, cinert, 10*nbody);
  } else {
    sim_math_copyRows(crb, cinert, body_awake_ind, nbody, 10);
  }

  // backward pass over bodies, accumulate composite inertias
  for (int b = nparent - 1; b >= 0; b--) {
    int i = sleep_filter ? parent_awake_ind[b] : b;
    if (body_parentid[i] > 0) {
      sim_math_addTo(crb + 10*body_parentid[i], crb + 10*i, 10);
    }
  }

  // clear M
  if (!sleep_filter) {
    sim_math_zero(M, m->nC);
  } else {
    sim_math_zeroSparse(M, rownnz, rowadr, dof_awake_ind, nv);
  }

  // dense forward pass over dofs
  for (int v=0; v < nv; v++) {
    int i = sleep_filter ? dof_awake_ind[v] : v;

    // simple dof: fixed diagonal inertia
    int adr = rowadr[i];
    if (dof_simplenum[i]) {
      M[adr] = dof_M0[i];
      continue;
    }

    // init M(i,i) with armature inertia
    int Madr_ij = adr + rownnz[i] - 1;
    M[Madr_ij] = dof_armature[i];

    // precompute buf = crb_body_i * cdof_i
    sim_scalar_t buf[6];
    sim_math_mulInertVec(buf, crb+10*dof_bodyid[i], cdof+6*i);

    // sparse backward pass over ancestors
    for (int j=i; j >= 0; j = dof_parentid[j]) {
      // M(i,j) += cdof_j * (crb_body_i * cdof_i)
      M[Madr_ij--] += sim_math_internal_dot6(cdof+6*j, buf);
    }
  }
}


void sim_makeM(const sim_model_t* m, sim_data_t* d) {
  TM_START;
  sim_crb(m, d);
  sim_tendonArmature(m, d);
  sim_math_scatter(d->qM, d->M, m->mapM2M, m->nC);  // TODO(tassa): scatter only awake dofs
  TM_END(SIM_TIMER_POS_INERTIA);
}


// sparse L'*D*L factorizaton of inertia-like matrix M, assumed spd
// (legacy implementation)
void sim_factorI_legacy(const sim_model_t* m, sim_data_t* d, const sim_scalar_t* M, sim_scalar_t* qLD,
                       sim_scalar_t* qLDiagInv) {
  int cnt;
  int Madr_kk, Madr_ki;
  sim_scalar_t tmp;

  // local copies of key variables
  int* dof_Madr = m->dof_Madr;
  int* dof_parentid = m->dof_parentid;
  int nv = m->nv;

  // copy M into LD
  sim_math_copy(qLD, M, m->nM);

  // dense backward loop over dofs (regular only, simple diagonal already copied)
  for (int k=nv-1; k >= 0; k--) {
    // get address of M(k,k)
    Madr_kk = dof_Madr[k];

    // check for small/negative numbers on diagonal
    if (qLD[Madr_kk] < SIM_MINVAL) {
      sim_runtime_warning(d, SIM_WARN_INERTIA, k);
      qLD[Madr_kk] = SIM_MINVAL;
    }

    // skip the rest if simple
    if (m->dof_simplenum[k]) {
      continue;
    }

    // sparse backward loop over ancestors of k (excluding k)
    Madr_ki = Madr_kk + 1;
    int i = dof_parentid[k];
    while (i >= 0) {
      tmp = qLD[Madr_ki] / qLD[Madr_kk];          // tmp = M(k,i) / M(k,k)

      // get number of ancestors of i (including i)
      if (i < nv-1) {
        cnt = dof_Madr[i+1] - dof_Madr[i];
      } else {
        cnt = m->nM - dof_Madr[i+1];
      }

      // M(i,j) -= M(k,j) * tmp
      sim_math_addToScl(qLD+dof_Madr[i], qLD+Madr_ki, -tmp, cnt);

      qLD[Madr_ki] = tmp;                         // M(k,i) = tmp

      // advance to i's parent
      i = dof_parentid[i];
      Madr_ki++;
    }
  }

  // compute 1/diag(D)
  for (int i=0; i < nv; i++) {
    qLDiagInv[i] = 1.0 / qLD[dof_Madr[i]];
  }
}


// sparse L'*D*L factorizaton of the inertia matrix M, assumed spd
void sim_factorM(const sim_model_t* m, sim_data_t* d) {
  TM_START;

  // sleep filtering
  int sleep_filter = SIM_ENABLED(SIM_ENBL_SLEEP) && d->nv_awake < m->nv;
  const int* index;
  int nv;

  // no sleep filtering: copy everything
  if (!sleep_filter) {
    index = NULL;
    nv = m->nv;
    sim_math_copy(d->qLD, d->M, m->nC);
  }

  // sleep filtering: copy only awake dofs
  else {
    index = d->dof_awake_ind;
    nv = d->nv_awake;
    sim_math_copySparse(d->qLD, d->M, m->M_rownnz, m->M_rowadr, d->dof_awake_ind, d->nv_awake);
  }

  // factorize
  sim_factorI(d->qLD, d->qLDiagInv, nv, m->M_rownnz, m->M_rowadr, m->M_colind, index);

  TM_ADD(SIM_TIMER_POS_INERTIA);
}


// sparse L'*D*L factorizaton of inertia-like matrix M, assumed spd (with dof skipping)
void sim_factorI(sim_scalar_t* mat, sim_scalar_t* diaginv, int nv,
                const int* rownnz, const int* rowadr, const int* colind,
                const int* index) {
  // backward loop over rows
  for (int j=nv-1; j >= 0; j--) {
    int k = index ? index[j] : j;

    // get row k's address, diagonal index, inverse diagonal value
    int start = rowadr[k];
    int diag = rownnz[k] - 1;
    int end = start + diag;
    sim_scalar_t invD = 1 / mat[end];
    if (diaginv) diaginv[k] = invD;

    // update triangle above row k
    for (int adr=end - 1; adr >= start; adr--) {
      // update row i < k:  L(i, 0..i) -= L(i, 0..i) * L(k, i) / L(k, k)
      int i = colind[adr];
      sim_math_addToScl(mat + rowadr[i], mat + start, -mat[adr] * invD, rownnz[i]);
    }

    // update row k:  L(k, :) /= L(k, k)
    sim_math_scl(mat + start, mat + start, invD, diag);
  }
}


// in-place sparse backsubstitution:  x = inv(L'*D*L)*x
// (legacy implementation)
void sim_solveLD_legacy(const sim_model_t* m, sim_scalar_t* restrict x, int n,
                       const sim_scalar_t* qLD, const sim_scalar_t* qLDiagInv) {
  // local copies of key variables
  int* dof_Madr = m->dof_Madr;
  int* dof_parentid = m->dof_parentid;
  int nv = m->nv;

  // single vector
  if (n == 1) {
    // x <- inv(L') * x; skip simple, exploit sparsity of input vector
    for (int i=nv-1; i >= 0; i--) {
      if (!m->dof_simplenum[i] && x[i]) {
        // init
        int Madr_ij = dof_Madr[i]+1;
        int j = dof_parentid[i];

        // traverse ancestors backwards
        // read directly from x[i] since i cannot be a parent of itself
        while (j >= 0) {
          x[j] -= qLD[Madr_ij++]*x[i];         // x(j) -= L(i,j) * x(i)

          // advance to parent
          j = dof_parentid[j];
        }
      }
    }

    // x <- inv(D) * x
    for (int i=0; i < nv; i++) {
      x[i] *= qLDiagInv[i];  // x(i) /= L(i,i)
    }

    // x <- inv(L) * x; skip simple
    for (int i=0; i < nv; i++) {
      if (!m->dof_simplenum[i]) {
        // init
        int Madr_ij = dof_Madr[i]+1;
        int j = dof_parentid[i];

        // traverse ancestors backwards
        // write directly in x[i] since i cannot be a parent of itself
        while (j >= 0) {
          x[i] -= qLD[Madr_ij++]*x[j];             // x(i) -= L(i,j) * x(j)

          // advance to parent
          j = dof_parentid[j];
        }
      }
    }
  }

  // multiple vectors
  else {
    int offset;
    sim_scalar_t tmp;

    // x <- inv(L') * x; skip simple
    for (int i=nv-1; i >= 0; i--) {
      if (!m->dof_simplenum[i]) {
        // init
        int Madr_ij = dof_Madr[i]+1;
        int j = dof_parentid[i];

        // traverse ancestors backwards
        while (j >= 0) {
          // process all vectors, exploit sparsity
          for (offset=0; offset < n*nv; offset+=nv)
            if ((tmp = x[i+offset])) {
              x[j+offset] -= qLD[Madr_ij]*tmp;  // x(j) -= L(i,j) * x(i)
            }

          // advance to parent
          Madr_ij++;
          j = dof_parentid[j];
        }
      }
    }

    // x <- inv(D) * x
    for (int i=0; i < nv; i++) {
      for (offset=0; offset < n*nv; offset+=nv) {
        x[i+offset] *= qLDiagInv[i];  // x(i) /= L(i,i)
      }
    }

    // x <- inv(L) * x; skip simple
    for (int i=0; i < nv; i++) {
      if (!m->dof_simplenum[i]) {
        // init
        int Madr_ij = dof_Madr[i]+1;
        int j = dof_parentid[i];

        // traverse ancestors backwards
        tmp = x[i+offset];
        while (j >= 0) {
          // process all vectors
          for (offset=0; offset < n*nv; offset+=nv) {
            x[i+offset] -= qLD[Madr_ij]*x[j+offset];  // x(i) -= L(i,j) * x(j)
          }

          // advance to parent
          Madr_ij++;
          j = dof_parentid[j];
        }
      }
    }
  }
}


// in-place sparse backsubstitution:  x = inv(L'*D*L)*x  (with dof skipping)
void sim_solveLD(sim_scalar_t* restrict x, const sim_scalar_t* qLD, const sim_scalar_t* qLDiagInv, int nv, int n,
                const int* rownnz, const int* rowadr, const int* colind, const int* index) {
  // x <- L^-T x
  for (int k = nv - 1; k >= 0; k--) {
    int i = index ? index[k] : k;

    // skip diagonal rows
    if (rownnz[i] == 1) {
      continue;
    }

    // one vector
    if (n == 1) {
      sim_scalar_t x_i;
      if ((x_i = x[i])) {
        int start = rowadr[i];
        int end = start + rownnz[i] - 1;
        for (int adr=start; adr < end; adr++) {
          x[colind[adr]] -= qLD[adr] * x_i;
        }
      }
    }

    // multiple vectors
    else {
      int start = rowadr[i];
      int end = start + rownnz[i] - 1;
      for (int offset=0; offset < n*nv; offset+=nv) {
        sim_scalar_t x_i;
        if ((x_i = x[i+offset])) {
          for (int adr=start; adr < end; adr++) {
            x[offset + colind[adr]] -= qLD[adr] * x_i;
          }
        }
      }
    }
  }

  // x <- D^-1 x
  for (int k = 0; k < nv; k++) {
    int i = index ? index[k] : k;

    sim_scalar_t invD_i = qLDiagInv[i];

    // one vector
    if (n == 1) {
      x[i] *= invD_i;
    }

    // multiple vectors
    else {
      for (int offset=0; offset < n*nv; offset+=nv) {
        x[i+offset] *= invD_i;
      }
    }
  }

  // x <- L^-1 x
  for (int k = 0; k < nv; k++) {
    int i = index ? index[k] : k;

    // skip diagonal rows
    if (rownnz[i] == 1) {
      continue;
    }

    int d;
    if ((d = rownnz[i] - 1) > 0) {
      int adr = rowadr[i];

      // one vector
      if (n == 1) {
        x[i] -= sim_math_dotSparse(qLD+adr, x, d, colind+adr);
      }

      // multiple vectors
      else {
        for (int offset=0; offset < n*nv; offset+=nv) {
          x[i+offset] -= sim_math_dotSparse(qLD+adr, x+offset, d, colind+adr);
        }
      }
    }
  }
}


// sparse backsubstitution:  x = inv(L'*D*L)*y
//  use factorization in d
void sim_solveM(const sim_model_t* m, sim_data_t* d, sim_scalar_t* x, const sim_scalar_t* y, int n) {
  if (x != y) {
    sim_math_copy(x, y, n*m->nv);
  }
  sim_solveLD(x, d->qLD, d->qLDiagInv, m->nv, n, m->M_rownnz, m->M_rowadr, m->M_colind, NULL);
}


// half of sparse backsubstitution:  x = sqrt(inv(D))*inv(L')*y
void sim_solveM2(const sim_model_t* m, sim_data_t* d, sim_scalar_t* x, const sim_scalar_t* y,
                const sim_scalar_t* sqrtInvD, int n) {
  int nv = m->nv;

  // local copies of key variables
  const int* rownnz = m->M_rownnz;
  const int* rowadr = m->M_rowadr;
  const int* colind = m->M_colind;
  const int* diagnum = m->dof_simplenum;
  const sim_scalar_t* qLD = d->qLD;

  // x = y
  sim_math_copy(x, y, n * nv);

  // x <- L^-T x
  for (int i=nv-1; i > 0; i--) {
    // skip diagonal rows
    if (diagnum[i]) {
      continue;
    }

    // prepare row i column address range
    int start = rowadr[i];
    int end = start + rownnz[i] - 1;

    // process all vectors
    for (int offset=0; offset < n*nv; offset+=nv) {
      sim_scalar_t x_i;
      if ((x_i = x[i+offset])) {
        for (int adr=start; adr < end; adr++) {
          x[offset + colind[adr]] -= qLD[adr] * x_i;
        }
      }
    }
  }

  // x <- D^-1/2 x
  for (int i=0; i < nv; i++) {
    sim_scalar_t invD_i = sqrtInvD[i];
    for (int offset=0; offset < n*nv; offset+=nv) {
      x[i+offset] *= invD_i;
    }
  }
}


//---------------------------------- velocity ------------------------------------------------------

// compute cvel, cdof_dot
void sim_comVel(const sim_model_t* m, sim_data_t* d) {
  int sleep_filter = SIM_ENABLED(SIM_ENBL_SLEEP) && d->nbody_awake < m->nbody;
  int nbody = sleep_filter ? d->nbody_awake : m->nbody;

  // set world vel to 0
  sim_math_zero(d->cvel, 6);

  // forward pass over bodies
  for (int b=1; b < nbody; b++) {
    int i = sleep_filter ? d->body_awake_ind[b] : b;

    // cvel = cvel_parent
    sim_scalar_t cvel[6];
    sim_math_internal_copy6(cvel, d->cvel+6*m->body_parentid[i]);

    // cvel = cvel_parent + cdof * qvel,  cdofdot = cvel x cdof
    int dofnum = m->body_dofnum[i];
    int bda = m->body_dofadr[i];
    sim_scalar_t cdofdot[36];
    for (int j=0; j < dofnum; j++) {
      sim_scalar_t tmp[6];

      // compute cvel and cdofdot
      switch ((SIM_tJoint) m->jnt_type[m->dof_jntid[bda+j]]) {
      case SIM_JNT_FREE:
        // cdofdot = 0
        sim_math_zero(cdofdot, 18);

        // update velocity
        sim_math_mulDofVec(tmp, d->cdof+6*bda, d->qvel+bda, 3);
        sim_math_addTo(cvel, tmp, 6);

        // continue with rotations
        j += 3;
        SIM_FALLTHROUGH;

      case SIM_JNT_BALL:
        // compute all 3 cdofdots using parent velocity
        sim_math_internal_crossMotion(cdofdot+6*(j+0), cvel, d->cdof+6*(bda+j+0));
        sim_math_internal_crossMotion(cdofdot+6*(j+1), cvel, d->cdof+6*(bda+j+1));
        sim_math_internal_crossMotion(cdofdot+6*(j+2), cvel, d->cdof+6*(bda+j+2));

        // update velocity
        sim_math_mulDofVec(tmp, d->cdof+6*(bda+j), d->qvel+bda+j, 3);
        sim_math_addTo(cvel, tmp, 6);

        // adjust for 3-dof joint
        j += 2;
        break;

      default:
        // in principle we should use the new velocity to compute cdofdot,
        // but it makes no difference because crossMotion(cdof, cdof) = 0,
        // and using the old velocity may be more accurate numerically
        sim_math_internal_crossMotion(cdofdot+6*j, cvel, d->cdof+6*(bda+j));

        // update velocity
        sim_math_mulDofVec(tmp, d->cdof+6*(bda+j), d->qvel+bda+j, 1);
        sim_math_addTo(cvel, tmp, 6);
      }
    }

    // assign cvel, cdofdot
    sim_math_internal_copy6(d->cvel+6*i, cvel);
    sim_math_copy(d->cdof_dot+6*bda, cdofdot, 6*dofnum);
  }
}


// subtree linear velocity and angular momentum
void sim_subtreeVel(const sim_model_t* m, sim_data_t* d) {
  int sleep_filter = SIM_ENABLED(SIM_ENBL_SLEEP) && d->nbody_awake < m->nbody;
  int nbody = sleep_filter ? d->nbody_awake : m->nbody;

  sim_markStack(d);
  sim_scalar_t* body_vel = SIM_STACK_ALLOC(d, 6*m->nbody, sim_scalar_t);

  // bodywise quantities
  for (int b=0; b < nbody; b++) {
    int i = sleep_filter ? d->body_awake_ind[b] : b;

    // compute and save body velocity
    sim_objectVelocity(m, d, SIM_OBJ_BODY, i, body_vel+6*i, 0);

    // body linear momentum
    sim_math_scale_3(d->subtree_linvel+3*i, body_vel+6*i+3, m->body_mass[i]);

    // body angular momentum
    sim_scalar_t dv[3];
    sim_math_mulMatTVec3(dv, d->ximat+9*i, body_vel+6*i);
    dv[0] *= m->body_inertia[3*i];
    dv[1] *= m->body_inertia[3*i+1];
    dv[2] *= m->body_inertia[3*i+2];
    sim_math_internal_mulMatVec3(d->subtree_angmom+3*i, d->ximat+9*i, dv);
  }

  // subtree linear velocity
  for (int b=nbody-1; b >= 0; b--) {
    int i = sleep_filter ? d->body_awake_ind[b] : b;

    // non-world: add linear momentum to parent
    if (i) {
      sim_math_internal_addTo3(d->subtree_linvel+3*m->body_parentid[i], d->subtree_linvel+3*i);
    }

    // convert linear momentum to linear velocity
    sim_math_scale_3(d->subtree_linvel+3*i, d->subtree_linvel+3*i,
             1/sim_math_max(SIM_MINVAL, m->body_subtreemass[i]));
  }

  // subtree angular momentum
  for (int b=nbody-1; b > 0; b--) {
    int i = sleep_filter ? d->body_awake_ind[b] : b;

    int parent = m->body_parentid[i];

    // momentum wrt body i
    sim_scalar_t dx[3], dv[3], dp[3], dL[3];
    sim_math_sub_3(dx, d->xipos+3*i, d->subtree_com+3*i);
    sim_math_sub_3(dv, body_vel+6*i+3, d->subtree_linvel+3*i);
    sim_math_scale_3(dp, dv, m->body_mass[i]);
    sim_math_internal_cross(dL, dx, dp);

    // add to subtree i
    sim_math_internal_addTo3(d->subtree_angmom+3*i, dL);

    // add to parent
    sim_math_internal_addTo3(d->subtree_angmom+3*parent, d->subtree_angmom+3*i);

    // momentum wrt parent
    sim_math_sub_3(dx, d->subtree_com+3*i, d->subtree_com+3*parent);
    sim_math_sub_3(dv, d->subtree_linvel+3*i, d->subtree_linvel+3*parent);
    sim_math_scale_3(dv, dv, m->body_subtreemass[i]);
    sim_math_internal_cross(dL, dx, dv);

    // add to parent
    sim_math_internal_addTo3(d->subtree_angmom+3*parent, dL);
  }

  sim_freeStack(d);

  // mark as computed
  d->flg_subtreevel = 1;
}


//---------------------------------- RNE -----------------------------------------------------------

// RNE: compute M(qpos)*qacc + C(qpos,qvel); flg_acc=0 removes inertial term
void sim_rne(const sim_model_t* m, sim_data_t* d, int flg_acc, sim_scalar_t* result) {
  int sleep_filter = SIM_ENABLED(SIM_ENBL_SLEEP) && d->nbody_awake < m->nbody;
  int nbody = sleep_filter ? d->nbody_awake : m->nbody;
  int nparent = sleep_filter ? d->nparent_awake : m->nbody;
  int nv = sleep_filter ? d->nv_awake : m->nv;

  sim_markStack(d);
  sim_scalar_t* loc_cacc = SIM_STACK_ALLOC(d, m->nbody*6, sim_scalar_t);
  sim_scalar_t* loc_cfrc_body = SIM_STACK_ALLOC(d, m->nbody*6, sim_scalar_t);

  // set world acceleration to -gravity
  sim_math_zero(loc_cacc, 6);
  if (!SIM_DISABLED(SIM_DSBL_GRAVITY)) {
    sim_math_scale_3(loc_cacc+3, m->opt.gravity, -1);
  }

  // forward pass over bodies: accumulate cacc, set cfrc_body
  for (int b=1; b < nbody; b++) {
    int i = sleep_filter ? d->body_awake_ind[b] : b;

    // get body's first dof address
    int bda = m->body_dofadr[i];

    // cacc = cacc_parent + cdofdot * qvel
    sim_scalar_t tmp[6];
    sim_math_mulDofVec(tmp, d->cdof_dot+6*bda, d->qvel+bda, m->body_dofnum[i]);
    sim_math_add(loc_cacc+6*i, loc_cacc+6*m->body_parentid[i], tmp, 6);

    // cacc += cdof * qacc
    if (flg_acc) {
      sim_math_mulDofVec(tmp, d->cdof+6*bda, d->qacc+bda, m->body_dofnum[i]);
      sim_math_addTo(loc_cacc+6*i, tmp, 6);
    }

    // cfrc_body = cinert * cacc + cvel x (cinert * cvel)
    sim_math_mulInertVec(loc_cfrc_body+6*i, d->cinert+10*i, loc_cacc+6*i);
    sim_math_mulInertVec(tmp, d->cinert+10*i, d->cvel+6*i);
    sim_scalar_t tmp1[6];
    sim_math_internal_crossForce(tmp1, d->cvel+6*i, tmp);
    sim_math_addTo(loc_cfrc_body+6*i, tmp1, 6);
  }

  // clear world cfrc_body
  sim_math_zero(loc_cfrc_body, 6);

  // backward pass over bodies: accumulate cfrc_body from children
  for (int b=nparent-1; b > 0; b--) {
    int i = sleep_filter ? d->parent_awake_ind[b] : b;
    int j = m->body_parentid[i];

    if (j) {
      sim_math_addTo(loc_cfrc_body+6*j, loc_cfrc_body+6*i, 6);
    }
  }

  // result = cdof * cfrc_body
  for (int v=0; v < nv; v++) {
    int i = sleep_filter ? d->dof_awake_ind[v] : v;
    result[i] = sim_math_internal_dot6(d->cdof+6*i, loc_cfrc_body+6*m->dof_bodyid[i]);
  }

  sim_freeStack(d);
}


// RNE with complete data: compute cacc, cfrc_ext, cfrc_int
void sim_rnePostConstraint(const sim_model_t* m, sim_data_t* d) {
  int nbody = m->nbody;
  sim_scalar_t cfrc_com[6], cfrc[6], lfrc[6];
  sim_contact_t* con;

  // clear cacc, set world acceleration to -gravity
  sim_math_zero(d->cacc, 6);
  if (!SIM_DISABLED(SIM_DSBL_GRAVITY)) {
    sim_math_scale_3(d->cacc+3, m->opt.gravity, -1);
  }

  // cfrc_ext = perturb
  sim_math_zero(d->cfrc_ext, 6*nbody);
  for (int i=1; i < nbody; i++) {
    if (!sim_math_isZero(d->xfrc_applied+6*i, 6)) {
      // rearrange as torque:force
      sim_math_internal_copy_3(cfrc, d->xfrc_applied+6*i+3);
      sim_math_internal_copy_3(cfrc+3, d->xfrc_applied+6*i);

      // map force from application point to com; both world-oriented
      sim_math_transformSpatial(cfrc_com, cfrc, 1, d->subtree_com+3*m->body_rootid[i], d->xipos+3*i, 0);

      // accumulate
      sim_math_addTo(d->cfrc_ext+6*i, cfrc_com, 6);
    }
  }

  // cfrc_ext += contacts
  int ncon = d->ncon;
  for (int i=0; i < ncon; i++) {
    // get contact pointer
    con = d->contact+i;

    // skip excluded contacts
    if (con->efc_address < 0) {
      continue;
    }

    // skip contact involving flex
    if (con->geom[0] < 0 || con->geom[1] < 0) {
      continue;
    }

    // tmp = contact-local force:torque vector
    sim_contactForce(m, d, i, lfrc);

    // cfrc = world-oriented torque:force vector (swap in the process)
    sim_math_mulMatTVec3(cfrc, con->frame, lfrc+3);
    sim_math_mulMatTVec3(cfrc+3, con->frame, lfrc);

    // body 1
    int k;
    if ((k = m->geom_bodyid[con->geom[0]])) {
      // tmp = subtree CoM-based torque_force vector
      sim_math_transformSpatial(cfrc_com, cfrc, 1, d->subtree_com+3*m->body_rootid[k], con->pos, 0);

      // apply (opposite for body 1)
      sim_math_subFrom(d->cfrc_ext+6*k, cfrc_com, 6);
    }

    // body 2
    if ((k = m->geom_bodyid[con->geom[1]])) {
      // tmp = subtree CoM-based torque_force vector
      sim_math_transformSpatial(cfrc_com, cfrc, 1, d->subtree_com+3*m->body_rootid[k], con->pos, 0);

      // apply
      sim_math_addTo(d->cfrc_ext+6*k, cfrc_com, 6);
    }
  }

  // cfrc_ext += connect, weld, flex constraints
  int i = 0, ne = d->ne;
  while (i < ne) {
    if (d->efc_type[i] != SIM_CNSTR_EQUALITY) {
      SIM_ERROR("row %d of efc is not an equality constraint", i);  // SHOULD NOT OCCUR
    }

    int id = d->efc_id[i];
    sim_scalar_t* eq_data = m->eq_data + SIM_NEQDATA*id;
    sim_scalar_t pos[3], *offset;
    int k, obj1, obj2, body_semantic;
    switch ((SIM_tEq) m->eq_type[id]) {
    case SIM_EQ_CONNECT:
    case SIM_EQ_WELD:
      // cfrc = world-oriented torque:force vector
      sim_math_internal_copy_3(cfrc + 3, d->efc_force + i);
      if (m->eq_type[id] == SIM_EQ_WELD) {
        sim_math_internal_copy_3(cfrc, d->efc_force + i + 3);
      } else {
        sim_math_zero_3(cfrc);  // no torque from connect
      }

      body_semantic = m->eq_objtype[id] == SIM_OBJ_BODY;

      // body 1
      obj1 = m->eq_obj1id[id];
      k = body_semantic ? obj1 : m->site_bodyid[obj1];
      if (k) {
        offset = body_semantic ? eq_data + 3 * (m->eq_type[id] == SIM_EQ_WELD) :
                                 m->site_pos + 3 * obj1;

        // transform point on body1: local -> global
        sim_local2Global(d, pos, 0, offset, 0, k, 0);

        // tmp = subtree CoM-based torque_force vector
        sim_math_transformSpatial(cfrc_com, cfrc, 1, d->subtree_com+3*m->body_rootid[k], pos, 0);

        // apply (opposite for body 1)
        sim_math_addTo(d->cfrc_ext+6*k, cfrc_com, 6);
      }

      // body 2
      obj2 = m->eq_obj2id[id];
      k = body_semantic ? obj2 : m->site_bodyid[obj2];
      if (k) {
        offset = body_semantic ? eq_data + 3 * (m->eq_type[id] == SIM_EQ_CONNECT) :
                                 m->site_pos + 3 * obj2;

        // transform point on body2: local -> global
        sim_local2Global(d, pos, 0, offset, 0, k, 0);

        // tmp = subtree CoM-based torque_force vector
        sim_math_transformSpatial(cfrc_com, cfrc, 1, d->subtree_com+3*m->body_rootid[k], pos, 0);

        // apply
        sim_math_subFrom(d->cfrc_ext+6*k, cfrc_com, 6);
      }

      // increment rows
      i += m->eq_type[id] == SIM_EQ_WELD ? 6 : 3;
      break;

    case SIM_EQ_JOINT:
    case SIM_EQ_TENDON:
      // increment 1 row
      i++;
      break;

    case SIM_EQ_FLEX:
      // increment with number of non-rigid edges
      k = m->eq_obj1id[id];
      int flex_edgeadr = m->flex_edgeadr[k];
      int flex_edgenum = m->flex_edgenum[k];

      for (int e=flex_edgeadr; e < flex_edgeadr+flex_edgenum; e++) {
        if (!m->flexedge_rigid[e]) {
          i++;
        }
      }
      break;

    case SIM_EQ_FLEXVERT:
      k = m->eq_obj1id[id];
      i += 2*m->flex_vertnum[k];
      break;

    default:
      SIM_ERROR("unknown constraint type type %d", m->eq_type[id]);    // SHOULD NOT OCCUR
    }
  }

  // forward pass over bodies: compute cacc, cfrc_int
  sim_scalar_t cacc[6], cfrc_body[6], cfrc_corr[6];
  sim_math_zero(d->cfrc_int, 6);
  for (int j=1; j < nbody; j++) {
    // get body's first dof address
    int bda = m->body_dofadr[j];

    // cacc = cacc_parent + cdofdot * qvel + cdof * qacc
    sim_math_mulDofVec(cacc, d->cdof_dot+6*bda, d->qvel+bda, m->body_dofnum[j]);
    sim_math_add(d->cacc+6*j, d->cacc+6*m->body_parentid[j], cacc, 6);
    sim_math_mulDofVec(cacc, d->cdof+6*bda, d->qacc+bda, m->body_dofnum[j]);
    sim_math_addTo(d->cacc+6*j, cacc, 6);

    // cfrc_body = cinert * cacc + cvel x (cinert * cvel)
    sim_math_mulInertVec(cfrc_body, d->cinert+10*j, d->cacc+6*j);
    sim_math_mulInertVec(cfrc_corr, d->cinert+10*j, d->cvel+6*j);
    sim_math_internal_crossForce(cfrc, d->cvel+6*j, cfrc_corr);
    sim_math_addTo(cfrc_body, cfrc, 6);

    // set cfrc_int = cfrc_body - cfrc_ext
    sim_math_sub(d->cfrc_int+6*j, cfrc_body, d->cfrc_ext+6*j, 6);
  }

  // backward pass over bodies: accumulate cfrc_int from children
  for (int j=nbody-1; j > 0; j--) {
    sim_math_addTo(d->cfrc_int+6*m->body_parentid[j], d->cfrc_int+6*j, 6);
  }

  // mark as computed
  d->flg_rnepost = 1;
}


// add bias force due to tendon armature
void sim_tendonBias(const sim_model_t* m, sim_data_t* d, sim_scalar_t* qfrc) {
  int sleep_filter = SIM_ENABLED(SIM_ENBL_SLEEP) && d->ntree_awake < m->ntree;
  int ntendon = m->ntendon, nv = m->nv;
  sim_scalar_t* ten_Jdot = NULL;
  sim_markStack(d);

  // add bias term due to tendon armature
  for (int i=0; i < ntendon; i++) {
    // skip sleeping tendon
    if (sleep_filter && sim_sleepState(m, d, SIM_OBJ_TENDON, i) == sim_spec_ASLEEP) {
      continue;
    }

    sim_scalar_t armature = m->tendon_armature[i];

    // no armature: skip
    if (!armature) {
      continue;
    }

    // allocate if required
    if (!ten_Jdot) {
      ten_Jdot = SIM_STACK_ALLOC(d, nv, sim_scalar_t);
    }

    // get dense d/dt(tendon Jacobian) for tendon i
    sim_tendonDot(m, d, i, ten_Jdot);

    // add bias term:  qfrc += ten_J * armature * dot(ten_Jdot, qvel)
    sim_scalar_t coef = armature * sim_math_dot(ten_Jdot, d->qvel, nv);

    if (coef) {
      // sparse
      int nnz = d->ten_J_rownnz[i];
      int adr = d->ten_J_rowadr[i];
      const int* colind = d->ten_J_colind + adr;
      const sim_scalar_t* ten_J = d->ten_J + adr;
      for (int j=0; j < nnz; j++) {
        qfrc[colind[j]] += coef * ten_J[j];
      }
    }
  }

  sim_freeStack(d);
}

