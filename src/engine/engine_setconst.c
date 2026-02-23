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

#include "engine/engine_setconst.h"

#include <stdio.h>
#include <string.h>

#include <simcore/SIM_data.h>
#include <simcore/SIM_macro.h>
#include <simcore/SIM_model.h>
#include <simcore/SIM_san.h>  // IWYU pragma: keep
#include "engine/engine_core_smooth.h"
#include "engine/engine_core_util.h"
#include "engine/engine_forward.h"
#include "engine/engine_io.h"
#include "engine/engine_memory.h"
#include "engine/engine_util_blas.h"
#include "engine/engine_util_errmem.h"
#include "engine/engine_util_misc.h"
#include "engine/engine_util_sparse.h"
#include "engine/engine_util_spatial.h"


// compute dof_M0 via composite rigid body algorithm
static void sim_setM0(sim_model_t* m, sim_data_t* d) {
  sim_scalar_t buf[6];
  sim_scalar_t* crb = d->crb;
  int last_body = m->nbody - 1, nv = m->nv;

  // copy cinert into crb
  sim_math_copy(crb, d->cinert, 10*m->nbody);

  // backward pass over bodies, accumulate composite inertias
  for (int i=last_body; i > 0; i--) {
    if (m->body_parentid[i] > 0) {
      sim_math_addTo(crb+10*m->body_parentid[i], crb+10*i, 10);
    }
  }

  for (int i=0; i < nv; i++) {
    // precomute buf = crb_body_i * cdof_i
    sim_math_mulInertVec(buf, crb+10*m->dof_bodyid[i], d->cdof+6*i);

    // dof_M0(i) = armature inertia + cdof_i * (crb_body_i * cdof_i)
    m->dof_M0[i] = m->dof_armature[i] + sim_math_dot(d->cdof+6*i, buf, 6);
  }
}


// helper function to get the tree id of a wrap object
static int GetWrapBodyTreeId(const sim_model_t* m, int wrap_index) {
  int bodyid = -1;
  int objid = m->wrap_objid[wrap_index];
  switch ((SIM_tWrap)m->wrap_type[wrap_index]) {
  case SIM_WRAP_JOINT:
    bodyid = m->jnt_bodyid[objid];
    break;
  case SIM_WRAP_SITE:
    bodyid = m->site_bodyid[objid];
    break;
  case SIM_WRAP_SPHERE:
  case SIM_WRAP_CYLINDER:
    bodyid = m->geom_bodyid[objid];
    break;
  case SIM_WRAP_PULLEY:
  case SIM_WRAP_NONE:
    break;
  }
  return (bodyid != -1) ? m->body_treeid[bodyid] : -1;
}

// set fixed quantities (do not depend on qpos0)
static void setFixed(sim_model_t* m, sim_data_t* d) {
  sim_markStack(d);

  // ----- general

  // compute subtreemass
  for (int i=0; i < m->nbody; i++) {
    m->body_subtreemass[i] = m->body_mass[i];
  }
  for (int i=m->nbody-1; i > 0; i--) {
    m->body_subtreemass[m->body_parentid[i]] += m->body_subtreemass[i];
  }

  // compute ngravcomp: number of bodies with gravity compensation
  int ngravcomp = 0;
  for (int i=0; i < m->nbody; i++) {
    ngravcomp += (m->body_gravcomp[i] > 0);
  }
  m->ngravcomp = ngravcomp;


  // ----- tree related (body_treeid and dof_treeid already computed)

  // compute body_treeid
  for (int i=0; i < m->nbody; i++) {
    int weldid = m->body_weldid[i];
    if (m->body_dofnum[weldid]) {
      m->body_treeid[i] = m->dof_treeid[m->body_dofadr[weldid]];
    } else {
      m->body_treeid[i] = -1;
    }
  }

  // compute tree_bodyadr, tree_bodynum
  sim_math_zeroInt(m->tree_bodynum, m->ntree);
  int tree_current = -1;
  for (int i=1; i < m->nbody; i++) {
    int treeid = m->body_treeid[i];
    if (treeid != -1) {
      if (treeid > tree_current) {
        m->tree_bodyadr[++tree_current] = i;
      }
      m->tree_bodynum[tree_current]++;
    }
  }

  // compute tree_dofadr, tree_dofnum
  sim_math_zeroInt(m->tree_dofnum, m->ntree);
  tree_current = -1;
  for (int i=0; i < m->nv; i++) {
    if (m->dof_treeid[i] > tree_current) {
      m->tree_dofadr[++tree_current] = i;
    }
    m->tree_dofnum[tree_current]++;
  }

  // compute tendon_treeid, tendon_treenum
  int* tree_marker = SIM_STACK_ALLOC(d, m->ntree, int);  // 1 if tree has been visited, 0 otherwise
  for (int i = 0; i < m->ntendon; i++) {
    sim_math_zeroInt(tree_marker, m->ntree);
    m->tendon_treenum[i] = 0;
    m->tendon_treeid[2*i] = -1;
    m->tendon_treeid[2*i+1] = -1;

    for (int j = m->tendon_adr[i]; j < m->tendon_adr[i] + m->tendon_num[i]; j++) {
      int wrap_treeid = GetWrapBodyTreeId(m, j);
      if (wrap_treeid != -1 && !tree_marker[wrap_treeid]) {
        tree_marker[wrap_treeid] = 1;
        if (m->tendon_treenum[i] == 0) {
          m->tendon_treeid[2*i] = wrap_treeid;
        } else if (m->tendon_treenum[i] == 1) {
          m->tendon_treeid[2*i+1] = wrap_treeid;
        }
        m->tendon_treenum[i]++;
      }
    }
  }

  // ----- apply compiler AUTO tree sleep policy

  // actuators: trees with any actuated joint, site, body, or tendon do not auto-sleep
  for (int i=0; i < m->nu; i++) {
    int bodyid = -1;
    int tid = m->actuator_trnid[2*i];
    switch ((SIM_tTrn)m->actuator_trntype[i]) {
    case SIM_TRN_JOINT:
    case SIM_TRN_JOINTINPARENT:
      bodyid = m->jnt_bodyid[tid];
      break;
    case SIM_TRN_SITE:
    case SIM_TRN_SLIDERCRANK:
      bodyid = m->site_bodyid[tid];
      break;
    case SIM_TRN_BODY:
      bodyid = tid;
      break;
    case SIM_TRN_TENDON:
      // wake all trees connected by this actuated tendon
      for (int j = m->tendon_adr[tid]; j < m->tendon_adr[tid] + m->tendon_num[tid]; j++) {
        int treeid = GetWrapBodyTreeId(m, j);
        if (treeid != -1 && m->tree_sleep_policy[treeid] == SIM_SLEEP_AUTO) {
          m->tree_sleep_policy[treeid] = SIM_SLEEP_AUTO_NEVER;
        }
      }
      continue;  // next actuator
    case SIM_TRN_UNDEFINED:
      continue;  // next actuator
    }

    // wake tree containing bodyid, if any
    if (bodyid != -1) {
      int treeid = m->body_treeid[bodyid];
      if (treeid != -1 && m->tree_sleep_policy[treeid] == SIM_SLEEP_AUTO) {
        m->tree_sleep_policy[treeid] = SIM_SLEEP_AUTO_NEVER;
      }
    }
  }

  // trees with inter-tree tendons that have non-zero stiffness or damping do not auto-sleep
  // if the tendon spans more than 2 trees.
  for (int i=0; i < m->ntendon; i++) {
    int treenum = m->tendon_treenum[i];

    // tendon spans 1 or 0 trees: skip
    if (treenum < 2) {
      continue;
    }

    // tendon spans 2 trees and has no stiffness or damping: skip
    if (treenum == 2 && m->tendon_stiffness[i] == 0 && m->tendon_damping[i] == 0) {
      continue;
    }

    // tendon spans two trees with stiffness or damping or more than two trees: wake all trees
    sim_math_zeroInt(tree_marker, m->ntree);
    for (int j = m->tendon_adr[i]; j < m->tendon_adr[i] + m->tendon_num[i]; j++) {
      int treeid = GetWrapBodyTreeId(m, j);

      // if the tree is not yet marked, mark it and wake it up
      if (treeid != -1 && !tree_marker[treeid]) {
        tree_marker[treeid] = 1;
        int policy = m->tree_sleep_policy[treeid];

        // mark tree as never sleeping
        if (policy == SIM_SLEEP_AUTO) {
          m->tree_sleep_policy[treeid] = SIM_SLEEP_AUTO_NEVER;
        }

        // if the user marked it as sleepable, throw an error
        else if (policy == SIM_SLEEP_ALLOWED || policy == SIM_SLEEP_INIT) {
          sim_freeStack(d);
          if (treenum > 2) {
            SIM_ERROR("tree %d connected to tendon %d which spans more than 2 trees, "
                    "sleeping not allowed", treeid, i);
          } else {
            SIM_ERROR("tree %d connected to tendon %d with non-zero stiffness or damping, "
                    "sleeping not allowed", treeid, i);
          }
        }
      }
    }
  }

  // flexes: trees containing bodies that are part of any flex are not allowed to sleep
  for (int i = 0; i < m->nflex; ++i) {
    // node-based flex
    if (m->flex_interp[i]) {
      int nodenum = m->flex_nodenum[i];
      int* bodyid = m->flex_nodebodyid + m->flex_nodeadr[i];
      for (int j = 0; j < nodenum; ++j) {
        int treeid = m->body_treeid[bodyid[j]];
        if (treeid != -1 && m->tree_sleep_policy[treeid] == SIM_SLEEP_AUTO) {
          m->tree_sleep_policy[treeid] = SIM_SLEEP_AUTO_NEVER;
        }
      }
    }

    // vertex-based flex
    else {
      int vertnum = m->flex_vertnum[i];
      int* bodyid = m->flex_vertbodyid + m->flex_vertadr[i];
      for (int j = 0; j < vertnum; ++j) {
        int treeid = m->body_treeid[bodyid[j]];
        if (treeid != -1 && m->tree_sleep_policy[treeid] == SIM_SLEEP_AUTO) {
          m->tree_sleep_policy[treeid] = SIM_SLEEP_AUTO_NEVER;
        }
      }
    }
  }

  // set remaining trees with SIM_SLEEP_AUTO policy to SIM_SLEEP_AUTO_ALLOWED
  for (int i = 0; i < m->ntree; i++) {
    if (m->tree_sleep_policy[i] == SIM_SLEEP_AUTO) {
      m->tree_sleep_policy[i] = SIM_SLEEP_AUTO_ALLOWED;
    }
  }

  sim_freeStack(d);
}

// compute flex sparsity: flexedge_J_{rowadr,rownnz,colind} and flexvert_J_{rowadr,rownnz}
static void makeFlexSparse(sim_model_t* m, sim_data_t* d) {
  int nv = m->nv;
  int* rowadr = m->flexedge_J_rowadr;
  int* rownnz = m->flexedge_J_rownnz;
  int* colind = m->flexedge_J_colind;
  int* vrowadr = m->flexvert_J_rowadr;
  int* vrownnz = m->flexvert_J_rownnz;

  if (!m->nflex) {
    return;
  }

  sim_markStack(d);
  int* chain = SIM_STACK_ALLOC(d, nv, int);
  int* chain1 = SIM_STACK_ALLOC(d, nv, int);
  int* chain2 = SIM_STACK_ALLOC(d, nv, int);
  int* buf_ind = SIM_STACK_ALLOC(d, nv, int);
  sim_scalar_t* dummy_pos = SIM_STACK_ALLOC(d, 3, sim_scalar_t);
  sim_math_zero(dummy_pos, 3);

  // clear
  sim_math_zeroInt(rowadr, m->nflexedge);
  sim_math_zeroInt(rownnz, m->nflexedge);
  sim_math_zeroInt(vrowadr, 2 * m->nflexvert);
  sim_math_zeroInt(vrowadr, 2 * m->nflexvert);
  sim_math_zeroInt(vrownnz, 2 * m->nflexvert);
  sim_math_zeroInt(m->flex_vertedgeadr, m->nflexvert);
  sim_math_zeroInt(m->flex_vertedgenum, m->nflexvert);
  sim_math_zeroInt(m->flex_vertedge, 2 * m->nflexedge);
  sim_math_zeroInt(m->flex_vertedge, 2 * m->nflexedge);
  sim_math_zero(m->flex_vertmetric, 4 * m->nflexvert);
  int current_adj_offset = 0;

  // compute lengths and Jacobians of edges
  for (int f = 0; f < m->nflex; f++) {
    // skip if edges cannot generate forces
    if (m->flex_rigid[f] || m->flex_interp[f]) {
      continue;
    }

    // skip Jacobian if no built-in passive force is needed
    int skipjacobian = !m->flex_edgeequality[f] && !m->flex_edgedamping[f] &&
                       !m->flex_edgestiffness[f] && !m->flex_damping[f];

    // process edges of this flex
    int vbase = m->flex_vertadr[f];
    int ebase = m->flex_edgeadr[f];
    for (int e = 0; e < m->flex_edgenum[f]; e++) {
      if (skipjacobian) {
        continue;
      }

      // set rowadr
      if (ebase + e > 0) {
        rowadr[ebase + e] = rowadr[ebase + e - 1] + rownnz[ebase + e - 1];
      }

      int v1 = m->flex_edge[2 * (ebase + e)];
      int v2 = m->flex_edge[2 * (ebase + e) + 1];
      int b1 = m->flex_vertbodyid[vbase + v1];
      int b2 = m->flex_vertbodyid[vbase + v2];

      // get sparsity
      int NV = sim_jacDifPair(m, d, chain, b1, b2, dummy_pos, dummy_pos, NULL,
                             NULL, NULL, NULL, NULL, NULL, /*issparse=*/1);

      // copy sparsity info
      rownnz[ebase + e] = NV;
      sim_math_copyInt(colind + rowadr[ebase + e], chain, NV);
    }

    // if dim=2 and constraints are active we use the vertex-based constraint
    if (m->flex_dim[f] == 2 && m->flex_edgeequality[f] == 2) {
      int nvert = m->flex_vertnum[f];

      // populate global vertex adjacency list
      int* v_edge_cnt = m->flex_vertedgenum + vbase;
      int* v_edge_adr = m->flex_vertedgeadr + vbase;
      int* adj_edges = m->flex_vertedge;  // global array

      for (int e = 0; e < m->flex_edgenum[f]; ++e) {
        v_edge_cnt[m->flex_edge[2 * (ebase + e) + 0]]++;
        v_edge_cnt[m->flex_edge[2 * (ebase + e) + 1]]++;
      }
      int total_adj_edges = 0;
      for (int v = 0; v < nvert; ++v) {
        v_edge_adr[v] = current_adj_offset + total_adj_edges;
        total_adj_edges += v_edge_cnt[v];
      }
      int* v_edge_fill = SIM_STACK_ALLOC(d, nvert, int);
      sim_math_zeroInt(v_edge_fill, nvert);
      for (int e = 0; e < m->flex_edgenum[f]; ++e) {
        int v1 = m->flex_edge[2 * (ebase + e) + 0];
        int v2 = m->flex_edge[2 * (ebase + e) + 1];
        adj_edges[v_edge_adr[v1] + v_edge_fill[v1]] = e;
        v_edge_fill[v1]++;
        adj_edges[v_edge_adr[v2] + v_edge_fill[v2]] = e;
        v_edge_fill[v2]++;
      }

      // precompute metric (Binv)
      for (int v = 0; v < nvert; ++v) {
        sim_scalar_t B[4] = {0};
        int v_global = vbase + v;

        for (int k = 0; k < v_edge_cnt[v]; ++k) {
          int e = adj_edges[v_edge_adr[v] + k];

          // compute rest edge vector
          sim_scalar_t dx[3];
          int v1 = m->flex_edge[2 * (ebase + e)];
          int v2 = m->flex_edge[2 * (ebase + e) + 1];
          sim_math_sub_3(dx, m->flex_vert0 + 3 * (vbase + v2),
                   m->flex_vert0 + 3 * (vbase + v1));

          // apply scaling since they are half sizes
          dx[0] *= 2 * m->flex_size[3 * f + 0];
          dx[1] *= 2 * m->flex_size[3 * f + 1];
          dx[2] *= 2 * m->flex_size[3 * f + 2];

          if (sim_math_abs(dx[2]) > SIM_MINVAL) {
            SIM_ERROR("flex vertices are not in the same plane");
          }

          // get mass of neighbor vertex
          sim_scalar_t weight = 1.0;
          int neighbor_v = (v == v1) ? v2 : v1;
          int b_neighbor = m->flex_vertbodyid[vbase + neighbor_v];
          if (b_neighbor >= 0) {
            weight = m->body_mass[b_neighbor];
            if (weight < SIM_MINVAL) weight = SIM_MINVAL;
          }

          // accumulate B += w * dx * dx'
          for (int row = 0; row < 2; row++) {
            for (int col = 0; col < 2; col++) {
              B[2 * row + col] += weight * dx[row] * dx[col];
            }
          }
        }

        sim_scalar_t* metric = m->flex_vertmetric + 4 * v_global;
        sim_scalar_t det = B[0] * B[3] - B[1] * B[2];

        if (sim_math_abs(det) < SIM_MINVAL) {
          sim_math_zero(metric, 4);
        } else {
          sim_scalar_t invdet = 1.0 / det;
          metric[0] = B[3] * invdet;
          metric[1] = -B[1] * invdet;
          metric[2] = -B[2] * invdet;
          metric[3] = B[0] * invdet;
        }
      }

      // advance global offset
      current_adj_offset += total_adj_edges;

      // determine start address for this flex
      int v0_base = 2 * vbase;
      int current_adr = 0;
      if (v0_base > 0) {
        current_adr = vrowadr[v0_base - 1] + vrownnz[v0_base - 1];
      }
      vrowadr[v0_base] = current_adr;

      for (int v = 0; v < nvert; ++v) {
        // clear buf_ind
        sim_math_zeroInt(buf_ind, nv);
        int current_nnz = 0;
        for (int i = 0; i < v_edge_cnt[v]; ++i) {
          int e = adj_edges[v_edge_adr[v] + i];
          int v1 = m->flex_edge[2 * (ebase + e)];
          int v2 = m->flex_edge[2 * (ebase + e) + 1];

          // chains from edge e
          int b1 = m->flex_vertbodyid[vbase + v1];
          int b2 = m->flex_vertbodyid[vbase + v2];
          int NV1 = sim_bodyChain(m, b1, chain1);
          int NV2 = sim_bodyChain(m, b2, chain2);

          for (int j = 0; j < NV1; ++j) {
            if (!buf_ind[chain1[j]]) {
              buf_ind[chain1[j]] = 1;
              current_nnz++;
            }
          }
          for (int j = 0; j < NV2; ++j) {
            if (!buf_ind[chain2[j]]) {
              buf_ind[chain2[j]] = 1;
              current_nnz++;
            }
          }
        }
        int row0 = 2 * (vbase + v);
        int row1 = 2 * (vbase + v) + 1;
        vrownnz[row0] = vrownnz[row1] = current_nnz;

        // set rowadr for next rows
        vrowadr[row1] = vrowadr[row0] + current_nnz;
        if (row1 + 1 < 2 * m->nflexvert) {
          vrowadr[row1 + 1] = vrowadr[row1] + current_nnz;
        }

        // fill colind
        int count = 0;
        for (int j = 0; j < nv; j++) {
          if (buf_ind[j]) {
            m->flexvert_J_colind[vrowadr[row0] + count] = j;
            m->flexvert_J_colind[vrowadr[row1] + count] = j;
            count++;
          }
        }
      }
    }
  }

  sim_freeStack(d);
}

// align 2D flexes to the XY plane
static void sim_alignFlex(sim_model_t* m, sim_data_t* d) {
  for (int f = 0; f < m->nflex; f++) {
    // only for 2D flexes with vertex equality constraints
    if (m->flex_dim[f] == 2 && m->flex_edgeequality[f] == 2) {
      // get element data
      int t_adr = m->flex_elemdataadr[f];
      int vbase = m->flex_vertadr[f];
      int t0 = m->flex_elem[t_adr];
      int t1 = m->flex_elem[t_adr + 1];
      int t2 = m->flex_elem[t_adr + 2];

      // compute normal from first element
      sim_scalar_t edge1[3], edge2[3], normal[3];
      sim_math_sub_3(edge1, m->flex_vert0 + 3 * (vbase + t1), m->flex_vert0 + 3 * (vbase + t0));
      sim_math_sub_3(edge2, m->flex_vert0 + 3 * (vbase + t2),
               m->flex_vert0 + 3 * (vbase + t0));
      sim_math_cross(normal, edge1, edge2);
      sim_math_normalize_3(normal);

      // compute rotation to Z
      sim_scalar_t quat[4], mat[9];
      sim_math_quatZ2Vec(quat, normal);
      sim_math_quat2Mat(mat, quat);

      // rotate all vertices of this flex
      int nvert = m->flex_vertnum[f];
      for (int v = 0; v < nvert; v++) {
        sim_scalar_t* vert = m->flex_vert0 + 3 * (vbase + v);
        sim_scalar_t res[3];

        sim_math_mulMatTVec3(res, mat, vert);
        sim_math_copy_3(vert, res);

        // check planarity (warning if not planar)
        if (sim_math_abs(vert[2] - m->flex_vert0[3 * (vbase + t0) + 2]) > 100 * SIM_MINVAL) {
          static int warned = 0;
          if (!warned) {
            warned = 1;
            sim_warning("flex %d is not planar", f);
          }
        }
      }
    }
  }
}

// set quantities that depend on qpos0
static void set0(sim_model_t* m, sim_data_t* d) {
  makeFlexSparse(m, d);
  sim_alignFlex(m, d);
  int nv = m->nv;
  sim_scalar_t A[36] = {0}, pos[3], quat[4];
  sim_markStack(d);
  sim_scalar_t* jac = SIM_STACK_ALLOC(d, 6*nv, sim_scalar_t);
  sim_scalar_t* tmp = SIM_STACK_ALLOC(d, 6*nv, sim_scalar_t);
  sim_scalar_t* moment = SIM_STACK_ALLOC(d, nv, sim_scalar_t);
  int* cammode = 0;
  int* lightmode = 0;

  // save camera and light mode, set to fixed
  if (m->ncam) {
    cammode = SIM_STACK_ALLOC(d, m->ncam, int);
    for (int i=0; i < m->ncam; i++) {
      cammode[i] = m->cam_mode[i];
      m->cam_mode[i] = SIM_CAMLIGHT_FIXED;
    }
  }
  if (m->nlight) {
    lightmode = SIM_STACK_ALLOC(d, m->nlight, int);
    for (int i=0; i < m->nlight; i++) {
      lightmode[i] = m->light_mode[i];
      m->light_mode[i] = SIM_CAMLIGHT_FIXED;
    }
  }

  // run computations in qpos0
  sim_math_copy(d->qpos, m->qpos0, m->nq);
  sim_kinematics(m, d);
  sim_comPos(m, d);
  sim_camlight(m, d);

  // compute dof_M0 for CRB algorithm
  sim_setM0(m, d);

  // save flex_rigid, temporarily make all flexes non-rigid
  sim_byte_t* rigid = NULL;
  if (m->nflex) {
    rigid = SIM_STACK_ALLOC(d, m->nflex, sim_byte_t);
    memcpy(rigid, m->flex_rigid, m->nflex);
    memset(m->flex_rigid, 0, m->nflex);
  }

  // run remaining computations
  sim_tendon(m, d);
  sim_makeM(m, d);
  sim_factorM(m, d);
  sim_flex(m, d);
  sim_transmission(m, d);

  // restore flex rigidity
  if (m->nflex) {
    memcpy(m->flex_rigid, rigid, m->nflex);
  }

  // restore camera and light mode
  for (int i=0; i < m->ncam; i++) {
    m->cam_mode[i] = cammode[i];
  }
  for (int i=0; i < m->nlight; i++) {
    m->light_mode[i] = lightmode[i];
  }

  // copy fields
  sim_math_copy(m->flexedge_length0, d->flexedge_length, m->nflexedge);
  sim_math_copy(m->tendon_length0, d->ten_length, m->ntendon);
  sim_math_copy(m->actuator_length0, d->actuator_length, m->nu);

  // compute body_invweight0
  m->body_invweight0[0] = m->body_invweight0[1] = 0.0;
  for (int i=1; i < m->nbody; i++) {
    // static bodies: zero invweight0
    if (m->body_weldid[i] == 0) {
      m->body_invweight0[2*i] = m->body_invweight0[2*i+1] = 0;
    }

    // accelerate simple bodies with no rotations
    else if (m->body_simple[i] == 2) {
      sim_scalar_t mass = m->body_mass[i];
      if (!mass) {  // SHOULD NOT OCCUR
        SIM_ERROR("moving body %d has 0 mass", i);
      }
      m->body_invweight0[2*i+0] = 1/sim_math_max(SIM_MINVAL, mass);
      m->body_invweight0[2*i+1] = 0;
    }

    // general body: full inertia
    else {
      if (nv) {
        // inverse spatial inertia: A = J*inv(M)*J'
        sim_jacBodyCom(m, d, jac, jac+3*nv, i);
        sim_solveM(m, d, tmp, jac, 6);
        sim_math_mulMatMatT(A, jac, tmp, 6, nv, 6);
      }

      // average diagonal and assign
      sim_scalar_t tran = (A[0] + A[7] + A[14])/3;
      sim_scalar_t rot = (A[21] + A[28] + A[35])/3;

      // if one is zero, use the other to prevent degenerate constraints
      if (tran < SIM_MINVAL && rot > SIM_MINVAL) {
        tran = rot;  // use rotation as fallback for translation
      } else if (rot < SIM_MINVAL && tran > SIM_MINVAL) {
        rot = tran;  // use translation as fallback for rotation
      }

      m->body_invweight0[2*i] = tran;
      m->body_invweight0[2*i+1] = rot;
    }
  }

  // compute dof_invweight0
  for (int i=0; i < m->njnt; i++) {
    // simple body with no rotations: no off-diagonal inertia
    if (m->body_simple[m->jnt_bodyid[i]] == 2) {
      int id = m->jnt_dofadr[i];
      int bi = m->jnt_bodyid[i];
      sim_scalar_t mass = m->body_mass[bi];
      if (!mass) {  // SHOULD NOT OCCUR
        SIM_ERROR("moving body %d has 0 mass", bi);
      }
      m->dof_invweight0[id] = 1/sim_math_max(SIM_MINVAL, mass);
    }

    // general joint: full inertia
    else {
      int dnum, id = m->jnt_dofadr[i];

      // get number of components
      if (m->jnt_type[i] == SIM_JNT_FREE) {
        dnum = 6;
      } else if (m->jnt_type[i] == SIM_JNT_BALL) {
        dnum = 3;
      } else {
        dnum = 1;
      }

      // inverse joint inertia:  A = J*inv(M)*J'
      if (nv) {
        sim_math_zero(jac, dnum*nv);
        for (int j=0; j < dnum; j++) {
          jac[j*(nv+1) + id] = 1;
        }
        sim_solveM(m, d, tmp, jac, dnum);
        sim_math_mulMatMatT(A, jac, tmp, dnum, nv, dnum);
      }

      // average diagonal and assign
      if (dnum == 6) {
        m->dof_invweight0[id] = m->dof_invweight0[id+1] = m->dof_invweight0[id+2] =
          (A[0] + A[7] + A[14])/3;
        m->dof_invweight0[id+3] = m->dof_invweight0[id+4] = m->dof_invweight0[id+5] =
          (A[21] + A[28] + A[35])/3;
      } else if (dnum == 3) {
        m->dof_invweight0[id] = m->dof_invweight0[id+1] = m->dof_invweight0[id+2] =
          (A[0] + A[4] + A[8])/3;
      } else {
        m->dof_invweight0[id] = A[0];
      }
    }
  }

  // compute flexedge_invweight0, tendon_invweight0, actuator_acc0
  if (nv) {
    // compute flexedge_invweight0
    for (int f=0; f < m->nflex; f++) {
      if (m->flex_interp[f]) {
        continue;
      }

      for (int i=m->flex_edgeadr[f]; i < m->flex_edgeadr[f]+m->flex_edgenum[f]; i++) {
        // bodies connected by edge
        int b1 = m->flex_vertbodyid[m->flex_vertadr[f] + m->flex_edge[2*i]];
        int b2 = m->flex_vertbodyid[m->flex_vertadr[f] + m->flex_edge[2*i+1]];

        // rigid edge: set to 0
        if (m->flexedge_rigid[i]) {
          m->flexedge_invweight0[i] = 0;
        }

        // accelerate edges that connect simple bodies with no rotations
        else if (m->body_simple[b1] == 2 && m->body_simple[b2] == 2) {
          m->flexedge_invweight0[i] = (1/m->body_mass[b1] + 1/m->body_mass[b2])/2;
        }

        // handle general edge
        else {
          // make dense vector into tmp
          sim_math_zero(tmp, nv);
          int end = m->flexedge_J_rowadr[i] + m->flexedge_J_rownnz[i];
          for (int j=m->flexedge_J_rowadr[i]; j < end; j++) {
            tmp[m->flexedge_J_colind[j]] = d->flexedge_J[j];
          }

          // solve into tmp+nv
          sim_solveM(m, d, tmp+nv, tmp, 1);
          m->flexedge_invweight0[i] = sim_math_dot(tmp, tmp+nv, nv);
        }
      }
    }

    // compute tendon_invweight0
    for (int i=0; i < m->ntendon; i++) {
      sim_math_sparse2dense(tmp, d->ten_J, 1, nv, d->ten_J_rownnz+i, d->ten_J_rowadr+i, d->ten_J_colind);

      // solve into tmp+nv
      sim_solveM(m, d, tmp+nv, tmp, 1);
      m->tendon_invweight0[i] = sim_math_dot(tmp, tmp+nv, nv);
    }

    // compute actuator_acc0
    for (int i=0; i < m->nu; i++) {
      sim_math_sparse2dense(moment, d->actuator_moment, 1, nv, d->moment_rownnz + i,
                       d->moment_rowadr + i, d->moment_colind);
      sim_solveM(m, d, tmp, moment, 1);
      m->actuator_acc0[i] = sim_math_norm(tmp, nv);
    }
  } else {
    sim_math_zero(m->tendon_invweight0, m->ntendon);
    sim_math_zero(m->actuator_acc0, m->nu);
  }

  // compute missing eq_data for body constraints
  for (int i=0; i < m->neq; i++) {
    // get ids
    int id1 = m->eq_obj1id[i];
    int id2 = m->eq_obj2id[i];

    // connect constraint
    if (m->eq_type[i] == SIM_EQ_CONNECT) {
      switch ((sim_obj_t) m->eq_objtype[i]) {
        case SIM_OBJ_BODY:
          // pos = anchor position in global frame
          sim_local2Global(d, pos, 0, m->eq_data+SIM_NEQDATA*i, 0, id1, 0);

          // data[3-5] = anchor position in body2 local frame
          sim_math_subFrom3(pos, d->xpos+3*id2);
          sim_math_mulMatTVec3(m->eq_data+SIM_NEQDATA*i+3, d->xmat+9*id2, pos);
          break;
        case SIM_OBJ_SITE:
          // site-based connect, eq_data is unused
          sim_math_zero(m->eq_data+SIM_NEQDATA*i, SIM_NEQDATA);
          break;
        default:
          SIM_ERROR("invalid objtype in connect constraint %d", i);
      }
    }

    // weld constraint
    else if (m->eq_type[i] == SIM_EQ_WELD) {
      switch ((sim_obj_t) m->eq_objtype[i]) {
        case SIM_OBJ_BODY: {
          // skip if user has set any quaternion data
          if (!sim_math_isZero(m->eq_data + SIM_NEQDATA*i + 6, 4)) {
            // normalize quaternion just in case
            sim_math_normalize4(m->eq_data+SIM_NEQDATA*i+6);
            continue;
          }

          // anchor position is in body2 local frame
          sim_local2Global(d, pos, 0, m->eq_data+SIM_NEQDATA*i, 0, id2, 0);

          // data[3-5] = anchor position in body1 local frame
          sim_math_subFrom3(pos, d->xpos+3*id1);
          sim_math_mulMatTVec3(m->eq_data+SIM_NEQDATA*i+3, d->xmat+9*id1, pos);

          // data[6-9] = neg(xquat1)*xquat2 = "xquat2-xquat1" in body1 local frame
          sim_math_negQuat(quat, d->xquat+4*id1);
          sim_math_mulQuat(m->eq_data+SIM_NEQDATA*i+6, quat, d->xquat+4*id2);
          break;
        }
        case SIM_OBJ_SITE: {
          break;
        }
        default:
          SIM_ERROR("invalid objtype in weld constraint %d", i);
      }
    }
  }

  // camera compos0, pos0, mat0
  for (int i=0; i < m->ncam; i++) {
    // get body ids
    int id = m->cam_bodyid[i];              // camera body
    int id1 = m->cam_targetbodyid[i];       // target body

    // compute positional offsets
    sim_math_sub_3(m->cam_pos0+3*i, d->cam_xpos+3*i, d->xpos+3*id);
    sim_math_sub_3(m->cam_poscom0+3*i, d->cam_xpos+3*i, d->subtree_com+ (id1 >= 0 ? 3*id1 : 3*id));

    // copy mat
    sim_math_copy9(m->cam_mat0+9*i, d->cam_xmat+9*i);
  }

  // light compos0, pos0, dir0
  for (int i=0; i < m->nlight; i++) {
    // get body ids
    int id = m->light_bodyid[i];            // light body
    int id1 = m->light_targetbodyid[i];     // target body

    // compute positional offsets
    sim_math_sub_3(m->light_pos0+3*i, d->light_xpos+3*i, d->xpos+3*id);
    sim_math_sub_3(m->light_poscom0+3*i, d->light_xpos+3*i, d->subtree_com + (id1 >= 0 ? 3*id1 : 3*id));

    // copy dir
    sim_math_copy_3(m->light_dir0+3*i, d->light_xdir+3*i);
  }

  // compute actuator damping from dampratio
  for (int i=0; i < m->nu; i++) {
    // get bias, gain parameters
    sim_scalar_t* biasprm = m->actuator_biasprm + i*SIM_NBIAS;
    sim_scalar_t* gainprm = m->actuator_gainprm + i*SIM_NGAIN;

    // not a position-like actuator: skip
    if (gainprm[0] != -biasprm[1]) {
      continue;
    }

    // damping is 0 or negative (interpreted as regular "kv"): skip
    if (biasprm[2] <= 0) {
      continue;
    }

    // === interpret biasprm[2] > 0 as dampratio for position-like actuators

    // "reflected" inertia (inversely scaled by transmission squared)
    int rownnz = d->moment_rownnz[i];
    int rowadr = d->moment_rowadr[i];
    sim_scalar_t* transmission = d->actuator_moment + rowadr;
    sim_scalar_t mass = 0;
    for (int j=0; j < rownnz; j++) {
      sim_scalar_t trn = sim_math_abs(transmission[j]);
      sim_scalar_t trn2 = trn*trn;  // transmission squared
      if (trn2 > SIM_MINVAL) {
        int dof = d->moment_colind[rowadr + j];
        mass += m->dof_M0[dof] / trn2;
      }
    }

    // damping = dampratio * 2 * sqrt(kp * mass)
    sim_scalar_t damping = biasprm[2] * 2 * sim_math_sqrt(gainprm[0] * mass);

    // set biasprm[2] to negative damping
    biasprm[2] = -damping;
  }

  sim_freeStack(d);
}


// accumulate bounding box
static void updateBox(sim_scalar_t* xmin, sim_scalar_t* xmax, sim_scalar_t* pos, sim_scalar_t radius) {
  for (int i=0; i < 3; i++) {
    xmin[i] = SIM_MIN(xmin[i], pos[i] - radius);
    xmax[i] = SIM_MAX(xmax[i], pos[i] + radius);
  }
}


// compute stat; assume computations already executed in qpos0
static void setStat(sim_model_t* m, sim_data_t* d) {
  sim_scalar_t xmin[3] = {1E+10, 1E+10, 1E+10};
  sim_scalar_t xmax[3] = {-1E+10, -1E+10, -1E+10};
  sim_scalar_t rbound;
  sim_markStack(d);

  // approximate length associated with each body
  sim_scalar_t* body = SIM_STACK_ALLOC(d, m->nbody, sim_scalar_t);

  // compute bounding box of bodies, joint centers, geoms and sites
  for (int i=1; i < m->nbody; i++) {
    updateBox(xmin, xmax, d->xpos+3*i, 0);
    updateBox(xmin, xmax, d->xipos+3*i, 0);
  }
  for (int i=0; i < m->njnt; i++) {
    updateBox(xmin, xmax, d->xanchor+3*i, 0);
  }
  for (int i=0; i < m->nsite; i++) {
    updateBox(xmin, xmax, d->site_xpos+3*i, 0);
  }
  for (int i=0; i < m->ngeom; i++) {
    // set rbound: regular geom rbound, or 0.1 of plane or hfield max size
    rbound = 0;
    if (m->geom_rbound[i] > 0) {
      rbound = m->geom_rbound[i];
    } else if (m->geom_type[i] == SIM_GEOM_PLANE) {
      // finite in at least one direction
      if (m->geom_size[3*i] || m->geom_size[3*i+1]) {
        rbound = SIM_MAX(m->geom_size[3*i], m->geom_size[3*i+1]) * 0.1;
      }

      // infinite in both directions
      else {
        rbound = 1;
      }
    } else if (m->geom_type[i] == SIM_GEOM_HFIELD) {
      int j = m->geom_dataid[i];
      rbound = SIM_MAX(m->hfield_size[4*j],
                     SIM_MAX(m->hfield_size[4*j+1],
                           SIM_MAX(m->hfield_size[4*j+2], m->hfield_size[4*j+3]))) * 0.1;
    }

    updateBox(xmin, xmax, d->geom_xpos+3*i, rbound);
  }

  // compute center
  sim_math_add3(m->stat.center, xmin, xmax);
  sim_math_scale_3(m->stat.center, m->stat.center, 0.5);

  // compute bounding box size
  if (xmax[0] > xmin[0])
    m->stat.extent = sim_math_max(1E-5,
                             sim_math_max(xmax[0]-xmin[0], sim_math_max(xmax[1]-xmin[1], xmax[2]-xmin[2])));

  // set body size to max com-joint distance
  sim_math_zero(body, m->nbody);
  for (int i=0; i < m->njnt; i++) {
    // handle this body
    int id = m->jnt_bodyid[i];
    body[id] = sim_math_max(body[id], sim_math_dist3(d->xipos+3*id, d->xanchor+3*i));

    // handle parent body
    id = m->body_parentid[id];
    body[id] = sim_math_max(body[id], sim_math_dist3(d->xipos+3*id, d->xanchor+3*i));
  }
  body[0] = 0;

  // set body size to max of old value, and geom rbound + com-geom dist
  for (int i=1; i < m->nbody; i++) {
    for (int id=m->body_geomadr[i]; id < m->body_geomadr[i]+m->body_geomnum[i]; id++) {
      if (m->geom_rbound[id] > 0) {
        body[i] = sim_math_max(body[i], m->geom_rbound[id] + sim_math_dist3(d->xipos+3*i, d->geom_xpos+3*id));
      }
    }
  }

  // adjust body size for flex edges involving body
  for (int f=0; f < m->nflex; f++) {
    if (m->flex_interp[f]) {
      for (int v1=m->flex_nodeadr[f]; v1 < m->flex_nodeadr[f]+m->flex_nodenum[f]; v1++) {
        for (int v2=m->flex_nodeadr[f]; v2 < m->flex_nodeadr[f]+m->flex_nodenum[f]; v2++) {
          sim_scalar_t edge = sim_math_dist3(d->xpos+3*m->flex_nodebodyid[v1],
                                  d->xpos+3*m->flex_nodebodyid[v2]);
          body[m->flex_nodebodyid[v1]] = sim_math_max(body[m->flex_nodebodyid[v1]], edge);
        }
      }
      continue;
    }
    for (int e=m->flex_edgeadr[f]; e < m->flex_edgeadr[f]+m->flex_edgenum[f]; e++) {
      int b1 = m->flex_vertbodyid[m->flex_vertadr[f]+m->flex_edge[2*e]];
      int b2 = m->flex_vertbodyid[m->flex_vertadr[f]+m->flex_edge[2*e+1]];

      body[b1] = sim_math_max(body[b1], m->flexedge_length0[e]);
      body[b2] = sim_math_max(body[b2], m->flexedge_length0[e]);
    }
  }

  // compute meansize, make sure all sizes are above min
  if (m->nbody > 1) {
    m->stat.meansize = 0;
    for (int i=1; i < m->nbody; i++) {
      body[i] = sim_math_max(body[i], 1E-5);
      m->stat.meansize += body[i]/(m->nbody-1);
    }
  }

  // inherit dof length from parent body
  for (int i=0; i < m->nv; i++) {
    // default to linear dof, already has length units
    m->dof_length[i] = 1;

    // if rotational dof, inherit from body
    int jnt = m->dof_jntid[i];
    SIM_tJoint type = m->jnt_type[jnt];
    int offset = i - m->jnt_dofadr[jnt];
    if (type == SIM_JNT_BALL  ||
        type == SIM_JNT_HINGE ||
        (type == SIM_JNT_FREE && offset >= 3)) {
      m->dof_length[i] = body[m->dof_bodyid[i]];
    }
  }

  // fix extent if too small compared to meanbody
  m->stat.extent = sim_math_max(m->stat.extent, 2 * m->stat.meansize);

  // compute meanmass
  if (m->nbody > 1) {
    m->stat.meanmass = 0;
    for (int i=1; i < m->nbody; i++) {
      m->stat.meanmass += m->body_mass[i];
    }
    m->stat.meanmass /= (m->nbody-1);
  }

  // compute meaninertia
  if (m->nv) {
    m->stat.meaninertia = 0;
    for (int i=0; i < m->nv; i++) {
      m->stat.meaninertia += d->qM[m->dof_Madr[i]];
    }
    m->stat.meaninertia /= m->nv;
  }

  sim_freeStack(d);
}


// set quantities that depend qpos_spring
static void setSpring(sim_model_t* m, sim_data_t* d) {
  // run computations in qpos_spring
  sim_math_copy(d->qpos, m->qpos_spring, m->nq);
  sim_kinematics(m, d);
  sim_comPos(m, d);
  sim_tendon(m, d);
  sim_transmission(m, d);

  // copy if model spring length is -1
  for (int i=0; i < m->ntendon; i++) {
    if (m->tendon_lengthspring[2*i] == -1 && m->tendon_lengthspring[2*i+1] == -1) {
      // explicit springlength unused, set equal to ten_length
      m->tendon_lengthspring[2*i] = m->tendon_lengthspring[2*i+1] = d->ten_length[i];
    }
  }
}


// entry point: set all remaining constant fields of sim_model_t, except for lengthrange
void sim_setConst(sim_model_t* m, sim_data_t* d) {
  // set fixed quantities
  setFixed(m, d);

  // set quantities that depend on qpos0
  set0(m, d);

  // compute statistics
  setStat(m, d);

  // set quantities that depend qpos_spring
  setSpring(m, d);
}


//----------------------------- actuator length range computation ----------------------------------

// evaluate actuator length, advance special dynamics
static sim_scalar_t evalAct(const sim_model_t* m, sim_data_t* d, int index, int side,
                      const SIM_LROpt* opt) {
  int nv = m->nv;

  // reduce velocity
  sim_math_scl(d->qvel, d->qvel, sim_math_exp(-m->opt.timestep/SIM_MAX(0.01, opt->timeconst)), nv);

  // step1: compute inertia and actuator moments
  sim_step1(m, d);

  // dense actuator_moment row
  sim_markStack(d);
  sim_scalar_t* moment = SIM_STACK_ALLOC(d, nv, sim_scalar_t);
  sim_math_sparse2dense(moment, d->actuator_moment, 1, nv, d->moment_rownnz + index,
                   d->moment_rowadr + index, d->moment_colind);

  // set force to generate desired acceleration
  sim_solveM(m, d, d->qfrc_applied, moment, 1);
  sim_scalar_t nrm = sim_math_norm(d->qfrc_applied, nv);
  sim_math_scl(d->qfrc_applied, moment, (2*side-1)*opt->accel/SIM_MAX(SIM_MINVAL, nrm), nv);

  // impose maxforce
  nrm = sim_math_norm(d->qfrc_applied, nv);
  if (opt->maxforce > 0 && nrm > opt->maxforce) {
    sim_math_scl(d->qfrc_applied, d->qfrc_applied, opt->maxforce/SIM_MAX(SIM_MINVAL, nrm), nv);
  }

  // step2: apply force
  sim_step2(m, d);

  sim_freeStack(d);

  // return actuator length
  return d->actuator_length[index];
}


// Set length range for specified actuator, return 1 if ok, 0 if error.
int sim_setLengthRange(sim_model_t* m, sim_data_t* d, int index,
                      const SIM_LROpt* opt, char* error, int error_sz) {
  // check index
  if (index < 0 || index >= m->nu) {
    SIM_ERROR("invalid actuator index");
  }

  // skip depending on mode and type
  int ismuscle = (m->actuator_gaintype[index] == SIM_GAIN_MUSCLE ||
                  m->actuator_biastype[index] == SIM_BIAS_MUSCLE);
  int isuser = (m->actuator_gaintype[index] == SIM_GAIN_USER ||
                m->actuator_biastype[index] == SIM_BIAS_USER);
  if ((opt->mode == SIM_LRMODE_NONE) ||
      (opt->mode == SIM_LRMODE_MUSCLE && !ismuscle) ||
      (opt->mode == SIM_LRMODE_MUSCLEUSER && !ismuscle && !isuser)) {
    return 1;
  }

  // use existing length range if available
  if (opt->useexisting && (m->actuator_lengthrange[2*index] < m->actuator_lengthrange[2*index+1])) {
    return 1;
  }

  // get transmission id
  int threadid = m->actuator_trnid[index];

  // use joint and tendon limits if available
  if (opt->uselimit) {
    // joint or jointinparent
    if (m->actuator_trntype[index] == SIM_TRN_JOINT ||
        m->actuator_trntype[index] == SIM_TRN_JOINTINPARENT) {
      // make sure joint is limited
      if (m->jnt_limited[threadid]) {
        // copy range
        m->actuator_lengthrange[2*index] = m->jnt_range[2*threadid];
        m->actuator_lengthrange[2*index+1] = m->jnt_range[2*threadid+1];

        // skip optimization
        return 1;
      }
    }

    // tendon
    if (m->actuator_trntype[index] == SIM_TRN_TENDON) {
      // make sure tendon is limited
      if (m->tendon_limited[threadid]) {
        // copy range
        m->actuator_lengthrange[2*index] = m->tendon_range[2*threadid];
        m->actuator_lengthrange[2*index+1] = m->tendon_range[2*threadid+1];

        // skip optimization
        return 1;
      }
    }
  }

  // optimize in both directions
  sim_scalar_t lmin[2] = {0, 0}, lmax[2] = {0, 0};
  int side;
  for (side=0; side < 2; side++) {
    // init at qpos0
    sim_resetData(m, d);

    // simulate
    int updated = 0;
    while (d->time < opt->inttotal) {
      // advance and get length
      sim_scalar_t len = evalAct(m, d, index, side, opt);

      // reset: cannot proceed
      if (d->time == 0) {
        snprintf(error, error_sz, "Unstable lengthrange simulation in actuator %d", index);
        return 0;
      }

      // update limits
      if (d->time > opt->inttotal-opt->interval) {
        if (len < lmin[side] || !updated) {
          lmin[side] = len;
        }
        if (len > lmax[side] || !updated) {
          lmax[side] = len;
        }

        updated = 1;
      }
    }

    // assign
    m->actuator_lengthrange[2*index+side] = (side == 0 ? lmin[side] : lmax[side]);
  }

  // check range
  sim_scalar_t dif = m->actuator_lengthrange[2*index+1] - m->actuator_lengthrange[2*index];
  if (dif <= 0) {
    snprintf(error, error_sz,
             "Invalid lengthrange (%g, %g) in actuator %d",
             m->actuator_lengthrange[2*index],
             m->actuator_lengthrange[2*index+1], index);
    return 0;
  }

  // check convergence, side 0
  if (lmax[0]-lmin[0] > opt->tolrange*dif) {
    snprintf(error, error_sz,
             "Lengthrange computation did not converge in actuator %d:\n"
             "  eval (%g, %g)\n  range (%g, %g)",
             index, lmin[0], lmax[0],
             m->actuator_lengthrange[2*index],
             m->actuator_lengthrange[2*index+1]);
    return 0;
  }

  // check convergence, side 1
  if (lmax[1]-lmin[1] > opt->tolrange*dif) {
    snprintf(error, error_sz,
             "Lengthrange computation did not converge in actuator %d:\n"
             "  eval (%g, %g)\n range (%g, %g)",
             index, lmin[1], lmax[1],
             m->actuator_lengthrange[2*index],
             m->actuator_lengthrange[2*index+1]);
    return 0;
  }

  return 1;
}
