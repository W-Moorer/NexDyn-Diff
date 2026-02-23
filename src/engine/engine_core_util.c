// Copyright 2025 DeepMind Technologies Limited
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

#include "engine/engine_core_util.h"

#include <stddef.h>

#include <simcore/SIM_data.h>
#include <simcore/SIM_model.h>
#include "engine/engine_inline.h"
#include "engine/engine_memory.h"
#include "engine/engine_util_blas.h"
#include "engine/engine_util_errmem.h"
#include "engine/engine_util_misc.h"
#include "engine/engine_util_sparse.h"
#include "engine/engine_util_spatial.h"



// determine type of constraint Jacobian
int sim_isSparse(const sim_model_t* m) {
  if (m->opt.jacobian == SIM_JAC_SPARSE ||
      (m->opt.jacobian == SIM_JAC_AUTO && m->nv >= 60)) {
    return 1;
  } else {
    return 0;
  }
}


// determine type of friction cone
int sim_isPyramidal(const sim_model_t* m) {
  if (m->opt.cone == SIM_CONE_PYRAMIDAL) {
    return 1;
  } else {
    return 0;
  }
}


//-------------------------- sparse chains ---------------------------------------------------------

// merge dof chains for two bodies
int sim_mergeChain(const sim_model_t* m, int* chain, int b1, int b2) {
  int da1, da2, NV = 0;

  // skip fixed bodies
  b1 = m->body_weldid[b1];
  b2 = m->body_weldid[b2];

  // neither body is movable: empty chain
  if (b1 == 0 && b2 == 0) {
    return 0;
  }

  // initialize last dof address for each body
  da1 = m->body_dofadr[b1] + m->body_dofnum[b1] - 1;
  da2 = m->body_dofadr[b2] + m->body_dofnum[b2] - 1;

  // merge chains
  while (da1 >= 0 || da2 >= 0) {
    chain[NV] = SIM_MAX(da1, da2);
    if (da1 == chain[NV]) {
      da1 = m->dof_parentid[da1];
    }
    if (da2 == chain[NV]) {
      da2 = m->dof_parentid[da2];
    }
    NV++;
  }

  // reverse order of chain: make it increasing
  for (int i=0; i < NV/2; i++) {
    int tmp = chain[i];
    chain[i] = chain[NV-i-1];
    chain[NV-i-1] = tmp;
  }

  return NV;
}


// merge dof chains for two simple bodies
int sim_mergeChainSimple(const sim_model_t* m, int* chain, int b1, int b2) {
  // swap bodies if wrong order
  if (b1 > b2) {
    int tmp = b1;
    b1 = b2;
    b2 = tmp;
  }

  // init
  int n1 = m->body_dofnum[b1];
  int n2 = m->body_dofnum[b2];

  // both fixed: nothing to do
  if (n1 == 0 && n2 == 0) {
    return 0;
  }

  // copy b1 dofs
  for (int i=0; i < n1; i++) {
    chain[i] = m->body_dofadr[b1] + i;
  }

  // copy b2 dofs
  for (int i=0; i < n2; i++) {
    chain[n1+i] = m->body_dofadr[b2] + i;
  }

  return (n1+n2);
}


// get body chain
int sim_bodyChain(const sim_model_t* m, int body, int* chain) {
  // simple body
  if (m->body_simple[body]) {
    int dofnum = m->body_dofnum[body];
    for (int i=0; i < dofnum; i++) {
      chain[i] = m->body_dofadr[body] + i;
    }
    return dofnum;
  }

  // general case
  else {
    // skip fixed bodies
    body = m->body_weldid[body];

    // not movable: empty chain
    if (body == 0) {
      return 0;
    }

    // initialize last dof
    int da = m->body_dofadr[body] + m->body_dofnum[body] - 1;
    int NV = 0;

    // construct chain from child to parent
    while (da >= 0) {
      chain[NV++] = da;
      da = m->dof_parentid[da];
    }

    // reverse order of chain: make it increasing
    for (int i=0; i < NV/2; i++) {
      int tmp = chain[i];
      chain[i] = chain[NV-i-1];
      chain[NV-i-1] = tmp;
    }

    return NV;
  }
}


//-------------------------- Jacobians -------------------------------------------------------------

// compute 3/6-by-nv Jacobian of global point attached to given body
void sim_jac(const sim_model_t* m, const sim_data_t* d,
            sim_scalar_t* jacp, sim_scalar_t* jacr, const sim_scalar_t point[3], int body) {
  int nv = m->nv;
  sim_scalar_t offset[3];

  // clear jacobians, compute offset if required
  if (jacp) {
    sim_math_zero(jacp, 3*nv);
    sim_math_sub_3(offset, point, d->subtree_com+3*m->body_rootid[body]);
  }
  if (jacr) {
    sim_math_zero(jacr, 3*nv);
  }

  // skip fixed bodies
  body = m->body_weldid[body];

  // no movable body found: nothing to do
  if (!body) {
    return;
  }

  // get last dof that affects this (as well as the original) body
  int i = m->body_dofadr[body] + m->body_dofnum[body] - 1;

  // backward pass over dof ancestor chain
  while (i >= 0) {
    sim_scalar_t* cdof = d->cdof+6*i;

    // construct rotation jacobian
    if (jacr) {
      jacr[i+0*nv] = cdof[0];
      jacr[i+1*nv] = cdof[1];
      jacr[i+2*nv] = cdof[2];
    }

    // construct translation jacobian (correct for rotation)
    if (jacp) {
      sim_scalar_t tmp[3];
      sim_math_internal_cross(tmp, cdof, offset);
      jacp[i+0*nv] = cdof[3] + tmp[0];
      jacp[i+1*nv] = cdof[4] + tmp[1];
      jacp[i+2*nv] = cdof[5] + tmp[2];
    }

    // advance to parent dof
    i = m->dof_parentid[i];
  }
}


// compute body Jacobian
void sim_jacBody(const sim_model_t* m, const sim_data_t* d, sim_scalar_t* jacp, sim_scalar_t* jacr, int body) {
  sim_jac(m, d, jacp, jacr, d->xpos+3*body, body);
}


// compute body-com Jacobian
void sim_jacBodyCom(const sim_model_t* m, const sim_data_t* d, sim_scalar_t* jacp, sim_scalar_t* jacr, int body) {
  sim_jac(m, d, jacp, jacr, d->xipos+3*body, body);
}


// compute subtree-com Jacobian
void sim_jacSubtreeCom(const sim_model_t* m, sim_data_t* d, sim_scalar_t* jacp, int body) {
  int nv = m->nv;
  sim_markStack(d);
  sim_scalar_t* jacp_b = SIM_STACK_ALLOC(d, 3*nv, sim_scalar_t);

  // clear output
  sim_math_zero(jacp, 3*nv);

  // forward pass starting from body
  for (int b=body; b < m->nbody; b++) {
    // end of body subtree, break from the loop
    if (b > body && m->body_parentid[b] < body) {
      break;
    }

    // b is in the body subtree, add mass-weighted Jacobian into jacp
    sim_jac(m, d, jacp_b, NULL, d->xipos+3*b, b);
    sim_math_addToScl(jacp, jacp_b, m->body_mass[b], 3*nv);
  }

  // normalize by subtree mass
  sim_math_scl(jacp, jacp, 1/m->body_subtreemass[body], 3*nv);

  sim_freeStack(d);
}


// compute geom Jacobian
void sim_jacGeom(const sim_model_t* m, const sim_data_t* d, sim_scalar_t* jacp, sim_scalar_t* jacr, int geom) {
  sim_jac(m, d, jacp, jacr, d->geom_xpos + 3*geom, m->geom_bodyid[geom]);
}


// compute site Jacobian
void sim_jacSite(const sim_model_t* m, const sim_data_t* d, sim_scalar_t* jacp, sim_scalar_t* jacr, int site) {
  sim_jac(m, d, jacp, jacr, d->site_xpos + 3*site, m->site_bodyid[site]);
}


// compute translation Jacobian of point, and rotation Jacobian of axis
void sim_jacPointAxis(const sim_model_t* m, sim_data_t* d, sim_scalar_t* jacPoint, sim_scalar_t* jacAxis,
                     const sim_scalar_t point[3], const sim_scalar_t axis[3], int body) {
  int nv = m->nv;

  // get full Jacobian of point
  sim_markStack(d);
  sim_scalar_t* jacp = (jacPoint ? jacPoint : SIM_STACK_ALLOC(d, 3*nv, sim_scalar_t));
  sim_scalar_t* jacr = SIM_STACK_ALLOC(d, 3*nv, sim_scalar_t);
  sim_jac(m, d, jacp, jacr, point, body);

  // jacAxis_col = cross(jacr_col, axis)
  if (jacAxis) {
    for (int i=0; i < nv; i++) {
      jacAxis[     i] = jacr[  nv+i]*axis[2] - jacr[2*nv+i]*axis[1];
      jacAxis[  nv+i] = jacr[2*nv+i]*axis[0] - jacr[     i]*axis[2];
      jacAxis[2*nv+i] = jacr[     i]*axis[1] - jacr[  nv+i]*axis[0];
    }
  }

  sim_freeStack(d);
}


// compute 3/6-by-nv sparse Jacobian of global point attached to given body
void sim_jacSparse(const sim_model_t* m, const sim_data_t* d,
                  sim_scalar_t* jacp, sim_scalar_t* jacr, const sim_scalar_t* point, int body,
                  int NV, const int* chain) {
  // clear jacobians
  if (jacp) {
    sim_math_zero(jacp, 3*NV);
  }
  if (jacr) {
    sim_math_zero(jacr, 3*NV);
  }

  // compute point-com offset
  sim_scalar_t offset[3];
  sim_math_sub_3(offset, point, d->subtree_com+3*m->body_rootid[body]);

  // skip fixed bodies
  body = m->body_weldid[body];

  // no movable body found: nothing to do
  if (!body) {
    return;
  }

  // get last dof that affects this (as well as the original) body
  int da = m->body_dofadr[body] + m->body_dofnum[body] - 1;

  // start and the end of the chain (chain is in increasing order)
  int ci = NV-1;

  // backward pass over dof ancestor chain
  while (da >= 0) {
    // find chain index for this dof
    while (ci >= 0 && chain[ci] > da) {
      ci--;
    }

    // make sure we found it; SHOULD NOT OCCUR
    if (ci < 0 || chain[ci] != da) {
      SIM_ERROR("dof index %d not found in chain", da);
    }

    const sim_scalar_t* cdof = d->cdof + 6*da;

    // construct rotation jacobian
    if (jacr) {
      jacr[ci+0*NV] = cdof[0];
      jacr[ci+1*NV] = cdof[1];
      jacr[ci+2*NV] = cdof[2];
    }

    // construct translation jacobian (correct for rotation)
    if (jacp) {
      sim_scalar_t tmp[3];
      sim_math_internal_cross(tmp, cdof, offset);

      jacp[ci+0*NV] = cdof[3] + tmp[0];
      jacp[ci+1*NV] = cdof[4] + tmp[1];
      jacp[ci+2*NV] = cdof[5] + tmp[2];
    }

    // advance to parent dof
    da = m->dof_parentid[da];
  }
}


// sparse Jacobian difference for simple body contacts
void sim_jacSparseSimple(const sim_model_t* m, const sim_data_t* d,
                        sim_scalar_t* jacdifp, sim_scalar_t* jacdifr, const sim_scalar_t* point,
                        int body, int flg_second, int NV, int start) {
  // compute point-com offset
  sim_scalar_t offset[3];
  sim_math_sub_3(offset, point, d->subtree_com+3*m->body_rootid[body]);

  // skip fixed body
  if (!m->body_dofnum[body]) {
    return;
  }

  // process dofs
  int ci = start;
  int end = m->body_dofadr[body] + m->body_dofnum[body];
  for (int da=m->body_dofadr[body]; da < end; da++) {
    sim_scalar_t *cdof = d->cdof+6*da;

    // construct rotation jacobian
    if (jacdifr) {
      // plus sign
      if (flg_second) {
        jacdifr[ci+0*NV] = cdof[0];
        jacdifr[ci+1*NV] = cdof[1];
        jacdifr[ci+2*NV] = cdof[2];
      }

      // minus sign
      else {
        jacdifr[ci+0*NV] = -cdof[0];
        jacdifr[ci+1*NV] = -cdof[1];
        jacdifr[ci+2*NV] = -cdof[2];
      }
    }

    // construct translation jacobian (correct for rotation)
    if (jacdifp) {
      sim_scalar_t tmp[3];
      sim_math_internal_cross(tmp, cdof, offset);

      // plus sign
      if (flg_second) {
        jacdifp[ci+0*NV] = (cdof[3] + tmp[0]);
        jacdifp[ci+1*NV] = (cdof[4] + tmp[1]);
        jacdifp[ci+2*NV] = (cdof[5] + tmp[2]);
      }

      // minus sign
      else {
        jacdifp[ci+0*NV] = -(cdof[3] + tmp[0]);
        jacdifp[ci+1*NV] = -(cdof[4] + tmp[1]);
        jacdifp[ci+2*NV] = -(cdof[5] + tmp[2]);
      }
    }

    // advance jacdif counter
    ci++;
  }
}


// dense or sparse Jacobian difference for two body points: pos2 - pos1, global
int sim_jacDifPair(const sim_model_t* m, const sim_data_t* d, int* chain,
                  int b1, int b2, const sim_scalar_t pos1[3], const sim_scalar_t pos2[3],
                  sim_scalar_t* jac1p, sim_scalar_t* jac2p, sim_scalar_t* jacdifp,
                  sim_scalar_t* jac1r, sim_scalar_t* jac2r, sim_scalar_t* jacdifr, int issparse) {
  int issimple = (m->body_simple[b1] && m->body_simple[b2]);
  int NV = m->nv;

  // skip if no DOFs
  if (!NV) {
    return 0;
  }

  // construct merged chain of body dofs
  if (issparse) {
    if (issimple) {
      NV = sim_mergeChainSimple(m, chain, b1, b2);
    } else {
      NV = sim_mergeChain(m, chain, b1, b2);
    }
  }

  // skip if empty chain
  if (!NV) {
    return 0;
  }

  // sparse case
  if (issparse) {
    // simple: fast processing
    if (issimple) {
      // first body
      sim_jacSparseSimple(m, d, jacdifp, jacdifr, pos1, b1, 0, NV,
                         b1 < b2 ? 0 : m->body_dofnum[b2]);

      // second body
      sim_jacSparseSimple(m, d, jacdifp, jacdifr, pos2, b2, 1, NV,
                         b2 < b1 ? 0 : m->body_dofnum[b1]);
    }

    // regular processing
    else {
      // Jacobians
      sim_jacSparse(m, d, jac1p, jac1r, pos1, b1, NV, chain);
      sim_jacSparse(m, d, jac2p, jac2r, pos2, b2, NV, chain);

      // differences
      if (jacdifp) {
        sim_math_sub(jacdifp, jac2p, jac1p, 3*NV);
      }
      if (jacdifr) {
        sim_math_sub(jacdifr, jac2r, jac1r, 3*NV);
      }
    }
  }

  // dense case
  else {
    // Jacobians
    sim_jac(m, d, jac1p, jac1r, pos1, b1);
    sim_jac(m, d, jac2p, jac2r, pos2, b2);

    // differences
    if (jacdifp) {
      sim_math_sub(jacdifp, jac2p, jac1p, 3*NV);
    }
    if (jacdifr) {
      sim_math_sub(jacdifr, jac2r, jac1r, 3*NV);
    }
  }

  return NV;
}


// dense or sparse weighted sum of multiple body Jacobians at same point
int sim_jacSum(const sim_model_t* m, sim_data_t* d, int* chain,
              int n, const int* body, const sim_scalar_t* weight,
              const sim_scalar_t point[3], sim_scalar_t* jac, int flg_rot) {
  int nv = m->nv, NV;
  sim_scalar_t* jacp = jac;
  sim_scalar_t* jacr = flg_rot ? jac + 3*nv : NULL;

  sim_markStack(d);
  sim_scalar_t* jtmp = SIM_STACK_ALLOC(d, flg_rot ? 6*nv : 3*nv, sim_scalar_t);
  sim_scalar_t* jp = jtmp;
  sim_scalar_t* jr = flg_rot ? jtmp + 3*nv : NULL;

  // sparse
  if (sim_isSparse(m)) {
    sim_scalar_t* buf = SIM_STACK_ALLOC(d, flg_rot ? 6*nv : 3*nv, sim_scalar_t);
    int* buf_ind = SIM_STACK_ALLOC(d, nv, int);
    int* bodychain = SIM_STACK_ALLOC(d, nv, int);

    // set first
    NV = sim_bodyChain(m, body[0], chain);
    if (NV) {
      // get Jacobian
      if (m->body_simple[body[0]]) {
        sim_jacSparseSimple(m, d, jacp, jacr, point, body[0], 1, NV, 0);
      } else {
        sim_jacSparse(m, d, jacp, jacr, point, body[0], NV, chain);
      }

      // apply weight
      sim_math_scl(jac, jac, weight[0], flg_rot ? 6*NV : 3*NV);
    }

    // accumulate remaining
    for (int i=1; i < n; i++) {
      // get body chain and Jacobian
      int bodyNV = sim_bodyChain(m, body[i], bodychain);
      if (!bodyNV) {
        continue;
      }
      if (m->body_simple[body[i]]) {
        sim_jacSparseSimple(m, d, jp, jr, point, body[i], 1, bodyNV, 0);
      } else {
        sim_jacSparse(m, d, jp, jr, point, body[i], bodyNV, bodychain);
      }

      // combine sparse matrices
      NV = sim_math_addToSparseMat(jac, jtmp, nv, flg_rot ? 6 : 3, weight[i],
                              NV, bodyNV, chain, bodychain, buf, buf_ind);
    }
  }

  // dense
  else {
    // set first
    sim_jac(m, d, jacp, jacr, point, body[0]);
    sim_math_scl(jac, jac, weight[0], flg_rot ? 6*nv : 3*nv);

    // accumulate remaining
    for (int i=1; i < n; i++) {
      sim_jac(m, d, jp, jr, point, body[i]);
      sim_math_addToScl(jac, jtmp, weight[i], flg_rot ? 6*nv : 3*nv);
    }

    NV = nv;
  }

  sim_freeStack(d);

  return NV;
}


// compute 3/6-by-nv Jacobian time derivative of global point attached to given body
void sim_jacDot(const sim_model_t* m, const sim_data_t* d,
               sim_scalar_t* jacp, sim_scalar_t* jacr, const sim_scalar_t point[3], int body) {
  int nv = m->nv;
  sim_scalar_t offset[3];
  sim_scalar_t pvel[6];  // point velocity (rot:lin order)

  // clear jacobians, compute offset and pvel if required
  if (jacp) {
    sim_math_zero(jacp, 3*nv);
    const sim_scalar_t* com = d->subtree_com+3*m->body_rootid[body];
    sim_math_sub_3(offset, point, com);
    sim_math_transformSpatial(pvel, d->cvel+6*body, 0, point, com, 0);
  }
  if (jacr) {
    sim_math_zero(jacr, 3*nv);
  }

  // skip fixed bodies
  body = m->body_weldid[body];

  // no movable body found: nothing to do
  if (!body) {
    return;
  }

  // get last dof that affects this (as well as the original) body
  int i = m->body_dofadr[body] + m->body_dofnum[body] - 1;

  // backward pass over dof ancestor chain
  while (i >= 0) {
    sim_scalar_t cdof_dot[6];
    sim_math_internal_copy6(cdof_dot, d->cdof_dot+6*i);
    sim_scalar_t* cdof = d->cdof+6*i;

    // check for quaternion
    SIM_tJoint type = m->jnt_type[m->dof_jntid[i]];
    int dofadr = m->jnt_dofadr[m->dof_jntid[i]];
    int is_quat = type == SIM_JNT_BALL || (type == SIM_JNT_FREE && i >= dofadr + 3);

    // compute cdof_dot for quaternion (use current body cvel)
    if (is_quat) {
      sim_math_internal_crossMotion(cdof_dot, d->cvel+6*m->dof_bodyid[i], cdof);
    }

    // construct rotation jacobian
    if (jacr) {
      jacr[i+0*nv] += cdof_dot[0];
      jacr[i+1*nv] += cdof_dot[1];
      jacr[i+2*nv] += cdof_dot[2];
    }

    // construct translation jacobian (correct for rotation)
    if (jacp) {
      // first correction term, account for varying cdof
      sim_scalar_t tmp1[3];
      sim_math_internal_cross(tmp1, cdof_dot, offset);

      // second correction term, account for point translational velocity
      sim_scalar_t tmp2[3];
      sim_math_internal_cross(tmp2, cdof, pvel + 3);

      jacp[i+0*nv] += cdof_dot[3] + tmp1[0] + tmp2[0];
      jacp[i+1*nv] += cdof_dot[4] + tmp1[1] + tmp2[1];
      jacp[i+2*nv] += cdof_dot[5] + tmp1[2] + tmp2[2];
    }

    // advance to parent dof
    i = m->dof_parentid[i];
  }
}


// compute subtree angular momentum matrix
void sim_angmomMat(const sim_model_t* m, sim_data_t* d, sim_scalar_t* mat, int body) {
  int nv = m->nv;
  sim_markStack(d);

  // stack allocations
  sim_scalar_t* jacp = SIM_STACK_ALLOC(d, 3*nv, sim_scalar_t);
  sim_scalar_t* jacr = SIM_STACK_ALLOC(d, 3*nv, sim_scalar_t);
  sim_scalar_t* term1 = SIM_STACK_ALLOC(d, 3*nv, sim_scalar_t);
  sim_scalar_t* term2 = SIM_STACK_ALLOC(d, 3*nv, sim_scalar_t);

  // clear output
  sim_math_zero(mat, 3*nv);

  // save the location of the subtree COM
  sim_scalar_t subtree_com[3];
  sim_math_copy_3(subtree_com, d->subtree_com+3*body);

  for (int b=body; b < m->nbody; b++) {
    // end of body subtree, break from the loop
    if (b > body && m->body_parentid[b] < body) {
      break;
    }

    // linear and angular velocity Jacobian of the body COM (inertial frame)
    sim_jacBodyCom(m, d, jacp, jacr, b);

    // orientation of the COM (inertial) frame of b-th body
    sim_scalar_t ximat[9];
    sim_math_internal_copy9(ximat, d->ximat+9*b);

    // save the inertia matrix of b-th body
    sim_scalar_t inertia[9] = {0};
    inertia[0] = m->body_inertia[3*b];   // inertia(1,1)
    inertia[4] = m->body_inertia[3*b+1]; // inertia(2,2)
    inertia[8] = m->body_inertia[3*b+2]; // inertia(3,3)

    // term1 = body angular momentum about self COM in world frame
    sim_scalar_t tmp1[9], tmp2[9];
    sim_math_internal_mulMatMat3(tmp1, ximat, inertia);          // tmp1  = ximat * inertia
    sim_math_mulMatMatT3(tmp2, tmp1, ximat);            // tmp2  = ximat * inertia * ximat^T
    sim_math_mulMatMat(term1, tmp2, jacr, 3, 3, nv);    // term1 = ximat * inertia * ximat^T * jacr

    // location of body COM w.r.t subtree COM
    sim_scalar_t com[3];
    sim_math_internal_sub3(com, d->xipos+3*b, subtree_com);

    // skew symmetric matrix representing body_com vector
    sim_scalar_t com_mat[9] = {0};
    com_mat[1] = -com[2];
    com_mat[2] = com[1];
    com_mat[3] = com[2];
    com_mat[5] = -com[0];
    com_mat[6] = -com[1];
    com_mat[7] = com[0];

    // term2 = moment of linear momentum
    sim_math_mulMatMat(term2, com_mat, jacp, 3, 3, nv);   // term2 = com_mat * jacp
    sim_math_scl(term2, term2, m->body_mass[b], 3 * nv);  // term2 = com_mat * jacp * mass

    // mat += term1 + term2
    sim_math_addTo(mat, term1, 3*nv);
    sim_math_addTo(mat, term2, 3*nv);
  }

  sim_freeStack(d);
}


//-------------------------- spatial frame utilities -----------------------------------------------

// compute object 6D velocity in object-centered frame, world/local orientation
void sim_objectVelocity(const sim_model_t* m, const sim_data_t* d,
                       int objtype, int objid, sim_scalar_t res[6], int flg_local) {
  int bodyid = 0;
  const sim_scalar_t *pos = 0, *rot = 0;

  // body-inertial
  if (objtype == SIM_OBJ_BODY) {
    bodyid = objid;
    pos = d->xipos+3*objid;
    rot = (flg_local ? d->ximat+9*objid : 0);
  }

  // body-regular
  else if (objtype == SIM_OBJ_XBODY) {
    bodyid = objid;
    pos = d->xpos+3*objid;
    rot = (flg_local ? d->xmat+9*objid : 0);
  }

  // geom
  else if (objtype == SIM_OBJ_GEOM) {
    bodyid = m->geom_bodyid[objid];
    pos = d->geom_xpos+3*objid;
    rot = (flg_local ? d->geom_xmat+9*objid : 0);
  }

  // site
  else if (objtype == SIM_OBJ_SITE) {
    bodyid = m->site_bodyid[objid];
    pos = d->site_xpos+3*objid;
    rot = (flg_local ? d->site_xmat+9*objid : 0);
  }

  // camera
  else if (objtype == SIM_OBJ_CAMERA) {
    bodyid = m->cam_bodyid[objid];
    pos = d->cam_xpos+3*objid;
    rot = (flg_local ? d->cam_xmat+9*objid : 0);
  }

  // object without spatial frame
  else {
    SIM_ERROR("invalid object type %d", objtype);
  }

  // static body: quick return
  if (m->body_weldid[bodyid] == 0) {
    sim_math_zero(res, 6);
    return;
  }

  // transform velocity
  sim_math_transformSpatial(res, d->cvel+6*bodyid, 0, pos, d->subtree_com+3*m->body_rootid[bodyid], rot);
}


// compute object 6D acceleration in object-centered frame, world/local orientation
void sim_objectAcceleration(const sim_model_t* m, const sim_data_t* d,
                           int objtype, int objid, sim_scalar_t res[6], int flg_local) {
  int bodyid = 0;
  const sim_scalar_t *pos = 0, *rot = 0;

  // body-inertial
  if (objtype == SIM_OBJ_BODY) {
    bodyid = objid;
    pos = d->xipos+3*objid;
    rot = (flg_local ? d->ximat+9*objid : 0);
  }

  // body-regular
  else if (objtype == SIM_OBJ_XBODY) {
    bodyid = objid;
    pos = d->xpos+3*objid;
    rot = (flg_local ? d->xmat+9*objid : 0);
  }

  // geom
  else if (objtype == SIM_OBJ_GEOM) {
    bodyid = m->geom_bodyid[objid];
    pos = d->geom_xpos+3*objid;
    rot = (flg_local ? d->geom_xmat+9*objid : 0);
  }

  // site
  else if (objtype == SIM_OBJ_SITE) {
    bodyid = m->site_bodyid[objid];
    pos = d->site_xpos+3*objid;
    rot = (flg_local ? d->site_xmat+9*objid : 0);
  }

  // camera
  else if (objtype == SIM_OBJ_CAMERA) {
    bodyid = m->cam_bodyid[objid];
    pos = d->cam_xpos+3*objid;
    rot = (flg_local ? d->cam_xmat+9*objid : 0);
  }

  // object without spatial frame
  else {
    SIM_ERROR("invalid object type %d", objtype);
  }

  // static body: quick return
  if (m->body_weldid[bodyid] == 0) {
    sim_math_zero(res, 6);
    return;
  }

  // transform com-based acceleration to local frame
  sim_math_transformSpatial(res, d->cacc+6*bodyid, 0, pos, d->subtree_com+3*m->body_rootid[bodyid], rot);

  // transform com-based velocity to local frame
  sim_scalar_t vel[6];
  sim_math_transformSpatial(vel, d->cvel+6*bodyid, 0, pos, d->subtree_com+3*m->body_rootid[bodyid], rot);

  // add Coriolis correction due to rotating frame:  acc_tran += vel_rot x vel_tran
  sim_scalar_t correction[3];
  sim_math_internal_cross(correction, vel, vel+3);
  sim_math_internal_addTo3(res+3, correction);
}


// map from body local to global Cartesian coordinates
void sim_local2Global(sim_data_t* d, sim_scalar_t xpos[3], sim_scalar_t xmat[9],
                     const sim_scalar_t pos[3], const sim_scalar_t quat[4],
                     int body, sim_byte_t sameframe) {
  SIM_tSameFrame sf = sameframe;

  // position
  if (xpos && pos) {
    switch (sf) {
    case SIM_SAMEFRAME_NONE:
    case SIM_SAMEFRAME_BODYROT:
    case SIM_SAMEFRAME_INERTIAROT:
      sim_math_internal_mulMatVec3(xpos, d->xmat+9*body, pos);
      sim_math_internal_addTo3(xpos, d->xpos+3*body);
      break;
    case SIM_SAMEFRAME_BODY:
      sim_math_internal_copy_3(xpos, d->xpos+3*body);
      break;
    case SIM_SAMEFRAME_INERTIA:
      sim_math_internal_copy_3(xpos, d->xipos+3*body);
      break;
    }
  }

  // orientation
  if (xmat && quat) {
    sim_scalar_t tmp[4];
    switch (sf) {
    case SIM_SAMEFRAME_NONE:
      sim_math_internal_mulQuat(tmp, d->xquat+4*body, quat);
      sim_math_quat2Mat(xmat, tmp);
      break;
    case SIM_SAMEFRAME_BODY:
    case SIM_SAMEFRAME_BODYROT:
      sim_math_internal_copy9(xmat, d->xmat+9*body);
      break;
    case SIM_SAMEFRAME_INERTIA:
    case SIM_SAMEFRAME_INERTIAROT:
      sim_math_internal_copy9(xmat, d->ximat+9*body);
      break;
    }
  }
}


//-------------------------- miscellaneous utilities -----------------------------------------------

// extract 6D force:torque for one contact, in contact frame
void sim_contactForce(const sim_model_t* m, const sim_data_t* d, int id, sim_scalar_t result[6]) {
  sim_contact_t* con;

  // clear result
  sim_math_zero(result, 6);

  // make sure contact is valid
  if (id >= 0 && id < d->ncon && d->contact[id].efc_address >= 0) {
    // get contact pointer
    con = d->contact + id;

    if (sim_isPyramidal(m)) {
      sim_math_decodePyramid(result, d->efc_force + con->efc_address, con->friction, con->dim);
    } else {
      sim_math_copy(result, d->efc_force + con->efc_address, con->dim);
    }
  }
}


// count the number of length limit violations for tendon i (0, 1 or 2)
int tendonLimit(const sim_model_t* m, const sim_scalar_t* ten_length, int i) {
  if (!m->tendon_limited[i]) {
    return 0;
  }

  int nl = 0;
  sim_scalar_t value = ten_length[i];
  sim_scalar_t margin = m->tendon_margin[i];

  // tendon limits can be bilateral, check both sides
  for (int side = -1; side <= 1; side += 2) {
    sim_scalar_t dist = side * (m->tendon_range[2 * i + (side + 1) / 2] - value);
    if (dist < margin) nl++;
  }

  return nl;
}


// count warnings, print only the first time
void sim_runtime_warning(sim_data_t* d, int warning, int info) {
  // check type
  if (warning < 0 || warning >= SIM_NWARNING) {
    SIM_ERROR("invalid warning type %d", warning);
  }

  // save info (override previous)
  d->warning[warning].lastinfo = info;

  // print message only the first time this warning is encountered
  if (!d->warning[warning].number) {
    sim_warning("%s Time = %.4f.", sim_math_warningText(warning, info), d->time);
  }

  // increase counter
  d->warning[warning].number++;
}

