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

#include "engine/engine_core_constraint.h"

#include <stdio.h>
#include <stddef.h>

#include <simcore/SIM_data.h>
#include <simcore/SIM_macro.h>
#include <simcore/SIM_model.h>
#include <simcore/SIM_san.h>  // IWYU pragma: keep
#include <simcore/SIM_xmacro.h>
#include "engine/engine_init.h"
#include "engine/engine_core_util.h"
#include "engine/engine_core_smooth.h"
#include "engine/engine_memory.h"
#include "engine/engine_sleep.h"
#include "engine/engine_util_blas.h"
#include "engine/engine_util_errmem.h"
#include "engine/engine_util_misc.h"
#include "engine/engine_util_sparse.h"
#include "engine/engine_util_spatial.h"

#ifdef MEMORY_SANITIZER
  #include <sanitizer/msan_interface.h>
#endif

#ifdef SIM_USEPLATFORMSIMD
  #if defined(__AVX__) && !defined(SIM_USESINGLE)
    #define SIM_USEAVX
  #endif  // defined(__AVX__) && !defined(SIM_USESINGLE)
#endif  // SIM_USEPLATFORMSIMD


//-------------------------- utility functions -----------------------------------------------------


// allocate efc arrays on arena, return 1 on success, 0 on failure
static int arenaAllocEfc(const sim_model_t* m, sim_data_t* d) {
#undef SIM_M
#define SIM_M(n) m->n
#undef SIM_D
#define SIM_D(n) d->n

  // move arena pointer to end of contact array
  d->parena = d->ncon * sizeof(sim_contact_t);

  // poison remaining memory
#ifdef ADDRESS_SANITIZER
  ASAN_POISON_MEMORY_REGION(
    (char*)d->arena + d->parena, d->narena - d->pstack - d->parena);
#endif

#define X(type, name, nr, nc)                                                 \
  d->name = sim_arenaAllocByte(d, sizeof(type) * (nr) * (nc), _Alignof(type)); \
  if (!d->name) {                                                             \
    sim_runtime_warning(d, SIM_WARN_CNSTRFULL, d->narena);                               \
    sim_clearEfc(d);                                                           \
    d->parena = d->ncon * sizeof(sim_contact_t);                                  \
    return 0;                                                                 \
  }

  SIMDATA_ARENA_POINTERS_SOLVER
#undef X

#undef SIM_M
#define SIM_M(n) n
#undef SIM_D
#define SIM_D(n) n

  return 1;
}


// determine type of solver
int sim_isDual(const sim_model_t* m) {
  if (m->opt.solver == SIM_SOL_PGS || m->opt.noslip_iterations > 0) {
    return 1;
  } else {
    return 0;
  }
}


// assign/clamp contact friction parameters
void sim_assignFriction(const sim_model_t* m, sim_scalar_t* target, const sim_scalar_t* source) {
  if (SIM_ENABLED(SIM_ENBL_OVERRIDE)) {
    for (int i=0; i < 5; i++) {
      target[i] = sim_math_max(SIM_MINMU, m->opt.o_friction[i]);
    }
  } else {
    for (int i=0; i < 5; i++) {
      target[i] = sim_math_max(SIM_MINMU, source[i]);
    }
  }
}



// assign/override contact reference parameters
void sim_assignRef(const sim_model_t* m, sim_scalar_t* target, const sim_scalar_t* source) {
  if (SIM_ENABLED(SIM_ENBL_OVERRIDE)) {
    sim_math_copy(target, m->opt.o_solref, SIM_NREF);
  } else {
    sim_math_copy(target, source, SIM_NREF);
  }
}


// assign/override contact impedance parameters
void sim_assignImp(const sim_model_t* m, sim_scalar_t* target, const sim_scalar_t* source) {
  if (SIM_ENABLED(SIM_ENBL_OVERRIDE)) {
    sim_math_copy(target, m->opt.o_solimp, SIM_NIMP);
  } else {
    sim_math_copy(target, source, SIM_NIMP);
  }
}


// assign/override contact margin
sim_scalar_t sim_assignMargin(const sim_model_t* m, sim_scalar_t source) {
  if (SIM_ENABLED(SIM_ENBL_OVERRIDE)) {
    return m->opt.o_margin;
  } else {
    return source;
  }
}


// compute element bodies and weights for given contact point, return #bodies
// if v is one of the element vertices, reduce element to fragment
static int sim_elemBodyWeight(const sim_model_t* m, const sim_data_t* d, int f, int e, int v,
                             const sim_scalar_t point[3], int* body, sim_scalar_t* weight) {
  // get flex info
  int dim = m->flex_dim[f];
  const int* edata = m->flex_elem + m->flex_elemdataadr[f] + e*(dim+1);
  const sim_scalar_t* vert = d->flexvert_xpos + 3*m->flex_vertadr[f];

  // compute inverse distances from contact point to element vertices
  // save body ids, find vertex v in element
  int vid = -1;
  for (int i=0; i <= dim; i++) {
    sim_scalar_t dist = sim_math_dist3(point, vert+3*edata[i]);
    weight[i] = 1.0/(sim_math_max(SIM_MINVAL, dist));
    body[i] = m->flex_vertadr[f] + edata[i];

    // check if element vertex matches v
    if (edata[i] == v) {
      vid = i;
    }
  }

  // v found in e: skip and shift remaining
  if (vid >= 0) {
    while (vid < dim) {
      weight[vid] = weight[vid+1];
      body[vid] = body[vid+1];
      vid++;
    }
    dim--;
  }

  // normalize weights
  sim_scalar_t sum = sim_math_sum(weight, dim+1);
  if (sum < SIM_MINVAL) {
    SIM_ERROR("element body weight sum < SIM_MINVAL");
  }
  sim_math_scl(weight, weight, 1.0/sum, dim+1);
  return dim+1;
}


// compute body weights for a given contact vertex, return #bodies
static int sim_vertBodyWeight(const sim_model_t* m, const sim_data_t* d, int f, int* v,
                             int* body, sim_scalar_t* bweight, const sim_scalar_t* vweight, int nw) {
  if (nw == 0) {
    return 0;
  }

  sim_scalar_t coord[3] = {0, 0, 0};
  for (int i = 0; i < nw; i++) {
    sim_math_add_to_scale_3(coord,  m->flex_vert0 + 3*v[i], vweight[i]);
  }
  int nstart = m->flex_nodeadr[f];
  int nend = m->flex_nodeadr[f] + m->flex_nodenum[f];
  int nb = 0;

  for (int i = nstart; i < nend; i++) {
    sim_scalar_t w = sim_math_evalBasis(coord, i-nstart, m->flex_interp[f]);
    if (w < 1e-5) {
      continue;
    }
    if (bweight) bweight[nb] = w;
    body[nb++] = m->flex_nodebodyid[i];
  }

  return nb;
}


// add contact to d->contact list; return 0 if success; 1 if buffer full
int sim_addContact(const sim_model_t* m, sim_data_t* d, const sim_contact_t* con) {
  // move arena pointer back to the end of the existing contact array and invalidate efc_ arrays
  d->parena = d->ncon * sizeof(sim_contact_t);
#ifdef ADDRESS_SANITIZER
  ASAN_POISON_MEMORY_REGION(
    (char*)d->arena + d->parena, d->narena - d->pstack - d->parena);
#endif
  sim_clearEfc(d);

  // copy contact
  sim_contact_t* dst = sim_arenaAllocByte(d, sizeof(sim_contact_t), _Alignof(sim_contact_t));
  if (!dst) {
    sim_runtime_warning(d, SIM_WARN_CONTACTFULL, d->ncon);
    return 1;
  }
  *dst = *con;

  // increase counter, return success
  d->ncon++;
  return 0;
}


// add #size rows to constraint Jacobian; set pos, margin, frictionloss, type, id
static void sim_addConstraint(const sim_model_t* m, sim_data_t* d,
                             const sim_scalar_t* jac, const sim_scalar_t* pos,
                             const sim_scalar_t* margin, sim_scalar_t frictionloss,
                             int size, int type, int id, int NV, const int* chain) {
  int empty, nv = m->nv, nefc = d->nefc;
  int *nnz = d->efc_J_rownnz, *adr = d->efc_J_rowadr, *ind = d->efc_J_colind;
  sim_scalar_t *J = d->efc_J;

  // init empty guard for constraints other than contact
  if (type == SIM_CNSTR_CONTACT_FRICTIONLESS ||
      type == SIM_CNSTR_CONTACT_PYRAMIDAL ||
      type == SIM_CNSTR_CONTACT_ELLIPTIC) {
    empty = 0;
  } else {
    empty = 1;
  }

  // dense: copy entire Jacobian
  if (!sim_isSparse(m)) {
    // make sure jac is not empty
    if (empty) {
      for (int i=0; i < size*nv; i++) {
        if (jac[i]) {
          empty = 0;
          break;
        }
      }
    }

    // copy if not empty
    if (!empty) {
      sim_math_copy(J + nefc*nv, jac, size*nv);
    }
  }

  // sparse: copy chain
  else {
    // clamp NV (in case -1 was used in constraint construction)
    NV = SIM_MAX(0, NV);

    if (NV) {
      empty = 0;
    } else if (empty) {
      // all rows are empty, return early
      return;
    }

    // chain required in sparse mode
    if (NV && !chain) {
      SIM_ERROR("called with dense arguments");
    }

    // process size elements
    for (int i=0; i < size; i++) {
      // set row address
      adr[nefc+i] = (nefc+i ? adr[nefc+i-1]+nnz[nefc+i-1] : 0);

      // set row descriptor
      nnz[nefc+i] = NV;

      // copy if not empty
      if (NV) {
        sim_math_copyInt(ind + adr[nefc+i], chain, NV);
        sim_math_copy(J + adr[nefc+i], jac + i*NV, NV);
      }
    }
  }

  // all rows empty: skip constraint
  if (empty) {
    return;
  }

  // set constraint pos, margin, frictionloss, type, id
  for (int i=0; i < size; i++) {
    d->efc_pos[nefc+i] = (pos ? pos[i] : 0);
    d->efc_margin[nefc+i] = (margin ? margin[i] : 0);
    d->efc_frictionloss[nefc+i] = frictionloss;
    d->efc_type[nefc+i] = type;
    d->efc_id[nefc+i] = id;
  }

  // increase counters
  d->nefc += size;
  if (type == SIM_CNSTR_EQUALITY) {
    d->ne += size;
  } else if (type == SIM_CNSTR_FRICTION_DOF || type == SIM_CNSTR_FRICTION_TENDON) {
    d->nf += size;
  } else if (type == SIM_CNSTR_LIMIT_JOINT || type == SIM_CNSTR_LIMIT_TENDON) {
    d->nl += size;
  }
}


// multiply Jacobian by vector
void sim_mulJacVec(const sim_model_t* m, const sim_data_t* d, sim_scalar_t* res, const sim_scalar_t* vec) {
  // exit if no constraints
  if (!d->nefc) {
    return;
  }

  // sparse Jacobian
  if (sim_isSparse(m))
    sim_math_mulMatVecSparse(res, d->efc_J, vec, d->nefc,
                        d->efc_J_rownnz, d->efc_J_rowadr,
                        d->efc_J_colind, d->efc_J_rowsuper);

  // dense Jacobian
  else {
    sim_math_mulMatVec(res, d->efc_J, vec, d->nefc, m->nv);
  }
}


// multiply JacobianT by vector
void sim_mulJacTVec(const sim_model_t* m, const sim_data_t* d, sim_scalar_t* res, const sim_scalar_t* vec) {
  // exit if no constraints
  if (!d->nefc) {
    return;
  }

  // sparse Jacobian
  if (sim_isSparse(m)) {
    sim_math_mulMatTVecSparse(res, d->efc_J, vec, d->nefc, m->nv,
                        d->efc_J_rownnz, d->efc_J_rowadr, d->efc_J_colind);
  }

  // dense Jacobian
  else {
    sim_math_mulMatTVec(res, d->efc_J, vec, d->nefc, m->nv);
  }
}


//--------------------- instantiate constraints by type --------------------------------------------

// equality constraints
void sim_instantiateEquality(const sim_model_t* m, sim_data_t* d) {
  int issparse = sim_isSparse(m), nv = m->nv;
  int id[2], size, NV, NV2, *chain = NULL, *chain2 = NULL, *buf_ind = NULL;
  int flex_edgeadr, flex_edgenum;
  int flex_vertadr, flex_vertnum;
  sim_scalar_t cpos[6], pos[2][3], ref[2], dif, deriv;
  sim_scalar_t quat[4], quat1[4], quat2[4], quat3[4], axis[3];
  sim_scalar_t *jac[2], *jacdif, *data, *sparse_buf = NULL;

  // disabled or no equality constraints: return
  if (SIM_DISABLED(SIM_DSBL_EQUALITY) || m->nemax == 0) {
    return;
  }

  // sleep filtering
  int sleep_filter = SIM_ENABLED(SIM_ENBL_SLEEP) && d->ntree_awake < m->ntree;

  sim_markStack(d);

  // allocate space
  jac[0] = SIM_STACK_ALLOC(d, 6*nv, sim_scalar_t);
  jac[1] = SIM_STACK_ALLOC(d, 6*nv, sim_scalar_t);
  jacdif = SIM_STACK_ALLOC(d, 6*nv, sim_scalar_t);
  if (issparse) {
    chain = SIM_STACK_ALLOC(d, nv, int);
    chain2 = SIM_STACK_ALLOC(d, nv, int);
    buf_ind = SIM_STACK_ALLOC(d, nv, int);
    sparse_buf = SIM_STACK_ALLOC(d, nv, sim_scalar_t);
  }

  // find active equality constraints
  for (int i=0; i < m->neq; i++) {
    // skip inactive
    if (!d->eq_active[i]) {
      continue;
    }

    // skip sleeping
    if (sleep_filter && sim_sleepState(m, d, SIM_OBJ_EQUALITY, i) == sim_spec_ASLEEP) {
      continue;
    }

    // get constraint data
    data = m->eq_data + SIM_NEQDATA*i;
    id[0] = m->eq_obj1id[i];
    id[1] = m->eq_obj2id[i];
    size = 0;
    NV = 0;
    NV2 = 0;
    int body_id[2];

    // process according to type
    switch ((SIM_tEq) m->eq_type[i]) {
    case SIM_EQ_CONNECT:              // connect bodies with ball joint
      // find global points, body semantic
      if (m->eq_objtype[i] == SIM_OBJ_BODY) {
        for (int j=0; j < 2; j++) {
          sim_math_mul_mat_vec_3(pos[j], d->xmat + 9*id[j], data + 3*j);
          sim_math_add_to_3(pos[j], d->xpos + 3*id[j]);
          body_id[j] = id[j];
        }
      }

      // find global points, site semantic
      else {
        for (int j=0; j < 2; j++) {
          sim_math_copy_3(pos[j], d->site_xpos + 3*id[j]);
          body_id[j] = m->site_bodyid[id[j]];
        }
      }

      // compute position error
      sim_math_sub_3(cpos, pos[0], pos[1]);

      // compute Jacobian difference (opposite of contact: 0 - 1)
      NV = sim_jacDifPair(m, d, chain, body_id[1], body_id[0], pos[1], pos[0],
                          jac[1], jac[0], jacdif, NULL, NULL, NULL, issparse);

      // copy difference into jac[0]
      sim_math_copy(jac[0], jacdif, 3*NV);

      size = 3;
      break;

    case SIM_EQ_WELD:                 // fix relative position and orientation
      // find global points, body semantic
      if (m->eq_objtype[i] == SIM_OBJ_BODY) {
        for (int j=0; j < 2; j++) {
          sim_scalar_t* anchor = data + 3*(1-j);
          sim_math_mul_mat_vec_3(pos[j], d->xmat + 9*id[j], anchor);
          sim_math_add_to_3(pos[j], d->xpos + 3*id[j]);
          body_id[j] = id[j];
        }
      }

      // find global points, site semantic
      else {
        for (int j=0; j < 2; j++) {
          sim_math_copy_3(pos[j], d->site_xpos + 3*id[j]);
          body_id[j] = m->site_bodyid[id[j]];
        }
      }

      // compute position error
      sim_math_sub_3(cpos, pos[0], pos[1]);

      // get torquescale coefficient
      sim_scalar_t torquescale = data[10];

      // compute error Jacobian (opposite of contact: 0 - 1)
      NV = sim_jacDifPair(m, d, chain, body_id[1], body_id[0], pos[1], pos[0],
                          jac[1], jac[0], jacdif,
                          jac[1]+3*nv, jac[0]+3*nv, jacdif+3*nv, issparse);

      // copy difference into jac[0], compress translation:rotation if sparse
      sim_math_copy(jac[0], jacdif, 3*NV);
      sim_math_copy(jac[0]+3*NV, jacdif+3*nv, 3*NV);

      // orientation, body semantic
      if (m->eq_objtype[i] == SIM_OBJ_BODY) {
        // compute orientation error: neg(q1) * q0 * relpose (axis components only)
        sim_scalar_t* relpose = data+6;
        sim_math_mulQuat(quat, d->xquat+4*id[0], relpose);   // quat = q0*relpose
        sim_math_negQuat(quat1, d->xquat+4*id[1]);           // quat1 = neg(q1)
      }

      // orientation, site semantic
      else {
        sim_scalar_t quat_site1[4];
        sim_math_mulQuat(quat, d->xquat+4*body_id[0], m->site_quat+4*id[0]);
        sim_math_mulQuat(quat_site1, d->xquat+4*body_id[1], m->site_quat+4*id[1]);
        sim_math_negQuat(quat1, quat_site1);
      }

      sim_math_mulQuat(quat2, quat1, quat);
      sim_math_scale_3(cpos+3, quat2+1, torquescale);         // scale axis components by torquescale

      // correct rotation Jacobian: 0.5 * neg(q1) * (jac0-jac1) * q0 * relpose
      for (int j=0; j < NV; j++) {
        // axis = [jac0-jac1]_col(j)
        axis[0] = jac[0][3*NV+j];
        axis[1] = jac[0][4*NV+j];
        axis[2] = jac[0][5*NV+j];

        // apply formula
        sim_math_mulQuatAxis(quat2, quat1, axis);    // quat2 = neg(q1)*(jac0-jac1)
        sim_math_mulQuat(quat3, quat2, quat);        // quat3 = neg(q1)*(jac0-jac1)*q0*relpose

        // correct Jacobian
        jac[0][3*NV+j] = 0.5*quat3[1];
        jac[0][4*NV+j] = 0.5*quat3[2];
        jac[0][5*NV+j] = 0.5*quat3[3];
      }

      // scale rotational jacobian by torquescale
      sim_math_scl(jac[0]+3*NV, jac[0]+3*NV, torquescale, 3*NV);

      size = 6;
      break;

    case SIM_EQ_JOINT:                // couple joint values with cubic
    case SIM_EQ_TENDON:               // couple tendon lengths with cubic
      // get scalar positions and their Jacobians
      for (int j=0; j < 1+(id[1] >= 0); j++) {
        if (m->eq_type[i] == SIM_EQ_JOINT) {    // joint object
          pos[j][0] = d->qpos[m->jnt_qposadr[id[j]]];
          ref[j] = m->qpos0[m->jnt_qposadr[id[j]]];

          // make Jacobian: sparse or dense
          if (issparse) {
            // add first or second joint
            if (j == 0) {
              NV = 1;
              chain[0] = m->jnt_dofadr[id[j]];
              jac[j][0] = 1;
            } else {
              NV2 = 1;
              chain2[0] = m->jnt_dofadr[id[j]];
              jac[j][0] = 1;
            }
          } else {
            sim_math_zero(jac[j], nv);
            jac[j][m->jnt_dofadr[id[j]]] = 1;
          }
        } else {                            // tendon object
          pos[j][0] = d->ten_length[id[j]];
          ref[j] = m->tendon_length0[id[j]];

          // set tendon_efcadr
          if (d->tendon_efcadr[id[j]] == -1) {
            d->tendon_efcadr[id[j]] = i;
          }

          // copy Jacobian: sparse or dense
          if (issparse) {
            if (j == 0) {
              NV = d->ten_J_rownnz[id[j]];
              sim_math_copyInt(chain, d->ten_J_colind+d->ten_J_rowadr[id[j]], NV);
              sim_math_copy(jac[j], d->ten_J+d->ten_J_rowadr[id[j]], NV);
            } else {
              NV2 = d->ten_J_rownnz[id[j]];
              sim_math_copyInt(chain2, d->ten_J_colind+d->ten_J_rowadr[id[j]], NV2);
              sim_math_copy(jac[j], d->ten_J+d->ten_J_rowadr[id[j]], NV2);
            }
          } else {
            sim_math_sparse2dense(jac[j], d->ten_J, 1, nv, d->ten_J_rownnz+id[j], d->ten_J_rowadr+id[j], d->ten_J_colind);
          }
        }
      }

      // both objects defined
      if (id[1] >= 0) {
        // compute position error
        dif = pos[1][0] - ref[1];
        cpos[0] = pos[0][0] - ref[0] - data[0] -
                  (data[1]*dif + data[2]*dif*dif + data[3]*dif*dif*dif + data[4]*dif*dif*dif*dif);

        // compute derivative
        deriv = data[1] + 2*data[2]*dif + 3*data[3]*dif*dif + 4*data[4]*dif*dif*dif;

        // compute Jacobian: sparse or dense
        if (issparse) {
          NV = sim_math_combineSparse(jac[0], jac[1], 1, -deriv, NV, NV2, chain,
                                  chain2, sparse_buf, buf_ind);
        } else {
          sim_math_addToScl(jac[0], jac[1], -deriv, nv);
        }
      }

      // only one object defined
      else {
        // compute position error
        cpos[0] = pos[0][0] - ref[0] - data[0];

        // jac[0] already has the correct Jacobian
      }

      size = 1;
      break;

    case SIM_EQ_FLEX:
      flex_edgeadr = m->flex_edgeadr[id[0]];
      flex_edgenum = m->flex_edgenum[id[0]];
      // add one constraint per non-rigid edge
      for (int e=flex_edgeadr; e < flex_edgeadr+flex_edgenum; e++) {
        // skip rigid
        if (m->flexedge_rigid[e]) {
          continue;
        }

        // position error
        cpos[0] = d->flexedge_length[e] - m->flexedge_length0[e];

        // add constraint: sparse or dense
        if (issparse) {
          sim_addConstraint(m, d, d->flexedge_J+m->flexedge_J_rowadr[e], cpos, 0, 0,
                           1, SIM_CNSTR_EQUALITY, i,
                           m->flexedge_J_rownnz[e],
                           m->flexedge_J_colind+m->flexedge_J_rowadr[e]);
        } else {
          sim_math_zero(jac[0], nv);  // reuse first row of jac[0]
          int rowadr = m->flexedge_J_rowadr[e];
          int rownnz = m->flexedge_J_rownnz[e];
          for (int k=0; k<rownnz; k++) {
            jac[0][m->flexedge_J_colind[rowadr+k]] = d->flexedge_J[rowadr+k];
          }
          sim_addConstraint(m, d, jac[0], cpos, 0, 0, 1, SIM_CNSTR_EQUALITY, i, 0, NULL);
        }
      }
      break;

    case SIM_EQ_FLEXVERT:
      // add two constraints per vertex
      flex_vertadr = m->flex_vertadr[id[0]];
      flex_vertnum = m->flex_vertnum[id[0]];
      for (int v=flex_vertadr; v < flex_vertadr+flex_vertnum; v++) {
        for (int j=0; j < 2; j++) {
          cpos[0] = d->flexvert_length[2*v+j];
          int row = 2*v+j;
          if (issparse) {
            sim_addConstraint(m, d, d->flexvert_J + m->flexvert_J_rowadr[row],
                             cpos, 0, 0, 1, SIM_CNSTR_EQUALITY, i,
                             m->flexvert_J_rownnz[row],
                             m->flexvert_J_colind + m->flexvert_J_rowadr[row]);
          } else {
            sim_math_zero(jac[0], nv);  // reuse first row of jac[0]
            int rowadr = m->flexvert_J_rowadr[row];
            int rownnz = m->flexvert_J_rownnz[row];
            for (int k=0; k<rownnz; k++) {
              jac[0][m->flexvert_J_colind[rowadr+k]] = d->flexvert_J[rowadr+k];
            }
            sim_addConstraint(m, d, jac[0], cpos, 0, 0, 1, SIM_CNSTR_EQUALITY, i, 0, NULL);
          }
        }
      }
      break;

    default:                    // SHOULD NOT OCCUR
      SIM_ERROR("invalid equality constraint type %d", m->eq_type[i]);
    }

    // add constraint
    if (size) {
      sim_addConstraint(m, d, jac[0], cpos, 0, 0,
                       size, SIM_CNSTR_EQUALITY, i,
                       issparse ? NV : 0,
                       issparse ? chain : NULL);
    }
  }

  sim_freeStack(d);
}


// frictional dofs and tendons
void sim_instantiateFriction(const sim_model_t* m, sim_data_t* d) {
  int nv = m->nv, issparse = sim_isSparse(m);
  sim_scalar_t* jac;

  // disabled: return
  if (SIM_DISABLED(SIM_DSBL_FRICTIONLOSS)) {
    return;
  }

  // sleep filtering
  int sleep_filter = SIM_ENABLED(SIM_ENBL_SLEEP) && d->ntree_awake < m->ntree;

  sim_markStack(d);

  // allocate Jacobian
  jac = SIM_STACK_ALLOC(d, nv, sim_scalar_t);

  // find frictional dofs
  for (int i=0; i < nv; i++) {
    // no friction loss: skip
    if (!m->dof_frictionloss[i]) {
      continue;
    }

    // sleeping tree: skip
    if (sleep_filter && sim_sleepState(m, d, SIM_OBJ_DOF, i) == sim_spec_ASLEEP) {
      continue;
    }

    // prepare Jacobian: sparse or dense
    if (issparse) {
      jac[0] = 1;
    } else {
      sim_math_zero(jac, nv);
      jac[i] = 1;
    }

    // add constraint
    sim_addConstraint(m, d, jac, 0, 0, m->dof_frictionloss[i],
                      1, SIM_CNSTR_FRICTION_DOF, i,
                      issparse ? 1 : 0,
                      issparse ? &i : NULL);
  }

  // find frictional tendons
  for (int i=0; i < m->ntendon; i++) {
    if (m->tendon_frictionloss[i] > 0) {
      int efcadr = d->nefc;
      // add constraint
      if (issparse) {
        sim_addConstraint(m, d, d->ten_J + d->ten_J_rowadr[i],
                         0, 0, m->tendon_frictionloss[i],
                         1, SIM_CNSTR_FRICTION_TENDON, i,
                         d->ten_J_rownnz[i],
                         d->ten_J_colind+d->ten_J_rowadr[i]);
      } else {
        sim_math_sparse2dense(jac, d->ten_J, 1, nv, d->ten_J_rownnz+i, d->ten_J_rowadr+i, d->ten_J_colind);
        sim_addConstraint(m, d, jac, 0, 0, m->tendon_frictionloss[i],
                         1, SIM_CNSTR_FRICTION_TENDON, i, 0, NULL);
      }
      // set tendon_efcadr
      if (d->tendon_efcadr[i] == -1) {
        d->tendon_efcadr[i] = efcadr;
      }
    }
  }

  sim_freeStack(d);
}


// joint and tendon limits
void sim_instantiateLimit(const sim_model_t* m, sim_data_t* d) {
  int nv = m->nv, issparse = sim_isSparse(m);
  sim_scalar_t margin, value, dist, angleAxis[3];
  sim_scalar_t *jac;

  // disabled: return
  if (SIM_DISABLED(SIM_DSBL_LIMIT)) {
    return;
  }

  // sleep filtering
  int sleep_filter = SIM_ENABLED(SIM_ENBL_SLEEP) && d->ntree_awake < m->ntree;

  sim_markStack(d);

  // allocate Jacobian
  jac = SIM_STACK_ALLOC(d, nv, sim_scalar_t);

  // find joint limits
  for (int i=0; i < m->njnt; i++) {
    // no limit: skip
    if (!m->jnt_limited[i]) {
      continue;
    }

    // sleeping tree: skip
    if (sleep_filter && sim_sleepState(m, d, SIM_OBJ_JOINT, i) == sim_spec_ASLEEP) {
      continue;
    }

    // get margin
    margin = m->jnt_margin[i];

    // HINGE or SLIDE joint
    if (m->jnt_type[i] == SIM_JNT_SLIDE || m->jnt_type[i] == SIM_JNT_HINGE) {
      // get joint value
      value = d->qpos[m->jnt_qposadr[i]];

      // process lower and upper limits
      for (int side=-1; side <= 1; side+=2) {
        // compute distance (negative: penetration)
        dist = side * (m->jnt_range[2*i+(side+1)/2] - value);

        // detect joint limit
        if (dist < margin) {
          // prepare Jacobian: sparse or dense
          if (issparse) {
            jac[0] = -(sim_scalar_t)side;
          } else {
            sim_math_zero(jac, nv);
            jac[m->jnt_dofadr[i]] = -(sim_scalar_t)side;
          }

          // add constraint
          sim_addConstraint(m, d, jac, &dist, &margin, 0,
                           1, SIM_CNSTR_LIMIT_JOINT, i,
                           issparse ? 1 : 0,
                           issparse ? m->jnt_dofadr+i : NULL);
        }
      }
    }

    // BALL joint
    else if (m->jnt_type[i] == SIM_JNT_BALL) {
      // convert joint quaternion to axis-angle
      int adr = m->jnt_qposadr[i];
      sim_scalar_t quat[4] = {d->qpos[adr], d->qpos[adr+1], d->qpos[adr+2], d->qpos[adr+3]};
      sim_math_normalize4(quat);
      sim_math_quat2Vel(angleAxis, quat, 1);

      // get rotation angle, normalize
      value = sim_math_normalize_3(angleAxis);

      // compute distance, using max of range (negative: penetration)
      dist = sim_math_max(m->jnt_range[2*i], m->jnt_range[2*i+1]) - value;

      // detect joint limit
      if (dist < margin) {
        // sparse
        if (issparse) {
          // prepare dof index array
          int chain[3] = {
            m->jnt_dofadr[i] + 0,
            m->jnt_dofadr[i] + 1,
            m->jnt_dofadr[i] + 2
          };

          // prepare Jacobian
          sim_math_scale_3(jac, angleAxis, -1);

          // add constraint
          sim_addConstraint(m, d, jac, &dist, &margin, 0,
                           1, SIM_CNSTR_LIMIT_JOINT, i, 3, chain);
        }

        // dense
        else {
          // prepare Jacobian
          sim_math_zero(jac, nv);
          sim_math_scale_3(jac + m->jnt_dofadr[i], angleAxis, -1);

          // add constraint
          sim_addConstraint(m, d, jac, &dist, &margin, 0,
                           1, SIM_CNSTR_LIMIT_JOINT, i, 0, 0);
        }
      }
    }
  }

  // find tendon limits
  for (int i=0; i < m->ntendon; i++) {
    if (m->tendon_limited[i]) {
      // get value = length, margin
      value = d->ten_length[i];
      margin = m->tendon_margin[i];

      // process lower and upper limits
      for (int side=-1; side <= 1; side+=2) {
        // compute distance (negative: penetration)
        dist = side * (m->tendon_range[2*i+(side+1)/2] - value);

        // detect tendon limit
        if (dist < margin) {
          // prepare Jacobian
          int efcadr = d->nefc;
          if (issparse) {
            sim_math_scl(jac, d->ten_J+d->ten_J_rowadr[i], -side, d->ten_J_rownnz[i]);
            sim_addConstraint(m, d, jac, &dist, &margin, 0,
                             1, SIM_CNSTR_LIMIT_TENDON, i,
                             d->ten_J_rownnz[i],
                             d->ten_J_colind+d->ten_J_rowadr[i]);
          } else {
            sim_math_sparse2dense(jac, d->ten_J, 1, nv, d->ten_J_rownnz+i, d->ten_J_rowadr+i, d->ten_J_colind);
            sim_math_scl(jac, jac, -side, nv);
            sim_addConstraint(m, d, jac, &dist, &margin, 0,
                             1, SIM_CNSTR_LIMIT_TENDON, i, 0, NULL);
          }
          // set tendon_efcadr
          if (d->tendon_efcadr[i] == -1) {
            d->tendon_efcadr[i] = efcadr;
          }
        }
      }
    }
  }

  sim_freeStack(d);
}


// compute Jacobian for contact, return number of DOFs affected
int sim_contactJacobian(const sim_model_t* m, sim_data_t* d, const sim_contact_t* con, int dim,
                       sim_scalar_t* jac, sim_scalar_t* jacdif, sim_scalar_t* jacdifp,
                       sim_scalar_t* jacdifr, sim_scalar_t* jac1p, sim_scalar_t* jac2p,
                       sim_scalar_t* jac1r, sim_scalar_t* jac2r, int* chain) {
  // special case: single body on each side
  if ((con->geom[0] >= 0 || (con->vert[0] >= 0 && m->flex_interp[con->flex[0]] == 0)) &&
      (con->geom[1] >= 0 || (con->vert[1] >= 0 && m->flex_interp[con->flex[1]] == 0))) {
    // get bodies
    int bid[2];
    for (int side=0; side < 2; side++) {
      bid[side] = (con->geom[side] >= 0) ?
                  m->geom_bodyid[con->geom[side]] :
                  m->flex_vertbodyid[m->flex_vertadr[con->flex[side]] + con->vert[side]];
    }

    // compute Jacobian differences
    if (dim > 3) {
      return sim_jacDifPair(m, d, chain, bid[0], bid[1], con->pos, con->pos,
                           jac1p, jac2p, jacdifp, jac1r, jac2r, jacdifr, sim_isSparse(m));
    } else {
      return sim_jacDifPair(m, d, chain, bid[0], bid[1], con->pos, con->pos,
                           jac1p, jac2p, jacdifp, NULL, NULL, NULL, sim_isSparse(m));
    }
  }

  // general case: flex elements involved
  else {
    // get bodies and weights
    int nb = 0;
    int bid[729];  // 729 = 27*27
    sim_scalar_t bweight[729];
    for (int side=0; side < 2; side++) {
      // geom
      if (con->geom[side] >= 0) {
        bid[nb] = m->geom_bodyid[con->geom[side]];
        bweight[nb] = side ? +1 : -1;
        nb++;
      }

      // flex
      else {
        int nw = 0;
        int vid[4];
        sim_scalar_t vweight[4];

        // vert
        if (con->vert[side] >= 0) {
          vid[0] = m->flex_vertadr[con->flex[side]] + con->vert[side];
          vweight[0] = side ? +1 : -1;
          nw = 1;
        }

        // elem
        else {
          nw = sim_elemBodyWeight(m, d, con->flex[side], con->elem[side],
                                con->vert[1-side], con->pos, vid, vweight);

          // negative sign for first side of contact
          if (side == 0) {
            sim_math_scl(vweight, vweight, -1, nw);
          }
        }

        // get body or node ids and weights
        if (m->flex_interp[con->flex[side]] == 0) {
          for (int k=0; k < nw; k++) {
            bid[nb] = m->flex_vertbodyid[vid[k]];
            bweight[nb] = vweight[k];
            nb++;
          }
        } else {
          nb += sim_vertBodyWeight(m, d, con->flex[side], vid, bid+nb, bweight+nb, vweight, nw);
        }
      }
    }

    // combine weighted Jacobians
    return sim_jacSum(m, d, chain, nb, bid, bweight, con->pos, jacdif, dim > 3);
  }
}


// frictionless and frictional contacts
void sim_instantiateContact(const sim_model_t* m, sim_data_t* d) {
  int ispyramid = sim_isPyramidal(m), issparse = sim_isSparse(m), ncon = d->ncon;
  int dim, NV, nv = m->nv, *chain = NULL;
  sim_contact_t* con;
  sim_scalar_t cpos[6], cmargin[6], *jac, *jacdif, *jacdifp, *jacdifr, *jac1p, *jac2p, *jac1r, *jac2r;

  if (SIM_DISABLED(SIM_DSBL_CONTACT) || ncon == 0 || nv == 0) {
    return;
  }

  sim_markStack(d);

  // allocate Jacobian
  jac = SIM_STACK_ALLOC(d, 6*nv, sim_scalar_t);
  jacdif = SIM_STACK_ALLOC(d, 6*nv, sim_scalar_t);
  jacdifp = jacdif;
  jacdifr = jacdif + 3*nv;
  jac1p = SIM_STACK_ALLOC(d, 3*nv, sim_scalar_t);
  jac2p = SIM_STACK_ALLOC(d, 3*nv, sim_scalar_t);
  jac1r = SIM_STACK_ALLOC(d, 3*nv, sim_scalar_t);
  jac2r = SIM_STACK_ALLOC(d, 3*nv, sim_scalar_t);
  if (issparse) {
    chain = SIM_STACK_ALLOC(d, nv, int);
  }

  // find contacts to be included
  for (int i=0; i < ncon; i++) {
    if (d->contact[i].exclude) {
      continue;
    }

    // get contact info, save efc_address
    con = d->contact + i;
    dim = con->dim;
    con->efc_address = d->nefc;
    NV = sim_contactJacobian(m, d, con, dim, jac, jacdif, jacdifp, jacdifr,
                            jac1p, jac2p, jac1r, jac2r, chain);

    // skip contact if no DOFs affected
    if (NV == 0) {
      con->efc_address = -1;
      con->exclude = 3;
      continue;
    }

    // rotate Jacobian differences to contact frame
    sim_math_mulMatMat(jac, con->frame, jacdifp, dim > 1 ? 3 : 1, 3, NV);
    if (dim > 3) {
      sim_math_mulMatMat(jac + 3*NV, con->frame, jacdifr, dim-3, 3, NV);
    }

    // make frictionless contact
    if (dim == 1) {
      // add constraint
      sim_addConstraint(m, d, jac, &(con->dist), &(con->includemargin), 0,
                       1, SIM_CNSTR_CONTACT_FRICTIONLESS, i,
                       issparse ? NV : 0,
                       issparse ? chain : NULL);
    }

    // make pyramidal friction cone
    else if (ispyramid) {
      // pos = dist
      cpos[0] = cpos[1] = con->dist;
      cmargin[0] = cmargin[1] = con->includemargin;

      // one pair per friction dimension
      for (int k=1; k < con->dim; k++) {
        // Jacobian for pair of opposing pyramid edges
        sim_math_addScl(jacdifp, jac, jac + k*NV, con->friction[k-1], NV);
        sim_math_addScl(jacdifp + NV, jac, jac + k*NV, -con->friction[k-1], NV);

        // add constraint
        sim_addConstraint(m, d, jacdifp, cpos, cmargin, 0,
                         2, SIM_CNSTR_CONTACT_PYRAMIDAL, i,
                         issparse ? NV : 0,
                         issparse ? chain : NULL);
      }
    }

    // make elliptic friction cone
    else {
      // normal pos = dist, all others 0
      sim_math_zero(cpos, con->dim);
      sim_math_zero(cmargin, con->dim);
      cpos[0] = con->dist;
      cmargin[0] = con->includemargin;

      // add constraint
      sim_addConstraint(m, d, jac, cpos, cmargin, 0,
                       con->dim, SIM_CNSTR_CONTACT_ELLIPTIC, i,
                       issparse ? NV : 0,
                       issparse ? chain : NULL);
    }
  }

  sim_freeStack(d);
}


//------------------------ compute constraint parameters -------------------------------------------

// compute diagApprox
void sim_diagApprox(const sim_model_t* m, sim_data_t* d) {
  int id, dim, b1, b2, f, weldcnt = 0;
  int nefc = d->nefc;
  sim_scalar_t tran, rot, fri, *dA = d->efc_diagApprox;
  sim_contact_t* con = NULL;

  // loop over all constraints, compute approximate inverse inertia
  for (int i=0; i < nefc; i++) {
    // get constraint id
    id = d->efc_id[i];

    // process according to constraint type
    switch ((SIM_tConstraint) d->efc_type[i]) {
    case SIM_CNSTR_EQUALITY:
      // process according to equality-constraint type
      switch (m->eq_type[id]) {
      case SIM_EQ_CONNECT:
        b1 = m->eq_obj1id[id];
        b2 = m->eq_obj2id[id];

        // get body ids if using site semantics
        if (m->eq_objtype[id] == SIM_OBJ_SITE) {
          b1 = m->site_bodyid[b1];
          b2 = m->site_bodyid[b2];
        }

        // body translation
        dA[i] = m->body_invweight0[2*b1] + m->body_invweight0[2*b2];
        break;

      case SIM_EQ_WELD:  // distinguish translation and rotation inertia
        b1 = m->eq_obj1id[id];
        b2 = m->eq_obj2id[id];

        // get body ids if using site semantics
        if (m->eq_objtype[id] == SIM_OBJ_SITE) {
          b1 = m->site_bodyid[b1];
          b2 = m->site_bodyid[b2];
        }

        // body translation or rotation depending on weldcnt
        dA[i] = m->body_invweight0[2*b1 + (weldcnt > 2)] +
                m->body_invweight0[2*b2 + (weldcnt > 2)];
        weldcnt = (weldcnt + 1) % 6;
        break;

      case SIM_EQ_JOINT:
      case SIM_EQ_TENDON:
        // object 1 contribution
        dA[i] = (m->eq_type[id] == SIM_EQ_JOINT ?
                 m->dof_invweight0[m->jnt_dofadr[m->eq_obj1id[id]]] :
                 m->tendon_invweight0[m->eq_obj1id[id]]);

        // add object 2 contribution if present
        if (m->eq_obj2id[id] >= 0)
          dA[i] += (m->eq_type[id] == SIM_EQ_JOINT ?
                    m->dof_invweight0[m->jnt_dofadr[m->eq_obj2id[id]]] :
                    m->tendon_invweight0[m->eq_obj2id[id]]);
        break;

      case SIM_EQ_FLEX:
        // process all non-rigid edges for this flex
        f = m->eq_obj1id[id];
        int flex_edgeadr = m->flex_edgeadr[f];
        int flex_edgenum = m->flex_edgenum[f];
        for (int e=flex_edgeadr; e<flex_edgeadr+flex_edgenum; e++) {
          if (!m->flexedge_rigid[e]) {
            dA[i++] = m->flexedge_invweight0[e];
          }
        }

        // adjust constraint counter
        i--;
        break;

      case SIM_EQ_FLEXVERT:
        // process all vertices for this flex
        f = m->eq_obj1id[id];
        int vertadr = m->flex_vertadr[f];
        int vertnum = m->flex_vertnum[f];
        for (int v=vertadr; v<vertadr+vertnum; v++) {
          int bodyid = m->flex_vertbodyid[v];
          dA[i++] = m->body_invweight0[2*bodyid];
          dA[i++] = m->body_invweight0[2*bodyid];
        }

        // adjust constraint counter
        i--;
        break;

      default:
        SIM_ERROR("unknown constraint type %d", d->efc_type[i]);    // SHOULD NOT OCCUR
      }
      break;

    case SIM_CNSTR_FRICTION_DOF:
      dA[i] = m->dof_invweight0[id];
      break;

    case SIM_CNSTR_LIMIT_JOINT:
      dA[i] = m->dof_invweight0[m->jnt_dofadr[id]];
      break;

    case SIM_CNSTR_FRICTION_TENDON:
    case SIM_CNSTR_LIMIT_TENDON:
      dA[i] = m->tendon_invweight0[id];
      break;

    case SIM_CNSTR_CONTACT_FRICTIONLESS:
    case SIM_CNSTR_CONTACT_PYRAMIDAL:
    case SIM_CNSTR_CONTACT_ELLIPTIC:
      // get contact info
      con = d->contact + id;
      dim = con->dim;

      // add the average translation and rotation components from both sides
      tran = rot = 0;
      for (int side=0; side < 2; side++) {
        // get bodies and weights
        int nb = 0, bid[729];
        sim_scalar_t bweight[729];

        // geom
        if (con->geom[side] >= 0) {
          bid[0] = m->geom_bodyid[con->geom[side]];
          bweight[0] = 1;
          nb = 1;
        }

        // flex
        else {
          int nw = 0;
          int vid[4];
          sim_scalar_t vweight[4];

          // vert
          if (con->vert[side] >= 0) {
            vid[0] = m->flex_vertadr[con->flex[side]] + con->vert[side];
            vweight[0] = 1;
            nw = 1;
          }

          // elem
          else {
            nw = sim_elemBodyWeight(m, d, con->flex[side], con->elem[side],
                                  con->vert[1-side], con->pos, vid, vweight);
          }

          // convert verted ids and weights to body ids and weights
          if (m->flex_interp[con->flex[side]] == 0) {
            for (int k=0; k < nw; k++) {
              bid[k] = m->flex_vertbodyid[vid[k]];
              bweight[k] = vweight[k];
              nb++;
            }
          } else {
            nb += sim_vertBodyWeight(m, d, con->flex[side], vid, bid, bweight, vweight, nw);
          }
        }

        // add weighted average over bodies
        for (int k=0; k < nb; k++) {
          tran += m->body_invweight0[2*bid[k]] * bweight[k];
          rot += m->body_invweight0[2*bid[k]+1] * bweight[k];
        }
      }

      // set frictionless
      if (d->efc_type[i] == SIM_CNSTR_CONTACT_FRICTIONLESS) {
        dA[i] = tran;
      }

      // set elliptical
      else if (d->efc_type[i] == SIM_CNSTR_CONTACT_ELLIPTIC) {
        for (int j=0; j < dim; j++) {
          dA[i+j] = (j < 3 ? tran : rot);
        }

        // processed dim elements in one i-loop iteration; advance counter
        i += (dim-1);
      }

      // set pyramidal
      else {
        for (int j=0; j < dim-1; j++) {
          fri = con->friction[j];
          dA[i+2*j] = dA[i+2*j+1] = tran + fri*fri*(j < 2 ? tran : rot);
        }

        // processed 2*dim-2 elements in one i-loop iteration; advance counter
        i += (2*dim-3);
      }
    }
  }
}


// get solref, solimp for specified constraint
static void getsolparam(const sim_model_t* m, const sim_data_t* d, int i,
                        sim_scalar_t* solref, sim_scalar_t* solreffriction, sim_scalar_t* solimp) {
  // get constraint id
  int id = d->efc_id[i];

  // clear solreffriction (applies only to contacts)
  sim_math_zero(solreffriction, SIM_NREF);

  // extract solver parameters from corresponding model element
  switch ((SIM_tConstraint) d->efc_type[i]) {
  case SIM_CNSTR_EQUALITY:
    sim_math_copy(solref, m->eq_solref+SIM_NREF*id, SIM_NREF);
    sim_math_copy(solimp, m->eq_solimp+SIM_NIMP*id, SIM_NIMP);
    break;

  case SIM_CNSTR_LIMIT_JOINT:
    sim_math_copy(solref, m->jnt_solref+SIM_NREF*id, SIM_NREF);
    sim_math_copy(solimp, m->jnt_solimp+SIM_NIMP*id, SIM_NIMP);
    break;

  case SIM_CNSTR_FRICTION_DOF:
    sim_math_copy(solref, m->dof_solref+SIM_NREF*id, SIM_NREF);
    sim_math_copy(solimp, m->dof_solimp+SIM_NIMP*id, SIM_NIMP);
    break;

  case SIM_CNSTR_LIMIT_TENDON:
    sim_math_copy(solref, m->tendon_solref_lim+SIM_NREF*id, SIM_NREF);
    sim_math_copy(solimp, m->tendon_solimp_lim+SIM_NIMP*id, SIM_NIMP);
    break;

  case SIM_CNSTR_FRICTION_TENDON:
    sim_math_copy(solref, m->tendon_solref_fri+SIM_NREF*id, SIM_NREF);
    sim_math_copy(solimp, m->tendon_solimp_fri+SIM_NIMP*id, SIM_NIMP);
    break;

  case SIM_CNSTR_CONTACT_FRICTIONLESS:
  case SIM_CNSTR_CONTACT_PYRAMIDAL:
  case SIM_CNSTR_CONTACT_ELLIPTIC:
    sim_math_copy(solref, d->contact[id].solref, SIM_NREF);
    sim_math_copy(solreffriction, d->contact[id].solreffriction, SIM_NREF);
    sim_math_copy(solimp, d->contact[id].solimp, SIM_NIMP);
  }

  // check reference format: standard or direct, cannot be mixed
  if ((solref[0] > 0) ^ (solref[1] > 0)) {
    sim_warning("mixed solref format, replacing with default");
    sim_defaultSolRefImp(solref, NULL);
  }

  // integrator safety: impose ref[0]>=2*timestep for standard format
  if (!SIM_DISABLED(SIM_DSBL_REFSAFE) && solref[0] > 0) {
    solref[0] = sim_math_max(solref[0], 2*m->opt.timestep);
  }

  // check reference format: standard or direct, cannot be mixed
  if ((solreffriction[0] > 0) ^ (solreffriction[1] > 0)) {
    sim_warning("solreffriction values should have the same sign, replacing with default");
    sim_math_zero(solreffriction, SIM_NREF);  // default solreffriction is (0, 0)
  }

  // integrator safety: impose ref[0]>=2*timestep for standard format
  if (!SIM_DISABLED(SIM_DSBL_REFSAFE) && solreffriction[0] > 0) {
    solreffriction[0] = sim_math_max(solreffriction[0], 2*m->opt.timestep);
  }

  // enforce constraints on solimp
  solimp[0] = sim_math_min(SIM_MAXIMP, sim_math_max(SIM_MINIMP, solimp[0]));
  solimp[1] = sim_math_min(SIM_MAXIMP, sim_math_max(SIM_MINIMP, solimp[1]));
  solimp[2] = sim_math_max(0, solimp[2]);
  solimp[3] = sim_math_min(SIM_MAXIMP, sim_math_max(SIM_MINIMP, solimp[3]));
  solimp[4] = sim_math_max(1, solimp[4]);
}


// get pos and dim for specified constraint
static void getposdim(const sim_model_t* m, const sim_data_t* d, int i, sim_scalar_t* pos, int* dim) {
  // get id of constraint-related object
  int id = d->efc_id[i];

  // set (dim, pos) for common case
  *dim = 1;
  *pos = d->efc_pos[i];

  // change (dim, distance) for special cases
  switch ((SIM_tConstraint) d->efc_type[i]) {
  case SIM_CNSTR_CONTACT_ELLIPTIC:
    *dim = d->contact[id].dim;
    break;

  case SIM_CNSTR_CONTACT_PYRAMIDAL:
    *dim = 2*(d->contact[id].dim-1);
    break;

  case SIM_CNSTR_EQUALITY:
    if (m->eq_type[id] == SIM_EQ_WELD) {
      *dim = 6;
      *pos = sim_math_norm(d->efc_pos+i, 6);
    } else if (m->eq_type[id] == SIM_EQ_CONNECT) {
      *dim = 3;
      *pos = sim_math_norm(d->efc_pos+i, 3);
    }
    break;
  default:
    // already handled
    break;
  }
}


// return a to the power of b, quick return for powers 1 and 2
// solimp[4] == 2 is the default, so these branches are common
static sim_scalar_t power(sim_scalar_t a, sim_scalar_t b) {
  if (b == 1) {
    return a;
  } else if (b == 2) {
    return a*a;
  }
  return sim_math_pow(a, b);
}


// compute impedance and derivative for one constraint
static void getimpedance(const sim_scalar_t* solimp, sim_scalar_t pos, sim_scalar_t margin,
                         sim_scalar_t* imp, sim_scalar_t* impP) {
  // flat function
  if (solimp[0] == solimp[1] || solimp[2] <= SIM_MINVAL) {
    *imp = 0.5*(solimp[0] + solimp[1]);
    *impP = 0;
    return;
  }

  // x = abs((pos-margin) / width)
  sim_scalar_t x = (pos-margin) / solimp[2];
  sim_scalar_t sgn = 1;
  if (x < 0) {
    x = -x;
    sgn = -1;
  }

  // fully saturated
  if (x >= 1 || x <= 0) {
    *imp = (x >= 1 ? solimp[1] : solimp[0]);
    *impP = 0;
    return;
  }

  // linear
  sim_scalar_t y, yP;
  if (solimp[4] == 1) {
    y = x;
    yP = 1;
  }

  // y(x) = a*x^p if x<=midpoint
  else if (x <= solimp[3]) {
    sim_scalar_t a = 1/power(solimp[3], solimp[4]-1);
    y = a*power(x, solimp[4]);
    yP = solimp[4] * a*power(x, solimp[4]-1);
  }

  // y(x) = 1-b*(1-x)^p if x>midpoint
  else {
    sim_scalar_t b = 1/power(1-solimp[3], solimp[4]-1);
    y = 1-b*power(1-x, solimp[4]);
    yP = solimp[4] * b*power(1-x, solimp[4]-1);
  }

  // scale
  *imp = solimp[0] + y*(solimp[1]-solimp[0]);
  *impP = yP * sgn * (solimp[1]-solimp[0]) / solimp[2];
}


// compute efc_R, efc_D, efc_KBIP, adjust efc_diagApprox
void sim_makeImpedance(const sim_model_t* m, sim_data_t* d) {
  int dim, nefc = d->nefc;
  sim_scalar_t *R = d->efc_R, *KBIP = d->efc_KBIP;
  sim_scalar_t pos, imp, impP, Rpy, solref[SIM_NREF], solreffriction[SIM_NREF], solimp[SIM_NIMP];

  // set efc_R, efc_KBIP
  for (int i=0; i < nefc; i++) {
    // get solref and solimp
    getsolparam(m, d, i, solref, solreffriction, solimp);

    // get pos and dim
    getposdim(m, d, i, &pos, &dim);

    // get imp and impP
    getimpedance(solimp, pos, d->efc_margin[i], &imp, &impP);

    // set R and KBIP for all constraint dimensions
    for (int j=0; j < dim; j++) {
      // R = (1-imp)/imp * diagApprox
      R[i+j] = sim_math_max(SIM_MINVAL, (1-imp)*d->efc_diagApprox[i+j]/imp);

      // constraint type
      int tp = d->efc_type[i+j];

      // elliptic contacts use solreffriction in non-normal directions, if non-zero
      int elliptic_friction = (tp == SIM_CNSTR_CONTACT_ELLIPTIC) && (j > 0);
      sim_scalar_t* ref = elliptic_friction && (solreffriction[0] || solreffriction[1]) ?
                    solreffriction : solref;

      // friction: K = 0
      if (tp == SIM_CNSTR_FRICTION_DOF || tp == SIM_CNSTR_FRICTION_TENDON || elliptic_friction) {
        KBIP[4*(i+j)] = 0;
      }

      // standard: K = 1 / (dmax^2 * timeconst^2 * dampratio^2)
      else if (ref[0] > 0)
        KBIP[4*(i+j)] = 1 / sim_math_max(SIM_MINVAL, solimp[1]*solimp[1] * ref[0]*ref[0] * ref[1]*ref[1]);

      // direct: K = -solref[0] / dmax^2
      else {
        KBIP[4*(i+j)] = -ref[0] / sim_math_max(SIM_MINVAL, solimp[1]*solimp[1]);
      }

      // standard: B = 2 / (dmax*timeconst)
      if (ref[1] > 0) {
        KBIP[4*(i+j)+1] = 2 / sim_math_max(SIM_MINVAL, solimp[1]*ref[0]);
      }

      // direct: B = -solref[1] / dmax
      else {
        KBIP[4*(i+j)+1] = -ref[1] / sim_math_max(SIM_MINVAL, solimp[1]);
      }

      // I = imp, P = imp'
      KBIP[4*(i+j)+2] = imp;
      KBIP[4*(i+j)+3] = impP;
    }

    // skip the rest of this constraint
    i += (dim-1);
  }

  // frictional contacts: adjust R in friction dimensions, set contact master mu
  for (int i=d->ne+d->nf; i < nefc; i++) {
    if (d->efc_type[i] == SIM_CNSTR_CONTACT_PYRAMIDAL ||
        d->efc_type[i] == SIM_CNSTR_CONTACT_ELLIPTIC) {
      // extract id, dim, mu
      int id = d->efc_id[i];
      dim = d->contact[id].dim;
      sim_scalar_t* friction = d->contact[id].friction;

      // set R[1] = R[0]/impratio
      R[i+1] = R[i]/sim_math_max(SIM_MINVAL, m->opt.impratio);

      // set mu of regularized cone = mu[1]*sqrt(R[1]/R[0])
      d->contact[id].mu = friction[0] * sim_math_sqrt(R[i+1]/R[i]);

      // elliptic
      if (d->efc_type[i] == SIM_CNSTR_CONTACT_ELLIPTIC) {
        // set remaining R's such that R[j]*mu[j]^2 = R[1]*mu[1]^2
        for (int j=1; j < dim-1; j++) {
          R[i+j+1] = R[i+1]*friction[0]*friction[0]/(friction[j]*friction[j]);
        }

        // skip the rest of this contact
        i += (dim-1);
      }

      // pyramidal: common R matching friction impedance of elliptic model
      else {
        // D0_el = 2*(dim-1)*D_py : normal match
        // D0_el = 2*mu^2*D_py    : friction match
        Rpy = 2*d->contact[id].mu*d->contact[id].mu*R[i];

        // assign Rpy to all pyramidal R
        for (int j=0; j < 2*(dim-1); j++) {
          R[i+j] = Rpy;
        }

        // skip the rest of this contact
        i += 2*(dim-1) - 1;
      }
    }
  }

  // set D = 1 / R
  for (int i=0; i < nefc; i++) {
    d->efc_D[i] = 1 / R[i];
  }

  // adjust diagApprox so that R = (1-imp)/imp * diagApprox
  for (int i=0; i < nefc; i++) {
    d->efc_diagApprox[i] = R[i] * KBIP[4*i+2] / (1-KBIP[4*i+2]);
  }
}


//------------------------------------- constraint counting ----------------------------------------

// count the non-zero columns in the Jacobian difference of two bodies
static int sim_jacDifPairCount(const sim_model_t* m, int* chain,
                              int b1, int b2, int issparse) {
  if (!m->nv) {
    return 0;
  }

  if (issparse) {
    if (m->body_simple[b1] && m->body_simple[b2]) {
      return sim_mergeChainSimple(m, chain, b1, b2);
    }
    return sim_mergeChain(m, chain, b1, b2);
  }

  return m->nv;
}


// count the non-zero columns of the Jacobian returned by sim_jacSum
static int sim_jacSumCount(const sim_model_t* m, sim_data_t* d, int* chain,
                          int n, const int* body) {
  int nv = m->nv, NV;

  sim_markStack(d);
  int* bodychain = SIM_STACK_ALLOC(d, nv, int);
  int* tempchain = SIM_STACK_ALLOC(d, nv, int);

  // set first
  NV = sim_bodyChain(m, body[0], chain);

  // accumulate remaining
  for (int i=1; i < n; i++) {
    // get body chain
    int bodyNV = sim_bodyChain(m, body[i], bodychain);
    if (!bodyNV) {
      continue;
    }

    // accumulate chains
    NV = sim_math_addChains(tempchain, nv, NV, bodyNV, chain, bodychain);
    if (NV) {
      sim_math_copyInt(chain, tempchain, NV);
    }
  }

  sim_freeStack(d);
  return NV;
}


// return number of constraint non-zeros, handle dense and dof-less cases
static inline int sim_addConstraintCount(const sim_model_t* m, int size, int NV) {
  // over count for dense allocation
  if (!sim_isSparse(m)) {
    return m->nv ? size : 0;
  }
  return SIM_MAX(0, NV) ? size : 0;
}


// count equality constraints, count Jacobian nonzeros if nnz is not NULL
static int sim_ne(const sim_model_t* m, sim_data_t* d, int* nnz) {
  int ne = 0, nnze = 0;
  int nv = m->nv, neq = m->neq;
  int id[2], size, NV, NV2, *chain = NULL, *chain2 = NULL;
  int issparse = (nnz != NULL);
  int flex_edgeadr, flex_edgenum, flex_vertadr, flex_vertnum;

  // disabled or no equality constraints: return
  if (SIM_DISABLED(SIM_DSBL_EQUALITY) || m->nemax == 0) {
    return 0;
  }

  // sleep filtering
  int sleep_filter = SIM_ENABLED(SIM_ENBL_SLEEP) && d->ntree_awake < m->ntree;

  sim_markStack(d);

  if (nnz) {
    chain = SIM_STACK_ALLOC(d, nv, int);
    chain2 = SIM_STACK_ALLOC(d, nv, int);
  }

  // find active equality constraints
  for (int i=0; i < neq; i++) {
    // skip inactive
    if (!d->eq_active[i]) {
      continue;
    }

    // skip sleeping
    if (sleep_filter && sim_sleepState(m, d, SIM_OBJ_EQUALITY, i) == sim_spec_ASLEEP) {
      continue;
    }

    id[0] = m->eq_obj1id[i];
    id[1] = m->eq_obj2id[i];
    size = 0;
    NV = 0;
    NV2 = 0;

    // process according to type
    switch ((SIM_tEq) m->eq_type[i]) {
    case SIM_EQ_CONNECT:
      size = 3;
      if (!nnz) {
        break;
      }

      // get body ids if using site semantics
      if (m->eq_objtype[i] == SIM_OBJ_SITE) {
        id[0] = m->site_bodyid[id[0]];
        id[1] = m->site_bodyid[id[1]];
      }

      NV = sim_jacDifPairCount(m, chain, id[1], id[0], issparse);
      break;

    case SIM_EQ_WELD:
      size = 6;
      if (!nnz) {
        break;
      }

      // get body ids if using site semantics
      if (m->eq_objtype[i] == SIM_OBJ_SITE) {
        id[0] = m->site_bodyid[id[0]];
        id[1] = m->site_bodyid[id[1]];
      }

      NV = sim_jacDifPairCount(m, chain, id[1], id[0], issparse);
      break;

    case SIM_EQ_JOINT:
    case SIM_EQ_TENDON:
      size = 1;
      if (!nnz) {
        break;
      }

      for (int j=0; j < 1+(id[1] >= 0); j++) {
        if (m->eq_type[i] == SIM_EQ_JOINT) {
          if (!j) {
            NV = 1;
            chain[0] = m->jnt_dofadr[id[j]];
          } else {
            NV2 = 1;
            chain2[0] = m->jnt_dofadr[id[j]];
          }
        } else {
          if (!j) {
            NV = d->ten_J_rownnz[id[j]];
            sim_math_copyInt(chain, d->ten_J_colind+d->ten_J_rowadr[id[j]], NV);
          } else {
            NV2 = d->ten_J_rownnz[id[j]];
            sim_math_copyInt(chain2, d->ten_J_colind+d->ten_J_rowadr[id[j]], NV2);
          }
        }
      }

      if (id[1] >= 0) {
        NV = sim_math_combineSparseCount(NV, NV2, chain, chain2);
      }
      break;

    case SIM_EQ_FLEX:
      flex_edgeadr = m->flex_edgeadr[id[0]];
      flex_edgenum = m->flex_edgenum[id[0]];

      // init with all edges, subtract rigid later
      size = flex_edgenum;

      // process edges of this flex
      for (int e=flex_edgeadr; e < flex_edgeadr+flex_edgenum; e++) {
        // rigid: reduce size and skip
        if (m->flexedge_rigid[e]) {
          size--;
          continue;
        }

        // accumulate NV if needed
        if (nnz) {
          int b1 = m->flex_vertbodyid[m->flex_vertadr[id[0]] + m->flex_edge[2*e]];
          int b2 = m->flex_vertbodyid[m->flex_vertadr[id[0]] + m->flex_edge[2*e+1]];
          NV += sim_jacDifPairCount(m, chain, b1, b2, issparse);
        }
      }
      break;

    case SIM_EQ_FLEXVERT:
      flex_vertadr = m->flex_vertadr[id[0]];
      flex_vertnum = m->flex_vertnum[id[0]];
      size = 2 * flex_vertnum;
      if (nnz) {
        for (int v=flex_vertadr; v < flex_vertadr+flex_vertnum; v++) {
          NV += m->flexvert_J_rownnz[2*v+0];
          NV += m->flexvert_J_rownnz[2*v+1];
        }
      }
      break;

    default:
      // might occur in case of the now-removed distance equality constraint
      SIM_ERROR("unknown constraint type %d", m->eq_type[i]);    // SHOULD NOT OCCUR
    }

    // accumulate counts; flex NV already accumulated
    ne += sim_addConstraintCount(m, size, NV);
    if (m->eq_type[i] == SIM_EQ_FLEX || m->eq_type[i] == SIM_EQ_FLEXVERT) {
      nnze += NV;
    } else {
      nnze += size*NV;
    }
  }

  if (nnz) {
    *nnz += nnze;
  }

  sim_freeStack(d);
  return ne;
}


// count frictional constraints, count Jacobian nonzeros if nnz is not NULL
static int sim_nf(const sim_model_t* m, const sim_data_t* d, int *nnz) {
  int nf = 0;
  int nv = m->nv, ntendon = m->ntendon;

  if (SIM_DISABLED(SIM_DSBL_FRICTIONLOSS)) {
    return 0;
  }

  // sleep filtering
  int sleep_filter = SIM_ENABLED(SIM_ENBL_SLEEP) && d->ntree_awake < m->ntree;

  for (int i=0; i < nv; i++) {
    // no friction loss: skip
    if (!m->dof_frictionloss[i]) {
      continue;
    }

    // sleeping tree: skip
    if (sleep_filter && !d->tree_awake[m->dof_treeid[i]]) {
      continue;
    }

    nf += sim_addConstraintCount(m, 1, 1);
    if (nnz) *nnz += 1;
  }

  for (int i=0; i < ntendon; i++) {
    if (m->tendon_frictionloss[i] > 0) {
      nf += sim_addConstraintCount(m, 1, d->ten_J_rownnz[i]);
      if (nnz) *nnz += d->ten_J_rownnz[i];
    }
  }

  return nf;
}


// count limit constraints, count Jacobian nonzeros if nnz is not NULL
static int sim_nl(const sim_model_t* m, const sim_data_t* d, int *nnz) {
  int nl = 0;
  int ntendon = m->ntendon;
  int side;
  sim_scalar_t margin, value, dist;

  // disabled: return
  if (SIM_DISABLED(SIM_DSBL_LIMIT)) {
    return 0;
  }

  // sleep filtering
  int sleep_filter = SIM_ENABLED(SIM_ENBL_SLEEP) && d->ntree_awake < m->ntree;

  for (int i=0; i < m->njnt; i++) {
    if (!m->jnt_limited[i]) {
      continue;
    }

    // sleeping tree: skip
    if (sleep_filter && !d->tree_awake[m->dof_treeid[m->jnt_dofadr[i]]]) {
      continue;
    }

    margin = m->jnt_margin[i];

    // SLIDE and HINGE joint limits can be bilateral, check both sides
    if (m->jnt_type[i] == SIM_JNT_SLIDE || m->jnt_type[i] == SIM_JNT_HINGE) {
      value = d->qpos[m->jnt_qposadr[i]];
      for (side=-1; side <= 1; side+=2) {
        dist = side * (m->jnt_range[2*i+(side+1)/2] - value);
        if (dist < margin) {
          nl += sim_addConstraintCount(m, 1, 1);
          if (nnz) *nnz += 1;
        }
      }
    }

    // BALL joint limits are always unilateral
    else if (m->jnt_type[i] == SIM_JNT_BALL) {
      sim_scalar_t angleAxis[3];
      int adr = m->jnt_qposadr[i];
      sim_scalar_t quat[4] = {d->qpos[adr], d->qpos[adr+1], d->qpos[adr+2], d->qpos[adr+3]};
      sim_math_normalize4(quat);
      sim_math_quat2Vel(angleAxis, quat, 1);
      value = sim_math_normalize_3(angleAxis);
      dist = sim_math_max(m->jnt_range[2*i], m->jnt_range[2*i+1]) - value;
      if (dist < margin) {
        nl += sim_addConstraintCount(m, 1, 3);
        if (nnz) *nnz += 3;
      }
    }
  }

  // tendon limits
  for (int i=0; i < ntendon; i++) {
    int count = tendonLimit(m, d->ten_length, i);
    for (int j = 0; j < count; j++) {
      nl += sim_addConstraintCount(m, 1, d->ten_J_rownnz[i]);
      if (nnz) *nnz += d->ten_J_rownnz[i];
    }
  }

  return nl;
}


// count contact constraints, count Jacobian nonzeros if nnz is not NULL
static int sim_nc(const sim_model_t* m, sim_data_t* d, int* nnz) {
  int nnzc = 0, nc = 0;
  int ispyramid = sim_isPyramidal(m), ncon = d->ncon;

  if (SIM_DISABLED(SIM_DSBL_CONTACT) || !ncon) {
    return 0;
  }

  // sleep filtering
  int sleep_filter = SIM_ENABLED(SIM_ENBL_SLEEP) && d->ntree_awake < m->ntree;

  sim_markStack(d);
  int *chain = SIM_STACK_ALLOC(d, m->nv, int);

  for (int i=0; i < ncon; i++) {
    sim_contact_t* con = d->contact + i;

    // skip if passive
    if ((con->flex[0] > -1 && m->flex_passive[con->flex[0]]) ||
        (con->flex[1] > -1 && m->flex_passive[con->flex[1]])) {
      con->efc_address = -1;
      con->exclude = 4;
    }

    // skip if excluded
    if (con->exclude) {
      continue;
    }

    // check for contact with sleeping tree; SHOULD NOT OCCUR
    if (sleep_filter) {
      int g1 = con->geom[0];
      int g2 = con->geom[1];
      if (g1 >= 0 && g2 >= 0) {
        int b1 = m->body_weldid[m->geom_bodyid[g1]];
        int b2 = m->body_weldid[m->geom_bodyid[g2]];
        int asleep1 = d->body_awake[b1] == sim_spec_ASLEEP;
        int asleep2 = d->body_awake[b2] == sim_spec_ASLEEP;
        if (asleep1 || asleep2) {
          SIM_ERROR("contact %d involves sleeping geom %d", i, asleep1 ? g1 : g2);
        }
      }
    }

    // compute NV only if nnz requested
    int NV = 0;
    if (nnz) {
      // get bodies
      int nb = 0, bid[729];
      for (int side=0; side < 2; side++) {
        // geom
        if (con->geom[side] >= 0) {
          bid[nb++] = m->geom_bodyid[con->geom[side]];
        }

        // flex
        else {
          int nw = 0;
          int vid[4];
          sim_scalar_t vweight[4];

          // flex vert
          if (con->vert[side] >= 0) {
            vid[nw++] = m->flex_vertadr[con->flex[side]] + con->vert[side];
            vweight[0] = 1;
          }

          // flex elem
          else {
            int f = con->flex[side];
            int fdim = m->flex_dim[f];
            const int* edata = m->flex_elem + m->flex_elemdataadr[f] + con->elem[side]*(fdim+1);
            for (int k=0; k <= fdim; k++) {
              vid[nw++] = m->flex_vertadr[f] + edata[k];
            }

            if (m->flex_interp[f]) {
              nw = sim_elemBodyWeight(m, d, con->flex[side], con->elem[side],
                                    con->vert[1-side], con->pos, vid, vweight);
            }
          }

          // get body or node ids and weights
          if (m->flex_interp[con->flex[side]] == 0) {
            for (int k=0; k < nw; k++) {
              bid[nb] = m->flex_vertbodyid[vid[k]];
              nb++;
            }
          } else {
            nb += sim_vertBodyWeight(m, d, con->flex[side], vid, bid+nb, NULL, vweight, nw);
          }
        }
      }

      // count non-zeros in merged chain
      NV = sim_jacSumCount(m, d, chain, nb, bid);
      if (!NV) {
        continue;
      }
    }

    // count according to friction type
    int dim = con->dim;
    if (dim == 1) {
      nc++;
      nnzc += NV;
    } else if (ispyramid) {
      nc += 2*(dim-1);
      nnzc += 2*(dim-1)*NV;
    } else {
      nc += dim;
      nnzc += dim*NV;
    }
  }

  if (nnz) {
    *nnz += nnzc;
  }

  sim_freeStack(d);
  return nc;
}


//---------------------------- top-level API for constraint construction ---------------------------

// driver: call all functions above
void sim_makeConstraint(const sim_model_t* m, sim_data_t* d) {
  // clear sizes
  d->ne = d->nf = d->nl = d->nefc = d->nJ = d->nA = 0;

  // disabled or Jacobian not allocated: return
  if (SIM_DISABLED(SIM_DSBL_CONSTRAINT)) {
    return;
  }

  // precount sizes for constraint Jacobian matrices
  int *nnz = sim_isSparse(m) ? &(d->nJ) : NULL;
  int ne_allocated = sim_ne(m, d, nnz);
  int nf_allocated = sim_nf(m, d, nnz);
  int nl_allocated = sim_nl(m, d, nnz);
  int nefc_allocated = ne_allocated + nf_allocated + nl_allocated + sim_nc(m, d, nnz);
  if (!sim_isSparse(m)) {
    d->nJ = nefc_allocated * m->nv;
  }
  d->nefc = nefc_allocated;

  // allocate efc arrays on arena
  if (!arenaAllocEfc(m, d)) {
    return;
  }

  // clear tendon_efcadr
  sim_math_fillInt(d->tendon_efcadr, -1, m->ntendon);

  // reset nefc for the instantiation functions, instantiate all elements of Jacobian
  d->nefc = 0;
  sim_instantiateEquality(m, d);
  sim_instantiateFriction(m, d);
  sim_instantiateLimit(m, d);
  sim_instantiateContact(m, d);

  // check sparse allocation
  if (sim_isSparse(m)) {
    if (d->ne != ne_allocated) {
      SIM_ERROR("ne mis-allocation: found ne=%d but allocated %d", d->ne, ne_allocated);
    }

    if (d->nf != nf_allocated) {
      SIM_ERROR("nf mis-allocation: found nf=%d but allocated %d", d->nf, nf_allocated);
    }

    if (d->nl != nl_allocated) {
      SIM_ERROR("nl mis-allocation: found nl=%d but allocated %d", d->nl, nl_allocated);
    }

    // check that nefc was computed correctly
    if (d->nefc != nefc_allocated) {
      SIM_ERROR("nefc mis-allocation: found nefc=%d but allocated %d", d->nefc, nefc_allocated);
    }

    // check that nJ was computed correctly
    if (d->nefc > 0) {
      int nJ = d->efc_J_rownnz[d->nefc - 1] + d->efc_J_rowadr[d->nefc - 1];
      if (d->nJ != nJ) {
        SIM_ERROR("constraint Jacobian mis-allocation: found nJ=%d but allocated %d", nJ, d->nJ);
      }
    }
  } else if (d->nefc > nefc_allocated) {
    SIM_ERROR("nefc under-allocation: found nefc=%d but allocated only %d",
            d->nefc, nefc_allocated);
  }

  // collect memory use statistics
  d->maxuse_con = SIM_MAX(d->maxuse_con, d->ncon);
  d->maxuse_efc = SIM_MAX(d->maxuse_efc, d->nefc);

  // no constraints: return
  if (!d->nefc) {
    return;
  }

  // transpose sparse Jacobian, make row supernodes
  if (sim_isSparse(m)) {
#ifdef SIM_USEAVX
    // compute supernodes of J; used by sim_math_mulMatVecSparse_avx
    sim_math_superSparse(d->nefc, d->efc_J_rowsuper,
                    d->efc_J_rownnz, d->efc_J_rowadr, d->efc_J_colind);
#else
  #ifdef MEMORY_SANITIZER
    // tell msan to treat the entire J rowsuper as uninitialized
    __msan_allocated_memory(d->efc_J_rowsuper, d->nefc);
  #endif  // MEMORY_SANITIZER
#endif  // SIM_USEAVX
  }

  // compute diagApprox
  sim_diagApprox(m, d);

  // compute KBIP, D, R, adjust diagApprox
  sim_makeImpedance(m, d);
}


// compute efc_AR
void sim_projectConstraint(const sim_model_t* m, sim_data_t* d) {
  int nefc = d->nefc, nv = m->nv;

  // nothing to do
  if (nefc == 0 || !sim_isDual(m)) {
    return;
  }

  sim_markStack(d);

  // inverse square root of D from inertia LDL decomposition
  sim_scalar_t* sqrtInvD = SIM_STACK_ALLOC(d, nv, sim_scalar_t);
  for (int i=0; i < nv; i++) {
    int diag = m->M_rowadr[i] + m->M_rownnz[i] - 1;
    sqrtInvD[i] = 1 / sim_math_sqrt(d->qLD[diag]);
  }

  // sparse
  if (sim_isSparse(m)) {
    // compute B = backsubM2(J')' and its transpose


    // === pre-count B_rownnz, B_rowadr, nB (total nonzeros)

    // allocate B rownnz and rowadr
    int* B_rownnz = SIM_STACK_ALLOC(d, nefc, int);
    int* B_rowadr = SIM_STACK_ALLOC(d, nefc, int);

    // markers for merged dofs, initialized to -1
    int* marker = SIM_STACK_ALLOC(d, nv, int);
    sim_math_fillInt(marker, -1, nv);

    B_rowadr[0] = 0;
    for (int r=0; r < nefc; r++) {
      int nnz = 0;  // nonzeros in row r of B

      // traverse row r of J in reverse, count unique nonzeros
      int start = d->efc_J_rowadr[r];
      int end = start + d->efc_J_rownnz[r];
      for (int i=end-1; i >= start; i--) {
        int j = d->efc_J_colind[i];

        // if dof j is marked, it was already counted by a child dof: skip it
        if (marker[j] == r) {
          continue;
        }

        // traverse row j of C, marking new unique nonzeros
        int nnzC = m->M_rownnz[j];
        int adrC = m->M_rowadr[j];
        for (int k=0; k < nnzC; k++) {
          int c = m->M_colind[adrC + k];
          if (marker[c] != r) {
            marker[c] = r;
            nnz++;
          }
        }
      }

      // update rownnz and rowadr
      B_rownnz[r] = nnz;
      if (r < nefc - 1) {
        B_rowadr[r+1] = B_rowadr[r] + nnz;
      }
    }

    // total non-zeros in B
    int nB = B_rowadr[nefc-1] + B_rownnz[nefc-1];


    // === fill in B column indices, copy values from J

    // allocate values and column indices
    sim_scalar_t* B = SIM_STACK_ALLOC(d, nB, sim_scalar_t);
    int* B_colind = SIM_STACK_ALLOC(d, nB, int);

    for (int r=0; r < nefc; r++) {
      // init row
      int end = B_rowadr[r] + B_rownnz[r];
      int adrJ = d->efc_J_rowadr[r];
      int remainJ = d->efc_J_rownnz[r];
      int nnzB = 0;

      // complete chain in reverse
      while (1) {
        // get previous dof in src and dst
        int prev_src = (remainJ > 0 ? d->efc_J_colind[adrJ + remainJ - 1] : -1);
        int prev_dst = (nnzB > 0 ? m->dof_parentid[B_colind[end - nnzB]] : -1);

        // both finished: break
        if (prev_src < 0 && prev_dst < 0) {
          break;
        }

        // add src
        else if (prev_src >= prev_dst) {
          nnzB++;
          remainJ--;
          B_colind[end - nnzB] = prev_src;
          B[end - nnzB] = d->efc_J[adrJ + remainJ];
        }

        // add dst
        else {
          nnzB++;
          B_colind[end - nnzB] = prev_dst;
          B[end - nnzB] = 0;
        }
      }

      // compare with B_rownnz: SHOULD NOT OCCUR
      if (nnzB != B_rownnz[r]) {
        SIM_ERROR("pre and post-count of B_rownnz are not equal on row %d", r);
      }
    }


    // === in-place sparse back-substitution:  B <- B * M^-1/2

    // sparse backsubM2 (half of LD back-substitution)
    for (int r=0; r < nefc; r++) {
      int nnzB = B_rownnz[r];
      int adrB = B_rowadr[r];

      // B(r,:) <- inv(L') * B(r,:), exploit sparsity of input vector
      for (int i=adrB + nnzB-1; i >= adrB; i--) {
        sim_scalar_t b = B[i];
        if (b == 0) {
          continue;
        }
        int j = B_colind[i];
        int adrC = m->M_rowadr[j];
        sim_math_addToSclSparseInc(B + adrB, d->qLD + adrC,
                              nnzB, B_colind + adrB,
                              m->M_rownnz[j]-1, m->M_colind + adrC, -b);
      }

      // B(r,:) <- sqrt(inv(D)) * B(r,:)
      for (int i=adrB; i < adrB + nnzB; i++) {
        int j = B_colind[i];
        B[i] *= sqrtInvD[j];
      }
    }

    // construct B supernodes
    int* B_rowsuper = SIM_STACK_ALLOC(d, nefc, int);
    sim_math_superSparse(nefc, B_rowsuper, B_rownnz, B_rowadr, B_colind);

    // construct B transposed
    int* BT_rownnz = SIM_STACK_ALLOC(d, nv, int);
    int* BT_rowadr = SIM_STACK_ALLOC(d, nv, int);
    int* BT_colind = SIM_STACK_ALLOC(d, nB, int);
    sim_scalar_t* BT = SIM_STACK_ALLOC(d, nB, sim_scalar_t);
    sim_math_transposeSparse(BT, B, nefc, nv,
                        BT_rownnz, BT_rowadr, BT_colind, NULL,
                        B_rownnz, B_rowadr, B_colind);

    // allocate AR row nonzeros and addresses on arena
    d->efc_AR_rownnz = sim_arenaAllocByte(d, sizeof(int) * nefc, _Alignof(int));
    d->efc_AR_rowadr = sim_arenaAllocByte(d, sizeof(int) * nefc, _Alignof(int));
    if (!d->efc_AR_rownnz || !d->efc_AR_rowadr) {
      sim_runtime_warning(d, SIM_WARN_CNSTRFULL, d->narena);
      sim_clearEfc(d);
      d->parena = d->ncon * sizeof(sim_contact_t);
      sim_freeStack(d);
      return;
    }

    // pre-count A nonzeros (compute AR_rownnz, AR_rowadr)
    d->nA = sim_math_sqrMatTDSparseCount(d->efc_AR_rownnz, d->efc_AR_rowadr, nefc,
                                    BT_rownnz, BT_rowadr, BT_colind,
                                    B_rownnz, B_rowadr, B_colind, B_rowsuper, d, /*flg_upper=*/1);

    // allocate A values and column indices on arena
    d->efc_AR = sim_arenaAllocByte(d, sizeof(sim_scalar_t) * d->nA, _Alignof(sim_scalar_t));
    d->efc_AR_colind = sim_arenaAllocByte(d, sizeof(int) * d->nA, _Alignof(int));
    if (!d->efc_AR || !d->efc_AR_colind) {
      sim_runtime_warning(d, SIM_WARN_CNSTRFULL, d->narena);
      sim_clearEfc(d);
      d->parena = d->ncon * sizeof(sim_contact_t);
      sim_freeStack(d);
      return;
    }

    // A = B * B'
    int* diagind = SIM_STACK_ALLOC(d, nefc, int);
    sim_math_sqrMatTDSparse(d->efc_AR, BT, B, NULL, nv, nefc,
                       d->efc_AR_rownnz, d->efc_AR_rowadr, d->efc_AR_colind,
                       BT_rownnz, BT_rowadr, BT_colind, NULL,
                       B_rownnz, B_rowadr, B_colind, B_rowsuper, d, diagind);

    // AR = A + diag(R)
    for (int i=0; i < nefc; i++) {
      d->efc_AR[diagind[i]] += d->efc_R[i];
    }
  }

  // dense
  else {
    d->nA = nefc * nefc;

    // arena-allocate efc_AR
    d->efc_AR = sim_arenaAllocByte(d, sizeof(sim_scalar_t) * d->nA, _Alignof(sim_scalar_t));
    if (!d->efc_AR) {
      sim_runtime_warning(d, SIM_WARN_CNSTRFULL, d->narena);
      sim_clearEfc(d);
      d->parena = d->ncon * sizeof(sim_contact_t);
      sim_freeStack(d);
      return;
    }

    // space for B = backsubM2(J')' and its transpose
    sim_scalar_t* B = SIM_STACK_ALLOC(d, nefc*nv, sim_scalar_t);
    sim_scalar_t* BT = SIM_STACK_ALLOC(d, nv*nefc, sim_scalar_t);

    // B = backsubM2(J')'
    sim_solveM2(m, d, B, d->efc_J, sqrtInvD, nefc);

    // construct BT
    sim_math_transpose(BT, B, nefc, nv);

    // AR = B * B'
    sim_math_sqrMatTD(d->efc_AR, BT, NULL, nv, nefc);

    // add R to diagonal of AR
    for (int r=0; r < nefc; r++) {
      d->efc_AR[r*(nefc+1)] += d->efc_R[r];
    }
  }

  sim_freeStack(d);
}


// compute efc_vel, efc_aref
void sim_referenceConstraint(const sim_model_t* m, sim_data_t* d) {
  int nefc = d->nefc;
  sim_scalar_t* KBIP = d->efc_KBIP;

  // compute efc_vel
  sim_mulJacVec(m, d, d->efc_vel, d->qvel);

  // compute aref = -B*vel - K*I*(pos-margin)
  for (int i=0; i < nefc; i++) {
    d->efc_aref[i] = -KBIP[4*i+1]*d->efc_vel[i]
                     -KBIP[4*i]*KBIP[4*i+2]*(d->efc_pos[i]-d->efc_margin[i]);
  }
}


//---------------------------- update constraint state ---------------------------------------------

// compute efc_state, efc_force
//  optional: cost(qacc) = s_hat(jar); cone Hessians
void sim_constraintUpdate_impl(int ne, int nf, int nefc,
                              const sim_scalar_t* D, const sim_scalar_t* R, const sim_scalar_t* floss,
                              const sim_scalar_t* jar, const int* type, const int* id,
                              sim_contact_t* contact, int* state, sim_scalar_t* force, sim_scalar_t cost[1],
                              int flg_coneHessian) {
  sim_scalar_t s = 0;

  // no constraints: clear cost, return
  if (!nefc) {
    if (cost) {
      *cost = 0;
    }
    return;
  }

  // compute unconstrained efc_force
  for (int i=0; i < nefc; i++) {
    force[i] = -D[i]*jar[i];
  }

  // update constraints
  for (int i=0; i < nefc; i++) {
    // ==== equality
    if (i < ne) {
      if (cost) {
        s += 0.5*D[i]*jar[i]*jar[i];
      }
      state[i] = SIM_CNSTRSTATE_QUADRATIC;
      continue;
    }

    // ==== friction
    if (i < ne + nf) {
      // linear negative
      if (jar[i] <= -R[i]*floss[i]) {
        if (cost) {
          s += -0.5*R[i]*floss[i]*floss[i] - floss[i]*jar[i];
        }

        force[i] = floss[i];
        state[i] = SIM_CNSTRSTATE_LINEARNEG;
      }

      // linear positive
      else if (jar[i] >= R[i]*floss[i]) {
        if (cost) {
          s += -0.5*R[i]*floss[i]*floss[i] + floss[i]*jar[i];
        }

        force[i] = -floss[i];
        state[i] = SIM_CNSTRSTATE_LINEARPOS;
      }

      // quadratic
      else {
        if (cost) {
          s += 0.5*D[i]*jar[i]*jar[i];
        }
        state[i] = SIM_CNSTRSTATE_QUADRATIC;
      }
      continue;
    }

    // ==== contact

    // non-negative constraint
    if (type[i] != SIM_CNSTR_CONTACT_ELLIPTIC) {
      // constraint is satisfied: no cost
      if (jar[i] >= 0) {
        force[i] = 0;

        state[i] = SIM_CNSTRSTATE_SATISFIED;
      }

      // quadratic
      else {
        if (cost) {
          s += 0.5*D[i]*jar[i]*jar[i];
        }
        state[i] = SIM_CNSTRSTATE_QUADRATIC;
      }
    }

    // contact with elliptic cone
    else {
      // get contact
      sim_contact_t* con = contact + id[i];
      sim_scalar_t mu = con->mu, *friction = con->friction;
      int dim = con->dim;

      // map to regular dual cone space
      sim_scalar_t U[6];
      U[0] = jar[i]*mu;
      for (int j=1; j < dim; j++) {
        U[j] = jar[i+j]*friction[j-1];
      }

      // decompose into normal and tangent
      sim_scalar_t N = U[0];
      sim_scalar_t T = sim_math_norm(U+1, dim-1);

      // top zone
      if (N >= mu*T || (T <= 0 && N >= 0)) {
        sim_math_zero(force+i, dim);
        state[i] = SIM_CNSTRSTATE_SATISFIED;
      }

      // bottom zone
      else if (mu*N+T <= 0 || (T <= 0 && N < 0)) {
        if (cost) {
          for (int j=0; j < dim; j++) {
            s += 0.5*D[i+j]*jar[i+j]*jar[i+j];
          }
        }
        state[i] = SIM_CNSTRSTATE_QUADRATIC;
      }

      // middle zone
      else {
        // cost: 0.5*D0/(mu*mu*(1+mu*mu))*(N-mu*T)^2
        sim_scalar_t Dm = D[i]/(mu*mu*(1+mu*mu));
        sim_scalar_t NmT = N - mu*T;

        if (cost) {
          s += 0.5*Dm*NmT*NmT;
        }

        // force: - ds/djar = dU/djar * ds/dU  (dU/djar = diag(mu, friction))
        force[i] = -Dm*NmT*mu;
        for (int j=1; j < dim; j++) {
          force[i+j] = -force[i]/T*U[j]*friction[j-1];
        }

        // set state
        state[i] = SIM_CNSTRSTATE_CONE;

        // cone Hessian
        if (flg_coneHessian) {
          // get Hessian pointer
          sim_scalar_t* H = contact[id[i]].H;

          // set first row: (1, -mu/T * U)
          sim_scalar_t scl = -mu/T;
          H[0] = 1;
          for (int j=1; j < dim; j++) {
            H[j] = scl*U[j];
          }

          // set upper block: mu*N/T^3 * U*U'
          scl = mu*N/(T*T*T);
          for (int k=1; k < dim; k++) {
            for (int j=k; j < dim; j++) {
              H[k*dim+j] = scl*U[j]*U[k];
            }
          }

          // add to diagonal: (mu^2 - mu*N/T) * I
          scl = mu*mu - mu*N/T;
          for (int j=1; j < dim; j++) {
            H[j*(dim+1)] += scl;
          }

          // pre and post multiply by diag(mu, friction), scale by Dm
          for (int k=0; k < dim; k++) {
            scl = Dm * (k == 0 ? mu : friction[k-1]);
            for (int j=k; j < dim; j++) {
              H[k*dim+j] *= scl * (j == 0 ? mu : friction[j-1]);
            }
          }

          // make symmetric: copy upper into lower
          for (int k=0; k < dim; k++) {
            for (int j=k+1; j < dim; j++) {
              H[j*dim+k] = H[k*dim+j];
            }
          }
        }
      }

      // replicate state in all cone dimensions
      for (int j=1; j < dim; j++) {
        state[i+j] = state[i];
      }

      // advance to end of contact
      i += (dim-1);
    }
  }

  // assign cost
  if (cost) {
    *cost = s;
  }
}


// compute efc_state, efc_force, qfrc_constraint
// optional: cost(qacc) = s_hat(jar) where jar = Jac*qacc-aref; cone Hessians
void sim_constraintUpdate(const sim_model_t* m, sim_data_t* d, const sim_scalar_t* jar,
                         sim_scalar_t cost[1], int flg_coneHessian) {
  sim_constraintUpdate_impl(d->ne, d->nf, d->nefc, d->efc_D, d->efc_R, d->efc_frictionloss,
                           jar, d->efc_type, d->efc_id, d->contact, d->efc_state, d->efc_force,
                           cost, flg_coneHessian);
  sim_mulJacTVec(m, d, d->qfrc_constraint, d->efc_force);
}

