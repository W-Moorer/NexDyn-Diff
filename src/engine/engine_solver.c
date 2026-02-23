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

#include "engine/engine_solver.h"

#include <stddef.h>
#include <string.h>

#include <simcore/SIM_data.h>
#include <simcore/SIM_macro.h>
#include <simcore/SIM_model.h>
#include <simcore/SIM_san.h>  // IWYU pragma: keep
#include "engine/engine_core_constraint.h"
#include "engine/engine_core_smooth.h"
#include "engine/engine_core_util.h"
#include "engine/engine_memory.h"
#include "engine/engine_util_blas.h"
#include "engine/engine_util_errmem.h"
#include "engine/engine_util_misc.h"
#include "engine/engine_util_solve.h"
#include "engine/engine_util_sparse.h"


//---------------------------------- utility functions ---------------------------------------------

// save solver statistics
static void saveStats(const sim_model_t* m, sim_data_t* d, int island, int iter,
                      sim_scalar_t improvement, sim_scalar_t gradient, sim_scalar_t lineslope,
                      int nactive, int nchange, int neval, int nupdate) {
  // if island out of range, return
  if (island >= SIM_NISLAND) {
    return;
  }

  // if no islands, use first island
  island = SIM_MAX(0, island);

  // if iter out of range, return
  if (iter >= SIM_NSOLVER) {
    return;
  }

  // get SIM_SolverStat pointer
  SIM_SolverStat* stat = d->solver + island*SIM_NSOLVER + iter;

  // save stats
  stat->improvement = improvement;
  stat->gradient = gradient;
  stat->lineslope = lineslope;
  stat->nactive = nactive;
  stat->nchange = nchange;
  stat->neval = neval;
  stat->nupdate = nupdate;
}


// finalize dual solver: map to joint space
// TODO: b/295296178 - add island support to Dual solvers
static void dualFinish(const sim_model_t* m, sim_data_t* d) {
  // map constraint force to joint space
  sim_mulJacTVec(m, d, d->qfrc_constraint, d->efc_force);

  // compute constrained acceleration in joint space
  sim_solveM(m, d, d->qacc, d->qfrc_constraint, 1);
  sim_math_addTo(d->qacc, d->qacc_smooth, m->nv);
}


// compute 1/diag(AR)
// TODO: b/295296178 - add island support to Dual solvers
static void ARdiaginv(const sim_model_t* m, const sim_data_t* d, sim_scalar_t* res, int flg_subR) {
  int nefc = d->nefc;
  const sim_scalar_t *AR = d->efc_AR;
  const sim_scalar_t *R = d->efc_R;

  // sparse
  if (sim_isSparse(m)) {
    const int *rowadr = d->efc_AR_rowadr;
    const int *rownnz = d->efc_AR_rownnz;
    const int *colind = d->efc_AR_colind;

    for (int i=0; i < nefc; i++) {
      int nnz = rownnz[i];
      for (int j=0; j < nnz; j++) {
        int adr = rowadr[i] + j;
        if (i == colind[adr]) {
          res[i] = 1 / (flg_subR ? sim_math_max(SIM_MINVAL, AR[adr] - R[i]) : AR[adr]);
          break;
        }
      }
    }
  }

  // dense
  else {
    for (int i=0; i < nefc; i++) {
      int adr = i * (nefc + 1);
      res[i] = 1 / (flg_subR ? sim_math_max(SIM_MINVAL, AR[adr] - R[i]) : AR[adr]);
    }
  }
}


// extract diagonal block from AR, clamp diag to 1e-10 if flg_subR
// TODO: b/295296178 - add island support to Dual solvers
static void extractBlock(const sim_model_t* m, const sim_data_t* d, sim_scalar_t* Ac,
                         int start, int n, int flg_subR) {
  int nefc = d->nefc;
  const sim_scalar_t *AR = d->efc_AR;

  // sparse
  if (sim_isSparse(m)) {
    const int* rownnz = d->efc_AR_rownnz;
    const int* rowadr = d->efc_AR_rowadr;
    const int* colind = d->efc_AR_colind;
    /*
            // GENERAL CASE
            sim_math_zero(Ac, n*n);
            for( j=0; j<n; j++ )
                for( k=0; k<rownnz[start+j]; k++ )
                {
                    int col = colind[rowadr[start+j]+k];
                    if( col>=start && col<start+n )
                        Ac[j*n+col-start] = AR[rowadr[start+j]+k];
                }
     */
    // assume full sub-matrix, find starting k: same for all rows
    int k;
    for (k=0; k < rownnz[start]; k++) {
      if (colind[rowadr[start]+k] == start) {
        break;
      }
    }

    // SHOULD NOT OCCUR
    if (k >= rownnz[start]) {
      SIM_ERROR("internal error");
    }

    // copy rows
    for (int j=0; j < n; j++) {
      sim_math_copy(Ac+j*n, AR+rowadr[start+j]+k, n);
    }
  }

  // dense
  else {
    for (int j=0; j < n; j++) {
      sim_math_copy(Ac+j*n, AR+start+(start+j)*nefc, n);
    }
  }

  // subtract R from diagonal, clamp to 1e-10 from below
  if (flg_subR) {
    const sim_scalar_t *R = d->efc_R;
    for (int j=0; j < n; j++) {
      Ac[j*(n+1)] -= R[start+j];
      Ac[j*(n+1)] = sim_math_max(1e-10, Ac[j*(n+1)]);
    }
  }
}


// compute residual for one block
// TODO: b/295296178 - add island support to Dual solvers
static void residual(const sim_model_t* m, const sim_data_t* d, sim_scalar_t* res, int i, int dim, int flg_subR) {
  int nefc = d->nefc;

  // sparse
  if (sim_isSparse(m)) {
    for (int j=0; j < dim; j++) {
      res[j] = d->efc_b[i+j] + sim_math_dotSparse(d->efc_AR + d->efc_AR_rowadr[i+j],
                                             d->efc_force,
                                             d->efc_AR_rownnz[i+j],
                                             d->efc_AR_colind + d->efc_AR_rowadr[i+j]);
    }
  }

  // dense
  else {
    for (int j=0; j < dim; j++) {
      res[j] = d->efc_b[i+j] + sim_math_dot(d->efc_AR+(i+j)*nefc, d->efc_force, nefc);
    }
  }

  if (flg_subR) {
    for (int j=0; j < dim; j++) {
      res[j] -= d->efc_R[i+j]*d->efc_force[i+j];
    }
  }
}


// compute cost change
// TODO: b/295296178 - add island support to Dual solvers
static sim_scalar_t costChange(const sim_scalar_t* A, sim_scalar_t* force, const sim_scalar_t* oldforce,
                         const sim_scalar_t* res, int dim) {
  sim_scalar_t change;

  // compute change
  if (dim == 1) {
    sim_scalar_t delta = force[0] - oldforce[0];
    change = 0.5*delta*delta*A[0] + delta*res[0];
  } else {
    sim_scalar_t delta[6];
    sim_math_sub(delta, force, oldforce, dim);
    change = 0.5*sim_math_mulVecMatVec(delta, A, delta, dim) + sim_math_dot(delta, res, dim);
  }

  // positive change: restore force
  if (change > 1e-10) {
    sim_math_copy(force, oldforce, dim);
    change = 0;
  }

  return change;
}


// set efc_state to dual constraint state; return nactive
// TODO: b/295296178 - add island support to Dual solvers
static int dualState(const sim_model_t* m, const sim_data_t* d, int* state) {
  int ne = d->ne, nf = d->nf, nefc = d->nefc;
  const sim_scalar_t* force = d->efc_force;
  const sim_scalar_t* floss = d->efc_frictionloss;

  // equality and friction always active
  int nactive = ne + nf;

  // equality
  sim_math_fillInt(state, SIM_CNSTRSTATE_QUADRATIC, ne);

  // friction
  for (int i=ne; i < ne+nf; i++) {
    if (force[i] <= -floss[i]) {
      state[i] = SIM_CNSTRSTATE_LINEARPOS;  // opposite of primal
    } else if (force[i] >= floss[i]) {
      state[i] = SIM_CNSTRSTATE_LINEARNEG;
    } else {
      state[i] = SIM_CNSTRSTATE_QUADRATIC;
    }
  }

  // limit and contact
  for (int i=ne+nf; i < nefc; i++) {
    // non-negative
    if (d->efc_type[i] != SIM_CNSTR_CONTACT_ELLIPTIC) {
      if (force[i] <= 0) {
        state[i] = SIM_CNSTRSTATE_SATISFIED;
      } else {
        state[i] = SIM_CNSTRSTATE_QUADRATIC;
        nactive++;
      }
    }

    // elliptic
    else {
      // get contact dimensionality, friction, mu
      sim_contact_t* con = d->contact + d->efc_id[i];
      int dim = con->dim, result = 0;
      sim_scalar_t mu = con->mu, f[6];

      // f = map force to regular-cone space
      f[0] = force[i]/mu;
      for (int j=1; j < dim; j++) {
        f[j] = force[i+j]/con->friction[j-1];
      }

      // N = normal, T = norm of tangent vector
      sim_scalar_t N = f[0];
      sim_scalar_t T = sim_math_norm(f+1, dim-1);

      // top zone
      if (mu*N >= T) {
        result = SIM_CNSTRSTATE_SATISFIED;
      }

      // bottom zone
      else if (N+mu*T <= 0) {
        result = SIM_CNSTRSTATE_QUADRATIC;
        nactive += dim;
      }

      // middle zone
      else {
        result = SIM_CNSTRSTATE_CONE;
        nactive += dim;
      }

      // replicate state in all cone dimensions
      sim_math_fillInt(state+i, result, dim);

      // advance
      i += (dim-1);
    }
  }

  return nactive;
}


//---------------------------- PGS solver ----------------------------------------------------------

// TODO: b/295296178 - add island support to Dual solvers
void sim_solPGS(const sim_model_t* m, sim_data_t* d, int maxiter) {
  int ne = d->ne, nf = d->nf, nefc = d->nefc;
  const sim_scalar_t *floss = d->efc_frictionloss;
  sim_scalar_t *force = d->efc_force;
  sim_markStack(d);
  sim_scalar_t* ARinv = SIM_STACK_ALLOC(d, nefc, sim_scalar_t);
  int* oldstate = SIM_STACK_ALLOC(d, nefc, int);

  // TODO: b/295296178 - Use island index (currently hardcoded to 0)
  int island = 0;
  sim_scalar_t scale = 1 / (m->stat.meaninertia * SIM_MAX(1, m->nv));

  // precompute inverse diagonal of AR
  ARdiaginv(m, d, ARinv, 0);

  // initial constraint state
  dualState(m, d, d->efc_state);

  // main iteration
  int iter = 0;
  while (iter < maxiter) {
    // clear improvement
    sim_scalar_t improvement = 0;

    // perform one sweep
    for (int i=0; i < nefc; i++) {
      // get constraint dimensionality
      int dim;
      if (d->efc_type[i] == SIM_CNSTR_CONTACT_ELLIPTIC) {
        dim = d->contact[d->efc_id[i]].dim;
      } else {
        dim = 1;
      }

      // compute residual for this constraint
      sim_scalar_t res[6];
      residual(m, d, res, i, dim, 0);

      // save old force
      sim_scalar_t oldforce[6];
      sim_math_copy(oldforce, force+i, dim);

      // allocate AR submatrix, required later for costChage
      sim_scalar_t Athis[36];

      // simple constraint
      if (d->efc_type[i] != SIM_CNSTR_CONTACT_ELLIPTIC) {
        // unconstrained minimum
        force[i] -= res[0]*ARinv[i];

        // impose interval and inequality constraints
        if (i >= ne && i < ne+nf) {
          if (force[i] < -floss[i]) {
            force[i] = -floss[i];
          } else if (force[i] > floss[i]) {
            force[i] = floss[i];
          }
        } else if (i >= ne+nf) {
          if (force[i] < 0) {
            force[i] = 0;
          }
        }
      }

      // elliptic cone constraint
      else {
        // get friction
        sim_scalar_t *mu =  d->contact[d->efc_id[i]].friction;

        //-------------------- perform normal or ray update

        // Athis = AR(this,this)
        extractBlock(m, d, Athis, i, dim, 0);

        // normal force too small: normal update
        if (force[i] < SIM_MINVAL) {
          // unconstrained minimum
          force[i] -= res[0]*ARinv[i];

          // clamp
          if (force[i] < 0) {
            force[i] = 0;
          }

          // clear friction (just in case)
          sim_math_zero(force+i+1, dim-1);
        }

        // ray update
        else {
          // v = ray
          sim_scalar_t v[6];
          sim_math_copy(v, force+i, dim);

          // denom = v' * AR(this,this) * v
          sim_scalar_t v1[6];
          sim_math_mulMatVec(v1, Athis, v, dim, dim);
          sim_scalar_t denom = sim_math_dot(v, v1, dim);

          // avoid division by 0
          if (denom >= SIM_MINVAL) {
            // x = v' * res / denom
            sim_scalar_t x = -sim_math_dot(v, res, dim) / denom;

            // make sure normal is non-negative
            if (force[i]+x*v[0] < 0) {
              x = -v[0]/force[i];
            }

            //  add x*v to f
            for (int j=0; j < dim; j++) {
              force[i+j] += x*v[j];
            }
          }
        }

        //-------------------- perform friction update, keep normal fixed

        // Ac = AR-submatrix; bc = b-subvector + Ac,rest * f_rest
        sim_scalar_t bc[5], Ac[25];
        sim_math_copy(bc, res+1, dim-1);
        for (int j=0; j < dim-1; j++) {
          sim_math_copy(Ac+j*(dim-1), Athis+(j+1)*dim+1, dim-1);
          bc[j] -= sim_math_dot(Ac+j*(dim-1), oldforce+1, dim-1);
          bc[j] += Athis[(j+1)*dim]*(force[i]-oldforce[0]);
        }

        // guard for f_normal==0
        if (force[i] < SIM_MINVAL) {
          sim_math_zero(force+i+1, dim-1);
        }

        // QCQP
        else {
          int flg_active;
          sim_scalar_t v[6];

          // solve
          if (dim == 3) {
            flg_active = sim_math_QCQP2(v, Ac, bc, mu, force[i]);
          } else if (dim == 4) {
            flg_active = sim_math_QCQP3(v, Ac, bc, mu, force[i]);
          } else {
            flg_active = sim_math_QCQP(v, Ac, bc, mu, force[i], dim-1);
          }

          // on constraint: put v on ellipsoid, in case QCQP is approximate
          if (flg_active) {
            sim_scalar_t s = 0;
            for (int j=0; j < dim-1; j++) {
              s += v[j]*v[j] / (mu[j]*mu[j]);
            }
            s = sim_math_sqrt(force[i]*force[i] / sim_math_max(SIM_MINVAL, s));
            for (int j=0; j < dim-1; j++) {
              v[j] *= s;
            }
          }

          // assign
          sim_math_copy(force+i+1, v, dim-1);
        }
      }

      // accumulate improvement
      if (dim == 1) {
        Athis[0] = 1/ARinv[i];
      }
      improvement -= costChange(Athis, force+i, oldforce, res, dim);

      // skip the rest of this constraint
      i += (dim-1);
    }

    // process state
    sim_math_copyInt(oldstate, d->efc_state, nefc);
    int nactive = dualState(m, d, d->efc_state);
    int nchange = 0;
    for (int i=0; i < nefc; i++) {
      nchange += (oldstate[i] != d->efc_state[i]);
    }

    // scale improvement, save stats
    improvement *= scale;
    saveStats(m, d, island, iter, improvement, 0, 0, nactive, nchange, 0, 0);

    // increment iteration count
    iter++;


    // terminate
    if (improvement < m->opt.tolerance) {
      break;
    }
  }

  // finalize statistics
  if (island < SIM_NISLAND) {
    // update solver iterations
    d->solver_niter[island] += iter;

    // set nnz
    if (sim_isSparse(m)) {
      d->solver_nnz[island] = 0;
      for (int i=0; i < nefc; i++) {
        d->solver_nnz[island] += d->efc_AR_rownnz[i];
      }
    } else {
      d->solver_nnz[island] = nefc*nefc;
    }
  }

  // map to joint space
  dualFinish(m, d);

  sim_freeStack(d);
}


//---------------------------- NoSlip solver -------------------------------------------------------

// TODO: b/295296178 - add island support to Dual solvers
void sim_solNoSlip(const sim_model_t* m, sim_data_t* d, int maxiter) {
  int dim, iter = 0, ne = d->ne, nf = d->nf, nefc = d->nefc;
  const sim_scalar_t *floss = d->efc_frictionloss;
  sim_scalar_t *force = d->efc_force;
  sim_scalar_t *mu, improvement;
  sim_scalar_t v[5], Ac[25], bc[5], res[5], oldforce[5], delta[5], mid, y, K0, K1;
  sim_contact_t* con;
  sim_markStack(d);
  sim_scalar_t* ARinv = SIM_STACK_ALLOC(d, nefc, sim_scalar_t);
  int* oldstate = SIM_STACK_ALLOC(d, nefc, int);

  // TODO: b/295296178 - Use island index (currently hardcoded to 0)
  int island = 0;
  sim_scalar_t scale = 1 / (m->stat.meaninertia * SIM_MAX(1, m->nv));

  // precompute inverse diagonal of A
  ARdiaginv(m, d, ARinv, 1);

  // initial constraint state
  dualState(m, d, d->efc_state);

  // main iteration
  while (iter < maxiter) {
    // clear improvement
    improvement = 0;

    // correct for cost change at iter 0
    if (iter == 0) {
      for (int i=0; i < nefc; i++) {
        improvement += 0.5*force[i]*force[i]*d->efc_R[i];
      }
    }

    // perform one sweep: dry friction
    for (int i=ne; i < ne+nf; i++) {
      // compute residual, save old
      residual(m, d, res, i, 1, 1);
      oldforce[0] = force[i];

      // unconstrained minimum
      force[i] -= res[0]*ARinv[i];

      // impose interval constraints
      if (force[i] < -floss[i]) {
        force[i] = -floss[i];
      } else if (force[i] > floss[i]) {
        force[i] = floss[i];
      }

      // add to improvement
      delta[0] = force[i] - oldforce[0];
      improvement -= 0.5*delta[0]*delta[0]/ARinv[i] + delta[0]*res[0];
    }

    // perform one sweep: contact friction
    for (int i=ne+nf; i < nefc; i++) {
      // pyramidal contact
      if (d->efc_type[i] == SIM_CNSTR_CONTACT_PYRAMIDAL) {
        // get contact info
        con = d->contact + d->efc_id[i];
        dim = con->dim;
        mu = con->friction;

        // loop over pairs of opposing pyramid edges
        for (int j=i; j < i+2*(dim-1); j+=2) {
          // compute residual, save old
          residual(m, d, res, j, 2, 1);
          sim_math_copy(oldforce, force+j, 2);

          // Ac = AR-submatirx
          extractBlock(m, d, Ac, j, 2, 1);

          // bc = b-subvector + Ac,rest * f_rest
          sim_math_copy(bc, res, 2);
          for (int k=0; k < 2; k++) {
            bc[k] -= sim_math_dot(Ac+k*2, oldforce, 2);
          }

          // f0 = mid+y, f1 = mid-y
          mid = 0.5*(force[j]+force[j+1]);
          y = 0.5*(force[j]-force[j+1]);

          // K1 = A00 + A11 - 2*A01,  K0 = mid*A00 - mid*A11 + b0 - b1
          K1 = Ac[0] + Ac[3] - Ac[1] - Ac[2];
          K0 = mid*(Ac[0] - Ac[3]) + bc[0] - bc[1];

          // guard against Ac==0
          if (K1 < SIM_MINVAL) {
            force[j] = force[j+1] = mid;
          }

          // otherwise minimize over y \in [-mid, mid]
          else {
            // unconstrained minimum
            y = -K0/K1;

            // clamp and assign
            if (y < -mid) {
              force[j] = 0;
              force[j+1] = 2*mid;
            } else if (y > mid) {
              force[j] = 2*mid;
              force[j+1] = 0;
            } else {
              force[j] = mid+y;
              force[j+1] = mid-y;
            }
          }

          // accumulate improvement
          improvement -= costChange(Ac, force+j, oldforce, res, 2);
        }

        // skip the rest of this contact
        i += 2*(dim-1)-1;
      }

      // elliptic contact
      else if (d->efc_type[i] == SIM_CNSTR_CONTACT_ELLIPTIC) {
        // get contact info
        con = d->contact + d->efc_id[i];
        dim = con->dim;
        mu = con->friction;

        // compute residual, save old
        residual(m, d, res, i+1, dim-1, 1);
        sim_math_copy(oldforce, force+i+1, dim-1);

        // Ac = AR-submatrix
        extractBlock(m, d, Ac, i+1, dim-1, 1);

        // bc = b-subvector + Ac,rest * f_rest
        sim_math_copy(bc, res, dim-1);
        for (int j=0; j < dim-1; j++) {
          bc[j] -= sim_math_dot(Ac+j*(dim-1), oldforce, dim-1);
        }

        // guard for f_normal==0
        if (force[i] < SIM_MINVAL) {
          sim_math_zero(force+i+1, dim-1);
        }

        // QCQP
        else {
          int flg_active = 0;

          // solve
          if (dim == 3) {
            flg_active = sim_math_QCQP2(v, Ac, bc, mu, force[i]);
          } else if (dim == 4) {
            flg_active = sim_math_QCQP3(v, Ac, bc, mu, force[i]);
          } else {
            flg_active = sim_math_QCQP(v, Ac, bc, mu, force[i], dim-1);
          }

          // on constraint: put v on ellipsoid, in case QCQP is approximate
          if (flg_active) {
            sim_scalar_t s = 0;
            for (int j=0; j < dim-1; j++) {
              s += v[j]*v[j]/(mu[j]*mu[j]);
            }
            s = sim_math_sqrt(force[i]*force[i] / sim_math_max(SIM_MINVAL, s));
            for (int j=0; j < dim-1; j++) {
              v[j] *= s;
            }
          }

          // assign
          sim_math_copy(force+i+1, v, dim-1);
        }

        // accumulate improvement
        improvement -= costChange(Ac, force+i+1, oldforce, res, dim-1);

        // skip the rest of this contact
        i += (dim-1);
      }
    }

    // process state
    sim_math_copyInt(oldstate, d->efc_state, nefc);
    int nactive = dualState(m, d, d->efc_state);
    int nchange = 0;
    for (int i=0; i < nefc; i++) {
      nchange += (oldstate[i] != d->efc_state[i]);
    }

    // scale improvement, save stats
    improvement *= scale;

    // save noslip stats after all the entries from regular solver
    int stats_iter = iter + d->solver_niter[island];
    saveStats(m, d, island, stats_iter, improvement, 0, 0, nactive, nchange, 0, 0);

    // increment iteration count
    iter++;

    // terminate
    if (improvement < m->opt.noslip_tolerance) {
      break;
    }
  }

  // update solver iterations
  d->solver_niter[island] += iter;

  // map to joint space
  dualFinish(m, d);

  sim_freeStack(d);
}


//------------------------- Primal solvers ---------------------------------------------------------

// Primal context
typedef struct {
  int is_sparse;          // 1: sparse, 0: dense
  int is_elliptic;        // 1: elliptic, 0: pyramidal
  int island;             // current island index, -1 if monolithic

  // sizes
  int nv;                 // number of dofs
  int ne;                 // number of equalities
  int nf;                 // number of friction constraints
  int nefc;               // number of all constraints
  int nJ;                 // number of nonzeros in Jacobian

  // contact array
  sim_contact_t* contact;

  // dof arrays
  const sim_scalar_t* qfrc_smooth;
  const sim_scalar_t* qacc_smooth;
  sim_scalar_t* qfrc_constraint;
  sim_scalar_t* qacc;

  // inertia
  const int* M_rownnz;
  const int* M_rowadr;
  const int* M_colind;
  const sim_scalar_t* M;
  const sim_scalar_t* qLD;
  const sim_scalar_t* qLDiagInv;

  // efc arrays
  const sim_scalar_t* efc_D;
  const sim_scalar_t* efc_R;
  const sim_scalar_t* efc_frictionloss;
  const sim_scalar_t* efc_aref;
  const int* efc_id;
  const int* efc_type;
  sim_scalar_t* efc_force;
  int* efc_state;

  // Jacobians
  const int* J_rownnz;
  const int* J_rowadr;
  const int* J_rowsuper;
  const int* J_colind;
  const sim_scalar_t* J;
  int* JT_rownnz;
  int* JT_rowadr;
  int* JT_rowsuper;
  int* JT_colind;
  sim_scalar_t* JT;

  // common arrays (PrimalAllocate)
  sim_scalar_t* Jaref;          // Jac*qacc - aref                              (nefc x 1)
  sim_scalar_t* Jv;             // Jac*search                                   (nefc x 1)
  sim_scalar_t* Ma;             // M*qacc                                       (nv x 1)
  sim_scalar_t* Mv;             // M*search                                     (nv x 1)
  sim_scalar_t* grad;           // gradient of master cost                      (nv x 1)
  sim_scalar_t* Mgrad;          // M\grad or H\grad                             (nv x 1)
  sim_scalar_t* search;         // linesearch vector                            (nv x 1)
  sim_scalar_t* quad;           // quadratic polynomials for constraint costs   (nefc x 3)

  // Newton arrays, known-size (PrimalAllocate)
  sim_scalar_t* D;              // constraint inertia                           (nefc x 1)
  int* H_rowadr;          // Hessian row addresses                        (nv x 1)
  int* H_rownnz;          // Hessian row nonzeros                         (nv x 1)
  int* HT_rownnz;         // Hessian transpose row nonzeros               (nv x 1)
  int* HT_rowadr;         // Hessian transpose row addresses              (nv x 1)
  int* L_rownnz;          // Hessian factor row nonzeros                  (nv x 1)
  int* L_rowadr;          // Hessian factor row addresses                 (nv x 1)
  int* LT_rownnz;         // Hessian factor transpose row nonzeros        (nv x 1)
  int* LT_rowadr;         // Hessian factor transpose row addresses       (nv x 1)
  int* buf_ind;           // index buffer for sparse addition             (nv x 1)
  sim_scalar_t* buf_val;        // value buffer for sparse addition             (nv x 1)

  // Newton arrays, computed-size (MakeHessian)
  int nH;                 // number of nonzeros in Hessian H
  int* H_colind;          // Hessian column indices                       (nH x 1)
  int* HT_colind;         // Hessian transpose column indices             (nH x 1)
  sim_scalar_t* H;              // Hessian                                      (nH x 1)
  int nL;                 // number of nonzeros in Cholesky factor L
  int* L_colind;          // Cholesky factor column indices               (nL x 1)
  int* LT_colind;         // Cholesky factor transpose column indices     (nL x 1)
  int* LT_map;            // CSC-to-CSR index mapping                     (nL x 1)
  sim_scalar_t* L;              // Cholesky factor                              (nL x 1)
  sim_scalar_t* Lcone;          // Cholesky factor with cone contributions      (nL x 1)

  // globals
  sim_scalar_t cost;            // constraint + Gauss cost
  sim_scalar_t quadGauss[3];    // quadratic polynomial for Gauss cost
  sim_scalar_t scale;           // scaling factor for improvement and gradient
  int nactive;            // number of active constraints
  int ncone;              // number of contacts in cone state
  int nupdate;            // number of Cholesky updates

  // linesearch diagnostics
  int LSiter;             // number of linesearch iterations
  int LSresult;           // linesearch result
  sim_scalar_t LSslope;         // linesearch slope at solution
} SIM_PrimalContext;


// set sizes and pointers to sim_data_t arrays in SIM_PrimalContext
static void PrimalPointers(const sim_model_t* m, const sim_data_t* d, SIM_PrimalContext* ctx, int island) {
  // clear everything
  memset(ctx, 0, sizeof(SIM_PrimalContext));

  // globals
  ctx->is_sparse = sim_isSparse(m);
  ctx->is_elliptic = (m->opt.cone == SIM_CONE_ELLIPTIC);
  ctx->contact = d->contact;
  ctx->island = island;

  // set sizes and pointers (monolithic)
  if (island < 0) {
    // sizes
    ctx->nv               = m->nv;
    ctx->ne               = d->ne;
    ctx->nf               = d->nf;
    ctx->nefc             = d->nefc;
    ctx->nJ               = d->nJ;

    // dof arrays
    ctx->qfrc_smooth      = d->qfrc_smooth;
    ctx->qfrc_constraint  = d->qfrc_constraint;
    ctx->qacc_smooth      = d->qacc_smooth;
    ctx->qacc             = d->qacc;

    // inertia
    ctx->M_rownnz         = m->M_rownnz;
    ctx->M_rowadr         = m->M_rowadr;
    ctx->M_colind         = m->M_colind;
    ctx->M                = d->M;
    ctx->qLD              = d->qLD;
    ctx->qLDiagInv        = d->qLDiagInv;

    // efc arrays
    ctx->efc_D            = d->efc_D;
    ctx->efc_R            = d->efc_R;
    ctx->efc_frictionloss = d->efc_frictionloss;
    ctx->efc_aref         = d->efc_aref;
    ctx->efc_id           = d->efc_id;
    ctx->efc_type         = d->efc_type;
    ctx->efc_force        = d->efc_force;
    ctx->efc_state        = d->efc_state;

    // Jacobians
    ctx->J                = d->efc_J;
    if (ctx->is_sparse) {
      ctx->J_rownnz       = d->efc_J_rownnz;
      ctx->J_rowadr       = d->efc_J_rowadr;
      ctx->J_rowsuper     = d->efc_J_rowsuper;
      ctx->J_colind       = d->efc_J_colind;
    }
  }

  // set sizes and pointers (per-island)
  else {
    // sizes
    ctx->nv               = d->island_nv[island];
    ctx->ne               = d->island_ne[island];
    ctx->nf               = d->island_nf[island];
    ctx->nefc             = d->island_nefc[island];

    // dof arrays
    int idofadr           = d->island_idofadr[island];
    ctx->qfrc_smooth      = d->ifrc_smooth       + idofadr;
    ctx->qfrc_constraint  = d->ifrc_constraint   + idofadr;
    ctx->qacc_smooth      = d->iacc_smooth       + idofadr;
    ctx->qacc             = d->iacc              + idofadr;

    // inertia
    ctx->M_rownnz         = d->iM_rownnz         + idofadr;
    ctx->M_rowadr         = d->iM_rowadr         + idofadr;
    ctx->M_colind         = d->iM_colind;
    ctx->M                = d->iM;
    ctx->qLD              = d->iLD;
    ctx->qLDiagInv        = d->iLDiagInv         + idofadr;

    // efc arrays
    int iefcadr           = d->island_iefcadr[island];
    ctx->efc_D            = d->iefc_D            + iefcadr;
    ctx->efc_R            = d->iefc_R            + iefcadr;
    ctx->efc_frictionloss = d->iefc_frictionloss + iefcadr;
    ctx->efc_aref         = d->iefc_aref         + iefcadr;
    ctx->efc_id           = d->iefc_id           + iefcadr;
    ctx->efc_type         = d->iefc_type         + iefcadr;
    ctx->efc_force        = d->iefc_force        + iefcadr;
    ctx->efc_state        = d->iefc_state        + iefcadr;

    // Jacobians
    if (!ctx->is_sparse) {
      ctx->J              = d->iefc_J + d->nidof * iefcadr;
    } else {
      ctx->J_rownnz       = d->iefc_J_rownnz     + iefcadr;
      ctx->J_rowadr       = d->iefc_J_rowadr     + iefcadr;
      ctx->J_rowsuper     = d->iefc_J_rowsuper   + iefcadr;
      ctx->J_colind       = d->iefc_J_colind;
      ctx->J              = d->iefc_J;
      ctx->nJ             = ctx->J_rowadr[ctx->nefc-1] + ctx->J_rownnz[ctx->nefc-1]
                            - ctx->J_rowadr[0];
    }
  }
}


// allocate fixed-size arrays in SIM_PrimalContext
//  sim_{mark/free}Stack in calling function!
static void PrimalAllocate(sim_data_t* d, SIM_PrimalContext* ctx, int flg_Newton) {
  // local sizes
  int nv = ctx->nv;
  int nefc = ctx->nefc;

  // common arrays
  ctx->Jaref  = SIM_STACK_ALLOC(d, nefc, sim_scalar_t);
  ctx->Jv     = SIM_STACK_ALLOC(d, nefc, sim_scalar_t);
  ctx->Ma     = SIM_STACK_ALLOC(d, nv, sim_scalar_t);
  ctx->Mv     = SIM_STACK_ALLOC(d, nv, sim_scalar_t);
  ctx->grad   = SIM_STACK_ALLOC(d, nv, sim_scalar_t);
  ctx->Mgrad  = SIM_STACK_ALLOC(d, nv, sim_scalar_t);
  ctx->search = SIM_STACK_ALLOC(d, nv, sim_scalar_t);
  ctx->quad   = SIM_STACK_ALLOC(d, nefc*3, sim_scalar_t);

  // sparse only, compute Jacobian transpose
  if (ctx->is_sparse) {
    ctx->JT_rownnz   = SIM_STACK_ALLOC(d, nv, int);
    ctx->JT_rowadr   = SIM_STACK_ALLOC(d, nv, int);
    ctx->JT_rowsuper = SIM_STACK_ALLOC(d, nv, int);
    ctx->JT_colind   = SIM_STACK_ALLOC(d, d->nJ, int);
    ctx->JT          = SIM_STACK_ALLOC(d, d->nJ, sim_scalar_t);
    int offset       = ctx->J_rowadr[0];
    sim_math_transposeSparse(ctx->JT, ctx->J + offset, nefc, nv,
                        ctx->JT_rownnz, ctx->JT_rowadr, ctx->JT_colind, ctx->JT_rowsuper,
                        ctx->J_rownnz, ctx->J_rowadr, ctx->J_colind + offset);
  }

  // Newton only, known-size arrays
  if (flg_Newton) {
    ctx->D = SIM_STACK_ALLOC(d, nefc, sim_scalar_t);

    // sparse Newton only
    if (ctx->is_sparse) {
      ctx->H_rowadr   = SIM_STACK_ALLOC(d, nv, int);
      ctx->H_rownnz   = SIM_STACK_ALLOC(d, nv, int);
      ctx->HT_rownnz  = SIM_STACK_ALLOC(d, nv, int);
      ctx->HT_rowadr  = SIM_STACK_ALLOC(d, nv, int);
      ctx->L_rownnz   = SIM_STACK_ALLOC(d, nv, int);
      ctx->L_rowadr   = SIM_STACK_ALLOC(d, nv, int);
      ctx->LT_rownnz  = SIM_STACK_ALLOC(d, nv, int);
      ctx->LT_rowadr  = SIM_STACK_ALLOC(d, nv, int);
      ctx->buf_val    = SIM_STACK_ALLOC(d, nv, sim_scalar_t);
      ctx->buf_ind    = SIM_STACK_ALLOC(d, nv, int);
    }
  }
}


// update efc_force, qfrc_constraint, cost-related
static void PrimalUpdateConstraint(SIM_PrimalContext* ctx, int flg_HessianCone) {
  int nefc = ctx->nefc, nv = ctx->nv;

  // update constraints
  sim_constraintUpdate_impl(ctx->ne, ctx->nf, ctx->nefc, ctx->efc_D, ctx->efc_R,
                           ctx->efc_frictionloss, ctx->Jaref, ctx->efc_type, ctx->efc_id,
                           ctx->contact, ctx->efc_state, ctx->efc_force,
                           &(ctx->cost), flg_HessianCone);

  // compute qfrc_constraint (dense or sparse)
  if (!ctx->is_sparse) {
    sim_math_mulMatTVec(ctx->qfrc_constraint, ctx->J, ctx->efc_force, nefc, nv);
  } else {
    sim_math_mulMatVecSparse(ctx->qfrc_constraint, ctx->JT, ctx->efc_force, nv,
                        ctx->JT_rownnz, ctx->JT_rowadr, ctx->JT_colind, ctx->JT_rowsuper);
  }

  // count active and cone
  ctx->nactive = 0;
  ctx->ncone = 0;
  for (int i=0; i < nefc; i++) {
    ctx->nactive += (ctx->efc_state[i] != SIM_CNSTRSTATE_SATISFIED);
    ctx->ncone += (ctx->efc_state[i] == SIM_CNSTRSTATE_CONE);
  }

  // add Gauss cost, set in quadratic[0]
  sim_scalar_t Gauss = 0;
  for (int i=0; i < nv; i++) {
    Gauss += 0.5 * (ctx->Ma[i] - ctx->qfrc_smooth[i]) * (ctx->qacc[i] - ctx->qacc_smooth[i]);
  }

  ctx->quadGauss[0] = Gauss;
  ctx->cost += Gauss;
}


// update grad, Mgrad
static void PrimalUpdateGradient(SIM_PrimalContext* ctx, int flg_Newton) {
  int nv = ctx->nv;

  // grad = M*qacc - qfrc_smooth - qfrc_constraint
  for (int i=0; i < nv; i++) {
    ctx->grad[i] = ctx->Ma[i] - ctx->qfrc_smooth[i] - ctx->qfrc_constraint[i];
  }

  // Newton: Mgrad = H \ grad
  if (flg_Newton) {
    if (ctx->is_sparse) {
      sim_math_cholSolveSparse(ctx->Mgrad, (ctx->ncone ? ctx->Lcone : ctx->L),
                          ctx->grad, nv, ctx->L_rownnz, ctx->L_rowadr, ctx->L_colind);
    } else {
      sim_math_cholSolve(ctx->Mgrad, (ctx->ncone ? ctx->Lcone : ctx->L), ctx->grad, nv);
    }
  }

  // CG: Mgrad = M \ grad
  else {
    sim_math_copy(ctx->Mgrad, ctx->grad, nv);
    sim_solveLD(ctx->Mgrad, ctx->qLD, ctx->qLDiagInv, nv, 1,
               ctx->M_rownnz, ctx->M_rowadr, ctx->M_colind, NULL);
  }
}


// prepare quadratic polynomials and contact cone quantities
static void PrimalPrepare(SIM_PrimalContext* ctx) {
  int nv = ctx->nv, nefc = ctx->nefc;
  const sim_scalar_t* v = ctx->search;

  // Gauss: alpha^2*0.5*v'*M*v + alpha*v'*(Ma-qfrc_smooth) + 0.5*(a-qacc_smooth)'*(Ma-qfrc_smooth)
  //  quadGauss[0] already computed in PrimalUpdateConstraint
  ctx->quadGauss[1] = sim_math_dot(v, ctx->Ma, nv) - sim_math_dot(ctx->qfrc_smooth, v, nv);
  ctx->quadGauss[2] = 0.5*sim_math_dot(v, ctx->Mv, nv);

  // process constraints
  for (int i=0; i < nefc; i++) {
    // pointers to numeric data
    const sim_scalar_t* Jv = ctx->Jv + i;
    const sim_scalar_t* Jaref = ctx->Jaref + i;
    const sim_scalar_t* D = ctx->efc_D + i;

    // pointer to this quadratic
    sim_scalar_t* quad = ctx->quad + 3*i;

    // init with scalar quadratic
    sim_scalar_t DJ0 = D[0]*Jaref[0];
    quad[0] = Jaref[0]*DJ0;
    quad[1] = Jv[0]*DJ0;
    quad[2] = Jv[0]*D[0]*Jv[0];

    // elliptic cone: extra processing
    if (ctx->efc_type[i] == SIM_CNSTR_CONTACT_ELLIPTIC) {
      // extract contact info
      const sim_contact_t* con = ctx->contact + ctx->efc_id[i];
      int dim = con->dim;
      sim_scalar_t U[6], V[6], UU = 0, UV = 0, VV = 0, mu = con->mu;
      const sim_scalar_t* friction = con->friction;

      // complete vector quadratic (for bottom zone)
      for (int j=1; j < dim; j++) {
        sim_scalar_t DJj = D[j]*Jaref[j];
        quad[0] += Jaref[j]*DJj;
        quad[1] += Jv[j]*DJj;
        quad[2] += Jv[j]*D[j]*Jv[j];
      }

      // rescale to make primal cone circular
      U[0] = Jaref[0]*mu;
      V[0] = Jv[0]*mu;
      for (int j=1; j < dim; j++) {
        U[j] = Jaref[j]*friction[j-1];
        V[j] = Jv[j]*friction[j-1];
      }

      // accumulate sums of squares
      for (int j=1; j < dim; j++) {
        UU += U[j]*U[j];
        UV += U[j]*V[j];
        VV += V[j]*V[j];
      }

      // store in quad[3-8], using the fact that dim>=3
      quad[3] = U[0];
      quad[4] = V[0];
      quad[5] = UU;
      quad[6] = UV;
      quad[7] = VV;
      quad[8] = D[0] / ((mu*mu) * (1 + (mu*mu)));

      // advance to next constraint
      i += (dim-1);
    }

    // apply scaling
    quad[0] *= 0.5;
    quad[2] *= 0.5;
  }
}


// linesearch evaluation point
struct _mjPrimalPnt {
  sim_scalar_t alpha;
  sim_scalar_t cost;
  sim_scalar_t deriv[2];
};
typedef struct _mjPrimalPnt SIM_PrimalPnt;


// evaluate linesearch cost, return first and second derivatives
static void PrimalEval(SIM_PrimalContext* ctx, SIM_PrimalPnt* p) {
  int ne = ctx->ne, nf = ctx->nf, nefc = ctx->nefc;

  // clear result
  sim_scalar_t cost = 0, alpha = p->alpha;
  sim_scalar_t deriv[2] = {0, 0};

  // init quad with Gauss
  sim_scalar_t quadTotal[3];
  sim_math_copy_3(quadTotal, ctx->quadGauss);

  // process constraints
  for (int i=0; i < nefc; i++) {
    // equality
    if (i < ne) {
      sim_math_add_to_3(quadTotal, ctx->quad+3*i);
      continue;
    }

    // friction
    if (i < ne + nf) {
      // search point, friction loss, bound (Rf)
      sim_scalar_t start = ctx->Jaref[i], dir = ctx->Jv[i];
      sim_scalar_t x = start + alpha*dir;
      sim_scalar_t f = ctx->efc_frictionloss[i];
      sim_scalar_t Rf = ctx->efc_R[i]*f;

      // -bound < x < bound : quadratic
      if (-Rf < x && x < Rf) {
        sim_math_add_to_3(quadTotal, ctx->quad+3*i);
      }

      // x < -bound : linear negative
      else if (x <= -Rf) {
        sim_scalar_t qf[3] = {f*(-0.5*Rf-start), -f*dir, 0};
        sim_math_add_to_3(quadTotal, qf);
      }

      // bound < x : linear positive
      else {
        sim_scalar_t qf[3] = {f*(-0.5*Rf+start), f*dir, 0};
        sim_math_add_to_3(quadTotal, qf);
      }
      continue;
    }

    // limit and contact
    if (ctx->efc_type[i] == SIM_CNSTR_CONTACT_ELLIPTIC) {         // elliptic cone
      // extract contact info
      const sim_contact_t* con = ctx->contact + ctx->efc_id[i];
      sim_scalar_t* quad = ctx->quad + 3*i;
      int dim = con->dim;
      sim_scalar_t mu = con->mu;

      // unpack quad
      sim_scalar_t U0 = quad[3];
      sim_scalar_t V0 = quad[4];
      sim_scalar_t UU = quad[5];
      sim_scalar_t UV = quad[6];
      sim_scalar_t VV = quad[7];
      sim_scalar_t Dm = quad[8];

      // compute N, Tsqr
      sim_scalar_t N = U0 + alpha*V0;
      sim_scalar_t Tsqr = UU + alpha*(2*UV + alpha*VV);

      // no tangential force : top or bottom zone
      if (Tsqr <= 0) {
        // bottom zone: quadratic cost
        if (N < 0) {
          sim_math_add_to_3(quadTotal, quad);
        }

        // top zone: nothing to do
      }

      // otherwise regular processing
      else {
        // tangential force
        sim_scalar_t T = sim_math_sqrt(Tsqr);

        // N>=mu*T : top zone
        if (N >= mu*T) {
          // nothing to do
        }

        // mu*N+T<=0 : bottom zone
        else if (mu*N+T <= 0) {
          sim_math_add_to_3(quadTotal, quad);
        }

        // otherwise middle zone
        else {
          // derivatives
          sim_scalar_t N1 = V0;
          sim_scalar_t T1 = (UV + alpha*VV)/T;
          sim_scalar_t T2 = VV/T - (UV + alpha*VV)*T1/(T*T);

          // add to cost
          cost += 0.5*Dm*(N-mu*T)*(N-mu*T);
          deriv[0] += Dm*(N-mu*T)*(N1-mu*T1);
          deriv[1] += Dm*((N1-mu*T1)*(N1-mu*T1) + (N-mu*T)*(-mu*T2));
        }
      }

      // advance to next constraint
      i += (dim-1);
    } else {                                                  // inequality
      // search point
      sim_scalar_t x = ctx->Jaref[i] + alpha*ctx->Jv[i];

      // active
      if (x < 0) {
        sim_math_add_to_3(quadTotal, ctx->quad+3*i);
      }
    }
  }

  // add total quadratic
  cost += alpha*alpha*quadTotal[2] + alpha*quadTotal[1] + quadTotal[0];
  deriv[0] += 2*alpha*quadTotal[2] + quadTotal[1];
  deriv[1] += 2*quadTotal[2];

  // check for convexity; SHOULD NOT OCCUR
  if (deriv[1] <= 0) {
    sim_warning("Linesearch objective is not convex");
    deriv[1] = SIM_MINVAL;
  }

  // assign and count
  p->cost = cost;
  p->deriv[0] = deriv[0];
  p->deriv[1] = deriv[1];
  ctx->LSiter++;
}


// update bracket point given 3 candidate points
static int updateBracket(SIM_PrimalContext* ctx,
                         SIM_PrimalPnt* p, const SIM_PrimalPnt candidates[3], SIM_PrimalPnt* pnext) {
  int flag = 0;
  for (int i=0; i < 3; i++) {
    // negative deriv
    if (p->deriv[0] < 0 && candidates[i].deriv[0] < 0 && p->deriv[0] < candidates[i].deriv[0]) {
      *p = candidates[i];
      flag = 1;
    }

    // positive deriv
    else if (p->deriv[0] > 0 &&
             candidates[i].deriv[0] > 0 &&
             p->deriv[0] > candidates[i].deriv[0]) {
      *p = candidates[i];
      flag = 2;
    }
  }

  // compute next point if updated
  if (flag) {
    pnext->alpha = p->alpha - p->deriv[0]/p->deriv[1];
    PrimalEval(ctx, pnext);
  }

  return flag;
}


// line search
static sim_scalar_t PrimalSearch(SIM_PrimalContext* ctx, sim_scalar_t tolerance, sim_scalar_t ls_iterations) {
  int nv = ctx->nv, nefc = ctx->nefc;
  SIM_PrimalPnt p0, p1, p2, pmid, p1next, p2next;

  // clear results
  ctx->LSiter = 0;
  ctx->LSresult = 0;
  ctx->LSslope = 1;       // means not computed

  // save search vector length, check
  sim_scalar_t snorm = sim_math_norm(ctx->search, nv);
  if (snorm < SIM_MINVAL) {
    ctx->LSresult = 1;                          // search vector too small
    return 0;
  }

  // compute scaled gradtol and slope scaling
  sim_scalar_t gtol = tolerance * snorm / ctx->scale;
  sim_scalar_t slopescl = ctx->scale / snorm;

  // compute Mv = M * v
  sim_math_mulSymVecSparse(ctx->Mv, ctx->M, ctx->search, nv,
                      ctx->M_rownnz, ctx->M_rowadr, ctx->M_colind);

  // compute Jv = J * search  (dense or sparse)
  if (!ctx->is_sparse) {
    sim_math_mulMatVec(ctx->Jv, ctx->J, ctx->search, nefc, nv);
  } else {
    sim_math_mulMatVecSparse(ctx->Jv, ctx->J, ctx->search, nefc,
                        ctx->J_rownnz, ctx->J_rowadr, ctx->J_colind, ctx->J_rowsuper);
  }

  // prepare quadratics and cones
  PrimalPrepare(ctx);

  // init at alpha = 0, save
  p0.alpha = 0;
  PrimalEval(ctx, &p0);

  // always attempt one Newton step
  p1.alpha = p0.alpha - p0.deriv[0]/p0.deriv[1];
  PrimalEval(ctx, &p1);

  // check for initial convergence
  if (sim_math_abs(p1.deriv[0]) < gtol) {
    if (p1.alpha == 0) {
      ctx->LSresult = 2;  // no improvement, initial convergence
    } else {
      ctx->LSresult = 0;  // SUCCESS
    }
    ctx->LSslope = sim_math_abs(p1.deriv[0])*slopescl;
    return p1.alpha;
  }

  // save direction
  int dir = (p1.deriv[0] < 0 ? +1 : -1);

  // SANITY CHECKS
  /*
     // descent direction
     if( sim_math_dot(ctx->grad, ctx->search, m->nv)>=0 )
      printf("NOT A DESCENT:  grad %g   search %g   dot %g\n",
          sim_math_norm(ctx->grad, m->nv),
          sim_math_norm(ctx->search, m->nv),
          sim_math_dot(ctx->grad, ctx->search, m->nv));

     // 2nd derivative for Newton cone
     if( ctx->flg_Newton && ctx->ncone )
     {
      sim_scalar_t dd = -p0.deriv[0]/p0.deriv[1];

      if( sim_math_abs(dd-1)>1e-6 )
          printf("2nd DERIVATIVE FAIL: d0  %g   d1  %g   alpha %g\n",
              p0.deriv[0], p0.deriv[1], dd);
     }

     // cost and gradient at 0: full-space vs. linesearch
     sim_scalar_t grd = sim_math_dot(ctx->grad, ctx->search, m->nv);
     if( sim_math_abs(p0.cost-ctx->cost)/SIM_MAX(SIM_MINVAL,sim_math_abs(p0.cost+ctx->cost)) > 1e-6 ||
      sim_math_abs(p0.deriv[0]-grd)/SIM_MAX(SIM_MINVAL,sim_math_abs(p0.deriv[0]+grd)) > 1e-6 )
     {
      printf("LSiter = %d:\n", ctx->LSiter);
      printf("COST:  %g  %g  %g\n",
          p0.cost, ctx->cost,
          sim_math_abs(p0.cost-ctx->cost)/SIM_MAX(SIM_MINVAL,sim_math_abs(p0.cost+ctx->cost)));
      printf("GRAD:  %g  %g  %g\n",
          p0.deriv[0], grd,
          sim_math_abs(p0.deriv[0]-grd)/SIM_MAX(SIM_MINVAL,sim_math_abs(p0.deriv[0]+grd)));
     }
   */

  // one-sided search
  p2 = p0;
  int p2update = 1;
  while (p1.deriv[0]*dir <= -gtol && ctx->LSiter < ls_iterations) {
    // save current
    p2 = p1;
    p2update = 1;

    // move to Newton point w.r.t current
    p1.alpha -= p1.deriv[0]/p1.deriv[1];
    PrimalEval(ctx, &p1);

    // check for convergence
    if (sim_math_abs(p1.deriv[0]) < gtol) {
      ctx->LSslope = sim_math_abs(p1.deriv[0])*slopescl;
      return p1.alpha;                          // SUCCESS
    }
  }

  // check for failure to bracket
  if (ctx->LSiter >= ls_iterations) {
    ctx->LSresult = 3;                          // could not bracket
    ctx->LSslope = sim_math_abs(p1.deriv[0])*slopescl;
    return p1.alpha;
  }

  // check for p2 update; SHOULD NOT OCCUR
  if (!p2update) {
    ctx->LSresult = 6;                          // no p2 update
    ctx->LSslope = sim_math_abs(p1.deriv[0])*slopescl;
    return p1.alpha;
  }

  // compute next-points for bracket
  p2next = p1;
  p1next.alpha = p1.alpha - p1.deriv[0]/p1.deriv[1];
  PrimalEval(ctx, &p1next);

  // bracketed search
  while (ctx->LSiter < ls_iterations) {
    // evaluate at midpoint
    pmid.alpha = 0.5*(p1.alpha + p2.alpha);
    PrimalEval(ctx, &pmid);

    // make list of candidates
    SIM_PrimalPnt candidates[3] = {p1next, p2next, pmid};

    // check candidates for convergence
    sim_scalar_t bestcost = 0;
    int bestind = -1;
    for (int i=0; i < 3; i++) {
      if (sim_math_abs(candidates[i].deriv[0]) < gtol &&
          (bestind == -1 || candidates[i].cost < bestcost)) {
        bestcost = candidates[i].cost;
        bestind = i;
      }
    }
    if (bestind >= 0) {
      ctx->LSslope = sim_math_abs(candidates[bestind].deriv[0])*slopescl;
      return candidates[bestind].alpha;       // SUCCESS
    }

    // update brackets
    int b1 = updateBracket(ctx, &p1, candidates, &p1next);
    int b2 = updateBracket(ctx, &p2, candidates, &p2next);

    // no update possible: numerical accuracy reached, use midpoint
    if (!b1 && !b2) {
      if (pmid.cost < p0.cost) {
        ctx->LSresult = 0;  // SUCCESS
      } else {
        ctx->LSresult = 7;  // no improvement, could not bracket
      }

      ctx->LSslope = sim_math_abs(pmid.deriv[0])*slopescl;
      return pmid.alpha;
    }
  }

  // choose bracket with best cost
  if (p1.cost <= p2.cost && p1.cost < p0.cost) {
    ctx->LSresult = 4;                          // improvement but no convergence
    ctx->LSslope = sim_math_abs(p1.deriv[0])*slopescl;
    return p1.alpha;
  } else if (p2.cost <= p1.cost && p2.cost < p0.cost) {
    ctx->LSresult = 4;                          // improvement but no convergence
    ctx->LSslope = sim_math_abs(p2.deriv[0])*slopescl;
    return p2.alpha;
  } else {
    ctx->LSresult = 5;                          // no improvement
    return 0;
  }
}


// allocate and compute Hessian given efc_state
//  sim_{mark/free}Stack in caller function!
static void MakeHessian(sim_data_t* d, SIM_PrimalContext* ctx) {
  int nv = ctx->nv, nefc = ctx->nefc;

  // compute constraint inertia
  for (int i=0; i < nefc; i++) {
    ctx->D[i] = ctx->efc_state[i] == SIM_CNSTRSTATE_QUADRATIC ? ctx->efc_D[i] : 0;
  }

  // sparse
  if (ctx->is_sparse) {
    // initialize Hessian rowadr, rownnz; get total nonzeros
    ctx->nH = sim_math_sqrMatTDSparseCount(ctx->H_rownnz, ctx->H_rowadr, nv,
                                      ctx->J_rownnz, ctx->J_rowadr, ctx->J_colind,
                                      ctx->JT_rownnz, ctx->JT_rowadr, ctx->JT_colind,
                                      ctx->JT_rowsuper, d, /*flg_upper=*/0);

    // add M nonzeros to Hessian total (unavoidable overcounting since H_colind is still unknown)
    ctx->nH += ctx->M_rowadr[nv - 1] + ctx->M_rownnz[nv - 1];

    // shift H row addresses to make room for C
    int shift = 0;
    for (int r = 0; r < nv - 1; r++) {
      shift += ctx->M_rownnz[r];
      ctx->H_rowadr[r + 1] += shift;
    }

    // allocate H_colind and H
    ctx->H_colind = SIM_STACK_ALLOC(d, ctx->nH, int);
    ctx->H = SIM_STACK_ALLOC(d, ctx->nH, sim_scalar_t);

    // compute H = J'*D*J
    sim_math_sqrMatTDSparse(ctx->H, ctx->J, ctx->JT, ctx->D, nefc, nv,
                       ctx->H_rownnz, ctx->H_rowadr, ctx->H_colind,
                       ctx->J_rownnz, ctx->J_rowadr, ctx->J_colind, NULL,
                       ctx->JT_rownnz, ctx->JT_rowadr, ctx->JT_colind, ctx->JT_rowsuper,
                       d, /*diagind=*/NULL);

    // add mass matrix: H = J'*D*J + C
    sim_math_addToMatSparse(ctx->H, ctx->H_rownnz, ctx->H_rowadr, ctx->H_colind, nv,
                       ctx->M, ctx->M_rownnz, ctx->M_rowadr, ctx->M_colind,
                       ctx->buf_val, ctx->buf_ind);

    // compute H' (upper triangle, required for symbolic Cholesky)
    ctx->HT_colind = SIM_STACK_ALLOC(d, ctx->nH, int);
    sim_math_transposeSparse(NULL, NULL, nv, nv, ctx->HT_rownnz, ctx->HT_rowadr, ctx->HT_colind, NULL,
                        ctx->H_rownnz, ctx->H_rowadr, ctx->H_colind);

    // count total and row non-zeros of reverse-Cholesky factors L and LT
    ctx->nL = sim_math_cholFactorSymbolic(NULL, ctx->L_rownnz, ctx->L_rowadr, NULL,
                                     ctx->LT_rownnz, ctx->LT_rowadr, NULL,
                                     ctx->HT_rownnz, ctx->HT_rowadr, ctx->HT_colind,
                                     nv, d);

    // allocate L_colind, L, Lcone
    ctx->L_colind = SIM_STACK_ALLOC(d, ctx->nL, int);
    ctx->L = SIM_STACK_ALLOC(d, ctx->nL, sim_scalar_t);
    if (ctx->is_elliptic) {
      ctx->Lcone = SIM_STACK_ALLOC(d, ctx->nL, sim_scalar_t);
    }

    // allocate LT (CSC representation of L)
    ctx->LT_colind = SIM_STACK_ALLOC(d, ctx->nL, int);
    ctx->LT_map = SIM_STACK_ALLOC(d, ctx->nL, int);

    // symbolic Cholesky: populate L_colind and LT structures
    sim_math_cholFactorSymbolic(ctx->L_colind, ctx->L_rownnz, ctx->L_rowadr,
                           ctx->LT_colind, ctx->LT_rownnz, ctx->LT_rowadr, ctx->LT_map,
                           ctx->HT_rownnz, ctx->HT_rowadr, ctx->HT_colind,
                           nv, d);
  }

  // dense
  else {
    // allocate L, Lcone
    ctx->nL = nv*nv;
    ctx->L = SIM_STACK_ALLOC(d, ctx->nL, sim_scalar_t);
    if (ctx->is_elliptic) {
      ctx->Lcone = SIM_STACK_ALLOC(d, ctx->nL, sim_scalar_t);
    }

    // compute H = M + J'*D*J
    sim_math_sqrMatTD_impl(ctx->L, ctx->J, ctx->D, nefc, nv, /*flg_upper=*/ 0);
    sim_math_addToSymSparse(ctx->L, ctx->M, ctx->nv,
                       ctx->M_rownnz, ctx->M_rowadr, ctx->M_colind,
                       /*flg_upper=*/ 0);
  }
}


// forward declaration of HessianCone (for readability)
static void HessianCone(sim_data_t* d, SIM_PrimalContext* ctx);

// factorize Hessian: L = chol(H), maybe (re)compute H given efc_state
static void FactorizeHessian(sim_data_t* d, SIM_PrimalContext* ctx, int flg_recompute) {
  int nv = ctx->nv, nefc = ctx->nefc;

  // maybe compute constraint inertia
  if (flg_recompute) {
    for (int i=0; i < nefc; i++) {
      ctx->D[i] = ctx->efc_state[i] == SIM_CNSTRSTATE_QUADRATIC ? ctx->efc_D[i] : 0;
    }
  }

  // sparse
  if (ctx->is_sparse) {
    // maybe compute H = M + J'*D*J
    if (flg_recompute) {
      // compute H = J'*D*J
      sim_math_sqrMatTDSparse(ctx->H, ctx->J, ctx->JT, ctx->D, nefc, nv,
                        ctx->H_rownnz, ctx->H_rowadr, ctx->H_colind,
                        ctx->J_rownnz, ctx->J_rowadr, ctx->J_colind, NULL,
                        ctx->JT_rownnz, ctx->JT_rowadr, ctx->JT_colind, ctx->JT_rowsuper,
                        d, /*diagind=*/NULL);

      // add mass matrix: H = J'*D*J + C
      sim_math_addToMatSparse(ctx->H, ctx->H_rownnz, ctx->H_rowadr, ctx->H_colind, nv,
                         ctx->M, ctx->M_rownnz, ctx->M_rowadr, ctx->M_colind,
                         ctx->buf_val, ctx->buf_ind);
    }

    // numeric sparse factorization: L = chol(H) using pre-computed sparsity pattern
    int rank = sim_math_cholFactorNumeric(
        ctx->L, nv, SIM_MINVAL,
        ctx->L_rownnz, ctx->L_rowadr, ctx->L_colind,
        ctx->LT_rownnz, ctx->LT_rowadr, ctx->LT_colind, ctx->LT_map,
        ctx->H, ctx->H_rownnz, ctx->H_rowadr, ctx->H_colind, d);

    // rank-deficient; SHOULD NOT OCCUR
    if (rank != nv) {
      SIM_ERROR("rank-deficient sparse Hessian");
    }
  }

  // dense
  else {
    // maybe compute H = M + J'*D*J
    if (flg_recompute) {
      sim_math_sqrMatTD_impl(ctx->L, ctx->J, ctx->D, nefc, nv, /*flg_upper=*/ 0);
      sim_math_addToSymSparse(ctx->L, ctx->M, ctx->nv,
                         ctx->M_rownnz, ctx->M_rowadr, ctx->M_colind,
                         /*flg_upper=*/ 0);
    }

    // factorize H
    sim_math_cholFactor(ctx->L, nv, SIM_MINVAL);
  }

  // add cones to factor if present
  if (ctx->ncone) {
    HessianCone(d, ctx);
  }

  // mark full update
  ctx->nupdate = nefc;
}


// elliptic case: Hcone = H + cone_contributions
static void HessianCone(sim_data_t* d, SIM_PrimalContext* ctx) {
  int nv = ctx->nv, nefc = ctx->nefc;
  sim_scalar_t local[36];

  // start with Hcone = H
  sim_math_copy(ctx->Lcone, ctx->L, ctx->nL);

  sim_markStack(d);

  // storage for L'*J
  sim_scalar_t* LTJ = SIM_STACK_ALLOC(d, 6*nv, sim_scalar_t);

  // add contributions
  for (int i=0; i < nefc; i++) {
    if (ctx->efc_state[i] == SIM_CNSTRSTATE_CONE) {
      sim_contact_t* con = ctx->contact + ctx->efc_id[i];
      int dim = con->dim;

      // Cholesky of local Hessian
      sim_math_copy(local, con->H, dim*dim);
      sim_math_cholFactor(local, dim, SIM_MINVAL);

      // sparse
      if (ctx->is_sparse) {
        // get nnz for row i (same for all rows in contact)
        const int nnz = ctx->J_rownnz[i];

        // compute LTJ = L'*J for this contact
        sim_math_zero(LTJ, dim*nnz);
        for (int r=0; r < dim; r++) {
          for (int c=0; c <= r; c++) {
            sim_math_addToScl(LTJ+c*nnz, ctx->J+ctx->J_rowadr[i+r], local[r*dim+c], nnz);
          }
        }

        // update
        for (int r=0; r < dim; r++) {
          sim_math_cholUpdateSparse(ctx->Lcone, LTJ+r*nnz, nv, 1,
                               ctx->L_rownnz, ctx->L_rowadr, ctx->L_colind, nnz,
                               ctx->J_colind+ctx->J_rowadr[i+r], d);
        }
      }

      // dense
      else {
        // compute LTJ = L'*J for this contact row
        sim_math_zero(LTJ, dim*nv);
        for (int r=0; r < dim; r++) {
          for (int c=0; c <= r; c++) {
            sim_math_addToScl(LTJ+c*nv, ctx->J+(i+r)*nv, local[r*dim+c], nv);
          }
        }

        // update
        for (int r=0; r < dim; r++) {
          sim_math_cholUpdate(ctx->Lcone, LTJ+r*nv, nv, 1);
        }
      }

      // count updates
      ctx->nupdate += dim;

      // advance to next constraint
      i += (dim-1);
    }
  }

  sim_freeStack(d);
}


// incremental update to Hessian factor due to changes in efc_state
static void HessianIncremental(sim_data_t* d, SIM_PrimalContext* ctx, const int* oldstate) {
  int rank, nv = ctx->nv, nefc = ctx->nefc;
  sim_markStack(d);

  // local space
  sim_scalar_t* vec = SIM_STACK_ALLOC(d, nv, sim_scalar_t);

  // clear update counter
  ctx->nupdate = 0;

  // update H factorization
  for (int i=0; i < nefc; i++) {
    int flag_update = -1;

    // add quad
    if (oldstate[i] != SIM_CNSTRSTATE_QUADRATIC && ctx->efc_state[i] == SIM_CNSTRSTATE_QUADRATIC) {
      flag_update = 1;
    }

    // subtract quad
    else if (oldstate[i] == SIM_CNSTRSTATE_QUADRATIC && ctx->efc_state[i] != SIM_CNSTRSTATE_QUADRATIC) {
      flag_update = 0;
    }

    // perform update if flagged
    if (flag_update != -1) {
      // update with vec = J(i,:)*sqrt(D[i]))
      if (ctx->is_sparse) {
        // get nnz and adr of row i
        const int nnz = ctx->J_rownnz[i], adr = ctx->J_rowadr[i];

        // scale vec
        sim_math_scl(vec, ctx->J+adr, sim_math_sqrt(ctx->efc_D[i]), nnz);

        // sparse update or downdate
        rank = sim_math_cholUpdateSparse(ctx->L, vec, nv, flag_update,
                                    ctx->L_rownnz, ctx->L_rowadr, ctx->L_colind, nnz,
                                    ctx->J_colind+adr, d);
      } else {
        sim_math_scl(vec, ctx->J+i*nv, sim_math_sqrt(ctx->efc_D[i]), nv);
        rank = sim_math_cholUpdate(ctx->L, vec, nv, flag_update);
      }
      ctx->nupdate++;

      // recompute H directly if accuracy lost
      if (rank < nv) {
        sim_freeStack(d);
        FactorizeHessian(d, ctx, /*flg_recompute=*/1);

        // nothing else to do
        return;
      }
    }
  }

  // add cones if present
  if (ctx->ncone) {
    HessianCone(d, ctx);
  }

  sim_freeStack(d);
}


// driver
static void sim_solPrimal(const sim_model_t* m, sim_data_t* d, int island, int maxiter, int flg_Newton) {
  int iter = 0;
  sim_scalar_t alpha, beta;
  sim_scalar_t *gradold = NULL, *Mgradold = NULL, *Mgraddif = NULL;
  SIM_PrimalContext ctx;
  sim_markStack(d);

  // make context
  PrimalPointers(m, d, &ctx, island);
  PrimalAllocate(d, &ctx, flg_Newton);

  // local copies
  int nv   = ctx.nv;
  int nefc = ctx.nefc;

  // allocate local storage
  if (!flg_Newton) {
    gradold     = SIM_STACK_ALLOC(d, nv, sim_scalar_t);
    Mgradold    = SIM_STACK_ALLOC(d, nv, sim_scalar_t);
    Mgraddif    = SIM_STACK_ALLOC(d, nv, sim_scalar_t);
  }
  int* oldstate = SIM_STACK_ALLOC(d, nefc, int);

  // compute Ma = M * qacc
  sim_math_mulSymVecSparse(ctx.Ma, ctx.M, ctx.qacc, nv,
                      ctx.M_rownnz, ctx.M_rowadr, ctx.M_colind);


  // compute Jaref = J * qacc - aref  (dense or sparse)
  if (!ctx.is_sparse) {
    sim_math_mulMatVec(ctx.Jaref, ctx.J, ctx.qacc, nefc, nv);
  } else {
    sim_math_mulMatVecSparse(ctx.Jaref, ctx.J, ctx.qacc, nefc,
                        ctx.J_rownnz, ctx.J_rowadr, ctx.J_colind, ctx.J_rowsuper);
  }
  sim_math_subFrom(ctx.Jaref, ctx.efc_aref, nefc);

  // first update
  PrimalUpdateConstraint(&ctx, flg_Newton & (m->opt.cone == SIM_CONE_ELLIPTIC));
  if (flg_Newton) {
    // compute and factorize Hessian
    MakeHessian(d, &ctx);
    FactorizeHessian(d, &ctx, /*flg_recompute=*/0);
  }
  PrimalUpdateGradient(&ctx, flg_Newton);

  // start both with preconditioned gradient
  sim_math_scl(ctx.search, ctx.Mgrad, -1, nv);

  // compute and save scaling factor
  sim_scalar_t scale;
  if (island < 0) {
    scale = 1 / (m->stat.meaninertia * SIM_MAX(1, m->nv));
  } else {
    sim_scalar_t island_inertia = 0;
    for (int i=0; i < nv; i++) {
      int diag_i = ctx.M_rowadr[i] + ctx.M_rownnz[i] - 1;
      island_inertia += ctx.M[diag_i];
    }
    scale = 1 / island_inertia;
  }
  ctx.scale = scale;

  // main loop
  while (iter < maxiter) {
    // perform linesearch
    alpha = PrimalSearch(&ctx, m->opt.tolerance * m->opt.ls_tolerance, m->opt.ls_iterations);

    // no improvement: done
    if (alpha == 0) {
      break;
    }

    // move to new solution
    sim_math_addToScl(ctx.qacc, ctx.search, alpha, nv);
    sim_math_addToScl(ctx.Ma, ctx.Mv, alpha, nv);
    sim_math_addToScl(ctx.Jaref, ctx.Jv, alpha, nefc);

    // save old
    if (!flg_Newton) {
      sim_math_copy(gradold, ctx.grad, nv);
      sim_math_copy(Mgradold, ctx.Mgrad, nv);
    }
    sim_math_copyInt(oldstate, ctx.efc_state, nefc);
    sim_scalar_t oldcost = ctx.cost;

    // update
    PrimalUpdateConstraint(&ctx, flg_Newton & (m->opt.cone == SIM_CONE_ELLIPTIC));
    if (flg_Newton) {
      HessianIncremental(d, &ctx, oldstate);
    }
    PrimalUpdateGradient(&ctx, flg_Newton);

    // count state changes
    int nchange = 0;
    for (int i=0; i < nefc; i++) {
      nchange += (ctx.efc_state[i] != oldstate[i]);
    }

    // scale improvement, gradient, save stats
    sim_scalar_t improvement = scale * (oldcost - ctx.cost);
    sim_scalar_t gradient = scale * sim_math_norm(ctx.grad, nv);
    saveStats(m, d, island, iter, improvement, gradient, ctx.LSslope,
              ctx.nactive, nchange, ctx.LSiter, ctx.nupdate);

    // increment iteration count
    iter++;

    // termination
    if (improvement < m->opt.tolerance || gradient < m->opt.tolerance) {
      break;
    }

    // update direction
    if (flg_Newton) {
      sim_math_scl(ctx.search, ctx.Mgrad, -1, nv);
    } else {
      // Polak-Ribiere
      sim_math_sub(Mgraddif, ctx.Mgrad, Mgradold, nv);
      beta = sim_math_dot(ctx.grad, Mgraddif, nv) /
             sim_math_max(SIM_MINVAL, sim_math_dot(gradold, Mgradold, nv));

      // reset if negative
      if (beta < 0) {
        beta = 0;
      }

      // update
      for (int i=0; i < nv; i++) {
        ctx.search[i] = -ctx.Mgrad[i] + beta*ctx.search[i];
      }
    }
  }

  // finalize statistics
  if (island < SIM_NISLAND) {
    // if island is -1 (monolithic), clamp to 0
    int island_stat = island < 0 ? 0 : island;

    // update solver iterations
    d->solver_niter[island_stat] += iter;

    // set solver_nnz
    if (flg_Newton) {
      if (sim_isSparse(m)) {
        // two L factors if Lcone is present
        int num_factors = 1 + (ctx.Lcone != NULL);
        d->solver_nnz[island_stat] = num_factors * ctx.nL + ctx.nH;
      } else {
        d->solver_nnz[island_stat] = nv*nv;
      }
    } else {
      d->solver_nnz[island_stat] = 0;
    }
  }

  sim_freeStack(d);
}


// CG entry point
void sim_solCG(const sim_model_t* m, sim_data_t* d, int maxiter) {
  sim_solPrimal(m, d, /*island=*/-1, maxiter, /*flg_Newton=*/0);
}


// CG entry point (one island)
void sim_solCG_island(const sim_model_t* m, sim_data_t* d, int island, int maxiter) {
  sim_solPrimal(m, d, island, maxiter, /*flg_Newton=*/0);
}


// Newton entry point
void sim_solNewton(const sim_model_t* m, sim_data_t* d, int maxiter) {
  sim_solPrimal(m, d, /*island=*/-1, maxiter, /*flg_Newton=*/1);
}


// Newton entry point (one island)
void sim_solNewton_island(const sim_model_t* m, sim_data_t* d, int island, int maxiter) {
  sim_solPrimal(m, d, island, maxiter, /*flg_Newton=*/1);
}
