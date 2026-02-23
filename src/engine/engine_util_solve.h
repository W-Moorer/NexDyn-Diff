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

#ifndef SIMCORE_SRC_ENGINE_ENGINE_UTIL_SOLVE_H_
#define SIMCORE_SRC_ENGINE_ENGINE_UTIL_SOLVE_H_

#include <simcore/SIM_data.h>
#include <simcore/SIM_export.h>

#ifdef __cplusplus
extern "C" {
#endif

// Cholesky decomposition: mat = L*L'; return rank
SIM_API int sim_math_cholFactor(sim_scalar_t* mat, int n, sim_scalar_t mindiag);

// Cholesky solve
SIM_API void sim_math_cholSolve(sim_scalar_t* res, const sim_scalar_t* mat, const sim_scalar_t* vec, int n);

// Cholesky rank-one update: L*L' +/- x*x'; return rank
SIM_API int sim_math_cholUpdate(sim_scalar_t* mat, sim_scalar_t* x, int n, int flg_plus);

// sparse reverse-order Cholesky decomposition: mat = L'*L; return 'rank'
//  mat must be lower-triangular, have preallocated space for fill-in
SIM_API int sim_math_cholFactorSparse(sim_scalar_t* mat, int n, sim_scalar_t mindiag,
                               int* rownnz, const int* rowadr, int* colind, sim_data_t* d);

// symbolic reverse-Cholesky: compute both L (CSR) and LT (CSC) structures
//   if L_colind is NULL, perform counting logic (fill rownnz/rowadr arrays and return total nnz)
//   if L_colind is not NULL, assume rownnz/rowadr are precomputed and fill colind/map arrays
//   reads pattern from upper triangle
//   based on ldl_symbolic from 'Algorithm 8xx: a concise sparse Cholesky factorization package'
SIM_API int sim_math_cholFactorSymbolic(int* L_colind, int* L_rownnz, int* L_rowadr,
                                 int* LT_colind, int* LT_rownnz, int* LT_rowadr, int* LT_map,
                                 const int* rownnz, const int* rowadr, const int* colind,
                                 int n, sim_data_t* d);

// numeric reverse-Cholesky: compute L values given fixed sparsity pattern, returns rank
//  L_colind must already contain the correct sparsity pattern (from sim_math_cholFactorSymbolic)
//  LT_map[k] gives index in L for LT_colind[k]
SIM_API int sim_math_cholFactorNumeric(sim_scalar_t* L, int n, sim_scalar_t mindiag,
                                const int* L_rownnz, const int* L_rowadr, const int* L_colind,
                                const int* LT_rownnz, const int* LT_rowadr, const int* LT_colind,
                                const int* LT_map, const sim_scalar_t* H,
                                const int* H_rownnz, const int* H_rowadr, const int* H_colind,
                                sim_data_t* d);

// sparse reverse-order Cholesky solve
void sim_math_cholSolveSparse(sim_scalar_t* res, const sim_scalar_t* mat, const sim_scalar_t* vec, int n,
                         const int* rownnz, const int* rowadr, const int* colind);

// sparse reverse-order Cholesky rank-one update: L'*L +/i x*x'; return rank
//  x is sparse, change in sparsity pattern of mat is not allowed
SIM_API int sim_math_cholUpdateSparse(sim_scalar_t* mat, const sim_scalar_t* x, int n, int flg_plus,
                               const int* rownnz, const int* rowadr, const int* colind,
                               int x_nnz, const int* x_ind, sim_data_t* d);

// band-dense Cholesky decomposition
//  returns minimum value in the factorized diagonal, or 0 if rank-deficient
//  mat has (ntotal-ndense) x nband + ndense x ntotal elements
//  the first (ntotal-ndense) x nband store the band part, left of diagonal, inclusive
//  the second ndense x ntotal store the band part as entire dense rows
//  add diagadd+diagmul*mat_ii to diagonal before factorization
SIM_API sim_scalar_t sim_math_cholFactorBand(sim_scalar_t* mat, int ntotal, int nband, int ndense,
                                sim_scalar_t diagadd, sim_scalar_t diagmul);

// solve (mat*mat')*res = vec with band-Cholesky decomposition
SIM_API void sim_math_cholSolveBand(sim_scalar_t* res, const sim_scalar_t* mat, const sim_scalar_t* vec,
                             int ntotal, int nband, int ndense);

// convert banded matrix to dense matrix, fill upper triangle if flg_sym>0
SIM_API void sim_math_band2Dense(sim_scalar_t* res, const sim_scalar_t* mat, int ntotal, int nband, int ndense,
                          sim_byte_t flg_sym);

// convert dense matrix to banded matrix
SIM_API void sim_math_dense2Band(sim_scalar_t* res, const sim_scalar_t* mat, int ntotal, int nband, int ndense);

// multiply band-diagonal matrix with vector, include upper triangle if flg_sym>0
SIM_API void sim_math_bandMulMatVec(sim_scalar_t* res, const sim_scalar_t* mat, const sim_scalar_t* vec,
                             int ntotal, int nband, int ndense, int nvec, sim_byte_t flg_sym);

// address of diagonal element i in band-dense matrix representation
SIM_API int sim_math_bandDiag(int i, int ntotal, int nband, int ndense);

// sparse reverse-order LU factorization, assume tree topology (only dofs in index, if given)
//  LU = L + U; original = (U+I) * L; scratch is size n
void sim_math_factorLUSparse(sim_scalar_t *LU, int n, int* scratch,
                        const int *rownnz, const int *rowadr, const int *colind, const int *index);

// solve mat*res=vec given LU factorization of mat (only dofs in index, if given)
void sim_math_solveLUSparse(sim_scalar_t *res, const sim_scalar_t *LU, const sim_scalar_t* vec, int n,
                       const int *rownnz, const int *rowadr, const int* diag, const int *colind,
                       const int *index);

// eigenvalue decomposition of symmetric 3x3 matrix
SIM_API int sim_math_eig3(sim_scalar_t eigval[3], sim_scalar_t eigvec[9], sim_scalar_t quat[4], const sim_scalar_t mat[9]);

// solve QCQP in 2 dimensions:
//  min  0.5*x'*A*x + x'*b  s.t.  sum (xi/di)^2 <= r^2
// return 0 if unconstrained, 1 if constrained
SIM_API int sim_math_QCQP2(sim_scalar_t* res, const sim_scalar_t* Ain, const sim_scalar_t* bin, const sim_scalar_t* d, sim_scalar_t r);

// solve QCQP in 3 dimensions:
//  min  0.5*x'*A*x + x'*b  s.t.  sum (xi/di)^2 <= r^2
// return 0 if unconstrained, 1 if constrained
SIM_API int sim_math_QCQP3(sim_scalar_t* res, const sim_scalar_t* Ain, const sim_scalar_t* bin, const sim_scalar_t* d, sim_scalar_t r);

// solve QCQP in n<=5 dimensions:
//  min  0.5*x'*A*x + x'*b  s.t.  sum (xi/di)^2 <= r^2
// return 0 if unconstrained, 1 if constrained
int sim_math_QCQP(sim_scalar_t* res, const sim_scalar_t* Ain, const sim_scalar_t* bin, const sim_scalar_t* d, sim_scalar_t r, int n);

// solve box-constrained Quadratic Program
//  min 0.5*x'*H*x + x'*g  s.t. lower <= x <=upper
// return rank of unconstrained subspace or -1 on failure
SIM_API int sim_math_boxQP(sim_scalar_t* res, sim_scalar_t* R, int* index,
                    const sim_scalar_t* H, const sim_scalar_t* g, int n,
                    const sim_scalar_t* lower, const sim_scalar_t* upper);

// allocate memory for box-constrained Quadratic Program
SIM_API void sim_math_boxQPmalloc(sim_scalar_t** res, sim_scalar_t** R, int** index,
                           sim_scalar_t** H, sim_scalar_t** g, int n,
                           sim_scalar_t** lower, sim_scalar_t** upper);

// minimize 0.5*x'*H*x + x'*g  s.t. lower <= x <=upper, explicit options (see implementation)
SIM_API int sim_math_boxQPoption(sim_scalar_t* res, sim_scalar_t* R, int* index,
                          const sim_scalar_t* H, const sim_scalar_t* g, int n,
                          const sim_scalar_t* lower, const sim_scalar_t* upper,
                          int maxiter, sim_scalar_t mingrad, sim_scalar_t backtrack,
                          sim_scalar_t minstep, sim_scalar_t armijo,
                          char* log, int logsz);

#ifdef __cplusplus
}
#endif

#endif  // SIMCORE_SRC_ENGINE_ENGINE_UTIL_SOLVE_H_
