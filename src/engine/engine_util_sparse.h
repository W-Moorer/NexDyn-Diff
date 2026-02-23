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

#ifndef SIMCORE_SRC_ENGINE_ENGINE_UTIL_SPARSE_H_
#define SIMCORE_SRC_ENGINE_ENGINE_UTIL_SPARSE_H_

#include <string.h>

#include <simcore/SIM_data.h>
#include <simcore/SIM_export.h>
#include <simcore/SIM_tnum.h>
#include "engine/engine_util_sparse_avx.h"

#ifdef __cplusplus
extern "C" {
#endif

//------------------------------ sparse operations -------------------------------------------------

// dot-product, both vectors are sparse
SIM_API sim_scalar_t sim_math_dotSparse2(const sim_scalar_t* vec1, const int* ind1, int nnz1,
                            const sim_scalar_t* vec2, const int* ind2, int nnz2);

// convert matrix from dense to sparse
//  nnz is size of res and colind, return 1 if too small, 0 otherwise
SIM_API int sim_math_dense2sparse(sim_scalar_t* res, const sim_scalar_t* mat, int nr, int nc,
                           int* rownnz, int* rowadr, int* colind, int nnz);

// convert matrix from sparse to dense
SIM_API void sim_math_sparse2dense(sim_scalar_t* res, const sim_scalar_t* mat, int nr, int nc, const int* rownnz,
                            const int* rowadr, const int* colind);

// res[row, :] = mat[row, :]
void sim_math_copySparse(sim_scalar_t* res, const sim_scalar_t* mat, const int* rownnz, const int* rowadr,
                    const int* row, int nrow);

// res[row, :] = 0
void sim_math_zeroSparse(sim_scalar_t* res, const int* rownnz, const int* rowadr, const int* row, int nrow);

// multiply sparse matrix and dense vector:  res = mat * vec
SIM_API void sim_math_mulMatVecSparse(sim_scalar_t* res, const sim_scalar_t* mat, const sim_scalar_t* vec,
                               int nr, const int* rownnz, const int* rowadr,
                               const int* colind, const int* rowsuper);

// multiply transposed sparse matrix and dense vector:  res = mat' * vec
SIM_API void sim_math_mulMatTVecSparse(sim_scalar_t* res, const sim_scalar_t* mat, const sim_scalar_t* vec, int nr, int nc,
                                const int* rownnz, const int* rowadr, const int* colind);

// add sparse matrix M to sparse destination matrix, requires pre-allocated buffers
SIM_API void sim_math_addToMatSparse(sim_scalar_t* dst, int* rownnz, int* rowadr, int* colind, int nr,
                              const sim_scalar_t* M, const int* M_rownnz, const int* M_rowadr,
                              const int* M_colind,
                              sim_scalar_t* buf_val, int* buf_ind);

// add symmetric matrix (only lower triangle represented) to dense matrix
SIM_API void sim_math_addToSymSparse(sim_scalar_t* res, const sim_scalar_t* mat, int n,
                              const int* rownnz, const int* rowadr, const int* colind,
                              int flg_upper);

// multiply symmetric matrix (only lower triangle represented) by vector:
//  res = (mat + strict_upper(mat')) * vec
SIM_API void sim_math_mulSymVecSparse(sim_scalar_t* res, const sim_scalar_t* mat, const sim_scalar_t* vec, int n,
                               const int* rownnz, const int* rowadr, const int* colind);

// compress sparse matrix, remove elements with abs(value) <= minval, return total non-zeros
SIM_API int sim_math_compressSparse(sim_scalar_t* mat, int nr, int nc,
                             int* rownnz, int* rowadr, int* colind, sim_scalar_t minval);

// count the number of non-zeros in the sum of two sparse vectors
SIM_API int sim_math_combineSparseCount(int a_nnz, int b_nnz, const int* a_ind, const int* b_ind);

// incomplete combine sparse: dst = a*dst + b*src at common indices
void sim_math_combineSparseInc(sim_scalar_t* dst, const sim_scalar_t* src, int n, sim_scalar_t a, sim_scalar_t b,
                          int dst_nnz, int src_nnz, const int* dst_ind, const int* src_ind);

// dst += scl * src, only at common non-zero indices
void sim_math_addToSclSparseInc(sim_scalar_t* dst, const sim_scalar_t* src,
                           int nnzdst, const int* inddst,
                           int nnzsrc, const int* indsrc, sim_scalar_t scl);

// add to sparse matrix: dst = dst + scl*src, return nnz of result
int sim_math_addToSparseMat(sim_scalar_t* dst, const sim_scalar_t* src, int n, int nrow, sim_scalar_t scl,
                       int dst_nnz, int src_nnz, int* dst_ind, const int* src_ind,
                       sim_scalar_t* buf, int* buf_ind);

// add(merge) two chains
int sim_math_addChains(int* res, int n, int NV1, int NV2,
                  const int* chain1, const int* chain2);

// transpose sparse matrix, optionally compute row supernodes
SIM_API void sim_math_transposeSparse(sim_scalar_t* res, const sim_scalar_t* mat, int nr, int nc,
                               int* res_rownnz, int* res_rowadr, int* res_colind, int* res_rowsuper,
                               const int* rownnz, const int* rowadr, const int* colind);

// construct row supernodes
SIM_API void sim_math_superSparse(int nr, int* rowsuper,
                           const int* rownnz, const int* rowadr, const int* colind);

// compute sparse M'*diag*M (diag=NULL: compute M'*M), res_rowadr must be precomputed
SIM_API void sim_math_sqrMatTDSparse(sim_scalar_t* res, const sim_scalar_t* mat, const sim_scalar_t* matT,
                              const sim_scalar_t* diag, int nr, int nc,
                              int* res_rownnz, const int* res_rowadr, int* res_colind,
                              const int* rownnz, const int* rowadr,
                              const int* colind, const int* rowsuper,
                              const int* rownnzT, const int* rowadrT,
                              const int* colindT, const int* rowsuperT,
                              sim_data_t* d, int* diagind);

// LEGACY: row-based implementation
SIM_API void sim_math_sqrMatTDSparse_row(sim_scalar_t* res, const sim_scalar_t* mat, const sim_scalar_t* matT,
                                  const sim_scalar_t* diag, int nr, int nc,
                                  int* res_rownnz, const int* res_rowadr, int* res_colind,
                                  const int* rownnz, const int* rowadr,
                                  const int* colind, const int* rowsuper,
                                  const int* rownnzT, const int* rowadrT,
                                  const int* colindT, const int* rowsuperT,
                                  sim_data_t* d, int* diagind);

// precount res_rownnz and precompute res_rowadr for sim_math_sqrMatTDSparse, return total non-zeros
SIM_API int sim_math_sqrMatTDSparseCount(int* res_rownnz, int* res_rowadr, int nr,
                                  const int* rownnz, const int* rowadr, const int* colind,
                                  const int* rownnzT, const int* rowadrT, const int* colindT,
                                  const int* rowsuperT, sim_data_t* d, int flg_upper);

// precompute res_rowadr for sim_math_sqrMatTDSparse using uncompressed memory
SIM_API void sim_math_sqrMatTDUncompressedInit(int* res_rowadr, int nc);

// block-diagonalize a dense matrix
SIM_API void sim_math_blockDiag(sim_scalar_t* res, const sim_scalar_t* mat,
                         int nc_mat, int nc_res, int nb,
                         const int* perm_r, const int* perm_c,
                         const int* block_nr, const int* block_nc,
                         const int* blockadr_r, const int* blockadr_c);

// block-diagonalize a sparse matrix
SIM_API void sim_math_blockDiagSparse(
  sim_scalar_t* res, int* res_rownnz, int* res_rowadr, int* res_colind,
  const sim_scalar_t* mat, const int* rownnz, const int* rowadr, const int* colind,
  int nr, int nb,
  const int* perm_r, const int* perm_c,
  const int* block_r, const int* block_c,
  sim_scalar_t* res2, const sim_scalar_t* mat2);

// ------------------------------ inlined functions ------------------------------------------------

// dot-product, first vector is sparse
static inline
sim_scalar_t sim_math_dotSparse(const sim_scalar_t* vec1, const sim_scalar_t* vec2, int nnz1, const int* ind1) {
#ifdef SIM_USEAVX
  return sim_math_dotSparse_avx(vec1, vec2, nnz1, ind1);
#else
  int i = 0;
  sim_scalar_t res = 0;
  int n_4 = nnz1 - 4;
  sim_scalar_t res0 = 0;
  sim_scalar_t res1 = 0;
  sim_scalar_t res2 = 0;
  sim_scalar_t res3 = 0;

  for (; i <= n_4; i+=4) {
    res0 += vec1[i+0] * vec2[ind1[i+0]];
    res1 += vec1[i+1] * vec2[ind1[i+1]];
    res2 += vec1[i+2] * vec2[ind1[i+2]];
    res3 += vec1[i+3] * vec2[ind1[i+3]];
  }

  res = (res0 + res2) + (res1 + res3);

  // scalar part
  for (; i < nnz1; i++) {
    res += vec1[i] * vec2[ind1[i]];
  }

  return res;
#endif  // SIM_USEAVX
}



// return 1 if vec1==vec2, 0 otherwise
static inline
int sim_math_compare(const int* vec1, const int* vec2, int n) {
#ifdef SIM_USEAVX
  return sim_math_compare_avx(vec1, vec2, n);
#else
  return !memcmp(vec1, vec2, n*sizeof(int));
#endif  // SIM_USEAVX
}



// merge unique sorted integers, merge array must be large enough (not checked for)
static inline
int sim_mergeSorted(int* merge, const int* chain1, int n1, const int* chain2, int n2) {
  // special case: one or both empty
  if (n1 == 0) {
    if (n2 == 0) {
      return 0;
    }
    memcpy(merge, chain2, n2 * sizeof(int));
    return n2;
  } else if (n2 == 0) {
    memcpy(merge, chain1, n1 * sizeof(int));
    return n1;
  }

  // special case: identical pattern
  if (n1 == n2 && sim_math_compare(chain1, chain2, n1)) {
    memcpy(merge, chain1, n1 * sizeof(int));
    return n1;
  }

  // merge while both chains are non-empty
  int i = 0, j = 0, k = 0;
  while (i < n1 && j < n2) {
    int c1 = chain1[i];
    int c2 = chain2[j];

    if (c1 < c2) {
      merge[k++] = c1;
      i++;
    } else if (c1 > c2) {
      merge[k++] = c2;
      j++;
    } else { // c1 == c2
      merge[k++] = c1;
      i++;
      j++;
    }
  }

  // copy remaining
  if (i < n1) {
    memcpy(merge + k, chain1 + i, (n1 - i)*sizeof(int));
    k += n1 - i;
  } else if (j < n2) {
    memcpy(merge + k, chain2 + j, (n2 - j)*sizeof(int));
    k += n2 - j;
  }

  return k;
}



// res = res*scl1 + vec*scl2
static inline
void sim_math_addToSclScl(sim_scalar_t* res, const sim_scalar_t* vec, sim_scalar_t scl1, sim_scalar_t scl2, int n) {
#ifdef SIM_USEAVX
  sim_math_addToSclScl_avx(res, vec, scl1, scl2, n);
#else
  for (int i=0; i < n; i++) {
    res[i] = res[i]*scl1 + vec[i]*scl2;
  }
#endif  // SIM_USEAVX
}



// combine two sparse vectors: dst = a*dst + b*src, return nnz of result
static inline
int sim_math_combineSparse(sim_scalar_t* dst, const sim_scalar_t* src, sim_scalar_t a, sim_scalar_t b,
                      int dst_nnz, int src_nnz, int* dst_ind, const int* src_ind,
                      sim_scalar_t* buf, int* buf_ind) {
  // check for identical pattern
  if (dst_nnz == src_nnz) {
    if (sim_math_compare(dst_ind, src_ind, dst_nnz)) {
      // combine sim_scalar_t data directly
      sim_math_addToSclScl(dst, src, a, b, dst_nnz);
      return dst_nnz;
    }
  }

  // copy dst into buf
  if (dst_nnz) {
    memcpy(buf, dst, dst_nnz * sizeof(sim_scalar_t));
    memcpy(buf_ind, dst_ind, dst_nnz * sizeof(int));
  }

  // prepare to merge buf and src into dst
  int bi = 0, si = 0, nnz = 0;
  int buf_nnz = dst_nnz;

  // merge vectors
  while (bi < buf_nnz && si < src_nnz) {
    int badr = buf_ind[bi];
    int sadr = src_ind[si];

    if (badr == sadr) {
      dst[nnz] = a*buf[bi++] + b*src[si++];
      dst_ind[nnz++] = badr;
    }

    // buf only
    else if (badr < sadr) {
      dst[nnz] = a*buf[bi++];
      dst_ind[nnz++] = badr;
    }

    // src only
    else {
      dst[nnz] = b*src[si++];
      dst_ind[nnz++] = sadr;
    }
  }

  // the rest of src only
  while (si < src_nnz) {
    dst[nnz] = b*src[si];
    dst_ind[nnz++] = src_ind[si++];
  }

  // the rest of buf only
  while (bi < buf_nnz) {
    dst[nnz] = a*buf[bi];
    dst_ind[nnz++] = buf_ind[bi++];
  }

  return nnz;
}

#ifdef __cplusplus
}
#endif

#endif  // SIMCORE_SRC_ENGINE_ENGINE_UTIL_SPARSE_H_
