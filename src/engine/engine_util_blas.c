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

#include "engine/engine_util_blas.h"

#include <string.h>

#include <simcore/SIM_tnum.h>

#ifdef SIM_USEPLATFORMSIMD
  #if defined(__AVX__) && !defined(SIM_USESINGLE)
    #define SIM_USEAVX
    #include "immintrin.h"
  #endif
#endif



//------------------------------ 3D vector and matrix-vector operations ----------------------------

// res = 0
void sim_math_zero_3(sim_scalar_t res[3]) {
  res[0] = 0;
  res[1] = 0;
  res[2] = 0;
}


// vec1 == vec2
int sim_math_equal3(const sim_scalar_t vec1[3], const sim_scalar_t vec2[3]) {
  return sim_math_abs(vec1[0] - vec2[0]) < SIM_MINVAL &&
         sim_math_abs(vec1[1] - vec2[1]) < SIM_MINVAL &&
         sim_math_abs(vec1[2] - vec2[2]) < SIM_MINVAL;
}


// res = vec
void sim_math_copy_3(sim_scalar_t res[3], const sim_scalar_t vec[3]) {
  res[0] = vec[0];
  res[1] = vec[1];
  res[2] = vec[2];
}

// res = mat
void sim_math_copy9(sim_scalar_t res[9], const sim_scalar_t mat[9]) {
  res[0] = mat[0];  res[1] = mat[1];  res[2] = mat[2];
  res[3] = mat[3];  res[4] = mat[4];  res[5] = mat[5];
  res[6] = mat[6];  res[7] = mat[7];  res[8] = mat[8];
}


// res = vec*scl
void sim_math_scale_3(sim_scalar_t res[3], const sim_scalar_t vec[3], sim_scalar_t scl) {
  res[0] = vec[0] * scl;
  res[1] = vec[1] * scl;
  res[2] = vec[2] * scl;
}


// res = vec1 + vec2
void sim_math_add3(sim_scalar_t res[3], const sim_scalar_t vec1[3], const sim_scalar_t vec2[3]) {
  res[0] = vec1[0] + vec2[0];
  res[1] = vec1[1] + vec2[1];
  res[2] = vec1[2] + vec2[2];
}


// res = vec1 - vec2
void sim_math_sub_3(sim_scalar_t res[3], const sim_scalar_t vec1[3], const sim_scalar_t vec2[3]) {
  res[0] = vec1[0] - vec2[0];
  res[1] = vec1[1] - vec2[1];
  res[2] = vec1[2] - vec2[2];
}


// res += vec
void sim_math_add_to_3(sim_scalar_t res[3], const sim_scalar_t vec[3]) {
  res[0] += vec[0];
  res[1] += vec[1];
  res[2] += vec[2];
}


// res -= vec
void sim_math_subFrom3(sim_scalar_t res[3], const sim_scalar_t vec[3]) {
  res[0] -= vec[0];
  res[1] -= vec[1];
  res[2] -= vec[2];
}


// res += vec*scl
void sim_math_add_to_scale_3(sim_scalar_t res[3], const sim_scalar_t vec[3], sim_scalar_t scl) {
  res[0] += vec[0] * scl;
  res[1] += vec[1] * scl;
  res[2] += vec[2] * scl;
}


// res = vec1 + vec2*scl
void sim_math_addScl3(sim_scalar_t res[3], const sim_scalar_t vec1[3], const sim_scalar_t vec2[3], sim_scalar_t scl) {
  res[0] = vec1[0] + scl*vec2[0];
  res[1] = vec1[1] + scl*vec2[1];
  res[2] = vec1[2] + scl*vec2[2];
}


// normalize vector, return length before normalization
sim_scalar_t sim_math_normalize_3(sim_scalar_t vec[3]) {
  sim_scalar_t norm = sim_math_sqrt(vec[0]*vec[0] + vec[1]*vec[1] + vec[2]*vec[2]);

  if (norm < SIM_MINVAL) {
    vec[0] = 1;
    vec[1] = 0;
    vec[2] = 0;
  } else {
    sim_scalar_t normInv = 1/norm;
    vec[0] *= normInv;
    vec[1] *= normInv;
    vec[2] *= normInv;
  }

  return norm;
}


// compute vector length (without normalizing)
sim_scalar_t sim_math_norm3(const sim_scalar_t vec[3]) {
  return sim_math_sqrt(vec[0]*vec[0] + vec[1]*vec[1] + vec[2]*vec[2]);
}


// vector dot-product
sim_scalar_t sim_math_dot_3(const sim_scalar_t vec1[3], const sim_scalar_t vec2[3]) {
  return vec1[0]*vec2[0] + vec1[1]*vec2[1] + vec1[2]*vec2[2];
}


// Cartesian distance between 3D vectors
sim_scalar_t sim_math_dist3(const sim_scalar_t pos1[3], const sim_scalar_t pos2[3]) {
  sim_scalar_t dif[3] = {pos1[0]-pos2[0], pos1[1]-pos2[1], pos1[2]-pos2[2]};
  return sim_math_sqrt(dif[0]*dif[0] + dif[1]*dif[1] + dif[2]*dif[2]);
}


// multiply 3-by-3 matrix by vector
void sim_math_mul_mat_vec_3(sim_scalar_t res[3], const sim_scalar_t mat[9], const sim_scalar_t vec[3]) {
  sim_scalar_t tmp[3] = {
    mat[0]*vec[0] + mat[1]*vec[1] + mat[2]*vec[2],
    mat[3]*vec[0] + mat[4]*vec[1] + mat[5]*vec[2],
    mat[6]*vec[0] + mat[7]*vec[1] + mat[8]*vec[2]
  };
  res[0] = tmp[0];
  res[1] = tmp[1];
  res[2] = tmp[2];
}


// multiply transposed 3-by-3 matrix by vector
void sim_math_mulMatTVec3(sim_scalar_t res[3], const sim_scalar_t mat[9], const sim_scalar_t vec[3]) {
  sim_scalar_t tmp[3] = {
    mat[0]*vec[0] + mat[3]*vec[1] + mat[6]*vec[2],
    mat[1]*vec[0] + mat[4]*vec[1] + mat[7]*vec[2],
    mat[2]*vec[0] + mat[5]*vec[1] + mat[8]*vec[2]
  };
  res[0] = tmp[0];
  res[1] = tmp[1];
  res[2] = tmp[2];
}


// multiply 3x3 matrices,
void sim_math_mulMatMat3(sim_scalar_t res[9], const sim_scalar_t mat1[9], const sim_scalar_t mat2[9]) {
  res[0] = mat1[0]*mat2[0] + mat1[1]*mat2[3] + mat1[2]*mat2[6];
  res[1] = mat1[0]*mat2[1] + mat1[1]*mat2[4] + mat1[2]*mat2[7];
  res[2] = mat1[0]*mat2[2] + mat1[1]*mat2[5] + mat1[2]*mat2[8];
  res[3] = mat1[3]*mat2[0] + mat1[4]*mat2[3] + mat1[5]*mat2[6];
  res[4] = mat1[3]*mat2[1] + mat1[4]*mat2[4] + mat1[5]*mat2[7];
  res[5] = mat1[3]*mat2[2] + mat1[4]*mat2[5] + mat1[5]*mat2[8];
  res[6] = mat1[6]*mat2[0] + mat1[7]*mat2[3] + mat1[8]*mat2[6];
  res[7] = mat1[6]*mat2[1] + mat1[7]*mat2[4] + mat1[8]*mat2[7];
  res[8] = mat1[6]*mat2[2] + mat1[7]*mat2[5] + mat1[8]*mat2[8];
}


// multiply 3x3 matrices, first argument transposed
void sim_math_mulMatTMat3(sim_scalar_t res[9], const sim_scalar_t mat1[9], const sim_scalar_t mat2[9]) {
  res[0] = mat1[0]*mat2[0] + mat1[3]*mat2[3] + mat1[6]*mat2[6];
  res[1] = mat1[0]*mat2[1] + mat1[3]*mat2[4] + mat1[6]*mat2[7];
  res[2] = mat1[0]*mat2[2] + mat1[3]*mat2[5] + mat1[6]*mat2[8];
  res[3] = mat1[1]*mat2[0] + mat1[4]*mat2[3] + mat1[7]*mat2[6];
  res[4] = mat1[1]*mat2[1] + mat1[4]*mat2[4] + mat1[7]*mat2[7];
  res[5] = mat1[1]*mat2[2] + mat1[4]*mat2[5] + mat1[7]*mat2[8];
  res[6] = mat1[2]*mat2[0] + mat1[5]*mat2[3] + mat1[8]*mat2[6];
  res[7] = mat1[2]*mat2[1] + mat1[5]*mat2[4] + mat1[8]*mat2[7];
  res[8] = mat1[2]*mat2[2] + mat1[5]*mat2[5] + mat1[8]*mat2[8];
}


// multiply 3x3 matrices, second argument transposed
void sim_math_mulMatMatT3(sim_scalar_t res[9], const sim_scalar_t mat1[9], const sim_scalar_t mat2[9]) {
  res[0] = mat1[0]*mat2[0] + mat1[1]*mat2[1] + mat1[2]*mat2[2];
  res[1] = mat1[0]*mat2[3] + mat1[1]*mat2[4] + mat1[2]*mat2[5];
  res[2] = mat1[0]*mat2[6] + mat1[1]*mat2[7] + mat1[2]*mat2[8];
  res[3] = mat1[3]*mat2[0] + mat1[4]*mat2[1] + mat1[5]*mat2[2];
  res[4] = mat1[3]*mat2[3] + mat1[4]*mat2[4] + mat1[5]*mat2[5];
  res[5] = mat1[3]*mat2[6] + mat1[4]*mat2[7] + mat1[5]*mat2[8];
  res[6] = mat1[6]*mat2[0] + mat1[7]*mat2[1] + mat1[8]*mat2[2];
  res[7] = mat1[6]*mat2[3] + mat1[7]*mat2[4] + mat1[8]*mat2[5];
  res[8] = mat1[6]*mat2[6] + mat1[7]*mat2[7] + mat1[8]*mat2[8];
}


//------------------------------ 4D vector and matrix-vector operations ----------------------------

// res = 0
void sim_math_zero4(sim_scalar_t res[4]) {
  res[0] = 0;
  res[1] = 0;
  res[2] = 0;
  res[3] = 0;
}


// res = (1,0,0,0)
void sim_math_unit4(sim_scalar_t res[4]) {
  res[0] = 1;
  res[1] = 0;
  res[2] = 0;
  res[3] = 0;
}


// res = vec
void sim_math_copy4(sim_scalar_t res[4], const sim_scalar_t data[4]) {
  res[0] = data[0];
  res[1] = data[1];
  res[2] = data[2];
  res[3] = data[3];
}


// normalize vector, return length before normalization
sim_scalar_t sim_math_normalize4(sim_scalar_t vec[4]) {
  sim_scalar_t norm = sim_math_sqrt(vec[0]*vec[0] + vec[1]*vec[1] + vec[2]*vec[2] + vec[3]*vec[3]);

  if (norm < SIM_MINVAL) {
    vec[0] = 1;
    vec[1] = 0;
    vec[2] = 0;
    vec[3] = 0;
  } else if (sim_math_abs(norm - 1) > SIM_MINVAL) {
    sim_scalar_t normInv = 1/norm;
    vec[0] *= normInv;
    vec[1] *= normInv;
    vec[2] *= normInv;
    vec[3] *= normInv;
  }

  return norm;
}


//------------------------------ vector operations -------------------------------------------------

// res = 0
void sim_math_zero(sim_scalar_t* res, int n) {
  memset(res, 0, n*sizeof(sim_scalar_t));
}


// res = 0, at given indices
void sim_math_zeroInd(sim_scalar_t* res, int n, const int* ind) {
  for (int i = 0; i < n; i++) {
    res[ind[i]] = 0;
  }
}


// res = val
void sim_math_fill(sim_scalar_t* res, sim_scalar_t val, int n) {
  for (int i=0; i < n; i++) {
    res[i] = val;
  }
}


// res = vec
void sim_math_copy(sim_scalar_t* res, const sim_scalar_t* vec, int n) {
  memcpy(res, vec, n*sizeof(sim_scalar_t));
}


// res = vec, at given indices
void sim_math_copyInd(sim_scalar_t* res, const sim_scalar_t* vec, const int* ind, int n) {
  for (int i = 0; i < n; i++) {
    res[ind[i]] = vec[ind[i]];
  }
}


// sum(vec)
sim_scalar_t sim_math_sum(const sim_scalar_t* vec, int n) {
  sim_scalar_t res = 0;

  for (int i=0; i < n; i++) {
    res += vec[i];
  }

  return res;
}


// sum(abs(vec))
sim_scalar_t sim_math_L1(const sim_scalar_t* vec, int n) {
  sim_scalar_t res = 0;

  for (int i=0; i < n; i++) {
    res += sim_math_abs(vec[i]);
  }

  return res;
}


// res = vec*scl
void sim_math_scl(sim_scalar_t* res, const sim_scalar_t* vec, sim_scalar_t scl, int n) {
  int i = 0;

#ifdef SIM_USEAVX
  int n_4 = n - 4;

  // vector part
  if (n_4 >= 0) {
    __m256d sclpar, val1, val1scl;

    // init
    sclpar = _mm256_set1_pd(scl);

    // parallel computation
    while (i <= n_4) {
      val1 = _mm256_loadu_pd(vec+i);
      val1scl = _mm256_mul_pd(val1, sclpar);
      _mm256_storeu_pd(res+i, val1scl);
      i += 4;
    }
  }

  // process remaining
  int n_i = n - i;
  if (n_i == 3) {
    res[i] = vec[i]*scl;
    res[i+1] = vec[i+1]*scl;
    res[i+2] = vec[i+2]*scl;
  } else if (n_i == 2) {
    res[i] = vec[i]*scl;
    res[i+1] = vec[i+1]*scl;
  } else if (n_i == 1) {
    res[i] = vec[i]*scl;
  }

#else
  for (; i < n; i++) {
    res[i] = vec[i]*scl;
  }
#endif
}


// res = vec1 + vec2
void sim_math_add(sim_scalar_t* res, const sim_scalar_t* vec1, const sim_scalar_t* vec2, int n) {
  int i = 0;

#ifdef SIM_USEAVX
  int n_4 = n - 4;

  // vector part
  if (n_4 >= 0) {
    __m256d sum, val1, val2;

    // parallel computation
    while (i <= n_4) {
      val1 = _mm256_loadu_pd(vec1+i);
      val2 = _mm256_loadu_pd(vec2+i);
      sum = _mm256_add_pd(val1, val2);
      _mm256_storeu_pd(res+i, sum);
      i += 4;
    }
  }

  // process remaining
  int n_i = n - i;
  if (n_i == 3) {
    res[i] = vec1[i] + vec2[i];
    res[i+1] = vec1[i+1] + vec2[i+1];
    res[i+2] = vec1[i+2] + vec2[i+2];
  } else if (n_i == 2) {
    res[i] = vec1[i] + vec2[i];
    res[i+1] = vec1[i+1] + vec2[i+1];
  } else if (n_i == 1) {
    res[i] = vec1[i] + vec2[i];
  }

#else
  for (; i < n; i++) {
    res[i] = vec1[i] + vec2[i];
  }
#endif
}


// res = vec1 + vec2, at selected indices
void sim_math_addInd(sim_scalar_t* res, const sim_scalar_t* vec1, const sim_scalar_t* vec2, const int* ind, int n) {
  for (int i = 0; i < n; i++) {
    int j = ind[i];
    res[j] = vec1[j] + vec2[j];
  }
}


// res = vec1 - vec2
void sim_math_sub(sim_scalar_t* res, const sim_scalar_t* vec1, const sim_scalar_t* vec2, int n) {
  int i = 0;

#ifdef SIM_USEAVX
  int n_4 = n - 4;

  // vector part
  if (n_4 >= 0) {
    __m256d dif, val1, val2;

    // parallel computation
    while (i <= n_4) {
      val1 = _mm256_loadu_pd(vec1+i);
      val2 = _mm256_loadu_pd(vec2+i);
      dif = _mm256_sub_pd(val1, val2);
      _mm256_storeu_pd(res+i, dif);
      i += 4;
    }
  }

  // process remaining
  int n_i = n - i;
  if (n_i == 3) {
    res[i] = vec1[i] - vec2[i];
    res[i+1] = vec1[i+1] - vec2[i+1];
    res[i+2] = vec1[i+2] - vec2[i+2];
  } else if (n_i == 2) {
    res[i] = vec1[i] - vec2[i];
    res[i+1] = vec1[i+1] - vec2[i+1];
  } else if (n_i == 1) {
    res[i] = vec1[i] - vec2[i];
  }

#else
  for (; i < n; i++) {
    res[i] = vec1[i] - vec2[i];
  }
#endif
}


// res = vec1 - vec2, at selected indices
void sim_math_subInd(sim_scalar_t* res, const sim_scalar_t* vec1, const sim_scalar_t* vec2, const int* ind, int n) {
  for (int i = 0; i < n; i++) {
    int j = ind[i];
    res[j] = vec1[j] - vec2[j];
  }
}


// res += vec
void sim_math_addTo(sim_scalar_t* res, const sim_scalar_t* vec, int n) {
  int i = 0;

#ifdef SIM_USEAVX
  int n_4 = n - 4;

  // vector part
  if (n_4 >= 0) {
    __m256d sum, val1, val2;

    // parallel computation
    while (i <= n_4) {
      val1 = _mm256_loadu_pd(res+i);
      val2 = _mm256_loadu_pd(vec+i);
      sum = _mm256_add_pd(val1, val2);
      _mm256_storeu_pd(res+i, sum);
      i += 4;
    }
  }

  // process remaining
  int n_i = n - i;
  if (n_i == 3) {
    res[i] += vec[i];
    res[i+1] += vec[i+1];
    res[i+2] += vec[i+2];
  } else if (n_i == 2) {
    res[i] += vec[i];
    res[i+1] += vec[i+1];
  } else if (n_i == 1) {
    res[i] += vec[i];
  }

#else
  for (; i < n; i++) {
    res[i] += vec[i];
  }
#endif
}


// res += vec, at selected indices
void sim_math_addToInd(sim_scalar_t* res, const sim_scalar_t* vec, const int* ind, int n) {
  for (int i = 0; i < n; i++) {
    int j = ind[i];
    res[j] += vec[j];
  }
}


// res -= vec
void sim_math_subFrom(sim_scalar_t* res, const sim_scalar_t* vec, int n) {
  int i = 0;

#ifdef SIM_USEAVX
  int n_4 = n - 4;

  // vector part
  if (n_4 >= 0) {
    __m256d dif, val1, val2;

    // parallel computation
    while (i <= n_4) {
      val1 = _mm256_loadu_pd(res+i);
      val2 = _mm256_loadu_pd(vec+i);
      dif = _mm256_sub_pd(val1, val2);
      _mm256_storeu_pd(res+i, dif);
      i += 4;
    }
  }

  // process remaining
  int n_i = n - i;
  if (n_i == 3) {
    res[i] -= vec[i];
    res[i+1] -= vec[i+1];
    res[i+2] -= vec[i+2];
  } else if (n_i == 2) {
    res[i] -= vec[i];
    res[i+1] -= vec[i+1];
  } else if (n_i == 1) {
    res[i] -= vec[i];
  }

#else
  for (; i < n; i++) {
    res[i] -= vec[i];
  }
#endif
}


// res += vec*scl
void sim_math_addToScl(sim_scalar_t* res, const sim_scalar_t* vec, sim_scalar_t scl, int n) {
  int i = 0;

#ifdef SIM_USEAVX
  int n_4 = n - 4;

  // vector part
  if (n_4 >= 0) {
    __m256d sclpar, sum, val1, val2, val2scl;

    // init
    sclpar = _mm256_set1_pd(scl);

    // parallel computation
    while (i <= n_4) {
      val1 = _mm256_loadu_pd(res+i);
      val2 = _mm256_loadu_pd(vec+i);
      val2scl = _mm256_mul_pd(val2, sclpar);
      sum = _mm256_add_pd(val1, val2scl);
      _mm256_storeu_pd(res+i, sum);
      i += 4;
    }
  }

  // process remaining
  int n_i = n - i;
  if (n_i == 3) {
    res[i] += vec[i]*scl;
    res[i+1] += vec[i+1]*scl;
    res[i+2] += vec[i+2]*scl;
  } else if (n_i == 2) {
    res[i] += vec[i]*scl;
    res[i+1] += vec[i+1]*scl;
  } else if (n_i == 1) {
    res[i] += vec[i]*scl;
  }

#else
  for (; i < n; i++) {
    res[i] += vec[i]*scl;
  }
#endif
}



// res += vec*scl, at given indices
void sim_math_addToSclInd(sim_scalar_t* res, const sim_scalar_t* vec, const int* ind, sim_scalar_t scl, int n) {
  for (int i=0; i < n; i++) {
    int k = ind[i];
    res[k] += vec[k]*scl;
  }
}



// res = vec1 + vec2*scl
void sim_math_addScl(sim_scalar_t* res, const sim_scalar_t* vec1, const sim_scalar_t* vec2, sim_scalar_t scl, int n) {
  int i = 0;

#if defined(__AVX__) && defined(SIM_USEAVX)  && !defined(SIM_USESINGLE)
  int n_4 = n - 4;

  // vector part
  if (n_4 >= 0) {
    __m256d sclpar, sum, val1, val2, val2scl;

    // init
    sclpar = _mm256_set1_pd(scl);

    // parallel computation
    while (i <= n_4) {
      val1 = _mm256_loadu_pd(vec1+i);
      val2 = _mm256_loadu_pd(vec2+i);
      val2scl = _mm256_mul_pd(val2, sclpar);
      sum = _mm256_add_pd(val1, val2scl);
      _mm256_storeu_pd(res+i, sum);
      i += 4;
    }
  }

  // process remaining
  int n_i = n - i;
  if (n_i == 3) {
    res[i] = vec1[i] + vec2[i]*scl;
    res[i+1] = vec1[i+1] + vec2[i+1]*scl;
    res[i+2] = vec1[i+2] + vec2[i+2]*scl;
  } else if (n_i == 2) {
    res[i] = vec1[i] + vec2[i]*scl;
    res[i+1] = vec1[i+1] + vec2[i+1]*scl;
  } else if (n_i == 1) {
    res[i] = vec1[i] + vec2[i]*scl;
  }

#else
  for (; i < n; i++) {
    res[i] = vec1[i] + vec2[i]*scl;
  }
#endif
}


// normalize vector, return length before normalization
sim_scalar_t sim_math_normalize(sim_scalar_t* res, int n) {
  sim_scalar_t norm = sim_math_sqrt(sim_math_dot(res, res, n));

  if (norm < SIM_MINVAL) {
    res[0] = 1;
    sim_math_zero(res + 1, n - 1);
  } else {
    sim_scalar_t normInv = 1 / norm;
    for (int i=0; i < n; i++) {
      res[i] *= normInv;
    }
  }

  return norm;
}


// compute vector length (without normalizing)
sim_scalar_t sim_math_norm(const sim_scalar_t* res, int n) {
  return sim_math_sqrt(sim_math_dot(res, res, n));
}


// vector dot-product
sim_scalar_t sim_math_dot(const sim_scalar_t* vec1, const sim_scalar_t* vec2, int n) {
  sim_scalar_t res = 0;
  int i = 0;
  int n_4 = n - 4;
#ifdef SIM_USEAVX

  // vector part
  if (n_4 >= 0) {
    __m256d sum, prod, val1, val2;
    __m128d vlow, vhigh, high64;

    // init
    val1 = _mm256_loadu_pd(vec1);
    val2 = _mm256_loadu_pd(vec2);
    sum = _mm256_mul_pd(val1, val2);
    i = 4;

    // parallel computation
    while (i <= n_4) {
      val1 = _mm256_loadu_pd(vec1+i);
      val2 = _mm256_loadu_pd(vec2+i);
      prod = _mm256_mul_pd(val1, val2);
      sum = _mm256_add_pd(sum, prod);
      i += 4;
    }

    // reduce
    vlow = _mm256_castpd256_pd128(sum);
    vhigh = _mm256_extractf128_pd(sum, 1);
    vlow = _mm_add_pd(vlow, vhigh);
    high64 = _mm_unpackhi_pd(vlow, vlow);
    res = _mm_cvtsd_f64(_mm_add_sd(vlow, high64));
  }

#else
  // do the same order of additions as the AVX intrinsics implementation.
  // this is faster than the simple for loop you'd expect for a dot product,
  // and produces exactly the same results.
  sim_scalar_t res0 = 0;
  sim_scalar_t res1 = 0;
  sim_scalar_t res2 = 0;
  sim_scalar_t res3 = 0;

  for (; i <= n_4; i+=4) {
    res0 += vec1[i] * vec2[i];
    res1 += vec1[i+1] * vec2[i+1];
    res2 += vec1[i+2] * vec2[i+2];
    res3 += vec1[i+3] * vec2[i+3];
  }
  res = (res0 + res2) + (res1 + res3);
#endif

  // process remaining
  int n_i = n - i;
  if (n_i == 3) {
    res += vec1[i]*vec2[i] + vec1[i+1]*vec2[i+1] + vec1[i+2]*vec2[i+2];
  } else if (n_i == 2) {
    res += vec1[i]*vec2[i] + vec1[i+1]*vec2[i+1];
  } else if (n_i == 1) {
    res += vec1[i]*vec2[i];
  }
  return res;
}



// vector dot-product, at given indices
sim_scalar_t sim_math_dotInd(const sim_scalar_t* vec1, const sim_scalar_t* vec2, const int* ind, int n) {
  sim_scalar_t res = 0;
  for (int i = 0; i < n; i++) {
    int k = ind[i];
    res += vec1[k] * vec2[k];
  }
  return res;
}



//------------------------------ matrix-vector operations ------------------------------------------

// multiply matrix and vector
void sim_math_mulMatVec(sim_scalar_t* res, const sim_scalar_t* mat, const sim_scalar_t* vec, int nr, int nc) {
  for (int r=0; r < nr; r++) {
    res[r] = sim_math_dot(mat + r*nc, vec, nc);
  }
}


// multiply transposed matrix and vector
void sim_math_mulMatTVec(sim_scalar_t* res, const sim_scalar_t* mat, const sim_scalar_t* vec, int nr, int nc) {
  sim_scalar_t tmp;
  sim_math_zero(res, nc);

  for (int r=0; r < nr; r++) {
    if ((tmp = vec[r])) {
      sim_math_addToScl(res, mat+r*nc, tmp, nc);
    }
  }
}


// multiply square matrix with vectors on both sides: return vec1'*mat*vec2
sim_scalar_t sim_math_mulVecMatVec(const sim_scalar_t* vec1, const sim_scalar_t* mat, const sim_scalar_t* vec2, int n) {
  sim_scalar_t res = 0;
  for (int i=0; i < n; i++) {
    res += vec1[i] * sim_math_dot(mat + i*n, vec2, n);
  }
  return res;
}


//------------------------------ matrix operations -------------------------------------------------

// transpose matrix
void sim_math_transpose(sim_scalar_t* res, const sim_scalar_t* mat, int nr, int nc) {
  for (int i=0; i < nr; i++) {
    for (int j=0; j < nc; j++) {
      res[j*nr+i] = mat[i*nc+j];
    }
  }
}


// symmetrize square matrix res = (mat + mat')/2
void sim_math_symmetrize(sim_scalar_t* res, const sim_scalar_t* mat, int n) {
  for (int i=0; i < n; i++) {
    res[i*(n+1)] = mat[i*(n+1)];
    for (int j=0; j < i; j++) {
      res[i*n+j] = res[j*n+i] = 0.5 * (mat[i*n+j] + mat[j*n+i]);
    }
  }
}


// identity matrix
void sim_math_eye(sim_scalar_t* mat, int n) {
  sim_math_zero(mat, n*n);
  for (int i=0; i < n; i++) {
    mat[i*(n + 1)] = 1;
  }
}


// res[ind, :] = mat[ind, :]
void sim_math_copyRows(sim_scalar_t* res, const sim_scalar_t* mat, const int* ind, int n, int nc) {
  for (int i = 0; i < n; i++) {
    sim_math_copy(res + nc*ind[i], mat + nc*ind[i], nc);
  }
}


//------------------------------ matrix-matrix operations ------------------------------------------

// multiply matrices, exploit sparsity of mat1
void sim_math_mulMatMat(sim_scalar_t* res, const sim_scalar_t* mat1, const sim_scalar_t* mat2,
                   int r1, int c1, int c2) {
  sim_scalar_t tmp;

  sim_math_zero(res, r1*c2);

  for (int i=0; i < r1; i++) {
    for (int k=0; k < c1; k++) {
      if ((tmp = mat1[i*c1+k])) {
        sim_math_addToScl(res+i*c2, mat2+k*c2, tmp, c2);
      }
    }
  }
}


// multiply matrices, second argument transposed
void sim_math_mulMatMatT(sim_scalar_t* res, const sim_scalar_t* mat1, const sim_scalar_t* mat2,
                    int r1, int c1, int r2) {
  for (int i=0; i < r1; i++) {
    for (int j=0; j < r2; j++) {
      res[i*r2+j] = sim_math_dot(mat1+i*c1, mat2+j*c1, c1);
    }
  }
}


// compute M'*diag*M (diag=NULL: compute M'*M), upper triangle optional
void sim_math_sqrMatTD_impl(sim_scalar_t* res, const sim_scalar_t* mat, const sim_scalar_t* diag,
                       int nr, int nc, int flg_upper) {
  sim_scalar_t tmp;

  // half of MatMat routine: only lower triangle
  sim_math_zero(res, nc*nc);
  if (diag) {
    for (int j=0; j < nr; j++) {
      if (diag[j]) {
        for (int i=0; i < nc; i++) {
          if ((tmp = mat[j*nc+i])) {
            sim_math_addToScl(res+i*nc, mat+j*nc, tmp*diag[j], i+1);
          }
        }
      }
    }
  } else {
    for (int i=0; i < nc; i++) {
      for (int j=0; j < nr; j++) {
        if ((tmp = mat[j*nc+i])) {
          sim_math_addToScl(res+i*nc, mat+j*nc, tmp, i+1);
        }
      }
    }
  }

  // flg_upper is set: make symmetric
  if (flg_upper) {
    for (int i=0; i < nc; i++) {
      for (int j=i+1; j < nc; j++) {
        res[i*nc+j] = res[j*nc+i];
      }
    }
  }
}


// compute M'*diag*M (diag=NULL: compute M'*M)
void sim_math_sqrMatTD(sim_scalar_t* res, const sim_scalar_t* mat, const sim_scalar_t* diag, int nr, int nc) {
  sim_math_sqrMatTD_impl(res, mat, diag, nr, nc, /*flg_upper=*/ 1);
}


// multiply matrices, first argument transposed
void sim_math_mulMatTMat(sim_scalar_t* res, const sim_scalar_t* mat1, const sim_scalar_t* mat2,
                    int r1, int c1, int c2) {
  sim_scalar_t tmp;

  sim_math_zero(res, c1*c2);

  for (int i=0; i < r1; i++) {
    for (int j=0; j < c1; j++) {
      if ((tmp = mat1[i*c1+j])) {
        sim_math_addToScl(res+j*c2, mat2+i*c2, tmp, c2);
      }
    }
  }
}
