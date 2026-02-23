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

#ifndef SIMCORE_SRC_ENGINE_ENGINE_UTIL_MISC_H_
#define SIMCORE_SRC_ENGINE_ENGINE_UTIL_MISC_H_

#include <simcore/SIM_export.h>
#include <simcore/SIM_model.h>
#include <simcore/SIM_tnum.h>

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>

//------------------------------ tendons and actuators ---------------------------------------------

// wrap tendons around spheres and cylinders
sim_scalar_t sim_math_wrap(sim_scalar_t wpnt[6], const sim_scalar_t x0[3], const sim_scalar_t x1[3], const sim_scalar_t xpos[3],
                const sim_scalar_t xmat[9], sim_scalar_t radius, int type, const sim_scalar_t side[3]);

// normalized muscle length-gain curve
SIM_API sim_scalar_t sim_math_muscleGainLength(sim_scalar_t length, sim_scalar_t lmin, sim_scalar_t lmax);

// muscle active force, prm = (range[2], force, scale, lmin, lmax, vmax, fpmax, fvmax)
SIM_API sim_scalar_t sim_math_muscleGain(sim_scalar_t len, sim_scalar_t vel, const sim_scalar_t lengthrange[2],
                            sim_scalar_t acc0, const sim_scalar_t prm[9]);

// muscle passive force, prm = (range[2], force, scale, lmin, lmax, vmax, fpmax, fvmax)
SIM_API sim_scalar_t sim_math_muscleBias(sim_scalar_t len, const sim_scalar_t lengthrange[2],
                            sim_scalar_t acc0, const sim_scalar_t prm[9]);

// muscle time constant with optional smoothing
SIM_API sim_scalar_t sim_math_muscleDynamicsTimescale(sim_scalar_t dctrl, sim_scalar_t tau_act, sim_scalar_t tau_deact,
                                         sim_scalar_t smoothing_width);

// muscle activation dynamics, prm = (tau_act, tau_deact, smoothing_width)
SIM_API sim_scalar_t sim_math_muscleDynamics(sim_scalar_t ctrl, sim_scalar_t act, const sim_scalar_t prm[3]);

// all 3 semi-axes of a geom
SIM_API void sim_math_geomSemiAxes(sim_scalar_t semiaxes[3], const sim_scalar_t size[3], SIM_tGeom type);

// return 1 if point is inside a primitive geom, 0 otherwise
int sim_math_insideGeom(const sim_scalar_t pos[3], const sim_scalar_t mat[9], const sim_scalar_t size[3], SIM_tGeom type,
                   const sim_scalar_t point[3]);

// compute ray origin and direction for pixel (col, row) in camera image
// directions are normalized so ray functions return actual 3D distance
void sim_math_camPixelRay(sim_scalar_t origin[3], sim_scalar_t direction[3],
                     const sim_scalar_t cam_xpos[3], const sim_scalar_t cam_xmat[9],
                     int col, int row, sim_scalar_t fx, sim_scalar_t fy, sim_scalar_t cx, sim_scalar_t cy,
                     int projection, sim_scalar_t ortho_extent);

// ----------------------------- Flex interpolation ------------------------------------------------

// evaluate the deformation gradient at p using the nodal dof values
SIM_API void sim_math_defGradient(sim_scalar_t res[9], const sim_scalar_t p[3], const sim_scalar_t* dof, int order);

// evaluate the basis function at x for the i-th node
SIM_API sim_scalar_t sim_math_evalBasis(const sim_scalar_t x[3], int i, int order);

// interpolate a function at x with given interpolation coefficients and order n
SIM_API void sim_math_interpolate3D(sim_scalar_t res[3], const sim_scalar_t x[3], const sim_scalar_t* coeff, int order);

// ----------------------------- Base64 ------------------------------------------------------------

// encode data as Base64 into buf (including padding and null char)
// returns number of chars written in buf: 4 * [(ndata + 2) / 3] + 1
SIM_API size_t sim_math_encodeBase64(char* buf, const uint8_t* data, size_t ndata);

// return size in decoded bytes if s is a valid Base64 encoding
// return 0 if s is empty or invalid Base64 encoding
SIM_API size_t sim_math_isValidBase64(const char* s);

// decode valid Base64 in string s into buf, undefined behavior if s is not valid Base64
// returns number of bytes decoded (upper limit of 3 * (strlen(s) / 4))
SIM_API size_t sim_math_decodeBase64(uint8_t* buf, const char* s);

//------------------------------ history buffers ---------------------------------------------------

// buffer layout: [user(1), cursor(1), times(n), values(n*dim)]
// - user: 1 sim_scalar_t reserved for user data (ignored by these functions)
// - cursor: 1 sim_scalar_t for circular buffer index (integer stored as sim_scalar_t)
// - times: n timestamps, contiguous at buf[2..n+1]
// - values: n*dim values, contiguous at buf[n+2..n+2+n*dim-1]
// total buffer size: 2 + n*(1 + dim)

// initialize history buffer with given times and values; times must be strictly increasing
// values is size n x dim
SIM_API void sim_math_historyInit(sim_scalar_t* buf, int n, int dim, const sim_scalar_t* times,
                           const sim_scalar_t* values, sim_scalar_t user);

// find insertion slot for sample at time t, maintaining sorted order
// returns pointer to value slot (size dim) where caller should write
SIM_API sim_scalar_t* sim_math_historyInsert(sim_scalar_t* buf, int n, int dim, sim_scalar_t t);

// read vector value at time t; interp: 0=zero-order-hold, 1=linear, 2=cubic spline
// returns pointer to sample in buffer on exact match (res untouched)
// returns NULL and writes interpolated result to res otherwise
SIM_API const sim_scalar_t* sim_math_historyRead(const sim_scalar_t* buf, int n, int dim,
                                    sim_scalar_t* res, sim_scalar_t t, int interp);

//------------------------------ miscellaneous -----------------------------------------------------

// convert contact force to pyramid representation
SIM_API void sim_math_encodePyramid(sim_scalar_t* pyramid, const sim_scalar_t* force,
                             const sim_scalar_t* mu, int dim);

// convert pyramid representation to contact force
SIM_API void sim_math_decodePyramid(sim_scalar_t* force, const sim_scalar_t* pyramid,
                             const sim_scalar_t* mu, int dim);

// integrate spring-damper analytically, return pos(dt)
SIM_API sim_scalar_t sim_math_springDamper(sim_scalar_t pos0, sim_scalar_t vel0, sim_scalar_t Kp, sim_scalar_t Kv, sim_scalar_t dt);

// return 1 if point is outside box given by pos, mat, size * inflate
// return -1 if point is inside box given by pos, mat, size / inflate
// return 0 if point is between the inflated and deflated boxes
SIM_API int sim_math_outsideBox(const sim_scalar_t point[3], const sim_scalar_t pos[3], const sim_scalar_t mat[9],
                         const sim_scalar_t size[3], sim_scalar_t inflate);

// print matrix
SIM_API void sim_math_printMat(const sim_scalar_t* mat, int nr, int nc);

// print sparse matrix to screen
SIM_API void sim_math_printMatSparse(const sim_scalar_t* mat, int nr,
                              const int* rownnz, const int* rowadr,
                              const int* colind);

// min function, single evaluation of a and b
SIM_API sim_scalar_t sim_math_min(sim_scalar_t a, sim_scalar_t b);

// max function, single evaluation of a and b
SIM_API sim_scalar_t sim_math_max(sim_scalar_t a, sim_scalar_t b);

// clip x to the range [min, max]
SIM_API sim_scalar_t sim_math_clip(sim_scalar_t x, sim_scalar_t min, sim_scalar_t max);

// sign function
SIM_API sim_scalar_t sim_math_sign(sim_scalar_t x);

// round to nearest integer
SIM_API int sim_math_round(sim_scalar_t x);

// convert type id (sim_obj_t) to type name
SIM_API const char* sim_math_type2Str(int type);

// convert type name to type id (sim_obj_t)
SIM_API int sim_math_str2Type(const char* str);

// return human readable number of bytes using standard letter suffix
SIM_API const char* sim_math_writeNumBytes(size_t nbytes);

// warning text
SIM_API const char* sim_math_warningText(int warning, size_t info);

// return 1 if nan or abs(x)>SIM_MAXVAL, 0 otherwise
SIM_API int sim_math_isBad(sim_scalar_t x);

// return 1 if all elements are numerically 0 (-0.0 treated as zero)
SIM_API int sim_math_isZero(const sim_scalar_t* vec, int n);

// return 1 if all elements are 0x00, faster than sim_math_isZero
SIM_API int sim_math_isZeroByte(const unsigned char* vec, int n);

// set integer vector to 0
SIM_API void sim_math_zeroInt(int* res, int n);

// copy int vector vec into res
SIM_API void sim_math_copyInt(int* res, const int* vec, int n);

// fill int vector with val
void sim_math_fillInt(int* res, int val, int n);

// standard normal random number generator (optional second number)
SIM_API sim_scalar_t sim_math_standardNormal(sim_scalar_t* num2);

// convert from float to sim_scalar_t
SIM_API void sim_math_f2n(sim_scalar_t* res, const float* vec, int n);

// convert from sim_scalar_t to float
SIM_API void sim_math_n2f(float* res, const sim_scalar_t* vec, int n);

// convert from double to sim_scalar_t
SIM_API void sim_math_d2n(sim_scalar_t* res, const double* vec, int n);

// convert from sim_scalar_t to double
SIM_API void sim_math_n2d(double* res, const sim_scalar_t* vec, int n);

// gather SIM_tNums
SIM_API void sim_math_gather(sim_scalar_t* res, const sim_scalar_t* vec, const int* ind, int n);

// gather SIM_tNums, set to 0 at negative indices
SIM_API void sim_math_gatherMasked(sim_scalar_t* res, const sim_scalar_t* vec, const int* ind, int n);

// scatter SIM_tNums
SIM_API void sim_math_scatter(sim_scalar_t* res, const sim_scalar_t* vec, const int* ind, int n);

// gather integers
SIM_API void sim_math_gatherInt(int* res, const int* vec, const int* ind, int n);

// scatter integers
SIM_API void sim_math_scatterInt(int* res, const int* vec, const int* ind, int n);

// build gather indices mapping src to res, assumes pattern(res) \subseteq pattern(src)
SIM_API void sim_math_sparseMap(int* map, int nr,
                         const int* res_rowadr, const int* res_rownnz, const int* res_colind,
                         const int* src_rowadr, const int* src_rownnz, const int* src_colind);

// build masked-gather map to copy a lower-triangular src into symmetric res
//  `cursor` is a preallocated buffer of size `nr`
SIM_API void sim_math_lower2SymMap(int* map, int nr,
                            const int* res_rowadr, const int* res_rownnz, const int* res_colind,
                            const int* src_rowadr, const int* src_rownnz, const int* src_colind,
                            int* cursor);

// insertion sort, increasing order
SIM_API void sim_math_insertionSort(sim_scalar_t* list, int n);

// integer insertion sort, increasing order
SIM_API void sim_math_insertionSortInt(int* list, int n);

// Halton sequence
SIM_API sim_scalar_t sim_math_Halton(int index, int base);

// call strncpy, then set dst[n-1] = 0
SIM_API char* sim_math_strncpy(char *dst, const char *src, int n);

// sigmoid function over 0<=x<=1 using quintic polynomial
SIM_API sim_scalar_t sim_math_sigmoid(sim_scalar_t x);

#ifdef __cplusplus
}
#endif
#endif  // SIMCORE_SRC_ENGINE_ENGINE_UTIL_MISC_H_
