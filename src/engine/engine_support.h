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

#ifndef SIMCORE_SRC_ENGINE_ENGINE_SUPPORT_H_
#define SIMCORE_SRC_ENGINE_ENGINE_SUPPORT_H_

#include <simcore/SIM_data.h>
#include <simcore/SIM_export.h>
#include <simcore/SIM_model.h>
#include <simcore/SIM_tnum.h>

#ifdef __cplusplus
extern "C" {
#endif

// strings
SIM_API extern const char* SIM_DISABLESTRING[SIM_NDISABLE];
SIM_API extern const char* SIM_ENABLESTRING[SIM_NENABLE];
SIM_API extern const char* SIM_TIMERSTRING[SIM_NTIMER];

// arrays
SIM_API extern const int SIM_CONDATA_SIZE[SIM_NCONDATA];  // TODO(tassa): expose in public header?
extern const int SIM_RAYDATA_SIZE[SIM_NRAYDATA];


//-------------------------- get/set state ---------------------------------------------------------

// return size of state signature
SIM_API int sim_stateSize(const sim_model_t* m, int sig);

// get state
SIM_API void sim_getState(const sim_model_t* m, const sim_data_t* d, sim_scalar_t* state, int sig);

// extract a sub-state from a state
SIM_API void sim_extractState(const sim_model_t* m, const sim_scalar_t* src, int srcsig,
                           sim_scalar_t* dst, int dstsig);

// set state
SIM_API void sim_setState(const sim_model_t* m, sim_data_t* d, const sim_scalar_t* state, int sig);

// copy state from src to dst
SIM_API void sim_copyState(const sim_model_t* m, const sim_data_t* src, sim_data_t* dst, int sig);

// copy current state to the k-th model keyframe
SIM_API void sim_setKeyframe(sim_model_t* m, const sim_data_t* d, int k);

//-------------------------- inertia functions -----------------------------------------------------

// convert sparse inertia matrix M into full matrix
SIM_API void sim_fullM(const sim_model_t* m, sim_scalar_t* dst, const sim_scalar_t* M);

// multiply vector by inertia matrix
SIM_API void sim_mulM(const sim_model_t* m, const sim_data_t* d, sim_scalar_t* res, const sim_scalar_t* vec);

// multiply vector by (inertia matrix)^(1/2)
SIM_API void sim_mulM2(const sim_model_t* m, const sim_data_t* d, sim_scalar_t* res, const sim_scalar_t* vec);

// add inertia matrix to destination matrix (lower triangle only)
//  destination can be sparse or dense when all int* are NULL
SIM_API void sim_addM(const sim_model_t* m, sim_data_t* d, sim_scalar_t* dst,
                   int* rownnz, int* rowadr, int* colind);


//-------------------------- perturbations ---------------------------------------------------------

// apply Cartesian force and torque
SIM_API void sim_applyFT(const sim_model_t* m, sim_data_t* d,
                      const sim_scalar_t force[3], const sim_scalar_t torque[3],
                      const sim_scalar_t point[3], int body, sim_scalar_t* qfrc_target);

// accumulate xfrc_applied in qfrc
void sim_xfrcAccumulate(const sim_model_t* m, sim_data_t* d, sim_scalar_t* qfrc);


//-------------------------- miscellaneous ---------------------------------------------------------

// returns the smallest distance between two geoms
SIM_API sim_scalar_t sim_geomDistance(const sim_model_t* m, const sim_data_t* d, int geom1, int geom2,
                             sim_scalar_t distmax, sim_scalar_t fromto[6]);

// compute velocity by finite-differencing two positions
SIM_API void sim_differentiatePos(const sim_model_t* m, sim_scalar_t* qvel, sim_scalar_t dt,
                               const sim_scalar_t* qpos1, const sim_scalar_t* qpos2);

// integrate qpos with given qvel for given body indices
SIM_API void sim_integratePosInd(const sim_model_t* m, sim_scalar_t* qpos, const sim_scalar_t* qvel, sim_scalar_t dt,
                              const int* index, int nbody);

// integrate position with given velocity
SIM_API void sim_integratePos(const sim_model_t* m, sim_scalar_t* qpos, const sim_scalar_t* qvel, sim_scalar_t dt);

// normalize all quaternions in qpos-type vector
SIM_API void sim_normalizeQuat(const sim_model_t* m, sim_scalar_t* qpos);

// return 1 if actuator i is disabled, 0 otherwise
SIM_API int sim_actuatorDisabled(const sim_model_t* m, int i);

// returns the next activation given current act_dot, after clamping
sim_scalar_t sim_nextActivation(const sim_model_t* m, const sim_data_t* d,
                         int actuator_id, int act_adr, sim_scalar_t act_dot);

// sum all body masses
SIM_API sim_scalar_t sim_getTotalmass(const sim_model_t* m);

// scale body masses and inertias to achieve specified total mass
SIM_API void sim_setTotalmass(sim_model_t* m, sim_scalar_t newmass);

// version number
SIM_API int sim_version(void);

// current version of SimCore as a null-terminated string
SIM_API const char* sim_versionString(void);

// return total size of data fields in a contact sensor bitfield specification
SIM_API int sim_math_condataSize(int dataSpec);

// return total size of data fields in a rangefinder sensor bitfield specification
int sim_math_raydataSize(int dataspec);

// compute camera pixel parameters from model
// outputs: fx, fy (focal length in pixels), cx, cy (principal point), ortho_extent
void sim_math_camIntrinsics(const sim_model_t* m, int camid,
                       sim_scalar_t* fx, sim_scalar_t* fy, sim_scalar_t* cx, sim_scalar_t* cy,
                       sim_scalar_t* ortho_extent);

// read ctrl value for actuator at given time
// returns d->ctrl[id] if no history, otherwise reads from history buffer
// interp: 0=zero-order-hold, 1=linear, 2=cubic spline
SIM_API sim_scalar_t sim_readCtrl(const sim_model_t* m, const sim_data_t* d, int id, sim_scalar_t time, int interp);

// read sensor value from history buffer at given time
// returns pointer to sensordata (no history) or history buffer (exact match),
// or NULL if interpolation performed (writes to result)
// interp: 0=zero-order-hold, 1=linear, 2=cubic spline
SIM_API const sim_scalar_t* sim_readSensor(const sim_model_t* m, const sim_data_t* d, int id, sim_scalar_t time,
                                  sim_scalar_t* result, int interp);

// initialize history buffer for actuator with given values
// if times is NULL, uses existing buffer timestamps
SIM_API void sim_initCtrlHistory(const sim_model_t* m, sim_data_t* d, int id,
                              const sim_scalar_t* times, const sim_scalar_t* values);

// initialize history buffer for sensor with given values
// if times is NULL, uses existing buffer timestamps
// phase sets the user slot (last computation time for interval sensors)
SIM_API void sim_initSensorHistory(const sim_model_t* m, sim_data_t* d, int id,
                                const sim_scalar_t* times, const sim_scalar_t* values, sim_scalar_t phase);

#ifdef __cplusplus
}
#endif

#endif  // SIMCORE_SRC_ENGINE_ENGINE_SUPPORT_H_
