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

#ifndef SIMCORE_INCLUDE_SIMCORE_CORE_API_H_
#define SIMCORE_INCLUDE_SIMCORE_CORE_API_H_

// header version; should match the library version as returned by sim_version()
#define SIM_VERSION_HEADER 3005001

// needed to define size_t, fabs and log10
#include <stdlib.h>
#include <math.h>

// type definitions
#include <simcore/SIM_data.h>
#include <simcore/SIM_export.h>
#include <simcore/SIM_model.h>
#include <simcore/SIM_macro.h>
#include <simcore/SIM_plugin.h>
#include <simcore/SIM_san.h>
#include <simcore/SIM_spec.h>
#include <simcore/SIM_thread.h>
#include <simcore/SIM_tnum.h>

// this is a C-API
#ifdef __cplusplus
extern "C" {
#endif

// user error and memory handlers
SIM_API extern void  (*sim_math_user_error)(const char*);
SIM_API extern void  (*sim_math_user_warning)(const char*);
SIM_API extern void* (*sim_math_user_malloc)(size_t);
SIM_API extern void  (*sim_math_user_free)(void*);


// callbacks extending computation pipeline
SIM_API extern SIM_fGeneric  SIM_cb_passive;
SIM_API extern SIM_fGeneric  SIM_cb_control;
SIM_API extern SIM_fConFilt  SIM_cb_contactfilter;
SIM_API extern SIM_fSensor   SIM_cb_sensor;
SIM_API extern SIM_fTime     SIM_cb_time;
SIM_API extern SIM_fAct      SIM_cb_act_dyn;
SIM_API extern SIM_fAct      SIM_cb_act_gain;
SIM_API extern SIM_fAct      SIM_cb_act_bias;


// collision function table
SIM_API extern SIM_fCollision SIM_COLLISIONFUNC[SIM_NGEOMTYPES][SIM_NGEOMTYPES];


// string names
SIM_API extern const char* SIM_DISABLESTRING[SIM_NDISABLE];
SIM_API extern const char* SIM_ENABLESTRING[SIM_NENABLE];
SIM_API extern const char* SIM_TIMERSTRING[SIM_NTIMER];


//---------------------------------- Virtual file system -------------------------------------------

// Initialize an empty VFS, sim_deleteVFS must be called to deallocate the VFS.
SIM_API void sim_defaultVFS(SIM_VFS* vfs);

// Mount a ResourceProvider to handle file operations under the given path; return 0: success,
// 2: repeated name, -1: invalid resource provider.
SIM_API int sim_mountVFS(SIM_VFS* vfs, const char* filepath, const SIM_pResourceProvider* provider);

// Unmount a previously mounted ResourceProvider; return 0: success, -1: not found in VFS.
SIM_API int sim_unmountVFS(SIM_VFS* vfs, const char* filename);

// Add file to VFS; return 0: success, 2: repeated name, -1: failed to load.
SIM_API int sim_addFileVFS(SIM_VFS* vfs, const char* directory, const char* filename);

// Add file to VFS from buffer; return 0: success, 2: repeated name, -1: failed to load.
SIM_API int sim_addBufferVFS(SIM_VFS* vfs, const char* name, const void* buffer, int nbuffer);

// Delete file from VFS; return 0: success, -1: not found in VFS.
SIM_API int sim_deleteFileVFS(SIM_VFS* vfs, const char* filename);

// Delete all files from VFS and deallocates VFS internal memory.
SIM_API void sim_deleteVFS(SIM_VFS* vfs);

//------------------------------------ Asset cache -------------------------------------------------

// Get the current size of the asset cache in bytes.
SIM_API size_t sim_getCacheSize(const SIM_Cache* cache);

// Get the capacity of the asset cache in bytes.
SIM_API size_t sim_getCacheCapacity(const SIM_Cache* cache);

// Set the capacity of the asset cache in bytes (0 to disable); return the new capacity.
SIM_API size_t sim_setCacheCapacity(SIM_Cache* cache, size_t size);

// Get the internal asset cache used by the compiler.
SIM_API SIM_Cache* sim_getCache(void);

// Clear the asset cache.
SIM_API void sim_clearCache(SIM_Cache* cache);

//---------------------------------- Parse and compile ---------------------------------------------

// Parse XML file in SIMCF or URDF format, compile it; return low-level model.
// If vfs is not NULL, look up files in vfs before reading from disk.
// If error is not NULL, it must have size error_sz.
// Nullable: vfs, error
SIM_API sim_model_t* sim_loadXML(const char* filename, const SIM_VFS* vfs, char* error, int error_sz);

// Parse spec from XML file.
// Nullable: vfs, error
SIM_API sim_spec_t* sim_parseXML(const char* filename, const SIM_VFS* vfs, char* error, int error_sz);

// Parse spec from XML string.
// Nullable: vfs, error
SIM_API sim_spec_t* sim_parseXMLString(const char* xml, const SIM_VFS* vfs, char* error, int error_sz);

// Parse spec from a file.
// Nullable: vfs, error
SIM_API sim_spec_t* sim_parse(const char* filename, const char* content_type,
                       const SIM_VFS* vfs, char* error, int error_sz);

// Compile spec to model.
// Nullable: vfs
SIM_API sim_model_t* sim_compile(sim_spec_t* s, const SIM_VFS* vfs);

// Copy real-valued arrays from model to spec; return 1 on success.
SIM_API int sim_copyBack(sim_spec_t* s, const sim_model_t* m);

// Recompile spec to model, preserving the state; return 0 on success.
// Nullable: vfs
SIM_API int sim_recompile(sim_spec_t* s, const SIM_VFS* vfs, sim_model_t* m, sim_data_t* d);

// Update XML data structures with info from low-level model created with sim_loadXML, save as SIMCF.
// If error is not NULL, it must have size error_sz.
// Nullable: error
SIM_API int sim_saveLastXML(const char* filename, const sim_model_t* m, char* error, int error_sz);

// Free last XML model if loaded. Called internally at each load.
SIM_API void sim_freeLastXML(void);

// Save spec to XML string; return 0 on success, -1 on failure.
// If length of the output buffer is too small; return the required size.
// Nullable: error
SIM_API int sim_saveXMLString(const sim_spec_t* s, char* xml, int xml_sz, char* error, int error_sz);

// Save spec to XML file; return 0 on success, -1 otherwise.
// Nullable: error
SIM_API int sim_saveXML(const sim_spec_t* s, const char* filename, char* error, int error_sz);

// Given SIMCF filename, fills dependencies with a list of all other asset files it depends on.
// The search is recursive, and the list includes the filename itself.
SIM_API void sim_math_getXMLDependencies(const char* filename, SIM_StringVec* dependencies);

//---------------------------------- Main simulation -----------------------------------------------

// Advance simulation, use control callback to obtain external force and control.
SIM_API void sim_step(const sim_model_t* m, sim_data_t* d);

// Advance simulation in two steps: before external force and control is set by user.
SIM_API void sim_step1(const sim_model_t* m, sim_data_t* d);

// Advance simulation in two steps: after external force and control is set by user.
SIM_API void sim_step2(const sim_model_t* m, sim_data_t* d);

// Forward dynamics: same as sim_step but do not integrate in time.
SIM_API void sim_forward(const sim_model_t* m, sim_data_t* d);

// Inverse dynamics: qacc must be set before calling.
SIM_API void sim_inverse(const sim_model_t* m, sim_data_t* d);

// Forward dynamics with skip; skipstage is SIM_tStage.
SIM_API void sim_forwardSkip(const sim_model_t* m, sim_data_t* d, int skipstage, int skipsensor);

// Inverse dynamics with skip; skipstage is SIM_tStage.
SIM_API void sim_inverseSkip(const sim_model_t* m, sim_data_t* d, int skipstage, int skipsensor);


//---------------------------------- Initialization ------------------------------------------------

// Set default options for length range computation.
SIM_API void sim_defaultLROpt(SIM_LROpt* opt);

// Set solver parameters to default values.
// Nullable: solref, solimp
SIM_API void sim_defaultSolRefImp(sim_scalar_t* solref, sim_scalar_t* solimp);

// Set physics options to default values.
SIM_API void sim_defaultOption(SIM_Option* opt);

// Copy sim_model_t, allocate new if dest is NULL.
// Nullable: dest
SIM_API sim_model_t* sim_copyModel(sim_model_t* dest, const sim_model_t* src);

// Save model to binary SIMB file or memory buffer; buffer has precedence when given.
// Nullable: filename, buffer
SIM_API void sim_saveModel(const sim_model_t* m, const char* filename, void* buffer, int buffer_sz);

// Load model from binary SIMB file.
// If vfs is not NULL, look up file in vfs before reading from disk.
// Nullable: vfs
SIM_API sim_model_t* sim_loadModel(const char* filename, const SIM_VFS* vfs);

// Load model from memory buffer.
SIM_API sim_model_t* sim_loadModelBuffer(const void* buffer, int buffer_sz);

// Free memory allocation in model.
SIM_API void sim_deleteModel(sim_model_t* m);

// Return size of buffer needed to hold model.
SIM_API sim_size_t sim_sizeModel(const sim_model_t* m);

// Allocate sim_data_t corresponding to given model.
// If the model buffer is unallocated the initial configuration will not be set.
SIM_API sim_data_t* sim_makeData(const sim_model_t* m);

// Copy sim_data_t.
// m is only required to contain the size fields from SIMMODEL_INTS.
SIM_API sim_data_t* sim_copyData(sim_data_t* dest, const sim_model_t* m, const sim_data_t* src);

// Reset data to defaults.
SIM_API void sim_resetData(const sim_model_t* m, sim_data_t* d);

// Reset data to defaults, fill everything else with debug_value.
SIM_API void sim_resetDataDebug(const sim_model_t* m, sim_data_t* d, unsigned char debug_value);

// Reset data. If 0 <= key < nkey, set fields from specified keyframe.
SIM_API void sim_resetDataKeyframe(const sim_model_t* m, sim_data_t* d, int key);

#ifndef ADDRESS_SANITIZER  // Stack management functions declared in SIM_san.h if ASAN is active.

// Mark a new frame on the sim_data_t stack.
SIM_API void sim_markStack(sim_data_t* d);

// Free the current sim_data_t stack frame. All pointers returned by sim_stackAlloc since the last call
// to sim_markStack must no longer be used afterwards.
SIM_API void sim_freeStack(sim_data_t* d);

#endif  // ADDRESS_SANITIZER

// Allocate a number of bytes on sim_data_t stack at a specific alignment.
// Call sim_error on stack overflow.
SIM_API void* sim_stackAllocByte(sim_data_t* d, size_t bytes, size_t alignment);

// Allocate array of SIM_tNums on sim_data_t stack. Call sim_error on stack overflow.
SIM_API sim_scalar_t* sim_stackAllocNum(sim_data_t* d, size_t size);

// Allocate array of ints on sim_data_t stack. Call sim_error on stack overflow.
SIM_API int* sim_stackAllocInt(sim_data_t* d, size_t size);

// Free memory allocation in sim_data_t.
SIM_API void sim_deleteData(sim_data_t* d);

// Reset all callbacks to NULL pointers (NULL is the default).
SIM_API void sim_resetCallbacks(void);

// Set constant fields of sim_model_t, corresponding to qpos0 configuration.
SIM_API void sim_setConst(sim_model_t* m, sim_data_t* d);

// Set actuator_lengthrange for specified actuator; return 1 if ok, 0 if error.
// Nullable: error
SIM_API int sim_setLengthRange(sim_model_t* m, sim_data_t* d, int index,
                            const SIM_LROpt* opt, char* error, int error_sz);

// Create empty spec.
SIM_API sim_spec_t* sim_makeSpec(void);

// Copy spec.
SIM_API sim_spec_t* sim_copySpec(const sim_spec_t* s);

// Free memory allocation in sim_spec_t.
SIM_API void sim_deleteSpec(sim_spec_t* s);

// Activate plugin; return 0 on success.
SIM_API int sim_spec_activatePlugin(sim_spec_t* s, const char* name);

// Turn deep copy on or off attach; return 0 on success.
SIM_API int sim_spec_setDeepCopy(sim_spec_t* s, int deepcopy);


//---------------------------------- Printing ------------------------------------------------------

// Print sim_model_t to text file, specifying format.
// float_format must be a valid printf-style format string for a single float value.
SIM_API void sim_printFormattedModel(const sim_model_t* m, const char* filename, const char* float_format);

// Print model to text file.
SIM_API void sim_printModel(const sim_model_t* m, const char* filename);

// Print sim_data_t to text file, specifying format.
// float_format must be a valid printf-style format string for a single float value.
SIM_API void sim_printFormattedData(const sim_model_t* m, const sim_data_t* d, const char* filename,
                                 const char* float_format);

// Print data to text file.
SIM_API void sim_printData(const sim_model_t* m, const sim_data_t* d, const char* filename);

// Print matrix to screen.
SIM_API void sim_math_printMat(const sim_scalar_t* mat, int nr, int nc);

// Print sparse matrix to screen.
SIM_API void sim_math_printMatSparse(const sim_scalar_t* mat, int nr,
                              const int* rownnz, const int* rowadr, const int* colind);

// Print internal XML schema as plain text or HTML, with style-padding or &nbsp;.
SIM_API int sim_printSchema(const char* filename, char* buffer, int buffer_sz,
                         int flg_html, int flg_pad);

//---------------------------------- Components ----------------------------------------------------

// Run all kinematics-like computations (kinematics, comPos, camlight, flex, tendon).
SIM_API void sim_fwdKinematics(const sim_model_t* m, sim_data_t* d);

// Run position-dependent computations.
SIM_API void sim_fwdPosition(const sim_model_t* m, sim_data_t* d);

// Run velocity-dependent computations.
SIM_API void sim_fwdVelocity(const sim_model_t* m, sim_data_t* d);

// Compute actuator force qfrc_actuator.
SIM_API void sim_fwdActuation(const sim_model_t* m, sim_data_t* d);

// Add up all non-constraint forces, compute qacc_smooth.
SIM_API void sim_fwdAcceleration(const sim_model_t* m, sim_data_t* d);

// Run selected constraint solver.
SIM_API void sim_fwdConstraint(const sim_model_t* m, sim_data_t* d);

// Euler integrator, semi-implicit in velocity.
SIM_API void sim_Euler(const sim_model_t* m, sim_data_t* d);

// Runge-Kutta explicit order-N integrator.
SIM_API void sim_RungeKutta(const sim_model_t* m, sim_data_t* d, int N);

// Implicit-in-velocity integrators.
SIM_API void sim_implicit(const sim_model_t* m, sim_data_t* d);

// Run position-dependent computations in inverse dynamics.
SIM_API void sim_invPosition(const sim_model_t* m, sim_data_t* d);

// Run velocity-dependent computations in inverse dynamics.
SIM_API void sim_invVelocity(const sim_model_t* m, sim_data_t* d);

// Apply the analytical formula for inverse constraint dynamics.
SIM_API void sim_invConstraint(const sim_model_t* m, sim_data_t* d);

// Compare forward and inverse dynamics, save results in fwdinv.
SIM_API void sim_compareFwdInv(const sim_model_t* m, sim_data_t* d);


//---------------------------------- Sub components ------------------------------------------------

// Evaluate position-dependent sensors.
SIM_API void sim_sensorPos(const sim_model_t* m, sim_data_t* d);

// Evaluate velocity-dependent sensors.
SIM_API void sim_sensorVel(const sim_model_t* m, sim_data_t* d);

// Evaluate acceleration and force-dependent sensors.
SIM_API void sim_sensorAcc(const sim_model_t* m, sim_data_t* d);

// Evaluate position-dependent energy (potential).
SIM_API void sim_energyPos(const sim_model_t* m, sim_data_t* d);

// Evaluate velocity-dependent energy (kinetic).
SIM_API void sim_energyVel(const sim_model_t* m, sim_data_t* d);

// Check qpos, reset if any element is too big or nan.
SIM_API void sim_checkPos(const sim_model_t* m, sim_data_t* d);

// Check qvel, reset if any element is too big or nan.
SIM_API void sim_checkVel(const sim_model_t* m, sim_data_t* d);

// Check qacc, reset if any element is too big or nan.
SIM_API void sim_checkAcc(const sim_model_t* m, sim_data_t* d);

// Run forward kinematics.
SIM_API void sim_kinematics(const sim_model_t* m, sim_data_t* d);

// Map inertias and motion dofs to global frame centered at CoM.
SIM_API void sim_comPos(const sim_model_t* m, sim_data_t* d);

// Compute camera and light positions and orientations.
SIM_API void sim_camlight(const sim_model_t* m, sim_data_t* d);

// Compute flex-related quantities.
SIM_API void sim_flex(const sim_model_t* m, sim_data_t* d);

// Compute tendon lengths, velocities and moment arms.
SIM_API void sim_tendon(const sim_model_t* m, sim_data_t* d);

// Compute actuator transmission lengths and moments.
SIM_API void sim_transmission(const sim_model_t* m, sim_data_t* d);

// Run composite rigid body inertia algorithm (CRB).
SIM_API void sim_crb(const sim_model_t* m, sim_data_t* d);

// Make inertia matrix.
SIM_API void sim_makeM(const sim_model_t* m, sim_data_t* d);

// Compute sparse L'*D*L factorizaton of inertia matrix.
SIM_API void sim_factorM(const sim_model_t* m, sim_data_t* d);

// Solve linear system M * x = y using factorization:  x = inv(L'*D*L)*y
SIM_API void sim_solveM(const sim_model_t* m, sim_data_t* d, sim_scalar_t* x, const sim_scalar_t* y, int n);

// Half of linear solve:  x = sqrt(inv(D))*inv(L')*y
SIM_API void sim_solveM2(const sim_model_t* m, sim_data_t* d, sim_scalar_t* x, const sim_scalar_t* y,
                      const sim_scalar_t* sqrtInvD, int n);

// Compute cvel, cdof_dot.
SIM_API void sim_comVel(const sim_model_t* m, sim_data_t* d);

// Compute qfrc_passive from spring-dampers, gravity compensation and fluid forces.
SIM_API void sim_passive(const sim_model_t* m, sim_data_t* d);

// Sub-tree linear velocity and angular momentum: compute subtree_linvel, subtree_angmom.
SIM_API void sim_subtreeVel(const sim_model_t* m, sim_data_t* d);

// RNE: compute M(qpos)*qacc + C(qpos,qvel); flg_acc=0 removes inertial term.
SIM_API void sim_rne(const sim_model_t* m, sim_data_t* d, int flg_acc, sim_scalar_t* result);

// RNE with complete data: compute cacc, cfrc_ext, cfrc_int.
SIM_API void sim_rnePostConstraint(const sim_model_t* m, sim_data_t* d);

// Run collision detection.
SIM_API void sim_collision(const sim_model_t* m, sim_data_t* d);

// Construct constraints.
SIM_API void sim_makeConstraint(const sim_model_t* m, sim_data_t* d);

// Find constraint islands.
SIM_API void sim_island(const sim_model_t* m, sim_data_t* d);

// Compute inverse constraint inertia efc_AR.
SIM_API void sim_projectConstraint(const sim_model_t* m, sim_data_t* d);

// Compute efc_vel, efc_aref.
SIM_API void sim_referenceConstraint(const sim_model_t* m, sim_data_t* d);

// Compute efc_state, efc_force, qfrc_constraint, and (optionally) cone Hessians.
// If cost is not NULL, set *cost = s(jar) where jar = Jac*qacc-aref.
// Nullable: cost
SIM_API void sim_constraintUpdate(const sim_model_t* m, sim_data_t* d, const sim_scalar_t* jar,
                               sim_scalar_t cost[1], int flg_coneHessian);


//---------------------------------- Support -------------------------------------------------------

// Return size of state signature.
SIM_API int sim_stateSize(const sim_model_t* m, int sig);

// Get state.
SIM_API void sim_getState(const sim_model_t* m, const sim_data_t* d, sim_scalar_t* state, int sig);

// Extract a subset of components from a state previously obtained via sim_getState.
SIM_API void sim_extractState(const sim_model_t* m, const sim_scalar_t* src, int srcsig,
                           sim_scalar_t* dst, int dstsig);

// Set state.
SIM_API void sim_setState(const sim_model_t* m, sim_data_t* d, const sim_scalar_t* state, int sig);

// Copy state from src to dst.
SIM_API void sim_copyState(const sim_model_t* m, const sim_data_t* src, sim_data_t* dst, int sig);

// Read ctrl value for actuator at given time.
// Returns d->ctrl[id] if no history, otherwise reads from history buffer.
// interp: 0=zero-order-hold, 1=linear, 2=cubic spline.
SIM_API sim_scalar_t sim_readCtrl(const sim_model_t* m, const sim_data_t* d, int id, sim_scalar_t time, int interp);

// Read sensor value from history buffer at given time.
// Returns pointer to sensordata (no history) or history buffer (exact match),
// or NULL if interpolation performed (writes to result).
// interp: 0=zero-order-hold, 1=linear, 2=cubic spline.
SIM_API const sim_scalar_t* sim_readSensor(const sim_model_t* m, const sim_data_t* d, int id, sim_scalar_t time,
                                  sim_scalar_t* result, int interp);

// Initialize history buffer for actuator; if times is NULL, uses existing buffer timestamps.
// Nullable: times
SIM_API void sim_initCtrlHistory(const sim_model_t* m, sim_data_t* d, int id,
                              const sim_scalar_t* times, const sim_scalar_t* values);

// Initialize history buffer for sensor; if times is NULL, uses existing buffer timestamps.
// phase sets the user slot (last computation time for interval sensors).
// Nullable: times
SIM_API void sim_initSensorHistory(const sim_model_t* m, sim_data_t* d, int id,
                                const sim_scalar_t* times, const sim_scalar_t* values, sim_scalar_t phase);

// Copy current state to the k-th model keyframe.
SIM_API void sim_setKeyframe(sim_model_t* m, const sim_data_t* d, int k);

// Add contact to d->contact list; return 0 if success; 1 if buffer full.
SIM_API int sim_addContact(const sim_model_t* m, sim_data_t* d, const sim_contact_t* con);

// Determine type of friction cone.
SIM_API int sim_isPyramidal(const sim_model_t* m);

// Determine type of constraint Jacobian.
SIM_API int sim_isSparse(const sim_model_t* m);

// Determine type of solver (PGS is dual, CG and Newton are primal).
SIM_API int sim_isDual(const sim_model_t* m);

// Multiply dense or sparse constraint Jacobian by vector.
SIM_API void sim_mulJacVec(const sim_model_t* m, const sim_data_t* d, sim_scalar_t* res, const sim_scalar_t* vec);

// Multiply dense or sparse constraint Jacobian transpose by vector.
SIM_API void sim_mulJacTVec(const sim_model_t* m, const sim_data_t* d, sim_scalar_t* res, const sim_scalar_t* vec);

// Compute 3/6-by-nv end-effector Jacobian of global point attached to given body.
// Nullable: jacp, jacr
SIM_API void sim_jac(const sim_model_t* m, const sim_data_t* d, sim_scalar_t* jacp, sim_scalar_t* jacr,
                  const sim_scalar_t point[3], int body);

// Compute body frame end-effector Jacobian.
// Nullable: jacp, jacr
SIM_API void sim_jacBody(const sim_model_t* m, const sim_data_t* d, sim_scalar_t* jacp, sim_scalar_t* jacr, int body);

// Compute body center-of-mass end-effector Jacobian.
// Nullable: jacp, jacr
SIM_API void sim_jacBodyCom(const sim_model_t* m, const sim_data_t* d, sim_scalar_t* jacp, sim_scalar_t* jacr, int body);

// Compute subtree center-of-mass end-effector Jacobian.
SIM_API void sim_jacSubtreeCom(const sim_model_t* m, sim_data_t* d, sim_scalar_t* jacp, int body);

// Compute geom end-effector Jacobian.
// Nullable: jacp, jacr
SIM_API void sim_jacGeom(const sim_model_t* m, const sim_data_t* d, sim_scalar_t* jacp, sim_scalar_t* jacr, int geom);

// Compute site end-effector Jacobian.
// Nullable: jacp, jacr
SIM_API void sim_jacSite(const sim_model_t* m, const sim_data_t* d, sim_scalar_t* jacp, sim_scalar_t* jacr, int site);

// Compute translation end-effector Jacobian of point, and rotation Jacobian of axis.
// Nullable: jacPoint, jacAxis
SIM_API void sim_jacPointAxis(const sim_model_t* m, sim_data_t* d, sim_scalar_t* jacPoint, sim_scalar_t* jacAxis,
                           const sim_scalar_t point[3], const sim_scalar_t axis[3], int body);

// Compute 3/6-by-nv Jacobian time derivative of global point attached to given body.
// Nullable: jacp, jacr
SIM_API void sim_jacDot(const sim_model_t* m, const sim_data_t* d, sim_scalar_t* jacp, sim_scalar_t* jacr,
                     const sim_scalar_t point[3], int body);

// Compute subtree angular momentum matrix.
SIM_API void sim_angmomMat(const sim_model_t* m, sim_data_t* d, sim_scalar_t* mat, int body);

// Get id of object with the specified sim_obj_t type and name; return -1 if id not found.
SIM_API int sim_name2id(const sim_model_t* m, int type, const char* name);

// Get name of object with the specified sim_obj_t type and id; return NULL if name not found.
SIM_API const char* sim_id2name(const sim_model_t* m, int type, int id);

// Convert sparse inertia matrix M into full (i.e. dense) matrix.
SIM_API void sim_fullM(const sim_model_t* m, sim_scalar_t* dst, const sim_scalar_t* M);

// Multiply vector by inertia matrix.
SIM_API void sim_mulM(const sim_model_t* m, const sim_data_t* d, sim_scalar_t* res, const sim_scalar_t* vec);

// Multiply vector by (inertia matrix)^(1/2).
SIM_API void sim_mulM2(const sim_model_t* m, const sim_data_t* d, sim_scalar_t* res, const sim_scalar_t* vec);

// Add inertia matrix to destination matrix (lower triangle only).
// Destination can be sparse or dense when all int* are NULL.
// Nullable: rownnz, rowadr, colind
SIM_API void sim_addM(const sim_model_t* m, sim_data_t* d, sim_scalar_t* dst, int* rownnz, int* rowadr, int* colind);

// Apply Cartesian force and torque (outside xfrc_applied mechanism).
// Nullable: force, torque
SIM_API void sim_applyFT(const sim_model_t* m, sim_data_t* d, const sim_scalar_t force[3], const sim_scalar_t torque[3],
                      const sim_scalar_t point[3], int body, sim_scalar_t* qfrc_target);

// Compute object 6D velocity (rot:lin) in object-centered frame, world/local orientation.
SIM_API void sim_objectVelocity(const sim_model_t* m, const sim_data_t* d,
                             int objtype, int objid, sim_scalar_t res[6], int flg_local);

// Compute object 6D acceleration (rot:lin) in object-centered frame, world/local orientation.
SIM_API void sim_objectAcceleration(const sim_model_t* m, const sim_data_t* d,
                                 int objtype, int objid, sim_scalar_t res[6], int flg_local);

// Return smallest signed distance between two geoms and optionally segment from geom1 to geom2.
// Nullable: fromto
SIM_API sim_scalar_t sim_geomDistance(const sim_model_t* m, const sim_data_t* d, int geom1, int geom2,
                             sim_scalar_t distmax, sim_scalar_t fromto[6]);

// Extract 6D force:torque given contact id, in the contact frame.
SIM_API void sim_contactForce(const sim_model_t* m, const sim_data_t* d, int id, sim_scalar_t result[6]);

// Compute velocity by finite-differencing two positions.
SIM_API void sim_differentiatePos(const sim_model_t* m, sim_scalar_t* qvel, sim_scalar_t dt,
                               const sim_scalar_t* qpos1, const sim_scalar_t* qpos2);

// Integrate position with given velocity.
SIM_API void sim_integratePos(const sim_model_t* m, sim_scalar_t* qpos, const sim_scalar_t* qvel, sim_scalar_t dt);

// Normalize all quaternions in qpos-type vector.
SIM_API void sim_normalizeQuat(const sim_model_t* m, sim_scalar_t* qpos);

// Map from body local to global Cartesian coordinates, sameframe takes values from SIM_tSameFrame.
SIM_API void sim_local2Global(sim_data_t* d, sim_scalar_t xpos[3], sim_scalar_t xmat[9], const sim_scalar_t pos[3],
                           const sim_scalar_t quat[4], int body, sim_byte_t sameframe);

// Sum all body masses.
SIM_API sim_scalar_t sim_getTotalmass(const sim_model_t* m);

// Scale body masses and inertias to achieve specified total mass.
SIM_API void sim_setTotalmass(sim_model_t* m, sim_scalar_t newmass);

// Return a config attribute value of a plugin instance;
// NULL: invalid plugin instance ID or attribute name
SIM_API const char* sim_getPluginConfig(const sim_model_t* m, int plugin_id, const char* attrib);

// Load a dynamic library. The dynamic library is assumed to register one or more plugins.
SIM_API void sim_loadPluginLibrary(const char* path);

// Scan a directory and load all dynamic libraries. Dynamic libraries in the specified directory
// are assumed to register one or more plugins. Optionally, if a callback is specified, it is called
// for each dynamic library encountered that registers plugins.
SIM_API void sim_loadAllPluginLibraries(const char* directory, SIM_fPluginLibraryLoadCallback callback);

// Return version number: 1.0.2 is encoded as 102.
SIM_API int sim_version(void);

// Return the current version of SimCore as a null-terminated string.
SIM_API const char* sim_versionString(void);


//---------------------------------- Ray casting ---------------------------------------------------

// Intersect ray (pnt+x*vec, x>=0) with visible geoms, except geoms in bodyexclude.
// Return distance (x) to nearest surface, or -1 if no intersection.
// geomgroup and flg_static control group/static filtering; geomgroup==NULL skips group exclusion.
// Nullable: geomgroup, geomid, normal
SIM_API sim_scalar_t sim_ray(const sim_model_t* m, const sim_data_t* d, const sim_scalar_t pnt[3], const sim_scalar_t vec[3],
                    const sim_byte_t* geomgroup, sim_byte_t flg_static, int bodyexclude,
                    int geomid[1], sim_scalar_t normal[3]);

// Intersect multiple rays emanating from a single point, compute normals if given.
// Similar semantics to sim_ray, but vec, normal and dist are arrays.
// Geoms further than cutoff are ignored.
// Nullable: geomgroup, geomid, normal
SIM_API void sim_multiRay(const sim_model_t* m, sim_data_t* d, const sim_scalar_t pnt[3], const sim_scalar_t* vec,
                       const sim_byte_t* geomgroup, sim_byte_t flg_static, int bodyexclude,
                       int* geomid, sim_scalar_t* dist, sim_scalar_t* normal, int nray, sim_scalar_t cutoff);

// Intersect ray with hfield; return nearest distance or -1 if no intersection.
// Nullable: normal
SIM_API sim_scalar_t sim_rayHfield(const sim_model_t* m, const sim_data_t* d, int geomid,
                          const sim_scalar_t pnt[3], const sim_scalar_t vec[3], sim_scalar_t normal[3]);

// Intersect ray with mesh; return nearest distance or -1 if no intersection.
// Nullable: normal
SIM_API sim_scalar_t sim_rayMesh(const sim_model_t* m, const sim_data_t* d, int geomid,
                        const sim_scalar_t pnt[3], const sim_scalar_t vec[3], sim_scalar_t normal[3]);

// Intersect ray with pure geom; return nearest distance or -1 if no intersection.
// Nullable: normal
SIM_API sim_scalar_t sim_math_rayGeom(const sim_scalar_t pos[3], const sim_scalar_t mat[9], const sim_scalar_t size[3],
                         const sim_scalar_t pnt[3], const sim_scalar_t vec[3], int geomtype,
                         sim_scalar_t normal[3]);

// Intersect ray with flex; return nearest distance or -1 if no intersection,
// and also output nearest vertex id and surface normal.
// Nullable: vertid, normal
SIM_API sim_scalar_t sim_rayFlex(const sim_model_t* m, const sim_data_t* d, int flex_layer,
                        sim_byte_t flg_vert, sim_byte_t flg_edge, sim_byte_t flg_face,
                        sim_byte_t flg_skin, int flexid, const sim_scalar_t pnt[3],
                        const sim_scalar_t vec[3], int vertid[1], sim_scalar_t normal[3]);

// Intersect ray with skin; return nearest distance or -1 if no intersection,
// and also output nearest vertex id.
// Nullable: vertid
SIM_API sim_scalar_t sim_math_raySkin(int nface, int nvert, const int* face, const float* vert,
                         const sim_scalar_t pnt[3], const sim_scalar_t vec[3], int vertid[1]);

//---------------------------------- Error and memory ----------------------------------------------

// Main error function; does not return to caller.
SIM_API void sim_error(const char* msg, ...) SIM_PRINTFLIKE(1, 2);

// Deprecated: use sim_error.
SIM_API void sim_math_error_i(const char* msg, int i);

// Deprecated: use sim_error.
SIM_API void sim_math_error_s(const char* msg, const char* text);

// Main warning function; returns to caller.
SIM_API void sim_warning(const char* msg, ...) SIM_PRINTFLIKE(1, 2);

// Deprecated: use sim_warning.
SIM_API void sim_math_warning_i(const char* msg, int i);

// Deprecated: use sim_warning.
SIM_API void sim_math_warning_s(const char* msg, const char* text);

// Clear user error and memory handlers.
SIM_API void sim_math_clearHandlers(void);

// Allocate memory; byte-align on 64; pad size to multiple of 64.
SIM_API void* sim_malloc(size_t size);

// Free memory, using free() by default.
SIM_API void sim_free(void* ptr);

// High-level warning function: count warnings in sim_data_t, print only the first.
SIM_API void sim_runtime_warning(sim_data_t* d, int warning, int info);

// Write [datetime, type: message] to SIMCORE_LOG.TXT.
SIM_API void sim_math_writeLog(const char* type, const char* msg);

// Get compiler error message from spec.
SIM_API const char* sim_spec_getError(sim_spec_t* s);

// Return 1 if compiler error is a warning.
SIM_API int sim_spec_isWarning(sim_spec_t* s);


//---------------------------------- Standard math -------------------------------------------------

#if !defined(SIM_USESINGLE)
  #define sim_math_sqrt    sqrt
  #define sim_math_exp     exp
  #define sim_math_sin     sin
  #define sim_math_cos     cos
  #define sim_math_tan     tan
  #define sim_math_asin    asin
  #define sim_math_acos    acos
  #define sim_math_atan2   atan2
  #define sim_math_tanh    tanh
  #define sim_math_pow     pow
  #define sim_math_abs     fabs
  #define sim_math_log     log
  #define sim_math_log10   log10
  #define sim_math_floor   floor
  #define sim_math_ceil    ceil

#else
  #define sim_math_sqrt    sqrtf
  #define sim_math_exp     expf
  #define sim_math_sin     sinf
  #define sim_math_cos     cosf
  #define sim_math_tan     tanf
  #define sim_math_asin    asinf
  #define sim_math_acos    acosf
  #define sim_math_atan2   atan2f
  #define sim_math_tanh    tanhf
  #define sim_math_pow     powf
  #define sim_math_abs     fabsf
  #define sim_math_log     logf
  #define sim_math_log10   log10f
  #define sim_math_floor   floorf
  #define sim_math_ceil    ceilf
#endif


//---------------------------------- Vector math ---------------------------------------------------

// Set res = 0.
SIM_API void sim_math_zero_3(sim_scalar_t res[3]);

// Set res = vec.
SIM_API void sim_math_copy_3(sim_scalar_t res[3], const sim_scalar_t data[3]);

// Set res = vec*scl.
SIM_API void sim_math_scale_3(sim_scalar_t res[3], const sim_scalar_t vec[3], sim_scalar_t scl);

// Set res = vec1 + vec2.
SIM_API void sim_math_add3(sim_scalar_t res[3], const sim_scalar_t vec1[3], const sim_scalar_t vec2[3]);

// Set res = vec1 - vec2.
SIM_API void sim_math_sub_3(sim_scalar_t res[3], const sim_scalar_t vec1[3], const sim_scalar_t vec2[3]);

// Set res = res + vec.
SIM_API void sim_math_add_to_3(sim_scalar_t res[3], const sim_scalar_t vec[3]);

// Set res = res - vec.
SIM_API void sim_math_subFrom3(sim_scalar_t res[3], const sim_scalar_t vec[3]);

// Set res = res + vec*scl.
SIM_API void sim_math_add_to_scale_3(sim_scalar_t res[3], const sim_scalar_t vec[3], sim_scalar_t scl);

// Set res = vec1 + vec2*scl.
SIM_API void sim_math_addScl3(sim_scalar_t res[3], const sim_scalar_t vec1[3], const sim_scalar_t vec2[3], sim_scalar_t scl);

// Normalize vector; return length before normalization.
SIM_API sim_scalar_t sim_math_normalize_3(sim_scalar_t vec[3]);

// Return vector length (without normalizing the vector).
SIM_API sim_scalar_t sim_math_norm3(const sim_scalar_t vec[3]);

// Return dot-product of vec1 and vec2.
SIM_API sim_scalar_t sim_math_dot_3(const sim_scalar_t vec1[3], const sim_scalar_t vec2[3]);

// Return Cartesian distance between 3D vectors pos1 and pos2.
SIM_API sim_scalar_t sim_math_dist3(const sim_scalar_t pos1[3], const sim_scalar_t pos2[3]);

// Multiply 3-by-3 matrix by vector: res = mat * vec.
SIM_API void sim_math_mul_mat_vec_3(sim_scalar_t res[3], const sim_scalar_t mat[9], const sim_scalar_t vec[3]);

// Multiply transposed 3-by-3 matrix by vector: res = mat' * vec.
SIM_API void sim_math_mulMatTVec3(sim_scalar_t res[3], const sim_scalar_t mat[9], const sim_scalar_t vec[3]);

// Compute cross-product: res = cross(a, b).
SIM_API void sim_math_cross(sim_scalar_t res[3], const sim_scalar_t a[3], const sim_scalar_t b[3]);

// Set res = 0.
SIM_API void sim_math_zero4(sim_scalar_t res[4]);

// Set res = (1,0,0,0).
SIM_API void sim_math_unit4(sim_scalar_t res[4]);

// Set res = vec.
SIM_API void sim_math_copy4(sim_scalar_t res[4], const sim_scalar_t data[4]);

// Normalize vector; return length before normalization.
SIM_API sim_scalar_t sim_math_normalize4(sim_scalar_t vec[4]);

// Set res = 0.
SIM_API void sim_math_zero(sim_scalar_t* res, int n);

// Set res = val.
SIM_API void sim_math_fill(sim_scalar_t* res, sim_scalar_t val, int n);

// Set res = vec.
SIM_API void sim_math_copy(sim_scalar_t* res, const sim_scalar_t* vec, int n);

// Return sum(vec).
SIM_API sim_scalar_t sim_math_sum(const sim_scalar_t* vec, int n);

// Return L1 norm: sum(abs(vec)).
SIM_API sim_scalar_t sim_math_L1(const sim_scalar_t* vec, int n);

// Set res = vec*scl.
SIM_API void sim_math_scl(sim_scalar_t* res, const sim_scalar_t* vec, sim_scalar_t scl, int n);

// Set res = vec1 + vec2.
SIM_API void sim_math_add(sim_scalar_t* res, const sim_scalar_t* vec1, const sim_scalar_t* vec2, int n);

// Set res = vec1 - vec2.
SIM_API void sim_math_sub(sim_scalar_t* res, const sim_scalar_t* vec1, const sim_scalar_t* vec2, int n);

// Set res = res + vec.
SIM_API void sim_math_addTo(sim_scalar_t* res, const sim_scalar_t* vec, int n);

// Set res = res - vec.
SIM_API void sim_math_subFrom(sim_scalar_t* res, const sim_scalar_t* vec, int n);

// Set res = res + vec*scl.
SIM_API void sim_math_addToScl(sim_scalar_t* res, const sim_scalar_t* vec, sim_scalar_t scl, int n);

// Set res = vec1 + vec2*scl.
SIM_API void sim_math_addScl(sim_scalar_t* res, const sim_scalar_t* vec1, const sim_scalar_t* vec2, sim_scalar_t scl, int n);

// Normalize vector; return length before normalization.
SIM_API sim_scalar_t sim_math_normalize(sim_scalar_t* res, int n);

// Return vector length (without normalizing vector).
SIM_API sim_scalar_t sim_math_norm(const sim_scalar_t* res, int n);

// Return dot-product of vec1 and vec2.
SIM_API sim_scalar_t sim_math_dot(const sim_scalar_t* vec1, const sim_scalar_t* vec2, int n);

// Multiply matrix and vector: res = mat * vec.
SIM_API void sim_math_mulMatVec(sim_scalar_t* res, const sim_scalar_t* mat, const sim_scalar_t* vec, int nr, int nc);

// Multiply transposed matrix and vector: res = mat' * vec.
SIM_API void sim_math_mulMatTVec(sim_scalar_t* res, const sim_scalar_t* mat, const sim_scalar_t* vec, int nr, int nc);

// Multiply square matrix with vectors on both sides: return vec1' * mat * vec2.
SIM_API sim_scalar_t sim_math_mulVecMatVec(const sim_scalar_t* vec1, const sim_scalar_t* mat, const sim_scalar_t* vec2, int n);

// Transpose matrix: res = mat'.
SIM_API void sim_math_transpose(sim_scalar_t* res, const sim_scalar_t* mat, int nr, int nc);

// Symmetrize square matrix res = (mat + mat')/2.
SIM_API void sim_math_symmetrize(sim_scalar_t* res, const sim_scalar_t* mat, int n);

// Set mat to the identity matrix.
SIM_API void sim_math_eye(sim_scalar_t* mat, int n);

// Multiply matrices: res = mat1 * mat2.
SIM_API void sim_math_mulMatMat(sim_scalar_t* res, const sim_scalar_t* mat1, const sim_scalar_t* mat2,
                         int r1, int c1, int c2);

// Multiply matrices, second argument transposed: res = mat1 * mat2'.
SIM_API void sim_math_mulMatMatT(sim_scalar_t* res, const sim_scalar_t* mat1, const sim_scalar_t* mat2,
                          int r1, int c1, int r2);

// Multiply matrices, first argument transposed: res = mat1' * mat2.
SIM_API void sim_math_mulMatTMat(sim_scalar_t* res, const sim_scalar_t* mat1, const sim_scalar_t* mat2,
                          int r1, int c1, int c2);

// Set res = mat' * diag * mat if diag is not NULL, and res = mat' * mat otherwise.
SIM_API void sim_math_sqrMatTD(sim_scalar_t* res, const sim_scalar_t* mat, const sim_scalar_t* diag, int nr, int nc);

// Coordinate transform of 6D motion or force vector in rotation:translation format.
// rotnew2old is 3-by-3, NULL means no rotation; flg_force specifies force or motion type.
// Nullable: rotnew2old
SIM_API void sim_math_transformSpatial(sim_scalar_t res[6], const sim_scalar_t vec[6], int flg_force,
                                const sim_scalar_t newpos[3], const sim_scalar_t oldpos[3],
                                const sim_scalar_t rotnew2old[9]);


//---------------------------------- Sparse math ---------------------------------------------------

// Convert matrix from dense to sparse.
//  nnz is size of res and colind; return 1 if too small, 0 otherwise.
SIM_API int sim_math_dense2sparse(sim_scalar_t* res, const sim_scalar_t* mat, int nr, int nc,
                           int* rownnz, int* rowadr, int* colind, int nnz);

// Convert matrix from sparse to dense.
SIM_API void sim_math_sparse2dense(sim_scalar_t* res, const sim_scalar_t* mat, int nr, int nc,
                            const int* rownnz, const int* rowadr, const int* colind);


//---------------------------------- Quaternions ---------------------------------------------------

// Rotate vector by quaternion.
SIM_API void sim_math_rotVecQuat(sim_scalar_t res[3], const sim_scalar_t vec[3], const sim_scalar_t quat[4]);

// Conjugate quaternion, corresponding to opposite rotation.
SIM_API void sim_math_negQuat(sim_scalar_t res[4], const sim_scalar_t quat[4]);

// Multiply quaternions.
SIM_API void sim_math_mulQuat(sim_scalar_t res[4], const sim_scalar_t quat1[4], const sim_scalar_t quat2[4]);

// Multiply quaternion and axis.
SIM_API void sim_math_mulQuatAxis(sim_scalar_t res[4], const sim_scalar_t quat[4], const sim_scalar_t axis[3]);

// Convert axisAngle to quaternion.
SIM_API void sim_math_axisAngle2Quat(sim_scalar_t res[4], const sim_scalar_t axis[3], sim_scalar_t angle);

// Convert quaternion (corresponding to orientation difference) to 3D velocity.
SIM_API void sim_math_quat2Vel(sim_scalar_t res[3], const sim_scalar_t quat[4], sim_scalar_t dt);

// Subtract quaternions, express as 3D velocity: qb*quat(res) = qa.
SIM_API void sim_math_subQuat(sim_scalar_t res[3], const sim_scalar_t qa[4], const sim_scalar_t qb[4]);

// Convert quaternion to 3D rotation matrix.
SIM_API void sim_math_quat2Mat(sim_scalar_t res[9], const sim_scalar_t quat[4]);

// Convert 3D rotation matrix to quaternion.
SIM_API void sim_math_mat2Quat(sim_scalar_t quat[4], const sim_scalar_t mat[9]);

// Compute time-derivative of quaternion, given 3D rotational velocity.
SIM_API void sim_math_derivQuat(sim_scalar_t res[4], const sim_scalar_t quat[4], const sim_scalar_t vel[3]);

// Integrate quaternion given 3D angular velocity.
SIM_API void sim_math_quatIntegrate(sim_scalar_t quat[4], const sim_scalar_t vel[3], sim_scalar_t scale);

// Construct quaternion performing rotation from z-axis to given vector.
SIM_API void sim_math_quatZ2Vec(sim_scalar_t quat[4], const sim_scalar_t vec[3]);

// Extract 3D rotation from an arbitrary 3x3 matrix by refining the input quaternion.
// Return the number of iterations required to converge.
SIM_API int sim_math_mat2Rot(sim_scalar_t quat[4], const sim_scalar_t mat[9]);

// Convert sequence of Euler angles (radians) to quaternion.
// seq[0,1,2] must be in 'xyzXYZ', lower/upper-case mean intrinsic/extrinsic rotations.
SIM_API void sim_math_euler2Quat(sim_scalar_t quat[4], const sim_scalar_t euler[3], const char* seq);


//---------------------------------- Poses ---------------------------------------------------------

// Multiply two poses.
SIM_API void sim_math_mulPose(sim_scalar_t posres[3], sim_scalar_t quatres[4],
                       const sim_scalar_t pos1[3], const sim_scalar_t quat1[4],
                       const sim_scalar_t pos2[3], const sim_scalar_t quat2[4]);

// Conjugate pose, corresponding to the opposite spatial transformation.
SIM_API void sim_math_negPose(sim_scalar_t posres[3], sim_scalar_t quatres[4],
                       const sim_scalar_t pos[3], const sim_scalar_t quat[4]);

// Transform vector by pose.
SIM_API void sim_math_trnVecPose(sim_scalar_t res[3], const sim_scalar_t pos[3], const sim_scalar_t quat[4],
                          const sim_scalar_t vec[3]);


//--------------------------------- Decompositions / Solvers ---------------------------------------

// Cholesky decomposition: mat = L*L'; return rank, decomposition performed in-place into mat.
SIM_API int sim_math_cholFactor(sim_scalar_t* mat, int n, sim_scalar_t mindiag);

// Solve (mat*mat') * res = vec, where mat is a Cholesky factor.
SIM_API void sim_math_cholSolve(sim_scalar_t* res, const sim_scalar_t* mat, const sim_scalar_t* vec, int n);

// Cholesky rank-one update: L*L' +/- x*x'; return rank.
SIM_API int sim_math_cholUpdate(sim_scalar_t* mat, sim_scalar_t* x, int n, int flg_plus);

// Band-dense Cholesky decomposition.
//  Return minimum value in the factorized diagonal, or 0 if rank-deficient.
//  mat has (ntotal-ndense) x nband + ndense x ntotal elements.
//  The first (ntotal-ndense) x nband store the band part, left of diagonal, inclusive.
//  The second ndense x ntotal store the band part as entire dense rows.
//  Add diagadd+diagmul*mat_ii to diagonal before factorization.
SIM_API sim_scalar_t sim_math_cholFactorBand(sim_scalar_t* mat, int ntotal, int nband, int ndense,
                                sim_scalar_t diagadd, sim_scalar_t diagmul);

// Solve (mat*mat')*res = vec where mat is a band-dense Cholesky factor.
SIM_API void sim_math_cholSolveBand(sim_scalar_t* res, const sim_scalar_t* mat, const sim_scalar_t* vec,
                             int ntotal, int nband, int ndense);

// Convert banded matrix to dense matrix, fill upper triangle if flg_sym>0.
SIM_API void sim_math_band2Dense(sim_scalar_t* res, const sim_scalar_t* mat, int ntotal, int nband, int ndense,
                          sim_byte_t flg_sym);

// Convert dense matrix to banded matrix.
SIM_API void sim_math_dense2Band(sim_scalar_t* res, const sim_scalar_t* mat, int ntotal, int nband, int ndense);

// Multiply band-diagonal matrix with nvec vectors, include upper triangle if flg_sym>0.
SIM_API void sim_math_bandMulMatVec(sim_scalar_t* res, const sim_scalar_t* mat, const sim_scalar_t* vec,
                             int ntotal, int nband, int ndense, int nvec, sim_byte_t flg_sym);

// Address of diagonal element i in band-dense matrix representation.
SIM_API int sim_math_bandDiag(int i, int ntotal, int nband, int ndense);

// Eigenvalue decomposition of symmetric 3x3 matrix, mat = eigvec * diag(eigval) * eigvec'.
SIM_API int sim_math_eig3(sim_scalar_t eigval[3], sim_scalar_t eigvec[9], sim_scalar_t quat[4], const sim_scalar_t mat[9]);

// minimize 0.5*x'*H*x + x'*g  s.t. lower <= x <= upper; return rank or -1 if failed
//   inputs:
//     n           - problem dimension
//     H           - SPD matrix                n*n
//     g           - bias vector               n
//     lower       - lower bounds              n
//     upper       - upper bounds              n
//     res         - solution warmstart        n
//   return value:
//     nfree <= n  - rank of unconstrained subspace, -1 if failure
//   outputs (required):
//     res         - solution                  n
//     R           - subspace Cholesky factor  nfree*nfree    allocated: n*(n+7)
//   outputs (optional):
//     index       - set of free dimensions    nfree          allocated: n
//   notes:
//     the initial value of res is used to warmstart the solver
//     R must have allocatd size n*(n+7), but only nfree*nfree values are used in output
//     index (if given) must have allocated size n, but only nfree values are used in output
//     only the lower triangles of H and R and are read from and written to, respectively
//     the convenience function sim_math_boxQPmalloc allocates the required data structures
// Nullable: index, lower, upper
SIM_API int sim_math_boxQP(sim_scalar_t* res, sim_scalar_t* R, int* index, const sim_scalar_t* H, const sim_scalar_t* g, int n,
                    const sim_scalar_t* lower, const sim_scalar_t* upper);

// allocate heap memory for box-constrained Quadratic Program
//   as in sim_math_boxQP, index, lower, and upper are optional
//   free all pointers with sim_free()
SIM_API void sim_math_boxQPmalloc(sim_scalar_t** res, sim_scalar_t** R, int** index, sim_scalar_t** H, sim_scalar_t** g, int n,
                           sim_scalar_t** lower, sim_scalar_t** upper);


//---------------------------------- Miscellaneous -------------------------------------------------

// Muscle active force, prm = (range[2], force, scale, lmin, lmax, vmax, fpmax, fvmax).
SIM_API sim_scalar_t sim_math_muscleGain(sim_scalar_t len, sim_scalar_t vel, const sim_scalar_t lengthrange[2],
                            sim_scalar_t acc0, const sim_scalar_t prm[9]);

// Muscle passive force, prm = (range[2], force, scale, lmin, lmax, vmax, fpmax, fvmax).
SIM_API sim_scalar_t sim_math_muscleBias(sim_scalar_t len, const sim_scalar_t lengthrange[2],
                            sim_scalar_t acc0, const sim_scalar_t prm[9]);

// Muscle activation dynamics, prm = (tau_act, tau_deact, smoothing_width).
SIM_API sim_scalar_t sim_math_muscleDynamics(sim_scalar_t ctrl, sim_scalar_t act, const sim_scalar_t prm[3]);

// Convert contact force to pyramid representation.
SIM_API void sim_math_encodePyramid(sim_scalar_t* pyramid, const sim_scalar_t* force, const sim_scalar_t* mu, int dim);

// Convert pyramid representation to contact force.
SIM_API void sim_math_decodePyramid(sim_scalar_t* force, const sim_scalar_t* pyramid, const sim_scalar_t* mu, int dim);

// Integrate spring-damper analytically; return pos(dt).
SIM_API sim_scalar_t sim_math_springDamper(sim_scalar_t pos0, sim_scalar_t vel0, sim_scalar_t Kp, sim_scalar_t Kv, sim_scalar_t dt);

// Return min(a,b) with single evaluation of a and b.
SIM_API sim_scalar_t sim_math_min(sim_scalar_t a, sim_scalar_t b);

// Return max(a,b) with single evaluation of a and b.
SIM_API sim_scalar_t sim_math_max(sim_scalar_t a, sim_scalar_t b);

// Clip x to the range [min, max].
SIM_API sim_scalar_t sim_math_clip(sim_scalar_t x, sim_scalar_t min, sim_scalar_t max);

// Return sign of x: +1, -1 or 0.
SIM_API sim_scalar_t sim_math_sign(sim_scalar_t x);

// Round x to nearest integer.
SIM_API int sim_math_round(sim_scalar_t x);

// Convert type id (sim_obj_t) to type name.
SIM_API const char* sim_math_type2Str(int type);

// Convert type name to type id (sim_obj_t).
SIM_API int sim_math_str2Type(const char* str);

// Return human readable number of bytes using standard letter suffix.
SIM_API const char* sim_math_writeNumBytes(size_t nbytes);

// Construct a warning message given the warning type and info.
SIM_API const char* sim_math_warningText(int warning, size_t info);

// Return 1 if nan or abs(x)>SIM_MAXVAL, 0 otherwise. Used by check functions.
SIM_API int sim_math_isBad(sim_scalar_t x);

// Return 1 if all elements are 0.
SIM_API int sim_math_isZero(const sim_scalar_t* vec, int n);

// Standard normal random number generator (optional second number).
SIM_API sim_scalar_t sim_math_standardNormal(sim_scalar_t* num2);

// Convert from float to sim_scalar_t.
SIM_API void sim_math_f2n(sim_scalar_t* res, const float* vec, int n);

// Convert from sim_scalar_t to float.
SIM_API void sim_math_n2f(float* res, const sim_scalar_t* vec, int n);

// Convert from double to sim_scalar_t.
SIM_API void sim_math_d2n(sim_scalar_t* res, const double* vec, int n);

// Convert from sim_scalar_t to double.
SIM_API void sim_math_n2d(double* res, const sim_scalar_t* vec, int n);

// Insertion sort, resulting list is in increasing order.
SIM_API void sim_math_insertionSort(sim_scalar_t* list, int n);

// Integer insertion sort, resulting list is in increasing order.
SIM_API void sim_math_insertionSortInt(int* list, int n);

// Generate Halton sequence.
SIM_API sim_scalar_t sim_math_Halton(int index, int base);

// Call strncpy, then set dst[n-1] = 0.
SIM_API char* sim_math_strncpy(char *dst, const char *src, int n);

// Sigmoid function over 0<=x<=1 using quintic polynomial.
SIM_API sim_scalar_t sim_math_sigmoid(sim_scalar_t x);


//---------------------------------- Signed Distance Function --------------------------------------

// get sdf from geom id
SIM_API const SIM_pPlugin* SIM_c_getSDF(const sim_model_t* m, int id);

// signed distance function
SIM_API sim_scalar_t SIM_c_distance(const sim_model_t* m, const sim_data_t* d, const SIM_SDF* s, const sim_scalar_t x[3]);

// gradient of sdf
SIM_API void SIM_c_gradient(const sim_model_t* m, const sim_data_t* d, const SIM_SDF* s, sim_scalar_t gradient[3],
                        const sim_scalar_t x[3]);


//---------------------------------- Derivatives ---------------------------------------------------

// Finite differenced transition matrices (control theory notation)
//   d(x_next) = A*dx + B*du
//   d(sensor) = C*dx + D*du
//   required output matrix dimensions:
//      A: (2*nv+na x 2*nv+na)
//      B: (2*nv+na x nu)
//      D: (nsensordata x 2*nv+na)
//      C: (nsensordata x nu)
// Nullable: A, B, C, D
SIM_API void SIM_d_transitionFD(const sim_model_t* m, sim_data_t* d, sim_scalar_t eps, sim_byte_t flg_centered,
                            sim_scalar_t* A, sim_scalar_t* B, sim_scalar_t* C, sim_scalar_t* D);

// Finite differenced Jacobians of (force, sensors) = sim_inverse(state, acceleration)
//   All outputs are optional. Output dimensions (transposed w.r.t Control Theory convention):
//     DfDq: (nv x nv)
//     DfDv: (nv x nv)
//     DfDa: (nv x nv)
//     DsDq: (nv x nsensordata)
//     DsDv: (nv x nsensordata)
//     DsDa: (nv x nsensordata)
//     DmDq: (nv x nM)
//   single-letter shortcuts:
//     inputs: q=qpos, v=qvel, a=qacc
//     outputs: f=qfrc_inverse, s=sensordata, m=qM
//   notes:
//     optionally computes mass matrix Jacobian DmDq
//     flg_actuation specifies whether to subtract qfrc_actuator from qfrc_inverse
// Nullable: DfDq, DfDv, DfDa, DsDq, DsDv, DsDa, DmDq
SIM_API void SIM_d_inverseFD(const sim_model_t* m, sim_data_t* d, sim_scalar_t eps, sim_byte_t flg_actuation,
                         sim_scalar_t *DfDq, sim_scalar_t *DfDv, sim_scalar_t *DfDa,
                         sim_scalar_t *DsDq, sim_scalar_t *DsDv, sim_scalar_t *DsDa,
                         sim_scalar_t *DmDq);

// Derivatives of sim_math_subQuat.
// Nullable: Da, Db
SIM_API void SIM_d_subQuat(const sim_scalar_t qa[4], const sim_scalar_t qb[4], sim_scalar_t Da[9], sim_scalar_t Db[9]);

// Derivatives of sim_math_quatIntegrate.
// Nullable: Dquat, Dvel, Dscale
SIM_API void SIM_d_quatIntegrate(const sim_scalar_t vel[3], sim_scalar_t scale,
                             sim_scalar_t Dquat[9], sim_scalar_t Dvel[9], sim_scalar_t Dscale[3]);


//---------------------------------- Plugins -------------------------------------------------------

// Set default plugin definition.
SIM_API void sim_plugin_defaultPlugin(SIM_pPlugin* plugin);

// Globally register a plugin. This function is thread-safe.
// If an identical SIM_pPlugin is already registered, this function does nothing.
// If a non-identical SIM_pPlugin with the same name is already registered, an sim_error is raised.
// Two SIM_pPlugins are considered identical if all member function pointers and numbers are equal,
// and the name and attribute strings are all identical, however the char pointers to the strings
// need not be the same.
SIM_API int sim_plugin_registerPlugin(const SIM_pPlugin* plugin);

// Return the number of globally registered plugins.
SIM_API int sim_plugin_pluginCount(void);

// Look up a plugin by name. If slot is not NULL, also write its registered slot number into it.
SIM_API const SIM_pPlugin* sim_plugin_getPlugin(const char* name, int* slot);

// Look up a plugin by the registered slot number that was returned by sim_plugin_registerPlugin.
SIM_API const SIM_pPlugin* sim_plugin_getPluginAtSlot(int slot);

// Set default resource provider definition.
SIM_API void sim_plugin_defaultResourceProvider(SIM_pResourceProvider* provider);

// Globally register a resource provider in a thread-safe manner. The provider must have a prefix
// that is not a sub-prefix or super-prefix of any current registered providers.
// Return a slot number >= 0 on success, -1 on failure.
SIM_API int sim_plugin_registerResourceProvider(const SIM_pResourceProvider* provider);

// Return the number of globally registered resource providers.
SIM_API int sim_plugin_resourceProviderCount(void);

// Return the resource provider with the prefix that matches against the resource name.
// If no match, return NULL.
SIM_API const SIM_pResourceProvider* sim_plugin_getResourceProvider(const char* resource_name);

// Look up a resource provider by slot number returned by sim_plugin_registerResourceProvider.
// If invalid slot number, return NULL.
SIM_API const SIM_pResourceProvider* sim_plugin_getResourceProviderAtSlot(int slot);

// Globally register a decoder. This function is thread-safe.
// If an identical SIM_pDecoder is already registered, this function does nothing.
// If a non-identical SIM_pDecoder with the same name is already registered, an sim_error is raised.
SIM_API void sim_plugin_registerDecoder(const SIM_pDecoder* decoder);

// Set default resource decoder definition.
SIM_API void sim_plugin_defaultDecoder(SIM_pDecoder* decoder);

// Return the resource provider with the prefix that matches against the resource name.
// If no match, return NULL.
SIM_API const SIM_pDecoder* sim_plugin_findDecoder(const SIM_Resource* resource, const char* content_type);


//---------------------------------- Resources -----------------------------------------------------

// Open a resource; if the name doesn't have a prefix matching a registered resource provider,
// then the OS filesystem is used.
// Nullable: dir, vfs, error
SIM_API SIM_Resource* sim_math_openResource(const char* dir, const char* name,
                                   const SIM_VFS* vfs, char* error, size_t nerror);

// Close a resource; no-op if resource is NULL.
SIM_API void sim_math_closeResource(SIM_Resource* resource);

// Set buffer to bytes read from the resource and return number of bytes in buffer;
// return negative value if error.
SIM_API int sim_math_readResource(SIM_Resource* resource, const void** buffer);

// For a resource with a name partitioned as {dir}{filename}, get the dir and ndir pointers.
SIM_API void sim_math_getResourceDir(SIM_Resource* resource, const char** dir, int* ndir);

// Compare resource timestamp to provided timestamp.
// Return 0 if timestamps match, >0 if resource is newer, <0 if resource is older.
SIM_API int sim_math_isModifiedResource(const SIM_Resource* resource, const char* timestamp);

// Find the decoder for a resource and return the decoded spec.
// The caller takes ownership of the spec and is responsible for cleaning it up.
// Nullable: vfs
SIM_API sim_spec_t* sim_math_decodeResource(SIM_Resource* resource, const char* content_type,
                                 const SIM_VFS* vfs);


//---------------------------------- Threads -------------------------------------------------------

// Create a thread pool with the specified number of threads running.
SIM_API SIM_ThreadPool* sim_math_threadPoolCreate(size_t number_of_threads);

// Adds a thread pool to sim_data_t and configures it for multi-threaded use.
SIM_API void sim_math_bindThreadPool(sim_data_t* d, void* thread_pool);

// Enqueue a task in a thread pool.
SIM_API void sim_math_threadPoolEnqueue(SIM_ThreadPool* thread_pool, SIM_Task* task);

// Destroy a thread pool.
SIM_API void sim_math_threadPoolDestroy(SIM_ThreadPool* thread_pool);

// Initialize an SIM_Task.
SIM_API void sim_math_defaultTask(SIM_Task* task);

// Wait for a task to complete.
SIM_API void sim_math_taskJoin(SIM_Task* task);


//---------------------------------- Attachment ----------------------------------------------------

// Attach child to a parent; return the attached element if success or NULL otherwise.
SIM_API sim_spec_element_t* sim_spec_attach(sim_spec_element_t* parent, const sim_spec_element_t* child,
                             const char* prefix, const char* suffix);


//---------------------------------- Tree elements -------------------------------------------------

// Add child body to body; return child.
// Nullable: def
SIM_API sim_spec_body_t* sim_spec_addBody(sim_spec_body_t* body, const sim_spec_default_t* def);

// Add site to body; return site spec.
// Nullable: def
SIM_API SIM_sSite* sim_spec_addSite(sim_spec_body_t* body, const sim_spec_default_t* def);

// Add joint to body.
// Nullable: def
SIM_API SIM_sJoint* sim_spec_addJoint(sim_spec_body_t* body, const sim_spec_default_t* def);

// Add freejoint to body.
SIM_API SIM_sJoint* sim_spec_addFreeJoint(sim_spec_body_t* body);

// Add geom to body.
// Nullable: def
SIM_API sim_spec_geom_t* sim_spec_addGeom(sim_spec_body_t* body, const sim_spec_default_t* def);

// Add frame to body.
SIM_API SIM_sFrame* sim_spec_addFrame(sim_spec_body_t* body, SIM_sFrame* parentframe);

// Remove object corresponding to the given element; return 0 on success.
SIM_API int sim_spec_delete(sim_spec_t* spec, sim_spec_element_t* element);


//---------------------------------- Non-tree elements ---------------------------------------------

// Add actuator.
// Nullable: def
SIM_API sim_spec_actuator_t* sim_spec_addActuator(sim_spec_t* s, const sim_spec_default_t* def);

// Add sensor.
SIM_API SIM_sSensor* sim_spec_addSensor(sim_spec_t* s);

// Add flex.
SIM_API SIM_sFlex* sim_spec_addFlex(sim_spec_t* s);

// Add contact pair.
// Nullable: def
SIM_API SIM_sPair* sim_spec_addPair(sim_spec_t* s, const sim_spec_default_t* def);

// Add excluded body pair.
SIM_API SIM_sExclude* sim_spec_addExclude(sim_spec_t* s);

// Add equality.
// Nullable: def
SIM_API SIM_sEquality* sim_spec_addEquality(sim_spec_t* s, const sim_spec_default_t* def);

// Add tendon.
// Nullable: def
SIM_API SIM_sTendon* sim_spec_addTendon(sim_spec_t* s, const sim_spec_default_t* def);

// Wrap site using tendon.
SIM_API SIM_sWrap* sim_spec_wrapSite(SIM_sTendon* tendon, const char* name);

// Wrap geom using tendon.
SIM_API SIM_sWrap* sim_spec_wrapGeom(SIM_sTendon* tendon, const char* name, const char* sidesite);

// Wrap joint using tendon.
SIM_API SIM_sWrap* sim_spec_wrapJoint(SIM_sTendon* tendon, const char* name, double coef);

// Wrap pulley using tendon.
SIM_API SIM_sWrap* sim_spec_wrapPulley(SIM_sTendon* tendon, double divisor);

// Add numeric.
SIM_API SIM_sNumeric* sim_spec_addNumeric(sim_spec_t* s);

// Add text.
SIM_API SIM_sText* sim_spec_addText(sim_spec_t* s);

// Add tuple.
SIM_API SIM_sTuple* sim_spec_addTuple(sim_spec_t* s);

// Add keyframe.
SIM_API SIM_sKey* sim_spec_addKey(sim_spec_t* s);

// Add plugin.
SIM_API SIM_sPlugin* sim_spec_addPlugin(sim_spec_t* s);

// Add default.
// Nullable: parent
SIM_API sim_spec_default_t* sim_spec_addDefault(sim_spec_t* s, const char* classname, const sim_spec_default_t* parent);


//---------------------------------- Set actuator parameters ---------------------------------------

// Set actuator to motor; return error if any.
SIM_API const char* sim_spec_setToMotor(sim_spec_actuator_t* actuator);

// Set actuator to position; return error if any.
SIM_API const char* sim_spec_setToPosition(sim_spec_actuator_t* actuator, double kp, double kv[1],
                                    double dampratio[1], double timeconst[1], double inheritrange);

// Set actuator to integrated velocity; return error if any.
SIM_API const char* sim_spec_setToIntVelocity(sim_spec_actuator_t* actuator, double kp, double kv[1],
                                       double dampratio[1], double timeconst[1], double inheritrange);

// Set actuator to velocity servo; return error if any.
SIM_API const char* sim_spec_setToVelocity(sim_spec_actuator_t* actuator, double kv);

// Set actuator to activate damper; return error if any.
SIM_API const char* sim_spec_setToDamper(sim_spec_actuator_t* actuator, double kv);

// Set actuator to hydraulic or pneumatic cylinder; return error if any.
SIM_API const char* sim_spec_setToCylinder(sim_spec_actuator_t* actuator, double timeconst,
                                    double bias, double area, double diameter);

// Set actuator to muscle; return error if any.a
SIM_API const char* sim_spec_setToMuscle(sim_spec_actuator_t* actuator, double timeconst[2], double tausmooth,
                                  double range[2], double force, double scale, double lmin,
                                  double lmax, double vmax, double fpmax, double fvmax);

// Set actuator to active adhesion; return error if any.
SIM_API const char* sim_spec_setToAdhesion(sim_spec_actuator_t* actuator, double gain);


//---------------------------------- Assets --------------------------------------------------------

// Add mesh.
// Nullable: def
SIM_API SIM_sMesh* sim_spec_addMesh(sim_spec_t* s, const sim_spec_default_t* def);

// Add height field.
SIM_API SIM_sHField* sim_spec_addHField(sim_spec_t* s);

// Add skin.
SIM_API SIM_sSkin* sim_spec_addSkin(sim_spec_t* s);

// Sets the vertices and normals of a mesh.
SIM_API int sim_spec_makeMesh(SIM_sMesh* mesh, SIM_tMeshBuiltin builtin, double* params, int nparams);

//---------------------------------- Find and get utilities ----------------------------------------

// Get spec from body.
SIM_API sim_spec_t* sim_spec_getSpec(sim_spec_element_t* element);

// Find spec (model asset) by name.
SIM_API sim_spec_t* sim_spec_findSpec(sim_spec_t* spec, const char* name);

// Find body in spec by name.
SIM_API sim_spec_body_t* sim_spec_findBody(sim_spec_t* s, const char* name);

// Find element in spec by name.
SIM_API sim_spec_element_t* sim_spec_findElement(sim_spec_t* s, sim_obj_t type, const char* name);

// Find child body by name.
SIM_API sim_spec_body_t* sim_spec_findChild(sim_spec_body_t* body, const char* name);

// Get parent body.
SIM_API sim_spec_body_t* sim_spec_getParent(sim_spec_element_t* element);

// Get parent frame.
SIM_API SIM_sFrame* sim_spec_getFrame(sim_spec_element_t* element);

// Find frame by name.
SIM_API SIM_sFrame* sim_spec_findFrame(sim_spec_t* s, const char* name);

// Get default corresponding to an element.
SIM_API sim_spec_default_t* sim_spec_getDefault(sim_spec_element_t* element);

// Find default in model by class name.
SIM_API sim_spec_default_t* sim_spec_findDefault(sim_spec_t* s, const char* classname);

// Get global default from model.
SIM_API sim_spec_default_t* sim_spec_getSpecDefault(sim_spec_t* s);

// Get element id.
SIM_API int sim_spec_getId(sim_spec_element_t* element);

// Return body's first child of given type. If recurse is nonzero, also search the body's subtree.
SIM_API sim_spec_element_t* sim_spec_firstChild(sim_spec_body_t* body, sim_obj_t type, int recurse);

// Return body's next child of the same type; return NULL if child is last.
// If recurse is nonzero, also search the body's subtree.
SIM_API sim_spec_element_t* sim_spec_nextChild(sim_spec_body_t* body, sim_spec_element_t* child, int recurse);

// Return spec's first element of selected type.
SIM_API sim_spec_element_t* sim_spec_firstElement(sim_spec_t* s, sim_obj_t type);

// Return spec's next element; return NULL if element is last.
SIM_API sim_spec_element_t* sim_spec_nextElement(sim_spec_t* s, sim_spec_element_t* element);

// Get wrapped element in tendon path.
SIM_API sim_spec_element_t* sim_spec_getWrapTarget(SIM_sWrap* wrap);

// Get wrapped element side site in tendon path if it has one, nullptr otherwise.
SIM_API SIM_sSite* sim_spec_getWrapSideSite(SIM_sWrap* wrap);

// Get divisor of SIM_sWrap wrapping a puller.
SIM_API double sim_spec_getWrapDivisor(SIM_sWrap* wrap);

// Get coefficient of SIM_sWrap wrapping a joint.
SIM_API double sim_spec_getWrapCoef(SIM_sWrap* wrap);

//---------------------------------- Attribute setters ---------------------------------------------

// Set element's name; return 0 on success.
SIM_API int sim_spec_set_name(sim_spec_element_t* element, const char* name);

// Copy buffer.
SIM_API void sim_spec_setBuffer(SIM_ByteVec* dest, const void* array, int size);

// Copy text to string.
SIM_API void sim_spec_set_string(sim_string_t* dest, const char* text);

// Split text to entries and copy to string vector.
SIM_API void sim_spec_setStringVec(SIM_StringVec* dest, const char* text);

// Set entry in string vector.
SIM_API sim_byte_t sim_spec_setInStringVec(SIM_StringVec* dest, int i, const char* text);

// Append text entry to string vector.
SIM_API void sim_spec_appendString(SIM_StringVec* dest, const char* text);

// Copy int array to vector.
SIM_API void sim_spec_setInt(SIM_IntVec* dest, const int* array, int size);

// Append int array to vector of arrays.
SIM_API void sim_spec_appendIntVec(SIM_IntVecVec* dest, const int* array, int size);

// Copy float array to vector.
SIM_API void sim_spec_setFloat(SIM_FloatVec* dest, const float* array, int size);

// Append float array to vector of arrays.
SIM_API void sim_spec_appendFloatVec(SIM_FloatVecVec* dest, const float* array, int size);

// Copy double array to vector.
SIM_API void sim_spec_setDouble(SIM_DoubleVec* dest, const double* array, int size);

// Set plugin attributes.
SIM_API void sim_spec_setPluginAttributes(SIM_sPlugin* plugin, void* attributes);


//---------------------------------- Attribute getters ---------------------------------------------

// Get element's name.
SIM_API sim_string_t* sim_spec_getName(sim_spec_element_t* element);

// Get string contents.
SIM_API const char* sim_spec_getString(const sim_string_t* source);

// Get double array contents and optionally its size.
// Nullable: size
SIM_API const double* sim_spec_getDouble(const SIM_DoubleVec* source, int* size);

// Get number of elements a tendon wraps.
SIM_API int sim_spec_getWrapNum(const SIM_sTendon* tendonspec);

// Get SIM_sWrap element at position i in the tendon path.
SIM_API SIM_sWrap* sim_spec_getWrap(const SIM_sTendon* tendonspec, int i);

// Get plugin attributes.
SIM_API const void* sim_spec_getPluginAttributes(const SIM_sPlugin* plugin);


//---------------------------------- Spec utilities ------------------------------------------------

// Set element's default.
SIM_API void sim_spec_setDefault(sim_spec_element_t* element, const sim_spec_default_t* def);

// Set element's enclosing frame; return 0 on success.
SIM_API int sim_spec_setFrame(sim_spec_element_t* dest, SIM_sFrame* frame);

// Resolve alternative orientations to quat; return error if any.
SIM_API const char* sim_spec_resolveOrientation(double quat[4], sim_byte_t degree, const char* sequence,
                                         const SIM_sOrientation* orientation);

// Transform body into a frame.
SIM_API SIM_sFrame* sim_spec_bodyToFrame(sim_spec_body_t** body);

// Set user payload, overriding the existing value for the specified key if present.
SIM_API void sim_spec_setUserValue(sim_spec_element_t* element, const char* key, const void* data);

// Set user payload, overriding the existing value for the specified key if
// present. This version differs from sim_spec_setUserValue in that it takes a
// cleanup function that will be called when the user payload is deleted.
SIM_API void sim_spec_setUserValueWithCleanup(sim_spec_element_t* element, const char* key,
                                       const void* data,
                                       void (*cleanup)(const void*));

// Return user payload or NULL if none found.
SIM_API const void* sim_spec_getUserValue(sim_spec_element_t* element, const char* key);

// Delete user payload.
SIM_API void sim_spec_deleteUserValue(sim_spec_element_t* element, const char* key);

// Return sensor dimension.
SIM_API int sim_spec_sensorDim(const SIM_sSensor* sensor);

//---------------------------------- Element initialization  ---------------------------------------

// Default spec attributes.
SIM_API void sim_spec_defaultSpec(sim_spec_t* spec);

// Default orientation attributes.
SIM_API void sim_spec_defaultOrientation(SIM_sOrientation* orient);

// Default body attributes.
SIM_API void sim_spec_defaultBody(sim_spec_body_t* body);

// Default frame attributes.
SIM_API void sim_spec_defaultFrame(SIM_sFrame* frame);

// Default joint attributes.
SIM_API void sim_spec_defaultJoint(SIM_sJoint* joint);

// Default geom attributes.
SIM_API void sim_spec_defaultGeom(sim_spec_geom_t* geom);

// Default site attributes.
SIM_API void sim_spec_defaultSite(SIM_sSite* site);

// Default flex attributes.
SIM_API void sim_spec_defaultFlex(SIM_sFlex* flex);

// Default mesh attributes.
SIM_API void sim_spec_defaultMesh(SIM_sMesh* mesh);

// Default height field attributes.
SIM_API void sim_spec_defaultHField(SIM_sHField* hfield);

// Default skin attributes.
SIM_API void sim_spec_defaultSkin(SIM_sSkin* skin);

// Default pair attributes.
SIM_API void sim_spec_defaultPair(SIM_sPair* pair);

// Default equality attributes.
SIM_API void sim_spec_defaultEquality(SIM_sEquality* equality);

// Default tendon attributes.
SIM_API void sim_spec_defaultTendon(SIM_sTendon* tendon);

// Default actuator attributes.
SIM_API void sim_spec_defaultActuator(sim_spec_actuator_t* actuator);

// Default sensor attributes.
SIM_API void sim_spec_defaultSensor(SIM_sSensor* sensor);

// Default numeric attributes.
SIM_API void sim_spec_defaultNumeric(SIM_sNumeric* numeric);

// Default text attributes.
SIM_API void sim_spec_defaultText(SIM_sText* text);

// Default tuple attributes.
SIM_API void sim_spec_defaultTuple(SIM_sTuple* tuple);

// Default keyframe attributes.
SIM_API void sim_spec_defaultKey(SIM_sKey* key);

// Default plugin attributes.
SIM_API void sim_spec_defaultPlugin(SIM_sPlugin* plugin);


//---------------------------------- Element casting -----------------------------------------------

// Safely cast an element as sim_spec_body_t, or return NULL if the element is not an sim_spec_body_t.
SIM_API sim_spec_body_t* sim_spec_asBody(sim_spec_element_t* element);

// Safely cast an element as sim_spec_geom_t, or return NULL if the element is not an sim_spec_geom_t.
SIM_API sim_spec_geom_t* sim_spec_asGeom(sim_spec_element_t* element);

// Safely cast an element as SIM_sJoint, or return NULL if the element is not an SIM_sJoint.
SIM_API SIM_sJoint* sim_spec_asJoint(sim_spec_element_t* element);

// Safely cast an element as SIM_sSite, or return NULL if the element is not an SIM_sSite.
SIM_API SIM_sSite* sim_spec_asSite(sim_spec_element_t* element);

// Safely cast an element as SIM_sFrame, or return NULL if the element is not an SIM_sFrame.
SIM_API SIM_sFrame* sim_spec_asFrame(sim_spec_element_t* element);

// Safely cast an element as sim_spec_actuator_t, or return NULL if the element is not an sim_spec_actuator_t.
SIM_API sim_spec_actuator_t* sim_spec_asActuator(sim_spec_element_t* element);

// Safely cast an element as SIM_sSensor, or return NULL if the element is not an SIM_sSensor.
SIM_API SIM_sSensor* sim_spec_asSensor(sim_spec_element_t* element);

// Safely cast an element as SIM_sFlex, or return NULL if the element is not an SIM_sFlex.
SIM_API SIM_sFlex* sim_spec_asFlex(sim_spec_element_t* element);

// Safely cast an element as SIM_sPair, or return NULL if the element is not an SIM_sPair.
SIM_API SIM_sPair* sim_spec_asPair(sim_spec_element_t* element);

// Safely cast an element as SIM_sEquality, or return NULL if the element is not an SIM_sEquality.
SIM_API SIM_sEquality* sim_spec_asEquality(sim_spec_element_t* element);

// Safely cast an element as SIM_sExclude, or return NULL if the element is not an SIM_sExclude.
SIM_API SIM_sExclude* sim_spec_asExclude(sim_spec_element_t* element);

// Safely cast an element as SIM_sTendon, or return NULL if the element is not an SIM_sTendon.
SIM_API SIM_sTendon* sim_spec_asTendon(sim_spec_element_t* element);

// Safely cast an element as SIM_sNumeric, or return NULL if the element is not an SIM_sNumeric.
SIM_API SIM_sNumeric* sim_spec_asNumeric(sim_spec_element_t* element);

// Safely cast an element as SIM_sText, or return NULL if the element is not an SIM_sText.
SIM_API SIM_sText* sim_spec_asText(sim_spec_element_t* element);

// Safely cast an element as SIM_sTuple, or return NULL if the element is not an SIM_sTuple.
SIM_API SIM_sTuple* sim_spec_asTuple(sim_spec_element_t* element);

// Safely cast an element as SIM_sKey, or return NULL if the element is not an SIM_sKey.
SIM_API SIM_sKey* sim_spec_asKey(sim_spec_element_t* element);

// Safely cast an element as SIM_sMesh, or return NULL if the element is not an SIM_sMesh.
SIM_API SIM_sMesh* sim_spec_asMesh(sim_spec_element_t* element);

// Safely cast an element as SIM_sHField, or return NULL if the element is not an SIM_sHField.
SIM_API SIM_sHField* sim_spec_asHField(sim_spec_element_t* element);

// Safely cast an element as SIM_sSkin, or return NULL if the element is not an SIM_sSkin.
SIM_API SIM_sSkin* sim_spec_asSkin(sim_spec_element_t* element);

// Safely cast an element as SIM_sPlugin, or return NULL if the element is not an SIM_sPlugin.
SIM_API SIM_sPlugin* sim_spec_asPlugin(sim_spec_element_t* element);

#ifdef __cplusplus
}
#endif

#endif  // SIMCORE_INCLUDE_SIMCORE_CORE_API_H_

