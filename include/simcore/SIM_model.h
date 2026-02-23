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

#ifndef SIMCORE_SIM_MODEL_H_
#define SIMCORE_SIM_MODEL_H_

#include <stddef.h>
#include <stdint.h>


#include <simcore/SIM_tnum.h>

// global constants
#define SIM_PI            3.14159265358979323846
#define SIM_MAXVAL        1E+10     // maximum value in qpos, qvel, qacc
#define SIM_MINMU         1E-5      // minimum friction coefficient
#define SIM_MINIMP        0.0001    // minimum constraint impedance
#define SIM_MAXIMP        0.9999    // maximum constraint impedance
#define SIM_MAXCONPAIR    50        // maximum number of contacts per geom pair
#define SIM_MAXTREEDEPTH  50        // maximum bounding volume hierarchy depth
#define SIM_MAXFLEXNODES  27        // maximum number of flex nodes
#define SIM_MINAWAKE      10        // minimum number of timesteps before sleeping
#define SIM_NGROUP        6         // number of geom/material grouping slots


//---------------------------------- sizes ---------------------------------------------------------

#define SIM_NEQDATA       11        // number of eq_data fields
#define SIM_NDYN          10        // number of actuator dynamics parameters
#define SIM_NGAIN         10        // number of actuator gain parameters
#define SIM_NBIAS         10        // number of actuator bias parameters
#define SIM_NFLUID        12        // number of fluid interaction parameters
#define SIM_NREF          2         // number of solver reference parameters
#define SIM_NIMP          5         // number of solver impedance parameters
#define SIM_NSENS         3         // number of sensor parameters
#define SIM_NSOLVER       200       // size of one sim_data_t.solver array
#define SIM_NISLAND       20        // number of sim_data_t.solver arrays


//---------------------------------- enum types (simt) ----------------------------------------------

typedef enum SIM_tDisableBit_ {     // disable default feature bitflags
  SIM_DSBL_CONSTRAINT   = 1<<0,     // entire constraint solver
  SIM_DSBL_EQUALITY     = 1<<1,     // equality constraints
  SIM_DSBL_FRICTIONLOSS = 1<<2,     // joint and tendon frictionloss constraints
  SIM_DSBL_LIMIT        = 1<<3,     // joint and tendon limit constraints
  SIM_DSBL_CONTACT      = 1<<4,     // contact constraints
  SIM_DSBL_SPRING       = 1<<5,     // passive spring forces
  SIM_DSBL_DAMPER       = 1<<6,     // passive damping forces
  SIM_DSBL_GRAVITY      = 1<<7,     // gravitational forces
  SIM_DSBL_CLAMPCTRL    = 1<<8,     // clamp control to specified range
  SIM_DSBL_WARMSTART    = 1<<9,     // warmstart constraint solver
  SIM_DSBL_FILTERPARENT = 1<<10,    // remove collisions with parent body
  SIM_DSBL_ACTUATION    = 1<<11,    // apply actuation forces
  SIM_DSBL_REFSAFE      = 1<<12,    // integrator safety: make ref[0]>=2*timestep
  SIM_DSBL_SENSOR       = 1<<13,    // sensors
  SIM_DSBL_MIDPHASE     = 1<<14,    // mid-phase collision filtering
  SIM_DSBL_EULERDAMP    = 1<<15,    // implicit integration of joint damping in Euler integrator
  SIM_DSBL_AUTORESET    = 1<<16,    // automatic reset when numerical issues are detected
  SIM_DSBL_NATIVECCD    = 1<<17,    // native convex collision detection
  SIM_DSBL_ISLAND       = 1<<18,    // constraint island discovery

  SIM_NDISABLE          = 19        // number of disable flags
} SIM_tDisableBit;


typedef enum SIM_tEnableBit_ {      // enable optional feature bitflags
  SIM_ENBL_OVERRIDE     = 1<<0,     // override contact parameters
  SIM_ENBL_ENERGY       = 1<<1,     // energy computation
  SIM_ENBL_FWDINV       = 1<<2,     // record solver statistics
  SIM_ENBL_INVDISCRETE  = 1<<3,     // discrete-time inverse dynamics
                                  // experimental features:
  SIM_ENBL_MULTICCD     = 1<<4,     // multi-point convex collision detection
  SIM_ENBL_SLEEP        = 1<<5,     // sleeping

  SIM_NENABLE           = 6         // number of enable flags
} SIM_tEnableBit;


typedef enum SIM_tJoint_ {          // type of degree of freedom
  SIM_JNT_FREE          = 0,        // global position and orientation (quat)       (7)
  SIM_JNT_BALL,                     // orientation (quat) relative to parent        (4)
  SIM_JNT_SLIDE,                    // sliding distance along body-fixed axis       (1)
  SIM_JNT_HINGE                     // rotation angle (rad) around body-fixed axis  (1)
} SIM_tJoint;


typedef enum SIM_tGeom_ {           // type of geometric shape
  // regular geom types
  SIM_GEOM_PLANE        = 0,        // plane
  SIM_GEOM_HFIELD,                  // height field
  SIM_GEOM_SPHERE,                  // sphere
  SIM_GEOM_CAPSULE,                 // capsule
  SIM_GEOM_ELLIPSOID,               // ellipsoid
  SIM_GEOM_CYLINDER,                // cylinder
  SIM_GEOM_BOX,                     // box
  SIM_GEOM_MESH,                    // mesh
  SIM_GEOM_SDF,                     // signed distance field

  SIM_NGEOMTYPES,                   // number of regular geom types

  // rendering-only geom types: not used in sim_model_t, not counted in SIM_NGEOMTYPES
  SIM_GEOM_ARROW        = 100,      // arrow
  SIM_GEOM_ARROW1,                  // arrow without wedges
  SIM_GEOM_ARROW2,                  // arrow in both directions
  SIM_GEOM_LINE,                    // line
  SIM_GEOM_LINEBOX,                 // box with line edges
  SIM_GEOM_FLEX,                    // flex
  SIM_GEOM_SKIN,                    // skin
  SIM_GEOM_LABEL,                   // text label
  SIM_GEOM_TRIANGLE,                // triangle

  SIM_GEOM_NONE         = 1001      // missing geom type
} SIM_tGeom;


typedef enum SIM_tProjection_ {     // type of camera projection
  SIM_PROJ_PERSPECTIVE  = 0,        // perspective
  SIM_PROJ_ORTHOGRAPHIC             // orthographic
} SIM_tProjection;


typedef enum SIM_tCamLight_ {       // tracking mode for camera and light
  SIM_CAMLIGHT_FIXED    = 0,        // pos and rot fixed in body
  SIM_CAMLIGHT_TRACK,               // pos tracks body, rot fixed in global
  SIM_CAMLIGHT_TRACKCOM,            // pos tracks subtree com, rot fixed in body
  SIM_CAMLIGHT_TARGETBODY,          // pos fixed in body, rot tracks target body
  SIM_CAMLIGHT_TARGETBODYCOM        // pos fixed in body, rot tracks target subtree com
} SIM_tCamLight;


typedef enum SIM_tLightType_ {      // type of light
  SIM_LIGHT_SPOT        = 0,        // spot
  SIM_LIGHT_DIRECTIONAL,            // directional
  SIM_LIGHT_POINT,                  // point
  SIM_LIGHT_IMAGE,                  // image-based
} SIM_tLightType;


typedef enum SIM_tTexture_ {        // type of texture
  SIM_TEXTURE_2D        = 0,        // 2d texture, suitable for planes and hfields
  SIM_TEXTURE_CUBE,                 // cube texture, suitable for all other geom types
  SIM_TEXTURE_SKYBOX                // cube texture used as skybox
} SIM_tTexture;


typedef enum SIM_tTextureRole_ {    // role of texture map in rendering
  SIM_TEXROLE_USER      = 0,        // unspecified
  SIM_TEXROLE_RGB,                  // base color (albedo)
  SIM_TEXROLE_OCCLUSION,            // ambient occlusion
  SIM_TEXROLE_ROUGHNESS,            // roughness
  SIM_TEXROLE_METALLIC,             // metallic
  SIM_TEXROLE_NORMAL,               // normal (bump) map
  SIM_TEXROLE_OPACITY,              // transperancy
  SIM_TEXROLE_EMISSIVE,             // light emission
  SIM_TEXROLE_RGBA,                 // base color, opacity
  SIM_TEXROLE_ORM,                  // occlusion, roughness, metallic
  SIM_NTEXROLE
} SIM_tTextureRole;


typedef enum SIM_tColorSpace_ {     // type of color space encoding
  SIM_COLORSPACE_AUTO   = 0,        // attempts to autodetect color space, defaults to linear
  SIM_COLORSPACE_LINEAR,            // linear color space
  SIM_COLORSPACE_SRGB               // standard RGB color space
} SIM_tColorSpace;


typedef enum SIM_tIntegrator_ {     // integrator mode
  SIM_INT_EULER         = 0,        // semi-implicit Euler
  SIM_INT_RK4,                      // 4th-order Runge Kutta
  SIM_INT_IMPLICIT,                 // implicit in velocity
  SIM_INT_IMPLICITFAST              // implicit in velocity, no rne derivative
} SIM_tIntegrator;


typedef enum SIM_tCone_ {           // type of friction cone
  SIM_CONE_PYRAMIDAL     = 0,       // pyramidal
  SIM_CONE_ELLIPTIC                 // elliptic
} SIM_tCone;


typedef enum SIM_tJacobian_ {       // type of constraint Jacobian
  SIM_JAC_DENSE          = 0,       // dense
  SIM_JAC_SPARSE,                   // sparse
  SIM_JAC_AUTO                      // dense if nv<60, sparse otherwise
} SIM_tJacobian;


typedef enum SIM_tSolver_ {         // constraint solver algorithm
  SIM_SOL_PGS            = 0,       // PGS    (dual)
  SIM_SOL_CG,                       // CG     (primal)
  SIM_SOL_NEWTON                    // Newton (primal)
} SIM_tSolver;


typedef enum SIM_tEq_ {             // type of equality constraint
  SIM_EQ_CONNECT        = 0,        // connect two bodies at a point (ball joint)
  SIM_EQ_WELD,                      // fix relative position and orientation of two bodies
  SIM_EQ_JOINT,                     // couple the values of two scalar joints with cubic
  SIM_EQ_TENDON,                    // couple the lengths of two tendons with cubic
  SIM_EQ_FLEX,                      // fix all edge lengths of a flex
  SIM_EQ_FLEXVERT,                  // fix all vertex lengths of a flex
  SIM_EQ_DISTANCE                   // unsupported, will cause an error if used
} SIM_tEq;


typedef enum SIM_tWrap_ {           // type of tendon wrap object
  SIM_WRAP_NONE         = 0,        // null object
  SIM_WRAP_JOINT,                   // constant moment arm
  SIM_WRAP_PULLEY,                  // pulley used to split tendon
  SIM_WRAP_SITE,                    // pass through site
  SIM_WRAP_SPHERE,                  // wrap around sphere
  SIM_WRAP_CYLINDER                 // wrap around (infinite) cylinder
} SIM_tWrap;


typedef enum SIM_tTrn_ {            // type of actuator transmission
  SIM_TRN_JOINT         = 0,        // force on joint
  SIM_TRN_JOINTINPARENT,            // force on joint, expressed in parent frame
  SIM_TRN_SLIDERCRANK,              // force via slider-crank linkage
  SIM_TRN_TENDON,                   // force on tendon
  SIM_TRN_SITE,                     // force on site
  SIM_TRN_BODY,                     // adhesion force on a body's geoms

  SIM_TRN_UNDEFINED     = 1000      // undefined transmission type
} SIM_tTrn;


typedef enum SIM_tDyn_ {            // type of actuator dynamics
  SIM_DYN_NONE          = 0,        // no internal dynamics; ctrl specifies force
  SIM_DYN_INTEGRATOR,               // integrator: da/dt = u
  SIM_DYN_FILTER,                   // linear filter: da/dt = (u-a) / tau
  SIM_DYN_FILTEREXACT,              // linear filter: da/dt = (u-a) / tau, with exact integration
  SIM_DYN_MUSCLE,                   // piece-wise linear filter with two time constants
  SIM_DYN_USER                      // user-defined dynamics type
} SIM_tDyn;


typedef enum SIM_tGain_ {           // type of actuator gain
  SIM_GAIN_FIXED        = 0,        // fixed gain
  SIM_GAIN_AFFINE,                  // const + kp*length + kv*velocity
  SIM_GAIN_MUSCLE,                  // muscle FLV curve computed by sim_math_muscleGain()
  SIM_GAIN_USER                     // user-defined gain type
} SIM_tGain;


typedef enum SIM_tBias_ {           // type of actuator bias
  SIM_BIAS_NONE         = 0,        // no bias
  SIM_BIAS_AFFINE,                  // const + kp*length + kv*velocity
  SIM_BIAS_MUSCLE,                  // muscle passive force computed by sim_math_muscleBias()
  SIM_BIAS_USER                     // user-defined bias type
} SIM_tBias;


typedef enum SIM_tObj_ {            // type of SimCore object
  SIM_OBJ_UNKNOWN       = 0,        // unknown object type
  SIM_OBJ_BODY,                     // body
  SIM_OBJ_XBODY,                    // body, used to access regular frame instead of i-frame
  SIM_OBJ_JOINT,                    // joint
  SIM_OBJ_DOF,                      // dof
  SIM_OBJ_GEOM,                     // geom
  SIM_OBJ_SITE,                     // site
  SIM_OBJ_CAMERA,                   // camera
  SIM_OBJ_LIGHT,                    // light
  SIM_OBJ_FLEX,                     // flex
  SIM_OBJ_MESH,                     // mesh
  SIM_OBJ_SKIN,                     // skin
  SIM_OBJ_HFIELD,                   // heightfield
  SIM_OBJ_TEXTURE,                  // texture
  SIM_OBJ_MATERIAL,                 // material for rendering
  SIM_OBJ_PAIR,                     // geom pair to include
  SIM_OBJ_EXCLUDE,                  // body pair to exclude
  SIM_OBJ_EQUALITY,                 // equality constraint
  SIM_OBJ_TENDON,                   // tendon
  SIM_OBJ_ACTUATOR,                 // actuator
  SIM_OBJ_SENSOR,                   // sensor
  SIM_OBJ_NUMERIC,                  // numeric
  SIM_OBJ_TEXT,                     // text
  SIM_OBJ_TUPLE,                    // tuple
  SIM_OBJ_KEY,                      // keyframe
  SIM_OBJ_PLUGIN,                   // plugin instance

  SIM_NOBJECT,                      // number of object types

  // meta elements, do not appear in sim_model_t
  SIM_OBJ_FRAME         = 100,      // frame
  SIM_OBJ_DEFAULT,                  // default
  SIM_OBJ_MODEL                     // entire model
} sim_obj_t;


typedef enum SIM_tSensor_ {         // type of sensor
  // common robotic sensors, attached to a site
  SIM_SENS_TOUCH        = 0,        // scalar contact normal forces summed over sensor zone
  SIM_SENS_ACCELEROMETER,           // 3D linear acceleration, in local frame
  SIM_SENS_VELOCIMETER,             // 3D linear velocity, in local frame
  SIM_SENS_GYRO,                    // 3D angular velocity, in local frame
  SIM_SENS_FORCE,                   // 3D force between site's body and its parent body
  SIM_SENS_TORQUE,                  // 3D torque between site's body and its parent body
  SIM_SENS_MAGNETOMETER,            // 3D magnetometer
  SIM_SENS_RANGEFINDER,             // scalar distance to nearest geom along z-axis
  SIM_SENS_CAMPROJECTION,           // pixel coordinates of a site in the camera image

  // sensors related to scalar joints, tendons, actuators
  SIM_SENS_JOINTPOS,                // scalar joint position (hinge and slide only)
  SIM_SENS_JOINTVEL,                // scalar joint velocity (hinge and slide only)
  SIM_SENS_TENDONPOS,               // scalar tendon position
  SIM_SENS_TENDONVEL,               // scalar tendon velocity
  SIM_SENS_ACTUATORPOS,             // scalar actuator position
  SIM_SENS_ACTUATORVEL,             // scalar actuator velocity
  SIM_SENS_ACTUATORFRC,             // scalar actuator force
  SIM_SENS_JOINTACTFRC,             // scalar actuator force, measured at the joint
  SIM_SENS_TENDONACTFRC,            // scalar actuator force, measured at the tendon

  // sensors related to ball joints
  SIM_SENS_BALLQUAT,                // 4D ball joint quaternion
  SIM_SENS_BALLANGVEL,              // 3D ball joint angular velocity

  // joint and tendon limit sensors, in constraint space
  SIM_SENS_JOINTLIMITPOS,           // joint limit distance-margin
  SIM_SENS_JOINTLIMITVEL,           // joint limit velocity
  SIM_SENS_JOINTLIMITFRC,           // joint limit force
  SIM_SENS_TENDONLIMITPOS,          // tendon limit distance-margin
  SIM_SENS_TENDONLIMITVEL,          // tendon limit velocity
  SIM_SENS_TENDONLIMITFRC,          // tendon limit force

  // sensors attached to an object with spatial frame: (x)body, geom, site, camera
  SIM_SENS_FRAMEPOS,                // 3D position
  SIM_SENS_FRAMEQUAT,               // 4D unit quaternion orientation
  SIM_SENS_FRAMEXAXIS,              // 3D unit vector: x-axis of object's frame
  SIM_SENS_FRAMEYAXIS,              // 3D unit vector: y-axis of object's frame
  SIM_SENS_FRAMEZAXIS,              // 3D unit vector: z-axis of object's frame
  SIM_SENS_FRAMELINVEL,             // 3D linear velocity
  SIM_SENS_FRAMEANGVEL,             // 3D angular velocity
  SIM_SENS_FRAMELINACC,             // 3D linear acceleration
  SIM_SENS_FRAMEANGACC,             // 3D angular acceleration

  // sensors related to kinematic subtrees; attached to a body (which is the subtree root)
  SIM_SENS_SUBTREECOM,              // 3D center of mass of subtree
  SIM_SENS_SUBTREELINVEL,           // 3D linear velocity of subtree
  SIM_SENS_SUBTREEANGMOM,           // 3D angular momentum of subtree

  // sensors of geometric relationships
  SIM_SENS_INSIDESITE,              // 1 if object is inside a site, 0 otherwise
  SIM_SENS_GEOMDIST,                // signed distance between two geoms
  SIM_SENS_GEOMNORMAL,              // normal direction between two geoms
  SIM_SENS_GEOMFROMTO,              // segment between two geoms

  // sensors for reporting contacts which occurred during the simulation
  SIM_SENS_CONTACT,                 // contacts which occurred during the simulation

  // global sensors
  SIM_SENS_E_POTENTIAL,             // potential energy
  SIM_SENS_E_KINETIC,               // kinetic energy
  SIM_SENS_CLOCK,                   // simulation time

  // sensors related to SDFs
  SIM_SENS_TACTILE,                 // tactile sensor

  // plugin-controlled sensors
  SIM_SENS_PLUGIN,                  // plugin-controlled

  // user-defined sensor
  SIM_SENS_USER                     // sensor data provided by SIM_cb_sensor callback
} SIM_tSensor;


typedef enum SIM_tStage_ {          // computation stage
  SIM_STAGE_NONE        = 0,        // no computations
  SIM_STAGE_POS,                    // position-dependent computations
  SIM_STAGE_VEL,                    // velocity-dependent computations
  SIM_STAGE_ACC                     // acceleration/force-dependent computations
} SIM_tStage;


typedef enum SIM_tDataType_ {       // data type for sensors
  SIM_DATATYPE_REAL     = 0,        // real values, no constraints
  SIM_DATATYPE_POSITIVE,            // positive values; 0 or negative: inactive
  SIM_DATATYPE_AXIS,                // 3D unit vector
  SIM_DATATYPE_QUATERNION           // unit quaternion
} SIM_tDataType;


typedef enum SIM_tConDataField_ {   // data fields returned by contact sensors
  SIM_CONDATA_FOUND     = 0,        // whether a contact was found
  SIM_CONDATA_FORCE,                // contact force
  SIM_CONDATA_TORQUE,               // contact torque
  SIM_CONDATA_DIST,                 // contact penetration distance
  SIM_CONDATA_POS,                  // contact position
  SIM_CONDATA_NORMAL,               // contact frame normal
  SIM_CONDATA_TANGENT,              // contact frame first tangent

  SIM_NCONDATA                      // number of contact sensor data fields
} SIM_tConDataField;


typedef enum SIM_tRayDataField_ {   // data fields returned by rangefinder sensors
  SIM_RAYDATA_DIST     = 0,         // distance from ray origin to nearest surface
  SIM_RAYDATA_DIR,                  // normalized ray direction
  SIM_RAYDATA_ORIGIN,               // ray origin
  SIM_RAYDATA_POINT,                // point at which ray intersects nearest surface
  SIM_RAYDATA_NORMAL,               // surface normal at intersection point
  SIM_RAYDATA_DEPTH,                // depth along z-axis

  SIM_NRAYDATA                      // number of rangefinder sensor data fields
} SIM_tRayDataField;


typedef enum SIM_tCamOutBit_ {      // camera output type bitflags
  SIM_CAMOUT_RGB        = 1<<0,     // RGB image
  SIM_CAMOUT_DEPTH      = 1<<1,     // depth image (distance from camera plane)
  SIM_CAMOUT_DIST       = 1<<2,     // distance image (distance from camera origin)
  SIM_CAMOUT_NORMAL     = 1<<3,     // normal image
  SIM_CAMOUT_SEG        = 1<<4,     // segmentation image

  SIM_NCAMOUT           = 5         // number of camera output types
} SIM_tCamOutBit;


typedef enum SIM_tSameFrame_ {      // frame alignment of bodies with their children
  SIM_SAMEFRAME_NONE    = 0,        // no alignment
  SIM_SAMEFRAME_BODY,               // frame is same as body frame
  SIM_SAMEFRAME_INERTIA,            // frame is same as inertial frame
  SIM_SAMEFRAME_BODYROT,            // frame orientation is same as body orientation
  SIM_SAMEFRAME_INERTIAROT          // frame orientation is same as inertia orientation
} SIM_tSameFrame;


typedef enum SIM_tSleepPolicy_ {    // per-tree sleep policy
  SIM_SLEEP_AUTO        = 0,        // compiler chooses sleep policy
  SIM_SLEEP_AUTO_NEVER,             // compiler sleep policy: never
  SIM_SLEEP_AUTO_ALLOWED,           // compiler sleep policy: allowed
  SIM_SLEEP_NEVER,                  // user sleep policy: never
  SIM_SLEEP_ALLOWED,                // user sleep policy: allowed
  SIM_SLEEP_INIT,                   // user sleep policy: initialized asleep
} SIM_tSleepPolicy;


typedef enum SIM_tLRMode_ {         // mode for actuator length range computation
  SIM_LRMODE_NONE       = 0,        // do not process any actuators
  SIM_LRMODE_MUSCLE,                // process muscle actuators
  SIM_LRMODE_MUSCLEUSER,            // process muscle and user actuators
  SIM_LRMODE_ALL                    // process all actuators
} SIM_tLRMode;


typedef enum SIM_tFlexSelf_ {       // mode for flex selfcollide
  SIM_FLEXSELF_NONE     = 0,        // no self-collisions
  SIM_FLEXSELF_NARROW,              // skip midphase, go directly to narrowphase
  SIM_FLEXSELF_BVH,                 // use BVH in midphase (if midphase enabled)
  SIM_FLEXSELF_SAP,                 // use SAP in midphase
  SIM_FLEXSELF_AUTO                 // choose between BVH and SAP automatically
} SIM_tFlexSelf;


typedef enum SIM_tSDFType_ {        // signed distance function (SDF) type
  SIM_SDFTYPE_SINGLE    = 0,        // single SDF
  SIM_SDFTYPE_INTERSECTION,         // max(A, B)
  SIM_SDFTYPE_MIDSURFACE,           // A - B
  SIM_SDFTYPE_COLLISION,            // A + B + abs(max(A, B))
} SIM_tSDFType;


//---------------------------------- SIM_LROpt -------------------------------------------------------

struct SIM_LROpt_ {                 // options for sim_setLengthRange()
  // flags
  int mode;                       // which actuators to process (SIM_tLRMode)
  int useexisting;                // use existing length range if available
  int uselimit;                   // use joint and tendon limits if available

  // algorithm parameters
  sim_scalar_t accel;                   // target acceleration used to compute force
  sim_scalar_t maxforce;                // maximum force; 0: no limit
  sim_scalar_t timeconst;               // time constant for velocity reduction; min 0.01
  sim_scalar_t timestep;                // simulation timestep; 0: use SIM_Option.timestep
  sim_scalar_t inttotal;                // total simulation time interval
  sim_scalar_t interval;                // evaluation time interval (at the end)
  sim_scalar_t tolrange;                // convergence tolerance (relative to range)
};
typedef struct SIM_LROpt_ SIM_LROpt;

//---------------------------------- SIM_Cache -------------------------------------------------------

struct SIM_Cache_ {                 // asset cache used by the compiler
  void* impl_;                    // internal pointer to cache
};
typedef struct SIM_Cache_ SIM_Cache;

//---------------------------------- SIM_VFS ---------------------------------------------------------

struct SIM_VFS_ {                   // virtual file system for loading from memory
  void* impl_;                    // internal pointer to VFS memory
};
typedef struct SIM_VFS_ SIM_VFS;

//---------------------------------- SIM_Option ------------------------------------------------------

struct SIM_Option_ {                // physics options
  // timing parameters
  sim_scalar_t timestep;                // timestep

  // solver parameters
  sim_scalar_t impratio;                // ratio of friction-to-normal contact impedance
  sim_scalar_t tolerance;               // main solver tolerance
  sim_scalar_t ls_tolerance;            // CG/Newton linesearch tolerance
  sim_scalar_t noslip_tolerance;        // noslip solver tolerance
  sim_scalar_t ccd_tolerance;           // convex collision solver tolerance

  // sleep settings
  sim_scalar_t sleep_tolerance;         // sleep velocity tolerance

  // physical constants
  sim_scalar_t gravity[3];              // gravitational acceleration
  sim_scalar_t wind[3];                 // wind (for lift, drag and viscosity)
  sim_scalar_t magnetic[3];             // global magnetic flux
  sim_scalar_t density;                 // density of medium
  sim_scalar_t viscosity;               // viscosity of medium

  // override contact solver parameters (if enabled)
  sim_scalar_t o_margin;                // margin
  sim_scalar_t o_solref[SIM_NREF];        // solref
  sim_scalar_t o_solimp[SIM_NIMP];        // solimp
  sim_scalar_t o_friction[5];           // friction

  // discrete settings
  int integrator;                 // integration mode (SIM_tIntegrator)
  int cone;                       // type of friction cone (SIM_tCone)
  int jacobian;                   // type of Jacobian (SIM_tJacobian)
  int solver;                     // solver algorithm (SIM_tSolver)
  int iterations;                 // maximum number of main solver iterations
  int ls_iterations;              // maximum number of CG/Newton linesearch iterations
  int noslip_iterations;          // maximum number of noslip solver iterations
  int ccd_iterations;             // maximum number of convex collision solver iterations
  int disableflags;               // bit flags for disabling standard features
  int enableflags;                // bit flags for enabling optional features
  int disableactuator;            // bit flags for disabling actuators by group id

  // sdf collision settings
  int sdf_initpoints;             // number of starting points for gradient descent
  int sdf_iterations;             // max number of iterations for gradient descent
};
typedef struct SIM_Option_ SIM_Option;


//---------------------------------- SIM_Statistic ---------------------------------------------------

struct SIM_Statistic_ {             // model statistics (in qpos0)
  sim_scalar_t meaninertia;             // mean diagonal inertia
  sim_scalar_t meanmass;                // mean body mass
  sim_scalar_t meansize;                // mean body size
  sim_scalar_t extent;                  // spatial extent
  sim_scalar_t center[3];               // center of model
};
typedef struct SIM_Statistic_ SIM_Statistic;


//---------------------------------- sim_model_t -------------------------------------------------------

struct SIM_Model_ {
  // ------------------------------- sizes

  // sizes needed at sim_model_t construction
  sim_size_t nq;                     // number of generalized coordinates = dim(qpos)
  sim_size_t nv;                     // number of degrees of freedom = dim(qvel)
  sim_size_t nu;                     // number of actuators/controls = dim(ctrl)
  sim_size_t na;                     // number of activation states = dim(act)
  sim_size_t nbody;                  // number of bodies
  sim_size_t nbvh;                   // number of total bounding volumes in all bodies
  sim_size_t nbvhstatic;             // number of static bounding volumes (aabb stored in sim_model_t)
  sim_size_t nbvhdynamic;            // number of dynamic bounding volumes (aabb stored in sim_data_t)
  sim_size_t noct;                   // number of total octree cells in all meshes
  sim_size_t njnt;                   // number of joints
  sim_size_t ntree;                  // number of kinematic trees under world body
  sim_size_t nM;                     // number of non-zeros in sparse inertia matrix
  sim_size_t nB;                     // number of non-zeros in sparse body-dof matrix
  sim_size_t nC;                     // number of non-zeros in sparse reduced dof-dof matrix
  sim_size_t nD;                     // number of non-zeros in sparse dof-dof matrix
  sim_size_t ngeom;                  // number of geoms
  sim_size_t nsite;                  // number of sites
  sim_size_t ncam;                   // number of cameras
  sim_size_t nlight;                 // number of lights
  sim_size_t nflex;                  // number of flexes
  sim_size_t nflexnode;              // number of dofs in all flexes
  sim_size_t nflexvert;              // number of vertices in all flexes
  sim_size_t nflexedge;              // number of edges in all flexes
  sim_size_t nflexelem;              // number of elements in all flexes
  sim_size_t nflexelemdata;          // number of element vertex ids in all flexes
  sim_size_t nflexelemedge;          // number of element edge ids in all flexes
  sim_size_t nflexshelldata;         // number of shell fragment vertex ids in all flexes
  sim_size_t nflexevpair;            // number of element-vertex pairs in all flexes
  sim_size_t nflextexcoord;          // number of vertices with texture coordinates
  sim_size_t nJfe;                   // number of non-zeros in sparse flexedge Jacobian matrix
  sim_size_t nJfv;                   // number of non-zeros in sparse flexvert Jacobian matrix
  sim_size_t nmesh;                  // number of meshes
  sim_size_t nmeshvert;              // number of vertices in all meshes
  sim_size_t nmeshnormal;            // number of normals in all meshes
  sim_size_t nmeshtexcoord;          // number of texcoords in all meshes
  sim_size_t nmeshface;              // number of triangular faces in all meshes
  sim_size_t nmeshgraph;             // number of ints in mesh auxiliary data
  sim_size_t nmeshpoly;              // number of polygons in all meshes
  sim_size_t nmeshpolyvert;          // number of vertices in all polygons
  sim_size_t nmeshpolymap;           // number of polygons in vertex map
  sim_size_t nskin;                  // number of skins
  sim_size_t nskinvert;              // number of vertices in all skins
  sim_size_t nskintexvert;           // number of vertices with texcoords in all skins
  sim_size_t nskinface;              // number of triangular faces in all skins
  sim_size_t nskinbone;              // number of bones in all skins
  sim_size_t nskinbonevert;          // number of vertices in all skin bones
  sim_size_t nhfield;                // number of heightfields
  sim_size_t nhfielddata;            // number of data points in all heightfields
  sim_size_t ntex;                   // number of textures
  sim_size_t ntexdata;               // number of bytes in texture rgb data
  sim_size_t nmat;                   // number of materials
  sim_size_t npair;                  // number of predefined geom pairs
  sim_size_t nexclude;               // number of excluded geom pairs
  sim_size_t neq;                    // number of equality constraints
  sim_size_t ntendon;                // number of tendons
  sim_size_t nwrap;                  // number of wrap objects in all tendon paths
  sim_size_t nsensor;                // number of sensors
  sim_size_t nnumeric;               // number of numeric custom fields
  sim_size_t nnumericdata;           // number of SIM_tNums in all numeric fields
  sim_size_t ntext;                  // number of text custom fields
  sim_size_t ntextdata;              // number of SIM_tBytes in all text fields
  sim_size_t ntuple;                 // number of tuple custom fields
  sim_size_t ntupledata;             // number of objects in all tuple fields
  sim_size_t nkey;                   // number of keyframes
  sim_size_t nmocap;                 // number of mocap bodies
  sim_size_t nplugin;                // number of plugin instances
  sim_size_t npluginattr;            // number of chars in all plugin config attributes
  sim_size_t nuser_body;             // number of SIM_tNums in body_user
  sim_size_t nuser_jnt;              // number of SIM_tNums in jnt_user
  sim_size_t nuser_geom;             // number of SIM_tNums in geom_user
  sim_size_t nuser_site;             // number of SIM_tNums in site_user
  sim_size_t nuser_cam;              // number of SIM_tNums in cam_user
  sim_size_t nuser_tendon;           // number of SIM_tNums in tendon_user
  sim_size_t nuser_actuator;         // number of SIM_tNums in actuator_user
  sim_size_t nuser_sensor;           // number of SIM_tNums in sensor_user
  sim_size_t nnames;                 // number of chars in all names
  sim_size_t npaths;                 // number of chars in all paths

  // sizes set after sim_model_t construction
  sim_size_t nnames_map;             // number of slots in the names hash map
  sim_size_t nJmom;                  // number of non-zeros in sparse actuator_moment matrix
  sim_size_t nJten;                  // number of non-zeros in sparse ten_J matrix
  sim_size_t ngravcomp;              // number of bodies with nonzero gravcomp
  sim_size_t nemax;                  // number of potential equality-constraint rows
  sim_size_t njmax;                  // number of available rows in constraint Jacobian (legacy)
  sim_size_t nconmax;                // number of potential contacts in contact list (legacy)
  sim_size_t nuserdata;              // number of SIM_tNums reserved for the user
  sim_size_t nsensordata;            // number of SIM_tNums in sensor data vector
  sim_size_t npluginstate;           // number of SIM_tNums in plugin state vector
  sim_size_t nhistory;               // number of SIM_tNums in history buffer

  // buffer sizes
  sim_size_t narena;                 // number of bytes in the sim_data_t arena (inclusive of stack)
  sim_size_t nbuffer;                // number of bytes in buffer

  // ------------------------------- options and statistics

  SIM_Option opt;                   // physics options
  SIM_Statistic stat;               // model statistics

  // ------------------------------- buffers

  // main buffer
  void*     buffer;               // main buffer; all pointers point in it    (nbuffer)

  // default generalized coordinates
  sim_scalar_t*   qpos0;                // qpos values at default pose              (nq x 1)
  sim_scalar_t*   qpos_spring;          // reference pose for springs               (nq x 1)

  // bodies
  int*      body_parentid;        // id of body's parent                      (nbody x 1)
  int*      body_rootid;          // ancestor that is direct child of world   (nbody x 1)
  int*      body_weldid;          // top ancestor with no dofs to this body   (nbody x 1)
  int*      body_mocapid;         // id of mocap data; -1: none               (nbody x 1)
  int*      body_jntnum;          // number of joints for this body           (nbody x 1)
  int*      body_jntadr;          // start addr of joints; -1: no joints      (nbody x 1)
  int*      body_dofnum;          // number of motion degrees of freedom      (nbody x 1)
  int*      body_dofadr;          // start addr of dofs; -1: no dofs          (nbody x 1)
  int*      body_treeid;          // id of body's kinematic tree; -1: static  (nbody x 1)
  int*      body_geomnum;         // number of geoms                          (nbody x 1)
  int*      body_geomadr;         // start addr of geoms; -1: no geoms        (nbody x 1)
  sim_byte_t*  body_simple;          // 1: diag M; 2: diag M, sliders only       (nbody x 1)
  sim_byte_t*  body_sameframe;       // same frame as inertia (SIM_tSameframe)     (nbody x 1)
  sim_scalar_t*   body_pos;             // position offset rel. to parent body      (nbody x 3)
  sim_scalar_t*   body_quat;            // orientation offset rel. to parent body   (nbody x 4)
  sim_scalar_t*   body_ipos;            // local position of center of mass         (nbody x 3)
  sim_scalar_t*   body_iquat;           // local orientation of inertia ellipsoid   (nbody x 4)
  sim_scalar_t*   body_mass;            // mass                                     (nbody x 1)
  sim_scalar_t*   body_subtreemass;     // mass of subtree starting at this body    (nbody x 1)
  sim_scalar_t*   body_inertia;         // diagonal inertia in ipos/iquat frame     (nbody x 3)
  sim_scalar_t*   body_invweight0;      // mean inv inert in qpos0 (trn, rot)       (nbody x 2)
  sim_scalar_t*   body_gravcomp;        // antigravity force, units of body weight  (nbody x 1)
  sim_scalar_t*   body_margin;          // MAX over all geom margins                (nbody x 1)
  sim_scalar_t*   body_user;            // user data                                (nbody x nuser_body)
  int*      body_plugin;          // plugin instance id; -1: not in use       (nbody x 1)
  int*      body_contype;         // OR over all geom contypes                (nbody x 1)
  int*      body_conaffinity;     // OR over all geom conaffinities           (nbody x 1)
  int*      body_bvhadr;          // address of bvh root                      (nbody x 1)
  int*      body_bvhnum;          // number of bounding volumes               (nbody x 1)

  // bounding volume hierarchy
  int*      bvh_depth;            // depth in the bounding volume hierarchy   (nbvh x 1)
  int*      bvh_child;            // left and right children in tree          (nbvh x 2)
  int*      bvh_nodeid;           // geom or elem id of node; -1: non-leaf    (nbvh x 1)
  sim_scalar_t*   bvh_aabb;             // local bounding box (center, size)        (nbvhstatic x 6)

  // octree spatial partitioning
  int*      oct_depth;            // depth in the octree                      (noct x 1)
  int*      oct_child;            // children of octree node                  (noct x 8)
  sim_scalar_t*   oct_aabb;             // octree node bounding box (center, size)  (noct x 6)
  sim_scalar_t*   oct_coeff;            // octree interpolation coefficients        (noct x 8)

  // joints
  int*      jnt_type;             // type of joint (SIM_tJoint)                 (njnt x 1)
  int*      jnt_qposadr;          // start addr in 'qpos' for joint's data    (njnt x 1)
  int*      jnt_dofadr;           // start addr in 'qvel' for joint's data    (njnt x 1)
  int*      jnt_bodyid;           // id of joint's body                       (njnt x 1)
  int*      jnt_group;            // group for visibility                     (njnt x 1)
  sim_byte_t*  jnt_limited;          // does joint have limits                   (njnt x 1)
  sim_byte_t*  jnt_actfrclimited;    // does joint have actuator force limits    (njnt x 1)
  sim_byte_t*  jnt_actgravcomp;      // is gravcomp force applied via actuators  (njnt x 1)
  sim_scalar_t*   jnt_solref;           // constraint solver reference: limit       (njnt x SIM_NREF)
  sim_scalar_t*   jnt_solimp;           // constraint solver impedance: limit       (njnt x SIM_NIMP)
  sim_scalar_t*   jnt_pos;              // local anchor position                    (njnt x 3)
  sim_scalar_t*   jnt_axis;             // local joint axis                         (njnt x 3)
  sim_scalar_t*   jnt_stiffness;        // stiffness coefficient                    (njnt x 1)
  sim_scalar_t*   jnt_range;            // joint limits                             (njnt x 2)
  sim_scalar_t*   jnt_actfrcrange;      // range of total actuator force            (njnt x 2)
  sim_scalar_t*   jnt_margin;           // min distance for limit detection         (njnt x 1)
  sim_scalar_t*   jnt_user;             // user data                                (njnt x nuser_jnt)

  // dofs
  int*      dof_bodyid;           // id of dof's body                         (nv x 1)
  int*      dof_jntid;            // id of dof's joint                        (nv x 1)
  int*      dof_parentid;         // id of dof's parent; -1: none             (nv x 1)
  int*      dof_treeid;           // id of dof's kinematic tree               (nv x 1)
  int*      dof_Madr;             // dof address in M-diagonal                (nv x 1)
  int*      dof_simplenum;        // number of consecutive simple dofs        (nv x 1)
  sim_scalar_t*   dof_solref;           // constraint solver reference:frictionloss (nv x SIM_NREF)
  sim_scalar_t*   dof_solimp;           // constraint solver impedance:frictionloss (nv x SIM_NIMP)
  sim_scalar_t*   dof_frictionloss;     // dof friction loss                        (nv x 1)
  sim_scalar_t*   dof_armature;         // dof armature inertia/mass                (nv x 1)
  sim_scalar_t*   dof_damping;          // damping coefficient                      (nv x 1)
  sim_scalar_t*   dof_invweight0;       // diag. inverse inertia in qpos0           (nv x 1)
  sim_scalar_t*   dof_M0;               // diag. inertia in qpos0                   (nv x 1)
  sim_scalar_t*   dof_length;           // linear: 1; angular: approx. length scale (nv x 1)

  // trees
  int*      tree_bodyadr;         // start addr of bodies                     (ntree x 1)
  int*      tree_bodynum;         // number of bodies in tree                 (ntree x 1)
  int*      tree_dofadr;          // start addr of dofs                       (ntree x 1)
  int*      tree_dofnum;          // number of dofs in tree                   (ntree x 1)
  int*      tree_sleep_policy;    // sleep policy (SIM_tSleepPolicy)            (ntree x 1)

  // geoms
  int*      geom_type;            // geometric type (SIM_tGeom)                 (ngeom x 1)
  int*      geom_contype;         // geom contact type                        (ngeom x 1)
  int*      geom_conaffinity;     // geom contact affinity                    (ngeom x 1)
  int*      geom_condim;          // contact dimensionality (1, 3, 4, 6)      (ngeom x 1)
  int*      geom_bodyid;          // id of geom's body                        (ngeom x 1)
  int*      geom_dataid;          // id of geom's mesh/hfield; -1: none       (ngeom x 1)
  int*      geom_matid;           // material id for rendering; -1: none      (ngeom x 1)
  int*      geom_group;           // group for visibility                     (ngeom x 1)
  int*      geom_priority;        // geom contact priority                    (ngeom x 1)
  int*      geom_plugin;          // plugin instance id; -1: not in use       (ngeom x 1)
  sim_byte_t*  geom_sameframe;       // same frame as body (SIM_tSameframe)        (ngeom x 1)
  sim_scalar_t*   geom_solmix;          // mixing coef for solref/imp in geom pair  (ngeom x 1)
  sim_scalar_t*   geom_solref;          // constraint solver reference: contact     (ngeom x SIM_NREF)
  sim_scalar_t*   geom_solimp;          // constraint solver impedance: contact     (ngeom x SIM_NIMP)
  sim_scalar_t*   geom_size;            // geom-specific size parameters            (ngeom x 3)
  sim_scalar_t*   geom_aabb;            // bounding box, (center, size)             (ngeom x 6)
  sim_scalar_t*   geom_rbound;          // radius of bounding sphere                (ngeom x 1)
  sim_scalar_t*   geom_pos;             // local position offset rel. to body       (ngeom x 3)
  sim_scalar_t*   geom_quat;            // local orientation offset rel. to body    (ngeom x 4)
  sim_scalar_t*   geom_friction;        // friction for (slide, spin, roll)         (ngeom x 3)
  sim_scalar_t*   geom_margin;          // detect contact if dist<margin            (ngeom x 1)
  sim_scalar_t*   geom_gap;             // include in solver if dist<margin-gap     (ngeom x 1)
  sim_scalar_t*   geom_fluid;           // fluid interaction parameters             (ngeom x SIM_NFLUID)
  sim_scalar_t*   geom_user;            // user data                                (ngeom x nuser_geom)
  float*    geom_rgba;            // rgba when material is omitted            (ngeom x 4)

  // sites
  int*      site_type;            // geom type for rendering (SIM_tGeom)        (nsite x 1)
  int*      site_bodyid;          // id of site's body                        (nsite x 1)
  int*      site_matid;           // material id for rendering; -1: none      (nsite x 1)
  int*      site_group;           // group for visibility                     (nsite x 1)
  sim_byte_t*  site_sameframe;       // same frame as body (SIM_tSameframe)        (nsite x 1)
  sim_scalar_t*   site_size;            // geom size for rendering                  (nsite x 3)
  sim_scalar_t*   site_pos;             // local position offset rel. to body       (nsite x 3)
  sim_scalar_t*   site_quat;            // local orientation offset rel. to body    (nsite x 4)
  sim_scalar_t*   site_user;            // user data                                (nsite x nuser_site)
  float*    site_rgba;            // rgba when material is omitted            (nsite x 4)

  // cameras
  int*      cam_mode;             // camera tracking mode (SIM_tCamLight)       (ncam x 1)
  int*      cam_bodyid;           // id of camera's body                      (ncam x 1)
  int*      cam_targetbodyid;     // id of targeted body; -1: none            (ncam x 1)
  sim_scalar_t*   cam_pos;              // position rel. to body frame              (ncam x 3)
  sim_scalar_t*   cam_quat;             // orientation rel. to body frame           (ncam x 4)
  sim_scalar_t*   cam_poscom0;          // global position rel. to sub-com in qpos0 (ncam x 3)
  sim_scalar_t*   cam_pos0;             // global position rel. to body in qpos0    (ncam x 3)
  sim_scalar_t*   cam_mat0;             // global orientation in qpos0              (ncam x 9)
  int*      cam_projection;       // projection type (SIM_tProjection)          (ncam x 1)
  sim_scalar_t*   cam_fovy;             // y field-of-view (ortho ? len : deg)      (ncam x 1)
  sim_scalar_t*   cam_ipd;              // inter-pupilary distance                  (ncam x 1)
  int*      cam_resolution;       // resolution: pixels [width, height]       (ncam x 2)
  int*      cam_output;           // output types (SIM_tCamOut bit flags)       (ncam x 1)
  float*    cam_sensorsize;       // sensor size: length [width, height]      (ncam x 2)
  float*    cam_intrinsic;        // [focal length; principal point]          (ncam x 4)
  sim_scalar_t*   cam_user;             // user data                                (ncam x nuser_cam)

  // lights
  int*      light_mode;           // light tracking mode (SIM_tCamLight)        (nlight x 1)
  int*      light_bodyid;         // id of light's body                       (nlight x 1)
  int*      light_targetbodyid;   // id of targeted body; -1: none            (nlight x 1)
  int*      light_type;           // spot, directional, etc. (SIM_tLightType)   (nlight x 1)
  int*      light_texid;          // texture id for image lights              (nlight x 1)
  sim_byte_t*  light_castshadow;     // does light cast shadows                  (nlight x 1)
  float*    light_bulbradius;     // light radius for soft shadows            (nlight x 1)
  float*    light_intensity;      // intensity, in candela                    (nlight x 1)
  float*    light_range;          // range of effectiveness                   (nlight x 1)
  sim_byte_t*  light_active;         // is light on                              (nlight x 1)
  sim_scalar_t*   light_pos;            // position rel. to body frame              (nlight x 3)
  sim_scalar_t*   light_dir;            // direction rel. to body frame             (nlight x 3)
  sim_scalar_t*   light_poscom0;        // global position rel. to sub-com in qpos0 (nlight x 3)
  sim_scalar_t*   light_pos0;           // global position rel. to body in qpos0    (nlight x 3)
  sim_scalar_t*   light_dir0;           // global direction in qpos0                (nlight x 3)
  float*    light_attenuation;    // OpenGL attenuation (quadratic model)     (nlight x 3)
  float*    light_cutoff;         // OpenGL cutoff                            (nlight x 1)
  float*    light_exponent;       // OpenGL exponent                          (nlight x 1)
  float*    light_ambient;        // ambient rgb (alpha=1)                    (nlight x 3)
  float*    light_diffuse;        // diffuse rgb (alpha=1)                    (nlight x 3)
  float*    light_specular;       // specular rgb (alpha=1)                   (nlight x 3)

  // flexes: contact properties
  int*      flex_contype;         // flex contact type                        (nflex x 1)
  int*      flex_conaffinity;     // flex contact affinity                    (nflex x 1)
  int*      flex_condim;          // contact dimensionality (1, 3, 4, 6)      (nflex x 1)
  int*      flex_priority;        // flex contact priority                    (nflex x 1)
  sim_scalar_t*   flex_solmix;          // mix coef for solref/imp in contact pair  (nflex x 1)
  sim_scalar_t*   flex_solref;          // constraint solver reference: contact     (nflex x SIM_NREF)
  sim_scalar_t*   flex_solimp;          // constraint solver impedance: contact     (nflex x SIM_NIMP)
  sim_scalar_t*   flex_friction;        // friction for (slide, spin, roll)         (nflex x 3)
  sim_scalar_t*   flex_margin;          // detect contact if dist<margin            (nflex x 1)
  sim_scalar_t*   flex_gap;             // include in solver if dist<margin-gap     (nflex x 1)
  sim_byte_t*  flex_internal;        // internal flex collision enabled          (nflex x 1)
  int*      flex_selfcollide;     // self collision mode (SIM_tFlexSelf)        (nflex x 1)
  int*      flex_activelayers;    // number of active element layers, 3D only (nflex x 1)
  int*      flex_passive;         // passive collisions enabled               (nflex x 1)

  // flexes: other properties
  int*      flex_dim;             // 1: lines, 2: triangles, 3: tetrahedra    (nflex x 1)
  int*      flex_matid;           // material id for rendering                (nflex x 1)
  int*      flex_group;           // group for visibility                     (nflex x 1)
  int*      flex_interp;          // interpolation (0: vertex, 1: nodes)      (nflex x 1)
  int*      flex_nodeadr;         // first node address                       (nflex x 1)
  int*      flex_nodenum;         // number of nodes                          (nflex x 1)
  int*      flex_vertadr;         // first vertex address                     (nflex x 1)
  int*      flex_vertnum;         // number of vertices                       (nflex x 1)
  int*      flex_edgeadr;         // first edge address                       (nflex x 1)
  int*      flex_edgenum;         // number of edges                          (nflex x 1)
  int*      flex_elemadr;         // first element address                    (nflex x 1)
  int*      flex_elemnum;         // number of elements                       (nflex x 1)
  int*      flex_elemdataadr;     // first element vertex id address          (nflex x 1)
  int*      flex_elemedgeadr;     // first element edge id address            (nflex x 1)
  int*      flex_shellnum;        // number of shells                         (nflex x 1)
  int*      flex_shelldataadr;    // first shell data address                 (nflex x 1)
  int*      flex_evpairadr;       // first evpair address                     (nflex x 1)
  int*      flex_evpairnum;       // number of evpairs                        (nflex x 1)
  int*      flex_texcoordadr;     // address in flex_texcoord; -1: none       (nflex x 1)
  int*      flex_nodebodyid;      // node body ids                            (nflexnode x 1)
  int*      flex_vertbodyid;      // vertex body ids                          (nflexvert x 1)
  int*      flex_vertedgeadr;     // first edge address                       (nflexvert x 1)
  int*      flex_vertedgenum;     // number of edges                          (nflexvert x 1)
  int*      flex_vertedge;        // edge indices                             (nflexedge x 2)
  int*      flex_edge;            // edge vertex ids (2 per edge)             (nflexedge x 2)
  int*      flex_edgeflap;        // adjacent vertex ids (dim=2 only)         (nflexedge x 2)
  int*      flex_elem;            // element vertex ids (dim+1 per elem)      (nflexelemdata x 1)
  int*      flex_elemtexcoord;    // element texture coordinates (dim+1)      (nflexelemdata x 1)
  int*      flex_elemedge;        // element edge ids                         (nflexelemedge x 1)
  int*      flex_elemlayer;       // element distance from surface, 3D only   (nflexelem x 1)
  int*      flex_shell;           // shell fragment vertex ids (dim per frag) (nflexshelldata x 1)
  int*      flex_evpair;          // (element, vertex) collision pairs        (nflexevpair x 2)
  sim_scalar_t*   flex_vert;            // vertex positions in local body frames    (nflexvert x 3)
  sim_scalar_t*   flex_vert0;           // vertex positions in qpos0 on [0, 1]^d    (nflexvert x 3)
  sim_scalar_t*   flex_vertmetric;      // inverse of reference shape matrix        (nflexvert x 4)
  sim_scalar_t*   flex_node;            // node positions in local body frames      (nflexnode x 3)
  sim_scalar_t*   flex_node0;           // Cartesian node positions in qpos0        (nflexnode x 3)
  sim_scalar_t*   flexedge_length0;     // edge lengths in qpos0                    (nflexedge x 1)
  sim_scalar_t*   flexedge_invweight0;  // edge inv. weight in qpos0                (nflexedge x 1)
  sim_scalar_t*   flex_radius;          // radius around primitive element          (nflex x 1)
  sim_scalar_t*   flex_size;            // vertex bounding box half sizes in qpos0  (nflex x 3)
  sim_scalar_t*   flex_stiffness;       // finite element stiffness matrix          (nflexelem x 21)
  sim_scalar_t*   flex_bending;         // bending stiffness                        (nflexedge x 17)
  sim_scalar_t*   flex_damping;         // Rayleigh's damping coefficient           (nflex x 1)
  sim_scalar_t*   flex_edgestiffness;   // edge stiffness                           (nflex x 1)
  sim_scalar_t*   flex_edgedamping;     // edge damping                             (nflex x 1)
  int*      flex_edgeequality;    // 0: none, 1: edges, 2: vertices           (nflex x 1)
  sim_byte_t*  flex_rigid;           // are all vertices in the same body        (nflex x 1)
  sim_byte_t*  flexedge_rigid;       // are both edge vertices in same body      (nflexedge x 1)
  sim_byte_t*  flex_centered;        // are all vertex coordinates (0,0,0)       (nflex x 1)
  sim_byte_t*  flex_flatskin;        // render flex skin with flat shading       (nflex x 1)
  int*      flex_bvhadr;          // address of bvh root; -1: no bvh          (nflex x 1)
  int*      flex_bvhnum;          // number of bounding volumes               (nflex x 1)
  int*      flexedge_J_rownnz;    // number of non-zeros in Jacobian row      (nflexedge x 1)
  int*      flexedge_J_rowadr;    // row start address in colind array        (nflexedge x 1)
  int*      flexedge_J_colind;    // column indices in sparse Jacobian        (nJfe x 1)
  int*      flexvert_J_rownnz;    // number of non-zeros in Jacobian row      (nflexvert x 2)
  int*      flexvert_J_rowadr;    // row start address in colind array        (nflexvert x 2)
  int*      flexvert_J_colind;    // column indices in sparse Jacobian        (nJfv x 2)
  float*    flex_rgba;            // rgba when material is omitted            (nflex x 4)
  float*    flex_texcoord;        // vertex texture coordinates               (nflextexcoord x 2)

  // meshes
  int*      mesh_vertadr;         // first vertex address                     (nmesh x 1)
  int*      mesh_vertnum;         // number of vertices                       (nmesh x 1)
  int*      mesh_faceadr;         // first face address                       (nmesh x 1)
  int*      mesh_facenum;         // number of faces                          (nmesh x 1)
  int*      mesh_bvhadr;          // address of bvh root                      (nmesh x 1)
  int*      mesh_bvhnum;          // number of bvh                            (nmesh x 1)
  int*      mesh_octadr;          // address of octree root                   (nmesh x 1)
  int*      mesh_octnum;          // number of octree nodes                   (nmesh x 1)
  int*      mesh_normaladr;       // first normal address                     (nmesh x 1)
  int*      mesh_normalnum;       // number of normals                        (nmesh x 1)
  int*      mesh_texcoordadr;     // texcoord data address; -1: no texcoord   (nmesh x 1)
  int*      mesh_texcoordnum;     // number of texcoord                       (nmesh x 1)
  int*      mesh_graphadr;        // graph data address; -1: no graph         (nmesh x 1)
  float*    mesh_vert;            // vertex positions for all meshes          (nmeshvert x 3)
  float*    mesh_normal;          // normals for all meshes                   (nmeshnormal x 3)
  float*    mesh_texcoord;        // vertex texcoords for all meshes          (nmeshtexcoord x 2)
  int*      mesh_face;            // vertex face data                         (nmeshface x 3)
  int*      mesh_facenormal;      // normal face data                         (nmeshface x 3)
  int*      mesh_facetexcoord;    // texture face data                        (nmeshface x 3)
  int*      mesh_graph;           // convex graph data                        (nmeshgraph x 1)
  sim_scalar_t*   mesh_scale;           // scaling applied to asset vertices        (nmesh x 3)
  sim_scalar_t*   mesh_pos;             // translation applied to asset vertices    (nmesh x 3)
  sim_scalar_t*   mesh_quat;            // rotation applied to asset vertices       (nmesh x 4)
  int*      mesh_pathadr;         // address of asset path for mesh; -1: none (nmesh x 1)
  int*      mesh_polynum;         // number of polygons per mesh              (nmesh x 1)
  int*      mesh_polyadr;         // first polygon address per mesh           (nmesh x 1)
  sim_scalar_t*   mesh_polynormal;      // all polygon normals                      (nmeshpoly x 3)
  int*      mesh_polyvertadr;     // polygon vertex start address             (nmeshpoly x 1)
  int*      mesh_polyvertnum;     // number of vertices per polygon           (nmeshpoly x 1)
  int*      mesh_polyvert;        // all polygon vertices                     (nmeshpolyvert x 1)
  int*      mesh_polymapadr;      // first polygon address per vertex         (nmeshvert x 1)
  int*      mesh_polymapnum;      // number of polygons per vertex            (nmeshvert x 1)
  int*      mesh_polymap;         // vertex to polygon map                    (nmeshpolymap x 1)

  // skins
  int*      skin_matid;           // skin material id; -1: none               (nskin x 1)
  int*      skin_group;           // group for visibility                     (nskin x 1)
  float*    skin_rgba;            // skin rgba                                (nskin x 4)
  float*    skin_inflate;         // inflate skin in normal direction         (nskin x 1)
  int*      skin_vertadr;         // first vertex address                     (nskin x 1)
  int*      skin_vertnum;         // number of vertices                       (nskin x 1)
  int*      skin_texcoordadr;     // texcoord data address; -1: no texcoord   (nskin x 1)
  int*      skin_faceadr;         // first face address                       (nskin x 1)
  int*      skin_facenum;         // number of faces                          (nskin x 1)
  int*      skin_boneadr;         // first bone in skin                       (nskin x 1)
  int*      skin_bonenum;         // number of bones in skin                  (nskin x 1)
  float*    skin_vert;            // vertex positions for all skin meshes     (nskinvert x 3)
  float*    skin_texcoord;        // vertex texcoords for all skin meshes     (nskintexvert x 2)
  int*      skin_face;            // triangle faces for all skin meshes       (nskinface x 3)
  int*      skin_bonevertadr;     // first vertex in each bone                (nskinbone x 1)
  int*      skin_bonevertnum;     // number of vertices in each bone          (nskinbone x 1)
  float*    skin_bonebindpos;     // bind pos of each bone                    (nskinbone x 3)
  float*    skin_bonebindquat;    // bind quat of each bone                   (nskinbone x 4)
  int*      skin_bonebodyid;      // body id of each bone                     (nskinbone x 1)
  int*      skin_bonevertid;      // mesh ids of vertices in each bone        (nskinbonevert x 1)
  float*    skin_bonevertweight;  // weights of vertices in each bone         (nskinbonevert x 1)
  int*      skin_pathadr;         // address of asset path for skin; -1: none (nskin x 1)

  // height fields
  sim_scalar_t*   hfield_size;          // (x, y, z_top, z_bottom)                  (nhfield x 4)
  int*      hfield_nrow;          // number of rows in grid                   (nhfield x 1)
  int*      hfield_ncol;          // number of columns in grid                (nhfield x 1)
  int*      hfield_adr;           // address in hfield_data                   (nhfield x 1)
  float*    hfield_data;          // elevation data                           (nhfielddata x 1)
  int*      hfield_pathadr;       // address of hfield asset path; -1: none   (nhfield x 1)

  // textures
  int*      tex_type;             // texture type (SIM_tTexture)                (ntex x 1)
  int*      tex_colorspace;       // texture colorspace (SIM_tColorSpace)       (ntex x 1)
  int*      tex_height;           // number of rows in texture image          (ntex x 1)
  int*      tex_width;            // number of columns in texture image       (ntex x 1)
  int*      tex_nchannel;         // number of channels in texture image      (ntex x 1)
  sim_size_t*  tex_adr;              // start address in tex_data                (ntex x 1)
  sim_byte_t*  tex_data;             // pixel values                             (ntexdata x 1)
  int*      tex_pathadr;          // address of texture asset path; -1: none  (ntex x 1)

  // materials
  int*      mat_texid;            // indices of textures; -1: none            (nmat x SIM_NTEXROLE)
  sim_byte_t*  mat_texuniform;       // make texture cube uniform                (nmat x 1)
  float*    mat_texrepeat;        // texture repetition for 2d mapping        (nmat x 2)
  float*    mat_emission;         // emission (x rgb)                         (nmat x 1)
  float*    mat_specular;         // specular (x white)                       (nmat x 1)
  float*    mat_shininess;        // shininess coef                           (nmat x 1)
  float*    mat_reflectance;      // reflectance (0: disable)                 (nmat x 1)
  float*    mat_metallic;         // metallic coef                            (nmat x 1)
  float*    mat_roughness;        // roughness coef                           (nmat x 1)
  float*    mat_rgba;             // rgba                                     (nmat x 4)

  // predefined geom pairs for collision detection; has precedence over exclude
  int*      pair_dim;             // contact dimensionality                   (npair x 1)
  int*      pair_geom1;           // id of geom1                              (npair x 1)
  int*      pair_geom2;           // id of geom2                              (npair x 1)
  int*      pair_signature;       // body1 << 16 + body2                      (npair x 1)
  sim_scalar_t*   pair_solref;          // solver reference: contact normal         (npair x SIM_NREF)
  sim_scalar_t*   pair_solreffriction;  // solver reference: contact friction       (npair x SIM_NREF)
  sim_scalar_t*   pair_solimp;          // solver impedance: contact                (npair x SIM_NIMP)
  sim_scalar_t*   pair_margin;          // detect contact if dist<margin            (npair x 1)
  sim_scalar_t*   pair_gap;             // include in solver if dist<margin-gap     (npair x 1)
  sim_scalar_t*   pair_friction;        // tangent1, 2, spin, roll1, 2              (npair x 5)

  // excluded body pairs for collision detection
  int*      exclude_signature;    // body1 << 16 + body2                      (nexclude x 1)

  // equality constraints
  int*      eq_type;              // constraint type (SIM_tEq)                  (neq x 1)
  int*      eq_obj1id;            // id of object 1                           (neq x 1)
  int*      eq_obj2id;            // id of object 2                           (neq x 1)
  int*      eq_objtype;           // type of both objects (sim_obj_t)            (neq x 1)
  sim_byte_t*  eq_active0;           // initial enable/disable constraint state  (neq x 1)
  sim_scalar_t*   eq_solref;            // constraint solver reference              (neq x SIM_NREF)
  sim_scalar_t*   eq_solimp;            // constraint solver impedance              (neq x SIM_NIMP)
  sim_scalar_t*   eq_data;              // numeric data for constraint              (neq x SIM_NEQDATA)

  // tendons
  int*      tendon_adr;           // address of first object in tendon's path (ntendon x 1)
  int*      tendon_num;           // number of objects in tendon's path       (ntendon x 1)
  int*      tendon_matid;         // material id for rendering                (ntendon x 1)
  int*      tendon_group;         // group for visibility                     (ntendon x 1)
  int*      tendon_treenum;       // number of trees along tendon's path      (ntendon x 1)
  int*      tendon_treeid;        // first two trees along tendon's path      (ntendon x 2)
  sim_byte_t*  tendon_limited;       // does tendon have length limits           (ntendon x 1)
  sim_byte_t*  tendon_actfrclimited; // does tendon have actuator force limits   (ntendon x 1)
  sim_scalar_t*   tendon_width;         // width for rendering                      (ntendon x 1)
  sim_scalar_t*   tendon_solref_lim;    // constraint solver reference: limit       (ntendon x SIM_NREF)
  sim_scalar_t*   tendon_solimp_lim;    // constraint solver impedance: limit       (ntendon x SIM_NIMP)
  sim_scalar_t*   tendon_solref_fri;    // constraint solver reference: friction    (ntendon x SIM_NREF)
  sim_scalar_t*   tendon_solimp_fri;    // constraint solver impedance: friction    (ntendon x SIM_NIMP)
  sim_scalar_t*   tendon_range;         // tendon length limits                     (ntendon x 2)
  sim_scalar_t*   tendon_actfrcrange;   // range of total actuator force            (ntendon x 2)
  sim_scalar_t*   tendon_margin;        // min distance for limit detection         (ntendon x 1)
  sim_scalar_t*   tendon_stiffness;     // stiffness coefficient                    (ntendon x 1)
  sim_scalar_t*   tendon_damping;       // damping coefficient                      (ntendon x 1)
  sim_scalar_t*   tendon_armature;      // inertia associated with tendon velocity  (ntendon x 1)
  sim_scalar_t*   tendon_frictionloss;  // loss due to friction                     (ntendon x 1)
  sim_scalar_t*   tendon_lengthspring;  // spring resting length range              (ntendon x 2)
  sim_scalar_t*   tendon_length0;       // tendon length in qpos0                   (ntendon x 1)
  sim_scalar_t*   tendon_invweight0;    // inv. weight in qpos0                     (ntendon x 1)
  sim_scalar_t*   tendon_user;          // user data                                (ntendon x nuser_tendon)
  float*    tendon_rgba;          // rgba when material is omitted            (ntendon x 4)

  // list of all wrap objects in tendon paths
  int*      wrap_type;            // wrap object type (SIM_tWrap)               (nwrap x 1)
  int*      wrap_objid;           // object id: geom, site, joint             (nwrap x 1)
  sim_scalar_t*   wrap_prm;             // divisor, joint coef, or site id          (nwrap x 1)

  // actuators
  int*      actuator_trntype;     // transmission type (SIM_tTrn)               (nu x 1)
  int*      actuator_dyntype;     // dynamics type (SIM_tDyn)                   (nu x 1)
  int*      actuator_gaintype;    // gain type (SIM_tGain)                      (nu x 1)
  int*      actuator_biastype;    // bias type (SIM_tBias)                      (nu x 1)
  int*      actuator_trnid;       // transmission id: joint, tendon, site     (nu x 2)
  int*      actuator_actadr;      // first activation address; -1: stateless  (nu x 1)
  int*      actuator_actnum;      // number of activation variables           (nu x 1)
  int*      actuator_group;       // group for visibility                     (nu x 1)
  int*      actuator_history;     // history buffer: [nsample, interp]        (nu x 2)
  int*      actuator_historyadr;  // address in history buffer; -1: none      (nu x 1)
  sim_scalar_t*   actuator_delay;       // delay time in seconds; 0: no delay       (nu x 1)
  sim_byte_t*  actuator_ctrllimited; // is control limited                       (nu x 1)
  sim_byte_t*  actuator_forcelimited;// is force limited                         (nu x 1)
  sim_byte_t*  actuator_actlimited;  // is activation limited                    (nu x 1)
  sim_scalar_t*   actuator_dynprm;      // dynamics parameters                      (nu x SIM_NDYN)
  sim_scalar_t*   actuator_gainprm;     // gain parameters                          (nu x SIM_NGAIN)
  sim_scalar_t*   actuator_biasprm;     // bias parameters                          (nu x SIM_NBIAS)
  sim_byte_t*  actuator_actearly;    // step activation before force             (nu x 1)
  sim_scalar_t*   actuator_ctrlrange;   // range of controls                        (nu x 2)
  sim_scalar_t*   actuator_forcerange;  // range of forces                          (nu x 2)
  sim_scalar_t*   actuator_actrange;    // range of activations                     (nu x 2)
  sim_scalar_t*   actuator_gear;        // scale length and transmitted force       (nu x 6)
  sim_scalar_t*   actuator_cranklength; // crank length for slider-crank            (nu x 1)
  sim_scalar_t*   actuator_acc0;        // acceleration from unit force in qpos0    (nu x 1)
  sim_scalar_t*   actuator_length0;     // actuator length in qpos0                 (nu x 1)
  sim_scalar_t*   actuator_lengthrange; // feasible actuator length range           (nu x 2)
  sim_scalar_t*   actuator_user;        // user data                                (nu x nuser_actuator)
  int*      actuator_plugin;      // plugin instance id; -1: not a plugin     (nu x 1)

  // sensors
  int*      sensor_type;          // sensor type (SIM_tSensor)                  (nsensor x 1)
  int*      sensor_datatype;      // numeric data type (SIM_tDataType)          (nsensor x 1)
  int*      sensor_needstage;     // required compute stage (SIM_tStage)        (nsensor x 1)
  int*      sensor_objtype;       // type of sensorized object (sim_obj_t)       (nsensor x 1)
  int*      sensor_objid;         // id of sensorized object                  (nsensor x 1)
  int*      sensor_reftype;       // type of reference frame (sim_obj_t)         (nsensor x 1)
  int*      sensor_refid;         // id of reference frame; -1: global frame  (nsensor x 1)
  int*      sensor_intprm;        // sensor parameters                        (nsensor x SIM_NSENS)
  int*      sensor_dim;           // number of scalar outputs                 (nsensor x 1)
  int*      sensor_adr;           // address in sensor array                  (nsensor x 1)
  sim_scalar_t*   sensor_cutoff;        // cutoff for real and positive; 0: ignore  (nsensor x 1)
  sim_scalar_t*   sensor_noise;         // noise standard deviation                 (nsensor x 1)
  int*      sensor_history;       // history buffer: [nsample, interp]        (nsensor x 2)
  int*      sensor_historyadr;    // address in history buffer; -1: none      (nsensor x 1)
  sim_scalar_t*   sensor_delay;         // delay time in seconds; 0: no delay       (nsensor x 1)
  sim_scalar_t*   sensor_interval;      // interval: [period, phase] in seconds     (nsensor x 2)
  sim_scalar_t*   sensor_user;          // user data                                (nsensor x nuser_sensor)
  int*      sensor_plugin;        // plugin instance id; -1: not a plugin     (nsensor x 1)

  // plugin instances
  int*      plugin;               // globally registered plugin slot number   (nplugin x 1)
  int*      plugin_stateadr;      // address in the plugin state array        (nplugin x 1)
  int*      plugin_statenum;      // number of states in the plugin instance  (nplugin x 1)
  char*     plugin_attr;          // config attributes of plugin instances    (npluginattr x 1)
  int*      plugin_attradr;       // address to each instance's config attrib (nplugin x 1)

  // custom numeric fields
  int*      numeric_adr;          // address of field in numeric_data         (nnumeric x 1)
  int*      numeric_size;         // size of numeric field                    (nnumeric x 1)
  sim_scalar_t*   numeric_data;         // array of all numeric fields              (nnumericdata x 1)

  // custom text fields
  int*      text_adr;             // address of text in text_data             (ntext x 1)
  int*      text_size;            // size of text field (strlen+1)            (ntext x 1)
  char*     text_data;            // array of all text fields (0-terminated)  (ntextdata x 1)

  // custom tuple fields
  int*      tuple_adr;            // address of text in text_data             (ntuple x 1)
  int*      tuple_size;           // number of objects in tuple               (ntuple x 1)
  int*      tuple_objtype;        // array of object types in all tuples      (ntupledata x 1)
  int*      tuple_objid;          // array of object ids in all tuples        (ntupledata x 1)
  sim_scalar_t*   tuple_objprm;         // array of object params in all tuples     (ntupledata x 1)

  // keyframes
  sim_scalar_t*   key_time;             // key time                                 (nkey x 1)
  sim_scalar_t*   key_qpos;             // key position                             (nkey x nq)
  sim_scalar_t*   key_qvel;             // key velocity                             (nkey x nv)
  sim_scalar_t*   key_act;              // key activation                           (nkey x na)
  sim_scalar_t*   key_mpos;             // key mocap position                       (nkey x nmocap*3)
  sim_scalar_t*   key_mquat;            // key mocap quaternion                     (nkey x nmocap*4)
  sim_scalar_t*   key_ctrl;             // key control                              (nkey x nu)

  // names
  int*      name_bodyadr;         // body name pointers                       (nbody x 1)
  int*      name_jntadr;          // joint name pointers                      (njnt x 1)
  int*      name_geomadr;         // geom name pointers                       (ngeom x 1)
  int*      name_siteadr;         // site name pointers                       (nsite x 1)
  int*      name_camadr;          // camera name pointers                     (ncam x 1)
  int*      name_lightadr;        // light name pointers                      (nlight x 1)
  int*      name_flexadr;         // flex name pointers                       (nflex x 1)
  int*      name_meshadr;         // mesh name pointers                       (nmesh x 1)
  int*      name_skinadr;         // skin name pointers                       (nskin x 1)
  int*      name_hfieldadr;       // hfield name pointers                     (nhfield x 1)
  int*      name_texadr;          // texture name pointers                    (ntex x 1)
  int*      name_matadr;          // material name pointers                   (nmat x 1)
  int*      name_pairadr;         // geom pair name pointers                  (npair x 1)
  int*      name_excludeadr;      // exclude name pointers                    (nexclude x 1)
  int*      name_eqadr;           // equality constraint name pointers        (neq x 1)
  int*      name_tendonadr;       // tendon name pointers                     (ntendon x 1)
  int*      name_actuatoradr;     // actuator name pointers                   (nu x 1)
  int*      name_sensoradr;       // sensor name pointers                     (nsensor x 1)
  int*      name_numericadr;      // numeric name pointers                    (nnumeric x 1)
  int*      name_textadr;         // text name pointers                       (ntext x 1)
  int*      name_tupleadr;        // tuple name pointers                      (ntuple x 1)
  int*      name_keyadr;          // keyframe name pointers                   (nkey x 1)
  int*      name_pluginadr;       // plugin instance name pointers            (nplugin x 1)
  char*     names;                // names of all objects, 0-terminated       (nnames x 1)
  int*      names_map;            // internal hash map of names               (nnames_map x 1)

  // paths
  char*     paths;                // paths to assets, 0-terminated            (npaths x 1)

  // sparse structures
  int*      B_rownnz;             // body-dof: non-zeros in each row          (nbody x 1)
  int*      B_rowadr;             // body-dof: row addresses                  (nbody x 1)
  int*      B_colind;             // body-dof: column indices                 (nB x 1)
  int*      M_rownnz;             // reduced inertia: non-zeros in each row   (nv x 1)
  int*      M_rowadr;             // reduced inertia: row addresses           (nv x 1)
  int*      M_colind;             // reduced inertia: column indices          (nC x 1)
  int*      mapM2M;               // index mapping from qM to M               (nC x 1)
  int*      D_rownnz;             // full inertia: non-zeros in each row      (nv x 1)
  int*      D_rowadr;             // full inertia: row addresses              (nv x 1)
  int*      D_diag;               // full inertia: index of diagonal element  (nv x 1)
  int*      D_colind;             // full inertia: column indices             (nD x 1)
  int*      mapM2D;               // index mapping from M to D                (nD x 1)
  int*      mapD2M;               // index mapping from D to M                (nC x 1)

  // compilation signature
  uint64_t  signature;            // also held by the sim_spec_t that compiled this model
};
typedef struct SIM_Model_ sim_model_t;

#endif  // SIMCORE_SIM_MODEL_H_
