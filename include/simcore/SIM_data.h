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

#ifndef SIMCORE_SIM_DATA_H_
#define SIMCORE_SIM_DATA_H_

#include <stddef.h>
#include <stdint.h>

#include <simcore/SIM_tnum.h>
#include <simcore/SIM_model.h>
#include <simcore/SIM_thread.h>

//---------------------------------- primitive types (simt) -----------------------------------------

typedef enum SIM_tState_ {            // state elements
  SIM_STATE_TIME           = 1<<0,    // time
  SIM_STATE_QPOS           = 1<<1,    // position
  SIM_STATE_QVEL           = 1<<2,    // velocity
  SIM_STATE_ACT            = 1<<3,    // actuator activation
  SIM_STATE_HISTORY        = 1<<4,    // history buffers (control, sensor)
  SIM_STATE_WARMSTART      = 1<<5,    // acceleration used for warmstart
  SIM_STATE_CTRL           = 1<<6,    // control
  SIM_STATE_QFRC_APPLIED   = 1<<7,    // applied generalized force
  SIM_STATE_XFRC_APPLIED   = 1<<8,    // applied Cartesian force/torque
  SIM_STATE_EQ_ACTIVE      = 1<<9,    // enable/disable constraints
  SIM_STATE_MOCAP_POS      = 1<<10,   // positions of mocap bodies
  SIM_STATE_MOCAP_QUAT     = 1<<11,   // orientations of mocap bodies
  SIM_STATE_USERDATA       = 1<<12,   // user data
  SIM_STATE_PLUGIN         = 1<<13,   // plugin state

  SIM_NSTATE               = 14,      // number of state elements

  // convenience values for commonly used state specifications
  SIM_STATE_PHYSICS        = SIM_STATE_QPOS | SIM_STATE_QVEL | SIM_STATE_ACT | SIM_STATE_HISTORY,
  SIM_STATE_FULLPHYSICS    = SIM_STATE_TIME | SIM_STATE_PHYSICS | SIM_STATE_PLUGIN,
  SIM_STATE_USER           = SIM_STATE_CTRL | SIM_STATE_QFRC_APPLIED | SIM_STATE_XFRC_APPLIED |
                          SIM_STATE_EQ_ACTIVE | SIM_STATE_MOCAP_POS | SIM_STATE_MOCAP_QUAT |
                          SIM_STATE_USERDATA,
  SIM_STATE_INTEGRATION    = SIM_STATE_FULLPHYSICS | SIM_STATE_USER | SIM_STATE_WARMSTART
} SIM_tState;


typedef enum SIM_tConstraint_ {       // type of constraint
  SIM_CNSTR_EQUALITY       = 0,       // equality constraint
  SIM_CNSTR_FRICTION_DOF,             // dof friction
  SIM_CNSTR_FRICTION_TENDON,          // tendon friction
  SIM_CNSTR_LIMIT_JOINT,              // joint limit
  SIM_CNSTR_LIMIT_TENDON,             // tendon limit
  SIM_CNSTR_CONTACT_FRICTIONLESS,     // frictionless contact
  SIM_CNSTR_CONTACT_PYRAMIDAL,        // frictional contact, pyramidal friction cone
  SIM_CNSTR_CONTACT_ELLIPTIC          // frictional contact, elliptic friction cone
} SIM_tConstraint;


typedef enum SIM_tConstraintState_ {  // constraint state
  SIM_CNSTRSTATE_SATISFIED = 0,       // constraint satisfied, zero cost (limit, contact)
  SIM_CNSTRSTATE_QUADRATIC,           // quadratic cost (equality, friction, limit, contact)
  SIM_CNSTRSTATE_LINEARNEG,           // linear cost, negative side (friction)
  SIM_CNSTRSTATE_LINEARPOS,           // linear cost, positive side (friction)
  SIM_CNSTRSTATE_CONE                 // squared distance to cone cost (elliptic contact)
} SIM_tConstraintState;


typedef enum SIM_tWarning_ {          // warning types
  SIM_WARN_INERTIA         = 0,       // (near) singular inertia matrix
  SIM_WARN_CONTACTFULL,               // too many contacts in contact list
  SIM_WARN_CNSTRFULL,                 // too many constraints
  SIM_WARN_VGEOMFULL,                 // too many visual geoms
  SIM_WARN_BADQPOS,                   // bad number in qpos
  SIM_WARN_BADQVEL,                   // bad number in qvel
  SIM_WARN_BADQACC,                   // bad number in qacc
  SIM_WARN_BADCTRL,                   // bad number in ctrl

  SIM_NWARNING                        // number of warnings
} SIM_tWarning;


typedef enum SIM_tTimer_ {            // internal timers
  // main api
  SIM_TIMER_STEP           = 0,       // step
  SIM_TIMER_FORWARD,                  // forward
  SIM_TIMER_INVERSE,                  // inverse

  // breakdown of step/forward
  SIM_TIMER_POSITION,                 // fwdPosition
  SIM_TIMER_VELOCITY,                 // fwdVelocity
  SIM_TIMER_ACTUATION,                // fwdActuation
  SIM_TIMER_CONSTRAINT,               // fwdConstraint
  SIM_TIMER_ADVANCE,                  // sim_Euler, sim_implicit

  // breakdown of fwdPosition
  SIM_TIMER_POS_KINEMATICS,           // kinematics, com, tendon, transmission
  SIM_TIMER_POS_INERTIA,              // inertia computations
  SIM_TIMER_POS_COLLISION,            // collision detection
  SIM_TIMER_POS_MAKE,                 // make constraints
  SIM_TIMER_POS_PROJECT,              // project constraints

  // breakdown of sim_collision
  SIM_TIMER_COL_BROAD,                // broadphase
  SIM_TIMER_COL_NARROW,               // narrowphase

  SIM_NTIMER                          // number of timers
} SIM_tTimer;


typedef enum SIM_tSleepState_ {       // sleep state of an object
  sim_spec_STATIC = -1,                  // object is static
  sim_spec_ASLEEP = 0,                   // object is asleep
  sim_spec_AWAKE  = 1                    // object is awake
} SIM_tSleepState;


//---------------------------------- sim_contact_t -----------------------------------------------------

struct SIM_Contact_ {                // result of collision detection functions
  // contact parameters set by near-phase collision function
  sim_scalar_t  dist;                    // distance between nearest points; neg: penetration
  sim_scalar_t  pos[3];                  // position of contact point: midpoint between geoms
  sim_scalar_t  frame[9];                // normal is in [0-2], points from geom[0] to geom[1]

  // contact parameters set by sim_collideGeoms
  sim_scalar_t  includemargin;           // include if dist<includemargin=margin-gap
  sim_scalar_t  friction[5];             // tangent1, 2, spin, roll1, 2
  sim_scalar_t  solref[SIM_NREF];          // constraint solver reference, normal direction
  sim_scalar_t  solreffriction[SIM_NREF];  // constraint solver reference, friction directions
  sim_scalar_t  solimp[SIM_NIMP];          // constraint solver impedance

  // internal storage used by solver
  sim_scalar_t  mu;                      // friction of regularized cone, set by sim_makeConstraint
  sim_scalar_t  H[36];                   // cone Hessian, set by sim_constraintUpdate

  // contact descriptors set by sim_collideXXX
  int     dim;                     // contact space dimensionality: 1, 3, 4 or 6
  int     geom1;                   // id of geom 1; deprecated, use geom[0]
  int     geom2;                   // id of geom 2; deprecated, use geom[1]
  int     geom[2];                 // geom ids; -1 for flex
  int     flex[2];                 // flex ids; -1 for geom
  int     elem[2];                 // element ids; -1 for geom or flex vertex
  int     vert[2];                 // vertex ids;  -1 for geom or flex element

  // flag set by sim_setContact or sim_instantiateContact
  int     exclude;                 // 0: include, 1: in gap, 2: fused, 3: no dofs, 4: passive

  // address computed by sim_instantiateContact
  int     efc_address;             // address in efc; -1: not included
};
typedef struct SIM_Contact_ sim_contact_t;


//---------------------------------- diagnostics ---------------------------------------------------

struct SIM_WarningStat_ {      // warning statistics
  int     lastinfo;          // info from last warning
  int     number;            // how many times was warning raised
};
typedef struct SIM_WarningStat_ SIM_WarningStat;


struct SIM_TimerStat_ {        // timer statistics
  sim_scalar_t  duration;          // cumulative duration
  int     number;            // how many times was timer called
};
typedef struct SIM_TimerStat_ SIM_TimerStat;


struct SIM_SolverStat_ {       // per-iteration solver statistics
  sim_scalar_t  improvement;       // cost reduction, scaled by 1/trace(M(qpos0))
  sim_scalar_t  gradient;          // gradient norm (primal only, scaled)
  sim_scalar_t  lineslope;         // slope in linesearch
  int     nactive;           // number of active constraints
  int     nchange;           // number of constraint state changes
  int     neval;             // number of cost evaluations in line search
  int     nupdate;           // number of Cholesky updates in line search
};
typedef struct SIM_SolverStat_ SIM_SolverStat;


//---------------------------------- sim_data_t --------------------------------------------------------

struct SIM_Data_ {
  // constant sizes
  sim_size_t narena;            // size of the arena in bytes (inclusive of the stack)
  sim_size_t nbuffer;           // size of main buffer in bytes
  int     nplugin;           // number of plugin instances

  // stack pointer
  size_t  pstack;            // first available byte in stack (mutable)
  size_t  pbase;             // value of pstack when sim_markStack was last called (mutable)

  // arena pointer
  size_t  parena;            // first available byte in arena

  // memory utilization statistics
  sim_size_t maxuse_stack;                       // maximum stack allocation in bytes (mutable)
  sim_size_t maxuse_threadstack[SIM_MAXTHREAD];    // maximum stack allocation per thread in bytes
  sim_size_t maxuse_arena;                       // maximum arena allocation in bytes
  int     maxuse_con;                         // maximum number of contacts
  int     maxuse_efc;                         // maximum number of scalar constraints

  // solver statistics
  SIM_SolverStat  solver[SIM_NISLAND*SIM_NSOLVER];  // solver statistics per island, per iteration
  int           solver_niter[SIM_NISLAND];      // number of solver iterations, per island
  int           solver_nnz[SIM_NISLAND];        // number of nonzeros in Hessian or efc_AR, per island
  sim_scalar_t        solver_fwdinv[2];             // forward-inverse comparison: qfrc, efc

  // diagnostics
  SIM_WarningStat warning[SIM_NWARNING];          // warning statistics (mutable)
  SIM_TimerStat   timer[SIM_NTIMER];              // timer statistics

  // variable sizes
  int     ncon;              // number of detected contacts
  int     ne;                // number of equality constraints
  int     nf;                // number of friction constraints
  int     nl;                // number of limit constraints
  int     nefc;              // number of constraints
  int     nJ;                // number of non-zeros in constraint Jacobian
  int     nA;                // number of non-zeros in constraint inverse inertia matrix
  int     nisland;           // number of detected constraint islands
  int     nidof;             // number of dofs in all islands
  int     ntree_awake;       // number of awake trees
  int     nbody_awake;       // number of awake dynamic and static bodies
  int     nparent_awake;     // number of bodies with awake parents
  int     nv_awake;          // number of awake dofs

  // flags marking lazily evaluated stages
  sim_byte_t flg_energypos;     // has sim_energyPos been called
  sim_byte_t flg_energyvel;     // has sim_energyVel been called
  sim_byte_t flg_subtreevel;    // has sim_subtreeVel been called
  sim_byte_t flg_rnepost;       // has sim_rnePostConstraint been called

  // global properties
  sim_scalar_t  time;              // simulation time
  sim_scalar_t  energy[2];         // potential, kinetic energy

  //-------------------- end of info header

  // buffers
  void*   buffer;            // main buffer; all pointers point in it            (nbuffer bytes)
  void*   arena;             // arena+stack buffer                               (narena bytes)

  //-------------------- main inputs and outputs of the computation

  // state
  sim_scalar_t* qpos;              // position                                         (nq x 1)
  sim_scalar_t* qvel;              // velocity                                         (nv x 1)
  sim_scalar_t* act;               // actuator activation                              (na x 1)
  sim_scalar_t* history;           // history buffer                                   (nhistory x 1)
  sim_scalar_t* qacc_warmstart;    // acceleration used for warmstart                  (nv x 1)
  sim_scalar_t* plugin_state;      // plugin state                                     (npluginstate x 1)

  // control
  sim_scalar_t* ctrl;              // control                                          (nu x 1)
  sim_scalar_t* qfrc_applied;      // applied generalized force                        (nv x 1)
  sim_scalar_t* xfrc_applied;      // applied Cartesian force/torque                   (nbody x 6)
  sim_byte_t* eq_active;        // enable/disable constraints                       (neq x 1)

  // mocap data
  sim_scalar_t* mocap_pos;         // positions of mocap bodies                        (nmocap x 3)
  sim_scalar_t* mocap_quat;        // orientations of mocap bodies                     (nmocap x 4)

  // dynamics
  sim_scalar_t* qacc;              // acceleration                                     (nv x 1)
  sim_scalar_t* act_dot;           // time-derivative of actuator activation           (na x 1)

  // user data
  sim_scalar_t* userdata;          // user data, not touched by engine                 (nuserdata x 1)

  // sensors
  sim_scalar_t* sensordata;        // sensor data array                                (nsensordata x 1)

  // sleep state
  int*    tree_asleep;       // <0: awake; >=0: index cycle of sleeping trees    (ntree x 1)

  // plugins
  int*       plugin;         // copy of m->plugin, required for deletion         (nplugin x 1)
  uintptr_t* plugin_data;    // pointer to plugin-managed data structure         (nplugin x 1)

  //-------------------- POSITION dependent

  // computed by sim_fwdPosition/sim_kinematics
  sim_scalar_t* xpos;              // Cartesian position of body frame                 (nbody x 3)
  sim_scalar_t* xquat;             // Cartesian orientation of body frame              (nbody x 4)
  sim_scalar_t* xmat;              // Cartesian orientation of body frame              (nbody x 9)
  sim_scalar_t* xipos;             // Cartesian position of body com                   (nbody x 3)
  sim_scalar_t* ximat;             // Cartesian orientation of body inertia            (nbody x 9)
  sim_scalar_t* xanchor;           // Cartesian position of joint anchor               (njnt x 3)
  sim_scalar_t* xaxis;             // Cartesian joint axis                             (njnt x 3)
  sim_scalar_t* geom_xpos;         // Cartesian geom position                          (ngeom x 3)
  sim_scalar_t* geom_xmat;         // Cartesian geom orientation                       (ngeom x 9)
  sim_scalar_t* site_xpos;         // Cartesian site position                          (nsite x 3)
  sim_scalar_t* site_xmat;         // Cartesian site orientation                       (nsite x 9)
  sim_scalar_t* cam_xpos;          // Cartesian camera position                        (ncam x 3)
  sim_scalar_t* cam_xmat;          // Cartesian camera orientation                     (ncam x 9)
  sim_scalar_t* light_xpos;        // Cartesian light position                         (nlight x 3)
  sim_scalar_t* light_xdir;        // Cartesian light direction                        (nlight x 3)

  // computed by sim_fwdPosition/sim_comPos
  sim_scalar_t* subtree_com;       // center of mass of each subtree                   (nbody x 3)
  sim_scalar_t* cdof;              // com-based motion axis of each dof (rot:lin)      (nv x 6)
  sim_scalar_t* cinert;            // com-based body inertia and mass                  (nbody x 10)

  // computed by sim_fwdPosition/sim_flex
  sim_scalar_t* flexvert_xpos;     // Cartesian flex vertex positions                  (nflexvert x 3)
  sim_scalar_t* flexelem_aabb;     // flex element bounding boxes (center, size)       (nflexelem x 6)
  sim_scalar_t* flexedge_J;        // flex edge Jacobian                               (nJfe x 1)
  sim_scalar_t* flexedge_length;   // flex edge lengths                                (nflexedge x 1)
  sim_scalar_t* flexvert_J;        // flex vertex Jacobian                             (nJfv x 2)
  sim_scalar_t* flexvert_length;   // flex vertex lengths                              (nflexvert x 2)
  sim_scalar_t* bvh_aabb_dyn;      // global bounding box (center, size)               (nbvhdynamic x 6)

  // computed by sim_fwdPosition/sim_tendon
  int*    ten_wrapadr;       // start address of tendon's path                   (ntendon x 1)
  int*    ten_wrapnum;       // number of wrap points in path                    (ntendon x 1)
  int*    ten_J_rownnz;      // number of non-zeros in Jacobian row              (ntendon x 1)
  int*    ten_J_rowadr;      // row start address in colind array                (ntendon x 1)
  int*    ten_J_colind;      // column indices in sparse Jacobian                (nJten x 1)
  sim_scalar_t* ten_J;             // tendon Jacobian                                  (nJten x 1)
  sim_scalar_t* ten_length;        // tendon lengths                                   (ntendon x 1)
  int*    wrap_obj;          // geom id; -1: site; -2: pulley                    (nwrap x 2)
  sim_scalar_t* wrap_xpos;         // Cartesian 3D points in all paths                 (nwrap x 6)

  // computed by sim_fwdPosition/sim_transmission
  sim_scalar_t* actuator_length;   // actuator lengths                                 (nu x 1)
  int*    moment_rownnz;     // number of non-zeros in actuator_moment row       (nu x 1)
  int*    moment_rowadr;     // row start address in colind array                (nu x 1)
  int*    moment_colind;     // column indices in sparse Jacobian                (nJmom x 1)
  sim_scalar_t* actuator_moment;   // actuator moments                                 (nJmom x 1)

  // computed by sim_fwdPosition/sim_makeM
  sim_scalar_t* crb;               // com-based composite inertia and mass             (nbody x 10)
  sim_scalar_t* qM;                // inertia (sparse)                                 (nM x 1)
  sim_scalar_t* M;                 // reduced inertia (compressed sparse row)          (nC x 1)

  // computed by sim_fwdPosition/sim_factorM
  sim_scalar_t* qLD;               // L'*D*L factorization of M (sparse)               (nC x 1)
  sim_scalar_t* qLDiagInv;         // 1/diag(D)                                        (nv x 1)

  // computed by sim_collision/sim_collideTree
  sim_byte_t* bvh_active;       // was bounding volume checked for collision        (nbvh x 1)

  // computed by sim_updateSleep
  int*    tree_awake;        // is tree awake; 0: asleep; 1: awake               (ntree x 1)
  int*    body_awake;        // body sleep state (SIM_tSleepState)                 (nbody x 1)
  int*    body_awake_ind;    // indices of awake and static bodies               (nbody x 1)
  int*    parent_awake_ind;  // indices of bodies with awake or static parents   (nbody x 1)
  int*    dof_awake_ind;     // indices of awake dofs                            (nv x 1)

  //-------------------- POSITION, VELOCITY dependent

  // computed by sim_fwdVelocity
  sim_scalar_t* flexedge_velocity; // flex edge velocities                             (nflexedge x 1)
  sim_scalar_t* ten_velocity;      // tendon velocities                                (ntendon x 1)
  sim_scalar_t* actuator_velocity; // actuator velocities                              (nu x 1)

  // computed by sim_fwdVelocity/sim_comVel
  sim_scalar_t* cvel;              // com-based velocity (rot:lin)                     (nbody x 6)
  sim_scalar_t* cdof_dot;          // time-derivative of cdof (rot:lin)                (nv x 6)

  // computed by sim_fwdVelocity/sim_rne (without acceleration)
  sim_scalar_t* qfrc_bias;         // C(qpos,qvel)                                     (nv x 1)

  // computed by sim_fwdVelocity/sim_passive
  sim_scalar_t* qfrc_spring;       // passive spring force                             (nv x 1)
  sim_scalar_t* qfrc_damper;       // passive damper force                             (nv x 1)
  sim_scalar_t* qfrc_gravcomp;     // passive gravity compensation force               (nv x 1)
  sim_scalar_t* qfrc_fluid;        // passive fluid force                              (nv x 1)
  sim_scalar_t* qfrc_passive;      // total passive force                              (nv x 1)

  // computed by sim_sensorVel/sim_subtreeVel if needed
  sim_scalar_t* subtree_linvel;    // linear velocity of subtree com                   (nbody x 3)
  sim_scalar_t* subtree_angmom;    // angular momentum about subtree com               (nbody x 3)

  // computed by sim_Euler or sim_implicit
  sim_scalar_t* qH;                // L'*D*L factorization of modified M               (nC x 1)
  sim_scalar_t* qHDiagInv;         // 1/diag(D) of modified M                          (nv x 1)

  // computed by sim_implicit/sim_derivative
  sim_scalar_t* qDeriv;            // d (passive + actuator - bias) / d qvel           (nD x 1)

  // computed by sim_implicit/sim_math_factorLUSparse
  sim_scalar_t* qLU;               // sparse LU of (qM - dt*qDeriv)                    (nD x 1)

  //-------------------- POSITION, VELOCITY, CONTROL/ACCELERATION dependent

  // computed by sim_fwdActuation
  sim_scalar_t* actuator_force;    // actuator force in actuation space                (nu x 1)
  sim_scalar_t* qfrc_actuator;     // actuator force                                   (nv x 1)

  // computed by sim_fwdAcceleration
  sim_scalar_t* qfrc_smooth;       // net unconstrained force                          (nv x 1)
  sim_scalar_t* qacc_smooth;       // unconstrained acceleration                       (nv x 1)

  // computed by sim_fwdConstraint/sim_inverse
  sim_scalar_t* qfrc_constraint;   // constraint force                                 (nv x 1)

  // computed by sim_inverse
  sim_scalar_t* qfrc_inverse;      // net external force; should equal:
                             // qfrc_applied + J'*xfrc_applied + qfrc_actuator   (nv x 1)

  // computed by sim_sensorAcc/sim_rnePostConstraint if needed; rotation:translation format
  sim_scalar_t* cacc;              // com-based acceleration                           (nbody x 6)
  sim_scalar_t* cfrc_int;          // com-based interaction force with parent          (nbody x 6)
  sim_scalar_t* cfrc_ext;          // com-based external force on body                 (nbody x 6)

  //-------------------- arena-allocated: POSITION dependent

  // computed by sim_collision
  sim_contact_t* contact;        // array of all detected contacts                   (ncon x 1)

  // computed by sim_makeConstraint
  int*    efc_type;          // constraint type (SIM_tConstraint)                  (nefc x 1)
  int*    efc_id;            // id of object of specified type                   (nefc x 1)
  int*    efc_J_rownnz;      // number of non-zeros in constraint Jacobian row   (nefc x 1)
  int*    efc_J_rowadr;      // row start address in colind array                (nefc x 1)
  int*    efc_J_rowsuper;    // number of subsequent rows in supernode           (nefc x 1)
  int*    efc_J_colind;      // column indices in constraint Jacobian            (nJ x 1)
  sim_scalar_t* efc_J;             // constraint Jacobian                              (nJ x 1)
  sim_scalar_t* efc_pos;           // constraint position (equality, contact)          (nefc x 1)
  sim_scalar_t* efc_margin;        // inclusion margin (contact)                       (nefc x 1)
  sim_scalar_t* efc_frictionloss;  // frictionloss (friction)                          (nefc x 1)
  sim_scalar_t* efc_diagApprox;    // approximation to diagonal of A                   (nefc x 1)
  sim_scalar_t* efc_KBIP;          // stiffness, damping, impedance, imp'              (nefc x 4)
  sim_scalar_t* efc_D;             // constraint mass                                  (nefc x 1)
  sim_scalar_t* efc_R;             // inverse constraint mass                          (nefc x 1)
  int*    tendon_efcadr;     // first efc address involving tendon; -1: none     (ntendon x 1)

  // computed by sim_island (island tree structure)
  int*    tree_island;       // island id of this tree; -1: none                 (ntree x 1)
  int*    island_ntree;      // number of trees in this island                   (nisland x 1)
  int*    island_itreeadr;   // island start address in itree vector             (nisland x 1)
  int*    map_itree2tree;    // map from itree to tree                           (ntree x 1)

  // computed by sim_island (island dof structure)
  int*    dof_island;        // island id of this dof; -1: none                  (nv x 1)
  int*    island_nv;         // number of dofs in this island                    (nisland x 1)
  int*    island_idofadr;    // island start address in idof vector              (nisland x 1)
  int*    island_dofadr;     // island start address in dof vector               (nisland x 1)
  int*    map_dof2idof;      // map from dof to idof                             (nv x 1)
  int*    map_idof2dof;      // map from idof to dof;  >= nidof: unconstrained   (nv x 1)

  // computed by sim_island (dofs sorted by island)
  sim_scalar_t* ifrc_smooth;       // net unconstrained force                          (nidof x 1)
  sim_scalar_t* iacc_smooth;       // unconstrained acceleration                       (nidof x 1)
  int*    iM_rownnz;         // inertia: non-zeros in each row                   (nidof x 1)
  int*    iM_rowadr;         // inertia: address of each row in iM_colind        (nidof x 1)
  int*    iM_colind;         // inertia: column indices of non-zeros             (nC x 1)
  sim_scalar_t* iM;                // total inertia (sparse)                           (nC x 1)
  sim_scalar_t* iLD;               // L'*D*L factorization of M (sparse)               (nC x 1)
  sim_scalar_t* iLDiagInv;         // 1/diag(D)                                        (nidof x 1)
  sim_scalar_t* iacc;              // acceleration                                     (nidof x 1)

  // computed by sim_island (island constraint structure)
  int*    efc_island;        // island id of this constraint                     (nefc x 1)
  int*    island_ne;         // number of equality constraints in island         (nisland x 1)
  int*    island_nf;         // number of friction constraints in island         (nisland x 1)
  int*    island_nefc;       // number of constraints in island                  (nisland x 1)
  int*    island_iefcadr;    // start address in iefc vector                     (nisland x 1)
  int*    map_efc2iefc;      // map from efc to iefc                             (nefc x 1)
  int*    map_iefc2efc;      // map from iefc to efc                             (nefc x 1)

  // computed by sim_island (constraints sorted by island)
  int*    iefc_type;         // constraint type (SIM_tConstraint)                  (nefc x 1)
  int*    iefc_id;           // id of object of specified type                   (nefc x 1)
  int*    iefc_J_rownnz;     // number of non-zeros in constraint Jacobian row   (nefc x 1)
  int*    iefc_J_rowadr;     // row start address in colind array                (nefc x 1)
  int*    iefc_J_rowsuper;   // number of subsequent rows in supernode           (nefc x 1)
  int*    iefc_J_colind;     // column indices in constraint Jacobian            (nJ x 1)
  sim_scalar_t* iefc_J;            // constraint Jacobian                              (nJ x 1)
  sim_scalar_t* iefc_frictionloss; // frictionloss (friction)                          (nefc x 1)
  sim_scalar_t* iefc_D;            // constraint mass                                  (nefc x 1)
  sim_scalar_t* iefc_R;            // inverse constraint mass                          (nefc x 1)

  // computed by sim_projectConstraint (PGS solver)
  int*    efc_AR_rownnz;     // number of non-zeros in AR                        (nefc x 1)
  int*    efc_AR_rowadr;     // row start address in colind array                (nefc x 1)
  int*    efc_AR_colind;     // column indices in sparse AR                      (nA x 1)
  sim_scalar_t* efc_AR;            // J*inv(M)*J' + R                                  (nA x 1)

  //-------------------- arena-allocated: POSITION, VELOCITY dependent

  // computed by sim_fwdVelocity/sim_referenceConstraint
  sim_scalar_t* efc_vel;           // velocity in constraint space: J*qvel             (nefc x 1)
  sim_scalar_t* efc_aref;          // reference pseudo-acceleration                    (nefc x 1)

  //-------------------- arena-allocated: POSITION, VELOCITY, CONTROL/ACCELERATION dependent

  // computed by sim_fwdConstraint/sim_inverse
  sim_scalar_t* efc_b;             // linear cost term: J*qacc_smooth - aref           (nefc x 1)
  sim_scalar_t* iefc_aref;         // reference pseudo-acceleration                    (nefc x 1)
  int*    iefc_state;        // constraint state (SIM_tConstraintState)            (nefc x 1)
  sim_scalar_t* iefc_force;        // constraint force in constraint space             (nefc x 1)
  int*    efc_state;         // constraint state (SIM_tConstraintState)            (nefc x 1)
  sim_scalar_t* efc_force;         // constraint force in constraint space             (nefc x 1)
  sim_scalar_t* ifrc_constraint;   // constraint force                                 (nidof x 1)

  // thread pool pointer
  uintptr_t threadpool;

  // compilation signature
  uint64_t  signature;       // also held by the sim_spec_t that compiled the model
};
typedef struct SIM_Data_ sim_data_t;


//---------------------------------- callback function types ---------------------------------------

// generic SimCore function
typedef void (*SIM_fGeneric)(const sim_model_t* m, sim_data_t* d);

// contact filter: 1- discard, 0- collide
typedef int (*SIM_fConFilt)(const sim_model_t* m, sim_data_t* d, int geom1, int geom2);

// sensor simulation
typedef void (*SIM_fSensor)(const sim_model_t* m, sim_data_t* d, int stage);

// timer
typedef sim_scalar_t (*SIM_fTime)(void);

// actuator dynamics, gain, bias
typedef sim_scalar_t (*SIM_fAct)(const sim_model_t* m, const sim_data_t* d, int id);

// collision detection
typedef int (*SIM_fCollision)(const sim_model_t* m, const sim_data_t* d,
                            sim_contact_t* con, int g1, int g2, sim_scalar_t margin);

#endif  // SIMCORE_SIM_DATA_H_
