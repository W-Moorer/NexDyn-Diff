// Copyright 2024 DeepMind Technologies Limited
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

#ifndef SIMCORE_INCLUDE_SIM_SPEC_H_
#define SIMCORE_INCLUDE_SIM_SPEC_H_

#include <stddef.h>
#include <simcore/SIM_model.h>
#include <simcore/SIM_tnum.h>


// this is a C-API
#ifdef __cplusplus
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

extern "C" {
#endif

//-------------------------------- handles to strings and arrays -----------------------------------

#ifdef __cplusplus
  // C++: defined to be compatible with corresponding std types
  using sim_string_t      = std::string;
  using SIM_StringVec   = std::vector<std::string>;
  using SIM_IntVec      = std::vector<int>;
  using SIM_IntVecVec   = std::vector<std::vector<int>>;
  using SIM_FloatVec    = std::vector<float>;
  using SIM_FloatVecVec = std::vector<std::vector<float>>;
  using SIM_DoubleVec   = std::vector<double>;
  using SIM_ByteVec     = std::vector<std::byte>;
#else
  // C: opaque types
  typedef void sim_string_t;
  typedef void SIM_StringVec;
  typedef void SIM_IntVec;
  typedef void SIM_IntVecVec;
  typedef void SIM_FloatVec;
  typedef void SIM_FloatVecVec;
  typedef void SIM_DoubleVec;
  typedef void SIM_ByteVec;
#endif


//-------------------------------- enum types (simt) ------------------------------------------------

typedef enum SIM_tGeomInertia_ {     // type of inertia inference
  SIM_INERTIA_VOLUME = 0,            // mass distributed in the volume
  SIM_INERTIA_SHELL,                 // mass distributed on the surface
} SIM_tGeomInertia;


typedef enum SIM_tMeshInertia_ {      // type of mesh inertia
  SIM_MESH_INERTIA_CONVEX = 0,        // convex mesh inertia
  SIM_MESH_INERTIA_EXACT,             // exact mesh inertia
  SIM_MESH_INERTIA_LEGACY,            // legacy mesh inertia
  SIM_MESH_INERTIA_SHELL              // shell mesh inertia
} SIM_tMeshInertia;


typedef enum SIM_tMeshBuiltin_ {      // type of built-in procedural mesh
  SIM_MESH_BUILTIN_NONE = 0,          // no built-in mesh
  SIM_MESH_BUILTIN_SPHERE,            // sphere
  SIM_MESH_BUILTIN_HEMISPHERE,        // hemisphere
  SIM_MESH_BUILTIN_CONE,              // cone
  SIM_MESH_BUILTIN_SUPERSPHERE,       // supersphere
  SIM_MESH_BUILTIN_SUPERTORUS,        // supertorus
  SIM_MESH_BUILTIN_WEDGE,             // wedge
  SIM_MESH_BUILTIN_PLATE,             // plate
} SIM_tMeshBuiltin;


typedef enum SIM_tBuiltin_ {         // type of built-in procedural texture
  SIM_BUILTIN_NONE = 0,              // no built-in texture
  SIM_BUILTIN_GRADIENT,              // gradient: rgb1->rgb2
  SIM_BUILTIN_CHECKER,               // checker pattern: rgb1, rgb2
  SIM_BUILTIN_FLAT                   // 2d: rgb1; cube: rgb1-up, rgb2-side, rgb3-down
} SIM_tBuiltin;


typedef enum SIM_tMark_ {            // mark type for procedural textures
  SIM_MARK_NONE = 0,                 // no mark
  SIM_MARK_EDGE,                     // edges
  SIM_MARK_CROSS,                    // cross
  SIM_MARK_RANDOM                    // random dots
} SIM_tMark;


typedef enum SIM_tLimited_ {         // type of limit specification
  SIM_LIMITED_FALSE = 0,             // not limited
  SIM_LIMITED_TRUE,                  // limited
  SIM_LIMITED_AUTO,                  // limited inferred from presence of range
} SIM_tLimited;


typedef enum SIM_tAlignFree_ {       // whether to align free joints with the inertial frame
  SIM_ALIGNFREE_FALSE = 0,           // don't align
  SIM_ALIGNFREE_TRUE,                // align
  SIM_ALIGNFREE_AUTO,                // respect the global compiler flag
} SIM_tAlignFree;


typedef enum SIM_tInertiaFromGeom_ { // whether to infer body inertias from child geoms
  SIM_INERTIAFROMGEOM_FALSE = 0,     // do not use; inertial element required
  SIM_INERTIAFROMGEOM_TRUE,          // always use; overwrite inertial element
  SIM_INERTIAFROMGEOM_AUTO           // use only if inertial element is missing
} SIM_tInertiaFromGeom;


typedef enum SIM_tOrientation_ {     // type of orientation specifier
  SIM_ORIENTATION_QUAT = 0,          // quaternion
  SIM_ORIENTATION_AXISANGLE,         // axis and angle
  SIM_ORIENTATION_XYAXES,            // x and y axes
  SIM_ORIENTATION_ZAXIS,             // z axis (minimal rotation)
  SIM_ORIENTATION_EULER,             // Euler angles
} SIM_tOrientation;


//-------------------------------- attribute structs (sims) -----------------------------------------

typedef struct SIM_sElement_ {       // element type, do not modify
  sim_obj_t elemtype;                 // element type
  uint64_t signature;              // compilation signature
} sim_spec_element_t;


typedef struct SIM_sCompiler_ {      // compiler options
  sim_byte_t autolimits;              // infer "limited" attribute based on range
  double boundmass;                // enforce minimum body mass
  double boundinertia;             // enforce minimum body diagonal inertia
  double settotalmass;             // rescale masses and inertias; <=0: ignore
  sim_byte_t balanceinertia;          // automatically impose A + B >= C rule
  sim_byte_t fitaabb;                 // meshfit to aabb instead of inertia box
  sim_byte_t degree;                  // angles in radians or degrees
  char eulerseq[3];                // sequence for euler rotations
  sim_byte_t discardvisual;           // discard visual geoms in parser
  sim_byte_t usethread;               // use multiple threads to speed up compiler
  sim_byte_t fusestatic;              // fuse static bodies with parent
  int inertiafromgeom;             // use geom inertias (SIM_tInertiaFromGeom)
  int inertiagrouprange[2];        // range of geom groups used to compute inertia
  sim_byte_t saveinertial;            // save explicit inertial clause for all bodies to XML
  int alignfree;                   // align free joints with inertial frame
  SIM_LROpt LRopt;                   // options for lengthrange computation
  sim_string_t* meshdir;               // mesh and hfield directory
  sim_string_t* texturedir;            // texture directory
} SIM_sCompiler;


typedef struct SIM_Spec_ {           // model specification
  sim_spec_element_t* element;             // element type
  sim_string_t* modelname;             // model name

  // compiler data
  SIM_sCompiler compiler;            // compiler options
  sim_byte_t strippath;               // automatically strip paths from mesh files

  // engine data
  SIM_Option option;                 // physics options
  SIM_Statistic stat;                // statistics override (if defined)

  // sizes
  sim_size_t memory;                  // number of bytes in arena+stack memory
  int nemax;                       // max number of equality constraints
  int nuserdata;                   // number of SIM_tNums in userdata
  int nuser_body;                  // number of SIM_tNums in body_user
  int nuser_jnt;                   // number of SIM_tNums in jnt_user
  int nuser_geom;                  // number of SIM_tNums in geom_user
  int nuser_site;                  // number of SIM_tNums in site_user
  int nuser_cam;                   // number of SIM_tNums in cam_user
  int nuser_tendon;                // number of SIM_tNums in tendon_user
  int nuser_actuator;              // number of SIM_tNums in actuator_user
  int nuser_sensor;                // number of SIM_tNums in sensor_user
  int nkey;                        // number of keyframes
  int njmax;                       // (deprecated) max number of constraints
  int nconmax;                     // (deprecated) max number of detected contacts
  sim_size_t nstack;                  // (deprecated) number of SIM_tNums in sim_data_t stack

  // global data
  sim_string_t* comment;               // comment at top of XML
  sim_string_t* modelfiledir;          // path to model file

  // other
  sim_byte_t hasImplicitPluginElem;   // already encountered an implicit plugin sensor/actuator
} sim_spec_t;


typedef struct SIM_sOrientation_ {   // alternative orientation specifiers
  SIM_tOrientation type;             // active orientation specifier
  double axisangle[4];             // axis and angle
  double xyaxes[6];                // x and y axes
  double zaxis[3];                 // z axis (minimal rotation)
  double euler[3];                 // Euler angles
} SIM_sOrientation;


typedef struct SIM_sPlugin_ {        // plugin specification
  sim_spec_element_t* element;             // element type
  sim_string_t* name;                  // instance name
  sim_string_t* plugin_name;           // plugin name
  sim_byte_t active;                  // is the plugin active
  sim_string_t* info;                  // message appended to compiler errors
} SIM_sPlugin;


typedef struct SIM_sBody_ {          // body specification
  sim_spec_element_t* element;             // element type
  sim_string_t* childclass;            // childclass name

  // body frame
  double pos[3];                   // frame position
  double quat[4];                  // frame orientation
  SIM_sOrientation alt;              // frame alternative orientation

  // inertial frame
  double mass;                     // mass
  double ipos[3];                  // inertial frame position
  double iquat[4];                 // inertial frame orientation
  double inertia[3];               // diagonal inertia (in i-frame)
  SIM_sOrientation ialt;             // inertial frame alternative orientation
  double fullinertia[6];           // non-axis-aligned inertia matrix

  // other
  sim_byte_t mocap;                   // is this a mocap body
  double gravcomp;                 // gravity compensation
  SIM_tSleepPolicy sleep;            // sleep policy
  SIM_DoubleVec* userdata;           // user data
  sim_byte_t explicitinertial;        // whether to save the body with explicit inertial clause
  SIM_sPlugin plugin;                // passive force plugin
  sim_string_t* info;                  // message appended to compiler errors
} sim_spec_body_t;


typedef struct SIM_sFrame_ {         // frame specification
  sim_spec_element_t* element;             // element type
  sim_string_t* childclass;            // childclass name
  double pos[3];                   // position
  double quat[4];                  // orientation
  SIM_sOrientation alt;              // alternative orientation
  sim_string_t* info;                  // message appended to compiler errors
} SIM_sFrame;


typedef struct SIM_sJoint_ {         // joint specification
  sim_spec_element_t* element;             // element type
  SIM_tJoint type;                   // joint type

  // kinematics
  double pos[3];                   // anchor position
  double axis[3];                  // joint axis
  double ref;                      // value at reference configuration: qpos0
  int align;                       // align free joint with body com (SIM_tAlignFree)

  // stiffness
  double stiffness;                // stiffness coefficient
  double springref;                // spring reference value: qpos_spring
  double springdamper[2];          // timeconst, dampratio

  // limits
  int limited;                     // does joint have limits (SIM_tLimited)
  double range[2];                 // joint limits
  double margin;                   // margin value for joint limit detection
  sim_scalar_t solref_limit[SIM_NREF];     // solver reference: joint limits
  sim_scalar_t solimp_limit[SIM_NIMP];     // solver impedance: joint limits
  int actfrclimited;               // are actuator forces on joint limited (SIM_tLimited)
  double actfrcrange[2];           // actuator force limits

  // dof properties
  double armature;                 // armature inertia (mass for slider)
  double damping;                  // damping coefficient
  double frictionloss;             // friction loss
  sim_scalar_t solref_friction[SIM_NREF];  // solver reference: dof friction
  sim_scalar_t solimp_friction[SIM_NIMP];  // solver impedance: dof friction

  // other
  int group;                       // group
  sim_byte_t actgravcomp;             // is gravcomp force applied via actuators
  SIM_DoubleVec* userdata;           // user data
  sim_string_t* info;                  // message appended to compiler errors
} SIM_sJoint;


typedef struct SIM_sGeom_ {          // geom specification
  sim_spec_element_t* element;             // element type
  SIM_tGeom type;                    // geom type

  // frame, size
  double pos[3];                   // position
  double quat[4];                  // orientation
  SIM_sOrientation alt;              // alternative orientation
  double fromto[6];                // alternative for capsule, cylinder, box, ellipsoid
  double size[3];                  // type-specific size

  // contact related
  int contype;                     // contact type
  int conaffinity;                 // contact affinity
  int condim;                      // contact dimensionality
  int priority;                    // contact priority
  double friction[3];              // one-sided friction coefficients: slide, roll, spin
  double solmix;                   // solver mixing for contact pairs
  sim_scalar_t solref[SIM_NREF];           // solver reference
  sim_scalar_t solimp[SIM_NIMP];           // solver impedance
  double margin;                   // margin for contact detection
  double gap;                      // include in solver if dist < margin-gap

  // inertia inference
  double mass;                     // used to compute density
  double density;                  // used to compute mass and inertia from volume or surface
  SIM_tGeomInertia typeinertia;      // selects between surface and volume inertia

  // fluid forces
  sim_scalar_t fluid_ellipsoid;          // whether ellipsoid-fluid model is active
  sim_scalar_t fluid_coefs[5];           // ellipsoid-fluid interaction coefs

  // visual
  sim_string_t* material;              // name of material
  float rgba[4];                   // rgba when material is omitted
  int group;                       // group

  // other
  sim_string_t* hfieldname;            // heightfield attached to geom
  sim_string_t* meshname;              // mesh attached to geom
  double fitscale;                 // scale mesh uniformly
  SIM_DoubleVec* userdata;           // user data
  SIM_sPlugin plugin;                // sdf plugin
  sim_string_t* info;                  // message appended to compiler errors
} sim_spec_geom_t;


typedef struct SIM_sSite_ {          // site specification
  sim_spec_element_t* element;             // element type

  // frame, size
  double pos[3];                   // position
  double quat[4];                  // orientation
  SIM_sOrientation alt;              // alternative orientation
  double fromto[6];                // alternative for capsule, cylinder, box, ellipsoid
  double size[3];                  // geom size

  // visual
  SIM_tGeom type;                    // geom type
  sim_string_t* material;              // name of material
  int group;                       // group
  float rgba[4];                   // rgba when material is omitted

  // other
  SIM_DoubleVec* userdata;           // user data
  sim_string_t* info;                  // message appended to compiler errors
} SIM_sSite;


typedef struct SIM_sCamera_ {        // camera specification
  sim_spec_element_t* element;             // element type

  // extrinsics
  double pos[3];                   // position
  double quat[4];                  // orientation
  SIM_sOrientation alt;              // alternative orientation
  SIM_tCamLight mode;                // tracking mode
  sim_string_t* targetbody;            // target body for tracking/targeting

  // intrinsics
  SIM_tProjection proj;              // camera projection type
  int resolution[2];               // resolution (pixel)
  int output;                      // bit flags for output type
  double fovy;                     // y-field of view
  double ipd;                      // inter-pupillary distance
  float intrinsic[4];              // camera intrinsics (length)
  float sensor_size[2];            // sensor size (length)
  float focal_length[2];           // focal length (length)
  float focal_pixel[2];            // focal length (pixel)
  float principal_length[2];       // principal point (length)
  float principal_pixel[2];        // principal point (pixel)

  // other
  SIM_DoubleVec* userdata;           // user data
  sim_string_t* info;                  // message appended to compiler errors
} SIM_sCamera;


typedef struct SIM_sLight_ {         // light specification
  sim_spec_element_t* element;             // element type

  // frame
  double pos[3];                   // position
  double dir[3];                   // direction
  SIM_tCamLight mode;                // tracking mode
  sim_string_t* targetbody;            // target body for targeting

  // intrinsics
  sim_byte_t active;                  // is light active
  SIM_tLightType type;               // type of light
  sim_string_t* texture;               // texture name for image lights
  sim_byte_t castshadow;              // does light cast shadows
  float bulbradius;                // bulb radius, for soft shadows
  float intensity;                 // intensity, in candelas
  float range;                     // range of effectiveness
  float attenuation[3];            // OpenGL attenuation (quadratic model)
  float cutoff;                    // OpenGL cutoff
  float exponent;                  // OpenGL exponent
  float ambient[3];                // ambient color
  float diffuse[3];                // diffuse color
  float specular[3];               // specular color

  // other
  sim_string_t* info;                  // message appended to compiler errorsx
} SIM_sLight;


typedef struct SIM_sFlex_ {          // flex specification
  sim_spec_element_t* element;             // element type

  // contact properties
  int contype;                     // contact type
  int conaffinity;                 // contact affinity
  int condim;                      // contact dimensionality
  int priority;                    // contact priority
  double friction[3];              // one-sided friction coefficients: slide, roll, spin
  double solmix;                   // solver mixing for contact pairs
  sim_scalar_t solref[SIM_NREF];           // solver reference
  sim_scalar_t solimp[SIM_NIMP];           // solver impedance
  double margin;                   // margin for contact detection
  double gap;                      // include in solver if dist<margin-gap

  // other properties
  int dim;                         // element dimensionality
  double radius;                   // radius around primitive element
  double size[3];                  // vertex bounding box half sizes in qpos0
  sim_byte_t internal;                // enable internal collisions
  sim_byte_t flatskin;                // render flex skin with flat shading
  int selfcollide;                 // mode for flex self collision
  int vertcollide;                 // mode for vertex collision
  int passive;                     // mode for passive collisions
  int activelayers;                // number of active element layers in 3D
  int group;                       // group for visualization
  double edgestiffness;            // edge stiffness
  double edgedamping;              // edge damping
  float rgba[4];                   // rgba when material is omitted
  sim_string_t* material;              // name of material used for rendering
  double young;                    // Young's modulus
  double poisson;                  // Poisson's ratio
  double damping;                  // Rayleigh's damping
  double thickness;                // thickness (2D only)
  int elastic2d;                   // 2D passive forces; 0: none, 1: bending, 2: stretching, 3: both

  // mesh properties
  SIM_StringVec* nodebody;           // node body names
  SIM_StringVec* vertbody;           // vertex body names
  SIM_DoubleVec* node;               // node positions
  SIM_DoubleVec* vert;               // vertex positions
  SIM_IntVec* elem;                  // element vertex ids
  SIM_FloatVec* texcoord;            // vertex texture coordinates
  SIM_IntVec* elemtexcoord;          // element texture coordinates

  // other
  sim_string_t* info;                  // message appended to compiler errors
} SIM_sFlex;


typedef struct SIM_sMesh_ {          // mesh specification
  sim_spec_element_t* element;             // element type
  sim_string_t* content_type;          // content type of file
  sim_string_t* file;                  // mesh file
  double refpos[3];                // reference position
  double refquat[4];               // reference orientation
  double scale[3];                 // rescale mesh
  SIM_tMeshInertia inertia;          // inertia type (convex, legacy, exact, shell)
  sim_byte_t smoothnormal;            // do not exclude large-angle faces from normals
  sim_byte_t needsdf;                 // compute sdf from mesh
  int maxhullvert;                 // maximum vertex count for the convex hull
  SIM_FloatVec* uservert;            // user vertex data
  SIM_FloatVec* usernormal;          // user normal data
  SIM_FloatVec* usertexcoord;        // user texcoord data
  SIM_IntVec* userface;              // user vertex indices
  SIM_IntVec* userfacenormal;        // user face normal indices
  SIM_IntVec* userfacetexcoord;      // user texcoord indices
  SIM_sPlugin plugin;                // sdf plugin
  sim_string_t* material;              // name of material
  sim_string_t* info;                  // message appended to compiler errors
} SIM_sMesh;


typedef struct SIM_sHField_ {        // height field specification
  sim_spec_element_t* element;             // element type
  sim_string_t* content_type;          // content type of file
  sim_string_t* file;                  // file: (nrow, ncol, [elevation data])
  double size[4];                  // hfield size (ignore referencing geom size)
  int nrow;                        // number of rows
  int ncol;                        // number of columns
  SIM_FloatVec* userdata;            // user-provided elevation data
  sim_string_t* info;                  // message appended to compiler errors
} SIM_sHField;



typedef struct SIM_sSkin_ {          // skin specification
  sim_spec_element_t* element;             // element type
  sim_string_t* file;                  // skin file
  sim_string_t* material;              // name of material used for rendering
  float rgba[4];                   // rgba when material is omitted
  float inflate;                   // inflate in normal direction
  int group;                       // group for visualization

  // mesh
  SIM_FloatVec* vert;                // vertex positions
  SIM_FloatVec* texcoord;            // texture coordinates
  SIM_IntVec* face;                  // faces

  // skin
  SIM_StringVec* bodyname;           // body names
  SIM_FloatVec* bindpos;             // bind pos
  SIM_FloatVec* bindquat;            // bind quat
  SIM_IntVecVec* vertid;             // vertex ids
  SIM_FloatVecVec* vertweight;       // vertex weights

  // other
  sim_string_t* info;                  // message appended to compiler errors
} SIM_sSkin;


typedef struct SIM_sTexture_ {       // texture specification
  sim_spec_element_t* element;             // element type
  SIM_tTexture type;                 // texture type
  SIM_tColorSpace colorspace;        // colorspace

  // method 1: builtin
  int builtin;                     // builtin type (SIM_tBuiltin)
  int mark;                        // mark type (SIM_tMark)
  double rgb1[3];                  // first color for builtin
  double rgb2[3];                  // second color for builtin
  double markrgb[3];               // mark color
  double random;                   // probability of random dots
  int height;                      // height in pixels (square for cube and skybox)
  int width;                       // width in pixels
  int nchannel;                    // number of channels

  // method 2: single file
  sim_string_t* content_type;          // content type of file
  sim_string_t* file;                  // png file to load; use for all sides of cube
  int gridsize[2];                 // size of grid for composite file; (1,1)-repeat
  char gridlayout[12];             // row-major: L,R,F,B,U,D for faces; . for unused

  // method 3: separate files
  SIM_StringVec* cubefiles;          // different file for each side of the cube

  // method 4: from buffer read by user
  SIM_ByteVec* data;                  // texture data

  // flip options
  sim_byte_t hflip;                   // horizontal flip
  sim_byte_t vflip;                   // vertical flip

  // other
  sim_string_t* info;                  // message appended to compiler errors
} SIM_sTexture;


typedef struct SIM_sMaterial_ {      // material specification
  sim_spec_element_t* element;             // element type
  SIM_StringVec* textures;           // names of textures (empty: none)
  sim_byte_t texuniform;              // make texture cube uniform
  float texrepeat[2];              // texture repetition for 2D mapping
  float emission;                  // emission
  float specular;                  // specular
  float shininess;                 // shininess
  float reflectance;               // reflectance
  float metallic;                  // metallic
  float roughness;                 // roughness
  float rgba[4];                   // rgba
  sim_string_t* info;                  // message appended to compiler errors
} SIM_sMaterial;


typedef struct SIM_sPair_ {          // pair specification
  sim_spec_element_t* element;             // element type
  sim_string_t* geomname1;             // name of geom 1
  sim_string_t* geomname2;             // name of geom 2

  // optional parameters: computed from geoms if not set by user
  int condim;                      // contact dimensionality
  sim_scalar_t solref[SIM_NREF];           // solver reference, normal direction
  sim_scalar_t solreffriction[SIM_NREF];   // solver reference, frictional directions
  sim_scalar_t solimp[SIM_NIMP];           // solver impedance
  double margin;                   // margin for contact detection
  double gap;                      // include in solver if dist<margin-gap
  double friction[5];              // full contact friction
  sim_string_t* info;                  // message appended to errors
} SIM_sPair;


typedef struct SIM_sExclude_ {       // exclude specification
  sim_spec_element_t* element;             // element type
  sim_string_t* bodyname1;             // name of geom 1
  sim_string_t* bodyname2;             // name of geom 2
  sim_string_t* info;                  // message appended to errors
} SIM_sExclude;


typedef struct SIM_sEquality_ {      // equality specification
  sim_spec_element_t* element;             // element type
  SIM_tEq type;                      // constraint type
  double data[SIM_NEQDATA];          // type-dependent data
  sim_byte_t active;                  // is equality initially active
  sim_string_t* name1;                 // name of object 1
  sim_string_t* name2;                 // name of object 2
  sim_obj_t objtype;                  // type of both objects
  sim_scalar_t solref[SIM_NREF];           // solver reference
  sim_scalar_t solimp[SIM_NIMP];           // solver impedance
  sim_string_t* info;                  // message appended to errors
} SIM_sEquality;


typedef struct SIM_sTendon_ {        // tendon specification
  sim_spec_element_t* element;             // element type

  // stiffness, damping, friction, armature
  double stiffness;                // stiffness coefficient
  double springlength[2];          // spring resting length; {-1, -1}: use qpos_spring
  double damping;                  // damping coefficient
  double frictionloss;             // friction loss
  sim_scalar_t solref_friction[SIM_NREF];  // solver reference: tendon friction
  sim_scalar_t solimp_friction[SIM_NIMP];  // solver impedance: tendon friction
  double armature;                 // inertia associated with tendon velocity

  // length range
  int limited;                     // does tendon have limits (SIM_tLimited)
  int actfrclimited;               // does tendon have actuator force limits
  double range[2];                 // length limits
  double actfrcrange[2];           // actuator force limits
  double margin;                   // margin value for tendon limit detection
  sim_scalar_t solref_limit[SIM_NREF];     // solver reference: tendon limits
  sim_scalar_t solimp_limit[SIM_NIMP];     // solver impedance: tendon limits

  // visual
  sim_string_t* material;              // name of material for rendering
  double width;                    // width for rendering
  float rgba[4];                   // rgba when material is omitted
  int group;                       // group

  // other
  SIM_DoubleVec* userdata;           // user data
  sim_string_t* info;                  // message appended to errors
} SIM_sTendon;


typedef struct SIM_sWrap_ {          // wrapping object specification
  sim_spec_element_t* element;             // element type
  SIM_tWrap type;                    // wrap type
  sim_string_t* info;                  // message appended to errors
} SIM_sWrap;


typedef struct SIM_sActuator_ {      // actuator specification
  sim_spec_element_t* element;             // element type

  // gain, bias
  SIM_tGain gaintype;                // gain type
  double gainprm[SIM_NGAIN];         // gain parameters
  SIM_tBias biastype;                // bias type
  double biasprm[SIM_NGAIN];         // bias parameters

  // activation state
  SIM_tDyn dyntype;                  // dynamics type
  double dynprm[SIM_NDYN];           // dynamics parameters
  int actdim;                      // number of activation variables
  sim_byte_t actearly;                // apply next activations to qfrc

  // transmission
  SIM_tTrn trntype;                  // transmission type
  double gear[6];                  // length and transmitted force scaling
  sim_string_t* target;                // name of transmission target
  sim_string_t* refsite;               // reference site, for site transmission
  sim_string_t* slidersite;            // site defining cylinder, for slider-crank
  double cranklength;              // crank length, for slider-crank
  double lengthrange[2];           // transmission length range
  double inheritrange;             // automatic range setting for position and intvelocity

  // input/output clamping
  int ctrllimited;                 // are control limits defined (SIM_tLimited)
  double ctrlrange[2];             // control range
  int forcelimited;                // are force limits defined (SIM_tLimited)
  double forcerange[2];            // force range
  int actlimited;                  // are activation limits defined (SIM_tLimited)
  double actrange[2];              // activation range

  // other
  int group;                       // group
  int nsample;                     // number of samples in history buffer
  int interp;                      // interpolation order (0=ZOH, 1=linear, 2=cubic)
  double delay;                    // delay time in seconds; 0: no delay
  SIM_DoubleVec* userdata;           // user data
  SIM_sPlugin plugin;                // actuator plugin
  sim_string_t* info;                  // message appended to compiler errors
} sim_spec_actuator_t;


typedef struct SIM_sSensor_ {        // sensor specification
  sim_spec_element_t* element;             // element type

  // sensor definition
  SIM_tSensor type;                  // type of sensor
  sim_obj_t objtype;                  // type of sensorized object
  sim_string_t* objname;               // name of sensorized object
  sim_obj_t reftype;                  // type of referenced object
  sim_string_t* refname;               // name of referenced object
  int intprm[SIM_NSENS];             // integer parameters

  // user-defined sensors
  SIM_tDataType datatype;            // data type for sensor measurement
  SIM_tStage needstage;              // compute stage needed to simulate sensor
  int dim;                         // number of scalar outputs

  // output post-processing
  double cutoff;                   // cutoff for real and positive datatypes
  double noise;                    // noise stdev

  // history buffer
  int nsample;                     // number of samples in history buffer
  int interp;                      // interpolation order (0=ZOH, 1=linear, 2=cubic)
  double delay;                    // delay time in seconds
  double interval[2];              // [period, time_prev] in seconds

  // other
  SIM_DoubleVec* userdata;           // user data
  SIM_sPlugin plugin;                // sensor plugin
  sim_string_t* info;                  // message appended to compiler errors
} SIM_sSensor;


typedef struct SIM_sNumeric_ {       // custom numeric field specification
  sim_spec_element_t* element;             // element type
  SIM_DoubleVec* data;               // initialization data
  int size;                        // array size, can be bigger than data size
  sim_string_t* info;                  // message appended to compiler errors
} SIM_sNumeric;


typedef struct SIM_sText_ {          // custom text specification
  sim_spec_element_t* element;             // element type
  sim_string_t* data;                  // text string
  sim_string_t* info;                  // message appended to compiler errors
} SIM_sText;


typedef struct SIM_sTuple_ {         // tuple specification
  sim_spec_element_t* element;             // element type
  SIM_IntVec* objtype;               // object types
  SIM_StringVec* objname;            // object names
  SIM_DoubleVec* objprm;             // object parameters
  sim_string_t* info;                  // message appended to compiler errors
} SIM_sTuple;


typedef struct SIM_sKey_ {           // keyframe specification
  sim_spec_element_t* element;             // element type
  double time;                     // time
  SIM_DoubleVec* qpos;               // qpos
  SIM_DoubleVec* qvel;               // qvel
  SIM_DoubleVec* act;                // act
  SIM_DoubleVec* mpos;               // mocap pos
  SIM_DoubleVec* mquat;              // mocap quat
  SIM_DoubleVec* ctrl;               // ctrl
  sim_string_t* info;                  // message appended to compiler errors
} SIM_sKey;


typedef struct SIM_sDefault_ {       // default specification
  sim_spec_element_t* element;             // element type
  SIM_sJoint* joint;                 // joint defaults
  sim_spec_geom_t* geom;                   // geom defaults
  SIM_sSite* site;                   // site defaults
  SIM_sCamera* camera;               // camera defaults
  SIM_sLight* light;                 // light defaults
  SIM_sFlex* flex;                   // flex defaults
  SIM_sMesh* mesh;                   // mesh defaults
  SIM_sMaterial* material;           // material defaults
  SIM_sPair* pair;                   // pair defaults
  SIM_sEquality* equality;           // equality defaults
  SIM_sTendon* tendon;               // tendon defaults
  sim_spec_actuator_t* actuator;           // actuator defaults
} sim_spec_default_t;

#ifdef __cplusplus
}
#endif

#endif  // SIMCORE_INCLUDE_SIM_SPEC_H_
