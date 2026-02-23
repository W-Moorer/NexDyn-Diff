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

#ifndef SIMCORE_SIM_XMACRO_H_
#define SIMCORE_SIM_XMACRO_H_


//-------------------------------- SIM_Option --------------------------------------------------------

// fields of SIM_Option
// XVEC means that a field is a vector (i.e. size > 1)
#define SIMOPTION_FIELDS                        \
    X   ( sim_scalar_t, timestep,          1      )  \
    X   ( sim_scalar_t, impratio,          1      )  \
    X   ( sim_scalar_t, tolerance,         1      )  \
    X   ( sim_scalar_t, ls_tolerance,      1      )  \
    X   ( sim_scalar_t, noslip_tolerance,  1      )  \
    X   ( sim_scalar_t, ccd_tolerance,     1      )  \
    X   ( sim_scalar_t, sleep_tolerance,   1      )  \
    XVEC( sim_scalar_t, gravity,           3      )  \
    XVEC( sim_scalar_t, wind,              3      )  \
    XVEC( sim_scalar_t, magnetic,          3      )  \
    X   ( sim_scalar_t, density,           1      )  \
    X   ( sim_scalar_t, viscosity,         1      )  \
    X   ( sim_scalar_t, o_margin,          1      )  \
    XVEC( sim_scalar_t, o_solref,          SIM_NREF )  \
    XVEC( sim_scalar_t, o_solimp,          SIM_NIMP )  \
    XVEC( sim_scalar_t, o_friction,        5      )  \
    X   ( int,    integrator,        1      )  \
    X   ( int,    cone,              1      )  \
    X   ( int,    jacobian,          1      )  \
    X   ( int,    solver,            1      )  \
    X   ( int,    iterations,        1      )  \
    X   ( int,    ls_iterations,     1      )  \
    X   ( int,    noslip_iterations, 1      )  \
    X   ( int,    ccd_iterations,    1      )  \
    X   ( int,    disableflags,      1      )  \
    X   ( int,    enableflags,       1      )  \
    X   ( int,    disableactuator,   1      )  \
    X   ( int,    sdf_initpoints,    1      )  \
    X   ( int,    sdf_iterations,    1      )


//-------------------------------- SIM_Statistic -----------------------------------------------------

// fields of SIM_Statistic
#define SIMSTATISTIC_FIELDS  \
    X   ( meaninertia, 1 )  \
    X   ( meanmass,    1 )  \
    X   ( meansize,    1 )  \
    X   ( extent,      1 )  \
    XVEC( center,      3 )


//-------------------------------- sim_model_t ---------------------------------------------------------

// size fields of sim_model_t
#define SIMMODEL_SIZES       \
    X( nq )                 \
    X( nv )                 \
    X( nu )                 \
    X( na )                 \
    X( nbody )              \
    X( nbvh )               \
    X( nbvhstatic )         \
    X( nbvhdynamic )        \
    X( noct )               \
    X( njnt )               \
    X( ntree )              \
    X( nM )                 \
    X( nB )                 \
    X( nC )                 \
    X( nD )                 \
    X( ngeom )              \
    X( nsite )              \
    X( ncam )               \
    X( nlight )             \
    X( nflex )              \
    X( nflexnode )          \
    X( nflexvert )          \
    X( nflexedge )          \
    X( nflexelem )          \
    X( nflexelemdata )      \
    X( nflexelemedge )      \
    X( nflexshelldata )     \
    X( nflexevpair )        \
    X( nflextexcoord )      \
    X( nJfe )               \
    X( nJfv )               \
    X( nmesh )              \
    X( nmeshvert )          \
    X( nmeshnormal )        \
    X( nmeshtexcoord )      \
    X( nmeshface )          \
    X( nmeshgraph )         \
    X( nmeshpoly )          \
    X( nmeshpolyvert )      \
    X( nmeshpolymap )       \
    X( nskin )              \
    X( nskinvert )          \
    X( nskintexvert )       \
    X( nskinface )          \
    X( nskinbone )          \
    X( nskinbonevert )      \
    X( nhfield )            \
    X( nhfielddata )        \
    X( ntex )               \
    X( ntexdata )           \
    X( nmat )               \
    X( npair )              \
    X( nexclude )           \
    X( neq )                \
    X( ntendon )            \
    X( nwrap )              \
    X( nsensor )            \
    X( nnumeric )           \
    X( nnumericdata )       \
    X( ntext )              \
    X( ntextdata )          \
    X( ntuple )             \
    X( ntupledata )         \
    X( nkey )               \
    X( nmocap )             \
    X( nplugin )            \
    X( npluginattr )        \
    X( nuser_body )         \
    X( nuser_jnt )          \
    X( nuser_geom )         \
    X( nuser_site )         \
    X( nuser_cam )          \
    X( nuser_tendon )       \
    X( nuser_actuator )     \
    X( nuser_sensor )       \
    X( nnames )             \
    X( npaths )             \
    X( nnames_map )         \
    X( nJmom )              \
    X( nJten )              \
    X( ngravcomp )          \
    X( nemax )              \
    X( njmax )              \
    X( nconmax )            \
    X( nuserdata )          \
    X( nsensordata )        \
    X( npluginstate )       \
    X( nhistory )           \
    X( narena )             \
    X( nbuffer )

    /* nbuffer needs to be the final field */


// define symbols needed in SIMMODEL_POINTERS (corresponding to number of columns)
#define SIMMODEL_POINTERS_PREAMBLE( m )      \
    int nuser_body = m->nuser_body;         \
    int nuser_jnt = m->nuser_jnt;           \
    int nuser_geom = m->nuser_geom;         \
    int nuser_site = m->nuser_site;         \
    int nuser_cam = m->nuser_cam;           \
    int nuser_tendon = m->nuser_tendon;     \
    int nuser_actuator = m->nuser_actuator; \
    int nuser_sensor = m->nuser_sensor;     \
    int nq = m->nq;                         \
    int nv = m->nv;                         \
    int na = m->na;                         \
    int nu = m->nu;                         \
    int nmocap = m->nmocap;

// macro for annotating that an array size in an X macro is a member of sim_model_t
// by default this macro does nothing, but users can redefine it as necessary
#define SIM_M(n) n


// pointer fields of sim_model_t
// XNV marks optional fields in specialized X-macro expansions.
// By default we define XNV to be the same as X.
#define SIMMODEL_POINTERS_BODY                                                   \
    X   ( int,     body_parentid,         nbody,         1                    ) \
    X   ( int,     body_rootid,           nbody,         1                    ) \
    X   ( int,     body_weldid,           nbody,         1                    ) \
    X   ( int,     body_mocapid,          nbody,         1                    ) \
    X   ( int,     body_jntnum,           nbody,         1                    ) \
    X   ( int,     body_jntadr,           nbody,         1                    ) \
    X   ( int,     body_dofnum,           nbody,         1                    ) \
    X   ( int,     body_dofadr,           nbody,         1                    ) \
    X   ( int,     body_treeid,           nbody,         1                    ) \
    X   ( int,     body_geomnum,          nbody,         1                    ) \
    X   ( int,     body_geomadr,          nbody,         1                    ) \
    X   ( sim_byte_t, body_simple,           nbody,         1                    ) \
    X   ( sim_byte_t, body_sameframe,        nbody,         1                    ) \
    X   ( sim_scalar_t,  body_pos,              nbody,         3                    ) \
    X   ( sim_scalar_t,  body_quat,             nbody,         4                    ) \
    X   ( sim_scalar_t,  body_ipos,             nbody,         3                    ) \
    X   ( sim_scalar_t,  body_iquat,            nbody,         4                    ) \
    X   ( sim_scalar_t,  body_mass,             nbody,         1                    ) \
    X   ( sim_scalar_t,  body_subtreemass,      nbody,         1                    ) \
    X   ( sim_scalar_t,  body_inertia,          nbody,         3                    ) \
    X   ( sim_scalar_t,  body_invweight0,       nbody,         2                    ) \
    X   ( sim_scalar_t,  body_gravcomp,         nbody,         1                    ) \
    X   ( sim_scalar_t,  body_margin,           nbody,         1                    ) \
    X   ( sim_scalar_t,  body_user,             nbody,         SIM_M(nuser_body)     ) \
    X   ( int,     body_plugin,           nbody,         1                    ) \
    X   ( int,     body_contype,          nbody,         1                    ) \
    X   ( int,     body_conaffinity,      nbody,         1                    ) \
    X   ( int,     body_bvhadr,           nbody,         1                    ) \
    X   ( int,     body_bvhnum,           nbody,         1                    )

#define SIMMODEL_POINTERS_JOINT                                                  \
    X   ( int,     jnt_type,              njnt,          1                    ) \
    X   ( int,     jnt_qposadr,           njnt,          1                    ) \
    X   ( int,     jnt_dofadr,            njnt,          1                    ) \
    X   ( int,     jnt_bodyid,            njnt,          1                    ) \
    X   ( int,     jnt_group,             njnt,          1                    ) \
    X   ( sim_byte_t, jnt_limited,           njnt,          1                    ) \
    X   ( sim_byte_t, jnt_actfrclimited,     njnt,          1                    ) \
    X   ( sim_byte_t, jnt_actgravcomp,       njnt,          1                    ) \
    X   ( sim_scalar_t,  jnt_solref,            njnt,          SIM_NREF               ) \
    X   ( sim_scalar_t,  jnt_solimp,            njnt,          SIM_NIMP               ) \
    X   ( sim_scalar_t,  jnt_pos,               njnt,          3                    ) \
    X   ( sim_scalar_t,  jnt_axis,              njnt,          3                    ) \
    X   ( sim_scalar_t,  jnt_stiffness,         njnt,          1                    ) \
    X   ( sim_scalar_t,  jnt_range,             njnt,          2                    ) \
    X   ( sim_scalar_t,  jnt_actfrcrange,       njnt,          2                    ) \
    X   ( sim_scalar_t,  jnt_margin,            njnt,          1                    ) \
    X   ( sim_scalar_t,  jnt_user,              njnt,          SIM_M(nuser_jnt)      )

#define SIMMODEL_POINTERS_DOF                                                    \
    X   ( int,     dof_bodyid,            nv,            1                    ) \
    X   ( int,     dof_jntid,             nv,            1                    ) \
    X   ( int,     dof_parentid,          nv,            1                    ) \
    X   ( int,     dof_treeid,            nv,            1                    ) \
    X   ( int,     dof_Madr,              nv,            1                    ) \
    X   ( int,     dof_simplenum,         nv,            1                    ) \
    X   ( sim_scalar_t,  dof_solref,            nv,            SIM_NREF               ) \
    X   ( sim_scalar_t,  dof_solimp,            nv,            SIM_NIMP               ) \
    X   ( sim_scalar_t,  dof_frictionloss,      nv,            1                    ) \
    X   ( sim_scalar_t,  dof_armature,          nv,            1                    ) \
    X   ( sim_scalar_t,  dof_damping,           nv,            1                    ) \
    X   ( sim_scalar_t,  dof_invweight0,        nv,            1                    ) \
    X   ( sim_scalar_t,  dof_M0,                nv,            1                    ) \
    X   ( sim_scalar_t,  dof_length,            nv,            1                    )

#define SIMMODEL_POINTERS_TREE                                                   \
    X   ( int,     tree_bodyadr,          ntree,         1                    ) \
    X   ( int,     tree_bodynum,          ntree,         1                    ) \
    X   ( int,     tree_dofadr,           ntree,         1                    ) \
    X   ( int,     tree_dofnum,           ntree,         1                    ) \
    X   ( int,     tree_sleep_policy,     ntree,         1                    )

#define SIMMODEL_POINTERS_GEOM                                                   \
    X   ( int,     geom_type,             ngeom,         1                    ) \
    X   ( int,     geom_contype,          ngeom,         1                    ) \
    X   ( int,     geom_conaffinity,      ngeom,         1                    ) \
    X   ( int,     geom_condim,           ngeom,         1                    ) \
    X   ( int,     geom_bodyid,           ngeom,         1                    ) \
    X   ( int,     geom_dataid,           ngeom,         1                    ) \
    X   ( int,     geom_matid,            ngeom,         1                    ) \
    X   ( int,     geom_group,            ngeom,         1                    ) \
    X   ( int,     geom_priority,         ngeom,         1                    ) \
    X   ( int,     geom_plugin,           ngeom,         1                    ) \
    X   ( sim_byte_t, geom_sameframe,        ngeom,         1                    ) \
    X   ( sim_scalar_t,  geom_solmix,           ngeom,         1                    ) \
    X   ( sim_scalar_t,  geom_solref,           ngeom,         SIM_NREF               ) \
    X   ( sim_scalar_t,  geom_solimp,           ngeom,         SIM_NIMP               ) \
    X   ( sim_scalar_t,  geom_size,             ngeom,         3                    ) \
    X   ( sim_scalar_t,  geom_aabb,             ngeom,         6                    ) \
    X   ( sim_scalar_t,  geom_rbound,           ngeom,         1                    ) \
    X   ( sim_scalar_t,  geom_pos,              ngeom,         3                    ) \
    X   ( sim_scalar_t,  geom_quat,             ngeom,         4                    ) \
    X   ( sim_scalar_t,  geom_friction,         ngeom,         3                    ) \
    X   ( sim_scalar_t,  geom_margin,           ngeom,         1                    ) \
    X   ( sim_scalar_t,  geom_gap,              ngeom,         1                    ) \
    XNV ( sim_scalar_t,  geom_fluid,            ngeom,         SIM_NFLUID             ) \
    X   ( sim_scalar_t,  geom_user,             ngeom,         SIM_M(nuser_geom)     ) \
    X   ( float,   geom_rgba,             ngeom,         4                    )

#define SIMMODEL_POINTERS_SITE                                                   \
    X   ( int,     site_type,             nsite,         1                    ) \
    X   ( int,     site_bodyid,           nsite,         1                    ) \
    X   ( int,     site_matid,            nsite,         1                    ) \
    X   ( int,     site_group,            nsite,         1                    ) \
    X   ( sim_byte_t, site_sameframe,        nsite,         1                    ) \
    X   ( sim_scalar_t,  site_size,             nsite,         3                    ) \
    X   ( sim_scalar_t,  site_pos,              nsite,         3                    ) \
    X   ( sim_scalar_t,  site_quat,             nsite,         4                    ) \
    X   ( sim_scalar_t,  site_user,             nsite,         SIM_M(nuser_site)     ) \
    X   ( float,   site_rgba,             nsite,         4                    )

#define SIMMODEL_POINTERS_CAMERA                                                 \
    X   ( int,     cam_mode,              ncam,          1                    ) \
    X   ( int,     cam_bodyid,            ncam,          1                    ) \
    X   ( int,     cam_targetbodyid,      ncam,          1                    ) \
    X   ( sim_scalar_t,  cam_pos,               ncam,          3                    ) \
    X   ( sim_scalar_t,  cam_quat,              ncam,          4                    ) \
    X   ( sim_scalar_t,  cam_poscom0,           ncam,          3                    ) \
    X   ( sim_scalar_t,  cam_pos0,              ncam,          3                    ) \
    X   ( sim_scalar_t,  cam_mat0,              ncam,          9                    ) \
    X   ( int,     cam_projection,        ncam,          1                    ) \
    X   ( sim_scalar_t,  cam_fovy,              ncam,          1                    ) \
    X   ( sim_scalar_t,  cam_ipd,               ncam,          1                    ) \
    X   ( int,     cam_resolution,        ncam,          2                    ) \
    X   ( int,     cam_output,            ncam,          1                    ) \
    X   ( float,   cam_sensorsize,        ncam,          2                    ) \
    X   ( float,   cam_intrinsic,         ncam,          4                    ) \
    X   ( sim_scalar_t,  cam_user,              ncam,          SIM_M(nuser_cam)      )

#define SIMMODEL_POINTERS_LIGHT                                                  \
    X   ( int,     light_mode,            nlight,        1                    ) \
    X   ( int,     light_bodyid,          nlight,        1                    ) \
    X   ( int,     light_targetbodyid,    nlight,        1                    ) \
    X   ( int,     light_type,            nlight,        1                    ) \
    X   ( int,     light_texid,           nlight,        1                    ) \
    X   ( sim_byte_t, light_castshadow,      nlight,        1                    ) \
    X   ( float,   light_bulbradius,      nlight,        1                    ) \
    X   ( float,   light_intensity,       nlight,        1                    ) \
    X   ( float,   light_range,           nlight,        1                    ) \
    X   ( sim_byte_t, light_active,          nlight,        1                    ) \
    X   ( sim_scalar_t,  light_pos,             nlight,        3                    ) \
    X   ( sim_scalar_t,  light_dir,             nlight,        3                    ) \
    X   ( sim_scalar_t,  light_poscom0,         nlight,        3                    ) \
    X   ( sim_scalar_t,  light_pos0,            nlight,        3                    ) \
    X   ( sim_scalar_t,  light_dir0,            nlight,        3                    ) \
    X   ( float,   light_attenuation,     nlight,        3                    ) \
    X   ( float,   light_cutoff,          nlight,        1                    ) \
    X   ( float,   light_exponent,        nlight,        1                    ) \
    X   ( float,   light_ambient,         nlight,        3                    ) \
    X   ( float,   light_diffuse,         nlight,        3                    ) \
    X   ( float,   light_specular,        nlight,        3                    )

#define SIMMODEL_POINTERS_FLEX                                                   \
    X   ( int,     flex_contype,          nflex,         1                    ) \
    X   ( int,     flex_conaffinity,      nflex,         1                    ) \
    X   ( int,     flex_condim,           nflex,         1                    ) \
    X   ( int,     flex_priority,         nflex,         1                    ) \
    X   ( sim_scalar_t,  flex_solmix,           nflex,         1                    ) \
    X   ( sim_scalar_t,  flex_solref,           nflex,         SIM_NREF               ) \
    X   ( sim_scalar_t,  flex_solimp,           nflex,         SIM_NIMP               ) \
    X   ( sim_scalar_t,  flex_friction,         nflex,         3                    ) \
    X   ( sim_scalar_t,  flex_margin,           nflex,         1                    ) \
    X   ( sim_scalar_t,  flex_gap,              nflex,         1                    ) \
    X   ( sim_byte_t, flex_internal,         nflex,         1                    ) \
    X   ( int,     flex_selfcollide,      nflex,         1                    ) \
    X   ( int,     flex_activelayers,     nflex,         1                    ) \
    X   ( int,     flex_passive,          nflex,         1                    ) \
    X   ( int,     flex_dim,              nflex,         1                    ) \
    X   ( int,     flex_matid,            nflex,         1                    ) \
    X   ( int,     flex_group,            nflex,         1                    ) \
    X   ( int,     flex_interp,           nflex,         1                    ) \
    X   ( int,     flex_nodeadr,          nflex,         1                    ) \
    X   ( int,     flex_nodenum,          nflex,         1                    ) \
    X   ( int,     flex_vertadr,          nflex,         1                    ) \
    X   ( int,     flex_vertnum,          nflex,         1                    ) \
    X   ( int,     flex_edgeadr,          nflex,         1                    ) \
    X   ( int,     flex_edgenum,          nflex,         1                    ) \
    X   ( int,     flex_elemadr,          nflex,         1                    ) \
    X   ( int,     flex_elemnum,          nflex,         1                    ) \
    X   ( int,     flex_elemdataadr,      nflex,         1                    ) \
    X   ( int,     flex_elemedgeadr,      nflex,         1                    ) \
    X   ( int,     flex_shellnum,         nflex,         1                    ) \
    X   ( int,     flex_shelldataadr,     nflex,         1                    ) \
    X   ( int,     flex_evpairadr,        nflex,         1                    ) \
    X   ( int,     flex_evpairnum,        nflex,         1                    ) \
    X   ( int,     flex_texcoordadr,      nflex,         1                    ) \
    X   ( int,     flex_nodebodyid,       nflexnode,     1                    ) \
    X   ( int,     flex_vertbodyid,       nflexvert,     1                    ) \
    X   ( int,     flex_vertedgeadr,      nflexvert,     1                    ) \
    X   ( int,     flex_vertedgenum,      nflexvert,     1                    ) \
    X   ( int,     flex_vertedge,         nflexedge,     2                    ) \
    X   ( int,     flex_edge,             nflexedge,     2                    ) \
    X   ( int,     flex_edgeflap,         nflexedge,     2                    ) \
    X   ( int,     flex_elem,             nflexelemdata, 1                    ) \
    X   ( int,     flex_elemtexcoord,     nflexelemdata, 1                    ) \
    X   ( int,     flex_elemedge,         nflexelemedge, 1                    ) \
    X   ( int,     flex_elemlayer,        nflexelem,     1                    ) \
    X   ( int,     flex_shell,            nflexshelldata,1                    ) \
    X   ( int,     flex_evpair,           nflexevpair,   2                    ) \
    X   ( sim_scalar_t,  flex_vert,             nflexvert,     3                    ) \
    X   ( sim_scalar_t,  flex_vert0,            nflexvert,     3                    ) \
    X   ( sim_scalar_t,  flex_vertmetric,       nflexvert,     4                    ) \
    X   ( sim_scalar_t,  flex_node,             nflexnode,     3                    ) \
    X   ( sim_scalar_t,  flex_node0,            nflexnode,     3                    ) \
    X   ( sim_scalar_t,  flexedge_length0,      nflexedge,     1                    ) \
    X   ( sim_scalar_t,  flexedge_invweight0,   nflexedge,     1                    ) \
    X   ( sim_scalar_t,  flex_radius,           nflex,         1                    ) \
    X   ( sim_scalar_t,  flex_size,             nflex,         3                    ) \
    X   ( sim_scalar_t,  flex_stiffness,        nflexelem,     21                   ) \
    X   ( sim_scalar_t,  flex_bending,          nflexedge,     17                   ) \
    X   ( sim_scalar_t,  flex_damping,          nflex,         1                    ) \
    X   ( sim_scalar_t,  flex_edgestiffness,    nflex,         1                    ) \
    X   ( sim_scalar_t,  flex_edgedamping,      nflex,         1                    ) \
    X   ( int,     flex_edgeequality,     nflex,         1                    ) \
    X   ( sim_byte_t, flex_rigid,            nflex,         1                    ) \
    X   ( sim_byte_t, flexedge_rigid,        nflexedge,     1                    ) \
    X   ( sim_byte_t, flex_centered,         nflex,         1                    ) \
    X   ( sim_byte_t, flex_flatskin,         nflex,         1                    ) \
    X   ( int,     flex_bvhadr,           nflex,         1                    ) \
    X   ( int,     flex_bvhnum,           nflex,         1                    ) \
    X   ( int,     flexedge_J_rownnz,     nflexedge,     1                    ) \
    X   ( int,     flexedge_J_rowadr,     nflexedge,     1                    ) \
    X   ( int,     flexedge_J_colind,     nJfe,          1                    ) \
    X   ( int,     flexvert_J_rownnz,     nflexvert,     2                    ) \
    X   ( int,     flexvert_J_rowadr,     nflexvert,     2                    ) \
    X   ( int,     flexvert_J_colind,     nJfv,          2                    ) \
    X   ( float,   flex_rgba,             nflex,         4                    ) \
    X   ( float,   flex_texcoord,         nflextexcoord, 2                    )

#define SIMMODEL_POINTERS_MESH                                                   \
    X   ( int,     mesh_vertadr,          nmesh,         1                    ) \
    X   ( int,     mesh_vertnum,          nmesh,         1                    ) \
    X   ( int,     mesh_faceadr,          nmesh,         1                    ) \
    X   ( int,     mesh_facenum,          nmesh,         1                    ) \
    X   ( int,     mesh_bvhadr,           nmesh,         1                    ) \
    X   ( int,     mesh_bvhnum,           nmesh,         1                    ) \
    X   ( int,     mesh_octadr,           nmesh,         1                    ) \
    X   ( int,     mesh_octnum,           nmesh,         1                    ) \
    X   ( int,     mesh_normaladr,        nmesh,         1                    ) \
    X   ( int,     mesh_normalnum,        nmesh,         1                    ) \
    X   ( int,     mesh_texcoordadr,      nmesh,         1                    ) \
    X   ( int,     mesh_texcoordnum,      nmesh,         1                    ) \
    X   ( int,     mesh_graphadr,         nmesh,         1                    ) \
    XNV ( float,   mesh_vert,             nmeshvert,     3                    ) \
    XNV ( float,   mesh_normal,           nmeshnormal,   3                    ) \
    XNV ( float,   mesh_texcoord,         nmeshtexcoord, 2                    ) \
    XNV ( int,     mesh_face,             nmeshface,     3                    ) \
    XNV ( int,     mesh_facenormal,       nmeshface,     3                    ) \
    XNV ( int,     mesh_facetexcoord,     nmeshface,     3                    ) \
    XNV ( int,     mesh_graph,            nmeshgraph,    1                    ) \
    X   ( sim_scalar_t,  mesh_scale,            nmesh,         3                    ) \
    X   ( sim_scalar_t,  mesh_pos,              nmesh,         3                    ) \
    X   ( sim_scalar_t,  mesh_quat,             nmesh,         4                    ) \
    X   ( int,     mesh_pathadr,          nmesh,         1                    ) \
    XNV ( int,     mesh_polynum,          nmesh,         1                    ) \
    XNV ( int,     mesh_polyadr,          nmesh,         1                    ) \
    XNV ( sim_scalar_t,  mesh_polynormal,       nmeshpoly,     3                    ) \
    XNV ( int,     mesh_polyvertadr,      nmeshpoly,     1                    ) \
    XNV ( int,     mesh_polyvertnum,      nmeshpoly,     1                    ) \
    XNV ( int,     mesh_polyvert,         nmeshpolyvert, 1                    ) \
    XNV ( int,     mesh_polymapadr,       nmeshvert,     1                    ) \
    XNV ( int,     mesh_polymapnum,       nmeshvert,     1                    ) \
    XNV ( int,     mesh_polymap,          nmeshpolymap,  1                    )

#define SIMMODEL_POINTERS_SKIN                                                   \
    X   ( int,     skin_matid,            nskin,         1                    ) \
    X   ( int,     skin_group,            nskin,         1                    ) \
    X   ( float,   skin_rgba,             nskin,         4                    ) \
    X   ( float,   skin_inflate,          nskin,         1                    ) \
    X   ( int,     skin_vertadr,          nskin,         1                    ) \
    X   ( int,     skin_vertnum,          nskin,         1                    ) \
    X   ( int,     skin_texcoordadr,      nskin,         1                    ) \
    X   ( int,     skin_faceadr,          nskin,         1                    ) \
    X   ( int,     skin_facenum,          nskin,         1                    ) \
    X   ( int,     skin_boneadr,          nskin,         1                    ) \
    X   ( int,     skin_bonenum,          nskin,         1                    ) \
    X   ( float,   skin_vert,             nskinvert,     3                    ) \
    X   ( float,   skin_texcoord,         nskintexvert,  2                    ) \
    X   ( int,     skin_face,             nskinface,     3                    ) \
    X   ( int,     skin_bonevertadr,      nskinbone,     1                    ) \
    X   ( int,     skin_bonevertnum,      nskinbone,     1                    ) \
    X   ( float,   skin_bonebindpos,      nskinbone,     3                    ) \
    X   ( float,   skin_bonebindquat,     nskinbone,     4                    ) \
    X   ( int,     skin_bonebodyid,       nskinbone,     1                    ) \
    X   ( int,     skin_bonevertid,       nskinbonevert, 1                    ) \
    X   ( float,   skin_bonevertweight,   nskinbonevert, 1                    ) \
    X   ( int,     skin_pathadr,          nskin,         1                    )

#define SIMMODEL_POINTERS_HFIELD                                                 \
    X   ( sim_scalar_t,  hfield_size,           nhfield,       4                    ) \
    X   ( int,     hfield_nrow,           nhfield,       1                    ) \
    X   ( int,     hfield_ncol,           nhfield,       1                    ) \
    X   ( int,     hfield_adr,            nhfield,       1                    ) \
    XNV ( float,   hfield_data,           nhfielddata,   1                    ) \
    X   ( int,     hfield_pathadr,        nhfield,       1                    )

#define SIMMODEL_POINTERS_TEXTURE                                                \
    X   ( int,     tex_type,              ntex,          1                    ) \
    X   ( int,     tex_colorspace,        ntex,          1                    ) \
    X   ( int,     tex_height,            ntex,          1                    ) \
    X   ( int,     tex_width,             ntex,          1                    ) \
    X   ( int,     tex_nchannel,          ntex,          1                    ) \
    X   ( sim_size_t, tex_adr,               ntex,          1                    ) \
    XNV ( sim_byte_t, tex_data,              ntexdata,      1                    ) \
    X   ( int,     tex_pathadr,           ntex,          1                    )

#define SIMMODEL_POINTERS_MATERIAL                                               \
    X   ( int,     mat_texid,             nmat,          SIM_NTEXROLE           ) \
    X   ( sim_byte_t, mat_texuniform,        nmat,          1                    ) \
    X   ( float,   mat_texrepeat,         nmat,          2                    ) \
    X   ( float,   mat_emission,          nmat,          1                    ) \
    X   ( float,   mat_specular,          nmat,          1                    ) \
    X   ( float,   mat_shininess,         nmat,          1                    ) \
    X   ( float,   mat_reflectance,       nmat,          1                    ) \
    X   ( float,   mat_metallic,          nmat,          1                    ) \
    X   ( float,   mat_roughness,         nmat,          1                    ) \
    X   ( float,   mat_rgba,              nmat,          4                    )

#define SIMMODEL_POINTERS_PAIR                                                   \
    X   ( int,     pair_dim,              npair,         1                    ) \
    X   ( int,     pair_geom1,            npair,         1                    ) \
    X   ( int,     pair_geom2,            npair,         1                    ) \
    X   ( int,     pair_signature,        npair,         1                    ) \
    X   ( sim_scalar_t,  pair_solref,           npair,         SIM_NREF               ) \
    X   ( sim_scalar_t,  pair_solreffriction,   npair,         SIM_NREF               ) \
    X   ( sim_scalar_t,  pair_solimp,           npair,         SIM_NIMP               ) \
    X   ( sim_scalar_t,  pair_margin,           npair,         1                    ) \
    X   ( sim_scalar_t,  pair_gap,              npair,         1                    ) \
    X   ( sim_scalar_t,  pair_friction,         npair,         5                    )

#define SIMMODEL_POINTERS_EXCLUDE                                                \
    X   ( int,     exclude_signature,     nexclude,      1                    )

#define SIMMODEL_POINTERS_EQUALITY                                               \
    X   ( int,     eq_type,               neq,           1                    ) \
    X   ( int,     eq_obj1id,             neq,           1                    ) \
    X   ( int,     eq_obj2id,             neq,           1                    ) \
    X   ( int,     eq_objtype,            neq,           1                    ) \
    X   ( sim_byte_t, eq_active0,            neq,           1                    ) \
    X   ( sim_scalar_t,  eq_solref,             neq,           SIM_NREF               ) \
    X   ( sim_scalar_t,  eq_solimp,             neq,           SIM_NIMP               ) \
    X   ( sim_scalar_t,  eq_data,               neq,           SIM_NEQDATA            )

#define SIMMODEL_POINTERS_TENDON                                                 \
    X   ( int,     tendon_adr,            ntendon,       1                    ) \
    X   ( int,     tendon_num,            ntendon,       1                    ) \
    X   ( int,     tendon_matid,          ntendon,       1                    ) \
    X   ( int,     tendon_group,          ntendon,       1                    ) \
    X   ( int,     tendon_treenum,        ntendon,       1                    ) \
    X   ( int,     tendon_treeid,         ntendon,       2                    ) \
    X   ( sim_byte_t, tendon_limited,        ntendon,       1                    ) \
    X   ( sim_byte_t, tendon_actfrclimited,  ntendon,       1                    ) \
    X   ( sim_scalar_t,  tendon_width,          ntendon,       1                    ) \
    X   ( sim_scalar_t,  tendon_solref_lim,     ntendon,       SIM_NREF               ) \
    X   ( sim_scalar_t,  tendon_solimp_lim,     ntendon,       SIM_NIMP               ) \
    X   ( sim_scalar_t,  tendon_solref_fri,     ntendon,       SIM_NREF               ) \
    X   ( sim_scalar_t,  tendon_solimp_fri,     ntendon,       SIM_NIMP               ) \
    X   ( sim_scalar_t,  tendon_range,          ntendon,       2                    ) \
    X   ( sim_scalar_t,  tendon_actfrcrange,    ntendon,       2                    ) \
    X   ( sim_scalar_t,  tendon_margin,         ntendon,       1                    ) \
    X   ( sim_scalar_t,  tendon_stiffness,      ntendon,       1                    ) \
    X   ( sim_scalar_t,  tendon_damping,        ntendon,       1                    ) \
    X   ( sim_scalar_t,  tendon_armature,       ntendon,       1                    ) \
    X   ( sim_scalar_t,  tendon_frictionloss,   ntendon,       1                    ) \
    X   ( sim_scalar_t,  tendon_lengthspring,   ntendon,       2                    ) \
    X   ( sim_scalar_t,  tendon_length0,        ntendon,       1                    ) \
    X   ( sim_scalar_t,  tendon_invweight0,     ntendon,       1                    ) \
    X   ( sim_scalar_t,  tendon_user,           ntendon,       SIM_M(nuser_tendon)   ) \
    X   ( float,   tendon_rgba,           ntendon,       4                    )

#define SIMMODEL_POINTERS_ACTUATOR                                               \
    X   ( int,     actuator_trntype,      nu,            1                    ) \
    X   ( int,     actuator_dyntype,      nu,            1                    ) \
    X   ( int,     actuator_gaintype,     nu,            1                    ) \
    X   ( int,     actuator_biastype,     nu,            1                    ) \
    X   ( int,     actuator_trnid,        nu,            2                    ) \
    X   ( int,     actuator_actadr,       nu,            1                    ) \
    X   ( int,     actuator_actnum,       nu,            1                    ) \
    X   ( int,     actuator_group,        nu,            1                    ) \
    X   ( int,     actuator_history,      nu,            2                    ) \
    X   ( int,     actuator_historyadr,   nu,            1                    ) \
    X   ( sim_scalar_t,  actuator_delay,        nu,            1                    ) \
    X   ( sim_byte_t, actuator_ctrllimited,  nu,            1                    ) \
    X   ( sim_byte_t, actuator_forcelimited, nu,            1                    ) \
    X   ( sim_byte_t, actuator_actlimited,   nu,            1                    ) \
    X   ( sim_scalar_t,  actuator_dynprm,       nu,            SIM_NDYN               ) \
    X   ( sim_scalar_t,  actuator_gainprm,      nu,            SIM_NGAIN              ) \
    X   ( sim_scalar_t,  actuator_biasprm,      nu,            SIM_NBIAS              ) \
    X   ( sim_byte_t, actuator_actearly,     nu,            1                    ) \
    X   ( sim_scalar_t,  actuator_ctrlrange,    nu,            2                    ) \
    X   ( sim_scalar_t,  actuator_forcerange,   nu,            2                    ) \
    X   ( sim_scalar_t,  actuator_actrange,     nu,            2                    ) \
    X   ( sim_scalar_t,  actuator_gear,         nu,            6                    ) \
    X   ( sim_scalar_t,  actuator_cranklength,  nu,            1                    ) \
    X   ( sim_scalar_t,  actuator_acc0,         nu,            1                    ) \
    X   ( sim_scalar_t,  actuator_length0,      nu,            1                    ) \
    X   ( sim_scalar_t,  actuator_lengthrange,  nu,            2                    ) \
    X   ( sim_scalar_t,  actuator_user,         nu,            SIM_M(nuser_actuator) ) \
    X   ( int,     actuator_plugin,       nu,            1                    )

#define SIMMODEL_POINTERS_SENSOR                                                 \
    X   ( int,     sensor_type,           nsensor,       1                    ) \
    X   ( int,     sensor_datatype,       nsensor,       1                    ) \
    X   ( int,     sensor_needstage,      nsensor,       1                    ) \
    X   ( int,     sensor_objtype,        nsensor,       1                    ) \
    X   ( int,     sensor_objid,          nsensor,       1                    ) \
    X   ( int,     sensor_reftype,        nsensor,       1                    ) \
    X   ( int,     sensor_refid,          nsensor,       1                    ) \
    X   ( int,     sensor_intprm,         nsensor,       SIM_NSENS              ) \
    X   ( int,     sensor_dim,            nsensor,       1                    ) \
    X   ( int,     sensor_adr,            nsensor,       1                    ) \
    X   ( sim_scalar_t,  sensor_cutoff,         nsensor,       1                    ) \
    X   ( sim_scalar_t,  sensor_noise,          nsensor,       1                    ) \
    X   ( int,     sensor_history,        nsensor,       2                    ) \
    X   ( int,     sensor_historyadr,     nsensor,       1                    ) \
    X   ( sim_scalar_t,  sensor_delay,          nsensor,       1                    ) \
    X   ( sim_scalar_t,  sensor_interval,       nsensor,       2                    ) \
    X   ( sim_scalar_t,  sensor_user,           nsensor,       SIM_M(nuser_sensor)   ) \
    X   ( int,     sensor_plugin,         nsensor,       1                    )

#define SIMMODEL_POINTERS                                                        \
    X   ( sim_scalar_t,  qpos0,                 nq,            1                    ) \
    X   ( sim_scalar_t,  qpos_spring,           nq,            1                    ) \
    SIMMODEL_POINTERS_BODY                                                       \
    X   ( int,     bvh_depth,             nbvh,          1                    ) \
    X   ( int,     bvh_child,             nbvh,          2                    ) \
    X   ( int,     bvh_nodeid,            nbvh,          1                    ) \
    X   ( sim_scalar_t,  bvh_aabb,              nbvhstatic,    6                    ) \
    X   ( int,     oct_depth,             noct,          1                    ) \
    X   ( int,     oct_child,             noct,          8                    ) \
    X   ( sim_scalar_t,  oct_aabb,              noct,          6                    ) \
    X   ( sim_scalar_t,  oct_coeff,             noct,          8                    ) \
    SIMMODEL_POINTERS_JOINT                                                      \
    SIMMODEL_POINTERS_DOF                                                        \
    SIMMODEL_POINTERS_TREE                                                       \
    SIMMODEL_POINTERS_GEOM                                                       \
    SIMMODEL_POINTERS_SITE                                                       \
    SIMMODEL_POINTERS_CAMERA                                                     \
    SIMMODEL_POINTERS_LIGHT                                                      \
    SIMMODEL_POINTERS_FLEX                                                       \
    SIMMODEL_POINTERS_MESH                                                       \
    SIMMODEL_POINTERS_SKIN                                                       \
    SIMMODEL_POINTERS_HFIELD                                                     \
    SIMMODEL_POINTERS_TEXTURE                                                    \
    SIMMODEL_POINTERS_MATERIAL                                                   \
    SIMMODEL_POINTERS_PAIR                                                       \
    SIMMODEL_POINTERS_EXCLUDE                                                    \
    SIMMODEL_POINTERS_EQUALITY                                                   \
    SIMMODEL_POINTERS_TENDON                                                     \
    X   ( int,     wrap_type,             nwrap,         1                    ) \
    X   ( int,     wrap_objid,            nwrap,         1                    ) \
    X   ( sim_scalar_t,  wrap_prm,              nwrap,         1                    ) \
    SIMMODEL_POINTERS_ACTUATOR                                                   \
    SIMMODEL_POINTERS_SENSOR                                                     \
    X   ( int,     plugin,                nplugin,       1                    ) \
    X   ( int,     plugin_stateadr,       nplugin,       1                    ) \
    X   ( int,     plugin_statenum,       nplugin,       1                    ) \
    X   ( char,    plugin_attr,           npluginattr,   1                    ) \
    X   ( int,     plugin_attradr,        nplugin,       1                    ) \
    X   ( int,     numeric_adr,           nnumeric,      1                    ) \
    X   ( int,     numeric_size,          nnumeric,      1                    ) \
    X   ( sim_scalar_t,  numeric_data,          nnumericdata,  1                    ) \
    X   ( int,     text_adr,              ntext,         1                    ) \
    X   ( int,     text_size,             ntext,         1                    ) \
    X   ( char,    text_data,             ntextdata,     1                    ) \
    X   ( int,     tuple_adr,             ntuple,        1                    ) \
    X   ( int,     tuple_size,            ntuple,        1                    ) \
    X   ( int,     tuple_objtype,         ntupledata,    1                    ) \
    X   ( int,     tuple_objid,           ntupledata,    1                    ) \
    X   ( sim_scalar_t,  tuple_objprm,          ntupledata,    1                    ) \
    X   ( sim_scalar_t,  key_time,              nkey,          1                    ) \
    X   ( sim_scalar_t,  key_qpos,              nkey,          SIM_M(nq)             ) \
    X   ( sim_scalar_t,  key_qvel,              nkey,          SIM_M(nv)             ) \
    X   ( sim_scalar_t,  key_act,               nkey,          SIM_M(na)             ) \
    X   ( sim_scalar_t,  key_mpos,              nkey,          SIM_M(nmocap)*3       ) \
    X   ( sim_scalar_t,  key_mquat,             nkey,          SIM_M(nmocap)*4       ) \
    X   ( sim_scalar_t,  key_ctrl,              nkey,          SIM_M(nu)             ) \
    X   ( int,     name_bodyadr,          nbody,         1                    ) \
    X   ( int,     name_jntadr,           njnt,          1                    ) \
    X   ( int,     name_geomadr,          ngeom,         1                    ) \
    X   ( int,     name_siteadr,          nsite,         1                    ) \
    X   ( int,     name_camadr,           ncam,          1                    ) \
    X   ( int,     name_lightadr,         nlight,        1                    ) \
    X   ( int,     name_flexadr,          nflex,         1                    ) \
    X   ( int,     name_meshadr,          nmesh,         1                    ) \
    X   ( int,     name_skinadr,          nskin,         1                    ) \
    X   ( int,     name_hfieldadr,        nhfield,       1                    ) \
    X   ( int,     name_texadr,           ntex,          1                    ) \
    X   ( int,     name_matadr,           nmat,          1                    ) \
    X   ( int,     name_pairadr,          npair,         1                    ) \
    X   ( int,     name_excludeadr,       nexclude,      1                    ) \
    X   ( int,     name_eqadr,            neq,           1                    ) \
    X   ( int,     name_tendonadr,        ntendon,       1                    ) \
    X   ( int,     name_actuatoradr,      nu,            1                    ) \
    X   ( int,     name_sensoradr,        nsensor,       1                    ) \
    X   ( int,     name_numericadr,       nnumeric,      1                    ) \
    X   ( int,     name_textadr,          ntext,         1                    ) \
    X   ( int,     name_tupleadr,         ntuple,        1                    ) \
    X   ( int,     name_keyadr,           nkey,          1                    ) \
    X   ( int,     name_pluginadr,        nplugin,       1                    ) \
    X   ( char,    names,                 nnames,        1                    ) \
    X   ( int,     names_map,             nnames_map,    1                    ) \
    X   ( char,    paths,                 npaths,        1                    ) \
    X   ( int,     B_rownnz,              nbody,         1                    ) \
    X   ( int,     B_rowadr,              nbody,         1                    ) \
    X   ( int,     B_colind,              nB,            1                    ) \
    X   ( int,     M_rownnz,              nv,            1                    ) \
    X   ( int,     M_rowadr,              nv,            1                    ) \
    X   ( int,     M_colind,              nC,            1                    ) \
    X   ( int,     mapM2M,                nC,            1                    ) \
    X   ( int,     D_rownnz,              nv,            1                    ) \
    X   ( int,     D_rowadr,              nv,            1                    ) \
    X   ( int,     D_diag,                nv,            1                    ) \
    X   ( int,     D_colind,              nD,            1                    ) \
    X   ( int,     mapM2D,                nD,            1                    ) \
    X   ( int,     mapD2M,                nC,            1                    )

//-------------------------------- sim_data_t ----------------------------------------------------------

// pointer fields of sim_data_t
// XNV marks optional fields in specialized X-macro expansions.
// By default we define XNV to be the same as X.
#define SIMDATA_POINTERS                                            \
    X   ( sim_scalar_t,    qpos,              nq,          1           ) \
    X   ( sim_scalar_t,    qvel,              nv,          1           ) \
    X   ( sim_scalar_t,    act,               na,          1           ) \
    X   ( sim_scalar_t,    history,           nhistory,    1           ) \
    X   ( sim_scalar_t,    qacc_warmstart,    nv,          1           ) \
    X   ( sim_scalar_t,    plugin_state,      npluginstate, 1          ) \
    X   ( sim_scalar_t,    ctrl,              nu,          1           ) \
    X   ( sim_scalar_t,    qfrc_applied,      nv,          1           ) \
    X   ( sim_scalar_t,    xfrc_applied,      nbody,       6           ) \
    X   ( sim_byte_t,   eq_active,         neq,         1           ) \
    X   ( sim_scalar_t,    mocap_pos,         nmocap,      3           ) \
    X   ( sim_scalar_t,    mocap_quat,        nmocap,      4           ) \
    X   ( sim_scalar_t,    qacc,              nv,          1           ) \
    X   ( sim_scalar_t,    act_dot,           na,          1           ) \
    X   ( sim_scalar_t,    userdata,          nuserdata,   1           ) \
    X   ( sim_scalar_t,    sensordata,        nsensordata, 1           ) \
    X   ( int,       tree_asleep,       ntree,       1           ) \
    X   ( int,       plugin,            nplugin,     1           ) \
    X   ( uintptr_t, plugin_data,       nplugin,     1           ) \
    X   ( sim_scalar_t,    xpos,              nbody,       3           ) \
    X   ( sim_scalar_t,    xquat,             nbody,       4           ) \
    X   ( sim_scalar_t,    xmat,              nbody,       9           ) \
    X   ( sim_scalar_t,    xipos,             nbody,       3           ) \
    X   ( sim_scalar_t,    ximat,             nbody,       9           ) \
    X   ( sim_scalar_t,    xanchor,           njnt,        3           ) \
    X   ( sim_scalar_t,    xaxis,             njnt,        3           ) \
    X   ( sim_scalar_t,    geom_xpos,         ngeom,       3           ) \
    X   ( sim_scalar_t,    geom_xmat,         ngeom,       9           ) \
    X   ( sim_scalar_t,    site_xpos,         nsite,       3           ) \
    X   ( sim_scalar_t,    site_xmat,         nsite,       9           ) \
    X   ( sim_scalar_t,    cam_xpos,          ncam,        3           ) \
    X   ( sim_scalar_t,    cam_xmat,          ncam,        9           ) \
    X   ( sim_scalar_t,    light_xpos,        nlight,      3           ) \
    X   ( sim_scalar_t,    light_xdir,        nlight,      3           ) \
    X   ( sim_scalar_t,    subtree_com,       nbody,       3           ) \
    X   ( sim_scalar_t,    cdof,              nv,          6           ) \
    X   ( sim_scalar_t,    cinert,            nbody,       10          ) \
    X   ( sim_scalar_t,    flexvert_xpos,     nflexvert,   3           ) \
    X   ( sim_scalar_t,    flexelem_aabb,     nflexelem,   6           ) \
    X   ( sim_scalar_t,    flexedge_J,        nJfe,        1           ) \
    X   ( sim_scalar_t,    flexedge_length,   nflexedge,   1           ) \
    X   ( sim_scalar_t,    flexvert_J,        nJfv,        2           ) \
    X   ( sim_scalar_t,    flexvert_length,   nflexvert,   2           ) \
    X   ( sim_scalar_t,    bvh_aabb_dyn,      nbvhdynamic, 6           ) \
    X   ( int,       ten_wrapadr,       ntendon,     1           ) \
    X   ( int,       ten_wrapnum,       ntendon,     1           ) \
    X   ( int,       ten_J_rownnz,      ntendon,     1           ) \
    X   ( int,       ten_J_rowadr,      ntendon,     1           ) \
    X   ( int,       ten_J_colind,      nJten,       1           ) \
    X   ( sim_scalar_t,    ten_J,             nJten,       1           ) \
    X   ( sim_scalar_t,    ten_length,        ntendon,     1           ) \
    X   ( int,       wrap_obj,          nwrap,       2           ) \
    X   ( sim_scalar_t,    wrap_xpos,         nwrap,       6           ) \
    X   ( sim_scalar_t,    actuator_length,   nu,          1           ) \
    X   ( int,       moment_rownnz,     nu,          1           ) \
    X   ( int,       moment_rowadr,     nu,          1           ) \
    X   ( int,       moment_colind,     nJmom,       1           ) \
    X   ( sim_scalar_t,    actuator_moment,   nJmom,       1           ) \
    XNV ( sim_scalar_t,    crb,               nbody,       10          ) \
    XNV ( sim_scalar_t,    qM,                nM,          1           ) \
    XNV ( sim_scalar_t,    M,                 nC,          1           ) \
    XNV ( sim_scalar_t,    qLD,               nC,          1           ) \
    X   ( sim_scalar_t,    qLDiagInv,         nv,          1           ) \
    X   ( sim_byte_t,   bvh_active,        nbvh,        1           ) \
    X   ( int,       tree_awake,        ntree,       1           ) \
    X   ( int,       body_awake,        nbody,       1           ) \
    X   ( int,       body_awake_ind,    nbody,       1           ) \
    X   ( int,       parent_awake_ind,  nbody,       1           ) \
    X   ( int,       dof_awake_ind,     nv,          1           ) \
    X   ( sim_scalar_t,    flexedge_velocity, nflexedge,   1           ) \
    X   ( sim_scalar_t,    ten_velocity,      ntendon,     1           ) \
    X   ( sim_scalar_t,    actuator_velocity, nu,          1           ) \
    X   ( sim_scalar_t,    cvel,              nbody,       6           ) \
    X   ( sim_scalar_t,    cdof_dot,          nv,          6           ) \
    X   ( sim_scalar_t,    qfrc_bias,         nv,          1           ) \
    X   ( sim_scalar_t,    qfrc_spring,       nv,          1           ) \
    X   ( sim_scalar_t,    qfrc_damper,       nv,          1           ) \
    X   ( sim_scalar_t,    qfrc_gravcomp,     nv,          1           ) \
    X   ( sim_scalar_t,    qfrc_fluid,        nv,          1           ) \
    X   ( sim_scalar_t,    qfrc_passive,      nv,          1           ) \
    X   ( sim_scalar_t,    subtree_linvel,    nbody,       3           ) \
    X   ( sim_scalar_t,    subtree_angmom,    nbody,       3           ) \
    XNV ( sim_scalar_t,    qH,                nC,          1           ) \
    X   ( sim_scalar_t,    qHDiagInv,         nv,          1           ) \
    XNV ( sim_scalar_t,    qDeriv,            nD,          1           ) \
    XNV ( sim_scalar_t,    qLU,               nD,          1           ) \
    X   ( sim_scalar_t,    actuator_force,    nu,          1           ) \
    X   ( sim_scalar_t,    qfrc_actuator,     nv,          1           ) \
    X   ( sim_scalar_t,    qfrc_smooth,       nv,          1           ) \
    X   ( sim_scalar_t,    qacc_smooth,       nv,          1           ) \
    X   ( sim_scalar_t,    qfrc_constraint,   nv,          1           ) \
    X   ( sim_scalar_t,    qfrc_inverse,      nv,          1           ) \
    X   ( sim_scalar_t,    cacc,              nbody,       6           ) \
    X   ( sim_scalar_t,    cfrc_int,          nbody,       6           ) \
    X   ( sim_scalar_t,    cfrc_ext,          nbody,       6           )


// macro for annotating that an array size in an X macro is a member of sim_data_t
// by default this macro does nothing, but users can redefine it as necessary
#define SIM_D(n) n

// array of contacts
#define SIMDATA_ARENA_POINTERS_CONTACT \
    X( sim_contact_t, contact, SIM_D(ncon), 1 )

// array fields of sim_data_t that are used in the primal problem
#define SIMDATA_ARENA_POINTERS_SOLVER                     \
    X  ( int,      efc_type,          SIM_D(nefc),    1 ) \
    X  ( int,      efc_id,            SIM_D(nefc),    1 ) \
    XNV( int,      efc_J_rownnz,      SIM_D(nefc),    1 ) \
    XNV( int,      efc_J_rowadr,      SIM_D(nefc),    1 ) \
    XNV( int,      efc_J_rowsuper,    SIM_D(nefc),    1 ) \
    XNV( int,      efc_J_colind,      SIM_D(nJ),      1 ) \
    XNV( sim_scalar_t,   efc_J,             SIM_D(nJ),      1 ) \
    X  ( sim_scalar_t,   efc_pos,           SIM_D(nefc),    1 ) \
    X  ( sim_scalar_t,   efc_margin,        SIM_D(nefc),    1 ) \
    X  ( sim_scalar_t,   efc_frictionloss,  SIM_D(nefc),    1 ) \
    X  ( sim_scalar_t,   efc_diagApprox,    SIM_D(nefc),    1 ) \
    X  ( sim_scalar_t,   efc_KBIP,          SIM_D(nefc),    4 ) \
    X  ( sim_scalar_t,   efc_D,             SIM_D(nefc),    1 ) \
    X  ( sim_scalar_t,   efc_R,             SIM_D(nefc),    1 ) \
    X  ( int,      tendon_efcadr,     SIM_M(ntendon), 1 ) \
    X  ( sim_scalar_t,   efc_vel,           SIM_D(nefc),    1 ) \
    X  ( sim_scalar_t,   efc_aref,          SIM_D(nefc),    1 ) \
    X  ( sim_scalar_t,   efc_b,             SIM_D(nefc),    1 ) \
    X  ( int,      efc_state,         SIM_D(nefc),    1 ) \
    X  ( sim_scalar_t,   efc_force,         SIM_D(nefc),    1 )

// array fields of sim_data_t that are used in the dual problem
#define SIMDATA_ARENA_POINTERS_DUAL                       \
    XNV( int,      efc_AR_rownnz,     SIM_D(nefc),    1 ) \
    XNV( int,      efc_AR_rowadr,     SIM_D(nefc),    1 ) \
    XNV( int,      efc_AR_colind,     SIM_D(nA),      1 ) \
    XNV( sim_scalar_t,   efc_AR,            SIM_D(nA),      1 )

// array fields of sim_data_t that are used for constraint islands
#define SIMDATA_ARENA_POINTERS_ISLAND                     \
    X  ( int,     tree_island,       SIM_M(ntree),    1 ) \
    X  ( int,     island_ntree,      SIM_D(nisland),  1 ) \
    X  ( int,     island_itreeadr,   SIM_D(nisland),  1 ) \
    X  ( int,     map_itree2tree,    SIM_M(ntree),    1 ) \
    X  ( int,     dof_island,        SIM_M(nv),       1 ) \
    X  ( int,     island_nv,         SIM_D(nisland),  1 ) \
    X  ( int,     island_idofadr,    SIM_D(nisland),  1 ) \
    X  ( int,     island_dofadr,     SIM_D(nisland),  1 ) \
    X  ( int,     map_dof2idof,      SIM_M(nv),       1 ) \
    X  ( int,     map_idof2dof,      SIM_M(nv),       1 ) \
    X  ( sim_scalar_t,  ifrc_smooth,       SIM_D(nidof),    1 ) \
    X  ( sim_scalar_t,  iacc_smooth,       SIM_D(nidof),    1 ) \
    XNV( int,     iM_rownnz,         SIM_D(nidof),    1 ) \
    XNV( int,     iM_rowadr,         SIM_D(nidof),    1 ) \
    XNV( int,     iM_colind,         SIM_M(nC),       1 ) \
    XNV( sim_scalar_t,  iM,                SIM_M(nC),       1 ) \
    XNV( sim_scalar_t,  iLD,               SIM_M(nC),       1 ) \
    X  ( sim_scalar_t,  iLDiagInv,         SIM_D(nidof),    1 ) \
    X  ( sim_scalar_t,  iacc,              SIM_D(nidof),    1 ) \
    X  ( int,     efc_island,        SIM_D(nefc),     1 ) \
    X  ( int,     island_ne,         SIM_D(nisland),  1 ) \
    X  ( int,     island_nf,         SIM_D(nisland),  1 ) \
    X  ( int,     island_nefc,       SIM_D(nisland),  1 ) \
    X  ( int,     island_iefcadr,    SIM_D(nisland),  1 ) \
    X  ( int,     map_efc2iefc,      SIM_D(nefc),     1 ) \
    X  ( int,     map_iefc2efc,      SIM_D(nefc),     1 ) \
    X  ( int,     iefc_type,         SIM_D(nefc),     1 ) \
    X  ( int,     iefc_id,           SIM_D(nefc),     1 ) \
    XNV( int,     iefc_J_rownnz,     SIM_D(nefc),     1 ) \
    XNV( int,     iefc_J_rowadr,     SIM_D(nefc),     1 ) \
    XNV( int,     iefc_J_rowsuper,   SIM_D(nefc),     1 ) \
    XNV( int,     iefc_J_colind,     SIM_D(nJ),       1 ) \
    XNV( sim_scalar_t,  iefc_J,            SIM_D(nJ),       1 ) \
    X  ( sim_scalar_t,  iefc_frictionloss, SIM_D(nefc),     1 ) \
    X  ( sim_scalar_t,  iefc_D,            SIM_D(nefc),     1 ) \
    X  ( sim_scalar_t,  iefc_R,            SIM_D(nefc),     1 ) \
    X  ( sim_scalar_t,  iefc_aref,         SIM_D(nefc),     1 ) \
    X  ( int,     iefc_state,        SIM_D(nefc),     1 ) \
    X  ( sim_scalar_t,  iefc_force,        SIM_D(nefc),     1 ) \
    X  ( sim_scalar_t,  ifrc_constraint,   SIM_D(nidof),    1 )

// array fields of sim_data_t that live in d->arena
#define SIMDATA_ARENA_POINTERS          \
    SIMDATA_ARENA_POINTERS_CONTACT      \
    SIMDATA_ARENA_POINTERS_SOLVER       \
    SIMDATA_ARENA_POINTERS_DUAL         \
    SIMDATA_ARENA_POINTERS_ISLAND


// scalar fields of sim_data_t
#define SIMDATA_SCALAR                  \
    X( size_t,    narena             ) \
    X( size_t,    nbuffer            ) \
    X( int,       nplugin            ) \
    X( size_t,    pstack             ) \
    X( size_t,    pbase              ) \
    X( size_t,    parena             ) \
    X( size_t,    maxuse_stack       ) \
    X( size_t,    maxuse_arena       ) \
    X( int,       maxuse_con         ) \
    X( int,       maxuse_efc         ) \
    X( int,       ncon               ) \
    X( int,       ne                 ) \
    X( int,       nf                 ) \
    X( int,       nl                 ) \
    X( int,       nefc               ) \
    X( int,       nJ                 ) \
    X( int,       nA                 ) \
    X( int,       nisland            ) \
    X( int,       nidof              ) \
    X( int,       ntree_awake        ) \
    X( int,       nbody_awake        ) \
    X( int,       nparent_awake      ) \
    X( int,       nv_awake           ) \
    X( sim_byte_t,   flg_energypos      ) \
    X( sim_byte_t,   flg_energyvel      ) \
    X( sim_byte_t,   flg_subtreevel     ) \
    X( sim_byte_t,   flg_rnepost        ) \
    X( sim_scalar_t,    time               ) \
    X( uintptr_t, threadpool         )


// vector fields of sim_data_t
#define SIMDATA_VECTOR                                                \
    X( size_t,         maxuse_threadstack, SIM_MAXTHREAD,  1         ) \
    X( SIM_SolverStat,   solver,             SIM_NISLAND,    SIM_NSOLVER ) \
    X( int,            solver_niter,       SIM_NISLAND,    1         ) \
    X( int,            solver_nnz,         SIM_NISLAND,    1         ) \
    X( sim_scalar_t,         solver_fwdinv,      2,            1         ) \
    X( SIM_WarningStat,  warning,            SIM_NWARNING,   1         ) \
    X( SIM_TimerStat,    timer,              SIM_NTIMER,     1         ) \
    X( sim_scalar_t,         energy,             2,            1         )

// alias XNV to be the same as X.
// Callers can redefine XNV to expand to nothing before including this header.
#define XNV X

#endif  // SIMCORE_SIM_XMACRO_H_
