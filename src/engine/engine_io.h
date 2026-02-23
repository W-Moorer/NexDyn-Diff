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

#ifndef SIMCORE_SRC_ENGINE_ENGINE_IO_H_
#define SIMCORE_SRC_ENGINE_ENGINE_IO_H_

#include <simcore/SIM_data.h>
#include <simcore/SIM_export.h>
#include <simcore/SIM_model.h>

#ifdef __cplusplus
#include <cstddef>
extern "C" {
#else
#include <stddef.h>
#endif

// internal hash map size factor (2 corresponds to a load factor of 0.5)
#define SIM_LOAD_MULTIPLE 2

//------------------------------- initialization ---------------------------------------------------

// Set default options for length range computation.
SIM_API void sim_defaultLROpt(SIM_LROpt* opt);

// set options to default values
SIM_API void sim_defaultOption(SIM_Option* opt);

// set statistics to default values; compute later in compiler
void sim_defaultStatistic(SIM_Statistic* stat);


//------------------------------- sim_model_t ----------------------------------------------------------

// allocate sim_model_t
void sim_makeModel(sim_model_t** dest,
    sim_size_t nq, sim_size_t nv, sim_size_t nu, sim_size_t na, sim_size_t nbody, sim_size_t nbvh, sim_size_t nbvhstatic,
    sim_size_t nbvhdynamic, sim_size_t noct, sim_size_t njnt, sim_size_t ntree, sim_size_t nM, sim_size_t nB,
    sim_size_t nC, sim_size_t nD, sim_size_t ngeom, sim_size_t nsite, sim_size_t ncam, sim_size_t nlight,
    sim_size_t nflex, sim_size_t nflexnode, sim_size_t nflexvert, sim_size_t nflexedge, sim_size_t nflexelem,
    sim_size_t nflexelemdata, sim_size_t nflexelemedge, sim_size_t nflexshelldata, sim_size_t nflexevpair,
    sim_size_t nflextexcoord, sim_size_t nJfe, sim_size_t nJfv, sim_size_t nmesh, sim_size_t nmeshvert,
    sim_size_t nmeshnormal, sim_size_t nmeshtexcoord, sim_size_t nmeshface, sim_size_t nmeshgraph,
    sim_size_t nmeshpoly, sim_size_t nmeshpolyvert, sim_size_t nmeshpolymap, sim_size_t nskin,
    sim_size_t nskinvert, sim_size_t nskintexvert, sim_size_t nskinface, sim_size_t nskinbone,
    sim_size_t nskinbonevert, sim_size_t nhfield, sim_size_t nhfielddata, sim_size_t ntex, sim_size_t ntexdata,
    sim_size_t nmat, sim_size_t npair, sim_size_t nexclude, sim_size_t neq, sim_size_t ntendon, sim_size_t nwrap,
    sim_size_t nsensor, sim_size_t nnumeric, sim_size_t nnumericdata, sim_size_t ntext, sim_size_t ntextdata,
    sim_size_t ntuple, sim_size_t ntupledata, sim_size_t nkey, sim_size_t nmocap, sim_size_t nplugin,
    sim_size_t npluginattr, sim_size_t nuser_body, sim_size_t nuser_jnt, sim_size_t nuser_geom,
    sim_size_t nuser_site, sim_size_t nuser_cam, sim_size_t nuser_tendon, sim_size_t nuser_actuator,
    sim_size_t nuser_sensor, sim_size_t nnames, sim_size_t npaths);

// copy sim_model_t; allocate new if dest is NULL
SIM_API sim_model_t* sim_copyModel(sim_model_t* dest, const sim_model_t* src);

// save model to binary file
SIM_API void sim_saveModel(const sim_model_t* m, const char* filename, void* buffer, int buffer_sz);

// load model from binary buffer
SIM_API sim_model_t* sim_loadModelBuffer(const void* buffer, int buffer_sz);

// deallocate model
SIM_API void sim_deleteModel(sim_model_t* m);

// size of buffer needed to hold model
SIM_API sim_size_t sim_sizeModel(const sim_model_t* m);

// validate reference fields in a model; return null if valid, error message otherwise
SIM_API const char* sim_validateReferences(const sim_model_t* m);

// construct sparse representation of dof-dof matrix
SIM_API void sim_makeDofDofSparse(int nv, int nC, int nD, int nM,
                               const int* dof_parentid, const int* dof_simplenum,
                               int* rownnz, int* rowadr, int* diag, int* colind,
                               int reduced, int upper, int* remaining);

// construct sparse representation of body-dof matrix
SIM_API void sim_makeBSparse(int nv, int nbody, int nB,
                          const int* body_dofnum, const int* body_parentid, const int* body_dofadr,
                          int* B_rownnz, int* B_rowadr, int* B_colind,
                          int* count);

// construct index mappings between M <-> D, M (legacy) -> M (CSR)
SIM_API void sim_makeDofDofMaps(int nv, int nM, int nC, int nD,
                             const int* dof_Madr, const int* dof_simplenum, const int* dof_parentid,
                             const int* D_rownnz, const int* D_rowadr, const int* D_colind,
                             const int* M_rownnz, const int* M_rowadr, const int* M_colind,
                             int* mapM2D, int* mapD2M, int* mapM2M,
                             int* M, int* scratch);

//------------------------------- sim_data_t -----------------------------------------------------------

// allocate sim_data_t corresponding to given model, initialize plugins, reset the state
// if the model buffer is unallocated the initial configuration will not be set
SIM_API sim_data_t* sim_makeData(const sim_model_t* m);

// allocate sim_data_t corresponding to given model, used internally
SIM_API void sim_makeRawData(sim_data_t** dest, const sim_model_t* m);

// Copy sim_data_t.
// m is only required to contain the size fields from SIMMODEL_INTS.
SIM_API sim_data_t* sim_copyData(sim_data_t* dest, const sim_model_t* m, const sim_data_t* src);

// set data to defaults
SIM_API void sim_resetData(const sim_model_t* m, sim_data_t* d);

// set data to defaults, fill everything else with debug_value
SIM_API void sim_resetDataDebug(const sim_model_t* m, sim_data_t* d, unsigned char debug_value);

// Reset data. If 0 <= key < nkey, set fields from specified keyframe.
SIM_API void sim_resetDataKeyframe(const sim_model_t* m, sim_data_t* d, int key);

// init plugins
SIM_API void sim_initPlugin(const sim_model_t* m, sim_data_t* d);

// deallocate data
SIM_API void sim_deleteData(sim_data_t* d);

#ifdef __cplusplus
}
#endif

#endif  // SIMCORE_SRC_ENGINE_ENGINE_IO_H_
