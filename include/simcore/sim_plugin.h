// Copyright 2022 DeepMind Technologies Limited
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

#ifndef SIMCORE_INCLUDE_SIM_PLUGIN_H_
#define SIMCORE_INCLUDE_SIM_PLUGIN_H_

#include <simcore/SIM_data.h>
#include <simcore/SIM_model.h>
#include <simcore/SIM_spec.h>
#include <simcore/SIM_tnum.h>


//---------------------------------- Resource Provider ---------------------------------------------

struct SIM_Resource_ {
  char* name;                                   // name of resource (filename, etc)
  void* data;                                   // opaque data pointer
  SIM_VFS* vfs;                                   // pointer to the VFS
  char timestamp[512];                          // timestamp of the resource
  const struct SIM_pResourceProvider* provider;   // pointer to the provider
};
typedef struct SIM_Resource_ SIM_Resource;

// callback for opening a resource, returns zero on failure
typedef int (*SIM_fOpenResource)(SIM_Resource* resource);

// callback for reading a resource
// return number of bytes stored in buffer, return -1 if error
typedef int (*SIM_fReadResource)(SIM_Resource* resource, const void** buffer);

// callback for closing a resource (responsible for freeing any allocated memory)
typedef void (*SIM_fCloseResource)(SIM_Resource* resource);

// callback for mounting a resource (provider), returns zero on failure
typedef int (*SIM_fMountResource)(SIM_Resource* resource);

// callback for unmounting a resource (provider), returns zero on failure
typedef int (*SIM_fUnmountResource)(SIM_Resource* resource);

// callback for checking if the current resource was modified from the time
// specified by the timestamp
// returns 0 if the resource's timestamp matches the provided timestamp
// returns > 0 if the resource is younger than the given timestamp
// returns < 0 if the resource is older than the given timestamp
typedef int (*SIM_fResourceModified)(const SIM_Resource* resource, const char* timestamp);

// struct describing a single resource provider
struct SIM_pResourceProvider {
  const char* prefix;               // prefix for match against a resource name
  SIM_fOpenResource open;             // opening callback
  SIM_fReadResource read;             // reading callback
  SIM_fCloseResource close;           // closing callback
  SIM_fMountResource mount;           // mounting callback (optional)
  SIM_fUnmountResource unmount;       // unmounting callback (optional)
  SIM_fResourceModified modified;     // resource modified callback (optional)
  void* data;                       // opaque data pointer (resource invariant)
};
typedef struct SIM_pResourceProvider SIM_pResourceProvider;

//---------------------------------- Decoder -------------------------------------------------------

// function pointer types
// return an sim_spec_t representing the decoded resource.
typedef sim_spec_t* (*SIM_fDecode)(SIM_Resource* resource, const SIM_VFS* vfs);
// return true if the given resource can be decoded.
typedef int (*SIM_fCanDecode)(const SIM_Resource* resource);

// the struct defining the decoder plugin's interface
struct SIM_pDecoder {
  const char* content_type;
  const char* extension;
  // user-facing functions
  SIM_fCanDecode can_decode;  // quickly check if this decoder can handle the resource
  SIM_fDecode decode;         // main decoding function
  // the caller takes ownership of the spec returned by decode and is responsible
  // for cleaning it up
};
typedef struct SIM_pDecoder SIM_pDecoder;

//---------------------------------- Plugins -------------------------------------------------------

typedef enum SIM_tPluginCapabilityBit_ {
  SIM_PLUGIN_ACTUATOR = 1<<0,       // actuator forces
  SIM_PLUGIN_SENSOR   = 1<<1,       // sensor measurements
  SIM_PLUGIN_PASSIVE  = 1<<2,       // passive forces
  SIM_PLUGIN_SDF      = 1<<3,       // signed distance fields
} SIM_tPluginCapabilityBit;

struct SIM_pPlugin_ {
  const char* name;               // globally unique name identifying the plugin

  int nattribute;                 // number of configuration attributes
  const char* const* attributes;  // name of configuration attributes

  int capabilityflags;            // plugin capabilities: bitfield of SIM_tPluginCapabilityBit
  int needstage;                  // sensor computation stage (SIM_tStage)

  // number of SIM_tNums needed to store the state of a plugin instance (required)
  int (*nstate)(const sim_model_t* m, int instance);

  // dimension of the specified sensor's output (required only for sensor plugins)
  int (*nsensordata)(const sim_model_t* m, int instance, int sensor_id);

  // called when a new sim_data_t is being created (required), returns 0 on success or -1 on failure
  int (*init)(const sim_model_t* m, sim_data_t* d, int instance);

  // called when an sim_data_t is being freed (optional)
  void (*destroy)(sim_data_t* d, int instance);

  // called when an sim_data_t is being copied (optional)
  void (*copy)(sim_data_t* dest, const sim_model_t* m, const sim_data_t* src, int instance);

  // called when an sim_data_t is being reset (required)
  void (*reset)(const sim_model_t* m, sim_scalar_t* plugin_state, void* plugin_data, int instance);

  // called when the plugin needs to update its outputs (required)
  void (*compute)(const sim_model_t* m, sim_data_t* d, int instance, int capability_bit);

  // called when time integration occurs (optional)
  void (*advance)(const sim_model_t* m, sim_data_t* d, int instance);

  // methods specific to actuators (optional)

  // updates the actuator plugin's entries in act_dot
  // called after native act_dot is computed and before the compute callback
  void (*actuator_act_dot)(const sim_model_t* m, sim_data_t* d, int instance);

  // methods specific to signed distance fields (optional)

  // signed distance from the surface
  sim_scalar_t (*sdf_distance)(const sim_scalar_t point[3], const sim_data_t* d, int instance);

  // gradient of distance with respect to local coordinates
  void (*sdf_gradient)(sim_scalar_t gradient[3], const sim_scalar_t point[3], const sim_data_t* d, int instance);

  // called during compilation for marching cubes
  sim_scalar_t (*sdf_staticdistance)(const sim_scalar_t point[3], const sim_scalar_t* attributes);

  // convert attributes and provide defaults if not present
  void (*sdf_attribute)(sim_scalar_t attribute[], const char* name[], const char* value[]);

  // bounding box of implicit surface
  void (*sdf_aabb)(sim_scalar_t aabb[6], const sim_scalar_t* attributes);
};
typedef struct SIM_pPlugin_ SIM_pPlugin;

struct SIM_SDF_ {
  const SIM_pPlugin** plugin;
  int* id;
  SIM_tSDFType type;
  sim_scalar_t* relpos;
  sim_scalar_t* relmat;
  SIM_tGeom* geomtype;
};
typedef struct SIM_SDF_ SIM_SDF;

#if defined(__has_attribute)

  #if __has_attribute(constructor)
    #define SIM_PLUGIN_LIB_INIT __attribute__((constructor)) static void _mjplugin_init(void)
  #endif  // __has_attribute(constructor)

#elif defined(_MSC_VER)

  #ifndef SIM_DLLMAIN
    #define SIM_DLLMAIN DllMain
  #endif

  #if !defined(SIM_EXTERNC)
    #if defined(__cplusplus)
      #define SIM_EXTERNC extern "C"
    #else
      #define SIM_EXTERNC
    #endif  // defined(__cplusplus)
  #endif  // !defined(SIM_EXTERNC)

  // NOLINTBEGIN(runtime/int)
  #define SIM_PLUGIN_LIB_INIT                                                                 \
    static void _mjplugin_dllmain(void);                                                    \
    SIM_EXTERNC int __stdcall SIM_DLLMAIN(void* hinst, unsigned long reason, void* reserved) {  \
      if (reason == 1) {                                                                    \
        _mjplugin_dllmain();                                                                \
      }                                                                                     \
      return 1;                                                                             \
    }                                                                                       \
    static void _mjplugin_dllmain(void)
  // NOLINTEND(runtime/int)

#endif  // defined(_MSC_VER)

// function pointer type for sim_loadAllPluginLibraries callback
typedef void (*SIM_fPluginLibraryLoadCallback)(const char* filename, int first, int count);

#endif  // SIMCORE_INCLUDE_SIM_PLUGIN_H_
