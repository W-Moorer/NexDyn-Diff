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

#ifndef SIMCORE_SRC_ENGINE_ENGINE_PLUGIN_H_
#define SIMCORE_SRC_ENGINE_ENGINE_PLUGIN_H_

#include <simcore/SIM_export.h>
#include <simcore/SIM_plugin.h>

#ifdef __cplusplus
extern "C" {
#endif

// set default plugin definition
SIM_API void sim_plugin_defaultPlugin(SIM_pPlugin* plugin);

// globally register a plugin (thread-safe), return new slot id
SIM_API int sim_plugin_registerPlugin(const SIM_pPlugin* plugin);

// globally register a resource provider (thread-safe), return new slot id
SIM_API int sim_plugin_registerResourceProvider(const SIM_pResourceProvider* provider);

// return the number of globally registered plugins
SIM_API int sim_plugin_pluginCount(void);

// return the number of globally registered resource providers
SIM_API int sim_plugin_resourceProviderCount(void);

// look up a plugin by name, optionally also get its registered slot number
SIM_API const SIM_pPlugin* sim_plugin_getPlugin(const char* name, int* slot);

// set default resource provider definition
SIM_API void sim_plugin_defaultResourceProvider(SIM_pResourceProvider* provider);

// look up a resource provider that matches its prefix against the given resource name
SIM_API const SIM_pResourceProvider* sim_plugin_getResourceProvider(const char* resource_name);

// look up a plugin by slot number
SIM_API const SIM_pPlugin* sim_plugin_getPluginAtSlot(int slot);

// look up a resource provider by slot number
SIM_API const SIM_pResourceProvider* sim_plugin_getResourceProviderAtSlot(int slot);

// return a config attribute of a plugin instance
// NULL: invalid plugin instance ID or attribute name
SIM_API const char* sim_getPluginConfig(const sim_model_t* m, int plugin_id, const char* attrib);

// load plugins from a dynamic library
SIM_API void sim_loadPluginLibrary(const char* path);

// scan a directory and load all dynamic libraries
SIM_API void sim_loadAllPluginLibraries(const char* directory, SIM_fPluginLibraryLoadCallback callback);

// registers a resource decoder
SIM_API void sim_plugin_registerDecoder(const SIM_pDecoder* decoder);

// set default decoder definition
SIM_API void sim_plugin_defaultDecoder(SIM_pDecoder* decoder);

// find a decoder that can process a given resource
SIM_API const SIM_pDecoder* sim_plugin_findDecoder(const SIM_Resource* resource, const char* content_type);

// =================================================================================================
// SimCore-internal functions beyond this point.
// "Unsafe" suffix indicates that improper use of these functions may result in data races.
//
// The unsafe functions assume that sim_plugin_pluginCount has already been called, and that all plugins
// up to `count` have been completely written into the global table.
// =================================================================================================

// internal version of sim_plugin_registerResourceProvider without prechecks on reserved prefixes
SIM_API int sim_plugin_registerResourceProviderInternal(const SIM_pResourceProvider* provider);

// look up a plugin by name, assuming that sim_plugin_pluginCount has already been called
const SIM_pPlugin* sim_plugin_getPluginUnsafe(const char* name, int* slot, int nslot);

// look up a plugin by slot number, assuming that sim_plugin_pluginCount has already been called
const SIM_pPlugin* sim_plugin_getPluginAtSlotUnsafe(int slot, int nslot);

#ifdef __cplusplus
}
#endif

#endif  // SIMCORE_SRC_ENGINE_ENGINE_PLUGIN_H_
