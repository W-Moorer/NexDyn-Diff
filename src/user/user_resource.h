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
// IWYU pragma: private, include "third_party/simcore/include/simcore/core_api.h"
// IWYU pragma: friend "third_party/simcore/src/.*"

#ifndef SIMCORE_SRC_ENGINE_ENGINE_RESOURCE_H_
#define SIMCORE_SRC_ENGINE_ENGINE_RESOURCE_H_

#include <cstddef>

#include <simcore/SIM_export.h>
#include <simcore/core_api.h>

#ifdef __cplusplus
extern "C" {
#endif

// open the given resource; if the name doesn't have a prefix matching with a
// resource provider, then the OS filesystem is used
SIM_API SIM_Resource* sim_math_openResource(const char* dir, const char* name,
                                   const SIM_VFS* vfs, char* error, size_t nerror);

// close the given resource; no-op if resource is NULL
SIM_API void sim_math_closeResource(SIM_Resource* resource);

// set buffer to bytes read from the resource and return number of bytes in buffer;
// return negative value if error
SIM_API int sim_math_readResource(SIM_Resource* resource, const void** buffer);

// set for a resource with a name partitioned as {dir}{filename}, the dir and ndir pointers
SIM_API void sim_math_getResourceDir(SIM_Resource* resource, const char** dir, int* ndir);

// return 0 if the resource's timestamp matches the provided timestamp
// return > 0 if the resource is younger than the given timestamp
// return < 0 if the resource is older than the given timestamp
SIM_API int sim_math_isModifiedResource(const SIM_Resource* resource, const char* timestamp);

// given a resource, find its decoder and return the decoded spec
// the caller takes ownership of the spec and is responsible for cleaning it up
SIM_API sim_spec_t* sim_math_decodeResource(SIM_Resource* resource, const char* content_type,
                                 const SIM_VFS* vfs);

#ifdef __cplusplus
}
#endif

#endif  // SIMCORE_SRC_ENGINE_ENGINE_RESOURCE_H_
