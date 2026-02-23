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

#include "user/user_resource.h"

#include <sys/types.h>
#include <sys/stat.h>

#include <cstddef>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <string>
#include <string_view>
#include <simcore/core_api.h>

#include <simcore/SIM_plugin.h>
#include "engine/engine_plugin.h"
#include "user/user_util.h"
#include "user/user_vfs.h"

SIM_Resource* sim_math_openResource(const char* dir, const char* name,
                             const SIM_VFS* vfs, char* error, size_t nerror) {
  // TODO: Update API to use non-const pointer. Unfortunately, while this is
  // ABI stable, it will cause compiler errors in user code that is const
  // correct.
  SIM_VFS* non_const_vfs = const_cast<SIM_VFS*>(vfs);

  // TODO: Eventually, we should make `vfs` a required argument. In the
  // meantime, when passing in a nullptr VFS, we will create a VFS dynamically
  // that self-destructs when the resource is closed (or if the resource could
  // not be opened).
  if (non_const_vfs == nullptr) {
    SIM_VFS* local_vfs = (SIM_VFS*)sim_malloc(sizeof(SIM_VFS));
    sim_defaultVFS(local_vfs);
    simcore::user::VFS::Upcast(local_vfs)->SetToSelfDestruct([](SIM_VFS* ptr) {
      sim_deleteVFS(ptr);
      sim_free(ptr);
    });

    non_const_vfs = local_vfs;
  }

  SIM_Resource* resource =
      simcore::user::VFS::Upcast(non_const_vfs)->Open(dir ? dir : "", name);

  if (error) {
    if (resource) {
      error[0] = '\0';
    } else {
      std::snprintf(error, nerror, "Error opening file '%s'", name);
    }
  }

  return resource;
}

void sim_math_closeResource(SIM_Resource* resource) {
  if (resource && resource->vfs) {
    simcore::user::VFS::Upcast(resource->vfs)->Close(resource);
  }
}

int sim_math_readResource(SIM_Resource* resource, const void** buffer) {
  if (resource && resource->vfs) {
    return simcore::user::VFS::Upcast(resource->vfs)->Read(resource, buffer);
  }
  return -1;  // default (error reading bytes)
}

void sim_math_getResourceDir(SIM_Resource* resource, const char** dir, int* ndir) {
  *dir = nullptr;
  *ndir = 0;

  if (resource && resource->name) {
    // ensure prefix is included even if there is no separator in the
    // resource name
    int prefix_len = 0;
    const SIM_pResourceProvider* provider = resource->provider;
    if (provider && provider->prefix) {
      prefix_len = strlen(provider->prefix) + 1;
    }

    *dir = resource->name;
    *ndir = prefix_len;
    for (int i = prefix_len; resource->name[i]; ++i) {
      if (resource->name[i] == '/' || resource->name[i] == '\\') {
        *ndir = i + 1;
      }
    }
  }
}

int sim_math_isModifiedResource(const SIM_Resource* resource, const char* timestamp) {
  if (resource && resource->provider && resource->provider->modified) {
    return resource->provider->modified(resource, timestamp);
  }
  return 1;  // default (assume modified)
}

sim_spec_t* sim_math_decodeResource(SIM_Resource* resource, const char* content_type, const SIM_VFS* vfs) {
  const SIM_pDecoder* decoder = nullptr;
  if (content_type) {
    decoder = sim_plugin_findDecoder(resource, content_type);
  } else {
    decoder = sim_plugin_findDecoder(resource, sim_math_internal_extToContentType(resource->name).c_str());
  }
  if (!decoder) {
    sim_warning("Could not find decoder for resource '%s'", resource->name);
    return nullptr;
  }
  return decoder->decode(resource, vfs);
}
