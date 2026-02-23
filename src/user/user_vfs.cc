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

#include "user/user_vfs.h"

#include <sys/stat.h>
#ifdef _WIN32
#define stat _stat
#endif

#include <algorithm>
#include <cctype>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <ctime>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

#include <simcore/core_api.h>
#include "engine/engine_util_misc.h"
#include "user/user_util.h"

namespace {

// struct for holding the contents of a file
struct ResourceFileData {
  std::vector<uint8_t> contents;
  time_t modified_time = 0;
  bool is_read = false;
};

int OpenFile(const char* filename, SIM_Resource* resource) {
  struct stat file_stat;
  if (stat(filename, &file_stat) == 0) {
    ResourceFileData* data = new ResourceFileData();
    resource->data = data;

    data->modified_time = file_stat.st_mtime;
    sim_math_encodeBase64(resource->timestamp, (uint8_t*)&file_stat.st_mtime,
                     sizeof(time_t));
    return 1;
  }
  return 0;
}

int ReadFile(const char* filename, SIM_Resource* resource, const void** buffer) {
  ResourceFileData* data = (ResourceFileData*)resource->data;
  if (!data->is_read) {
    data->contents = simcore::user::FileToMemory(filename);
    data->is_read = true;
  }
  *buffer = data->contents.data();
  return static_cast<int>(data->contents.size());
}

void CloseFile(SIM_Resource* resource) {
  delete (ResourceFileData*)resource->data;
  resource->data = nullptr;
}

int FileModified(const SIM_Resource* resource, const char* timestamp) {
  if (sim_math_isValidBase64(timestamp) != sizeof(time_t)) {
    return 1;
  }
  time_t time;
  sim_math_decodeBase64((uint8_t*)&time, timestamp);

  ResourceFileData* data = (ResourceFileData*)resource->data;
  const double diff = difftime(data->modified_time, time);

  if (diff < 0) return -1;
  if (diff > 0) return 1;
  return 0;
}

std::string StripPathAndLower(std::string path) {
  std::size_t n = path.find_last_of("/\\");
  if (n != std::string::npos) {
    path = path.substr(n + 1);
  }
  std::transform(path.begin(), path.end(), path.begin(),
                 [](unsigned char c) { return std::tolower(c); });
  return path;
}

}  // namespace

namespace simcore::user {

VFS::VFS(SIM_VFS* vfs) : self_(vfs) {
  sim_plugin_defaultResourceProvider(&default_provider_);
  default_provider_.open = [](SIM_Resource* res) {
    return OpenFile(res->name, res);
  };
  default_provider_.read = [](SIM_Resource* res, const void** buffer) {
    return ReadFile(res->name, res, buffer);
  };
  default_provider_.close = [](SIM_Resource* res) {
    CloseFile(res);
  };
  default_provider_.modified = [](const SIM_Resource* res, const char* time) {
    return FileModified(res, time);
  };

  default_provider_.prefix = nullptr;
  default_mount_.vfs = self_;
  default_mount_.provider = &default_provider_;
  default_mount_.data = nullptr;
  default_mount_.name = nullptr;
}

VFS::~VFS() {
  if (!open_resources_.empty()) {
    sim_warning(
        "VFS destroyed with %zu open resources. Resources will be invalidated.",
        open_resources_.size());
  }
  for (auto& [ptr, res] : open_resources_) {
    if (res->provider->close) {
      res->provider->close(res.get());
    }
  }
  open_resources_.clear();

  for (auto& [path, res] : mounts_) {
    if (res->provider->unmount) {
      res->provider->unmount(res.get());
    }
  }
  mounts_.clear();
}

SIM_Resource* VFS::Open(const char* dir, const char* name) {
  const std::string path = FilePath(dir, name).Str();

  const SIM_Resource* mount = FindMount(path);
  if (!mount) {
    MaybeSelfDestruct();
    return nullptr;
  }

  ResourcePtr res = CreateResource(path.c_str(), mount->provider);

  // Smuggle the mounted resource provider's mount-specific data pointer in the
  // requested resource's data pointer. This allows the provider to access its
  // own per-mount data without any intrusive changes to the provider interface.
  res->data = mount->data;
  const int result = mount->provider->open(res.get());
  // If the data pointer was not modified, then that means the resource did not
  // set its own data pointer. So, we need to set it back to nullptr.
  if (res->data == mount->data) {
    res->data = nullptr;
  }

  if (result == 0) {
    res.reset();
    MaybeSelfDestruct();
    return nullptr;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  SIM_Resource* res_ptr = res.get();
  open_resources_.emplace(res_ptr, std::move(res));
  return res_ptr;
}

VFS::Status VFS::Mount(const FilePath& path,
                       const SIM_pResourceProvider* provider) {
  if (!provider) {
    return kInvalidResourceProvider;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (mounts_.contains(path.Str())) {
      return kRepeatedName;
    }
  }

  ResourcePtr res = CreateResource(path.c_str(), provider);
  provider->mount(res.get());

  std::lock_guard<std::mutex> lock(mutex_);
  mounts_.emplace(path.Str(), std::move(res));
  return kSuccess;
}

VFS::Status VFS::Close(SIM_Resource* res) {
  VFS::Status status = kInvalidResource;
  bool last_resource = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (auto it = open_resources_.find(res); it != open_resources_.end()) {
      if (res->provider->close) {
        res->provider->close(res);
      }
      open_resources_.erase(it);
      last_resource = open_resources_.empty();
      status = kSuccess;
    }
  }

  if (status == kSuccess && last_resource) {
    MaybeSelfDestruct();
  }
  return status;
}

VFS::Status VFS::Unmount(const FilePath& path) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (auto it = mounts_.find(path.Str()); it != mounts_.end()) {
    if (it->second->provider->unmount) {
      it->second->provider->unmount(it->second.get());
    }
    mounts_.erase(it);
    return kSuccess;
  }
  return kInvalidResourceProvider;
}

int VFS::Read(SIM_Resource* resource, const void** buffer) {
  if (resource && resource->provider && resource->provider->read) {
    return resource->provider->read(resource, buffer);
  }
  return kFailedToRead;
}

VFS::ResourcePtr VFS::CreateResource(std::string_view name,
                                     const SIM_pResourceProvider* provider) {
  SIM_Resource* res = new SIM_Resource();
  res->vfs = self_;
  res->provider = provider;
  res->data = nullptr;
  res->name = new char[name.size() + 1];
  std::strncpy(res->name, name.data(), name.size());
  res->name[name.size()] = 0;
  res->timestamp[0] = 0;

  return ResourcePtr(res, [](SIM_Resource* ptr) {
    if (ptr->data) {
      // TODO: Make this an error eventually. For now, we continue to allow
      // users to free their data pointers without resetting them to nullptr.
      sim_warning(
          "SIM_Resource::data is not null; did you forget to close/unmount it?");
    }
    delete[] ptr->name;
    delete ptr;
  });
}

SIM_Resource* VFS::FindMount(const std::string& fullpath) {
  std::lock_guard<std::mutex> lock(mutex_);

  std::string str = fullpath;
  while (!str.empty()) {
    auto it = mounts_.find(str);
    if (it != mounts_.end()) {
      return it->second.get();
    }

    std::size_t n = str.find_last_of("/\\");
    if (n == std::string::npos) {
      str = "";
    } else {
      str = str.substr(0, n);
    }
  }

  const SIM_pResourceProvider* provider =
      sim_plugin_getResourceProvider(fullpath.c_str());
  if (provider) {
    if (auto it = mounts_.find(provider->prefix); it != mounts_.end()) {
      return it->second.get();
    }

    ResourcePtr res = CreateResource(provider->prefix, provider);
    SIM_Resource* res_ptr = res.get();
    mounts_.emplace(provider->prefix, std::move(res));
    if (provider->mount) {
      provider->mount(res_ptr);
    }
    return res_ptr;
  }

  // Legacy use-case: match on just the case-insensitive filename.
  const std::string filename = StripPathAndLower(fullpath);
  for (auto& [path, res] : mounts_) {
    if (StripPathAndLower(path) == filename) {
      return res.get();
    }
  }

  return &default_mount_;
}

void VFS::MaybeSelfDestruct() {
  if (destructor_) {
    destructor_(self_);
  }
}

void VFS::SetToSelfDestruct(std::function<void(SIM_VFS*)> destructor) {
  destructor_ = std::move(destructor);
}

VFS* VFS::Upcast(SIM_VFS* vfs) {
  return vfs ? static_cast<VFS*>(vfs->impl_) : nullptr;
}

const VFS* VFS::Upcast(const SIM_VFS* vfs) {
  return vfs ? static_cast<const VFS*>(vfs->impl_) : nullptr;
}

}  // namespace simcore::user

void sim_defaultVFS(SIM_VFS* vfs) {
  if (vfs == nullptr) {
    sim_error("SIM_VFS is null.");
  } else {
    vfs->impl_ = new simcore::user::VFS(vfs);
  }
}

void sim_deleteVFS(SIM_VFS* vfs) {
  if (vfs) {
    delete simcore::user::VFS::Upcast(vfs);
    vfs->impl_ = nullptr;
  }
}

int sim_mountVFS(SIM_VFS* vfs, const char* filepath,
                const SIM_pResourceProvider* provider) {
  simcore::user::VFS* impl = simcore::user::VFS::Upcast(vfs);
  if (impl == nullptr) {
    sim_error("SIM_VFS is null.");
    return simcore::user::VFS::kInvalidVfs;
  }

  if (filepath == nullptr) {
    return simcore::user::VFS::kNotFound;
  }
  const simcore::user::FilePath path(filepath);
  const simcore::user::VFS::Status status = impl->Mount(path, provider);
  return static_cast<int>(status);
}

int sim_unmountVFS(SIM_VFS* vfs, const char* filename) {
  simcore::user::VFS* impl = simcore::user::VFS::Upcast(vfs);
  if (impl == nullptr) {
    sim_error("SIM_VFS is null.");
    return simcore::user::VFS::kInvalidVfs;
  }

  if (filename == nullptr) {
    return simcore::user::VFS::kNotFound;
  }
  const simcore::user::FilePath path(filename);
  const simcore::user::VFS::Status status = impl->Unmount(path);
  return static_cast<int>(status);
}

namespace {

// Custom provider for sim_addFileVFS and sim_addBufferVFS.
class BufferProvider : public SIM_pResourceProvider {
 public:
  template <typename... Args>
  static int Mount(SIM_VFS* vfs, Args&&... args) {
    simcore::user::VFS* impl = simcore::user::VFS::Upcast(vfs);
    if (impl == nullptr) {
      sim_error("SIM_VFS is null.");
      return -1;
    }

    BufferProvider* provider = new BufferProvider(std::forward<Args>(args)...);
    provider->mount = [](SIM_Resource* res) {
      return static_cast<int>(simcore::user::VFS::kSuccess);
    };
    provider->unmount = [](SIM_Resource* res) {
      delete (BufferProvider*)res->provider;
      return static_cast<int>(simcore::user::VFS::kSuccess);
    };
    provider->open = [](SIM_Resource* res) {
      BufferProvider* self = (BufferProvider*)res->provider;
      sim_math_encodeBase64(res->timestamp, (std::uint8_t*)&self->hash_,
                       sizeof(self->hash_));
      return 1;
    };
    provider->read = [](SIM_Resource* res, const void** out) {
      BufferProvider* self = (BufferProvider*)res->provider;
      *out = reinterpret_cast<void*>(self->contents_.data());
      return static_cast<int>(self->contents_.size());
    };
    provider->close = [](SIM_Resource* res) {
      // no-op
    };
    provider->modified = [](const SIM_Resource* res, const char* timestamp) {
      const BufferProvider* self = (const BufferProvider*)res->provider;
      if (sim_math_isValidBase64(timestamp) > sizeof(std::uint64_t)) {
        return 1;
      }
      std::uint64_t test = 0;
      sim_math_decodeBase64((std::uint8_t*)&test, timestamp);
      if (self->hash_ != test) {
        return 1;
      }
      return 0;
    };

    const simcore::user::VFS::Status status =
        impl->Mount(provider->path_, provider);
    if (status != simcore::user::VFS::kSuccess) {
      delete provider;
    }
    return static_cast<int>(status);
  }

 private:
  BufferProvider(const char* dir, const char* filename) {
    sim_plugin_defaultResourceProvider(this);

    simcore::user::FilePath file_path(dir ? dir : "", filename);
    path_ = file_path.StripPath().Lower();
    contents_ = simcore::user::FileToMemory(file_path.c_str());

    static constexpr std::uint64_t prime = 0x100000001b3;
    hash_ = contents_.empty() ? 0 : 0xcbf29ce484222325;
    for (const std::uint8_t& byte : contents_) {
      hash_ |= byte;
      hash_ *= prime;
    }
  }

  BufferProvider(const char* name, const void* src, size_t n) {
    sim_plugin_defaultResourceProvider(this);
    path_ = simcore::user::FilePath(name);

    static constexpr std::uint64_t prime = 0x100000001b3;
    hash_ = n ? 0xcbf29ce484222325 : 0;

    const std::uint8_t* src_bytes = static_cast<const std::uint8_t*>(src);
    contents_.reserve(n);
    for (size_t i = 0; i < n; i++) {
      contents_.push_back(src_bytes[i]);
      hash_ |= src_bytes[i];
      hash_ *= prime;
    }
  }

  simcore::user::FilePath path_;
  std::vector<std::uint8_t> contents_;
  std::uint64_t hash_ = 0;
};

}  // namespace

int sim_addFileVFS(SIM_VFS* vfs, const char* directory, const char* filename) {
  // Opens the files and copies its contents into the BufferProvider, then
  // mounts the provider at the given path.
  return BufferProvider::Mount(vfs, directory, filename);
}

int sim_addBufferVFS(SIM_VFS* vfs, const char* name, const void* buffer,
                    int nbuffer) {
  // Copies the buffer into the BufferProvider and mounts it at the given path.
  return BufferProvider::Mount(vfs, name, buffer, nbuffer);
}

int sim_deleteFileVFS(SIM_VFS* vfs, const char* filename) {
  if (filename == nullptr) {
    return simcore::user::VFS::kNotFound;
  }

  if (sim_unmountVFS(vfs, filename) != 0) {
    simcore::user::FilePath path(filename);
    return sim_unmountVFS(vfs, path.StripPath().Lower().c_str());
  }
  return simcore::user::VFS::kSuccess;
}
