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

#ifndef SIMCORE_SRC_USER_CACHE_H_
#define SIMCORE_SRC_USER_CACHE_H_

#include <cstddef>
#include <cstdlib>
#include <functional>
#include <set>
#include <string>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include <simcore/SIM_plugin.h>

typedef std::function<bool(const void*)> SIM_CDataFunc;
typedef void (*SIM_CDeallocFunc)(const void*);

// A class container for a thread-safe asset cache
//
// Each SIM_CAsset is used to store raw and/or processed data loaded from a
// resource and is defined by a unique ID (usually the full filename of the
// asset).
class SIM_CAsset {
  friend class SIM_CCache;
 public:
  SIM_CAsset(std::string modelname, std::string id, const SIM_Resource* resource,
           std::shared_ptr<const void> data, std::size_t size) :
             id_(id), timestamp_(resource->timestamp),
             size_(size), data_(std::move(data)) {
    AddReference(modelname);
  }

  // move and copy constructors
  SIM_CAsset(SIM_CAsset&& other) = default;
  SIM_CAsset& operator=(SIM_CAsset&& other) = default;
  SIM_CAsset(const SIM_CAsset& other) = default;
  SIM_CAsset& operator=(const SIM_CAsset& other) = default;

  const std::string& Timestamp() const { return timestamp_; }
  const std::string& Id() const { return id_; }
  std::size_t InsertNum() const { return insert_num_; }
  std::size_t AccessCount() const { return access_count_; }

  // pass data in the cache to the given function, return true if data was copied
  bool PopulateData(SIM_CDataFunc fn) const {
    return fn(data_.get());
  }

 private:
  SIM_CAsset() = default;

  // helpers for managing models referencing this asset
  void AddReference(std::string xml_file) { references_.insert(xml_file); }
  void RemoveReference(const std::string& xml_file) {
    references_.erase(xml_file);
  }

  void ReplaceData(const SIM_CAsset& other)  {
    data_ = other.data_;
    size_ = other.size_;
  }

  bool HasReferences() const { return !references_.empty(); }

  void IncrementAccess() { access_count_++; }

  // makes a copy for user (strip unnecessary references)
  static SIM_CAsset Copy(const SIM_CAsset& other);

  // setters
  void SetInsertNum(std::size_t num) { insert_num_ = num; }
  void SetTimestamp(std::string timestamp) { timestamp_ = timestamp; }

  // accessors
  std::size_t BytesCount() const { return size_; }
  const void* Data() const {
    return data_.get();
  }
  const std::set<std::string>& References() const { return references_; }

  std::string id_;                    // unique id associated with asset
  std::string timestamp_;             // opaque timestamp of asset
  std::size_t insert_num_;            // number when asset was inserted
  std::size_t access_count_ = 0;      // incremented when getting 0th block
  std::size_t size_ = 0;              // how many bytes taken up by the asset
  std::shared_ptr<const void> data_;  // actual data of the asset

  // list of models referencing this asset
  std::set<std::string> references_;
};

struct SIM_CAssetCompare {
  bool operator()(const SIM_CAsset* e1, const SIM_CAsset* e2) const {
    if (e1->AccessCount() != e2->AccessCount()) {
      return e1->AccessCount() < e2->AccessCount();
    }
    return e1->InsertNum() < e2->InsertNum();
  }
};

// the class container for a thread-safe asset cache
class SIM_CCache {
 public:
  explicit SIM_CCache(std::size_t size)  :  capacity_(size) {}

  // move only
  SIM_CCache(SIM_CCache&& other) = delete;
  SIM_CCache& operator=(SIM_CCache&& other) = delete;
  SIM_CCache(const SIM_CCache& other) = delete;
  SIM_CCache& operator=(const SIM_CCache& other) = delete;

  // sets the capacity of the cache in bytes
  // low-priority cached assets will be dropped to make the new memory
  // requirement
  void SetCapacity(std::size_t size);

  // returns the corresponding timestamp, if the given asset is stored in
  // the cache
  const std::string* HasAsset(const std::string& id);

  // inserts an asset into the cache, if asset is already in the cache, its data
  // is updated only if the timestamps disagree
  bool Insert(const std::string& modelname, const std::string& id, const SIM_Resource *resource,
              std::shared_ptr<const void> data, std::size_t size);

  // populate data from the cache into the given function
  bool PopulateData(const std::string& id, const SIM_Resource* resource, SIM_CDataFunc fn);

  // deletes the asset from the cache with the given id
  void DeleteAsset(const std::string& id);

  // removes model from the cache, assets only referenced by the model will be
  // deleted
  void RemoveModel(const std::string& filename);

  // Wipes out all assets from the cache for the given model
  void Reset(const std::string& filename);

  // Wipes out all internal data
  void Reset();

  // accessors
  std::size_t Capacity() const;
  std::size_t Size() const;

 private:
  void Delete(SIM_CAsset* asset);
  void Delete(SIM_CAsset* asset, const std::string& skip);
  void Trim();

  // TODO(kylebayes): We should consider a shared mutex like in
  // engine/engine_plugin.cc as some of these methods don't need to be fully
  // locked.
  mutable std::mutex mutex_;
  std::size_t insert_num_ = 0;  // a running counter of assets being inserted
  std::size_t size_ = 0;        // current size of the cache in bytes
  std::size_t capacity_ = 0;    // capacity of the cache in bytes

  // internal constant look up table for assets
  std::unordered_map<std::string, SIM_CAsset> lookup_;

  // internal priority queue for the cache
  std::set<SIM_CAsset*, SIM_CAssetCompare> entries_;

  // models using the cache along with the assets they reference
  std::unordered_map<std::string, std::unordered_set<SIM_CAsset*>> models_;
};

#endif  // SIMCORE_SRC_USER_CACHE_H_
