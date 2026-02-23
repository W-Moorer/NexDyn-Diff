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

#include <algorithm>
#include <array>
#include <climits>
#include <cmath>
#include <csetjmp>
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <deque>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

#include <simcore/SIM_spec.h>
#include "user/user_api.h"
#include <TriangleMeshDistance/include/tmd/TriangleMeshDistance.h>

#ifdef SIMCORE_TINYOBJLOADER_IMPL
#define TINYOBJLOADER_IMPLEMENTATION
#endif

#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wgnu-anonymous-struct"
#pragma clang diagnostic ignored "-Wnested-anon-types"
#elif defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#endif
#include <MC.h>
#if defined(__clang__)
#pragma clang diagnostic pop
#elif defined(__GNUC__)
#pragma GCC diagnostic pop
#endif

#include <simcore/SIM_macro.h>
#include <simcore/SIM_model.h>
#include <simcore/SIM_plugin.h>
#include <simcore/SIM_tnum.h>
#include "engine/engine_crossplatform.h"  // IWYU pragma: keep
#include "engine/engine_plugin.h"
#include "engine/engine_util_errmem.h"
#include "user/user_cache.h"
#include "user/user_model.h"
#include "user/user_objects.h"
#include "user/user_resource.h"
#include "user/user_util.h"

extern "C" {
#include "qhull_ra.h"
}

namespace {
  using simcore::user::FilePath;
  using std::max;
  using std::min;
  using std::sin;
  using std::cos;
  using std::pow;

  // Parametrized linear/quintic interpolated nonlinearity.
  double Fovea(double x, double gamma) {
    // Quick return.
    if (!gamma) return x;

    // Foveal deformation.
    double g = SIM_MAX(0, SIM_MIN(1, gamma));
    return g * pow(x, 5) + (1 - g) * x;
  }

  // Evenly spaced numbers over a specified interval.
  void LinSpace(double lower, double upper, int n, double array[]) {
    double increment = n > 1 ? (upper - lower) / (n - 1) : 0;
    for (int i = 0; i < n; ++i) {
      *array = lower;
      ++array;
      lower += increment;
    }
  }

  // Make bin edges.
  void BinEdges(double* x_edges, double* y_edges, int size[2], double fov[2],
                double gamma) {
    // Make unit bin edges.
    LinSpace(-1, 1, size[0] + 1, x_edges);
    LinSpace(-1, 1, size[1] + 1, y_edges);

    // Apply foveal deformation.
    for (int i = 0; i < size[0] + 1; i++) {
      x_edges[i] = Fovea(x_edges[i], gamma);
    }
    for (int i = 0; i < size[1] + 1; i++) {
      y_edges[i] = Fovea(y_edges[i], gamma);
    }

    // Scale by field-of-view.
    sim_math_internal_scalevec(x_edges, x_edges, fov[0] * SIM_PI / 180, size[0] + 1);
    sim_math_internal_scalevec(y_edges, y_edges, fov[1] * SIM_PI / 180, size[1] + 1);
  }

  // Transform spherical (azimuth, elevation, radius) to Cartesian (x,y,z).
  void SphericalToCartesian(const double aer[3], float xyz[3]) {
    double a = aer[0], e = aer[1], r = aer[2];
    xyz[0] = r * cos(e) * sin(a);
    xyz[1] = r * sin(e);
    xyz[2] = -r * cos(e) * cos(a);
  }

  // Tangent frame in Cartesian coordinates.
  void TangentFrame(const double aer[3], float mat[9]) {
    double a = aer[0], e = aer[1], r = aer[2];
    double ta[3] = {r * cos(e) * cos(a), 0, r * cos(e) * sin(a)};
    double te[3] = {-r * sin(e) * sin(a), r * cos(e), r * sin(e) * cos(a)};
    double n[3];
    sim_math_internal_normvec(ta, 3);
    sim_math_internal_normvec(te, 3);
    sim_math_internal_copy_vec(mat + 3, ta, 3);
    sim_math_internal_copy_vec(mat + 6, te, 3);
    sim_math_internal_crossvec(n, te, ta);
    sim_math_internal_copy_vec(mat, n, 3);
  }

  // parametric superellipsoid/supertoroid helper functions
  double aux_c(double omega, double m) {
    return std::copysign(pow(std::abs(cos(omega)), m), cos(omega));
  }
  double aux_s(double omega, double m) {
    return std::copysign(pow(std::abs(sin(omega)), m), sin(omega));
  }
}  // namespace

// compute triangle area, surface normal, center
static double triangle(double* normal, double* center,
                       const double* v1, const double* v2, const double* v3) {
  double normal_local[3];  // if normal is nullptr
  double* normal_ptr = (normal) ? normal : normal_local;
  // center
  if (center) {
    center[0] = (v1[0] + v2[0] + v3[0])/3;
    center[1] = (v1[1] + v2[1] + v3[1])/3;
    center[2] = (v1[2] + v2[2] + v3[2])/3;
  }

  // normal = (v2-v1) cross (v3-v1)
  double b[3] = { v2[0] - v1[0], v2[1] - v1[1], v2[2] - v1[2] };
  double c[3] = { v3[0] - v1[0], v3[1] - v1[1], v3[2] - v1[2] };
  sim_math_internal_crossvec(normal_ptr, b, c);

  // get length
  double len = sqrt(sim_math_internal_dot3(normal_ptr, normal_ptr));

  // ignore small faces
  if (len < SIM_MINVAL) {
    return 0;
  }

  // normalize
  if (normal) {
    normal_ptr[0] /= len;
    normal_ptr[1] /= len;
    normal_ptr[2] /= len;
  }

  // return area
  return 0.5 * len;
}


// Read data of type T from a potentially unaligned buffer pointer.
template <typename T>
static void ReadFromBuffer(T* dst, const char* src) {
  std::memcpy(dst, src, sizeof(T));
}



//------------------ class sim_builder_mesh_t implementation --------------------------------------------------

sim_builder_mesh_t::sim_builder_mesh_t(sim_builder_model_t* _model, sim_builder_default_t* _def) {
  sim_spec_defaultMesh(&spec);
  elemtype = SIM_OBJ_MESH;

  // clear internal variables
  sim_math_internal_set_vec(pos_, 0, 0, 0);
  sim_math_internal_set_vec(quat_, 1, 0, 0, 0);

  sim_math_internal_set_vec(boxsz_, 0, 0, 0);
  sim_math_internal_set_vec(aamm_, 1e10, 1e10, 1e10);
  sim_math_internal_set_vec(aamm_+3, -1e10, -1e10, -1e10);
  szgraph_ = 0;
  center_ = nullptr;
  graph_ = nullptr;
  needhull_ = false;
  maxhullvert_ = -1;
  processed_ = false;
  visual_ = true;
  needreorient_ = true;

  // reset to default if given
  if (_def) {
    *this = _def->Mesh();
  }

  // set model, def
  model = _model;
  if (_model) compiler = &_model->spec.compiler;
  classname = (_def ? _def->name : (_model ? "main" : ""));

  // in case this body is not compiled
  CopyFromSpec();

  // point to local
  PointToLocal();
}



sim_builder_mesh_t::sim_builder_mesh_t(const sim_builder_mesh_t& other) {
  *this = other;
}



sim_builder_mesh_t& sim_builder_mesh_t::operator=(const sim_builder_mesh_t& other) {
  if (this != &other) {
    this->spec = other.spec;
    *static_cast<SIM_CMesh_*>(this) = static_cast<const SIM_CMesh_&>(other);
    *static_cast<SIM_sMesh*>(this) = static_cast<const SIM_sMesh&>(other);
    if (other.center_) {
      size_t ncenter = 3*other.nface()*sizeof(double);
      this->center_ = (double*)sim_malloc(ncenter);
      memcpy(this->center_, other.center_, ncenter);
    } else {
      this->center_ = nullptr;
    }
    if (other.graph_) {
      size_t szgraph = szgraph_*sizeof(int);
      this->graph_ = (int*)sim_malloc(szgraph);
      memcpy(this->graph_, other.graph_, szgraph);
    } else {
      this->graph_ = nullptr;
    }
  }
  PointToLocal();
  return *this;
}



void sim_builder_mesh_t::PointToLocal() {
  spec.element = static_cast<sim_spec_element_t*>(this);
  spec.file = &spec_file_;
  spec.content_type = &spec_content_type_;
  spec.uservert = &spec_vert_;
  spec.usernormal = &spec_normal_;
  spec.userface = &spec_face_;
  spec.userfacenormal = &spec_facenormal_;
  spec.usertexcoord = &spec_texcoord_;
  spec.userfacetexcoord = &spec_facetexcoord_;
  spec.material = &spec_material_;
  spec.plugin.plugin_name = &plugin_name;
  spec.plugin.name = &plugin_instance_name;
  spec.info = &info;
  file = nullptr;
  content_type = nullptr;
  uservert = nullptr;
  usernormal = nullptr;
  userface = nullptr;
  userfacenormal = nullptr;
  usertexcoord = nullptr;
  userfacetexcoord = nullptr;
}



void sim_builder_mesh_t::NameSpace(const sim_builder_model_t* m) {
  if (name.empty()) {
    std::string stripped = sim_math_internal_strippath(spec_file_);
    name = sim_math_internal_stripext(stripped);
  }
  sim_builder_base_t::NameSpace(m);
  if (modelfiledir_.empty()) {
    modelfiledir_ = FilePath(m->spec_modelfiledir_);
  }
  if (!plugin_instance_name.empty()) {
    plugin_instance_name = m->prefix + plugin_instance_name + m->suffix;
  }
}



void sim_builder_mesh_t::CopyFromSpec() {
  *static_cast<SIM_sMesh*>(this) = spec;
  file_ = spec_file_;
  content_type_ = spec_content_type_;
  normal_ = spec_normal_;
  face_ = spec_face_;
  material_ = spec_material_;
  ProcessVertices(spec_vert_);
  texcoord_ = spec_texcoord_;
  facetexcoord_ = spec_facetexcoord_;
  maxhullvert_ = spec.maxhullvert;
  plugin.active = spec.plugin.active;
  plugin.element = spec.plugin.element;
  plugin.plugin_name = spec.plugin.plugin_name;
  plugin.name = spec.plugin.name;

  // clear precompiled asset. TODO: use asset cache
  if (center_) sim_free(center_);
  if (graph_) sim_free(graph_);
  szgraph_ = 0;
  center_ = nullptr;
  graph_ = nullptr;

  // use filename if name is missing
  if (name.empty()) {
    std::string stripped = sim_math_internal_strippath(file_);
    name = sim_math_internal_stripext(stripped);
  }
}



void sim_builder_mesh_t::CopyPlugin() {
  model->CopyExplicitPlugin(this);
}



sim_builder_mesh_t::~sim_builder_mesh_t() {
  if (center_) sim_free(center_);
  if (graph_) sim_free(graph_);
}



// generate mesh using marching cubes
void sim_builder_mesh_t::LoadSDF() {
  if (plugin_name.empty() && plugin_instance_name.empty()) {
    throw sim_builder_error_t(
            this, "neither 'plugin' nor 'instance' is specified for mesh '%s', (id = %d)",
            name.c_str(), id);
  }

  if (scale[0] != 1 || scale[1] != 1 || scale[2] != 1) {
    throw sim_builder_error_t(this, "attribute scale is not compatible with SDFs in mesh '%s', (id = %d)",
                   name.c_str(), id);
  }

  sim_builder_plugin_t* plugin_instance = static_cast<sim_builder_plugin_t*>(plugin.element);
  model->ResolvePlugin(this, plugin_name, plugin_instance_name, &plugin_instance);
  plugin.element = plugin_instance;
  const SIM_pPlugin* pplugin = sim_plugin_getPluginAtSlot(plugin_instance->plugin_slot);
  if (!(pplugin->capabilityflags & SIM_PLUGIN_SDF)) {
    throw sim_builder_error_t(this, "plugin '%s' does not support signed distance fields", pplugin->name);
  }

  std::vector<sim_scalar_t> attributes(pplugin->nattribute, 0);
  std::vector<const char*> names(pplugin->nattribute, 0);
  std::vector<const char*> values(pplugin->nattribute, 0);
  for (int i=0; i < pplugin->nattribute; i++) {
    names[i] = pplugin->attributes[i];
    values[i] = plugin_instance->config_attribs[names[i]].c_str();
  }

  if (pplugin->sdf_attribute) {
    pplugin->sdf_attribute(attributes.data(), names.data(), values.data());
  }

  sim_scalar_t aabb[6] = {0};
  pplugin->sdf_aabb(aabb, attributes.data());
  sim_scalar_t total = aabb[3] + aabb[4] + aabb[5];

  const double n = 1024;
  int nx, ny, nz;
  nx = floor(n / total * aabb[3]) + 1;
  ny = floor(n / total * aabb[4]) + 1;
  nz = floor(n / total * aabb[5]) + 1;
  MC::MC_FLOAT* field = new MC::MC_FLOAT[nx * ny * nz];

  for (int i = 0; i < nx; i++) {
    for (int j = 0; j < ny; j++) {
      for (int k = 0; k < nz; k++) {
        sim_scalar_t point[] = {aabb[0]-aabb[3] + 2 * aabb[3] * i / (nx-1),
                          aabb[1]-aabb[4] + 2 * aabb[4] * j / (ny-1),
                          aabb[2]-aabb[5] + 2 * aabb[5] * k / (nz-1)};
        field[(k * ny + j) * nx + i] =  pplugin->sdf_staticdistance(point, attributes.data());
      }
    }
  }

  MC::mcMesh mesh;
  MC::marching_cube(field, nx, ny, nz, mesh);
  std::vector<float> uservert;
  std::vector<float> usernormal;
  std::vector<int> userface;

  uservert.reserve(mesh.vertices.size() * 3);
  usernormal.reserve(mesh.normals.size() * 3);
  userface.reserve(mesh.indices.size());

  for (const auto& vertex : mesh.vertices) {
    uservert.push_back(2*aabb[3]*vertex.x/(nx-1) + aabb[0]-aabb[3]);
    uservert.push_back(2*aabb[4]*vertex.y/(ny-1) + aabb[1]-aabb[4]);
    uservert.push_back(2*aabb[5]*vertex.z/(nz-1) + aabb[2]-aabb[5]);
  }

  for (const auto& normal : mesh.normals) {
    usernormal.push_back(normal.x);
    usernormal.push_back(normal.y);
    usernormal.push_back(normal.z);
  }

  for (unsigned int index : mesh.indices) {
    userface.push_back(index);
  }

  needreorient_ = false;
  needsdf = false;
  normal_ = std::move(usernormal);
  face_ = std::move(userface);
  ProcessVertices(uservert);
  delete[] field;
}



void sim_builder_mesh_t::CacheMesh(SIM_CCache* cache, const SIM_Resource* resource) {
  if (cache == nullptr) return;

  // cache mesh data into new mesh object
  sim_builder_mesh_t *mesh =  new sim_builder_mesh_t();

  // mesh properties
  mesh->maxhullvert_ = maxhullvert_;
  mesh->inertia = inertia;
  std::copy(scale, scale + 3, mesh->scale);

  // mesh processed data
  mesh->processed_ = processed_;
  mesh->vert_ = vert_;
  mesh->normal_ = normal_;
  mesh->texcoord_ = texcoord_;
  mesh->face_ = face_;
  mesh->facenormal_ = facenormal_;
  mesh->facetexcoord_ = facetexcoord_;
  mesh->halfedge_ = halfedge_;
  mesh->szgraph_ = szgraph_;
  if (szgraph_) {
    mesh->graph_ = (int*)sim_malloc(szgraph_*sizeof(int));
    std::copy(graph_, graph_ + szgraph_, mesh->graph_);
  }
  mesh->polygons_ = polygons_;
  mesh->polygon_normals_ = polygon_normals_;
  mesh->polygon_map_ = polygon_map_;
  mesh->surface_ = surface_;
  mesh->volume_ = volume_;
  mesh->material_ = material_;
  std::copy(boxsz_, boxsz_ + 3, mesh->boxsz_);
  std::copy(aamm_, aamm_ + 6, mesh->aamm_);
  std::copy(pos_, pos_ + 3, mesh->pos_);
  std::copy(quat_, quat_ + 4, mesh->quat_);
  int ncenter = face_.size();
  if (ncenter) {
    mesh->center_ = (double*)sim_malloc(ncenter * sizeof(double));
    std::copy(center_, center_ + ncenter, mesh->center_);
  }
  mesh->tree_ = tree_;
  mesh->face_aabb_ = face_aabb_;
  mesh->octree_ = octree_;

  // calculate estimated size of mesh
  std::size_t size = sizeof(sim_builder_mesh_t)
                     + (sizeof(double) * vert_.size())
                     + (sizeof(float) * normal_.size())
                     + (sizeof(float) * texcoord_.size())
                     + (sizeof(int) * face_.size())
                     + (sizeof(int) * facenormal_.size())
                     + (sizeof(int) * facetexcoord_.size())
                     + (sizeof(int) * 2 * halfedge_.size())
                     + (sizeof(int) * szgraph_)
                     + (sizeof(int) * npolygonvert())
                     + (sizeof(double) * polygon_normals_.size())
                     + (sizeof(int) * npolygonmap())
                     + (sizeof(double) * 18)
                     + (sizeof(int) * ncenter)
                     + tree_.Size()
                     + octree_.Size()
                     + (sizeof(double) * face_aabb_.size());

  std::shared_ptr<const void> cached_data(mesh, +[] (const void* data) {
    const sim_builder_mesh_t* mesh = static_cast<const sim_builder_mesh_t*>(data);
    delete mesh;
  });
  cache->Insert("", resource->name, resource, cached_data, size);
}

namespace {

// vertex key for hash map
struct VertexKey {
  float v[3];

  bool operator==(const VertexKey& other) const {
    return (v[0] == other.v[0] && v[1] == other.v[1] && v[2] == other.v[2]);
  }

  std::size_t operator()(const VertexKey& vertex) const {
    // combine all three hash values into a single hash value
    return ((std::hash<float>()(vertex.v[0])
            ^ (std::hash<float>()(vertex.v[1]) << 1)) >> 1)
            ^ (std::hash<float>()(vertex.v[2]) << 1);
  }
};

}  // namespace



// convert vertices to double precision and remove repeated vertices if requested
void sim_builder_mesh_t::ProcessVertices(const std::vector<float>& vert, bool remove_repeated) {
  vert_.clear();
  int nvert = vert.size();

  if (nvert % 3) {
    throw sim_builder_error_t(this, "vertex data must be a multiple of 3");
  }
  if (face_.size() % 3) {
    throw sim_builder_error_t(this, "face data must be a multiple of 3");
  }

  // convert vertices to double precision, may contain repeated vertices
  if (!remove_repeated) {
    vert_.reserve(nvert);
    for (int i = 0; i < nvert / 3; ++i) {
      const float* v = &vert[3 * i];
      if (!std::isfinite(v[0]) || !std::isfinite(v[1]) || !std::isfinite(v[2])) {
        throw sim_builder_error_t(this, "vertex coordinate %d is not finite", nullptr, i);
      }
      vert_.push_back(v[0]);
      vert_.push_back(v[1]);
      vert_.push_back(v[2]);
    }
    return;
  }

  int index = 0;
  std::unordered_map<VertexKey, int, VertexKey> vertex_map;

  // populate vertex map with new vertex indices
  for (int i = 0; i < nvert; i += 3) {
    const float* v = &vert[i];

    if (!std::isfinite(v[0]) || !std::isfinite(v[1]) || !std::isfinite(v[2])) {
      throw sim_builder_error_t(this, "vertex coordinate %d is not finite", nullptr, i);
    }

    VertexKey key = {v[0], v[1], v[2]};
    if (vertex_map.find(key) == vertex_map.end()) {
      vertex_map.insert({key, index});
      ++index;
    }
  }

  // no repeated vertices (just copy vertex data)
  if (3*index == nvert) {
    vert_.reserve(nvert);
    for (float v : vert) {
      vert_.push_back(v);
    }
    return;
  }

  // update face vertex indices
  for (int i = 0; i < face_.size(); ++i) {
    VertexKey key = {vert[3*face_[i]], vert[3*face_[i] + 1],
                     vert[3*face_[i] + 2]};
    face_[i] = vertex_map[key];
  }

  // repopulate vertex data
  vert_.resize(3 * index);
  for (const auto& pair : vertex_map) {
    const VertexKey& key = pair.first;
    int index = pair.second;

    // double precision
    vert_[3*index + 0] = key.v[0];
    vert_[3*index + 1] = key.v[1];
    vert_[3*index + 2] = key.v[2];
  }
}

bool sim_builder_mesh_t::IsObj(std::string_view filename, std::string_view ct) {
  std::string asset_type = GetAssetContentType(filename, ct);
  return asset_type == "model/obj";
}

bool sim_builder_mesh_t::IsSTL(std::string_view filename, std::string_view ct) {
  std::string asset_type = GetAssetContentType(filename, ct);
  return asset_type == "model/stl";
}

bool sim_builder_mesh_t::IsMSH(std::string_view filename, std::string_view ct) {
  std::string asset_type = GetAssetContentType(filename, ct);
  return asset_type == "model/vnd.simcore.msh";
}

bool sim_builder_mesh_t::IsObj() const {
  return content_type_ == "model/obj";
}

bool sim_builder_mesh_t::IsSTL() const {
  return content_type_ == "model/stl";
}

bool sim_builder_mesh_t::IsMSH() const {
  return content_type_ == "model/vnd.simcore.msh";
}



// load mesh from resource; throw error on failure
void sim_builder_mesh_t::LoadFromResource(SIM_Resource* resource, bool remove_repeated) {
  // set content type from resource name
  std::string asset_type = GetAssetContentType(resource->name, content_type_);
  if (asset_type.empty()) {
    if (!content_type_.empty()) {
      throw sim_builder_error_t(this, "invalid content type: '%s'", content_type_.c_str());
    }
    throw sim_builder_error_t(this, "unknown or unsupported mesh file: '%s'", resource->name);
  }
  content_type_ = asset_type;

  if (IsSTL()) {
    LoadSTL(resource);
  } else if (IsObj()) {
    LoadOBJ(resource, remove_repeated);
  } else if (IsMSH()) {
    LoadMSH(resource, remove_repeated);
  } else {
    throw sim_builder_error_t(this, "unsupported mesh type: '%s'", asset_type.c_str());
  }
}



// compiler wrapper
void sim_builder_mesh_t::Compile(const SIM_VFS* vfs) {
  try {
    TryCompile(vfs);
  } catch (sim_builder_error_t err) {
    if (resource_ != nullptr) {
      sim_math_closeResource(resource_);
      resource_ = nullptr;
    }
    throw err;
  }
}



// compiler
void sim_builder_mesh_t::TryCompile(const SIM_VFS* vfs) {
  bool fromCache = false;
  CopyFromSpec();
  visual_ = true;
  SIM_CCache *cache = reinterpret_cast<SIM_CCache*>(sim_getCache()->impl_);

  // load file
  if (!file_.empty()) {
    vert_.clear();
    face_.clear();
    normal_.clear();
    texcoord_.clear();
    facenormal_.clear();
    facetexcoord_.clear();
    if (resource_ != nullptr) {
      sim_math_closeResource(resource_);
      resource_ = nullptr;
    }

    // copy paths from model if not already defined
    simcore::user::FilePath meshdir_;
    meshdir_ = FilePath(sim_spec_getString(compiler->meshdir));

    if (modelfiledir_.empty()) {
      modelfiledir_ = FilePath(model->modelfiledir_);
    }

    // remove path from file if necessary
    if (model->strippath) {
      file_ = sim_math_internal_strippath(file_);
    }

    FilePath filename = meshdir_ + FilePath(file_);
    resource_ = LoadResource(modelfiledir_.Str(), filename.Str(), vfs);

    // try loading from cache
    if (cache != nullptr && LoadCachedMesh(cache, resource_)) {
      sim_math_closeResource(resource_);
      resource_ = nullptr;
      fromCache = true;
    }

    if (!fromCache) {
      LoadFromResource(resource_);

      // check repeated mesh data
      if (!normal_.empty() && !spec_normal_.empty()) {
        throw sim_builder_error_t(this, "repeated normal specification");
      } else if (normal_.empty()) {
        normal_ = spec_normal_;
      }
      if (!texcoord_.empty() && !spec_texcoord_.empty()) {
        throw sim_builder_error_t(this, "repeated texcoord specification");
      } else if (texcoord_.empty()) {
        texcoord_ = spec_texcoord_;
      }
      if (!face_.empty() && !spec_face_.empty()) {
        throw sim_builder_error_t(this, "repeated face specification");
      } else if (face_.empty()) {
        face_ = spec_face_;
      }
      if (!vert_.empty() && !spec_vert_.empty()) {
        throw sim_builder_error_t(this, "repeated vertex specification");
      } else if (vert_.empty()) {
        ProcessVertices(spec_vert_);
      }
      if (!facenormal_.empty() && !spec_normal_.empty()) {
        throw sim_builder_error_t(this, "repeated facenormal specification");
      } else if (facenormal_.empty()) {
        facenormal_ = spec_facenormal_;
      }
      if (!facetexcoord_.empty() && !spec_facetexcoord_.empty()) {
        throw sim_builder_error_t(this, "repeated facetexcoord specification");
      } else if (facetexcoord_.empty()) {
        facetexcoord_ = spec_facetexcoord_;
      }
    }
  } else if (plugin.active) {
    LoadSDF();  // create using marching cubes
  }

  CheckInitialMesh();

  // compute mesh properties
  if (!fromCache) {
    Process();
  }

  // make octree
  if (!needsdf) {
    octree_.Clear();  // this occurs when a non-SDF mesh is loaded from a cached SDF mesh
  } else if (octree_.NumNodes() == 0) {
    octree_.SetFace(vert_, face_);
    octree_.CreateOctree(aamm_);

    // compute sdf coefficients
    if (!plugin.active) {
      tmd::TriangleMeshDistance sdf(vert_.data(), nvert(), face_.data(), nface());

      std::vector<double> coeffs(octree_.NumVerts());
      std::vector<bool> processed(octree_.NumVerts(), false);
      std::deque<int> queue;

      if (octree_.NumNodes() > 0) {
        queue.push_back(0);  // start traversal from the root node
      }

      while (!queue.empty()) {
        int node_idx = queue.front();
        queue.pop_front();

        for (int j = 0; j < 8; ++j) {
          int vert_id = octree_.VertId(node_idx, j);
          if (processed[vert_id]) {
            continue;
          }
          if (octree_.Hang(vert_id).empty()) {
            coeffs[vert_id] = sdf.signed_distance(octree_.Vert(vert_id)).distance;
          } else {
            double sum_coeff = 0;
            for (int dep_id : octree_.Hang(vert_id)) {
              sum_coeff += coeffs[dep_id];
              if (!processed[dep_id]) {
                throw sim_builder_error_t(this, "sdf coefficient computation failed");
              }
            }
            coeffs[vert_id] = sum_coeff / octree_.Hang(vert_id).size();
          }
          processed[vert_id] = true;
        }

        for (int child_idx : octree_.Children(node_idx)) {
          if (child_idx != -1) {
            queue.push_back(child_idx);
          }
        }
      }

      for (int i = 0; i < octree_.NumNodes(); ++i) {
        for (int j = 0; j < 8; j++) {
            octree_.AddCoeff(i, j, coeffs[octree_.VertId(i, j)]);
        }
      }
    }
  }

  // cache mesh
  if (!fromCache && !file_.empty()) {
    CacheMesh(cache, resource_);
  }

  // close resource
  if (resource_ != nullptr) {
    sim_math_closeResource(resource_);
    resource_ = nullptr;
  }
}



// get bounding volume
void sim_builder_mesh_t::SetBoundingVolume(int faceid) {
  constexpr double kMaxVal = std::numeric_limits<double>::max();
  double face_aamm[6] = {kMaxVal, kMaxVal, kMaxVal, -kMaxVal, -kMaxVal, -kMaxVal};

  for (int j = 0; j < 3; j++) {
    int vertid = face_[3*faceid + j];
    face_aamm[0] = std::min(face_aamm[0], vert_[3*vertid + 0]);
    face_aamm[1] = std::min(face_aamm[1], vert_[3*vertid + 1]);
    face_aamm[2] = std::min(face_aamm[2], vert_[3*vertid + 2]);
    face_aamm[3] = std::max(face_aamm[3], vert_[3*vertid + 0]);
    face_aamm[4] = std::max(face_aamm[4], vert_[3*vertid + 1]);
    face_aamm[5] = std::max(face_aamm[5], vert_[3*vertid + 2]);
  }

  face_aabb_.push_back(.5 * (face_aamm[0] + face_aamm[3]));
  face_aabb_.push_back(.5 * (face_aamm[1] + face_aamm[4]));
  face_aabb_.push_back(.5 * (face_aamm[2] + face_aamm[5]));
  face_aabb_.push_back(.5 * (face_aamm[3] - face_aamm[0]));
  face_aabb_.push_back(.5 * (face_aamm[4] - face_aamm[1]));
  face_aabb_.push_back(.5 * (face_aamm[5] - face_aamm[2]));

  tree_.AddBoundingVolume(faceid, 1, 1, center_ + 3*faceid, nullptr,
                          &face_aabb_[6*faceid]);
}



double* sim_builder_mesh_t::GetPosPtr() {
  return pos_;
}



double* sim_builder_mesh_t::GetQuatPtr() {
  return quat_;
}



bool sim_builder_mesh_t::HasTexcoord() const {
  return !texcoord_.empty();
}



void sim_builder_mesh_t::CopyVert(float* arr) const {
  for (int i = 0; i < vert_.size(); ++i) {
    arr[i] = (float)vert_[i];
  }
}



void sim_builder_mesh_t::CopyNormal(float* arr) const {
  std::copy(normal_.begin(), normal_.end(), arr);
}



void sim_builder_mesh_t::CopyFace(int* arr) const {
  std::copy(face_.begin(), face_.end(), arr);
}



void sim_builder_mesh_t::CopyFaceTexcoord(int* arr) const {
  std::copy(facetexcoord_.begin(), facetexcoord_.end(), arr);
}



void sim_builder_mesh_t::CopyFaceNormal(int* arr) const {
  std::copy(facenormal_.begin(), facenormal_.end(), arr);
}



void sim_builder_mesh_t::CopyTexcoord(float* arr) const {
  std::copy(texcoord_.begin(), texcoord_.end(), arr);
}



void sim_builder_mesh_t::CopyGraph(int* arr) const {
  std::copy(graph_, graph_+szgraph_, arr);
}



void sim_builder_mesh_t::CopyPolygons(int* verts, int* adr, int* num, int poly_adr) const {
  int n = polygons_.size(), count = 0;
  for (int i = 0; i < n; ++i) {
    int m = num[i] = polygons_[i].size();
    adr[i] = poly_adr + count;
    count += m;
    for (int j = 0; j < m; ++j) {
      verts[adr[i] + j - poly_adr] = polygons_[i][j];
    }
  }
}



void sim_builder_mesh_t::CopyPolygonMap(int* faces, int* adr, int* num, int poly_adr) const {
  int n = polygon_map_.size(), count = 0;
  for (int i = 0; i < n; ++i) {
    int m = num[i] = polygon_map_[i].size();
    adr[i] = poly_adr + count;
    count += m;
    for (int j = 0; j < m; ++j) {
      faces[adr[i] + j - poly_adr] = polygon_map_[i][j];
    }
  }
}



void sim_builder_mesh_t::CopyPolygonNormals(sim_scalar_t* arr) {
  for (int i = 0; i < polygon_normals_.size(); i += 3) {
    arr[i + 0] = (sim_scalar_t)polygon_normals_[i + 0];
    arr[i + 1] = (sim_scalar_t)polygon_normals_[i + 1];
    arr[i + 2] = (sim_scalar_t)polygon_normals_[i + 2];
  }
}



void sim_builder_mesh_t::DelTexcoord() {
  texcoord_.clear();
}



// set geom size to match mesh
void sim_builder_mesh_t::FitGeom(sim_builder_geom_t* geom, double center[3]) {
  // use inertial box
  if (!model->compiler.fitaabb) {
    // get inertia box type (shell or volume)
    double* boxsz = GetInertiaBoxPtr();
    switch (geom->type) {
      case SIM_GEOM_SPHERE:
        geom->size[0] = (boxsz[0] + boxsz[1] + boxsz[2])/3;
        break;

      case SIM_GEOM_CAPSULE:
        geom->size[0] = (boxsz[0] + boxsz[1])/2;
        geom->size[1] = max(0.0, boxsz[2] - geom->size[0]/2);
        break;

      case SIM_GEOM_CYLINDER:
        geom->size[0] = (boxsz[0] + boxsz[1])/2;
        geom->size[1] = boxsz[2];
        break;

      case SIM_GEOM_ELLIPSOID:
      case SIM_GEOM_BOX:
        geom->size[0] = boxsz[0];
        geom->size[1] = boxsz[1];
        geom->size[2] = boxsz[2];
        break;

      default:
        throw sim_builder_error_t(this, "invalid geom type in fitting mesh %s", name.c_str());
    }
  }

  // use aamm
  else {
    // find aabb box center and size
    center[0] = (aamm_[0]+aamm_[3])/2;
    center[1] = (aamm_[1]+aamm_[4])/2;
    center[2] = (aamm_[2]+aamm_[5])/2;
    double size[3] = {aamm_[3] - center[0], aamm_[4] - center[1], aamm_[5] - center[2]};

    // compute smallest geom whose aabb contains the mesh aabb
    switch (geom->type) {
      case SIM_GEOM_SPHERE:
        geom->size[0] = max(max(size[0], size[1]), size[2]);
        break;

      case SIM_GEOM_CAPSULE:
      case SIM_GEOM_CYLINDER:
        // find maximum distance in XY, separately in Z
        geom->size[0] = max(size[0], size[1]);
        geom->size[1] = size[2];

        // special handling of capsule: consider curved cap
        if (geom->type == SIM_GEOM_CAPSULE) {
          geom->size[1] -= geom->size[0];
        }
        break;

      case SIM_GEOM_ELLIPSOID:
      case SIM_GEOM_BOX:
        geom->size[0] = size[0];
        geom->size[1] = size[1];
        geom->size[2] = size[2];
        break;

      default:
        throw sim_builder_error_t(this, "invalid fittype in mesh %s", name.c_str());
    }
  }

  // rescale size
  geom->size[0] *= geom->fitscale;
  geom->size[1] *= geom->fitscale;
  geom->size[2] *= geom->fitscale;
}



// load OBJ mesh
void sim_builder_mesh_t::LoadOBJ(SIM_Resource* resource, bool remove_repeated) {
  tinyobj::ObjReader objReader;
  const void* bytes = nullptr;

  int buffer_sz = sim_math_readResource(resource, &bytes);
  if (buffer_sz < 0) {
    throw sim_builder_error_t(this, "could not read OBJ file '%s'", resource->name);
  }

  // TODO(etom): support .mtl files?
  const char* buffer = (const char*) bytes;
  objReader.ParseFromString(std::string(buffer, buffer_sz), std::string());

  if (!objReader.Valid()) {
    throw sim_builder_error_t(this, "could not parse OBJ file '%s'", resource->name);
  }

  const auto& attrib = objReader.GetAttrib();
  normal_ = attrib.normals;
  texcoord_ = attrib.texcoords;
  facenormal_.clear();
  facetexcoord_.clear();

  if (!objReader.GetShapes().empty()) {
    const auto& mesh = objReader.GetShapes()[0].mesh;
    bool righthand = scale[0] * scale[1] * scale[2] > 0;

    // iterate over mesh faces
    std::vector<tinyobj::index_t> face_indices;
    for (int face = 0, idx = 0; idx < mesh.indices.size();) {
      int nfacevert = mesh.num_face_vertices[face];
      if (nfacevert < 3 || nfacevert > 4) {
        throw sim_builder_error_t(
            this, "only tri or quad meshes are supported for OBJ (file '%s')",
            resource->name);
      }

      face_indices.push_back(mesh.indices[idx]);
      face_indices.push_back(mesh.indices[idx + (righthand == 1 ? 1 : 2)]);
      face_indices.push_back(mesh.indices[idx + (righthand == 1 ? 2 : 1)]);

      if (nfacevert == 4) {
        face_indices.push_back(mesh.indices[idx]);
        face_indices.push_back(mesh.indices[idx + (righthand == 1 ? 2 : 3)]);
        face_indices.push_back(mesh.indices[idx + (righthand == 1 ? 3 : 2)]);
      }
      idx += nfacevert;
      ++face;
    }

    // for each vertex, store index, normal, and texcoord
    for (const auto& mesh_index : face_indices) {
      face_.push_back(mesh_index.vertex_index);

      if (!normal_.empty()) {
        facenormal_.push_back(mesh_index.normal_index);
      }

      if (!texcoord_.empty()) {
        facetexcoord_.push_back(mesh_index.texcoord_index);
      }
    }
  }

  // flip the second texcoord
  for (int i=0; i < texcoord_.size()/2; i++) {
    texcoord_[2*i+1] = 1-texcoord_[2*i+1];
  }

  // copy vertex data
  ProcessVertices(attrib.vertices, remove_repeated);
}



// load mesh from cached asset, return true on success
bool sim_builder_mesh_t::LoadCachedMesh(SIM_CCache *cache, const SIM_Resource* resource) {
  auto process_mesh = [&](const void* data) {
    const sim_builder_mesh_t* mesh = static_cast<const sim_builder_mesh_t*>(data);
    // check if maxhullvert is different
    if (maxhullvert_ != mesh->maxhullvert_) {
      return false;
    }

    // check if inertia is different
    if (inertia != mesh->inertia) {
      return false;
    }

    // check if scale is different
    if (scale[0] != mesh->scale[0] ||
        scale[1] != mesh->scale[1] ||
        scale[2] != mesh->scale[2]) {
      return false;
    }

    // check if need hull
    if (needhull_ && !mesh->szgraph_) {
      return false;
    }

    processed_ = mesh->processed_;
    vert_ = mesh->vert_;
    normal_ = mesh->normal_;
    texcoord_ = mesh->texcoord_;
    face_ = mesh->face_;
    facenormal_ = mesh->facenormal_;
    facetexcoord_ = mesh->facetexcoord_;
    halfedge_ = mesh->halfedge_;

    // only copy graph if needed
    if (needhull_ || mesh->face_.empty()) {
      szgraph_ = mesh->szgraph_;
      graph_ = nullptr;
      if (szgraph_) {
        graph_ = (int*)sim_malloc(szgraph_*sizeof(int));
        std::copy(mesh->graph_, mesh->graph_ + szgraph_, graph_);
      }
    }

    polygons_ = mesh->polygons_;
    polygon_normals_ = mesh->polygon_normals_;
    polygon_map_ = mesh->polygon_map_;
    surface_ = mesh->surface_;
    volume_ = mesh->volume_;
    std::copy(mesh->boxsz_, mesh->boxsz_ + 3, boxsz_);
    std::copy(mesh->aamm_, mesh->aamm_ + 6, aamm_);
    std::copy(mesh->pos_, mesh->pos_ + 3, pos_);
    std::copy(mesh->quat_, mesh->quat_ + 4, quat_);

    center_ = nullptr;
    int ncenter = mesh->face_.size();
    if (ncenter) {
      center_ = (double*)sim_malloc(ncenter * sizeof(double));
      std::copy(mesh->center_, mesh->center_ + ncenter, center_);
    }
    tree_ = mesh->tree_;
    face_aabb_ = mesh->face_aabb_;
    octree_ = mesh->octree_;
    return true;
  };

  // check that cached asset has all data
  return cache->PopulateData(resource->name, resource, process_mesh);
}



// load STL binary mesh
void sim_builder_mesh_t::LoadSTL(SIM_Resource* resource) {
  bool righthand = scale[0] * scale[1] * scale[2] > 0;

  // get file data in buffer
  char* buffer = 0;
  int buffer_sz = sim_math_readResource(resource, (const void**)&buffer);

  // still not found
  if (buffer_sz < 0) {
    throw sim_builder_error_t(this, "could not read STL file '%s'", resource->name);
  } else if (!buffer_sz) {
    throw sim_builder_error_t(this, "STL file '%s' is empty", resource->name);
  }

  // make sure there is enough data for header
  if (buffer_sz < 84) {
    throw sim_builder_error_t(this, "invalid header in STL file '%s'", resource->name);
  }

  // get number of triangles, check bounds
  int nfaces = 0;
  ReadFromBuffer(&nfaces, buffer + 80);
  if (nfaces < 1 || nfaces > 200000) {
    throw sim_builder_error_t(this,
                   "number of faces should be between 1 and 200000 in STL file '%s';"
                   " perhaps this is an ASCII file?", resource->name);
  }

  // check remaining buffer size
  if (nfaces*50 != buffer_sz-84) {
    throw sim_builder_error_t(this,
                   "STL file '%s' has wrong size; perhaps this is an ASCII file?",
                   resource->name);
  }

  // assign stl data pointer
  const char* stl = buffer + 84;

  // allocate face and vertex data
  face_.assign(3*nfaces, 0);
  std::vector<float> vert;

  // add vertices and faces, including repeated for now
  for (int i=0; i < nfaces; i++) {
    for (int j=0; j < 3; j++) {
      // read vertex coordinates
      float v[3];
      ReadFromBuffer(&v, stl + 50*i + 12*(j + 1));

      // check if vertex can be cast to an int safely
      if (fabs(v[0]) > pow(2, 30) || fabs(v[1]) > pow(2, 30) || fabs(v[2]) > pow(2, 30)) {
        throw sim_builder_error_t(this, "vertex in STL file '%s' exceed maximum bounds", resource->name);
      }

      // add vertex address in face; change order if scale makes it lefthanded
      if (righthand || j == 0) {
        face_[3*i + j] = vert.size() / 3;
      } else {
        face_[3*i + 3 - j] = vert.size() / 3;
      }

      // add vertex data
      vert.push_back(v[0]);
      vert.push_back(v[1]);
      vert.push_back(v[2]);
    }
  }
  ProcessVertices(vert, true);
}



// load MSH binary mesh
void sim_builder_mesh_t::LoadMSH(SIM_Resource* resource, bool remove_repeated) {
  bool righthand = scale[0] * scale[1] * scale[2] > 0;

  // get file data in buffer
  char* buffer = 0;
  int buffer_sz = sim_math_readResource(resource, (const void**)&buffer);

  // still not found
  if (buffer_sz < 0) {
    throw sim_builder_error_t(this, "could not read MSH file '%s'", resource->name);
  } else if (!buffer_sz) {
    throw sim_builder_error_t(this, "MSH file '%s' is empty", resource->name);
  }

  // make sure header is present
  if (buffer_sz < 4*sizeof(int)) {
    throw sim_builder_error_t(this, "missing header in MSH file '%s'", resource->name);
  }

  // get sizes from header
  int nvbuf = 0, nfbuf = 0, nnbuf = 0, ntbuf = 0;
  ReadFromBuffer(&nvbuf, buffer);
  ReadFromBuffer(&nnbuf, buffer + sizeof(int));
  ReadFromBuffer(&ntbuf, buffer + 2*sizeof(int));
  ReadFromBuffer(&nfbuf, buffer + 3*sizeof(int));

  // check sizes
  if (nvbuf < 4 || nfbuf < 0 || nnbuf < 0 || ntbuf < 0 ||
      (nnbuf > 0 && nnbuf != nvbuf) ||
      (ntbuf > 0 && ntbuf != nvbuf)) {
    throw sim_builder_error_t(this, "invalid sizes in MSH file '%s'", resource->name);
  }

  if (nvbuf >= INT_MAX / sizeof(float) / 3 ||
      nnbuf >= INT_MAX / sizeof(float) / 3 ||
      ntbuf >= INT_MAX / sizeof(float) / 2 ||
      nfbuf >= INT_MAX / sizeof(int) / 3) {
    throw sim_builder_error_t(this, "too large sizes in MSH file '%s'.", resource->name);
  }
  // check file size
  if (buffer_sz != 4*sizeof(int) + 3*nvbuf*sizeof(float) + 3*nnbuf*sizeof(float) +
      2*ntbuf*sizeof(float) + 3*nfbuf*sizeof(int)) {
    throw sim_builder_error_t(this, "unexpected file size in MSH file '%s'", resource->name);
  }

  // allocate and copy
  using UnalignedFloat = char[sizeof(float)];
  auto fdata = reinterpret_cast<UnalignedFloat*>(buffer + 4*sizeof(int));
  std::vector<float> vert;
  int nvert = 0;
  if (nvbuf) {
    vert.assign(3*nvbuf, 0);
    nvert = 3*nvbuf;
    memcpy(vert.data(), fdata, nvert*sizeof(float));
    fdata += nvert;
  }
  if (nnbuf) {
    normal_.assign(nvert, 0);
    memcpy(normal_.data(), fdata, nvert*sizeof(float));
    fdata += nvert;
  }
  if (ntbuf) {
    texcoord_.assign(2*(nvert / 3), 0);
    memcpy(texcoord_.data(), fdata, 2*(nvert/3)*sizeof(float));
    fdata += 2*(nvert / 3);
  }
  if (nfbuf) {
    face_.assign(3*nfbuf, 0);
    facenormal_.assign(3*nfbuf, 0);
    memcpy(face_.data(), fdata, 3*nfbuf*sizeof(int));
    memcpy(facenormal_.data(), fdata, 3*nfbuf*sizeof(int));
  }
  if  (nfbuf && !texcoord_.empty()) {
    facetexcoord_.assign(3*nfbuf, 0);
    memcpy(facetexcoord_.data(), fdata, 3*nfbuf*sizeof(int));
  }

  // rearrange face data if left-handed scaling
  if (nfbuf && !righthand) {
    for (int i=0; i < nfbuf; i++) {
      int tmp = face_[3*i+1];
      face_[3*i+1] = face_[3*i+2];
      face_[3*i+2] = tmp;
    }
  }
  ProcessVertices(vert, remove_repeated);
}



// compute the volume and center-of-mass of the mesh given the face centroid
double sim_builder_mesh_t::ComputeVolume(double CoM[3], const double facecen[3]) const {
  double normal[3], center[3], total_volume = 0;
  CoM[0] = CoM[1] = CoM[2] = 0;
  int nf = (inertia == SIM_MESH_INERTIA_CONVEX) ? graph_[1] : nface();
  const int* f = (inertia == SIM_MESH_INERTIA_CONVEX) ? GraphFaces() : face_.data();

  for (int i = 0; i < nf; i++) {
    // get area, normal and center
    double area = triangle(normal, center, &vert_[3*f[3*i]], &vert_[3*f[3*i + 1]],
                           &vert_[3*f[3*i + 2]]);

    // compute and add volume
    double vec[3] = {center[0] - facecen[0], center[1] - facecen[1], center[2] - facecen[2]};
    double volume = sim_math_internal_dot3(vec, normal) * area / 3;

    // if legacy computation requested, then always positive
    if (inertia == SIM_MESH_INERTIA_LEGACY) {
      volume = std::abs(volume);
    }

    // add pyramid com
    total_volume += volume;
    CoM[0] += volume*(center[0]*3.0/4.0 + facecen[0]/4.0);
    CoM[1] += volume*(center[1]*3.0/4.0 + facecen[1]/4.0);
    CoM[2] += volume*(center[2]*3.0/4.0 + facecen[2]/4.0);
  }

  // if volume is valid normalize CoM
  if (total_volume >= SIM_MINVAL) {
    CoM[0] /= total_volume;
    CoM[1] /= total_volume;
    CoM[2] /= total_volume;
  }
  return total_volume;
}



// compute the surface area and center-of-mass of the mesh given the face centroid
double sim_builder_mesh_t::ComputeSurfaceArea(double CoM[3], const double facecen[3]) const {
  double surface = 0;
  CoM[0] = CoM[1] = CoM[2] = 0;
  for (int i = 0; i < nface(); i++) {
    // get area and center
    double area, center[3];
    area = triangle(nullptr, center, &vert_[3*face_[3*i]],
                    &vert_[3*face_[3*i + 1]], &vert_[3*face_[3*i + 2]]);

    // add pyramid com
    surface += area;
    CoM[0] += area*(center[0]*3.0/4.0 + facecen[0]/4.0);
    CoM[1] += area*(center[1]*3.0/4.0 + facecen[1]/4.0);
    CoM[2] += area*(center[2]*3.0/4.0 + facecen[2]/4.0);
  }

  // if area is valid normalize CoM
  if (surface >= SIM_MINVAL) {
    CoM[0] /= surface;
    CoM[1] /= surface;
    CoM[2] /= surface;
  }
  return surface;
}



// apply transformations
void sim_builder_mesh_t::ApplyTransformations() {
  // translate
  if (refpos[0] != 0 || refpos[1] != 0 || refpos[2] != 0) {
    for (int i = 0; i < nvert(); i++) {
      vert_[3*i + 0] -= refpos[0];
      vert_[3*i + 1] -= refpos[1];
      vert_[3*i + 2] -= refpos[2];
    }
  }

  // rotate
  if (refquat[0] != 1 || refquat[1] != 0 || refquat[2] != 0 || refquat[3] != 0) {
    // prepare rotation
    double quat[4] = {refquat[0], refquat[1], refquat[2], refquat[3]};
    double mat[9];
    sim_math_internal_normvec(quat, 4);
    sim_math_internal_quat2mat(mat, quat);

    // process vertices
    for (int i = 0; i < nvert(); i++) {
      sim_math_internal_mulvecmatT(&vert_[3*i], &vert_[3*i], mat);
    }

    // process normals
    for (int i = 0; i < nnormal(); i++) {
      double n1[3], n0[3] = {normal_[3*i], normal_[3*i+1], normal_[3*i+2]};
      sim_math_internal_mulvecmatT(n1, n0, mat);
      normal_[3*i] = (float) n1[0];
      normal_[3*i+1] = (float) n1[1];
      normal_[3*i+2] = (float) n1[2];
    }
  }

  // scale
  if (scale[0] != 1 || scale[1] != 1 || scale[2] != 1) {
    for (int i = 0; i < nvert(); i++) {
      vert_[3*i + 0] *= scale[0];
      vert_[3*i + 1] *= scale[1];
      vert_[3*i + 2] *= scale[2];
    }

    for (int i = 0; i < nnormal(); i++) {
      normal_[3*i + 0] *= scale[0];
      normal_[3*i + 1] *= scale[1];
      normal_[3*i + 2] *= scale[2];
    }
  }

  // normalize normals
  for (int i = 0; i < nnormal(); i++) {
    // compute length
    float len = normal_[3*i]*normal_[3*i] + normal_[3*i+1]*normal_[3*i+1] + normal_[3*i+2]*normal_[3*i+2];

    // rescale
    if (len > SIM_MINVAL) {
      float scl = 1/sqrtf(len);
      normal_[3*i + 0] *= scl;
      normal_[3*i + 1] *= scl;
      normal_[3*i + 2] *= scl;
    } else {
      normal_[3*i + 0] = 0;
      normal_[3*i + 1] = 0;
      normal_[3*i + 2] = 1;
    }
  }
}



// find centroid of faces, return total area
double sim_builder_mesh_t::ComputeFaceCentroid(double facecen[3]) const {
  double total_area = 0;

  for (int i = 0; i < nface(); i++) {
    // get area and center
    double area, center[3];
    area = triangle(nullptr, center, &vert_[3*face_[3*i]],
                    &vert_[3*face_[3*i + 1]], &vert_[3*face_[3*i + 2]]);

    // accumulate
    facecen[0] += area * center[0];
    facecen[1] += area * center[1];
    facecen[2] += area * center[2];
    total_area += area;
  }

  // finalize centroid of faces
  if (total_area >= SIM_MINVAL) {
    facecen[0] /= total_area;
    facecen[1] /= total_area;
    facecen[2] /= total_area;
  }
  return total_area;
}



void sim_builder_mesh_t::Process() {
  // create half-edge structure (if mesh was in XML)
  if (halfedge_.empty()) {
    for (int i = 0; i < nface(); i++) {
      int v0 = face_[3*i + 0];
      int v1 = face_[3*i + 1];
      int v2 = face_[3*i + 2];
      if (triangle(nullptr, nullptr, &vert_[3*v0], &vert_[3*v1], &vert_[3*v2]) > sqrt(SIM_MINVAL)) {
        halfedge_.push_back({v0, v1});
        halfedge_.push_back({v1, v2});
        halfedge_.push_back({v2, v0});
      } else {
        // TODO(b/255525326)
      }
    }
  }

  // check for inconsistent face orientations
  if (!halfedge_.empty()) {
    std::stable_sort(halfedge_.begin(), halfedge_.end());
    auto iterator = std::adjacent_find(halfedge_.begin(), halfedge_.end());
    if (iterator != halfedge_.end() && inertia == SIM_MESH_INERTIA_EXACT) {
      throw sim_builder_error_t(this,
                     "faces of mesh '%s' have inconsistent orientation. Please check the "
                     "faces containing the vertices %d and %d.",
                     name.c_str(), iterator->first + 1, iterator->second + 1);
    }
  }

  // make graph describing convex hull
  if (needhull_ || face_.empty()) {
    MakeGraph();
  }

  // no faces: copy from convex hull
  if (face_.empty()) {
    CopyGraph();
  }

  // no normals: make
  if (normal_.empty()) {
    MakeNormal();
  }

  // check facenormal size
  if (!facenormal_.empty() && facenormal_.size() != face_.size()) {
    throw sim_builder_error_t(this, "face data must have the same size as face normal data");
  }

  // no facetexcoord: copy from faces
  if (facetexcoord_.empty() && !texcoord_.empty()) {
    facetexcoord_ = face_;
  }

  // facenormal might not exist if usernormal was specified
  if (facenormal_.empty()) {
    int normal_per_vertex = normal_.size() / vert_.size();
    facenormal_.assign(face_.size(), 0);
    for (int i = 0; i < face_.size(); i++) {
      facenormal_[i] = normal_per_vertex * face_[i];
    }
  }

  MakePolygons();

  // user offset, rotation, scaling
  ApplyTransformations();

  // find centroid of faces
  double facecen[3] = {0, 0, 0};
  if (ComputeFaceCentroid(facecen) < SIM_MINVAL) {
    throw sim_builder_error_t(this, "mesh surface area is too small: %s", name.c_str());
  }

  // compute inertia and transform mesh. The mesh is transformed such that it is
  // centered at the CoM and the axes are the principle axes of inertia
  double CoM[3] = {0, 0, 0};
  double inert[6] = {0, 0, 0, 0, 0, 0};

  // compute CoM and volume/area
  if (inertia == SIM_MESH_INERTIA_SHELL) {
    surface_ = ComputeSurfaceArea(CoM, facecen);
    if (surface_ < SIM_MINVAL) {
      throw sim_builder_error_t(this, "mesh surface area is too small: %s", name.c_str());
    }
  } else {
    if ((volume_ = ComputeVolume(CoM, facecen)) < SIM_MINVAL) {
      if (volume_ < 0) {
        throw sim_builder_error_t(this, "mesh volume is negative (misoriented triangles): %s", name.c_str());
      } else {
        throw sim_builder_error_t(this, "mesh volume is too small: %s . Try setting inertia to shell",
                       name.c_str());
      }
    }
  }

  // compute inertia
  double total_volume = ComputeInertia(inert, CoM);
  if (inertia == SIM_MESH_INERTIA_SHELL) {
    surface_ = total_volume;
  } else {
    volume_ = total_volume;
  }

  // get quaternion and diagonal inertia
  double eigval[3], eigvec[9], quattmp[4];
  double full[9] = {
    inert[0], inert[3], inert[4],
    inert[3], inert[1], inert[5],
    inert[4], inert[5], inert[2]
  };
  sim_math_internal_eig3(eigval, eigvec, quattmp, full);

  constexpr double inequality_atol = 1e-9;
  constexpr double inequality_rtol = 1e-6;

  // check eigval - SHOULD NOT OCCUR
  if (eigval[2] <= 0) {
    throw sim_builder_error_t(this, "eigenvalue of mesh inertia must be positive: %s", name.c_str());
  }

  if (eigval[0] + eigval[1] < eigval[2] * (1.0 - inequality_rtol) - inequality_atol ||
      eigval[0] + eigval[2] < eigval[1] * (1.0 - inequality_rtol) - inequality_atol ||
      eigval[1] + eigval[2] < eigval[0] * (1.0 - inequality_rtol) - inequality_atol) {
    throw sim_builder_error_t(this, "eigenvalues of mesh inertia violate A + B >= C: %s", name.c_str());
  }

  // compute sizes of equivalent inertia box
  double volume = GetVolumeRef();
  boxsz_[0] = 0.5 * std::sqrt(6*(eigval[1] + eigval[2] - eigval[0])/volume);
  boxsz_[1] = 0.5 * std::sqrt(6*(eigval[0] + eigval[2] - eigval[1])/volume);
  boxsz_[2] = 0.5 * std::sqrt(6*(eigval[0] + eigval[1] - eigval[2])/volume);

  // prevent reorientation if the mesh was autogenerated using marching cubes
  if (!needreorient_) {
    sim_math_internal_set_vec(CoM, 0, 0, 0);
    sim_math_internal_set_vec(quattmp, 1, 0, 0, 0);
  }

  // transform CoM to origin
  for (int i=0; i < nvert(); i++) {
    vert_[3*i + 0] -= CoM[0];
    vert_[3*i + 1] -= CoM[1];
    vert_[3*i + 2] -= CoM[2];
  }
  Rotate(quattmp);

  // save the pos and quat that was used to transform the mesh
  sim_math_internal_copy_vec(pos_, CoM, 3);
  sim_math_internal_copy_vec(quat_, quattmp, 4);

  processed_ = true;

  // no radii: make
  if (!center_) {
    MakeCenter();
  }

  // recompute polygon normals
  MakePolygonNormals();

  // make bounding volume hierarchy
  if (tree_.Bvh().empty()) {
    face_aabb_.clear();
    face_aabb_.reserve(3*face_.size());
    tree_.AllocateBoundingVolumes(nface());
    for (int i = 0; i < nface(); i++) {
      SetBoundingVolume(i);
    }
    tree_.CreateBVH();
  }
}



// compute abstract (unitless) inertia, recompute area / volume
double sim_builder_mesh_t::ComputeInertia(double inert[6], const double CoM[3]) const {
  double total_volume = 0;

  // copy vertices to avoid modifying the original mesh
  std::vector<double> vert_centered;
  vert_centered.reserve(3*nvert());

  // translate vertices to origin in order to compute inertia
  for (int i =  0; i < nvert(); i++) {
    vert_centered.push_back(vert_[3*i + 0] - CoM[0]);
    vert_centered.push_back(vert_[3*i + 1] - CoM[1]);
    vert_centered.push_back(vert_[3*i + 2] - CoM[2]);
  }

  // accumulate products of inertia, recompute volume
  const int k[6][2] = {{0, 0}, {1, 1}, {2, 2}, {0, 1}, {0, 2}, {1, 2}};
  double P[6] = {0, 0, 0, 0, 0, 0};
  int nf = (inertia == SIM_MESH_INERTIA_CONVEX) ? graph_[1] : nface();
  const int* f = (inertia == SIM_MESH_INERTIA_CONVEX) ? GraphFaces() : face_.data();
  for (int i=0; i < nf; i++) {
    const double* D = &vert_centered[3*f[3*i + 0]];
    const double* E = &vert_centered[3*f[3*i + 1]];
    const double* F = &vert_centered[3*f[3*i + 2]];

    // get area, normal and center; update volume
    double normal[3], center[3];
    double volume, area = triangle(normal, center, D, E, F);
    if (inertia == SIM_MESH_INERTIA_SHELL) {
      volume = area;
    } else {
      volume = sim_math_internal_dot3(center, normal) * area / 3;
    }

    // if legacy computation requested, then always positive
    if (inertia == SIM_MESH_INERTIA_LEGACY) {
      volume = abs(volume);
    }

    // apply formula, accumulate
    total_volume += volume;

    int C = (inertia == SIM_MESH_INERTIA_SHELL) ? 12 : 20;
    for (int j = 0; j < 6; j++) {
      P[j] += volume /
              C * (
        2*(D[k[j][0]] * D[k[j][1]] +
           E[k[j][0]] * E[k[j][1]] +
           F[k[j][0]] * F[k[j][1]]) +
        D[k[j][0]] * E[k[j][1]]  +  D[k[j][1]] * E[k[j][0]] +
        D[k[j][0]] * F[k[j][1]]  +  D[k[j][1]] * F[k[j][0]] +
        E[k[j][0]] * F[k[j][1]]  +  E[k[j][1]] * F[k[j][0]]);
    }
  }

  // convert from products of inertia to moments of inertia
  inert[0] = P[1] + P[2];
  inert[1] = P[0] + P[2];
  inert[2] = P[0] + P[1];
  inert[3] = -P[3];
  inert[4] = -P[4];
  inert[5] = -P[5];
  return total_volume;
}



void sim_builder_mesh_t::Rotate(double quat[4]) {
  // rotate vertices and normals of mesh by quaternion
  double neg[4] = {quat[0], -quat[1], -quat[2], -quat[3]};
  double mat[9];
  sim_math_internal_quat2mat(mat, neg);
  for (int i = 0; i < nvert(); i++) {
    sim_math_internal_mulvecmat(&vert_[3*i], &vert_[3*i], mat);

    // axis-aligned bounding box
    aamm_[0] = std::min(aamm_[0], vert_[3*i + 0]);
    aamm_[3] = std::max(aamm_[3], vert_[3*i + 0]);
    aamm_[1] = std::min(aamm_[1], vert_[3*i + 1]);
    aamm_[4] = std::max(aamm_[4], vert_[3*i + 1]);
    aamm_[2] = std::min(aamm_[2], vert_[3*i + 2]);
    aamm_[5] = std::max(aamm_[5], vert_[3*i + 2]);
  }

  for (int i=0; i < nnormal(); i++) {
    // normals
    const double nrm[3] = {normal_[3*i], normal_[3*i+1], normal_[3*i+2]};
    double res[3];
    sim_math_internal_mulvecmat(res, nrm, mat);
    for (int j=0; j < 3; j++) {
      normal_[3*i+j] = (float) res[j];
    }
  }
}
void sim_builder_mesh_t::CheckInitialMesh() const {
  if (vert_.size() < 12) {
    throw sim_builder_error_t(this, "at least 4 vertices required");
  }
  if (vert_.size() % 3) {
    throw sim_builder_error_t(this, "vertex data must be a multiple of 3");
  }
  if (normal_.size() % 3) {
    throw sim_builder_error_t(this, "normal data must be a multiple of 3");
  }
  if (texcoord_.size() % 2) {
    throw sim_builder_error_t(this, "texcoord must be a multiple of 2");
  }
  if (face_.size() % 3) {
    throw sim_builder_error_t(this, "face data must be a multiple of 3");
  }

  // check texcoord size if no face texcoord indices are given
  if (!texcoord_.empty() && texcoord_.size() != 2 * nvert() &&
      facetexcoord_.empty() && !IsObj()) {
    throw sim_builder_error_t(this,
        "texcoord must be 2*nv if face texcoord indices are not provided in an OBJ file");
  }

  // require vertices
  if (vert_.empty()) {
    throw sim_builder_error_t(this, "no vertices");
  }

  // check vertices exist
  for (int i = 0; i < face_.size(); i++) {
    if (face_[i] >= nvert() || face_[i] < 0) {
      throw sim_builder_error_t(this, "in face %d, vertex index %d does not exist",
                     nullptr, i / 3, face_[i]);
    }
  }
}



// return inertia pointer
double* sim_builder_mesh_t::GetInertiaBoxPtr() {
  return boxsz_;
}



// return volume or surface area
double sim_builder_mesh_t::GetVolumeRef() const {
  return (inertia == SIM_MESH_INERTIA_SHELL) ? surface_ : volume_;
}



// make graph describing convex hull
void sim_builder_mesh_t::MakeGraph() {
  int adr, ok, curlong, totlong, exitcode;
  facetT* facet, **facetp;
  vertexT* vertex, *vertex1, **vertex1p;

  std::string qhopt = "qhull Qt";
  if (maxhullvert_ > -1) {
    // qhull "TA" actually means "number of vertices added after the initial simplex"
    qhopt += " TA" + std::to_string(maxhullvert_ - 4);
  }

  // graph not needed for small meshes
  if (nvert() < 4) {
    return;
  }

  qhT qh_qh;
  qhT* qh = &qh_qh;
  qh_zero(qh, stderr);

  // qhull basic init
  qh_init_A(qh, stdin, stdout, stderr, 0, nullptr);

  // install longjmp error handler
  exitcode = setjmp(qh->errexit);
  qh->NOerrexit = false;
  if (!exitcode) {
    // actual init
    qh_initflags(qh, const_cast<char*>(qhopt.c_str()));
    qh_init_B(qh, vert_.data(), nvert(), 3, qh_False);

    // construct convex hull
    qh_qhull(qh);
    qh_triangulate(qh);
    qh_vertexneighbors(qh);

    // allocate graph:
    //  numvert, numface, vert_edgeadr[numvert], vert_globalid[numvert],
    //  edge_localid[numvert+3*numface], face_globalid[3*numface]
    int numvert = qh->num_vertices;
    int numface = qh->num_facets;
    szgraph_ = 2 + 3*numvert + 6*numface;
    graph_ = (int*) sim_malloc(szgraph_*sizeof(int));
    graph_[0] = numvert;
    graph_[1] = numface;

    // pointers for convenience
    int* vert_edgeadr = graph_ + 2;
    int* vert_globalid = graph_ + 2 + numvert;
    int* edge_localid = graph_ + 2 + 2*numvert;
    int* face_globalid = graph_ + 2 + 3*numvert + 3*numface;

    // fill in graph data
    int i = adr = 0;
    ok = 1;
    FORALLvertices {
      // point id of this vertex, check
      int pid = qh_pointid(qh, vertex->point);
      if (pid < 0 || pid >= nvert()) {
        ok = 0;
        break;
      }

      // save edge address and global id of this vertex
      vert_edgeadr[i] = adr;
      vert_globalid[i] = pid;

      // process neighboring faces and their vertices
      int start = adr;
      FOREACHsetelement_(facetT, vertex->neighbors, facet) {
        int cnt = 0;
        FOREACHsetelement_(vertexT, facet->vertices, vertex1) {
          cnt++;

          // point id of face vertex, check
          int pid1 = qh_pointid(qh, vertex1->point);
          if (pid1 < 0 || pid1 >= nvert()) {
            ok = 0;
            break;
          }

          // if different from vertex id, try to insert
          if (pid != pid1) {
            // check for previous record
            int j;
            for (j=start; j < adr; j++)
              if (pid1 == edge_localid[j]) {
                break;
              }

            // not found: insert
            if (j >= adr) {
              edge_localid[adr++] = pid1;
            }
          }
        }

        // make sure we have triangle: SHOULD NOT OCCUR
        if (cnt != 3) {
          sim_error("Qhull did not return triangle");
        }
      }

      // insert separator, advance to next vertex
      edge_localid[adr++] = -1;
      i++;
    }

    // size check: SHOULD NOT OCCUR
    if (adr != numvert+3*numface) {
      sim_error("Wrong size in convex hull graph");
    }

    // add triangle data, reorient faces if flipped
    adr = 0;
    FORALLfacets {
      int ii = 0;
      int ind[3] = {0, 1, 2};
      if (facet->toporient) {
        ind[0] = 1;
        ind[1] = 0;
      }

      // copy triangle data
      FOREACHsetelement_(vertexT, facet->vertices, vertex1) {
        // make sure we have triangle: SHOULD NOT OCCUR
        if (ii >= 3) {
          sim_error("Qhull did not return triangle");
        }

        face_globalid[adr + ind[ii++]] = qh_pointid(qh, vertex1->point);
      }

      // advance to next triangle
      adr += 3;
    }

    // free all
    qh_freeqhull(qh, !qh_ALL);
    qh_memfreeshort(qh, &curlong, &totlong);

    // bad graph: delete
    if (!ok) {
      szgraph_ = 0;
      sim_free(graph_);
      graph_ = 0;
      sim_warning("Could not construct convex hull graph");
    }

    // replace global ids with local ids in edge data
    for (int i=0; i < numvert+3*numface; i++) {
      if (edge_localid[i] >= 0) {
        // search vert_globalid for match
        int adr;
        for (adr=0; adr < numvert; adr++) {
          if (vert_globalid[adr] == edge_localid[i]) {
            edge_localid[i] = adr;
            break;
          }
        }

        // make sure we found a match: SHOULD NOT OCCUR
        if (adr >= numvert) {
          sim_error("Vertex id not found in convex hull");
        }
      }
    }
  }

  // longjmp error handler
  else {
    // free all
    qh_freeqhull(qh, !qh_ALL);
    qh_memfreeshort(qh, &curlong, &totlong);
    if (graph_) {
      sim_free(graph_);
      szgraph_ = 0;
    }

    throw sim_builder_error_t(this, "qhull error");
  }
}



// copy graph into face data
void sim_builder_mesh_t::CopyGraph() {
  // only if face data is missing
  if (!face_.empty()) {
    return;
  }

  // get info from graph, allocate
  int numvert = graph_[0];
  face_.assign(3*graph_[1], 0);

  // copy faces
  for (int i=0; i < nface(); i++) {
    // address in graph
    int j = 2 + 3*numvert + 3*nface() + 3*i;

    // copy
    face_[3*i + 0] = graph_[j + 0];
    face_[3*i + 1] = graph_[j + 1];
    face_[3*i + 2] = graph_[j + 2];
  }
}



// make a mesh of a hemisphere (quad projected)
void sim_builder_mesh_t::MakeHemisphere(int res, bool make_faces, bool make_cap) {
  constexpr double kNorthPole[3] = {0, 0, 1};
  constexpr double kEquator[4][3] = {
      {1, 0, 0},
      {0, 1, 0},
      {-1, 0, 0},
      {0, -1, 0},
  };

  // allocate vertices
  int nvert = 1 + 2 * (res + 1) * (res + 2);
  nvert += make_cap && make_faces;  // add center vertex for bottom cap faces
  std::vector<float> vert(3 * nvert);

  // north pole
  vert[0] = kNorthPole[0];
  vert[1] = kNorthPole[1];
  vert[2] = kNorthPole[2];

  // iterate through rows from north pole to equator, compute vertices
  int v = 1;
  for (int row = 0; row <= res; row++) {
    // iterate through the four sides
    for (int side = 0; side < 4; side++) {
      double factor = static_cast<double>(row + 1) / (res + 1);
      double start[3], end[3];

      // start and end points of current arc
      for (int i = 0; i < 3; i++) {
        start[i] = kNorthPole[i] + factor * (kEquator[side][i] - kNorthPole[i]);
        end[i] = kNorthPole[i] + factor * (kEquator[(side + 1) % 4][i] - kNorthPole[i]);
      }

      // step size for interpolation along the arc
      double delta[3];
      for (int i = 0; i < 3; i++) {
        delta[i] = (end[i] - start[i]) / (row + 1);
      }

      // interpolate points along the arc
      for (int i = 0; i < row + 1; i++) {
        double p[3];
        for (int j = 0; j < 3; j++) {
          p[j] = start[j] + i * delta[j];
        }

        // normalize point to lie on hemisphere surface
        double norm = std::sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);
        vert[3 * v + 0] = p[0] / norm;
        vert[3 * v + 1] = p[1] / norm;
        vert[3 * v + 2] = p[2] / norm;
        v++;
      }
    }
  }

  // optional center vertex for bottom cap (for symmetry)
  if (make_faces && make_cap) {
    vert[3 * (nvert - 1) + 0] = 0;
    vert[3 * (nvert - 1) + 1] = 0;
    vert[3 * (nvert - 1) + 2] = 0;
  }

  // save vertices
  sim_spec_setFloat(spec.uservert, vert.data(), 3 * nvert);

  if (make_faces) {
    // allocate faces
    int nface = 4 * (res + 1) * (res + 1);
    nface += make_cap * (4 * (res + 1));  // bottom cap faces
    std::vector<int> face(3 * nface);

    // faces connected to north pole
    int f = 0;
    face[f++] = 0; face[f++] = 1; face[f++] = 2;
    face[f++] = 0; face[f++] = 2; face[f++] = 3;
    face[f++] = 0; face[f++] = 3; face[f++] = 4;
    face[f++] = 0; face[f++] = 4; face[f++] = 1;

    // faces on the hemisphere from north pole to equator
    for (int row = 0; row < res; row++) {
      const int start_curr = 2 * row * (row + 1) + 1;
      const int count_curr = 4 * (row + 1);
      const int start_next = start_curr + count_curr;
      const int count_next = 4 * (row + 2);

      for (int side = 0; side < 4; side++) {
        for (int i = 0; i < row + 2; i++) {
          const int v_curr = i + (row + 1) * side;
          const int v_next = i + (row + 2) * side;
          face[f++] = start_curr + v_curr % count_curr;
          face[f++] = start_next + v_next % count_next;
          face[f++] = start_next + (v_next + 1) % count_next;

          if (i < row + 1) {
            face[f++] = start_curr + v_curr % count_curr;
            face[f++] = start_next + (v_next + 1) % count_next;
            face[f++] = start_curr + (v_curr + 1) % count_curr;
          }
        }
      }
    }

    if (make_cap) {
      // add faces for the bottom cap
      const int start = 2 * res * (res + 1) + 1;
      const int count = 4 * (res + 1);
      for (int i = 0; i < count; i++) {
        face[f++] = start + i;
        face[f++] = nvert - 1;
        face[f++] = start + (i + 1) % count;
      }
    }

    // save faces
    sim_spec_setInt(spec.userface, face.data(), 3 * nface);
  }
}



// make a mesh of a sphere using icosaheral subdivision
void sim_builder_mesh_t::MakeSphere(int subdiv, bool make_faces) {
  // make icosahedron
  const float phi = (1.0 + std::sqrt(5.0)) / 2.0;
  std::vector<float> vert = {
      -1.0,  phi, 0.0,
       1.0,  phi, 0.0,
      -1.0, -phi, 0.0,
       1.0, -phi, 0.0,

       0.0, -1.0,  phi,
       0.0,  1.0,  phi,
       0.0, -1.0, -phi,
       0.0,  1.0, -phi,

       phi,  0.0, -1.0,
       phi,  0.0,  1.0,
      -phi,  0.0, -1.0,
      -phi,  0.0,  1.0,
  };

  // normalize vertices to be on a unit sphere
  const double norm = std::sqrt(1.0 + phi * phi);
  for (float& v : vert) {
    v /= norm;
  }

  std::vector<int> face = {
    0,  11, 5,    0,  5,  1,    0,  1,  7,    0,  7,  10,   0,  10, 11,
    1,  5,  9,    5,  11, 4,    11, 10, 2,    10, 7,  6,    7,  1,  8,
    3,  9,  4,    3,  4,  2,    3,  2,  6,    3,  6,  8,    3,  8,  9,
    4,  9,  5,    2,  4,  11,   6,  2,  10,   8,  6,  7,    9,  8,  1
  };

  // subdivision
  if (subdiv > 0) {
    // helper to get or create a midpoint vertex
    auto get_midpoint = [&vert](
        int v1_idx, int v2_idx, std::map<std::pair<int, int>, int>& cache) -> int {
      // key is the pair of vertex indices, sorted
      std::pair<int, int> key = std::minmax(v1_idx, v2_idx);

      // if midpoint is already in cache, return its index
      auto it = cache.find(key);
      if (it != cache.end()) {
        return it->second;
      }

      // otherwise, create it
      const float* v1 = &vert[v1_idx * 3];
      const float* v2 = &vert[v2_idx * 3];

      float mid_x = (v1[0] + v2[0]) / 2.0f;
      float mid_y = (v1[1] + v2[1]) / 2.0f;
      float mid_z = (v1[2] + v2[2]) / 2.0f;

      // normalize the new vertex to put it on the sphere
      float mid_norm = std::sqrt(mid_x * mid_x + mid_y * mid_y + mid_z * mid_z);
      mid_x /= mid_norm;
      mid_y /= mid_norm;
      mid_z /= mid_norm;

      // add the new vertex to the list
      int new_idx = vert.size() / 3;
      vert.push_back(mid_x);
      vert.push_back(mid_y);
      vert.push_back(mid_z);

      // add to cache
      cache[key] = new_idx;

      return new_idx;
    };

    // subdivision loop
    for (int i = 0; i < subdiv; ++i) {
      std::map<std::pair<int, int>, int> midpoint_cache;
      std::vector<int> new_face;
      new_face.reserve(face.size() * 4);
      for (size_t j = 0; j < face.size(); j += 3) {
        int v1 = face[j];
        int v2 = face[j+1];
        int v3 = face[j+2];

        int m12 = get_midpoint(v1, v2, midpoint_cache);
        int m23 = get_midpoint(v2, v3, midpoint_cache);
        int m31 = get_midpoint(v3, v1, midpoint_cache);

        new_face.insert(new_face.end(), {v1, m12, m31});
        new_face.insert(new_face.end(), {v2, m23, m12});
        new_face.insert(new_face.end(), {v3, m31, m23});
        new_face.insert(new_face.end(), {m12, m23, m31});
      }
      face = std::move(new_face);
    }
  }

  // save vertices and maybe faces
  sim_spec_setFloat(spec.uservert, vert.data(), vert.size());
  if (make_faces) sim_spec_setInt(spec.userface, face.data(), face.size());
}



// make a mesh of a supersphere
void sim_builder_mesh_t::MakeSupersphere(int res, double e, double n) {
  // allocate vertices and faces
  int nvert = (res - 1) * res + 2;
  int nface = 2 * res * (res - 1);
  std::vector<float> vert;
  vert.reserve(3 * nvert);
  std::vector<int> face;
  face.reserve(3 * nface);

  // south pole
  vert.insert(vert.end(), {0.0f, 0.0f, -1.0f});

  // rings
  for (int i = 1; i < res; i++) {
    double v = -SIM_PI/2 + i * SIM_PI / res;
    for (int j = 0; j < res; j++) {
      double u = -SIM_PI + j * 2 * SIM_PI / res;
      vert.push_back(aux_c(v, n) * aux_c(u, e));
      vert.push_back(aux_c(v, n) * aux_s(u, e));
      vert.push_back(aux_s(v, n));
    }
  }

  // north pole
  vert.insert(vert.end(), {0.0f, 0.0f, 1.0f});

  // south pole faces
  for (int j = 0; j < res; j++) {
    int v2 = 1 + j;
    int v3 = 1 + (j + 1) % res;
    face.insert(face.end(), {0, v3, v2});
  }

  // ring faces
  for (int i = 0; i < res - 2; i++) {
    for (int j = 0; j < res; j++) {
      int v1 = 1 + i * res + j;
      int v2 = 1 + i * res + (j + 1) % res;
      int v4 = 1 + (i + 1) * res + j;
      int v3 = 1 + (i + 1) * res + (j + 1) % res;
      face.insert(face.end(), {v1, v2, v4});
      face.insert(face.end(), {v2, v3, v4});
    }
  }

  // north pole faces
  int north_pole_idx = nvert - 1;
  int last_ring_start_idx = 1 + (res - 2) * res;
  for (int j = 0; j < res; j++) {
    int v1 = last_ring_start_idx + j;
    int v2 = last_ring_start_idx + (j + 1) % res;
    face.insert(face.end(), {v1, v2, north_pole_idx});
  }

  // save vertices and faces
  sim_spec_setFloat(spec.uservert, vert.data(), vert.size());
  sim_spec_setInt(spec.userface, face.data(), face.size());
}



// make a mesh of a torus (subsumed by supertorus, kept for reference only)
void sim_builder_mesh_t::MakeTorus(int res, double radius) {
  // allocate vertices and faces
  int nvert = res * res;
  int nface = res * res * 2;
  std::vector<float> vert(3 * nvert);
  std::vector<int> face(3 * nface);

  // generate vertices
  for (int i = 0; i < res; ++i) {
    for (int j = 0; j < res; ++j) {
      double u = 2 * SIM_PI * i / res;
      double v = 2 * SIM_PI * j / res;
      int vidx = i * res + j;
      vert[3 * vidx + 0] = (1 + radius * cos(v)) * cos(u);
      vert[3 * vidx + 1] = (1 + radius * cos(v)) * sin(u);
      vert[3 * vidx + 2] = radius * sin(v);
    }
  }

  // generate faces
  int fidx = 0;
  for (int i = 0; i < res; ++i) {
    for (int j = 0; j < res; ++j) {
      int i_next = (i + 1) % res;
      int j_next = (j + 1) % res;

      int v1 = i * res + j;
      int v2 = i_next * res + j;
      int v3 = i_next * res + j_next;
      int v4 = i * res + j_next;

      // first triangle
      face[3 * fidx + 0] = v1;
      face[3 * fidx + 1] = v2;
      face[3 * fidx + 2] = v4;
      fidx++;

      // second triangle
      face[3 * fidx + 0] = v2;
      face[3 * fidx + 1] = v3;
      face[3 * fidx + 2] = v4;
      fidx++;
    }
  }

  // save vertices and faces
  sim_spec_setFloat(spec.uservert, vert.data(), vert.size());
  sim_spec_setInt(spec.userface, face.data(), face.size());
}



// make a mesh of a supertoroid, see https://en.wikipedia.org/wiki/Supertoroid
void sim_builder_mesh_t::MakeSupertorus(int res, double radius, double s, double t) {
  // allocate vertices and faces
  int nvert = res * res;
  int nface = res * res * 2;
  std::vector<float> vert(3 * nvert);
  std::vector<int> face(3 * nface);

  // generate vertices
  for (int i = 0; i < res; ++i) {
    for (int j = 0; j < res; ++j) {
      double u = 2 * SIM_PI * i / res;
      double v = 2 * SIM_PI * j / res;
      int vidx = i * res + j;
      vert[3 * vidx + 0] = (1 + radius * aux_c(v, s)) * aux_c(u, t);
      vert[3 * vidx + 1] = (1 + radius * aux_c(v, s)) * aux_s(u, t);
      vert[3 * vidx + 2] = radius * aux_s(v, s);
    }
  }

  // generate faces
  int fidx = 0;
  for (int i = 0; i < res; ++i) {
    for (int j = 0; j < res; ++j) {
      int i_next = (i + 1) % res;
      int j_next = (j + 1) % res;

      int v1 = i * res + j;
      int v2 = i_next * res + j;
      int v3 = i_next * res + j_next;
      int v4 = i * res + j_next;

      // first triangle
      face[3 * fidx + 0] = v1;
      face[3 * fidx + 1] = v2;
      face[3 * fidx + 2] = v4;
      fidx++;

      // second triangle
      face[3 * fidx + 0] = v2;
      face[3 * fidx + 1] = v3;
      face[3 * fidx + 2] = v4;
      fidx++;
    }
  }

  // save vertices and faces
  sim_spec_setFloat(spec.uservert, vert.data(), vert.size());
  sim_spec_setInt(spec.userface, face.data(), face.size());
}



// make a mesh of a spherical wedge
void sim_builder_mesh_t::MakeWedge(int resolution[2], double fov[2], double gamma) {
  std::vector<double> x_edges(resolution[0] + 1, 0);
  std::vector<double> y_edges(resolution[1] + 1, 0);
  BinEdges(x_edges.data(), y_edges.data(), resolution, fov, gamma);
  std::vector<float> uservert(3 * resolution[0] * resolution[1], 0);
  std::vector<float> usernormal(9 * resolution[0] * resolution[1], 0);

  for (int i = 0; i < resolution[0]; i++) {
    for (int j = 0; j < resolution[1]; j++) {
      double aer[3];
      aer[0] = 0.5 * (x_edges[i + 1] + x_edges[i]);
      aer[1] = 0.5 * (y_edges[j + 1] + y_edges[j]);
      aer[2] = 1;
      SphericalToCartesian(aer, uservert.data() + 3 * (i * resolution[1] + j));
      TangentFrame(aer, usernormal.data() + 9 * (i * resolution[1] + j));
    }
  }

  sim_spec_setFloat(spec.uservert, uservert.data(),
               3 * resolution[0] * resolution[1]);
  sim_spec_setFloat(spec.usernormal, usernormal.data(),
               9 * resolution[0] * resolution[1]);
}



// make a mesh of a rectangle
void sim_builder_mesh_t::MakeRect(int resolution[2]) {
  std::vector<double> x_edges(resolution[0] + 1, 0);
  std::vector<double> y_edges(resolution[1] + 1, 0);
  LinSpace(-1, 1, resolution[0] + 1, x_edges.data());
  LinSpace(-1, 1, resolution[1] + 1, y_edges.data());
  std::vector<float> uservert(3 * resolution[0] * resolution[1], 0);
  std::vector<float> usernormal(9 * resolution[0] * resolution[1], 0);
  std::vector<int> userface(6 * (resolution[0] - 1) * (resolution[1] - 1), 0);
  spec.inertia = SIM_MESH_INERTIA_SHELL;

  for (int i = 0; i < resolution[0]; i++) {
    for (int j = 0; j < resolution[1]; j++) {
      int vert = i * resolution[1] + j;
      sim_scalar_t dx = 2. / resolution[0];
      sim_scalar_t dy = 2. / resolution[1];
      uservert[3 * vert + 0] = -1 + (i + 0.5) * dx;
      uservert[3 * vert + 1] = -1 + (j + 0.5) * dy;
      uservert[3 * vert + 2] = -1;
      usernormal[9 * vert + 0] = 1;
      usernormal[9 * vert + 4] = 1;
      usernormal[9 * vert + 8] = 1;
      if (i > 0 && j > 0) {
        int cell = (i - 1) * (resolution[1] - 1) + j - 1;
        userface[6 * cell + 0] = (i - 1) * resolution[1] + j - 1;
        userface[6 * cell + 1] = (i - 0) * resolution[1] + j - 1;
        userface[6 * cell + 2] = (i - 1) * resolution[1] + j - 0;
        userface[6 * cell + 3] = (i - 0) * resolution[1] + j - 0;
        userface[6 * cell + 4] = (i - 1) * resolution[1] + j - 0;
        userface[6 * cell + 5] = (i - 0) * resolution[1] + j - 1;
      }
    }
  }

  sim_spec_setFloat(spec.uservert, uservert.data(),
               3 * resolution[0] * resolution[1]);
  sim_spec_setFloat(spec.usernormal, usernormal.data(),
               9 * resolution[0] * resolution[1]);
  sim_spec_setInt(spec.userface, userface.data(),
             6 * (resolution[0] - 1) * (resolution[1] - 1));
}



// make a mesh of a generalized discrete cone
void sim_builder_mesh_t::MakeCone(int nedge, double radius) {
  int n = 3 * (nedge + (radius > 0 ? nedge : 1));
  std::vector<float> uservert(n, 0);

  // bottom face
  for (int i = 0; i < nedge; i++) {
    uservert[3 * i + 0] = cos(2 * i * SIM_PI / nedge);
    uservert[3 * i + 1] = sin(2 * i * SIM_PI / nedge);
    uservert[3 * i + 2] = -1;
  }

  // top face or single point
  if (radius > 0) {
    for (int i = nedge; i < 2 * nedge; i++) {
      uservert[3 * i + 0] = radius * cos(2 * i * SIM_PI / nedge);
      uservert[3 * i + 1] = radius * sin(2 * i * SIM_PI / nedge);
      uservert[3 * i + 2] = 1;
    }
  } else {
    uservert[3 * nedge + 2] = 1;
  }

  sim_spec_setFloat(spec.uservert, uservert.data(), n);
}



// compute vertex normals
void sim_builder_mesh_t::MakeNormal() {
  // only if normal data is missing
  if (!normal_.empty()) {
    return;
  }

  // allocate and clear normals
  normal_.assign(3*nvert(), 0);

  if (facenormal_.empty()) {
    facenormal_.assign(3*nface(), 0);
  }

  // loop over faces, accumulate vertex normals
  for (int i=0; i < nface(); i++) {
    // get vertex ids
    int vertid[3];
    for (int j=0; j < 3; j++) {
      vertid[j] = face_[3*i+j];
    }

    // get triangle edges
    double vec01[3], vec02[3];
    for (int j=0; j < 3; j++) {
      vec01[j] = vert_[3*vertid[1]+j] - vert_[3*vertid[0]+j];
      vec02[j] = vert_[3*vertid[2]+j] - vert_[3*vertid[0]+j];
    }

    // compute face normal
    double nrm[3];
    sim_math_internal_crossvec(nrm, vec01, vec02);
    double area = sim_math_internal_normvec(nrm, 3);

    // add normal to each vertex with weight = area
    for (int j=0; j < 3; j++) {
      for (int k=0; k < 3; k++) {
        normal_[3*vertid[j]+k] += nrm[k]*area;
      }
      facenormal_[3*i+j] = vertid[j];
    }
  }

  // remove large-angle faces
  if (!smoothnormal) {
    // allocate removal and clear
    float* nremove = (float*) sim_malloc(3*nnormal()*sizeof(float));
    memset(nremove, 0, 3*nnormal()*sizeof(float));

    // remove contributions from faces at large angles with vertex normal
    for (int i=0; i < nface(); i++) {
      // get vertex ids
      int vertid[3];
      for (int j=0; j < 3; j++) {
        vertid[j] = face_[3*i+j];
      }

      // get triangle edges
      double vec01[3], vec02[3];
      for (int j=0; j < 3; j++) {
        vec01[j] = vert_[3*vertid[1]+j] - vert_[3*vertid[0]+j];
        vec02[j] = vert_[3*vertid[2]+j] - vert_[3*vertid[0]+j];
      }

      // compute face normal
      double nrm[3];
      sim_math_internal_crossvec(nrm, vec01, vec02);
      double area = sim_math_internal_normvec(nrm, 3);

      // compare to vertex normal, subtract contribution if dot product too small
      for (int j=0; j < 3; j++) {
        // normalized vertex normal
        double vnrm[3] = {normal_[3*vertid[j]], normal_[3*vertid[j]+1], normal_[3*vertid[j]+2]};
        sim_math_internal_normvec(vnrm, 3);

        // dot too small: remove
        if (sim_math_internal_dot3(nrm, vnrm) < 0.8) {
          for (int k=0; k < 3; k++) {
            nremove[3*vertid[j]+k] += nrm[k]*area;
          }
        }
      }
    }

    // apply removal, free nremove
    for (int i=0; i < 3*nnormal(); i++) {
      normal_[i] -= nremove[i];
    }
    sim_free(nremove);
  }

  // normalize normals
  for (int i=0; i < nnormal(); i++) {
    // compute length
    float len = sqrtf(normal_[3*i]*normal_[3*i] +
                      normal_[3*i+1]*normal_[3*i+1] +
                      normal_[3*i+2]*normal_[3*i+2]);

    // divide by length
    if (len > SIM_MINVAL) {
      for (int j=0; j < 3; j++) {
        normal_[3*i+j] /= len;
      }
    } else {
      normal_[3*i] = normal_[3*i+1] = 0;
      normal_[3*i+2] = 1;
    }
  }
}



// compute face circumradii
void sim_builder_mesh_t::MakeCenter() {
  if (center_) {
    return;
  }

  // allocate and clear
  center_ = (double*) sim_malloc(3*nface()*sizeof(double));
  memset(center_, 0, 3*nface()*sizeof(double));

  for (int i=0; i < nface(); i++) {
    // get vertex ids
    int* vertid = face_.data() + 3*i;

    // get triangle edges
    double a[3], b[3];
    for (int j=0; j < 3; j++) {
      a[j] = vert_[3*vertid[0]+j] - vert_[3*vertid[2]+j];
      b[j] = vert_[3*vertid[1]+j] - vert_[3*vertid[2]+j];
    }

    // compute face normal
    double nrm[3];
    sim_math_internal_crossvec(nrm, a, b);

    // compute circumradius
    double norm_a_2 = sim_math_internal_dot3(a, a);
    double norm_b_2 = sim_math_internal_dot3(b, b);
    double area = sqrt(sim_math_internal_dot3(nrm, nrm));

    // compute circumcenter
    double res[3], vec[3] = {
      norm_a_2 * b[0] - norm_b_2 * a[0],
      norm_a_2 * b[1] - norm_b_2 * a[1],
      norm_a_2 * b[2] - norm_b_2 * a[2]
    };
    sim_math_internal_crossvec(res, vec, nrm);
    center_[3*i+0] = res[0]/(2*area*area) + vert_[3*vertid[2]+0];
    center_[3*i+1] = res[1]/(2*area*area) + vert_[3*vertid[2]+1];
    center_[3*i+2] = res[2]/(2*area*area) + vert_[3*vertid[2]+2];
  }
}



// compute the normals of the polygons
void sim_builder_mesh_t::MakePolygonNormals() {
  for (int i = 0; i < polygons_.size(); ++i) {
    double n[3];
    sim_math_internal_makenormal(n, &vert_[3*polygons_[i][0]], &vert_[3*polygons_[i][1]],
                    &vert_[3*polygons_[i][2]]);
    polygon_normals_[3*i + 0] = n[0];
    polygon_normals_[3*i + 1] = n[1];
    polygon_normals_[3*i + 2] = n[2];
  }
}



// helper class to compute the polygons of a mesh
class MeshPolygon {
 public:
  // constructors (need starting face)
  MeshPolygon(const double v1[3], const double v2[3], const double v3[3],
              int v1i, int v2i, int v3i);
  MeshPolygon() = delete;
  MeshPolygon(const MeshPolygon&) = delete;
  MeshPolygon& operator=(const MeshPolygon&) = delete;

  void InsertFace(int v1, int v2, int v3);          // insert a face into the polygon
  std::vector<std::vector<int>> Paths() const;      // return trace of the polygons
  const double* Normal() const { return normal_; }  // return the normal of the polygon

  // return the ith component of the normal of the polygon
  double Normal(int i) const { return normal_[i]; }

 private:
  std::vector<std::pair<int, int>> edges_;

  // inserted faces do not necessarily share edges with the current polygon, so they're grouped as
  // islands until they can be combined with later face insertions
  std::vector<int> islands_;
  int nisland_ = 0;
  double normal_[3] = {0.0, 0.0, 0.0};
  void CombineIslands(int& island1, int& island2);
};



MeshPolygon::MeshPolygon(const double v1[3], const double v2[3], const double v3[3],
                         int v1i, int v2i, int v3i) {
  sim_math_internal_makenormal(normal_, v1, v2, v3);
  edges_ = {{v1i, v2i}, {v2i, v3i}, {v3i, v1i}};
  nisland_ = 1;
  islands_ = {0, 0, 0};
}



// comparison operator for std::set
bool PolygonCmp(const MeshPolygon& p1, const MeshPolygon& p2)  {
  const double* n1 = p1.Normal();
  const double* n2 = p2.Normal();
  double dot3 = n1[0] * n2[0] + n1[1] * n2[1] + n1[2] * n2[2];

  // TODO(kylebayes): The tolerance should be a parameter set the user, as it should be optimized
  // from mesh to mesh.
  if (dot3 > 0.99999872) {
    return false;
  }

  if (std::abs(n1[0] - n2[0]) > SIM_MINVAL) {
    return n1[0] > n2[0];
  }
  if (std::abs(n1[1] - n2[1]) > SIM_MINVAL) {
    return n1[1] > n2[1];
  }
  if (std::abs(n1[2] - n2[2]) > SIM_MINVAL) {
    return n1[2] > n2[2];
  }
  return false;
}



// combine two islands when a newly inserted face connects them
void MeshPolygon::CombineIslands(int& island1, int& island2) {
  // pick the smaller island
  if (island2 < island1) {
    int tmp = island1;
    island1 = island2;
    island2 = tmp;
  }

  // renumber the islands
  for (int k = 0; k < islands_.size(); ++k) {
    if (islands_[k] == island2) {
      islands_[k] = island1;
    } else if (islands_[k] > island2) {
      islands_[k]--;
    }
  }
}



// insert a triangular face into the polygon
void MeshPolygon::InsertFace(int v1, int v2, int v3) {
  int add1 = 1, add2 = 1, add3 = 1;
  int island = -1;

  // check if face can be attached via edge v1v2
  for (int i = 0; i < edges_.size(); ++i) {
    if (edges_[i].first == v2 && edges_[i].second == v1) {
      add1 = 0;
      island = islands_[i];
      edges_.erase(edges_.begin() + i);
      islands_.erase(islands_.begin() + i);
      break;
    }
  }

  // check if face can be attached via edge v2v3
  for (int i = 0; i < edges_.size(); ++i) {
    if (edges_[i].first == v3 && edges_[i].second == v2) {
      int island2 = islands_[i];
      if (island == -1) {
        island = island2;
      } else if (island2 != island) {
        nisland_--;
        CombineIslands(island, island2);
      }
      add2 = 0;
      edges_.erase(edges_.begin() + i);
      islands_.erase(islands_.begin() + i);
      break;
    }
  }

  // check if face can be attached via edge v3v1
  for (int i = 0; i < edges_.size(); ++i) {
    if (edges_[i].first == v1 && edges_[i].second == v3) {
      int island3 = islands_[i];
      if (island == -1) {
        island = island3;
      } else if (island3 != island) {
        nisland_--;
        CombineIslands(island, island3);
      }
      add3 = 0;
      edges_.erase(edges_.begin() + i);
      islands_.erase(islands_.begin() + i);
      break;
    }
  }

  if (island == -1) {
    island = nisland_++;
  }

  // add only new edges to the polygon

  if (add1) {
    edges_.push_back({v1, v2});
    islands_.push_back(island);
  }
  if (add2) {
    edges_.push_back({v2, v3});
    islands_.push_back(island);
  }
  if (add3) {
    edges_.push_back({v3, v1});
    islands_.push_back(island);
  }
}



// return the transverse vertices of the polygon, multiple paths possible if not connected
std::vector<std::vector<int> > MeshPolygon::Paths() const {
  std::vector<std::vector<int> > paths;
  // shortcut if polygon is just a triangular face
  if (edges_.size() == 3) {
    return {{edges_[0].first, edges_[1].first, edges_[2].first}};
  }

  // go through each connected component of the polygon
  for (int i = 0; i < nisland_; ++i) {
    std::vector<int> path;

    // find starting vertex
    for (int j = 0; j < edges_.size(); ++j) {
      if (islands_[j] == i) {
        path.push_back(edges_[j].first);
        path.push_back(edges_[j].second);
        break;
      }
    }

    // SHOULD NOT OCCUR (See logic in MeshPolygon::CombineIslands)
    if (path.empty()) {
      continue;
    }

    // visit the next vertex given the current edge
    int next = path.back();
    for (int l = 0; l < edges_.size(); ++l) {
      int finished = 0;
      for (int k = 1; k < edges_.size(); ++k) {
        if (islands_[k] == i && edges_[k].first == next) {
          next = edges_[k].second;
          if (next == path[0]) {
            paths.push_back(path);
            finished = 1;
            break;
          }
          path.push_back(next);
          break;
        }
      }

      // back at start
      if (finished) {
        break;
      }
    }
  }
  return paths;
}



// merge coplanar mesh triangular faces into polygonal sides to represent the geometry of the mesh
void sim_builder_mesh_t::MakePolygons() {
  std::set<MeshPolygon, decltype(PolygonCmp)*> polygons(PolygonCmp);
  polygons_.clear();
  polygon_normals_.clear();
  polygon_map_.clear();

  // initialize polygon map
  for (int i = 0; i < nvert(); i++) {
    polygon_map_.push_back(std::vector<int>());
  }

  // use graph data if available
  int *faces, nfaces;
  if (graph_) {
    nfaces = graph_[1];
    faces = GraphFaces();
  } else {
    nfaces = nface();
    faces = face_.data();
  }

  // process each face
  for (int i = 0; i < nfaces; i++) {
    double* v1 = &vert_[3*faces[3*i + 0]];
    double* v2 = &vert_[3*faces[3*i + 1]];
    double* v3 = &vert_[3*faces[3*i + 2]];

    MeshPolygon face(v1, v2, v3, faces[3*i + 0], faces[3*i + 1], faces[3*i + 2]);
    auto it = polygons.find(face);
    if (it == polygons.end()) {
      polygons.emplace(v1, v2, v3, faces[3*i + 0], faces[3*i + 1], faces[3*i + 2]);
    } else {
      MeshPolygon& p = const_cast<MeshPolygon&>(*it);
      p.InsertFace(faces[3*i + 0], faces[3*i + 1], faces[3*i + 2]);
    }
  }

  for (const auto& polygon : polygons) {
    std::vector<std::vector<int> > paths = polygon.Paths();

    // separate the polygons if they were grouped together
    for (const auto& path : paths) {
      if (path.size() < 3) continue;
      polygons_.push_back(path);
      polygon_normals_.push_back(polygon.Normal(0));
      polygon_normals_.push_back(polygon.Normal(1));
      polygon_normals_.push_back(polygon.Normal(2));
    }
  }

  // populate the polygon map
  for (int i = 0; i < polygons_.size(); i++) {
    for (int j = 0; j < polygons_[i].size(); ++j) {
      polygon_map_[polygons_[i][j]].push_back(i);
    }
  }
}



//------------------ class SIM_CSkin implementation --------------------------------------------------

// constructor
SIM_CSkin::SIM_CSkin(sim_builder_model_t* _model) {
  sim_spec_defaultSkin(&spec);
  elemtype = SIM_OBJ_SKIN;

  // set model pointer
  model = _model;
  if (model) compiler = &model->spec.compiler;

  // clear data
  spec_file_.clear();
  spec_material_.clear();
  spec_vert_.clear();
  spec_texcoord_.clear();
  spec_face_.clear();
  spec_bodyname_.clear();
  spec_bindpos_.clear();
  spec_bindquat_.clear();
  spec_vertid_.clear();
  spec_vertweight_.clear();

  bodyid.clear();
  matid = -1;

  // point to local
  PointToLocal();

  // in case this camera is not compiled
  CopyFromSpec();
}



SIM_CSkin::SIM_CSkin(const SIM_CSkin& other) {
  *this = other;
}



SIM_CSkin& SIM_CSkin::operator=(const SIM_CSkin& other) {
  if (this != &other) {
    this->spec = other.spec;
    *static_cast<SIM_CSkin_*>(this) = static_cast<const SIM_CSkin_&>(other);
    *static_cast<SIM_sSkin*>(this) = static_cast<const SIM_sSkin&>(other);
  }
  PointToLocal();
  return *this;
}



void SIM_CSkin::PointToLocal() {
  spec.element = static_cast<sim_spec_element_t*>(this);
  spec.file = &spec_file_;
  spec.material = &spec_material_;
  spec.vert = &spec_vert_;
  spec.texcoord = &spec_texcoord_;
  spec.face = &spec_face_;
  spec.bodyname = &spec_bodyname_;
  spec.bindpos = &spec_bindpos_;
  spec.bindquat = &spec_bindquat_;
  spec.vertid = &spec_vertid_;
  spec.vertweight = &spec_vertweight_;
  spec.info = &info;
  file = nullptr;
  material = nullptr;
  vert = nullptr;
  texcoord = nullptr;
  face = nullptr;
  bodyname = nullptr;
  bindpos = nullptr;
  bindquat = nullptr;
  vertid = nullptr;
  vertweight = nullptr;
}



void SIM_CSkin::NameSpace(const sim_builder_model_t* m) {
  // use filename if name is missing
  if (name.empty()) {
    std::string stripped = sim_math_internal_strippath(spec_file_);
    name = sim_math_internal_stripext(stripped);
  }
  for (auto& name : spec_bodyname_) {
    name = m->prefix + name + m->suffix;
  }
  if (modelfiledir_.empty()) {
    modelfiledir_ = FilePath(m->spec_modelfiledir_);
  }
}



void SIM_CSkin::CopyFromSpec() {
  *static_cast<SIM_sSkin*>(this) = spec;
  file_ = spec_file_;
  material_ = spec_material_;
  vert_ = spec_vert_;
  texcoord_ = spec_texcoord_;
  face_ = spec_face_;
  bodyname_ = spec_bodyname_;
  bindpos_ = spec_bindpos_;
  bindquat_ = spec_bindquat_;
  vertid_ = spec_vertid_;
  vertweight_ = spec_vertweight_;

  // use filename if name is missing
  if (name.empty()) {
    std::string stripped = sim_math_internal_strippath(file_);
    name = sim_math_internal_stripext(stripped);
  }
}



// destructor
SIM_CSkin::~SIM_CSkin() {
  spec_file_.clear();
  spec_material_.clear();
  spec_vert_.clear();
  spec_texcoord_.clear();
  spec_face_.clear();
  spec_bodyname_.clear();
  spec_bindpos_.clear();
  spec_bindquat_.clear();
  spec_vertid_.clear();
  spec_vertweight_.clear();
  bodyid.clear();
}



void SIM_CSkin::ResolveReferences(const sim_builder_model_t* m) {
  size_t nbone = bodyname_.size();
  bodyid.resize(nbone);
  for (int i=0; i < nbone; i++) {
    sim_builder_base_t* pbody = m->FindObject(SIM_OBJ_BODY, bodyname_[i]);
    if (!pbody) {
      throw sim_builder_error_t(this, "unknown body '%s' in skin", bodyname_[i].c_str());
    }
    bodyid[i] = pbody->id;
  }
}



// compiler
void SIM_CSkin::Compile(const SIM_VFS* vfs) {
  CopyFromSpec();

  // load file
  if (!file_.empty()) {
    // make sure data is not present
    if (!spec_vert_.empty() ||
        !spec_texcoord_.empty() ||
        !spec_face_.empty() ||
        !spec_bodyname_.empty() ||
        !spec_bindpos_.empty() ||
        !spec_bindquat_.empty() ||
        !spec_vertid_.empty() ||
        !spec_vertweight_.empty()) {
      throw sim_builder_error_t(this, "Both skin data and file were specified: %s", file_.c_str());
    }

    // remove path from file if necessary
    if (model->strippath) {
      file_ = sim_math_internal_strippath(file_);
    }

    // load SKN
    std::string ext = sim_math_internal_getext(file_);
    if (strcasecmp(ext.c_str(), ".skn")) {
      throw sim_builder_error_t(this, "Unknown skin file type: %s", file_.c_str());
    }

    // copy paths from model if not already defined
    if (modelfiledir_.empty()) {
      modelfiledir_ = FilePath(model->modelfiledir_);
    }
    simcore::user::FilePath meshdir_;
    meshdir_ = FilePath(sim_spec_getString(compiler->meshdir));

    FilePath filename = meshdir_ + FilePath(file_);
    SIM_Resource* resource = LoadResource(modelfiledir_.Str(), filename.Str(), vfs);

    try {
      LoadSKN(resource);
      sim_math_closeResource(resource);
    } catch(sim_builder_error_t err) {
      sim_math_closeResource(resource);
      throw err;
    }
  }

  // make sure all data is present
  if (vert_.empty() ||
      face_.empty() ||
      bodyname_.empty() ||
      bindpos_.empty() ||
      bindquat_.empty() ||
      vertid_.empty() ||
      vertweight_.empty()) {
    throw sim_builder_error_t(this, "Missing data in skin");
  }

  // check mesh sizes
  if (vert_.size()%3) {
    throw sim_builder_error_t(this, "Vertex data must be multiple of 3");
  }
  if (!texcoord_.empty() && texcoord_.size() != 2*vert_.size()/3) {
    throw sim_builder_error_t(this, "Vertex and texcoord data incompatible size");
  }
  if (face_.size()%3) {
    throw sim_builder_error_t(this, "Face data must be multiple of 3");
  }

  // check bone sizes
  size_t nbone = bodyname_.size();
  if (bindpos_.size() != 3*nbone) {
    throw sim_builder_error_t(this, "Unexpected bindpos size in skin");
  }
  if (bindquat_.size() != 4*nbone) {
    throw sim_builder_error_t(this, "Unexpected bindquat size in skin");
  }
  if (vertid_.size() != nbone) {
    throw sim_builder_error_t(this, "Unexpected vertid size in skin");
  }
  if (vertweight_.size() != nbone) {
    throw sim_builder_error_t(this, "Unexpected vertweight size in skin");
  }

  // resolve body names
  ResolveReferences(model);

  // resolve material name
  sim_builder_base_t* pmat = model->FindObject(SIM_OBJ_MATERIAL, material_);
  if (pmat) {
    matid = pmat->id;
  } else if (!material_.empty()) {
    throw sim_builder_error_t(this, "unknown material '%s' in skin", material_.c_str());
  }

  // set total vertex weights to 0
  std::vector<float> vw;
  size_t nvert = vert_.size()/3;
  vw.resize(nvert);
  fill(vw.begin(), vw.end(), 0.0f);

  // accumulate vertex weights from all bones
  for (int i=0; i < nbone; i++) {
    // make sure bone has vertices and sizes match
    size_t nbv = vertid_[i].size();
    if (vertweight_[i].size() != nbv || nbv == 0) {
      throw sim_builder_error_t(this, "vertid and vertweight must have same non-zero size in skin");
    }

    // accumulate weights in global array
    for (int j=0; j < nbv; j++) {
      // get index and check range
      int jj = vertid_[i][j];
      if (jj < 0 || jj >= nvert) {
        throw sim_builder_error_t(this, "vertid %d out of range in skin", nullptr, jj);
      }

      // accumulate
      vw[jj] += vertweight_[i][j];
    }
  }

  // check coverage
  for (int i=0; i < nvert; i++) {
    if (vw[i] <= SIM_MINVAL) {
      throw sim_builder_error_t(this, "vertex %d must have positive total weight in skin", nullptr, i);
    }
  }

  // normalize vertex weights
  for (int i=0; i < nbone; i++) {
    for (int j=0; j < vertid_[i].size(); j++) {
      vertweight_[i][j] /= vw[vertid_[i][j]];
    }
  }

  // normalize bindquat
  for (int i=0; i < nbone; i++) {
    double quat[4] = {
      (double)bindquat_[4*i],
      (double)bindquat_[4*i+1],
      (double)bindquat_[4*i+2],
      (double)bindquat_[4*i+3]
    };
    sim_math_internal_normvec(quat, 4);

    bindquat_[4*i]   = (float) quat[0];
    bindquat_[4*i+1] = (float) quat[1];
    bindquat_[4*i+2] = (float) quat[2];
    bindquat_[4*i+3] = (float) quat[3];
  }
}



// load skin in SKN BIN format
void SIM_CSkin::LoadSKN(SIM_Resource* resource) {
  char* buffer = 0;
  int buffer_sz = sim_math_readResource(resource, (const void**)&buffer);

  if (buffer_sz < 0) {
    throw sim_builder_error_t(this, "could not read SKN file '%s'", resource->name);
  } else if (!buffer_sz) {
    throw sim_builder_error_t(this, "SKN file '%s' is empty", resource->name);
  }

  // make sure header is present
  if (buffer_sz < 16) {
    throw sim_builder_error_t(this, "missing header in SKN file '%s'", resource->name);
  }

  // get sizes from header
  int nvert = ((int*)buffer)[0];
  int ntexcoord = ((int*)buffer)[1];
  int nface = ((int*)buffer)[2];
  int nbone = ((int*)buffer)[3];

  // negative sizes not allowed
  if (nvert < 0 || ntexcoord < 0 || nface < 0 || nbone < 0) {
    throw sim_builder_error_t(this, "negative size in header of SKN file '%s'", resource->name);
  }

  // make sure we have data for vert, texcoord, face
  if (buffer_sz < 16 + 12*nvert + 8*ntexcoord + 12*nface) {
    throw sim_builder_error_t(this, "insufficient data in SKN file '%s'", resource->name);
  }

  // data pointer and counter
  float* pdata = (float*)(buffer+16);
  int cnt = 0;

  // copy vert
  if (nvert) {
    vert_.resize(3*nvert);
    memcpy(vert_.data(), pdata+cnt, 3*nvert*sizeof(float));
    cnt += 3*nvert;
  }

  // copy texcoord
  if (ntexcoord) {
    texcoord_.resize(2*ntexcoord);
    memcpy(texcoord_.data(), pdata+cnt, 2*ntexcoord*sizeof(float));
    cnt += 2*ntexcoord;
  }

  // copy face
  if (nface) {
    face_.resize(3*nface);
    memcpy(face_.data(), pdata+cnt, 3*nface*sizeof(int));
    cnt += 3*nface;
  }

  // allocate bone arrays
  bodyname_.clear();
  bindpos_.resize(3*nbone);
  bindquat_.resize(4*nbone);
  vertid_.resize(nbone);
  vertweight_.resize(nbone);

  // read bones
  for (int i=0; i < nbone; i++) {
    // check size
    if (buffer_sz/4-4-cnt < 18) {
      throw sim_builder_error_t(this, "insufficient data in SKN file '%s', bone %d", resource->name, i);
    }

    // read name
    char txt[40];
    strncpy(txt, (char*)(pdata+cnt), 39);
    txt[39] = '\0';
    cnt += 10;
    bodyname_.push_back(txt);

    // read bindpos
    memcpy(bindpos_.data()+3*i, pdata+cnt, 3*sizeof(float));
    cnt += 3;

    // read bind quat
    memcpy(bindquat_.data()+4*i, pdata+cnt, 4*sizeof(float));
    cnt += 4;

    // read vertex count
    int vcount = *(int*)(pdata+cnt);
    cnt += 1;

    // check for negative
    if (vcount < 1) {
      throw sim_builder_error_t(this, "vertex count must be positive in SKN file '%s', bone %d",
                     resource->name, i);
    }

    // check size
    if (buffer_sz/4-4-cnt < 2*vcount) {
      throw sim_builder_error_t(this, "insufficient vertex data in SKN file '%s', bone %d",
                     resource->name, i);
    }

    // read vertid
    vertid_[i].resize(vcount);
    memcpy(vertid_[i].data(), (int*)(pdata+cnt), vcount*sizeof(int));
    cnt += vcount;

    // read vertweight
    vertweight_[i].resize(vcount);
    memcpy(vertweight_[i].data(), (int*)(pdata+cnt), vcount*sizeof(int));
    cnt += vcount;
  }

  // check final size
  if (buffer_sz != 16+4*cnt) {
    throw sim_builder_error_t(this, "unexpected buffer size in SKN file '%s'", resource->name);
  }
}



//-------------------------- nonlinear elasticity --------------------------------------------------

// hash function for std::pair
struct PairHash
{
  template <class T1, class T2>
  std::size_t operator() (const std::pair<T1, T2>& pair) const {
    return std::hash<T1>()(pair.first) ^ std::hash<T2>()(pair.second);
  }
};

// simplex connectivity
constexpr int eledge[3][6][2] = {{{ 0,  1}, {-1, -1}, {-1, -1},
                                  {-1, -1}, {-1, -1}, {-1, -1}},
                                 {{ 1,  2}, { 2,  0}, { 0,  1},
                                  {-1, -1}, {-1, -1}, {-1, -1}},
                                 {{ 0,  1}, { 1,  2}, { 2,  0},
                                  { 2,  3}, { 0,  3}, { 1,  3}}};

struct Stencil2D {
  static constexpr int kNumEdges = 3;
  static constexpr int kNumVerts = 3;
  static constexpr int kNumFaces = 2;
  static constexpr int edge[kNumEdges][2] = {{1, 2}, {2, 0}, {0, 1}};
  static constexpr int face[kNumVerts][2] = {{1, 2}, {2, 0}, {0, 1}};
  static constexpr int edge2face[kNumEdges][2] = {{1, 2}, {2, 0}, {0, 1}};
  int vertices[kNumVerts];
  int edges[kNumEdges];
};

struct Stencil3D {
  static constexpr int kNumEdges = 6;
  static constexpr int kNumVerts = 4;
  static constexpr int kNumFaces = 3;
  static constexpr int edge[kNumEdges][2] = {{0, 1}, {1, 2}, {2, 0},
                                             {2, 3}, {0, 3}, {1, 3}};
  static constexpr int face[kNumVerts][3] = {{2, 1, 0}, {0, 1, 3},
                                             {1, 2, 3}, {2, 0, 3}};
  static constexpr int edge2face[kNumEdges][2] = {{2, 3}, {1, 3}, {2, 1},
                                                  {1, 0}, {0, 2}, {0, 3}};
  int vertices[kNumVerts];
  int edges[kNumEdges];
};

template <typename T>
inline double ComputeVolume(const double* x, const int v[T::kNumVerts]);

template <>
inline double ComputeVolume<Stencil2D>(const double* x,
                                       const int v[Stencil2D::kNumVerts]) {
  double normal[3];
  const double* x0 = x + 3*v[0];
  const double* x1 = x + 3*v[1];
  const double* x2 = x + 3*v[2];
  double edge1[3] = {x1[0]-x0[0], x1[1]-x0[1], x1[2]-x0[2]};
  double edge2[3] = {x2[0]-x0[0], x2[1]-x0[1], x2[2]-x0[2]};
  sim_math_internal_crossvec(normal, edge1, edge2);
  return sim_math_internal_normvec(normal, 3) / 2;
}

template<>
inline double ComputeVolume<Stencil3D>(const double* x,
                                       const int v[Stencil3D::kNumVerts]) {
  double normal[3];
  const double* x0 = x + 3*v[0];
  const double* x1 = x + 3*v[1];
  const double* x2 = x + 3*v[2];
  const double* x3 = x + 3*v[3];
  double edge1[3] = {x1[0]-x0[0], x1[1]-x0[1], x1[2]-x0[2]};
  double edge2[3] = {x2[0]-x0[0], x2[1]-x0[1], x2[2]-x0[2]};
  double edge3[3] = {x3[0]-x0[0], x3[1]-x0[1], x3[2]-x0[2]};
  sim_math_internal_crossvec(normal, edge1, edge2);
  return sim_math_internal_dot3(normal, edge3) / 6;
}

// compute metric tensor of edge lengths inner product
template <typename T>
void inline MetricTensor(double* metric, int idx, double mu,
                         double la, const double basis[T::kNumEdges][9]) {
  double trE[T::kNumEdges] = {0};
  double trEE[T::kNumEdges*T::kNumEdges] = {0};
  double k[T::kNumEdges*T::kNumEdges];

  // compute first invariant i.e. trace(strain)
  for (int e = 0; e < T::kNumEdges; e++) {
    for (int i = 0; i < 3; i++) {
      trE[e] += basis[e][4*i];
    }
  }

  // compute second invariant i.e. trace(strain^2)
  for (int ed1 = 0; ed1 < T::kNumEdges; ed1++) {
    for (int ed2 = 0; ed2 < T::kNumEdges; ed2++) {
      for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
          trEE[T::kNumEdges*ed1+ed2] += basis[ed1][3*i+j] * basis[ed2][3*j+i];
        }
      }
    }
  }

  // assembly of strain metric tensor
  for (int ed1 = 0; ed1 < T::kNumEdges; ed1++) {
    for (int ed2 = 0; ed2 < T::kNumEdges; ed2++) {
      k[T::kNumEdges*ed1 + ed2] = mu * trEE[T::kNumEdges * ed1 + ed2] +
                                  la * trE[ed2] * trE[ed1];
    }
  }

  // copy to triangular representation
  int id = 0;
  for (int ed1 = 0; ed1 < T::kNumEdges; ed1++) {
    for (int ed2 = ed1; ed2 < T::kNumEdges; ed2++) {
      metric[21*idx + id++] = k[T::kNumEdges*ed1 + ed2];
    }
  }

  if (id != T::kNumEdges*(T::kNumEdges+1)/2) {
    sim_error("incorrect stiffness matrix size");
  }
}

// compute local basis
template <typename T>
void inline ComputeBasis(double basis[9], const double* x,
                         const int v[T::kNumVerts],
                         const int faceL[T::kNumFaces],
                         const int faceR[T::kNumFaces], double volume);

template <>
void inline ComputeBasis<Stencil2D>(double basis[9], const double* x,
                                    const int v[Stencil2D::kNumVerts],
                                    const int faceL[Stencil2D::kNumFaces],
                                    const int faceR[Stencil2D::kNumFaces],
                                    double volume) {
  double basisL[3], basisR[3];
  double normal[3];

  const double* xL0 = x + 3*v[faceL[0]];
  const double* xL1 = x + 3*v[faceL[1]];
  const double* xR0 = x + 3*v[faceR[0]];
  const double* xR1 = x + 3*v[faceR[1]];
  double edgesL[3] = {xL0[0]-xL1[0], xL0[1]-xL1[1], xL0[2]-xL1[2]};
  double edgesR[3] = {xR1[0]-xR0[0], xR1[1]-xR0[1], xR1[2]-xR0[2]};

  sim_math_internal_crossvec(normal, edgesR, edgesL);
  sim_math_internal_normvec(normal, 3);
  sim_math_internal_crossvec(basisL, normal, edgesL);
  sim_math_internal_crossvec(basisR, edgesR, normal);

  // we use as basis the symmetrized tensor products of the edge normals of the
  // other two edges; this is shown in Weischedel "A discrete geometric view on
  // shear-deformable shell models" in the remark at the end of section 4.1;
  // equivalent to linear finite elements but in a coordinate-free formulation.

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      basis[3*i+j] = (basisL[i]*basisR[j] +
                      basisR[i]*basisL[j]) / (8*volume*volume);
    }
  }
}

// compute local basis
template <>
void inline ComputeBasis<Stencil3D>(double basis[9], const double* x,
                                    const int v[Stencil3D::kNumVerts],
                                    const int faceL[Stencil3D::kNumFaces],
                                    const int faceR[Stencil3D::kNumFaces],
                                    double volume) {
  const double* xL0 = x + 3*v[faceL[0]];
  const double* xL1 = x + 3*v[faceL[1]];
  const double* xL2 = x + 3*v[faceL[2]];
  const double* xR0 = x + 3*v[faceR[0]];
  const double* xR1 = x + 3*v[faceR[1]];
  const double* xR2 = x + 3*v[faceR[2]];
  double edgesL[6] = {xL1[0] - xL0[0], xL1[1] - xL0[1], xL1[2] - xL0[2],
                      xL2[0] - xL0[0], xL2[1] - xL0[1], xL2[2] - xL0[2]};
  double edgesR[6] = {xR1[0] - xR0[0], xR1[1] - xR0[1], xR1[2] - xR0[2],
                      xR2[0] - xR0[0], xR2[1] - xR0[1], xR2[2] - xR0[2]};

  double normalL[3], normalR[3];
  sim_math_internal_crossvec(normalL, edgesL, edgesL+3);
  sim_math_internal_crossvec(normalR, edgesR, edgesR+3);

  // we use as basis the symmetrized tensor products of the area normals of the
  // two faces not adjacent to the edge; this is the 3D equivalent to the basis
  // proposed in Weischedel "A discrete geometric view on shear-deformable shell
  // models" in the remark at the end of section 4.1. This is also equivalent to
  // linear finite elements but in a coordinate-free formulation.

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      basis[3*i+j] = (normalL[i]*normalR[j] +
                      normalR[i]*normalL[j]) / (36*2*volume*volume);
    }
  }
}

// compute stiffness for a single element
template <typename T>
void inline ComputeStiffness(std::vector<double>& stiffness,
                             const std::vector<double>& body_pos,
                             const int* v, int t, double E,
                             double nu, double thickness = 4) {
  // triangles area
  double volume = ComputeVolume<T>(body_pos.data(), v);

  // material parameters
  double mu = E / (2*(1+nu)) * std::abs(volume) / 4 * thickness;
  double la = E*nu / ((1+nu)*(1-2*nu)) * std::abs(volume) / 4 * thickness;

  // local geometric quantities
  double basis[T::kNumEdges][9] = {{0}};

  // compute edge basis
  for (int e = 0; e < T::kNumEdges; e++) {
    ComputeBasis<T>(basis[e], body_pos.data(), v,
                    T::face[T::edge2face[e][0]],
                    T::face[T::edge2face[e][1]], volume);
  }

  // compute metric tensor
  MetricTensor<T>(stiffness.data(), t, mu, la, basis);
}

// local tetrahedron numbering
constexpr int kNumEdges = Stencil2D::kNumEdges;
constexpr int kNumVerts = Stencil2D::kNumVerts;
constexpr int edge[kNumEdges][2] = {{1, 2}, {2, 0}, {0, 1}};

// create map from triangles to vertices and edges and from edges to vertices
static void CreateFlapStencil(std::vector<StencilFlap>& flaps,
                              const std::vector<int>& simplex,
                              const std::vector<int>& edgeidx) {
  // populate stencil
  int ne = 0;
  int nt = simplex.size() / kNumVerts;
  std::vector<Stencil2D> elements(nt);
  for (int t = 0; t < nt; t++) {
    for (int v = 0; v < kNumVerts; v++) {
      elements[t].vertices[v] = simplex[kNumVerts * t + v];
    }
  }

  // map from edge vertices to their index in `edges` vector
  std::unordered_map<std::pair<int, int>, int, PairHash> edge_indices;

  // loop over all triangles
  for (int t = 0; t < nt; t++) {
    int* v = elements[t].vertices;

    // compute edges to vertices map for fast computations
    for (int e = 0; e < kNumEdges; e++) {
      auto pair = std::pair(std::min(v[edge[e][0]], v[edge[e][1]]),
                            std::max(v[edge[e][0]], v[edge[e][1]]));

      // if edge is already present in the vector only store its index
      auto [it, inserted] = edge_indices.insert({pair, ne});

      if (inserted) {
        StencilFlap flap;
        flap.vertices[0] = v[edge[e][0]];
        flap.vertices[1] = v[edge[e][1]];
        flap.vertices[2] = v[(edge[e][1] + 1) % 3];
        flap.vertices[3] = -1;
        flaps.push_back(flap);
        elements[t].edges[e] = ne++;
      } else {
        elements[t].edges[e] = it->second;
        flaps[it->second].vertices[3] = v[(edge[e][1] + 1) % 3];
      }

      // double check that the edge indices are consistent
      if (!edgeidx.empty()) {
        if (elements[t].edges[e] != edgeidx[kNumEdges * t + e]) {
          sim_error("edge indices do not match in CreateFlapStencil");
        }
      }
    }
  }
}

// cotangent between two edges
double inline cot(const double* x, int v0, int v1, int v2) {
  double normal[3];
  double edge1[3] = {x[3*v1]-x[3*v0], x[3*v1+1]-x[3*v0+1], x[3*v1+2]-x[3*v0+2]};
  double edge2[3] = {x[3*v2]-x[3*v0], x[3*v2+1]-x[3*v0+1], x[3*v2+2]-x[3*v0+2]};

  sim_math_internal_crossvec(normal, edge1, edge2);
  return sim_math_internal_dot3(edge1, edge2) / sqrt(sim_math_internal_dot3(normal, normal));
}

// area of a triangle
double inline ComputeVolume(const double* x, const int v[Stencil2D::kNumVerts]) {
  double normal[3];
  double edge1[3] = {x[3*v[1]]-x[3*v[0]], x[3*v[1]+1]-x[3*v[0]+1], x[3*v[1]+2]-x[3*v[0]+2]};
  double edge2[3] = {x[3*v[2]]-x[3*v[0]], x[3*v[2]+1]-x[3*v[0]+1], x[3*v[2]+2]-x[3*v[0]+2]};

  sim_math_internal_crossvec(normal, edge1, edge2);
  return sqrt(sim_math_internal_dot3(normal, normal)) / 2;
}

// compute bending stiffness for a single edge
template <typename T>
void inline ComputeBending(double* bending, double* pos, const int v[4], double mu,
                           double thickness) {
  int vadj[3] = {v[1], v[0], v[3]};

  if (v[3]== -1) {
    // skip boundary edges
    return;
  }

  // cotangent operator from Wardetzky at al., "Discrete Quadratic Curvature
  // Energies", https://cims.nyu.edu/gcl/papers/wardetzky2007dqb.pdf

  double a01 = cot(pos, v[0], v[1], v[2]);
  double a02 = cot(pos, v[0], v[3], v[1]);
  double a03 = cot(pos, v[1], v[2], v[0]);
  double a04 = cot(pos, v[1], v[0], v[3]);
  double c[4] = {a03 + a04, a01 + a02, -(a01 + a03), -(a02 + a04)};
  double volume = ComputeVolume(pos, v) + ComputeVolume(pos, vadj);
  double stiffness = 3 * mu * pow(thickness, 3) / (24 * volume);

  // Garg et al., "Cubic Shells", https://cims.nyu.edu/gcl/papers/garg2007cs.pdf
  const double* v0 = pos + 3*v[0];
  const double* v1 = pos + 3*v[1];
  const double* v2 = pos + 3*v[2];
  const double* v3 = pos + 3*v[3];
  double e0[3] = {v1[0] - v0[0], v1[1] - v0[1], v1[2] - v0[2]};
  double e1[3] = {v2[0] - v0[0], v2[1] - v0[1], v2[2] - v0[2]};
  double e2[3] = {v3[0] - v0[0], v3[1] - v0[1], v3[2] - v0[2]};
  double e3[3] = {v2[0] - v1[0], v2[1] - v1[1], v2[2] - v1[2]};
  double e4[3] = {v3[0] - v1[0], v3[1] - v1[1], v3[2] - v1[2]};
  double t0[3] = {-(a03*e1[0] + a01*e3[0]), -(a03*e1[1] + a01*e3[1]), -(a03*e1[2] + a01*e3[2])};
  double t1[3] = {-(a04*e2[0] + a02*e4[0]), -(a04*e2[1] + a02*e4[1]), -(a04*e2[2] + a02*e4[2])};
  double sqr = sim_math_internal_dot3(e0, e0);
  double cos_theta = -sim_math_internal_dot3(t0, t1) / sqr;

  for (int v1 = 0; v1 < T::kNumVerts; v1++) {
    for (int v2 = 0; v2 < T::kNumVerts; v2++) {
      bending[4 * v1 + v2] += c[v1] * c[v2] * cos_theta * stiffness;
    }
  }

  double n[3];
  sim_math_internal_crossvec(n, e0, e1);
  bending[16] = sim_math_internal_dot3(n, e2) * (a01 - a03) * (a04 - a02) * stiffness / (sqr * sqrt(sqr));
}

//----------------------------- linear elasticity --------------------------------------------------

// Gauss Legendre quadrature points in 1 dimension on the interval [a, b]
void quadratureGaussLegendre(double* points, double* weights,
                             const int order, const double a, const double b) {
  if (order > 3)
    sim_error("Integration order > 3 not yet supported.");

  // x is on [-1, 1], p on [a, b]
  double p0 = (a+b)/2.;
  double dpdx = (b-a)/2;

  if (order == 2) {
    points[0] = -dpdx / sqrt(3) + p0;
    points[1] =  dpdx / sqrt(3) + p0;
    weights[0] = dpdx;
    weights[1] = dpdx;
  } else {
    points[0] = p0;
    points[1] = -dpdx / sqrt(3. / 5.) + p0;
    points[2] =  dpdx / sqrt(3. / 5.) + p0;
    weights[0] = 8. / 9. * dpdx;
    weights[1] = 5. / 9. * dpdx;
    weights[2] = 5. / 9. * dpdx;
  }
}

// evaluate 1-dimensional basis function
double phi(const double s, const int i, const int order) {
  if (order == 1) {
    return i == 0 ? 1 - s : s;
  } else if (order == 2) {
    switch (i) {
      case 0:
        return 2 * s * s - 3 * s + 1;
      case 1:
        return 4 * (s - s * s);
      case 2:
        return 2 * s * s - s;
      default:
        SIM_ERROR("invalid index %d", i);
        return 0;
    }
  } else {
    sim_error("Order must be 1 or 2.");
    return 0;
  }
}

// evaluate gradient of 1-dimensional basis function
double dphi(const double s, const int i, const int order) {
  if (order == 1) {
    return i == 0 ? -1 : 1;
  } else if (order == 2) {
    switch (i) {
      case 0:
        return 4 * s - 3;
      case 1:
        return 4 * (1 - 2 * s);
      case 2:
        return 4 * s - 1;
      default:
        SIM_ERROR("invalid index %d, must be 0, 1, or 2", i);
        return 0;
    }
  } else {
    sim_error("Order must be 1 or 2.");
    return 0;
  }
}

typedef std::array<std::array<double, 3>, 3> Matrix;

// symmetrize a tensor
Matrix inline sym(const Matrix& tensor) {
  Matrix eps;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      eps[i][j] = (tensor[i][j] + tensor[j][i]) / 2;
    }
  }
  return eps;
}

// compute tensor inner product
Matrix inline inner(const Matrix& tensor1, const Matrix& tensor2) {
  Matrix inner;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      inner[i][j] = tensor1[i][0] * tensor2[0][j] +
                    tensor1[i][1] * tensor2[1][j] +
                    tensor1[i][2] * tensor2[2][j];
    }
  }
  return inner;
}

// compute trace of a tensor
double inline trace(const Matrix& tensor) {
  return tensor[0][0] + tensor[1][1] + tensor[2][2];
}

void inline ComputeLinearStiffness(std::vector<double>& K,
                                   const double* pos,
                                   double E, double nu, int order) {
  int nbasis = order + 1;
  int n = pow(nbasis, 3);
  int ndof = 3*n;

  // compute quadrature points
  std::vector<double> points(nbasis);     // quadrature points
  std::vector<double> weight(nbasis);     // quadrature weights
  quadratureGaussLegendre(points.data(), weight.data(), nbasis, 0, 1);

  // compute element transformation
  double dx = (pos+3*(n-1))[0] - pos[0];
  double dy = (pos+3*(n-1))[1] - pos[1];
  double dz = (pos+3*(n-1))[2] - pos[2];
  double detJ = dx * dy * dz;
  double invJ[3] = {1.0 / dx, 1.0 / dy, 1.0 / dz};

  // compute stiffness matrix
  std::vector<std::array<double, 3> > F(n);
  double la = E * nu / (1 + nu) / (1 - 2 * nu);
  double mu = E / (2 * (1 + nu));

  // loop over quadrature points
  for (int ps=0; ps < nbasis; ps++) {
    for (int pt=0; pt < nbasis; pt++) {
      for (int pu=0; pu < nbasis; pu++) {
        double s = points[ps];
        double t = points[pt];
        double u = points[pu];
        double dvol = weight[ps] * weight[pt] * weight[pu] * detJ;
        int dof = 0;

        // cartesian product of basis functions
        for (int bx=0; bx < nbasis; bx++) {
          for (int by=0; by < nbasis; by++) {
            for (int bz=0; bz < nbasis; bz++) {
              std::array<double, 3> gradient;
              gradient[0] = dphi(s, bx, order) *  phi(t, by, order) *  phi(u, bz, order);
              gradient[1] =  phi(s, bx, order) * dphi(t, by, order) *  phi(u, bz, order);
              gradient[2] =  phi(s, bx, order) *  phi(t, by, order) * dphi(u, bz, order);
              F[dof++] = gradient;
            }
          }
        }

        if (dof != n) {  // SHOULD NOT OCCUR
          throw sim_builder_error_t(nullptr, "incorrect number of basis functions");
        }

        // tensor contraction of the gradients of elastic strains
        // (d(F+F')/dx : d(F+F')/dx)
        for (int i=0; i < n; i++) {
          for (int j=0; j < n; j++) {
            Matrix du;
            Matrix dv;
            du.fill({0, 0, 0});
            dv.fill({0, 0, 0});
            for (int k=0; k < 3; k++) {
              for (int l=0; l < 3; l++) {
                du[k][0] = invJ[0] * F[i][0];
                du[k][1] = invJ[1] * F[i][1];
                du[k][2] = invJ[2] * F[i][2];
                dv[l][0] = invJ[0] * F[j][0];
                dv[l][1] = invJ[1] * F[j][1];
                dv[l][2] = invJ[2] * F[j][2];
                K[ndof*(3*i+k) + 3*j+l] -= la * trace(du) * trace(dv) * dvol;
                K[ndof*(3*i+k) + 3*j+l] -= mu * trace(inner(sym(du), sym(dv))) * dvol;
                sim_math_internal_zerovec(du[k].data(), 3);
                sim_math_internal_zerovec(dv[l].data(), 3);
              }
            }
          }
        }
      }
    }
  }
}

//------------------ class SIM_CFlex implementation --------------------------------------------------

// constructor
SIM_CFlex::SIM_CFlex(sim_builder_model_t* _model) {
  sim_spec_defaultFlex(&spec);
  elemtype = SIM_OBJ_FLEX;

  // set model
  model = _model;
  if (_model) compiler = &_model->spec.compiler;

  // clear internal variables
  nvert = 0;
  nnode = 0;
  nedge = 0;
  nelem = 0;
  matid = -1;
  rigid = false;
  centered = false;

  PointToLocal();
  CopyFromSpec();
}


SIM_CFlex::SIM_CFlex(const SIM_CFlex& other) {
  *this = other;
}


SIM_CFlex& SIM_CFlex::operator=(const SIM_CFlex& other) {
  if (this != &other) {
    this->spec = other.spec;
    *static_cast<SIM_CFlex_*>(this) = static_cast<const SIM_CFlex_&>(other);
    *static_cast<SIM_sFlex*>(this) = static_cast<const SIM_sFlex&>(other);
  }
  PointToLocal();
  return *this;
}


void SIM_CFlex::PointToLocal() {
  spec.element = static_cast<sim_spec_element_t*>(this);
  spec.material = &spec_material_;
  spec.vertbody = &spec_vertbody_;
  spec.nodebody = &spec_nodebody_;
  spec.vert = &spec_vert_;
  spec.node = &spec_node_;
  spec.texcoord = &spec_texcoord_;
  spec.elemtexcoord = &spec_elemtexcoord_;
  spec.elem = &spec_elem_;
  spec.info = &info;
  material = nullptr;
  vertbody = nullptr;
  nodebody = nullptr;
  vert = nullptr;
  node = nullptr;
  texcoord = nullptr;
  elemtexcoord = nullptr;
  elem = nullptr;
}



void SIM_CFlex::NameSpace(const sim_builder_model_t* m) {
  for (auto& name : spec_vertbody_) {
    name = m->prefix + name + m->suffix;
  }
  for (auto& name : spec_nodebody_) {
    name = m->prefix + name + m->suffix;
  }
  if (!spec_material_.empty() && model != m) {
    spec_material_ = m->prefix + spec_material_ + m->suffix;
  }
}



void SIM_CFlex::CopyFromSpec() {
  *static_cast<SIM_sFlex*>(this) = spec;
  spec.info = &info;
  material_ = spec_material_;
  vertbody_ = spec_vertbody_;
  nodebody_ = spec_nodebody_;
  vert_ = spec_vert_;
  node_ = spec_node_;
  texcoord_ = spec_texcoord_;
  elemtexcoord_ = spec_elemtexcoord_;
  elem_ = spec_elem_;

  // clear precompiled asset. TODO: use asset cache
  nedge = 0;
  edge.clear();
  shell.clear();
  evpair.clear();
}


bool SIM_CFlex::HasTexcoord() const {
  return !texcoord_.empty();
}


void SIM_CFlex::DelTexcoord() {
  texcoord_.clear();
}


void SIM_CFlex::ResolveReferences(const sim_builder_model_t* m) {
  vertbodyid.clear();
  nodebodyid.clear();
  for (const auto& vertbody : vertbody_) {
    sim_builder_body_t* pbody = static_cast<sim_builder_body_t*>(m->FindObject(SIM_OBJ_BODY, vertbody));
    if (pbody) {
      vertbodyid.push_back(pbody->id);
      if (pbody->joints.size() != 3 && dim == 2 && (elastic2d == 1 || elastic2d == 3)) {
        // TODO(quaglino): add support for pins
        throw sim_builder_error_t(this, "pins are not supported for bending");
      }
    } else {
      throw sim_builder_error_t(this, "unknown body '%s' in flex", vertbody.c_str());
    }
  }
  for (const auto& nodebody : nodebody_) {
    sim_builder_base_t* pbody = m->FindObject(SIM_OBJ_BODY, nodebody);
    if (pbody) {
      nodebodyid.push_back(pbody->id);
    } else {
      throw sim_builder_error_t(this, "unknown body '%s' in flex", nodebody.c_str());
    }
  }
}


// compiler
void SIM_CFlex::Compile(const SIM_VFS* vfs) {
  CopyFromSpec();
  interpolated = !nodebody_.empty();

  // set nelem; check sizes
  if (dim < 1 || dim > 3) {
    throw sim_builder_error_t(this, "dim must be 1, 2 or 3");
  }
  if (elem_.empty()) {
    throw sim_builder_error_t(this, "elem is empty");
  }
  if (elem_.size() % (dim+1)) {
    throw sim_builder_error_t(this, "elem size must be multiple of (dim+1)");
  }
  if (vertbody_.empty() && !interpolated) {
    throw sim_builder_error_t(this, "vertbody and nodebody are both empty");
  }
  if (vert_.size() % 3) {
    throw sim_builder_error_t(this, "vert size must be a multiple of 3");
  }
  if (edgestiffness > 0 && dim > 1) {
    throw sim_builder_error_t(this, "edge stiffness only available for dim=1, please use elasticity plugins");
  }
  if (interpolated && selfcollide != SIM_FLEXSELF_NONE) {
    throw sim_builder_error_t(this, "trilinear interpolation cannot do self-collision");
  }
  if (interpolated && internal) {
    throw sim_builder_error_t(this, "trilinear interpolation cannot do internal collisions");
  }
  nelem = (int)elem_.size()/(dim+1);

  // set nvert, rigid, centered; check size
  if (vert_.empty()) {
    centered = true;
    nvert = (int)vertbody_.size();
  }
  else {
    nvert = (int)vert_.size()/3;
    if (vertbody_.size() == 1) {
      rigid = true;
    } else if (vertbody_.size() != nvert) {
      throw sim_builder_error_t(this, "vertbody size must be 1 or nvert");
    }
  }
  if (nvert < dim+1) {
    throw sim_builder_error_t(this, "not enough vertices");
  }

  // set nnode
  nnode = static_cast<int>(nodebody_.size());
  if (nnode && !order_) {
    order_ = std::pow(nnode, 1.0 / 3) - 1;
    if (nnode != std::pow(order_ + 1, 3)) {
      throw sim_builder_error_t(this, "number of nodes must be %d^3 but it is %d", nullptr, order_, nnode);
    }
  }

  // check elem vertex ids
  for (const auto& elem : elem_) {
    if (elem < 0 || elem >= nvert) {
      throw sim_builder_error_t(this, "elem vertex id out of range");
    }
  }

  // check texcoord
  if (!texcoord_.empty() && texcoord_.size() != 2*nvert && elemtexcoord_.empty()) {
    throw sim_builder_error_t(this, "two texture coordinates per vertex expected");
  }

  // no elemtexcoord: copy from faces
  if (elemtexcoord_.empty() && !texcoord_.empty()) {
    elemtexcoord_.assign(3*nelem, 0);
    memcpy(elemtexcoord_.data(), elem_.data(), 3*nelem*sizeof(int));
  }

  // resolve material name
  sim_builder_base_t* pmat = model->FindObject(SIM_OBJ_MATERIAL, material_);
  if (pmat) {
    matid = pmat->id;
  } else if (!material_.empty()) {
    throw sim_builder_error_t(this, "unknown material '%s' in flex", material_.c_str());
  }

  // resolve body ids
  ResolveReferences(model);

  // process elements
  for (int e=0; e < (int)elem_.size()/(dim+1); e++) {
    // make sorted copy of element
    std::vector<int> el;
    el.assign(elem_.begin()+e*(dim+1), elem_.begin()+(e+1)*(dim+1));
    std::sort(el.begin(), el.end());

    // check for repeated vertices
    for (int k=0; k < dim; k++) {
      if (el[k] == el[k+1]) {
        throw sim_builder_error_t(this, "repeated vertex in element");
      }
    }
  }

  // determine rigid if not already set
  if (!rigid && !interpolated) {
    rigid = true;
    for (unsigned i=1; i < vertbodyid.size(); i++) {
      if (vertbodyid[i] != vertbodyid[0]) {
        rigid = false;
        break;
      }
    }
  }

  // determine centered if not already set
  if (!centered && !interpolated) {
    centered = true;
    for (const auto& vert : vert_) {
      if (vert != 0) {
        centered = false;
        break;
      }
    }
  }

  if (!centered && interpolated) {
    centered = true;
    for (const auto& node : node_) {
      if (node != 0) {
        centered = false;
        break;
      }
    }
  }

  // compute global vertex positions
  vertxpos = std::vector<double> (3*nvert);
  for (int i=0; i < nvert; i++) {
    // get body id, set vertxpos = body.xpos0
    int b = rigid ? vertbodyid[0] : vertbodyid[i];
    sim_math_internal_copy_vec(vertxpos.data()+3*i, model->Bodies()[b]->xpos0, 3);

    // add vertex offset within body if not centered
    if (!centered || interpolated) {
      double offset[3];
      sim_math_internal_rotVecQuat(offset, vert_.data()+3*i, model->Bodies()[b]->xquat0);
      sim_math_internal_addtovec(vertxpos.data()+3*i, offset, 3);
    }

    if (interpolated) {
      // this should happen in ResolveReferences but we need a body id in this loop to compute
      // the global vertex position, this is a hack since it is the id of the parent body
      vertbodyid[i] = -1;
    }
  }

  // compute global node positions
  std::vector<double> nodexpos = std::vector<double> (3*nnode);
  for (int i=0; i < nnode; i++) {
    // get body id, set nodexpos = body.xpos0
    int b = nodebodyid[i];
    sim_math_internal_copy_vec(nodexpos.data()+3*i, model->Bodies()[b]->xpos0, 3);

    // add node offset within body if not centered
    if (!centered) {
      double offset[3];
      sim_math_internal_rotVecQuat(offset, node_.data()+3*i, model->Bodies()[b]->xquat0);
      sim_math_internal_addtovec(nodexpos.data()+3*i, offset, 3);
    }
  }

  // reorder tetrahedra so right-handed face orientation is outside
  // faces are (0,1,2); (0,2,3); (0,3,1); (1,3,2)
  if (dim == 3) {
    for (int e=0; e < nelem; e++) {
      const int* edata = elem_.data() + e*(dim+1);
      double* v0 = vertxpos.data() + 3*edata[0];
      double* v1 = vertxpos.data() + 3*edata[1];
      double* v2 = vertxpos.data() + 3*edata[2];
      double* v3 = vertxpos.data() + 3*edata[3];
      double v01[3] = {v1[0]-v0[0], v1[1]-v0[1], v1[2]-v0[2]};
      double v02[3] = {v2[0]-v0[0], v2[1]-v0[1], v2[2]-v0[2]};
      double v03[3] = {v3[0]-v0[0], v3[1]-v0[1], v3[2]-v0[2]};

      // detect wrong orientation
      double nrm[3];
      sim_math_internal_crossvec(nrm, v01, v02);
      if (sim_math_internal_dot3(nrm, v03) > 0) {
        // flip orientation
        int tmp = elem_[e*(dim+1)+1];
        elem_[e*(dim+1)+1] = elem_[e*(dim+1)+2];
        elem_[e*(dim+1)+2] = tmp;
      }
    }
  }

  // create edges
  edgeidx_.assign(elem_.size()*kNumEdges[dim-1]/(dim+1), 0);

  // map from edge vertices to their index in `edges` vector
  std::unordered_map<std::pair<int, int>, int, PairHash> edge_indices;

  // insert local edges into global vector
  for (unsigned f = 0; f < elem_.size()/(dim+1); f++) {
    int* v = elem_.data() + f*(dim+1);
    for (int e = 0; e < kNumEdges[dim-1]; e++) {
      auto pair = std::pair(
        min(v[eledge[dim-1][e][0]], v[eledge[dim-1][e][1]]),
        max(v[eledge[dim-1][e][0]], v[eledge[dim-1][e][1]]));

      // if edge is already present in the vector only store its index
      auto [it, inserted] = edge_indices.insert({pair, nedge});

      if (inserted) {
        edge.push_back(pair);
        edgeidx_[f*kNumEdges[dim-1]+e] = nedge++;
      } else {
        edgeidx_[f*kNumEdges[dim-1]+e] = it->second;
      }
    }
  }

  // set size
  nedge = (int)edge.size();

  // create flap stencil
  if (dim == 2) {
    CreateFlapStencil(flaps, elem_, edgeidx_);
  }

  // compute elasticity
  if (young > 0) {
    if (poisson < 0 || poisson >= 0.5) {
      throw sim_builder_error_t(this, "Poisson ratio must be in [0, 0.5)");
    }

    // linear elasticity
    stiffness.assign(21*nelem, 0);
    if (interpolated) {
      int min_size = ceil(nodexpos.size()*nodexpos.size() / 21);
      if (min_size > nelem) {
        throw sim_builder_error_t(this, "Trilinear dofs are require at least %d elements", "", min_size);
      }
      ComputeLinearStiffness(stiffness, nodexpos.data(), young, poisson, order_);
    }

    // geometrically nonlinear elasticity
    for (unsigned int t = 0; t < nelem; t++) {
      if (interpolated) {
        continue;
      }
      if (dim == 2 && elastic2d >= 2 && thickness > 0) {
        ComputeStiffness<Stencil2D>(stiffness, vertxpos,
                                    elem_.data() + (dim + 1) * t, t, young,
                                    poisson, thickness);
      } else if (dim == 3) {
        ComputeStiffness<Stencil3D>(stiffness, vertxpos,
                                    elem_.data() + (dim + 1) * t, t, young,
                                    poisson);
      }
    }

    // bending stiffness (2D only)
    if (dim == 2 && (elastic2d == 1 || elastic2d == 3)) {
      if (thickness < 0) {
        throw sim_builder_error_t(this, "thickness must be positive for bending stiffness");
      }
      bending.assign(nedge*17, 0);

      for (unsigned int e = 0; e < nedge; e++) {
        ComputeBending<StencilFlap>(bending.data() + 17 * e, vertxpos.data(), flaps[e].vertices,
                                    young / (2 * (1 + poisson)), thickness);
      }
    }
  }

  // placeholder for setting plugins parameters, currently not used
  for (const auto& vbodyid : vertbodyid) {
    if (vbodyid < 0) {
      continue;
    }
    if (model->Bodies()[vbodyid]->plugin.element) {
      sim_builder_plugin_t* plugin_instance =
        static_cast<sim_builder_plugin_t*>(model->Bodies()[vbodyid]->plugin.element);
      if (!plugin_instance) {
        throw sim_builder_error_t(this, "plugin instance not found");
      }
    }
  }

  // create shell fragments and element-vertex collision pairs
  CreateShellPair();

  // create bounding volume hierarchy
  CreateBVH();

  // compute bounding box coordinates
  vert0_.assign(3*nvert, 0);
  const sim_scalar_t* bvh = tree.Bvh().data();
  size[0] = bvh[3] - radius;
  size[1] = bvh[4] - radius;
  size[2] = bvh[5] - radius;
  for (int j=0; j < nvert; j++) {
    for (int k=0; k < 3; k++) {
      if (size[k] > SIM_MINVAL) {
        vert0_[3*j+k] = (vertxpos[3*j+k] - bvh[k]) / (2*size[k]) + 0.5;
      } else {
        vert0_[3*j+k] = 0.5;
      }
    }
  }

  // store node cartesian positions
  node0_.assign(3*nnode, 0);
  for (int i=0; i < nnode; i++) {
    sim_math_internal_copy_vec(node0_.data()+3*i, nodexpos.data()+3*i, 3);
  }
}



// create flex BVH
void SIM_CFlex::CreateBVH() {
  int nbvh = 0;

  // allocate element bounding boxes
  elemaabb_.resize(6*nelem);
  tree.AllocateBoundingVolumes(nelem);

  // construct element bounding boxes, add to hierarchy
  for (int e=0; e < nelem; e++) {
    const int* edata = elem_.data() + e*(dim+1);

    // skip inactive in 3D
    if (dim == 3 && elemlayer[e] >= activelayers) {
      continue;
    }

    // compute min and max along each global axis
    double xmin[3], xmax[3];
    sim_math_internal_copy_vec(xmin, vertxpos.data() + 3*edata[0], 3);
    sim_math_internal_copy_vec(xmax, vertxpos.data() + 3*edata[0], 3);
    for (int i=1; i <= dim; i++) {
      for (int j=0; j < 3; j++) {
        xmin[j] = std::min(xmin[j], vertxpos[3*edata[i]+j]);
        xmax[j] = std::max(xmax[j], vertxpos[3*edata[i]+j]);
      }
    }

    // compute aabb (center, size)
    elemaabb_[6*e+0] = 0.5*(xmax[0]+xmin[0]);
    elemaabb_[6*e+1] = 0.5*(xmax[1]+xmin[1]);
    elemaabb_[6*e+2] = 0.5*(xmax[2]+xmin[2]);
    elemaabb_[6*e+3] = 0.5*(xmax[0]-xmin[0]) + radius;
    elemaabb_[6*e+4] = 0.5*(xmax[1]-xmin[1]) + radius;
    elemaabb_[6*e+5] = 0.5*(xmax[2]-xmin[2]) + radius;

    // add bounding volume for this element
    // contype and conaffinity are set to nonzero to force bvh generation
    const double* aabb = elemaabb_.data() + 6*e;
    tree.AddBoundingVolume(e, 1, 1, aabb, nullptr, aabb);
    nbvh++;
  }

  // create hierarchy
  tree.RemoveInactiveVolumes(nbvh);
  tree.CreateBVH();
}



// create shells and element-vertex collision pairs
void SIM_CFlex::CreateShellPair(void) {
  std::vector<std::vector<int> > fragspec(nelem*(dim+1));   // [sorted frag vertices, elem, original frag vertices]
  std::vector<std::vector<int> > connectspec;               // [elem1, elem2, common sorted frag vertices]
  std::vector<bool> border(nelem, false);              // is element on the border
  std::vector<bool> borderfrag(nelem*(dim+1), false);  // is fragment on the border

  // make fragspec
  for (int e=0; e < nelem; e++) {
    int n = e*(dim+1);

    // element vertices in original (unsorted) order
    std::vector<int> el;
    el.assign(elem_.begin()+n, elem_.begin()+n+dim+1);

    // line: 2 vertex fragments
    if (dim == 1) {
      fragspec[n].push_back(el[0]);
      fragspec[n].push_back(e);
      fragspec[n].push_back(el[0]);

      fragspec[n+1].push_back(el[1]);
      fragspec[n+1].push_back(e);
      fragspec[n+1].push_back(el[1]);
    }

    // triangle: 3 edge fragments
    else if (dim == 2) {
      fragspec[n].push_back(el[0]);
      fragspec[n].push_back(el[1]);
      fragspec[n].push_back(e);
      fragspec[n].push_back(el[0]);
      fragspec[n].push_back(el[1]);

      fragspec[n+2].push_back(el[1]);
      fragspec[n+2].push_back(el[2]);
      fragspec[n+2].push_back(e);
      fragspec[n+2].push_back(el[1]);
      fragspec[n+2].push_back(el[2]);

      fragspec[n+1].push_back(el[2]);
      fragspec[n+1].push_back(el[0]);
      fragspec[n+1].push_back(e);
      fragspec[n+1].push_back(el[2]);
      fragspec[n+1].push_back(el[0]);
    }

    // tetrahedron: 4 face fragments
    else {
      fragspec[n].push_back(el[0]);
      fragspec[n].push_back(el[1]);
      fragspec[n].push_back(el[2]);
      fragspec[n].push_back(e);
      fragspec[n].push_back(el[0]);
      fragspec[n].push_back(el[1]);
      fragspec[n].push_back(el[2]);

      fragspec[n+2].push_back(el[0]);
      fragspec[n+2].push_back(el[2]);
      fragspec[n+2].push_back(el[3]);
      fragspec[n+2].push_back(e);
      fragspec[n+2].push_back(el[0]);
      fragspec[n+2].push_back(el[2]);
      fragspec[n+2].push_back(el[3]);

      fragspec[n+1].push_back(el[0]);
      fragspec[n+1].push_back(el[3]);
      fragspec[n+1].push_back(el[1]);
      fragspec[n+1].push_back(e);
      fragspec[n+1].push_back(el[0]);
      fragspec[n+1].push_back(el[3]);
      fragspec[n+1].push_back(el[1]);

      fragspec[n+3].push_back(el[1]);
      fragspec[n+3].push_back(el[3]);
      fragspec[n+3].push_back(el[2]);
      fragspec[n+3].push_back(e);
      fragspec[n+3].push_back(el[1]);
      fragspec[n+3].push_back(el[3]);
      fragspec[n+3].push_back(el[2]);
    }
  }

  // sort first segment of each fragspec
  if (dim > 1) {
    for (int n=0; n < nelem*(dim+1); n++) {
      std::sort(fragspec[n].begin(), fragspec[n].begin()+dim);
    }
  }

  // sort fragspec
  std::sort(fragspec.begin(), fragspec.end());

  // make border and connectspec, record borderfrag
  int cnt = 1;
  for (int n=1; n < nelem*(dim+1); n++) {
    // extract frag vertices, without elem
    std::vector<int> previous = {fragspec[n-1].begin(), fragspec[n-1].begin()+dim};
    std::vector<int> current = {fragspec[n].begin(), fragspec[n].begin()+dim};

    // same sequential fragments
    if (previous == current) {
      // found pair of elements connected by common fragment
      std::vector<int> connect;
      connect.insert(connect.end(), fragspec[n-1][dim]);
      connect.insert(connect.end(), fragspec[n][dim]);
      connect.insert(connect.end(), fragspec[n].begin(), fragspec[n].begin()+dim);
      connectspec.push_back(connect);

      // count same sequential fragments
      cnt++;
    }

    // different sequential fragments
    else {
      // found border fragment
      if (cnt == 1) {
        border[fragspec[n-1][dim]] = true;
        borderfrag[n-1] = true;
      }

      // reset count
      cnt = 1;
    }
  }

  // last fragment is border
  if (cnt == 1) {
    int n = nelem*(dim+1);
    border[fragspec[n-1][dim]] = true;
    borderfrag[n-1] = true;
  }

  // create shell
  for (unsigned i=0; i < borderfrag.size(); i++) {
    if (borderfrag[i]) {
      // add fragment vertices, in original order
      shell.insert(shell.end(), fragspec[i].begin()+dim+1, fragspec[i].end());
    }
  }

  // compute elemlayer (distance from border) via value iteration in 3D
  if (dim < 3) {
    elemlayer = std::vector<int> (nelem, 0);
  }
  else {
    elemlayer = std::vector<int> (nelem, nelem+1);   // init with greater than max value
    for (int e=0; e < nelem; e++) {
      if (border[e]) {
        elemlayer[e] = 0;                       // set border elements to 0
      }
    }

    bool change = true;
    while (change) {                            // repeat while changes are happening
      change = false;

      // process edges of element connectivity graph
      for (const auto& connect : connectspec) {
        int e1 = connect[0];             // get element pair for this edge
        int e2 = connect[1];
        if (elemlayer[e1] > elemlayer[e2]+1) {
          elemlayer[e1] = elemlayer[e2]+1;      // better value found for e1: update
          change = true;
        } else if (elemlayer[e2] > elemlayer[e1]+1) {
          elemlayer[e2] = elemlayer[e1]+1;      // better value found for e2: update
          change = true;
        }
      }
    }
  }

  // create evpairs in 1D and 2D
  if (dim < 3) {
    // process connected element pairs containing a border element
    for (const auto& connect : connectspec) {
      if (border[connect[0]] || border[connect[1]]) {
        // extract common fragment
        std::vector<int> frag = {connect.begin()+2, connect.end()};

        // process both elements
        for (int ei=0; ei < 2; ei++) {
          const int* edata = elem_.data() + connect[ei]*(dim+1);

          // find element vertex that is not in the common fragment
          for (int i=0; i <= dim; i++) {
            if (frag.end() == std::find(frag.begin(), frag.end(), edata[i])) {
              // add ev pair, involving the other element in connectspec
              evpair.push_back(connect[1-ei]);
              evpair.push_back(edata[i]);

              // one such vertex exists
              break;
            }
          }
        }
      }
    }
  }
}
