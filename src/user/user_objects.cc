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

#include "user/user_objects.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <functional>
#include <limits>
#include <memory>
#include <new>
#include <optional>
#include <random>
#include <sstream>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

#include "lodepng.h"
#include "cc/array_safety.h"
#include "engine/engine_passive.h"
#include "engine/engine_support.h"
#include <simcore/SIM_spec.h>
#include <simcore/core_api.h>
#include "user/user_api.h"
#include "user/user_cache.h"
#include "user/user_model.h"
#include "user/user_resource.h"
#include "user/user_util.h"

namespace {
namespace sim_math = ::simcore::util;
using simcore::user::FilePath;

class PNGImage {
 public:
  static PNGImage Load(const sim_builder_base_t* obj, SIM_Resource* resource,
                       LodePNGColorType color_type);
  int Width() const { return width_; }
  int Height() const { return height_; }
  bool IsSRGB() const { return is_srgb_; }

  std::byte operator[] (int i) const { return data_[i]; }

  SIM_ByteVec&& MoveData() && { return std::move(data_); }

 private:
  std::size_t Size() const {
    return data_.size() + (3 * sizeof(int));
  }

  int width_;
  int height_;
  bool is_srgb_;
  LodePNGColorType color_type_;
  SIM_ByteVec data_;
};

PNGImage PNGImage::Load(const sim_builder_base_t* obj, SIM_Resource* resource,
                        LodePNGColorType color_type) {
  PNGImage image;
  image.color_type_ = color_type;

  // open PNG resource
  const unsigned char* buffer;
  int nbuffer = sim_math_readResource(resource, (const void**) &buffer);

  if (nbuffer < 0) {
    throw sim_builder_error_t(obj, "could not read PNG file '%s'", resource->name);
  }

  if (!nbuffer) {
    throw sim_builder_error_t(obj, "empty PNG file '%s'", resource->name);
  }

  // decode PNG from buffer
  unsigned int w, h;

  lodepng::State state;
  state.info_raw.colortype = image.color_type_;
  state.info_raw.bitdepth = 8;
  unsigned char* data_ptr = nullptr;
  unsigned err = lodepng_decode(&data_ptr, &w, &h, &state, buffer, nbuffer);
  struct free_delete {
    void operator()(unsigned char* ptr) const { std::free(ptr); }
  };
  std::unique_ptr<unsigned char, free_delete> data{data_ptr};

  // check for errors
  if (err) {
    std::stringstream ss;
    ss << "error decoding PNG file '" << resource->name << "': " << lodepng_error_text(err);
    throw sim_builder_error_t(obj, "%s", ss.str().c_str());
  }

  if (data) {
    size_t buffersize = lodepng_get_raw_size(w, h, &state.info_raw);
    image.data_.insert(image.data_.end(),
                       reinterpret_cast<std::byte*>(data.get()),
                       reinterpret_cast<std::byte*>(&data.get()[buffersize]));
  }

  image.width_ = w;
  image.height_ = h;
  image.is_srgb_ = (state.info_png.srgb_defined == 1);

  if (image.width_ <= 0 || image.height_ < 0) {
    std::stringstream ss;
    ss << "error decoding PNG file '" << resource->name << "': " << "dimensions are invalid";
    throw sim_builder_error_t(obj, "%s", ss.str().c_str());
  }

  return image;
}

// associate all child list elements with a frame and copy them to parent list, clear child list
template <typename T>
void MapFrame(std::vector<T*>& parent, std::vector<T*>& child,
              sim_builder_frame_t* frame, sim_builder_body_t* parent_body) {
  std::for_each(child.begin(), child.end(), [frame, parent_body](T* element) {
      element->SetFrame(frame);
      element->SetParent(parent_body);
    });
  parent.insert(parent.end(), child.begin(), child.end());
  child.clear();
}

}  // namespace


// utiility function for checking size parameters
static void checksize(double* size, SIM_tGeom type, sim_builder_base_t* object, const char* name, int id) {
  // plane: handle infinite
  if (type == SIM_GEOM_PLANE) {
    if (size[2] <= 0) {
      throw sim_builder_error_t(object, "plane size(3) must be positive");
    }
  }

  // regular geom
  else {
    for (int i=0; i < SIM_GEOMINFO[type]; i++) {
      if (size[i] <= 0) {
        throw sim_builder_error_t(object, "size %d must be positive in geom", nullptr, i);
      }
    }
  }
}

// error message for missing "limited" attribute
static void checklimited(
  const sim_builder_base_t* obj,
  bool autolimits, const char* entity, const char* attr, int limited, bool hasrange) {
  if (!autolimits && limited == 2 && hasrange) {
    std::stringstream ss;
    ss << entity << " has `" << attr << "range` but not `" << attr << "limited`. "
       << "set the autolimits=\"true\" compiler option, specify `" << attr << "limited` "
       << "explicitly (\"true\" or \"false\"), or remove the `" << attr << "range` attribute.";
    throw sim_builder_error_t(obj, "%s", ss.str().c_str());
  }
}

// returns true if limits should be active
static bool islimited(int limited, const double range[2]) {
  if (limited == SIM_LIMITED_TRUE || (limited == SIM_LIMITED_AUTO && range[0] < range[1])) {
    return true;
  }
  return false;
}

//------------------------- class sim_builder_error_t implementation ------------------------------------------

// constructor
sim_builder_error_t::sim_builder_error_t(const sim_builder_base_t* obj, const char* msg, const char* str, int pos1, int pos2) {
  char temp[600];

  // init
  warning = false;
  if (obj || msg) {
    sim_math::sprintf_arr(message, "Error");
  } else {
    message[0] = 0;
  }

  // construct error message
  if (msg) {
    if (str) {
      sim_math::sprintf_arr(temp, msg, str, pos1, pos2);
    } else {
      sim_math::sprintf_arr(temp, msg, pos1, pos2);
    }

    sim_math::strcat_arr(message, ": ");
    sim_math::strcat_arr(message, temp);
  }

  // append info from sim_builder_base_t element
  if (obj) {
    // with or without xml position
    if (!obj->info.empty()) {
      sim_math::sprintf_arr(temp, "Element name '%s', id %d, %s",
                       obj->name.c_str(), obj->id, obj->info.c_str());
    } else {
      sim_math::sprintf_arr(temp, "Element name '%s', id %d", obj->name.c_str(), obj->id);
    }

    // append to message
    sim_math::strcat_arr(message, "\n");
    sim_math::strcat_arr(message, temp);
  }
}



//------------------ alternative orientation implementation ----------------------------------------

// compute frame orientation given alternative specifications
// used for geom, site, body and camera frames
const char* ResolveOrientation(double* quat, bool degree, const char* sequence,
                               const SIM_sOrientation& orient) {
  double axisangle[4];
  double xyaxes[6];
  double zaxis[3];
  double euler[3];

  sim_math_internal_copy_vec(axisangle, orient.axisangle, 4);
  sim_math_internal_copy_vec(xyaxes, orient.xyaxes, 6);
  sim_math_internal_copy_vec(zaxis, orient.zaxis, 3);
  sim_math_internal_copy_vec(euler, orient.euler, 3);

  // set quat using axisangle
  if (orient.type == SIM_ORIENTATION_AXISANGLE) {
    // convert to radians if necessary, normalize axis
    if (degree) {
      axisangle[3] = axisangle[3] / 180.0 * SIM_PI;
    }
    if (sim_math_internal_normvec(axisangle, 3) < SIM_EPS) {
      return "axisangle too small";
    }

    // construct quaternion
    double ang2 = axisangle[3]/2;
    quat[0] = cos(ang2);
    quat[1] = sin(ang2)*axisangle[0];
    quat[2] = sin(ang2)*axisangle[1];
    quat[3] = sin(ang2)*axisangle[2];
  }

  // set quat using xyaxes
  if (orient.type == SIM_ORIENTATION_XYAXES) {
    // normalize x axis
    if (sim_math_internal_normvec(xyaxes, 3) < SIM_EPS) {
      return "xaxis too small";
    }

    // make y axis orthogonal to x axis, normalize
    double d = sim_math_internal_dot3(xyaxes, xyaxes+3);
    xyaxes[3] -= xyaxes[0]*d;
    xyaxes[4] -= xyaxes[1]*d;
    xyaxes[5] -= xyaxes[2]*d;
    if (sim_math_internal_normvec(xyaxes+3, 3) < SIM_EPS) {
      return "yaxis too small";
    }

    // compute and normalize z axis
    double z[3];
    sim_math_internal_crossvec(z, xyaxes, xyaxes+3);
    if (sim_math_internal_normvec(z, 3) < SIM_EPS) {
      return "cross(xaxis, yaxis) too small";
    }

    // convert frame into quaternion
    sim_math_internal_frame2quat(quat, xyaxes, xyaxes+3, z);
  }

  // set quat using zaxis
  if (orient.type == SIM_ORIENTATION_ZAXIS) {
    if (sim_math_internal_normvec(zaxis, 3) < SIM_EPS) {
      return "zaxis too small";
    }
    sim_math_internal_z2quat(quat, zaxis);
  }


  // handle euler
  if (orient.type == SIM_ORIENTATION_EULER) {
    // convert to radians if necessary
    if (degree) {
      for (int i=0; i < 3; i++) {
        euler[i] = euler[i] / 180.0 * SIM_PI;
      }
    }

    // init
    sim_math_internal_set_vec(quat, 1, 0, 0, 0);

    // loop over euler angles, accumulate rotations
    for (int i=0; i < 3; i++) {
      double tmp[4], qrot[4] = {cos(euler[i]/2), 0, 0, 0};
      double sa = sin(euler[i]/2);

      // construct quaternion rotation
      if (sequence[i] == 'x' || sequence[i] == 'X') {
        qrot[1] = sa;
      } else if (sequence[i] == 'y' || sequence[i] == 'Y') {
        qrot[2] = sa;
      } else if (sequence[i] == 'z' || sequence[i] == 'Z') {
        qrot[3] = sa;
      } else {
        return "euler sequence can only contain x, y, z, X, Y, Z";
      }

      // accumulate rotation
      if (sequence[i] == 'x' || sequence[i] == 'y' || sequence[i] == 'z') {
        sim_math_internal_mulquat(tmp, quat, qrot);  // moving axes: post-multiply
      } else {
        sim_math_internal_mulquat(tmp, qrot, quat);  // fixed axes: pre-multiply
      }
      sim_math_internal_copy_vec(quat, tmp, 4);
    }

    // normalize, just in case
    sim_math_internal_normvec(quat, 4);
  }

  return 0;
}



//------------------------- class SIM_CBoundingVolumeHierarchy implementation ------------------------


// assign position and orientation
void SIM_CBoundingVolumeHierarchy::Set(double ipos_element[3], double iquat_element[4]) {
  sim_math_internal_copy_vec(ipos_, ipos_element, 3);
  sim_math_internal_copy_vec(iquat_, iquat_element, 4);
}



void SIM_CBoundingVolumeHierarchy::AllocateBoundingVolumes(int nleaf) {
  nbvh_ = 0;
  bvh_.clear();
  child_.clear();
  nodeid_.clear();
  level_.clear();
  bvleaf_.clear();
  bvleaf_.reserve(nleaf);
}


void SIM_CBoundingVolumeHierarchy::RemoveInactiveVolumes(int nmax) {
  bvleaf_.erase(bvleaf_.begin() + nmax, bvleaf_.end());
}

const SIM_CBoundingVolume*
SIM_CBoundingVolumeHierarchy::AddBoundingVolume(int id, int contype, int conaffinity,
                                              const double* pos, const double* quat,
                                              const double* aabb) {
  bvleaf_.emplace_back(id, contype, conaffinity, pos, quat, aabb);
  return &bvleaf_.back();
}


const SIM_CBoundingVolume*
SIM_CBoundingVolumeHierarchy::AddBoundingVolume(const int* id, int contype, int conaffinity,
                                              const double* pos, const double* quat,
                                              const double* aabb) {
  bvleaf_.emplace_back(id, contype, conaffinity, pos, quat, aabb);
  return &bvleaf_.back();
}


// create bounding volume hierarchy
void SIM_CBoundingVolumeHierarchy::CreateBVH() {
  std::vector<BVElement> elements;
  Make(elements);
  MakeBVH(elements.begin(), elements.end());
}


void SIM_CBoundingVolumeHierarchy::Make(std::vector<BVElement>& elements) {
  // precompute the positions of each element in the hierarchy's axes, and drop
  // visual-only elements.
  elements.reserve(bvleaf_.size());
  double qinv[4] = {iquat_[0], -iquat_[1], -iquat_[2], -iquat_[3]};
  for (int i = 0; i < bvleaf_.size(); i++) {
    if (bvleaf_[i].Conaffinity() || bvleaf_[i].Contype()) {
      BVElement element;
      element.e = &bvleaf_[i];
      double vert[3] = {element.e->Pos(0) - ipos_[0],
                        element.e->Pos(1) - ipos_[1],
                        element.e->Pos(2) - ipos_[2]};
      sim_math_internal_rotVecQuat(element.lpos, vert, qinv);
      elements.push_back(std::move(element));
    }
  }
}


// compute bounding volume hierarchy
int SIM_CBoundingVolumeHierarchy::MakeBVH(
  std::vector<BVElement>::iterator elements_begin,
  std::vector<BVElement>::iterator elements_end, int lev) {
  int nelements = elements_end - elements_begin;
  if (nelements == 0) {
    return -1;
  }
  constexpr double kMaxVal = std::numeric_limits<double>::max();
  double AAMM[6] = {kMaxVal, kMaxVal, kMaxVal, -kMaxVal, -kMaxVal, -kMaxVal};

  // inverse transformation
  double qinv[4] = {iquat_[0], -iquat_[1], -iquat_[2], -iquat_[3]};

  // accumulate AAMM over elements
  for (auto element = elements_begin; element != elements_end; ++element) {
    // transform element aabb to aamm format
    double aamm[6] = {element->e->AABB(0) - element->e->AABB(3),
                      element->e->AABB(1) - element->e->AABB(4),
                      element->e->AABB(2) - element->e->AABB(5),
                      element->e->AABB(0) + element->e->AABB(3),
                      element->e->AABB(1) + element->e->AABB(4),
                      element->e->AABB(2) + element->e->AABB(5)};

    // update node AAMM
    for (int v=0; v < 8; v++) {
      double vert[3], box[3];
      vert[0] = (v&1 ? aamm[3] : aamm[0]);
      vert[1] = (v&2 ? aamm[4] : aamm[1]);
      vert[2] = (v&4 ? aamm[5] : aamm[2]);

      // rotate to the body inertial frame if specified
      if (element->e->Quat()) {
        sim_math_internal_rotVecQuat(box, vert, element->e->Quat());
        box[0] += element->e->Pos(0) - ipos_[0];
        box[1] += element->e->Pos(1) - ipos_[1];
        box[2] += element->e->Pos(2) - ipos_[2];
        sim_math_internal_rotVecQuat(vert, box, qinv);
      }

      AAMM[0] = std::min(AAMM[0], vert[0]);
      AAMM[1] = std::min(AAMM[1], vert[1]);
      AAMM[2] = std::min(AAMM[2], vert[2]);
      AAMM[3] = std::max(AAMM[3], vert[0]);
      AAMM[4] = std::max(AAMM[4], vert[1]);
      AAMM[5] = std::max(AAMM[5], vert[2]);
    }
  }

  // inflate flat AABBs
  for (int i = 0; i < 3; i++) {
    if (std::abs(AAMM[i] - AAMM[i+3]) < SIM_EPS) {
      AAMM[i + 0] -= SIM_EPS;
      AAMM[i + 3] += SIM_EPS;
    }
  }

  // store current index
  int index = nbvh_++;
  child_.push_back(-1);
  child_.push_back(-1);
  nodeid_.push_back(-1);
  nodeidptr_.push_back(nullptr);
  level_.push_back(lev);

  // store bounding box of the current node
  bvh_.push_back((AAMM[3] + AAMM[0]) / 2);
  bvh_.push_back((AAMM[4] + AAMM[1]) / 2);
  bvh_.push_back((AAMM[5] + AAMM[2]) / 2);
  bvh_.push_back((AAMM[3] - AAMM[0]) / 2);
  bvh_.push_back((AAMM[4] - AAMM[1]) / 2);
  bvh_.push_back((AAMM[5] - AAMM[2]) / 2);

  // leaf node, return
  if (nelements == 1) {
    child_[2*index + 0] = -1;
    child_[2*index + 1] = -1;
    nodeid_[index] = *elements_begin->e->Id();
    nodeidptr_[index] = (int*)elements_begin->e->Id();
    return index;
  }

  // find longest axis, by a margin of at least SIM_EPS, default to 0
  int axis = 0;
  double edges[3] = {AAMM[3] - AAMM[0], AAMM[4] - AAMM[1], AAMM[5] - AAMM[2]};
  if (edges[1] >= edges[0] + SIM_EPS) axis = 1;
  if (edges[2] >= edges[axis] + SIM_EPS) axis = 2;

  // find median along the axis
  auto compare = [&](const BVElement& e1, const BVElement& e2) {
    if (std::abs(e1.lpos[axis] - e2.lpos[axis]) > SIM_EPS) {
      return e1.lpos[axis] < e2.lpos[axis];
    }
    // comparing pointers gives a stable sort, because they both come from the same array
    return e1.e < e2.e;
  };

  // note: nth_element performs a partial sort of elements
  int m = nelements / 2;
  std::nth_element(elements_begin, elements_begin + m, elements_end, compare);

  // recursive calls
  if (m > 0) {
    child_[2*index + 0] = MakeBVH(elements_begin, elements_begin + m, lev + 1);
  }

  if (m != nelements) {
    child_[2*index + 1] = MakeBVH(elements_begin + m, elements_end, lev + 1);
  }

  // SHOULD NOT OCCUR
  if (child_[2*index + 0] == -1 && child_[2*index + 1] == -1) {
    sim_error("this should have been a leaf, body=%s nelements=%d",
              name_.c_str(), nelements);
  }

  if (lev > SIM_MAXTREEDEPTH) {
    sim_warning("max tree depth exceeded in body=%s", name_.c_str());
  }

  return index;
}



//------------------------- class SIM_COctree implementation --------------------------------------------

void SIM_COctree::CopyLevel(int* level) const {
  for (int i = 0; i < node_.size(); ++i) {
    level[i] = node_[i].level;
  }
}

void SIM_COctree::CopyChild(int* child) const {
  for (int i = 0; i < node_.size(); ++i) {
    for (int j = 0; j < 8; ++j) {
      child[i * 8 + j] = node_[i].child[j];
    }
  }
}

void SIM_COctree::CopyAabb(sim_scalar_t* aabb) const {
  for (int i = 0; i < node_.size(); ++i) {
    aabb[i * 6 + 0] = (node_[i].aamm[0] + node_[i].aamm[3]) / 2;
    aabb[i * 6 + 1] = (node_[i].aamm[1] + node_[i].aamm[4]) / 2;
    aabb[i * 6 + 2] = (node_[i].aamm[2] + node_[i].aamm[5]) / 2;
    aabb[i * 6 + 3] = (node_[i].aamm[3] - node_[i].aamm[0]) / 2;
    aabb[i * 6 + 4] = (node_[i].aamm[4] - node_[i].aamm[1]) / 2;
    aabb[i * 6 + 5] = (node_[i].aamm[5] - node_[i].aamm[2]) / 2;
  }
}

void SIM_COctree::CopyCoeff(sim_scalar_t* coeff) const {
  for (int i = 0; i < node_.size(); ++i) {
    for (int j = 0; j < 8; ++j) {
      coeff[i * 8 + j] = node_[i].coeff[j];
    }
  }
}

void SIM_COctree::SetFace(const std::vector<double>& vert, const std::vector<int>& face) {
  for (int i = 0; i < face.size(); i += 3) {
    std::array<double, 3> v0 = {vert[3*face[i+0]], vert[3*face[i+0]+1], vert[3*face[i+0]+2]};
    std::array<double, 3> v1 = {vert[3*face[i+1]], vert[3*face[i+1]+1], vert[3*face[i+1]+2]};
    std::array<double, 3> v2 = {vert[3*face[i+2]], vert[3*face[i+2]+1], vert[3*face[i+2]+2]};
    face_.push_back({v0, v1, v2});
  }
}


// TODO: use the same code as SIM_CBoundingVolumeHierarchy::Make()
void SIM_COctree::Make(std::vector<Triangle>& elements) {
  // rotate triangles to the body inertial frame
  elements.assign(face_.size(), {{{0}}});
  double qinv[4] = {iquat_[0], -iquat_[1], -iquat_[2], -iquat_[3]};
  for (int i = 0; i < face_.size(); i++) {
    for (int j = 0; j < 3; j++) {
      double vert[3] = {face_[i][j][0] - ipos_[0],
                        face_[i][j][1] - ipos_[1],
                        face_[i][j][2] - ipos_[2]};
      sim_math_internal_rotVecQuat(elements[i][j].data(), vert, qinv);
    }
  }
}


void SIM_COctree::CreateOctree(const double aamm[6]) {
  double aabb[6] = {(aamm[0] + aamm[3]) / 2, (aamm[1] + aamm[4]) / 2, (aamm[2] + aamm[5]) / 2,
                    (aamm[3] - aamm[0]) / 2, (aamm[4] - aamm[1]) / 2, (aamm[5] - aamm[2]) / 2};
  double box[6] = {aabb[0] - 1.1 * aabb[3], aabb[1] - 1.1 * aabb[4], aabb[2] - 1.1 * aabb[5],
                   aabb[0] + 1.1 * aabb[3], aabb[1] + 1.1 * aabb[4], aabb[2] + 1.1 * aabb[5]};
  std::vector<Triangle> elements;
  Make(elements);
  std::vector<Triangle*> elements_ptrs(elements.size());
  std::transform(elements.begin(), elements.end(), elements_ptrs.begin(),
                 [](Triangle& triangle) { return &triangle; });
  std::unordered_map<Point, int> vert_map;
  MakeOctree(elements_ptrs, box, vert_map);
  MarkHangingNodes();
}


static double dot2(const double* a, const double* b) {
  return a[0] * b[0] + a[1] * b[1];
}


// From M. Schwarz and H.-P. Seidel, "Fast Parallel Surface and Solid Voxelization on GPUs".
static bool boxTriangle(const Triangle& v, const double aamm[6]) {
  // bounding box tests
  for (int i = 0; i < 3; i++) {
    if (v[0][i] < aamm[i] && v[1][i] < aamm[i] && v[2][i] < aamm[i]) {
      return false;
    }
    int j = i + 3;
    if (v[0][i] > aamm[j] && v[1][i] > aamm[j] && v[2][i] > aamm[j]) {
      return false;
    }
  }

  // test for triangle plane and box overlap
  double n[3];
  double e[3][3];
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      e[i][j] = v[(i+1)%3][j] - v[i][j];
    }
  }

  sim_math_internal_crossvec(n, e[0], e[1]);
  double size[3] = {aamm[3] - aamm[0], aamm[4] - aamm[1], aamm[5] - aamm[2]};
  double c[3] = {n[0] > 0 ? size[0] : 0, n[1] > 0 ? size[1] : 0, n[2] > 0 ? size[2] : 0};
  double c1[3] = {c[0] - v[0][0], c[1] - v[0][1], c[2] - v[0][2]};
  double c2[3] = {size[0] - c[0] - v[0][0], size[1] - c[1] - v[0][1], size[2] - c[2] - v[0][2]};

  if ((sim_math_internal_dot3(n, aamm) + sim_math_internal_dot3(n, c1)) * (sim_math_internal_dot3(n, aamm) + sim_math_internal_dot3(n, c2)) > 0) {
    return false;
  }

  // test projection overlap
  for (int a = 0; a < 3; a++) {
    int b = (a + 1) % 3;
    int c = (a + 2) % 3;
    for (int i = 0; i < 3; i++) {
      double sign = n[a] >= 0 ? 1 : -1;
      double ne[2] = {-e[i][c] * sign, e[i][b] * sign};
      double vi[2] = {v[i][b], v[i][c]};
      double d = -dot2(ne, vi) + sim_math_max(0, size[b]*ne[0]) + sim_math_max(0, size[c]*ne[1]);
      double p[2] = {aamm[b], aamm[c]};
      if (dot2(ne, p) + d < 0) {
        return false;
      }
    }
  }

  return true;
}


void SIM_COctree::TaskToNode(const OctreeTask& task, OctNode& node,
                           std::unordered_map<Point, int>& vert_map) {
  node.level = task.lev;
  node.parent_index = task.parent_index;
  node.child_slot = task.child_slot;

  if (task.parent_index != -1) {
    node_[task.parent_index].child[task.child_slot] = task.node_index;
    const auto parent_aamm = node_[task.parent_index].aamm;
    node.aamm[0] = task.child_slot & 1 ? (parent_aamm[3] + parent_aamm[0]) / 2 : parent_aamm[0];
    node.aamm[1] = task.child_slot & 2 ? (parent_aamm[4] + parent_aamm[1]) / 2 : parent_aamm[1];
    node.aamm[2] = task.child_slot & 4 ? (parent_aamm[5] + parent_aamm[2]) / 2 : parent_aamm[2];
    node.aamm[3] = task.child_slot & 1 ? parent_aamm[3] : (parent_aamm[0] + parent_aamm[3]) / 2;
    node.aamm[4] = task.child_slot & 2 ? parent_aamm[4] : (parent_aamm[1] + parent_aamm[4]) / 2;
    node.aamm[5] = task.child_slot & 4 ? parent_aamm[5] : (parent_aamm[2] + parent_aamm[5]) / 2;
  }

  for (int i = 0; i < 8; i++) {
    Point v = {{(i & 1) ? node.aamm[3] : node.aamm[0],
                (i & 2) ? node.aamm[4] : node.aamm[1],
                (i & 4) ? node.aamm[5] : node.aamm[2]}};
    auto it = vert_map.find(v);
    if (it != vert_map.end()) {
      node.vertid[i] = it->second;
    } else {
      node.vertid[i] = nvert_;
      vert_map[v] = nvert_++;
      vert_.push_back(v);
    }
    node.child[i] = -1;
  }
}


void SIM_COctree::Subdivide(const OctreeTask& task, std::unordered_map<Point, int>& vert_map,
                          std::deque<OctreeTask>* queue, const std::vector<Triangle*>& colliding) {
  for (int i = 0; i < 8; i++) {
    OctreeTask new_task;
    new_task.elements = colliding;
    new_task.lev = task.lev + 1;
    new_task.parent_index = task.node_index;
    new_task.node_index = nnode_++;
    new_task.child_slot = i;

    node_.push_back(OctNode());
    TaskToNode(new_task, node_.back(), vert_map);
    if (queue) {
      queue->push_back(std::move(new_task));
    }
  }
}


// recursively finds the adjacent ancestor neighbor region
int SIM_COctree::FindCoarseNeighbor(int node_idx, int dir) {
  if (node_idx == -1) {
    return -1;
  }

  int parent_idx = node_[node_idx].parent_index;

  // if we are at the root, we have no parent and thus no siblings or external neighbors
  if (parent_idx == -1) {
    return -1;
  }

  int child_slot = node_[node_idx].child_slot;
  int dim = dir / 2;
  int side = dir % 2;
  int bit = 1 << dim;

  if (side != ((child_slot & bit) != 0)) {
    // internal neighbor case: This is the successful termination of the climb
    // return the adjacent sibling node
    return node_[parent_idx].child[child_slot ^ bit];
  } else {
    // external neighbor case: Recurse up the tree
    // ask our parent to find its neighbor in the same direction
    return FindCoarseNeighbor(parent_idx, dir);
  }
}


int SIM_COctree::FindNeighbor(int node_idx, int dir) {
  if (node_idx == -1) {
    return -1;
  }

  // call the helper to find the adjacent to the coarse neighbor.
  // this might be an internal node (e.g., our parent's sibling)
  int result = FindCoarseNeighbor(node_idx, dir);

  if (result == -1) {
    // no neighbor found (either at tree boundary or some other error)
    return -1;
  }

  // leaf descent
  double node_center[3] = {
      (node_[node_idx].aamm[0] + node_[node_idx].aamm[3]) / 2,
      (node_[node_idx].aamm[1] + node_[node_idx].aamm[4]) / 2,
      (node_[node_idx].aamm[2] + node_[node_idx].aamm[5]) / 2,
  };

  while (node_[result].child[0] != -1) {
    double result_center[3] = {
        (node_[result].aamm[0] + node_[result].aamm[3]) / 2,
        (node_[result].aamm[1] + node_[result].aamm[4]) / 2,
        (node_[result].aamm[2] + node_[result].aamm[5]) / 2,
    };

    // find relative octant of our node w.r.t. the neighbor's center
    int next_child_slot = 0;
    if (node_center[0] > result_center[0]) next_child_slot |= 1;
    if (node_center[1] > result_center[1]) next_child_slot |= 2;
    if (node_center[2] > result_center[2]) next_child_slot |= 4;

    int dim = dir / 2;
    int side = dir % 2;
    int bit = 1 << dim;

    // we need the child on the opposite side (adjacent to this node)
    int op_side = (side != 1);
    next_child_slot = (next_child_slot & ~bit) | (op_side * bit);
    result = node_[result].child[next_child_slot];
  }

  return result;
}


// refine the octree by subdividing nodes that are too coarse such that the
// maximum level difference between adjacent nodes is at most 1.
void SIM_COctree::BalanceOctree(std::unordered_map<Point, int>& vert_map) {
  bool changed = true;
  while (changed) {
    changed = false;
    std::vector<int> leaves;
    for (int i = 0; i < nnode_; ++i) {
      if (node_[i].child[0] == -1) {
        leaves.push_back(i);
      }
    }

    // find the nodes that are too coarse, only leaves need to be checked
    std::vector<int> leaves_to_subdivide;
    for (int leaf_idx : leaves) {
      if (node_[leaf_idx].child[0] != -1) {
        continue;
      }

      for (int dir = 0; dir < 6; ++dir) {
        int neighbor_idx = FindNeighbor(leaf_idx, dir);
        if (neighbor_idx == -1) {
          continue;
        }
        int neighbor_level = node_[neighbor_idx].level;
        if (neighbor_level > node_[leaf_idx].level + 1) {
          leaves_to_subdivide.push_back(leaf_idx);
        }
        if (node_[leaf_idx].level > neighbor_level + 1) {
          leaves_to_subdivide.push_back(neighbor_idx);
        }
      }
    }

    // subdivide the nodes that are too coarse
    if (!leaves_to_subdivide.empty()) {
      changed = true;
      for (int node_idx : leaves_to_subdivide) {
        if (node_[node_idx].child[0] == -1) {  // check if not already subdivided
          OctreeTask task;
          task.node_index = node_idx;
          task.lev = node_[node_idx].level;
          Subdivide(task, vert_map);
        }
      }
    }
  }
}


// mark all hanging vertices in the octree
void SIM_COctree::MarkHangingNodes() {
  hang_.assign(nvert_, std::vector<int>());

  std::vector<int> leaves;
  for (int i = 0; i < nnode_; ++i) {
    if (node_[i].child[0] == -1) {
      leaves.push_back(i);
    }
  }

  for (int leaf_idx : leaves) {
    for (int dir = 0; dir < 6; ++dir) {
      int neighbor_idx = FindNeighbor(leaf_idx, dir);
      if (neighbor_idx == -1 ||
          node_[neighbor_idx].level >= node_[leaf_idx].level) {
        continue;
      }

      // coarser neighbor found, this leaf's face has hanging nodes
      int dim = dir / 2;
      int side = dir % 2;

      // iterate over the 4 vertices of the leaf's face
      for (int i = 0; i < 4; ++i) {
        // construct vertex index on the face
        int v_idx = side << dim;
        int d1 = (dim + 1) % 3;
        int d2 = (dim + 2) % 3;
        v_idx |= (i & 1) << d1;
        v_idx |= ((i >> 1) & 1) << d2;

        int hv_id = node_[leaf_idx].vertid[v_idx];
        if (!hang_[hv_id].empty()) {
          continue;  // already processed
        }

        const double* hv_pos = vert_[hv_id].p.data();
        const auto& neighbor_aamm = node_[neighbor_idx].aamm;

        bool is_min[3], is_max[3];
        int on_boundary_planes = 0;
        for (int d = 0; d < 3; ++d) {
          is_min[d] = std::abs(hv_pos[d] - neighbor_aamm[d]) < 1e-9;
          is_max[d] = std::abs(hv_pos[d] - neighbor_aamm[d + 3]) < 1e-9;
          if (is_min[d] || is_max[d]) {
            on_boundary_planes++;
          }
        }

        if (on_boundary_planes == 2) {  // edge hanging
          int d_mid = -1;
          for (int d = 0; d < 3; ++d) {
            if (!is_min[d] && !is_max[d]) {
              d_mid = d;
              break;
            }
          }

          int bits[3];
          bits[d_mid] = 0;  // this will be toggled
          bits[(d_mid + 1) % 3] = is_max[(d_mid + 1) % 3];
          bits[(d_mid + 2) % 3] = is_max[(d_mid + 2) % 3];

          int nv_idx1 = (bits[2] << 2) | (bits[1] << 1) | bits[0];
          bits[d_mid] = 1;
          int nv_idx2 = (bits[2] << 2) | (bits[1] << 1) | bits[0];

          hang_[hv_id].push_back(node_[neighbor_idx].vertid[nv_idx1]);
          hang_[hv_id].push_back(node_[neighbor_idx].vertid[nv_idx2]);
        } else if (on_boundary_planes == 1) {  // face hanging
          int d_face = -1;
          for (int d = 0; d < 3; ++d) {
            if (is_min[d] || is_max[d]) {
              d_face = d;
              break;
            }
          }

          int bits[3];
          bits[d_face] = is_max[d_face];

          for (int j = 0; j < 4; ++j) {
            bits[(d_face + 1) % 3] = j & 1;
            bits[(d_face + 2) % 3] = (j >> 1) & 1;
            int nv_idx = (bits[2] << 2) | (bits[1] << 1) | bits[0];
            hang_[hv_id].push_back(node_[neighbor_idx].vertid[nv_idx]);
          }
        }
      }
    }
  }
}


void SIM_COctree::MakeOctree(const std::vector<Triangle*>& elements, const double aamm[6],
                           std::unordered_map<Point, int>& vert_map) {
  std::deque<OctreeTask> queue;
  OctreeTask initial_task;
  initial_task.elements = elements;
  initial_task.lev = 0;
  initial_task.parent_index = -1;
  initial_task.child_slot = -1;
  initial_task.node_index = nnode_++;
  queue.push_back(std::move(initial_task));

  // create root node
  node_.push_back(OctNode());
  OctNode& root = node_.back();
  std::copy(aamm, aamm + 6, root.aamm.data());
  TaskToNode(queue.front(), node_.back(), vert_map);

  while (!queue.empty()) {
    OctreeTask task = std::move(queue.front());
    queue.pop_front();

    // find all triangles that intersect the current box
    std::vector<Triangle*> colliding;
    for (auto* element : task.elements) {
      if (boxTriangle(*element, node_[task.node_index].aamm.data())) {
        colliding.push_back(element);
      }
    }

    // skip if the box is empty
    if (colliding.empty() || task.lev >= 6) {
      continue;
    }

    // subdivide the node
    Subdivide(task, vert_map, &queue, colliding);
  }

  // store the neighbors of each node
  BalanceOctree(vert_map);
}

//------------------------- class sim_builder_default_t implementation --------------------------------------------

// constructor
sim_builder_default_t::sim_builder_default_t() {
  name.clear();
  id = 0;
  parent = nullptr;
  model = 0;
  child.clear();
  elemtype = SIM_OBJ_DEFAULT;
  sim_spec_defaultJoint(&joint_.spec);
  sim_spec_defaultGeom(&geom_.spec);
  sim_spec_defaultSite(&site_.spec);
  sim_spec_defaultCamera(&camera_.spec);
  sim_spec_defaultLight(&light_.spec);
  sim_spec_defaultFlex(&flex_.spec);
  sim_spec_defaultMesh(&mesh_.spec);
  sim_spec_defaultMaterial(&material_.spec);
  sim_spec_defaultPair(&pair_.spec);
  sim_spec_defaultEquality(&equality_.spec);
  sim_spec_defaultTendon(&tendon_.spec);
  sim_spec_defaultActuator(&actuator_.spec);

  // make sure all the pointers are local
  PointToLocal();
}



// constructor with model
sim_builder_default_t::sim_builder_default_t(sim_builder_model_t* _model) : sim_builder_default_t() {
  model = _model;
}



// copy constructor
sim_builder_default_t::sim_builder_default_t(const sim_builder_default_t& other) {
  *this = other;
}



// compiler
void sim_builder_default_t::Compile(const sim_builder_model_t* model) {
  CopyFromSpec();

  // enforce length of all default userdata arrays
  joint_.userdata_.resize(model->nuser_jnt);
  geom_.userdata_.resize(model->nuser_geom);
  site_.userdata_.resize(model->nuser_site);
  camera_.userdata_.resize(model->nuser_cam);
  tendon_.userdata_.resize(model->nuser_tendon);
  actuator_.userdata_.resize(model->nuser_actuator);
}



// assignment operator
sim_builder_default_t& sim_builder_default_t::operator=(const sim_builder_default_t& other) {
  if (this != &other) {
    CopyWithoutChildren(other);

    // copy the rest of the default tree
    *this += other;
  }
  return *this;
}



sim_builder_default_t& sim_builder_default_t::operator+=(const sim_builder_default_t& other) {
  for (unsigned int i=0; i < other.child.size(); i++) {
    child.push_back(new sim_builder_default_t(*other.child[i]));  // triggers recursive call
    child.back()->parent = this;
  }
  return *this;
}



void sim_builder_default_t::NameSpace(const sim_builder_model_t* m) {
  if (!name.empty()) {
    name = m->prefix + name + m->suffix;
  }
  for (auto c : child) {
    c->NameSpace(m);
  }
}



void sim_builder_default_t::CopyWithoutChildren(const sim_builder_default_t& other) {
  name = other.name;
  elemtype = other.elemtype;
  parent = nullptr;
  child.clear();
  joint_ = other.joint_;
  geom_ = other.geom_;
  site_ = other.site_;
  camera_ = other.camera_;
  light_ = other.light_;
  flex_ = other.flex_;
  mesh_ = other.mesh_;
  material_ = other.material_;
  pair_ = other.pair_;
  equality_ = other.equality_;
  tendon_ = other.tendon_;
  actuator_ = other.actuator_;
  PointToLocal();
}



void sim_builder_default_t::PointToLocal() {
  joint_.PointToLocal();
  geom_.PointToLocal();
  site_.PointToLocal();
  camera_.PointToLocal();
  light_.PointToLocal();
  flex_.PointToLocal();
  mesh_.PointToLocal();
  material_.PointToLocal();
  pair_.PointToLocal();
  equality_.PointToLocal();
  tendon_.PointToLocal();
  actuator_.PointToLocal();
  spec.element = static_cast<sim_spec_element_t*>(this);
  spec.joint = &joint_.spec;
  spec.geom = &geom_.spec;
  spec.site = &site_.spec;
  spec.camera = &camera_.spec;
  spec.light = &light_.spec;
  spec.flex = &flex_.spec;
  spec.mesh = &mesh_.spec;
  spec.material = &material_.spec;
  spec.pair = &pair_.spec;
  spec.equality = &equality_.spec;
  spec.tendon = &tendon_.spec;
  spec.actuator = &actuator_.spec;
}



void sim_builder_default_t::CopyFromSpec() {
  joint_.CopyFromSpec();
  geom_.CopyFromSpec();
  site_.CopyFromSpec();
  camera_.CopyFromSpec();
  light_.CopyFromSpec();
  flex_.CopyFromSpec();
  mesh_.CopyFromSpec();
  material_.CopyFromSpec();
  pair_.CopyFromSpec();
  equality_.CopyFromSpec();
  tendon_.CopyFromSpec();
  actuator_.CopyFromSpec();
}



//------------------------- class sim_builder_base_t implementation -------------------------------------------

// constructor
sim_builder_base_t::sim_builder_base_t() {
  name.clear();
  classname.clear();
  id = -1;
  info = "";
  model = 0;
  frame = nullptr;
}



sim_builder_base_t::sim_builder_base_t(const sim_builder_base_t& other) {
  *this = other;
}



sim_builder_base_t& sim_builder_base_t::operator=(const sim_builder_base_t& other) {
  if (this != &other) {
    *static_cast<SIM_CBase_*>(this) = static_cast<const SIM_CBase_&>(other);
  }
  return *this;
}



void sim_builder_base_t::NameSpace(const sim_builder_model_t* m) {
  if (!name.empty()) {
    name = m->prefix + name + m->suffix;
  }
  if (!classname.empty() && classname != "main" && m != model) {
    classname = m->prefix + classname + m->suffix;
  }
}



SIM_sCompiler* sim_builder_base_t::FindCompiler(const SIM_sCompiler* compiler) const {
  sim_spec_t* origin = model->FindSpec(compiler);
  return origin ? &origin->compiler : &model->spec.compiler;
}



// load resource if found (fallback to OS filesystem)
SIM_Resource* sim_builder_base_t::LoadResource(const std::string& modelfiledir,
                                  const std::string& filename,
                                  const SIM_VFS* vfs) {
  // try reading from provided VFS or fallback to OS filesystem
  std::array<char, 1024> error;
  SIM_Resource* resource = sim_math_openResource(modelfiledir.c_str(), filename.c_str(), vfs,
                                          error.data(), error.size());
  if (!resource) {
    throw sim_builder_error_t(nullptr, "%s", error.data());
  }
  return resource;
}


// Get and sanitize content type from raw_text if not empty, otherwise parse
// content type from resource_name; throw error on failure
std::string sim_builder_base_t::GetAssetContentType(std::string_view resource_name,
                                         std::string_view raw_text) {
  if (!raw_text.empty()) {
    auto type = sim_math_internal_parseContentTypeAttrType(raw_text);
    auto subtype = sim_math_internal_parseContentTypeAttrSubtype(raw_text);
    if (!type.has_value() || !subtype.has_value()) {
      return "";
    }
    return std::string(*type) + "/" + std::string(*subtype);
  } else {
    return sim_math_internal_extToContentType(resource_name);
  }
}


void sim_builder_base_t::SetFrame(sim_builder_frame_t* _frame) {
  if (!_frame) {
    return;
  }
  if (_frame->body && GetParent() != _frame->body) {
    throw sim_builder_error_t(this, "Frame and body '%s' have mismatched parents", name.c_str());
  }
  frame = _frame;
}

void sim_builder_base_t::SetUserValue(std::string_view key, const void* data,
                           void (*cleanup)(const void*)) {
  user_payload_[std::string(key)] = UserValue(data, cleanup);
}

const void* sim_builder_base_t::GetUserValue(std::string_view key) {
  auto found = user_payload_.find(std::string(key));
  return found != user_payload_.end() ? found->second.value : nullptr;
}


void sim_builder_base_t::DeleteUserValue(std::string_view key) {
  user_payload_.erase(std::string(key));
}


//------------------ class sim_builder_body_t implementation --------------------------------------------------

// constructor
sim_builder_body_t::sim_builder_body_t(sim_builder_model_t* _model) {
  // set model pointer
  model = _model;
  if (_model) compiler = &_model->spec.compiler;

  refcount = 1;
  sim_spec_defaultBody(&spec);
  elemtype = SIM_OBJ_BODY;
  parent = nullptr;
  weldid = -1;
  dofnum = 0;
  lastdof = -1;
  subtreedofs = 0;
  contype = 0;
  conaffinity = 0;
  margin = 0;
  sim_math_internal_zerovec(xpos0, 3);
  sim_math_internal_set_vec(xquat0, 1, 0, 0, 0);
  last_attached = nullptr;
  mocapid = -1;

  // clear object lists
  bodies.clear();
  geoms.clear();
  frames.clear();
  joints.clear();
  sites.clear();
  cameras.clear();
  lights.clear();
  spec_userdata_.clear();

  // in case this body is not compiled
  CopyFromSpec();

  // point to local (needs to be after defaults)
  PointToLocal();
}



sim_builder_body_t::sim_builder_body_t(const sim_builder_body_t& other, sim_builder_model_t* _model) {
  model = _model;
  compiler = FindCompiler(other.compiler);
  *this = other;
  CopyPlugin();
}



sim_builder_body_t& sim_builder_body_t::operator=(const sim_builder_body_t& other) {
  if (this != &other) {
    spec = other.spec;
    *static_cast<SIM_CBody_*>(this) = static_cast<const SIM_CBody_&>(other);
    *static_cast<sim_spec_body_t*>(this) = static_cast<const sim_spec_body_t&>(other);
    bodies.clear();
    frames.clear();
    geoms.clear();
    joints.clear();
    sites.clear();
    cameras.clear();
    lights.clear();
    id = -1;
    subtreedofs = 0;

    // add elements to lists
    *this += other;
  }
  PointToLocal();
  return *this;
}



// copy children of other body into body
sim_builder_body_t& sim_builder_body_t::operator+=(const sim_builder_body_t& other) {
  // map other frames to indices
  std::map<sim_builder_frame_t*, int> fmap;
  for (int i=0; i < other.frames.size(); i++) {
    fmap[other.frames[i]] = i + frames.size();
  }

  // copy frames, needs to happen first
  CopyList(frames, other.frames, fmap);

  // copy all children
  CopyList(geoms, other.geoms, fmap);
  CopyList(joints, other.joints, fmap);
  CopyList(sites, other.sites, fmap);
  CopyList(cameras, other.cameras, fmap);
  CopyList(lights, other.lights, fmap);

  for (int i=0; i < other.bodies.size(); i++) {
    bodies.push_back(new sim_builder_body_t(*other.bodies[i], model));  // triggers recursive call
    bodies.back()->parent = this;
    bodies.back()->frame = nullptr;
    if (other.bodies[i]->frame) {
      if (fmap.find(other.bodies[i]->frame) != fmap.end()) {
        bodies.back()->frame = frames[fmap[other.bodies[i]->frame]];
      } else {
        throw sim_builder_error_t(this, "Frame '%s' not found in other body",
                       other.bodies[i]->frame->name.c_str());
      }
      if (bodies.back()->frame && bodies.back()->frame->body != this) {
        throw sim_builder_error_t(this, "Frame and body '%s' have mismatched parents", name.c_str());
      }
    }
  }

  return *this;
}



// attach frame to body
sim_builder_body_t& sim_builder_body_t::operator+=(const sim_builder_frame_t& other) {
  // append a copy of the attached spec
  if (other.model != model && !model->FindSpec(&other.model->spec.compiler)) {
    model->AppendSpec(&other.model->spec, &other.model->spec.compiler);
    static_cast<sim_builder_model_t*>(other.model->spec.element)->AddRef();
  }

  // create a copy of the subtree that contains the frame
  sim_builder_body_t* subtree = other.body;
  other.model->prefix = other.prefix;
  other.model->suffix = other.suffix;
  other.model->StoreKeyframes(model);
  sim_builder_model_t* other_model = other.model;

  // attach defaults
  if (other_model != model) {
    sim_builder_default_t* subdef = new sim_builder_default_t(*other_model->Default());
    subdef->NameSpace(other_model);
    *model += *subdef;
  }

  // copy input frame
  sim_spec_t* origin = model->FindSpec(other.compiler);
  sim_builder_frame_t* newframe(model->deepcopy_ ? new sim_builder_frame_t(other) : (sim_builder_frame_t*)&other);
  frames.push_back(newframe);
  frames.back()->body = this;
  frames.back()->model = model;
  frames.back()->compiler = origin ? &origin->compiler : &model->spec.compiler;
  frames.back()->frame = other.frame;
  if (model->deepcopy_) {
    frames.back()->NameSpace(other_model);
  } else {
    frames.back()->AddRef();
  }
  int i = frames.size();
  last_attached = &frames.back()->spec;

  // map input frames to index in this->frames
  std::map<sim_builder_frame_t*, int> fmap;
  for (auto frame : subtree->frames) {
    if (frame == static_cast<const sim_builder_frame_t*>(&other)) {
      fmap[frame] = frames.size() - 1;
    } else if (other.IsAncestor(frame)) {
      fmap[frame] = i++;
    }
  }

  // copy children that are inside the input frame
  CopyList(frames, subtree->frames, fmap, &other);  // needs to be done first
  CopyList(geoms, subtree->geoms, fmap, &other);
  CopyList(joints, subtree->joints, fmap, &other);
  CopyList(sites, subtree->sites, fmap, &other);
  CopyList(cameras, subtree->cameras, fmap, &other);
  CopyList(lights, subtree->lights, fmap, &other);

  if (!model->deepcopy_) {
    std::string name = subtree->name;
    subtree->SetModel(model);
    subtree->NameSpace(other_model);
    subtree->name = name;
  }

  int nbodies = (int)subtree->bodies.size();
  for (int i=0; i < nbodies; i++) {
    if (!other.IsAncestor(subtree->bodies[i]->frame)) {
      continue;
    }
    if (model->deepcopy_) {
      sim_builder_body_t* newbody(new sim_builder_body_t(*subtree->bodies[i], model));  // triggers recursive call
      bodies.push_back(newbody);
      subtree->bodies[i]->ForgetKeyframes();
      bodies.back()->NameSpace_(other_model, /*propagate=*/ false);
    } else {
      bodies.push_back(subtree->bodies[i]);
      bodies.back()->SetModel(model);
      bodies.back()->ResetId();
      bodies.back()->AddRef();
    }
    bodies.back()->parent = this;
    bodies.back()->frame =
      subtree->bodies[i]->frame ? frames[fmap[subtree->bodies[i]->frame]] : nullptr;
  }

  // attach referencing elements
  other_model->SetAttached(model->deepcopy_);
  *model += *other_model;

  // leave the source model in a clean state
  if (other_model != model) {
    other_model->key_pending_.clear();
  }

  // clear namespace and return body
  other_model->prefix.clear();
  other_model->suffix.clear();
  return *this;
}



// copy src list of elements into dst; set body, model and frame
template <typename T>
void sim_builder_body_t::CopyList(std::vector<T*>& dst, const std::vector<T*>& src,
                       std::map<sim_builder_frame_t*, int>& fmap, const sim_builder_frame_t* pframe) {
  int nsrc = (int)src.size();
  int ndst = (int)dst.size();
  for (int i=0; i < nsrc; i++) {
    if (pframe && !pframe->IsAncestor(src[i]->frame)) {
      continue;  // skip if the element is not inside pframe
    }
    sim_spec_t* origin = model->FindSpec(src[i]->compiler);
    T* new_obj = model->deepcopy_ ? new T(*src[i]) : src[i];
    dst.push_back(new_obj);
    dst.back()->body = this;
    dst.back()->model = model;
    dst.back()->compiler = origin ? &origin->compiler : &model->spec.compiler;
    dst.back()->id = -1;
    dst.back()->CopyPlugin();
    dst.back()->classname = src[i]->classname;

    // increment refcount if shallow copy is made
    if (!model->deepcopy_) {
      dst.back()->AddRef();
    }

    // set namespace
    dst.back()->NameSpace(src[i]->model);
  }

  // assign dst frame to src frame
  // needs to be done after the copy in case T is an sim_builder_frame_t
  int j = 0;
  for (int i = 0; i < src.size(); i++) {
    if (pframe && !pframe->IsAncestor(src[i]->frame)) {
      continue;  // skip if the element is not inside pframe
    }
    dst[ndst + j++]->frame = src[i]->frame ? frames[fmap[src[i]->frame]] : nullptr;
  }
}



// find and remove subtree
sim_builder_body_t& sim_builder_body_t::operator-=(const sim_builder_body_t& subtree) {
  for (int i=0; i < bodies.size(); i++) {
    if (bodies[i] == &subtree) {
      bodies.erase(bodies.begin() + i);
      break;
    }
    *bodies[i] -= subtree;
  }

  return *this;
}



// set model of this body and its subtree
void sim_builder_body_t::SetModel(sim_builder_model_t* _model) {
  model = _model;
  sim_spec_t* origin = _model->FindSpec(compiler);
  compiler = origin ? &origin->compiler : &model->spec.compiler;

  for (auto& body : bodies) {
    body->SetModel(_model);
  }
  for (auto& frame : frames) {
    origin = _model->FindSpec(frame->compiler);
    frame->model = _model;
    frame->compiler = origin ? &origin->compiler : &model->spec.compiler;
  }
  for (auto& geom : geoms) {
    origin = _model->FindSpec(geom->compiler);
    geom->model = _model;
    geom->compiler = origin ? &origin->compiler : &model->spec.compiler;
  }
  for (auto& joint : joints) {
    origin = _model->FindSpec(joint->compiler);
    joint->model = _model;
    joint->compiler = origin ? &origin->compiler : &model->spec.compiler;
  }
  for (auto& site : sites) {
    origin = _model->FindSpec(site->compiler);
    site->model = _model;
    site->compiler = origin ? &origin->compiler : &model->spec.compiler;
  }
  for (auto& camera : cameras) {
    origin = _model->FindSpec(camera->compiler);
    camera->model = _model;
    camera->compiler = origin ? &origin->compiler : &model->spec.compiler;
  }
  for (auto& light : lights) {
    origin = _model->FindSpec(light->compiler);
    light->model = _model;
    light->compiler = origin ? &origin->compiler : &model->spec.compiler;
  }
}



// reset ids of all objects in this body
void sim_builder_body_t::ResetId() {
  id = -1;
  for (auto& body : bodies) {
    body->ResetId();
  }
  for (auto& frame : frames) {
    frame->id = -1;
  }
  for (auto& geom : geoms) {
    geom->id = -1;
  }
  for (auto& joint : joints) {
    joint->id = -1;
    joint->qposadr_ = -1;
    joint->dofadr_ = -1;
  }
  for (auto& site : sites) {
    site->id = -1;
  }
  for (auto& camera : cameras) {
    camera->id = -1;
  }
  for (auto& light : lights) {
    light->id = -1;
  }
}



void sim_builder_body_t::PointToLocal() {
  spec.element = static_cast<sim_spec_element_t*>(this);
  spec.childclass = &classname;
  spec.userdata = &spec_userdata_;
  spec.plugin.plugin_name = &plugin_name;
  spec.plugin.name = (&plugin_instance_name);
  spec.info = &info;
  userdata = nullptr;
}


void sim_builder_body_t::CopyFromSpec() {
  *static_cast<sim_spec_body_t*>(this) = spec;
  userdata_ = spec_userdata_;
  plugin.active = spec.plugin.active;
  plugin.element = spec.plugin.element;
  plugin.plugin_name = spec.plugin.plugin_name;
  plugin.name = spec.plugin.name;
}



void sim_builder_body_t::CopyPlugin() {
  model->CopyExplicitPlugin(this);
}



// destructor
sim_builder_body_t::~sim_builder_body_t() {
  for (int i=0; i < bodies.size(); i++) bodies[i]->Release();
  for (int i=0; i < geoms.size(); i++) geoms[i]->Release();
  for (int i=0; i < frames.size(); i++) frames[i]->Release();
  for (int i=0; i < joints.size(); i++) joints[i]->Release();
  for (int i=0; i < sites.size(); i++) sites[i]->Release();
  for (int i=0; i < cameras.size(); i++) cameras[i]->Release();
  for (int i=0; i < lights.size(); i++) lights[i]->Release();
}



// apply prefix and suffix, propagate to children
void sim_builder_body_t::NameSpace(const sim_builder_model_t* m) {
  NameSpace_(m, true);
}



// apply prefix and suffix, propagate to all descendants or only to child bodies
void sim_builder_body_t::NameSpace_(const sim_builder_model_t* m, bool propagate) {
  sim_builder_base_t::NameSpace(m);
  if (!plugin_instance_name.empty()) {
    plugin_instance_name = m->prefix + plugin_instance_name + m->suffix;
  }

  for (auto& body : bodies) {
    body->prefix = m->prefix;
    body->suffix = m->suffix;
    body->NameSpace_(m, propagate);
  }

  if (!propagate) {
    return;
  }

  for (auto& joint : joints) {
    joint->NameSpace(m);
  }

  for (auto& geom : geoms) {
    geom->NameSpace(m);
  }

  for (auto& site : sites) {
    site->NameSpace(m);
  }

  for (auto& camera : cameras) {
    camera->NameSpace(m);
  }

  for (auto& light : lights) {
    light->NameSpace(m);
  }

  for (auto& frame : frames) {
    frame->NameSpace(m);
  }
}



// create child body and add it to body
sim_builder_body_t* sim_builder_body_t::AddBody(sim_builder_default_t* _def) {
  // create body
  sim_builder_body_t* obj = new sim_builder_body_t(model);

  // handle def recursion (i.e. childclass)
  obj->classname = _def ? _def->name : classname;

  bodies.push_back(obj);

  // recompute lists
  model->ResetTreeLists();
  model->MakeTreeLists();

  obj->parent = this;

  // update signature
  model->spec.element->signature = model->Signature();
  return obj;
}



// create new frame and add it to body
sim_builder_frame_t* sim_builder_body_t::AddFrame(sim_builder_frame_t* _frame) {
  sim_builder_frame_t* obj = new sim_builder_frame_t(model, _frame ? _frame : NULL);
  frames.push_back(obj);
  model->ResetTreeLists();
  model->MakeTreeLists();

  // update signature
  model->spec.element->signature = model->Signature();
  return obj;
}



// create new free joint (no default inheritance) and add it to body
sim_builder_joint_t* sim_builder_body_t::AddFreeJoint() {
  // create free joint, don't inherit from defaults
  sim_builder_joint_t* obj = new sim_builder_joint_t(model, NULL);
  obj->spec.type = SIM_JNT_FREE;

  // set body pointer, add
  obj->body = this;

  joints.push_back(obj);

  // recompute lists
  model->ResetTreeLists();
  model->MakeTreeLists();

  // update signature
  model->spec.element->signature = model->Signature();
  return obj;
}



// create new joint and add it to body
sim_builder_joint_t* sim_builder_body_t::AddJoint(sim_builder_default_t* _def) {
  // create joint
  sim_builder_joint_t* obj = new sim_builder_joint_t(model, _def ? _def : model->def_map[classname]);

  // set body pointer, add
  obj->body = this;

  joints.push_back(obj);

  // recompute lists
  model->ResetTreeLists();
  model->MakeTreeLists();

  // update signature
  model->spec.element->signature = model->Signature();
  return obj;
}



// create new geom and add it to body
sim_builder_geom_t* sim_builder_body_t::AddGeom(sim_builder_default_t* _def) {
  // create geom
  sim_builder_geom_t* obj = new sim_builder_geom_t(model, _def ? _def : model->def_map[classname]);

  //  set body pointer, add
  obj->body = this;

  geoms.push_back(obj);

  // recompute lists
  model->ResetTreeLists();
  model->MakeTreeLists();

  // update signature
  model->spec.element->signature = model->Signature();
  return obj;
}



// create new site and add it to body
sim_builder_site_t* sim_builder_body_t::AddSite(sim_builder_default_t* _def) {
  // create site
  sim_builder_site_t* obj = new sim_builder_site_t(model, _def ? _def : model->def_map[classname]);

  // set body pointer, add
  obj->body = this;

  sites.push_back(obj);

  // recompute lists
  model->ResetTreeLists();
  model->MakeTreeLists();

  // update signature
  model->spec.element->signature = model->Signature();
  return obj;
}



// create new camera and add it to body
SIM_CCamera* sim_builder_body_t::AddCamera(sim_builder_default_t* _def) {
  // create camera
  SIM_CCamera* obj = new SIM_CCamera(model, _def ? _def : model->def_map[classname]);

  // set body pointer, add
  obj->body = this;

  cameras.push_back(obj);

  // recompute lists
  model->ResetTreeLists();
  model->MakeTreeLists();

  // update signature
  model->spec.element->signature = model->Signature();
  return obj;
}



// create new light and add it to body
SIM_CLight* sim_builder_body_t::AddLight(sim_builder_default_t* _def) {
  // create light
  SIM_CLight* obj = new SIM_CLight(model, _def ? _def : model->def_map[classname]);

  // set body pointer, add
  obj->body = this;

  lights.push_back(obj);

  // recompute lists
  model->ResetTreeLists();
  model->MakeTreeLists();

  // update signature
  model->spec.element->signature = model->Signature();
  return obj;
}



// create a frame in the parent body and move all contents of this body into it
sim_builder_frame_t* sim_builder_body_t::ToFrame() {
  sim_builder_frame_t* newframe = parent->AddFrame(frame);
  sim_math_internal_copy_vec(newframe->spec.pos, spec.pos, 3);
  sim_math_internal_copy_vec(newframe->spec.quat, spec.quat, 4);
  if (parent->name != "world" && mass >= SIM_MINVAL) {
    if (!parent->explicitinertial) {
      parent->MakeInertialExplicit();
      sim_math_internal_zerovec(parent->spec.ipos, 3);
      sim_math_internal_zerovec(parent->spec.iquat, 4);
      sim_math_internal_zerovec(parent->spec.inertia, 3);
    }
    parent->AccumulateInertia(&this->spec, &parent->spec);
  }
  MapFrame(parent->bodies, bodies, newframe, parent);
  MapFrame(parent->geoms, geoms, newframe, parent);
  MapFrame(parent->joints, joints, newframe, parent);
  MapFrame(parent->sites, sites, newframe, parent);
  MapFrame(parent->cameras, cameras, newframe, parent);
  MapFrame(parent->lights, lights, newframe, parent);
  MapFrame(parent->frames, frames, newframe, parent);
  parent->bodies.erase(
      std::remove_if(parent->bodies.begin(), parent->bodies.end(),
                     [this](sim_builder_body_t* body) { return body == this; }),
      parent->bodies.end());
  model->ResetTreeLists();
  model->MakeTreeLists();
  model->spec.element->signature = model->Signature();
  return newframe;
}



// get number of objects of specified type
int sim_builder_body_t::NumObjects(sim_obj_t type) {
  switch (type) {
    case SIM_OBJ_BODY:
    case SIM_OBJ_XBODY:
      return (int)bodies.size();
    case SIM_OBJ_JOINT:
      return (int)joints.size();
    case SIM_OBJ_GEOM:
      return (int)geoms.size();
    case SIM_OBJ_SITE:
      return (int)sites.size();
    case SIM_OBJ_CAMERA:
      return (int)cameras.size();
    case SIM_OBJ_LIGHT:
      return (int)lights.size();
    default:
      return 0;
  }
}



// get poiner to specified object
sim_builder_base_t* sim_builder_body_t::GetObject(sim_obj_t type, int i) {
  if (i >= 0 && i < NumObjects(type)) {
    switch (type) {
      case SIM_OBJ_BODY:
      case SIM_OBJ_XBODY:
        return bodies[i];
      case SIM_OBJ_JOINT:
        return joints[i];
      case SIM_OBJ_GEOM:
        return geoms[i];
      case SIM_OBJ_SITE:
        return sites[i];
      case SIM_OBJ_CAMERA:
        return cameras[i];
      case SIM_OBJ_LIGHT:
        return lights[i];
      default:
        return 0;
    }
  }

  return 0;
}



// find object by name in given list
template <class T>
static T* findobject(std::string name, std::vector<T*>& list) {
  for (unsigned int i=0; i < list.size(); i++) {
    if (list[i]->name == name) {
      return list[i];
    }
  }

  return 0;
}



// recursive find by name
sim_builder_base_t* sim_builder_body_t::FindObject(sim_obj_t type, std::string _name, bool recursive) {
  sim_builder_base_t* res = 0;

  // check self: just in case
  if (name == _name) {
    return this;
  }

  // search elements of this body
  if (type == SIM_OBJ_BODY || type == SIM_OBJ_XBODY) {
    res = findobject(_name, bodies);
  } else if (type == SIM_OBJ_JOINT) {
    res = findobject(_name, joints);
  } else if (type == SIM_OBJ_GEOM) {
    res = findobject(_name, geoms);
  } else if (type == SIM_OBJ_SITE) {
    res = findobject(_name, sites);
  } else if (type == SIM_OBJ_CAMERA) {
    res = findobject(_name, cameras);
  } else if (type == SIM_OBJ_LIGHT) {
    res = findobject(_name, lights);
  }

  // found
  if (res) {
    return res;
  }

  // search children
  if (recursive) {
    for (int i=0; i < (int)bodies.size(); i++) {
      if ((res = bodies[i]->FindObject(type, _name, true))) {
        return res;
      }
    }
  }

  // not found
  return res;
}



// get list of a given type
template<>
const std::vector<sim_builder_body_t*>& sim_builder_body_t::GetList<sim_builder_body_t>() const {
  return bodies;
}

template<>
const std::vector<sim_builder_joint_t*>& sim_builder_body_t::GetList<sim_builder_joint_t>() const {
  return joints;
}

template<>
const std::vector<sim_builder_geom_t*>& sim_builder_body_t::GetList<sim_builder_geom_t>() const {
  return geoms;
}

template<>
const std::vector<sim_builder_site_t*>& sim_builder_body_t::GetList<sim_builder_site_t>() const {
  return sites;
}

template<>
const std::vector<SIM_CCamera*>& sim_builder_body_t::GetList<SIM_CCamera>() const {
  return cameras;
}

template<>
const std::vector<SIM_CLight*>& sim_builder_body_t::GetList<SIM_CLight>() const {
  return lights;
}

template<>
const std::vector<sim_builder_frame_t*>& sim_builder_body_t::GetList<sim_builder_frame_t>() const {
  return frames;
}



// gets next child of the same type, recursively depth first if requested
template <class T>
static sim_spec_element_t* GetNext(const sim_builder_body_t* body, const sim_spec_element_t* child,
                           bool* found, bool recursive) {
  std::vector<T*> list = body->GetList<T>();

  for (unsigned int i = 0; i < list.size(); i++) {
    if (*found) {
      return list[i]->spec.element;
    }

    if (list[i]->spec.element == child) {
      *found = true;
    }
  }

  if (!recursive) {
    return nullptr;
  }

  for (auto& other : body->Bodies()) {
    sim_spec_element_t* candidate = GetNext<T>(other, child, found, true);
    if (candidate) {
      return candidate;
    }
  }

  return nullptr;
}



// get next body depth first
static sim_spec_element_t* GetNextBody(const sim_builder_body_t* body, const sim_spec_element_t* child,
                              bool* found, bool recursive) {
  for (auto& other : body->Bodies()) {
    if (*found) {
      return other->spec.element;
    }

    if (other->spec.element == child) {
      *found = true;
    }

    if (!recursive) {
      continue;
    }

    sim_spec_element_t* candidate = GetNextBody(other, child, found, true);
    if (candidate) {
      return candidate;
    }
  }

  return nullptr;
}




// get next child of given type
sim_spec_element_t* sim_builder_body_t::NextChild(const sim_spec_element_t* child, sim_obj_t type, bool recursive) {
  if (type == SIM_OBJ_UNKNOWN) {
    if (!child) {
      throw sim_builder_error_t(this, "child type must be specified if no child element is given");
    } else {
      type = child->elemtype;
    }
  } else if (child && child->elemtype != type) {
    throw sim_builder_error_t(this, "child element is not of requested type");
  }

  sim_spec_element_t* candidate = nullptr;
  bool found = child == nullptr;
  switch (type) {
    case SIM_OBJ_BODY:
    case SIM_OBJ_XBODY:
      candidate = GetNextBody(this, child, &found, recursive);
      break;
    case SIM_OBJ_JOINT:
      candidate = GetNext<sim_builder_joint_t>(this, child, &found, recursive);
      break;
    case SIM_OBJ_GEOM:
      candidate = GetNext<sim_builder_geom_t>(this, child, &found, recursive);
      break;
    case SIM_OBJ_SITE:
      candidate = GetNext<sim_builder_site_t>(this, child, &found, recursive);
      break;
    case SIM_OBJ_CAMERA:
      candidate = GetNext<SIM_CCamera>(this, child, &found, recursive);
      break;
    case SIM_OBJ_LIGHT:
      candidate = GetNext<SIM_CLight>(this, child, &found, recursive);
      break;
    case SIM_OBJ_FRAME:
      candidate = GetNext<sim_builder_frame_t>(this, child, &found, recursive);
      break;
    default:
      throw sim_builder_error_t(this,
                     "Body.NextChild supports the types: body, frame, geom, "
                     "site, light, camera");
      break;
  }

  return candidate;
}



// compute geom inertial frame: ipos, iquat, mass, inertia
void sim_builder_body_t::InertiaFromGeom(void) {
  int sz;
  double com[3] = {0, 0, 0};
  double toti[6] = {0, 0, 0, 0, 0, 0};
  std::vector<sim_builder_geom_t*> sel;

  // select geoms based on group, ignore tiny masses
  sel.clear();
  for (int i=0; i < geoms.size(); i++) {
    if (geoms[i]->group >= compiler->inertiagrouprange[0] &&
        geoms[i]->group <= compiler->inertiagrouprange[1] &&
        geoms[i]->mass_ > SIM_EPS) {
      sel.push_back(geoms[i]);
    }
  }
  sz = sel.size();

  // single geom: copy
  if (sz == 1) {
    sim_math_internal_copy_vec(ipos, sel[0]->pos, 3);
    sim_math_internal_copy_vec(iquat, sel[0]->quat, 4);
    mass = sel[0]->mass_;
    sim_math_internal_copy_vec(inertia, sel[0]->inertia, 3);
  }

  // multiple geoms
  else if (sz > 1) {
    // compute total mass and center of mass
    mass = 0;
    for (int i=0; i < sz; i++) {
      mass += sel[i]->mass_;
      com[0] += sel[i]->mass_ * sel[i]->pos[0];
      com[1] += sel[i]->mass_ * sel[i]->pos[1];
      com[2] += sel[i]->mass_ * sel[i]->pos[2];
    }

    // check for small mass
    if (mass < SIM_EPS) {
      throw sim_builder_error_t(this, "body mass is too small, cannot compute center of mass");
    }

    // ipos = geom com
    ipos[0] = com[0]/mass;
    ipos[1] = com[1]/mass;
    ipos[2] = com[2]/mass;

    // add geom inertias
    for (int i=0; i < sz; i++) {
      double inert0[6], inert1[6];
      double dpos[3] = {
        sel[i]->pos[0] - ipos[0],
        sel[i]->pos[1] - ipos[1],
        sel[i]->pos[2] - ipos[2]
      };

      sim_math_internal_globalinertia(inert0, sel[i]->inertia, sel[i]->quat);
      sim_math_internal_offcenter(inert1, sel[i]->mass_, dpos);
      for (int j=0; j < 6; j++) {
        toti[j] = toti[j] + inert0[j] + inert1[j];
      }
    }

    // compute principal axes of inertia
    sim_math_internal_copy_vec(fullinertia, toti, 6);
    const char* errq = sim_math_internal_fullInertia(iquat, inertia, fullinertia);
    if (errq) {
      throw sim_builder_error_t(this, "error '%s' in alternative for principal axes", errq);
    }
  }
}



// set explicitinertial to true
void sim_builder_body_t::MakeInertialExplicit() {
  spec.explicitinertial = true;
}



// accumulate inertia of another body into this body
void sim_builder_body_t::AccumulateInertia(const sim_spec_body_t* other, sim_spec_body_t* result) {
  if (!result) {
    result = this;  // use the private sim_spec_body_t
  }

  // body_ipose = body_pose * body_ipose
  double other_ipos[3];
  double other_iquat[4];
  sim_math_internal_copy_vec(other_ipos, other->ipos, 3);
  sim_math_internal_copy_vec(other_iquat, other->iquat, 4);
  sim_math_internal_frameaccum(other_ipos, other_iquat, other->pos, other->quat);

  // organize data
  double mass[2] = {
    result->mass,
    other->mass
  };
  double inertia[2][3] = {
    {result->inertia[0], result->inertia[1], result->inertia[2]},
    {other->inertia[0], other->inertia[1], other->inertia[2]}
  };
  double ipos[2][3] = {
    {result->ipos[0], result->ipos[1], result->ipos[2]},
    {other_ipos[0], other_ipos[1], other_ipos[2]}
  };
  double iquat[2][4] = {
    {result->iquat[0], result->iquat[1], result->iquat[2], result->iquat[3]},
    {other->iquat[0], other->iquat[1], other->iquat[2], other->iquat[3]}
  };

  // compute total mass
  result->mass = 0;
  sim_math_internal_set_vec(result->ipos, 0, 0, 0);
  for (int j=0; j < 2; j++) {
    result->mass += mass[j];
    result->ipos[0] += mass[j]*ipos[j][0];
    result->ipos[1] += mass[j]*ipos[j][1];
    result->ipos[2] += mass[j]*ipos[j][2];
  }

  // small mass: allow for now, check for errors later
  if (result->mass < SIM_MINVAL) {
    result->mass = 0;
    sim_math_internal_set_vec(result->inertia, 0, 0, 0);
    sim_math_internal_set_vec(result->ipos, 0, 0, 0);
    sim_math_internal_set_vec(result->iquat, 1, 0, 0, 0);
  }

  // proceed with regular computation
  else {
    // locipos = center-of-mass
    result->ipos[0] /= result->mass;
    result->ipos[1] /= result->mass;
    result->ipos[2] /= result->mass;

    // add inertias
    double toti[6] = {0, 0, 0, 0, 0, 0};
    for (int j=0; j < 2; j++) {
      double inertA[6], inertB[6];
      double dpos[3] = {
        ipos[j][0] - result->ipos[0],
        ipos[j][1] - result->ipos[1],
        ipos[j][2] - result->ipos[2]
      };

      sim_math_internal_globalinertia(inertA, inertia[j], iquat[j]);
      sim_math_internal_offcenter(inertB, mass[j], dpos);
      for (int k=0; k < 6; k++) {
        toti[k] += inertA[k] + inertB[k];
      }
    }

    // compute principal axes of inertia
    sim_math_internal_copy_vec(result->fullinertia, toti, 6);
    const char* err1 = sim_math_internal_fullInertia(result->iquat, result->inertia, result->fullinertia);
    if (err1) {
      throw sim_builder_error_t(nullptr, "error '%s' in fusing static body inertias", err1);
    }
  }
}



// compute bounding volume hierarchy
void sim_builder_body_t::ComputeBVH() {
  if (geoms.empty()) {
    return;
  }

  tree.Set(ipos, iquat);
  tree.AllocateBoundingVolumes(geoms.size());
  for (const sim_builder_geom_t* geom : geoms) {
    tree.AddBoundingVolume(&geom->id, geom->contype, geom->conaffinity,
                           geom->pos, geom->quat, geom->aabb);
  }
  tree.CreateBVH();
}



// reset keyframe references for allowing self-attach
void sim_builder_body_t::ForgetKeyframes() const {
  for (auto joint : joints) {
    joint->qpos_.clear();
    joint->qvel_.clear();
  }
  ((sim_builder_body_t*)this)->mpos_.clear();
  ((sim_builder_body_t*)this)->mquat_.clear();
  for (auto body : bodies) {
    body->ForgetKeyframes();
  }
}



sim_scalar_t* sim_builder_body_t::mpos(const std::string& state_name) {
  if (mpos_.find(state_name) == mpos_.end()) {
    mpos_[state_name] = {SIM_NAN, 0, 0};
  }
  return mpos_.at(state_name).data();
}



sim_scalar_t* sim_builder_body_t::mquat(const std::string& state_name) {
  if (mquat_.find(state_name) == mquat_.end()) {
    mquat_[state_name] = {SIM_NAN, 0, 0, 0};
  }
  return mquat_.at(state_name).data();
}



// compiler
void sim_builder_body_t::Compile(void) {
  CopyFromSpec();

  // compile all frames
  for (int i=0; i < frames.size(); i++) {
    frames[i]->Compile();
  }

  // resize userdata
  if (userdata_.size() > model->nuser_body) {
    throw sim_builder_error_t(this, "user has more values than nuser_body in body");
  }
  userdata_.resize(model->nuser_body);

  // normalize user-defined quaternions
  sim_math_internal_normvec(quat, 4);
  sim_math_internal_normvec(iquat, 4);

  // set parentid and weldid of children
  for (int i=0; i < bodies.size(); i++) {
    bodies[i]->weldid = (!bodies[i]->joints.empty() ? bodies[i]->id : weldid);
  }

  // check and process orientation alternatives for body
  if (alt.type != SIM_ORIENTATION_QUAT) {
    const char* err = ResolveOrientation(quat, compiler->degree, compiler->eulerseq, alt);
    if (err) {
      throw sim_builder_error_t(this, "error '%s' in frame alternative", err);
    }
  }

  // check orientation alternatives for inertia
  if (sim_math_internal_defined(fullinertia[0]) && ialt.type != SIM_ORIENTATION_QUAT) {
    throw sim_builder_error_t(this, "fullinertia and inertial orientation cannot both be specified");
  }
  if (sim_math_internal_defined(fullinertia[0]) && (inertia[0] || inertia[1] || inertia[2])) {
    throw sim_builder_error_t(this, "fullinertia and diagonal inertia cannot both be specified");
  }

  // process orientation alternatives for inertia
  if (sim_math_internal_defined(fullinertia[0])) {
    const char* err = sim_math_internal_fullInertia(iquat, inertia, this->fullinertia);
    if (err) {
      throw sim_builder_error_t(this, "error '%s' in fullinertia", err);
    }
  }

  if (ialt.type != SIM_ORIENTATION_QUAT) {
    const char* err = ResolveOrientation(iquat, compiler->degree, compiler->eulerseq, ialt);
    if (err) {
      throw sim_builder_error_t(this, "error '%s' in inertia alternative", err);
    }
  }

  // compile all geoms
  for (int i=0; i < geoms.size(); i++) {
    geoms[i]->inferinertia = id > 0 &&
      (!explicitinertial || compiler->inertiafromgeom == SIM_INERTIAFROMGEOM_TRUE) &&
      geoms[i]->spec.group >= compiler->inertiagrouprange[0] &&
      geoms[i]->spec.group <= compiler->inertiagrouprange[1];
    geoms[i]->Compile();
  }

  // set inertial frame from geoms if necessary
  if (id > 0 && (compiler->inertiafromgeom == SIM_INERTIAFROMGEOM_TRUE ||
                 (!sim_math_internal_defined(ipos[0]) && compiler->inertiafromgeom == SIM_INERTIAFROMGEOM_AUTO))) {
    InertiaFromGeom();
  }

  // ipos undefined: copy body frame into inertial
  if (!sim_math_internal_defined(ipos[0])) {
    sim_math_internal_copy_vec(ipos, pos, 3);
    sim_math_internal_copy_vec(iquat, quat, 4);
  }

  // check and correct mass and inertia
  if (id > 0) {
    // fix minimum
    mass = std::max(mass, compiler->boundmass);
    inertia[0] = std::max(inertia[0], compiler->boundinertia);
    inertia[1] = std::max(inertia[1], compiler->boundinertia);
    inertia[2] = std::max(inertia[2], compiler->boundinertia);

    // check for negative values
    if (mass < 0 || inertia[0] < 0 || inertia[1] < 0 ||inertia[2] < 0) {
      throw sim_builder_error_t(this, "mass and inertia cannot be negative");
    }

    // check for non-physical inertia
    if (inertia[0] + inertia[1] < inertia[2] ||
        inertia[0] + inertia[2] < inertia[1] ||
        inertia[1] + inertia[2] < inertia[0]) {
      if (compiler->balanceinertia) {
        inertia[0] = inertia[1] = inertia[2] = (inertia[0] + inertia[1] + inertia[2])/3.0;
      } else {
        throw sim_builder_error_t(this, "inertia must satisfy A + B >= C; use 'balanceinertia' to fix");
      }
    }
  }

  // frame
  if (frame) {
    sim_math_internal_frameaccumChild(frame->pos, frame->quat, pos, quat);
  }

  // accumulate rbound, contype, conaffinity over geoms
  contype = conaffinity = 0;
  margin = 0;
  for (int i=0; i < geoms.size(); i++) {
    contype |= geoms[i]->contype;
    conaffinity |= geoms[i]->conaffinity;
    margin = std::max(margin, geoms[i]->margin);
  }

  // check conditions for free-joint alignment
  bool align_free = (joints.size() == 1                  &&  // only one joint AND
                     joints[0]->spec.type == SIM_JNT_FREE  &&  // it is a free joint AND
                     bodies.empty()                      &&  // no child bodies AND
                     (joints[0]->spec.align == 1 ||          // either joint.align="true"
                      (joints[0]->spec.align == 2        &&  // or (joint.align="auto"
                       compiler->alignfree)));               //     and compiler->align="true")

  // free-joint alignment, phase 1 (this body + child geoms)
  double ipos_inverse[3], iquat_inverse[4];
  if (align_free) {
    // accumulate iframe transformation to body frame
    sim_math_internal_frameaccum(pos, quat, ipos, iquat);

    // compute inverse iframe transformation
    sim_math_internal_frameinvert(ipos_inverse, iquat_inverse, ipos, iquat);

    // save iframe, set it to null
    sim_math_internal_set_vec(ipos, 0, 0, 0);
    sim_math_internal_set_vec(iquat, 1, 0, 0, 0);

    // apply inverse iframe transformation to all child geoms
    for (int i=0; i < geoms.size(); i++) {
      sim_math_internal_frameaccumChild(ipos_inverse, iquat_inverse, geoms[i]->pos, geoms[i]->quat);
    }
  }

  // compute bounding volume hierarchy
  ComputeBVH();

  // compile all joints, count dofs
  dofnum = 0;
  for (int i=0; i < joints.size(); i++) {
    dofnum += joints[i]->Compile();
  }

  // check for excessive number of dofs
  if (dofnum > 6) {
    throw sim_builder_error_t(this, "more than 6 dofs in body '%s'", name.c_str());
  }

  // check for rotation dof after ball joint
  bool hasball = false;
  for (int i=0; i < joints.size(); i++) {
    if ((joints[i]->type == SIM_JNT_BALL || joints[i]->type == SIM_JNT_HINGE) && hasball) {
      throw sim_builder_error_t(this, "ball followed by rotation in body '%s'", name.c_str());
    }
    if (joints[i]->type == SIM_JNT_BALL) {
      hasball = true;
    }
  }

  // make sure mocap body is fixed child of world
  if (mocap && (dofnum || (parent && parent->name != "world"))) {
    throw sim_builder_error_t(this, "mocap body '%s' is not a fixed child of world", name.c_str());
  }

  // compute body global pose (no joint transformations in qpos0)
  if (id > 0) {
    sim_math_internal_rotVecQuat(xpos0, pos, parent->xquat0);
    sim_math_internal_addtovec(xpos0, parent->xpos0, 3);
    sim_math_internal_mulquat(xquat0, parent->xquat0, quat);
  }

  // compile all sites
  for (int i=0; i < sites.size(); i++)sites[i]->Compile();

  // compile all cameras
  for (int i=0; i < cameras.size(); i++)cameras[i]->Compile();

  // compile all lights
  for (int i=0; i < lights.size(); i++)lights[i]->Compile();

  // plugin
  if (plugin.active) {
    if (plugin_name.empty() && plugin_instance_name.empty()) {
      throw sim_builder_error_t(
              this, "neither 'plugin' nor 'instance' is specified for body '%s', (id = %d)",
              name.c_str(), id);
    }

    sim_builder_plugin_t* plugin_instance = static_cast<sim_builder_plugin_t*>(plugin.element);
    model->ResolvePlugin(this, plugin_name, plugin_instance_name, &plugin_instance);
    plugin.element = plugin_instance;
    const SIM_pPlugin* pplugin = sim_plugin_getPluginAtSlot(plugin_instance->plugin_slot);
    if (!(pplugin->capabilityflags & SIM_PLUGIN_PASSIVE)) {
      throw sim_builder_error_t(this, "plugin '%s' does not support passive forces", pplugin->name);
    }
  }

  // free joint alignment, phase 2 (transform sites, cameras and lights)
  if (align_free) {
    // frames have already been compiled and applied to children

    // sites
    for (int i=0; i < sites.size(); i++) {
      sim_math_internal_frameaccumChild(ipos_inverse, iquat_inverse, sites[i]->pos, sites[i]->quat);
    }

    // cameras
    for (int i=0; i < cameras.size(); i++) {
      sim_math_internal_frameaccumChild(ipos_inverse, iquat_inverse, cameras[i]->pos, cameras[i]->quat);
    }

    // lights
    for (int i=0; i < lights.size(); i++) {
      double qunit[4]= {1, 0, 0, 0};
      sim_math_internal_frameaccumChild(ipos_inverse, iquat_inverse, lights[i]->pos, qunit);
      sim_math_internal_rotVecQuat(lights[i]->dir, lights[i]->dir, iquat_inverse);
    }
  }
}



//------------------ class sim_builder_frame_t implementation -------------------------------------------------

// initialize frame
sim_builder_frame_t::sim_builder_frame_t(sim_builder_model_t* _model, sim_builder_frame_t* _frame) {
  sim_spec_defaultFrame(&spec);
  elemtype = SIM_OBJ_FRAME;
  compiled = false;
  model = _model;
  if (_model) compiler = &_model->spec.compiler;
  body = NULL;
  frame = _frame ? _frame : NULL;
  last_attached = nullptr;
  PointToLocal();
  CopyFromSpec();
}



sim_builder_frame_t::sim_builder_frame_t(const sim_builder_frame_t& other) {
  *this = other;
}



sim_builder_frame_t& sim_builder_frame_t::operator=(const sim_builder_frame_t& other) {
  if (this != &other) {
    this->spec = other.spec;
    *static_cast<SIM_CFrame_*>(this) = static_cast<const SIM_CFrame_&>(other);
    *static_cast<SIM_sFrame*>(this) = static_cast<const SIM_sFrame&>(other);
  }
  PointToLocal();
  return *this;
}



// attach body to frame
sim_builder_frame_t& sim_builder_frame_t::operator+=(const sim_builder_body_t& other) {
  // append a copy of the attached spec
  if (other.model != model && !model->FindSpec(&other.model->spec.compiler)) {
    model->AppendSpec(&other.model->spec, &other.model->spec.compiler);
    static_cast<sim_builder_model_t*>(other.model->spec.element)->AddRef();
  }

  // apply namespace and store keyframes in the source model
  other.model->prefix = other.prefix;
  other.model->suffix = other.suffix;
  other.model->StoreKeyframes(model);
  other.model->prefix = "";
  other.model->suffix = "";
  sim_builder_model_t* other_model = other.model;

  // attach or copy the subtree
  sim_builder_body_t* subtree = model->deepcopy_ ? new sim_builder_body_t(other, model) : (sim_builder_body_t*)&other;
  if (model->deepcopy_) {
    other.ForgetKeyframes();
  } else {
    subtree->SetModel(model);
    subtree->ResetId();
    subtree->AddRef();
  }
  other_model->prefix = subtree->prefix;
  other_model->suffix = subtree->suffix;
  subtree->SetParent(body);
  subtree->SetFrame(this);
  subtree->NameSpace(other_model);

  // attach defaults
  if (other_model != model) {
    sim_builder_default_t* subdef = new sim_builder_default_t(*other_model->Default());
    subdef->NameSpace(other_model);
    *model += *subdef;
  }

  // add to body children
  body->bodies.push_back(subtree);
  last_attached = &body->bodies.back()->spec;

  // attach referencing elements
  other_model->SetAttached(model->deepcopy_);
  *model += *other_model;

  // leave the source model in a clean state
  if (other_model != model) {
    other_model->key_pending_.clear();
  }

  // clear suffixes and return
  other_model->suffix.clear();
  other_model->prefix.clear();
  return *this;
}



// return true if child is descendent of this frame
bool sim_builder_frame_t::IsAncestor(const sim_builder_frame_t* child) const {
  if (!child) {
    return false;
  }

  if (child == this) {
    return true;
  }

  return IsAncestor(child->frame);
}



void sim_builder_frame_t::PointToLocal() {
  spec.element = static_cast<sim_spec_element_t*>(this);
  spec.childclass = &classname;
  spec.info = &info;
}



void sim_builder_frame_t::CopyFromSpec() {
  *static_cast<SIM_sFrame*>(this) = spec;
  sim_math_internal_copy_vec(pos, spec.pos, 3);
  sim_math_internal_copy_vec(quat, spec.quat, 4);
}



void sim_builder_frame_t::Compile() {
  if (compiled) {
    return;
  }

  CopyFromSpec();
  const char* err = ResolveOrientation(quat, compiler->degree, compiler->eulerseq, alt);
  if (err) {
    throw sim_builder_error_t(this, "orientation specification error '%s' in site %d", err, id);
  }

  // compile parents and accumulate result
  if (frame) {
    frame->Compile();
    sim_math_internal_frameaccumChild(frame->pos, frame->quat, pos, quat);
  }

  sim_math_internal_normvec(quat, 4);
  compiled = true;
}



//------------------ class sim_builder_joint_t implementation -------------------------------------------------

// initialize default joint
sim_builder_joint_t::sim_builder_joint_t(sim_builder_model_t* _model, sim_builder_default_t* _def) {
  sim_spec_defaultJoint(&spec);
  elemtype = SIM_OBJ_JOINT;

  // clear internal variables
  spec_userdata_.clear();
  body = 0;

  // reset to default if given
  if (_def) {
    *this = _def->Joint();
  }

  // set model, def
  model = _model;
  if (_model) compiler = &_model->spec.compiler;
  classname = _def ? _def->name : "main";

  // point to local
  PointToLocal();

  // in case this joint is not compiled
  CopyFromSpec();

  // no previous state when a joint is created
  qposadr_ = -1;
  dofadr_ = -1;
}



sim_builder_joint_t::sim_builder_joint_t(const sim_builder_joint_t& other) {
  *this = other;
}



sim_builder_joint_t& sim_builder_joint_t::operator=(const sim_builder_joint_t& other) {
  if (this != &other) {
    this->spec = other.spec;
    *static_cast<SIM_CJoint_*>(this) = static_cast<const SIM_CJoint_&>(other);
    *static_cast<SIM_sJoint*>(this) = static_cast<const SIM_sJoint&>(other);
    qposadr_ = -1;
    dofadr_ = -1;
  }
  PointToLocal();
  return *this;
}



bool sim_builder_joint_t::is_limited() const {
  return islimited(limited, range);
}
bool sim_builder_joint_t::is_actfrclimited() const {
  return islimited(actfrclimited, actfrcrange);
}



int sim_builder_joint_t::nq(SIM_tJoint joint_type) {
  switch (joint_type) {
    case SIM_JNT_FREE:
      return 7;
    case SIM_JNT_BALL:
      return 4;
    case SIM_JNT_SLIDE:
    case SIM_JNT_HINGE:
      return 1;
  }
  return 1;
}



int sim_builder_joint_t::nv(SIM_tJoint joint_type) {
  switch (joint_type) {
    case SIM_JNT_FREE:
      return 6;
    case SIM_JNT_BALL:
      return 3;
    case SIM_JNT_SLIDE:
    case SIM_JNT_HINGE:
      return 1;
  }
  return 1;
}



sim_scalar_t* sim_builder_joint_t::qpos(const std::string& state_name) {
  if (qpos_.find(state_name) == qpos_.end()) {
    qpos_[state_name] = {SIM_NAN, 0, 0, 0, 0, 0, 0};
  }
  return qpos_.at(state_name).data();
}



sim_scalar_t* sim_builder_joint_t::qvel(const std::string& state_name) {
  if (qvel_.find(state_name) == qvel_.end()) {
    qvel_[state_name] = {SIM_NAN, 0, 0, 0, 0, 0};
  }
  return qvel_.at(state_name).data();
}



void sim_builder_joint_t::PointToLocal() {
  spec.element = static_cast<sim_spec_element_t*>(this);
  spec.userdata = &spec_userdata_;
  spec.info = &info;
  userdata = nullptr;
}



void sim_builder_joint_t::CopyFromSpec() {
  *static_cast<SIM_sJoint*>(this) = spec;
  userdata_ = spec_userdata_;
}



// compiler
int sim_builder_joint_t::Compile(void) {
  CopyFromSpec();

  // resize userdata
  if (userdata_.size() > model->nuser_jnt) {
    throw sim_builder_error_t(this, "user has more values than nuser_jnt in joint");
  }
  userdata_.resize(model->nuser_jnt);

  // check springdamper
  if (springdamper[0] || springdamper[1]) {
    if (springdamper[0] <= 0 || springdamper[1] <= 0) {
      throw sim_builder_error_t(this, "when defined, springdamper values must be positive in joint");
    }
  }

  // free joints cannot be limited
  if (type == SIM_JNT_FREE) {
    limited = SIM_LIMITED_FALSE;
  }

  // otherwise if limited is auto, check consistency wrt auto-limits
  else if (limited == SIM_LIMITED_AUTO) {
    bool hasrange = !(range[0] == 0 && range[1] == 0);
    checklimited(this, compiler->autolimits, "joint", "", limited, hasrange);
  }

  // resolve limits
  if (is_limited()) {
    // check data
    if (range[0] >= range[1] && type != SIM_JNT_BALL) {
      throw sim_builder_error_t(this, "range[0] should be smaller than range[1] in joint");
    }
    if (range[0] && type == SIM_JNT_BALL) {
      throw sim_builder_error_t(this, "range[0] should be 0 in ball joint");
    }

    // convert limits to radians
    if (compiler->degree && (type == SIM_JNT_HINGE || type == SIM_JNT_BALL)) {
      if (range[0]) {
        range[0] *= SIM_PI/180.0;
      }
      if (range[1]) {
        range[1] *= SIM_PI/180.0;
      }
    }
  }

  // actuator force range: none for free or ball joints
  if (type == SIM_JNT_FREE || type == SIM_JNT_BALL) {
    actfrclimited = SIM_LIMITED_FALSE;
  }

  // otherwise if actfrclimited is auto, check consistency wrt auto-limits
  else if (actfrclimited == SIM_LIMITED_AUTO) {
    bool hasrange = !(actfrcrange[0] == 0 && actfrcrange[1] == 0);
    checklimited(this, compiler->autolimits, "joint", "", actfrclimited, hasrange);
  }

  // resolve actuator force range limits
  if (is_actfrclimited()) {
    // check data
    if (actfrcrange[0] >= actfrcrange[1]) {
      throw sim_builder_error_t(this, "actfrcrange[0] should be smaller than actfrcrange[1] in joint");
    }
  }

  // axis: FREE or BALL are fixed to (0,0,1)
  if (type == SIM_JNT_FREE || type == SIM_JNT_BALL) {
    axis[0] = axis[1] = 0;
    axis[2] = 1;
  }

  // otherwise accumulate frame rotation
  else if (frame) {
    sim_math_internal_rotVecQuat(axis, axis, frame->quat);
  }

  // normalize axis, check norm
  if (sim_math_internal_normvec(axis, 3) < SIM_EPS) {
    throw sim_builder_error_t(this, "axis too small in joint");
  }

  // check data
  if (type == SIM_JNT_FREE && limited == SIM_LIMITED_TRUE) {
    throw sim_builder_error_t(this, "limits should not be defined in free joint");
  }

  // pos: FREE is fixed to (0,0,0)
  if (type == SIM_JNT_FREE) {
    sim_math_internal_zerovec(pos, 3);
  }

  // otherwise accumulate frame translation
  else if (frame) {
    double qunit[4] = {1, 0, 0, 0};
    sim_math_internal_frameaccumChild(frame->pos, frame->quat, pos, qunit);
  }

  // convert reference angles to radians for hinge joints
  if (type == SIM_JNT_HINGE && compiler->degree) {
    ref *= SIM_PI/180.0;
    springref *= SIM_PI/180.0;
  }

  // return dofnum
  if (type == SIM_JNT_FREE) {
    return 6;
  } else if (type == SIM_JNT_BALL) {
    return 3;
  } else {
    return 1;
  }
}



//------------------ class sim_builder_geom_t implementation --------------------------------------------------

// initialize default geom
sim_builder_geom_t::sim_builder_geom_t(sim_builder_model_t* _model, sim_builder_default_t* _def) {
  sim_spec_defaultGeom(&spec);
  elemtype = SIM_OBJ_GEOM;

  mass_ = 0;
  body = 0;
  matid = -1;
  mesh = nullptr;
  hfield = nullptr;
  visual_ = false;
  sim_math_internal_set_vec(inertia, 0, 0, 0);
  inferinertia = true;
  spec_material_.clear();
  spec_userdata_.clear();
  spec_meshname_.clear();
  spec_hfieldname_.clear();
  spec_userdata_.clear();

  for (int i = 0; i < SIM_NFLUID; i++){
    fluid[i] = 0;
  }

  // reset to default if given
  if (_def) {
    *this = _def->Geom();
  }

  // set model, def
  model = _model;
  if (_model) compiler = &_model->spec.compiler;
  classname = _def ? _def->name : "main";

  // point to local
  PointToLocal();

  // in case this geom is not compiled
  CopyFromSpec();
}



sim_builder_geom_t::sim_builder_geom_t(const sim_builder_geom_t& other) {
  *this = other;
}



sim_builder_geom_t& sim_builder_geom_t::operator=(const sim_builder_geom_t& other) {
  if (this != &other) {
    this->spec = other.spec;
    *static_cast<SIM_CGeom_*>(this) = static_cast<const SIM_CGeom_&>(other);
    *static_cast<sim_spec_geom_t*>(this) = static_cast<const sim_spec_geom_t&>(other);
  }
  PointToLocal();
  return *this;
}



// to be called after any default copy constructor
void sim_builder_geom_t::PointToLocal(void) {
  spec.element = static_cast<sim_spec_element_t*>(this);
  spec.info = &info;
  spec.userdata = &spec_userdata_;
  spec.material = &spec_material_;
  spec.meshname = &spec_meshname_;
  spec.hfieldname = &spec_hfieldname_;
  spec.plugin.plugin_name = &plugin_name;
  spec.plugin.name = &plugin_instance_name;
  userdata = nullptr;
  hfieldname = nullptr;
  meshname = nullptr;
  material = nullptr;
}



void sim_builder_geom_t::CopyFromSpec() {
  *static_cast<sim_spec_geom_t*>(this) = spec;
  userdata_ = spec_userdata_;
  hfieldname_ = spec_hfieldname_;
  meshname_ = spec_meshname_;
  material_ = spec_material_;
  plugin.active = spec.plugin.active;
  plugin.element = spec.plugin.element;
  plugin.plugin_name = spec.plugin.plugin_name;
  plugin.name = spec.plugin.name;
}



void sim_builder_geom_t::CopyPlugin() {
  model->CopyExplicitPlugin(this);
}



void sim_builder_geom_t::NameSpace(const sim_builder_model_t* m) {
  sim_builder_base_t::NameSpace(m);
  if (!spec_material_.empty() && model != m) {
    spec_material_ = m->prefix + spec_material_ + m->suffix;
  }
  if (!spec_hfieldname_.empty() && model != m) {
    spec_hfieldname_ = m->prefix + spec_hfieldname_ + m->suffix;
  }
  if (!spec_meshname_.empty() && model != m) {
    spec_meshname_ = m->prefix + spec_meshname_ + m->suffix;
  }
  if (!plugin_instance_name.empty()) {
    plugin_instance_name = m->prefix + plugin_instance_name + m->suffix;
  }
}



// compute geom volume / surface area
double sim_builder_geom_t::GetVolume() const {
  // get from mesh
  if (type == SIM_GEOM_MESH || type == SIM_GEOM_SDF) {
    if (mesh->id < 0 || !((std::size_t) mesh->id <= model->Meshes().size())) {
      throw sim_builder_error_t(this, "invalid mesh id in mesh geom");
    }

    return mesh->GetVolumeRef();
  }

  // compute from geom shape (type) and inertia type (typeinertia)
  switch (type) {
    case SIM_GEOM_SPHERE: {
      double radius = size[0];
      switch (typeinertia) {
        case SIM_INERTIA_VOLUME:
          return 4 * SIM_PI * radius * radius * radius / 3;
        case SIM_INERTIA_SHELL:
          return 4 * SIM_PI * radius * radius;
      }
      break;
    }
    case SIM_GEOM_CAPSULE: {
      double height = 2 * size[1];
      double radius = size[0];
      switch (typeinertia) {
        case SIM_INERTIA_VOLUME:
          return SIM_PI * (radius * radius * height + 4 * radius * radius * radius / 3);
        case SIM_INERTIA_SHELL:
          return 4 * SIM_PI * radius * radius + 2 * SIM_PI * radius * height;
      }
      break;
    }
    case SIM_GEOM_CYLINDER: {
      double height = 2 * size[1];
      double radius = size[0];
      switch (typeinertia) {
        case SIM_INERTIA_VOLUME:
          return SIM_PI * radius * radius * height;
        case SIM_INERTIA_SHELL:
          return 2 * SIM_PI * radius * radius + 2 * SIM_PI * radius * height;
      }
      break;
    }
    case SIM_GEOM_ELLIPSOID: {
      switch (typeinertia) {
        case SIM_INERTIA_VOLUME:
          return 4 * SIM_PI * size[0] * size[1] * size[2] / 3;
        case SIM_INERTIA_SHELL: {
          // Thomsen approximation
          // https://www.numericana.com/answer/ellipsoid.htm#thomsen
          double p = 1.6075;
          double tmp = std::pow(size[0] * size[1], p) +
                       std::pow(size[1] * size[2], p) +
                       std::pow(size[2] * size[0], p);
          return 4 * SIM_PI * std::pow(tmp / 3, 1 / p);
        }
      }
      break;
    }
    case SIM_GEOM_HFIELD:
    case SIM_GEOM_BOX: {
      switch (typeinertia) {
        case SIM_INERTIA_VOLUME:
          return size[0] * size[1] * size[2] * 8;
        case SIM_INERTIA_SHELL:
          return 8 * (size[0] * size[1] + size[1] * size[2] + size[2] * size[0]);
      }
      break;
    }
    default:
      break;
  }
  return 0;
}



// set geom diagonal inertia given density
void sim_builder_geom_t::SetInertia(void) {
  // get from mesh
  if (type == SIM_GEOM_MESH || type == SIM_GEOM_SDF) {
    if (mesh->id < 0 || !((std::size_t)mesh->id <= model->Meshes().size())) {
      throw sim_builder_error_t(this, "invalid mesh id in mesh geom");
    }

    double* boxsz = mesh->GetInertiaBoxPtr();
    inertia[0] = mass_ * (boxsz[1] * boxsz[1] + boxsz[2] * boxsz[2]) / 3;
    inertia[1] = mass_ * (boxsz[0] * boxsz[0] + boxsz[2] * boxsz[2]) / 3;
    inertia[2] = mass_ * (boxsz[0] * boxsz[0] + boxsz[1] * boxsz[1]) / 3;

    return;
  }

  // compute from geom shape (type) and inertia type (typeinertia)
  switch (type) {
    case SIM_GEOM_SPHERE: {
      switch (typeinertia) {
        case SIM_INERTIA_VOLUME:
          inertia[0] = inertia[1] = inertia[2] = 2 * mass_ * size[0] * size[0] / 5;
          return;
        case SIM_INERTIA_SHELL:
          inertia[0] = inertia[1] = inertia[2] = 2 * mass_ * size[0] * size[0] / 3;
          return;
      }
      break;
    }
    case SIM_GEOM_CAPSULE: {
      double halfheight = size[1];
      double height = 2 * size[1];
      double radius = size[0];
      switch (typeinertia) {
        case SIM_INERTIA_VOLUME: {
          double sphere_mass =
            mass_ * 4 * radius / (4 * radius + 3 * height);    // mass*(sphere_vol/total_vol)
          double cylinder_mass = mass_ - sphere_mass;

          // cylinder part
          inertia[0] = inertia[1] = cylinder_mass * (3 * radius * radius + height * height) / 12;
          inertia[2] = cylinder_mass * radius * radius / 2;

          // add two hemispheres, displace along third axis
          double sphere_inertia = 2 * sphere_mass * radius * radius / 5;
          inertia[0] += sphere_inertia + sphere_mass * height * (3 * radius + 2 * height) / 8;
          inertia[1] += sphere_inertia + sphere_mass * height * (3 * radius + 2 * height) / 8;
          inertia[2] += sphere_inertia;
          return;
        }
        case SIM_INERTIA_SHELL: {
          // surface area
          double Asphere = 4 * SIM_PI * radius * radius;
          double Acylinder = 2 * SIM_PI * radius * height;
          double Atotal = Asphere + Acylinder;

          // mass
          double sphere_mass = mass_ * Asphere / Atotal;  // mass*(sphere_area/total_area)
          double cylinder_mass = mass_ - sphere_mass;

          // cylinder part
          inertia[0] = inertia[1] = cylinder_mass * (6 * radius * radius + height * height) / 12;
          inertia[2] = cylinder_mass * radius * radius;

          // add two hemispheres, displace along third axis
          double sphere_inertia = 2 * sphere_mass * radius * radius / 3;
          double hs_com = radius / 2;           // hemisphere center of mass
          double hs_pos = halfheight + hs_com;  // hemisphere position
          inertia[0] += sphere_inertia + sphere_mass * (hs_pos * hs_pos - hs_com * hs_com);
          inertia[1] += sphere_inertia + sphere_mass * (hs_pos * hs_pos - hs_com * hs_com);
          inertia[2] += sphere_inertia;
          return;
        }
        break;
      }
      break;
    }
    case SIM_GEOM_CYLINDER: {
      double halfheight = size[1];
      double height = 2 * halfheight;
      double radius = size[0];
      switch (typeinertia) {
        case SIM_INERTIA_VOLUME:

          inertia[0] = inertia[1] = mass_ * (3 * radius * radius + height * height) / 12;
          inertia[2] = mass_ * radius * radius / 2;
          return;
        case SIM_INERTIA_SHELL: {
          // surface area
          double Adisk = SIM_PI * radius * radius;
          double Acylinder = 2 * SIM_PI * radius * height;
          double Atotal = 2 * Adisk + Acylinder;

          // mass
          double mass_disk = mass_ * Adisk / Atotal;
          double mass_cylinder = mass_ - 2 * mass_disk;

          // cylinder contribution
          inertia[0] = inertia[1] = mass_cylinder * (6 * radius * radius + height * height) / 12;
          inertia[2] = mass_cylinder * radius * radius;

          // disk inertia
          double inertia_disk_x = mass_disk * radius * radius / 4 +
                                  mass_disk * halfheight * halfheight;
          double inertia_disk_z = mass_disk * radius * radius / 2;

          // top and bottom disk contributions
          inertia[0] += 2 * inertia_disk_x;
          inertia[1] += 2 * inertia_disk_x;
          inertia[2] += 2 * inertia_disk_z;
          return;
        }
      }
      break;
    }
    case SIM_GEOM_ELLIPSOID: {
      double s00 = size[0] * size[0];
      double s11 = size[1] * size[1];
      double s22 = size[2] * size[2];
      switch (typeinertia) {
        case SIM_INERTIA_VOLUME: {
          inertia[0] = mass_ * (s11 + s22) / 5;
          inertia[1] = mass_ * (s00 + s22) / 5;
          inertia[2] = mass_ * (s00 + s11) / 5;
          return;
        }
        case SIM_INERTIA_SHELL: {
          // approximate shell inertia by subtracting ellipsoid from expanded ellipsoid
          double eps = 1e-6;

          // solid volume (a)
          double Va = 4 * SIM_PI * size[0] * size[1] * size[2] / 3;

          // expanded volume (b)
          double ae = size[0] + eps;
          double be = size[1] + eps;
          double ce = size[2] + eps;
          double Vb = 4 * SIM_PI * ae * be * ce / 3;

          // density
          double density = mass_ / (Vb - Va);

          // inertia
          double mass_a = Va * density;
          double inertia_a[3];
          inertia_a[0] = mass_a * (s11 + s22) / 5;
          inertia_a[1] = mass_a * (s00 + s22) / 5;
          inertia_a[2] = mass_a * (s00 + s11) / 5;

          double mass_b = Vb * density;
          double inertia_b[3];
          inertia_b[0] = mass_b * (be * be + ce * ce) / 5;
          inertia_b[1] = mass_b * (ae * ae + ce * ce) / 5;
          inertia_b[2] = mass_b * (ae * ae + be * be) / 5;

          // shell inertia
          inertia[0] = inertia_b[0] - inertia_a[0];
          inertia[1] = inertia_b[1] - inertia_a[1];
          inertia[2] = inertia_b[2] - inertia_a[2];
          return;
        }
      }
      break;
    }
    case SIM_GEOM_HFIELD:
    case SIM_GEOM_BOX: {
      double s00 = size[0] * size[0];
      double s11 = size[1] * size[1];
      double s22 = size[2] * size[2];
      switch (typeinertia) {
        case SIM_INERTIA_VOLUME: {
          inertia[0] = mass_ * (s11 + s22) / 3;
          inertia[1] = mass_ * (s00 + s22) / 3;
          inertia[2] = mass_ * (s00 + s11) / 3;
          return;
        }
        case SIM_INERTIA_SHELL: {
          // length
          double lx = 2 * size[0];  // side 0
          double ly = 2 * size[1];  // side 1
          double lz = 2 * size[2];  // side 2

          // surface area
          double A0 = lx * ly;  // side 0
          double A1 = ly * lz;  // side 1
          double A2 = lz * lx;  // side 2
          double Atotal = 2 * (A0 + A1 + A2);

          // side 0
          double mass0 = mass_ * A0 / Atotal;
          double Ix0 = mass0 * ly * ly / 12;
          double Iy0 = mass0 * lx * lx / 12;
          double Iz0 = mass0 * (lx * lx + ly * ly) / 12;

          // side 1
          double mass1 = mass_ * A1 / Atotal;
          double Ix1 = mass1 * (ly * ly + lz * lz) / 12;
          double Iy1 = mass1 * lz * lz / 12;
          double Iz1 = mass1 * ly * ly / 12;

          // side 3
          double mass2 = mass_ * A2 / Atotal;
          double Ix2 = mass2 * lz * lz / 12;
          double Iy2 = mass2 * (lx * lx + lz * lz) / 12;
          double Iz2 = mass2 * lx * lx / 12;

          // total inertia
          inertia[0] = 2 * (mass0 * s22 + mass2 * s11 + Ix0 + Ix1 + Ix2);
          inertia[1] = 2 * (mass0 * s22 + mass1 * s00 + Iy0 + Iy1 + Iy2);
          inertia[2] = 2 * (mass1 * s00 + mass2 * s11 + Iz0 + Iz1 + Iz2);
          return;
        }
        break;
      }
      break;
    }
    default:
      inertia[0] = inertia[1] = inertia[2] = 0;
      return;
  }
}



// compute radius of bounding sphere
double sim_builder_geom_t::GetRBound(void) {
  const double *aamm, *hsize;
  double haabb[3] = {0};

  switch (type) {
    case SIM_GEOM_HFIELD:
      hsize = hfield->size;
      return sqrt(hsize[0]*hsize[0] + hsize[1]*hsize[1] +
                  std::max(hsize[2]*hsize[2], hsize[3]*hsize[3]));

    case SIM_GEOM_SPHERE:
      return size[0];

    case SIM_GEOM_CAPSULE:
      return size[0]+size[1];

    case SIM_GEOM_CYLINDER:
      return sqrt(size[0]*size[0]+size[1]*size[1]);

    case SIM_GEOM_ELLIPSOID:
      return std::max(std::max(size[0], size[1]), size[2]);

    case SIM_GEOM_BOX:
      return sqrt(size[0]*size[0]+size[1]*size[1]+size[2]*size[2]);

    case SIM_GEOM_MESH:
    case SIM_GEOM_SDF:
      aamm = mesh->aamm();
      haabb[0] = std::max(std::abs(aamm[0]), std::abs(aamm[3]));
      haabb[1] = std::max(std::abs(aamm[1]), std::abs(aamm[4]));
      haabb[2] = std::max(std::abs(aamm[2]), std::abs(aamm[5]));
      return sqrt(haabb[0]*haabb[0] + haabb[1]*haabb[1] + haabb[2]*haabb[2]);

    default:
      return 0;
  }
}



// Compute the coefficients of the added inertia due to the surrounding fluid.
double sim_builder_geom_t::GetAddedMassKappa(double dx, double dy, double dz) {
  // Integration by Gauss鈥揔ronrod quadrature on interval l in [0, infinity] of
  // f(l) = dx*dy*dz / np.sqrt((dx*dx+ l)**3 * (dy*dy+ l) * (dz*dz+ l))
  // 15-point Gauss鈥揔ronrod quadrature (K15) points x in [0, 1].

  // static constexpr sim_scalar_t kronrod_x[15] = [     // unused, left in comment for completeness
  //   0.00427231, 0.02544604, 0.06756779, 0.12923441, 0.20695638,
  //   0.29707742, 0.39610752, 0.50000000, 0.60389248, 0.70292258,
  //   0.79304362, 0.87076559, 0.93243221, 0.97455396, 0.99572769];
  // 15-point Gauss鈥揔ronrod quadrature (K15) weights.
  static constexpr double kronrod_w[15] = {
    0.01146766, 0.03154605, 0.05239501, 0.07032663, 0.08450236,
    0.09517529, 0.10221647, 0.10474107, 0.10221647, 0.09517529,
    0.08450236, 0.07032663, 0.05239501, 0.03154605, 0.01146766};
  // Integrate from 0 to inf by change of variables:
  // l = x^3 / (1-x)^2. Exponents 3 and 2 found to minimize error.
  static constexpr double kronrod_l[15] = {
    7.865151709349917e-08, 1.7347976913907274e-05, 0.0003548008144506193,
    0.002846636252924549, 0.014094260903596077, 0.053063261727396636,
    0.17041978741317773, 0.5, 1.4036301548686991, 3.9353484827022642,
    11.644841677041734, 39.53187807410903, 177.5711362220801,
    1429.4772912937397, 54087.416549217705};
  // dl = dl/dx dx. The following are dl/dx(x).
  static constexpr double kronrod_d[15] = {
    5.538677720489877e-05, 0.002080868285293228, 0.016514126520723166,
    0.07261900344370877, 0.23985243401862602, 0.6868318249020725,
    1.8551129519182894, 5.0, 14.060031152313941, 43.28941239611009,
    156.58546376397112, 747.9826085305024, 5827.4042950027115,
    116754.0197944512, 25482945.327264845};

  const double invdx2 = 1.0 / (dx * dx);
  const double invdy2 = 1.0 / (dy * dy);
  const double invdz2 = 1.0 / (dz * dz);

  // for added numerical stability we non-dimensionalize x by scale
  // because 1 + l/d^2 in denom, l should be scaled by d^2
  const double scale = std::pow(dx*dx*dx * dy * dz, 0.4);  // ** (2/5)
  double kappa = 0.0;
  for (int i = 0; i < 15; ++i) {
    const double lambda = scale * kronrod_l[i];
    const double denom = (1 + lambda*invdx2) * std::sqrt(
      (1 + lambda*invdx2) * (1 + lambda*invdy2) * (1 + lambda*invdz2));
    kappa += scale * kronrod_d[i] / denom * kronrod_w[i];
  }
  return kappa * invdx2;
}



// Compute the kappa coefs of the added inertia due to the surrounding fluid.
void sim_builder_geom_t::SetFluidCoefs(void) {
  double dx, dy, dz;

  // get semiaxes
  switch (type) {
    case SIM_GEOM_SPHERE:
      dx = size[0];
      dy = size[0];
      dz = size[0];
      break;

    case SIM_GEOM_CAPSULE:
      dx = size[0];
      dy = size[0];
      dz = size[1] + size[0];
      break;

    case SIM_GEOM_CYLINDER:
      dx = size[0];
      dy = size[0];
      dz = size[1];
      break;

    default:
      dx = size[0];
      dy = size[1];
      dz = size[2];
  }

  // volume of equivalent ellipsoid
  const double volume = 4.0 / 3.0 * SIM_PI * dx * dy * dz;

  // GetAddedMassKappa is invariant to permutation of last two arguments
  const double kx = GetAddedMassKappa(dx, dy, dz);
  const double ky = GetAddedMassKappa(dy, dz, dx);
  const double kz = GetAddedMassKappa(dz, dx, dy);

  // coefficients of virtual moment of inertia. Note: if (kz-ky) in numerator
  // is negative, also the denom is negative. Abs both and clip to MINVAL
  const auto pow2 = [](const double val) { return val * val; };
  const double Ixfac = pow2(dy*dy - dz*dz) * std::abs(kz - ky) / std::max(
    SIM_EPS, std::abs(2*(dy*dy - dz*dz) + (dy*dy + dz*dz)*(ky - kz)));
  const double Iyfac = pow2(dz*dz - dx*dx) * std::abs(kx - kz) / std::max(
    SIM_EPS, std::abs(2*(dz*dz - dx*dx) + (dz*dz + dx*dx)*(kz - kx)));
  const double Izfac = pow2(dx*dx - dy*dy) * std::abs(ky - kx) / std::max(
    SIM_EPS, std::abs(2*(dx*dx - dy*dy) + (dx*dx + dy*dy)*(kx - ky)));

  sim_scalar_t virtual_mass[3];
  virtual_mass[0] = volume * kx / std::max(SIM_EPS, 2-kx);
  virtual_mass[1] = volume * ky / std::max(SIM_EPS, 2-ky);
  virtual_mass[2] = volume * kz / std::max(SIM_EPS, 2-kz);
  sim_scalar_t virtual_inertia[3];
  virtual_inertia[0] = volume*Ixfac/5;
  virtual_inertia[1] = volume*Iyfac/5;
  virtual_inertia[2] = volume*Izfac/5;

  writeFluidGeomInteraction(fluid, &fluid_ellipsoid, &fluid_coefs[0],
                            &fluid_coefs[1], &fluid_coefs[2],
                            &fluid_coefs[3], &fluid_coefs[4],
                            virtual_mass, virtual_inertia);
}


// compute bounding box
void sim_builder_geom_t::ComputeAABB(void) {
  double aamm[6]; // axis-aligned bounding box in (min, max) format
  switch (type) {
    case SIM_GEOM_HFIELD:
      aamm[0] = -hfield->size[0];
      aamm[1] = -hfield->size[1];
      aamm[2] = -hfield->size[3];
      aamm[3] = hfield->size[0];
      aamm[4] = hfield->size[1];
      aamm[5] = hfield->size[2];
      break;

    case SIM_GEOM_SPHERE:
      aamm[3] = aamm[4] = aamm[5] = size[0];
      sim_math_internal_set_vec(aamm, -aamm[3], -aamm[4], -aamm[5]);
      break;

    case SIM_GEOM_CAPSULE:
      aamm[3] = aamm[4] = size[0];
      aamm[5] = size[0] + size[1];
      sim_math_internal_set_vec(aamm, -aamm[3], -aamm[4], -aamm[5]);
      break;

    case SIM_GEOM_CYLINDER:
      aamm[3] = aamm[4] = size[0];
      aamm[5] = size[1];
      sim_math_internal_set_vec(aamm, -aamm[3], -aamm[4], -aamm[5]);
      break;

    case SIM_GEOM_MESH:
    case SIM_GEOM_SDF:
      sim_math_internal_copy_vec(aamm, mesh->aamm(), 6);
      break;

    case SIM_GEOM_PLANE:
      aamm[0] = aamm[1] = aamm[2] = -SIM_MAXVAL;
      aamm[3] = aamm[4] = SIM_MAXVAL;
      aamm[5] = 0;
      break;

    default:
      sim_math_internal_copy_vec(aamm+3, size, 3);
      sim_math_internal_set_vec(aamm, -size[0], -size[1], -size[2]);
      break;
  }

  // convert aamm to aabb (center, size) format
  double pos[] = {(aamm[3] + aamm[0]) / 2, (aamm[4] + aamm[1]) / 2,
                  (aamm[5] + aamm[2]) / 2};
  double size[] = {(aamm[3] - aamm[0]) / 2, (aamm[4] - aamm[1]) / 2,
                   (aamm[5] - aamm[2]) / 2};
  sim_math_internal_copy_vec(aabb, pos, 3);
  sim_math_internal_copy_vec(aabb+3, size, 3);
}

const std::string& sim_builder_geom_t::get_material() const {
  if (mesh && spec_material_.empty()) {
    return mesh->Material();
  }
  return spec_material_;
}

// compiler
void sim_builder_geom_t::Compile(void) {
  CopyFromSpec();

  // resize userdata
  if (userdata_.size() > model->nuser_geom) {
    throw sim_builder_error_t(this, "user has more values than nuser_geom in geom '%s' (id = %d)",
                   name.c_str(), id);
  }
  userdata_.resize(model->nuser_geom);

  // check type
  if (type < 0 || type >= SIM_NGEOMTYPES) {
    throw sim_builder_error_t(this, "invalid type in geom");
  }

  // check condim
  if (condim != 1 && condim != 3 && condim != 4 && condim != 6) {
    throw sim_builder_error_t(this, "invalid condim in geom");
  }

  // check mesh
  if ((type == SIM_GEOM_MESH || type == SIM_GEOM_SDF) && !mesh) {
    throw sim_builder_error_t(this, "mesh geom '%s' (id = %d) must have valid meshid", name.c_str(), id);
  }

  // check hfield
  if ((type == SIM_GEOM_HFIELD && !hfield) || (type != SIM_GEOM_HFIELD && hfield)) {
    throw sim_builder_error_t(this, "hfield geom '%s' (id = %d) must have valid hfieldid", name.c_str(), id);
  }

  // plane only allowed in static bodies
  if (type == SIM_GEOM_PLANE && body->weldid != 0) {
    throw sim_builder_error_t(this, "plane only allowed in static bodies");
  }

  // check if can collide
  visual_ = !contype && !conaffinity;

  // normalize quaternion
  sim_math_internal_normvec(quat, 4);

  // 'fromto': compute pos, quat, size
  if (sim_math_internal_defined(fromto[0])) {
    // check type
    if (type != SIM_GEOM_CAPSULE &&
        type != SIM_GEOM_CYLINDER &&
        type != SIM_GEOM_ELLIPSOID &&
        type != SIM_GEOM_BOX) {
      throw sim_builder_error_t(this, "fromto requires capsule, cylinder, box or ellipsoid in geom");
    }

    // make sure pos is not defined; cannot use sim_math_internal_defined because default is (0,0,0)
    if (pos[0] || pos[1] || pos[2]) {
      throw sim_builder_error_t(this, "both pos and fromto defined in geom");
    }

    // size[1] = length (for capsule and cylinder)
    double vec[3] = {
      fromto[0]-fromto[3],
      fromto[1]-fromto[4],
      fromto[2]-fromto[5]
    };
    size[1] = sim_math_internal_normvec(vec, 3)/2;
    if (size[1] < SIM_EPS) {
      throw sim_builder_error_t(this, "fromto points too close in geom");
    }

    // adjust size for ellipsoid and box
    if (type == SIM_GEOM_ELLIPSOID || type == SIM_GEOM_BOX) {
      size[2] = size[1];
      size[1] = size[0];
    }

    // compute position
    pos[0] = (fromto[0]+fromto[3])/2;
    pos[1] = (fromto[1]+fromto[4])/2;
    pos[2] = (fromto[2]+fromto[5])/2;

    // compute orientation
    sim_math_internal_z2quat(quat, vec);
  }

  // not 'fromto': try alternative
  else {
    const char* err = ResolveOrientation(quat, compiler->degree, compiler->eulerseq, alt);
    if (err) {
      throw sim_builder_error_t(this, "orientation specification error '%s' in geom %d", err, id);
    }
  }

  // mesh: accumulate frame, fit geom if needed
  if (mesh) {
    // check for inapplicable fromto
    if (sim_math_internal_defined(fromto[0])) {
      throw sim_builder_error_t(this, "fromto cannot be used with mesh geom");
    }

    // save reference in case this is not an SIM_GEOM_MESH
    sim_builder_mesh_t* pmesh = mesh;
    double center[3] = {0, 0, 0};

    // fit geom if type is not SIM_GEOM_MESH
    if (type != SIM_GEOM_MESH && type != SIM_GEOM_SDF) {
      mesh->FitGeom(this, center);

      // remove reference to mesh
      meshname_.clear();
      mesh = nullptr;
    } else if (typeinertia == SIM_INERTIA_SHELL) {
      throw sim_builder_error_t(this, "for mesh geoms, inertia should be specified in the mesh asset");
    }

    // rotate center to geom frame and add it to mesh frame
    double meshpos[3];
    sim_math_internal_rotVecQuat(meshpos, center, pmesh->GetQuatPtr());
    sim_math_internal_addtovec(meshpos, pmesh->GetPosPtr(), 3);

    // accumulate mesh frame into geom frame
    sim_math_internal_frameaccum(pos, quat, meshpos, pmesh->GetQuatPtr());
  }

  // check size parameters
  checksize(size, type, this, name.c_str(), id);

  // set hfield sizes in geom.size
  if (type == SIM_GEOM_HFIELD) {
    size[0] = hfield->size[0];
    size[1] = hfield->size[1];
    size[2] = 0.25 * hfield->size[2] + 0.5 * hfield->size[3];
  } else if (type == SIM_GEOM_MESH || type == SIM_GEOM_SDF) {
    const double* aamm = mesh->aamm();
    size[0] = std::max(std::abs(aamm[0]), std::abs(aamm[3]));
    size[1] = std::max(std::abs(aamm[1]), std::abs(aamm[4]));
    size[2] = std::max(std::abs(aamm[2]), std::abs(aamm[5]));
  }

  for (double s : size) {
    if (std::isnan(s)) {
      throw sim_builder_error_t(this, "nan size in geom");
    }
  }
  // compute aabb
  ComputeAABB();

  // compute geom mass and inertia
  if (inferinertia) {
    // mass is defined
    if (sim_math_internal_defined(mass)) {
      if (mass == 0) {
        mass_ = 0;
        density = 0;
      } else if (GetVolume() > SIM_EPS) {
        mass_ = mass;
        density = mass / GetVolume();
        SetInertia();
      }
    }

    // mass is not defined
    else {
      if (density == 0) {
        mass_ = 0;
      } else {
        mass_ = density * GetVolume();
        SetInertia();
      }
    }


    // check for negative values
    if (mass_ < 0 || inertia[0] < 0 || inertia[1] < 0 || inertia[2] < 0 || density < 0)
      throw sim_builder_error_t(this, "mass, inertia or density are negative in geom");
  }

  // fluid-interaction coefficients, requires computed inertia and mass
  if (fluid_ellipsoid > 0) {
    SetFluidCoefs();
  }

  // plugin
  if (plugin.active) {
    if (plugin_name.empty() && plugin_instance_name.empty()) {
      throw sim_builder_error_t(
              this, "neither 'plugin' nor 'instance' is specified for geom");
    }

    sim_builder_plugin_t* plugin_instance = static_cast<sim_builder_plugin_t*>(plugin.element);
    model->ResolvePlugin(this, plugin_name, plugin_instance_name, &plugin_instance);
    plugin.element = plugin_instance;
    const SIM_pPlugin* pplugin = sim_plugin_getPluginAtSlot(plugin_instance->plugin_slot);
    if (!(pplugin->capabilityflags & SIM_PLUGIN_SDF)) {
      throw sim_builder_error_t(this, "plugin '%s' does not support sign distance fields", pplugin->name);
    }
  }

  // frame
  if (frame) {
    sim_math_internal_frameaccumChild(frame->pos, frame->quat, pos, quat);
  }
}



//------------------ class sim_builder_site_t implementation --------------------------------------------------

// initialize default site
sim_builder_site_t::sim_builder_site_t(sim_builder_model_t* _model, sim_builder_default_t* _def) {
  sim_spec_defaultSite(&spec);
  elemtype = SIM_OBJ_SITE;

  // clear internal variables
  body = 0;
  matid = -1;
  spec_material_.clear();
  spec_userdata_.clear();

  // reset to default if given
  if (_def) {
    *this = _def->Site();
  }

  // point to local
  PointToLocal();

  // in case this site is not compiled
  CopyFromSpec();

  // set model, def
  model = _model;
  if (_model) compiler = &_model->spec.compiler;
  classname = _def ? _def->name : "main";
}



sim_builder_site_t::sim_builder_site_t(const sim_builder_site_t& other) {
  *this = other;
}



sim_builder_site_t& sim_builder_site_t::operator=(const sim_builder_site_t& other) {
  if (this != &other) {
    this->spec = other.spec;
    *static_cast<SIM_CSite_*>(this) = static_cast<const SIM_CSite_&>(other);
    *static_cast<SIM_sSite*>(this) = static_cast<const SIM_sSite&>(other);
  }
  PointToLocal();
  return *this;
}



void sim_builder_site_t::PointToLocal() {
  spec.element = static_cast<sim_spec_element_t*>(this);
  spec.info = &info;
  spec.material = &spec_material_;
  spec.userdata = &spec_userdata_;
  userdata = nullptr;
  material = nullptr;
}



void sim_builder_site_t::CopyFromSpec() {
  *static_cast<SIM_sSite*>(this) = spec;
  userdata_ = spec_userdata_;
  material_ = spec_material_;
}



void sim_builder_site_t::NameSpace(const sim_builder_model_t* m) {
  sim_builder_base_t::NameSpace(m);
  if (!spec_material_.empty() && model != m) {
    spec_material_ = m->prefix + spec_material_ + m->suffix;
  }
}



// compiler
void sim_builder_site_t::Compile(void) {
  CopyFromSpec();

  // resize userdata
  if (userdata_.size() > model->nuser_site) {
    throw sim_builder_error_t(this, "user has more values than nuser_site in site");
  }
  userdata_.resize(model->nuser_site);

  // check type
  if (type < 0 || type >= SIM_NGEOMTYPES) {
    throw sim_builder_error_t(this, "invalid type in site");
  }

  // do not allow meshes, hfields and planes
  if (type == SIM_GEOM_MESH || type == SIM_GEOM_HFIELD || type == SIM_GEOM_PLANE) {
    throw sim_builder_error_t(this, "meshes, hfields and planes not allowed in site");
  }

  // 'fromto': compute pos, quat, size
  if (sim_math_internal_defined(fromto[0])) {
    // check type
    if (type != SIM_GEOM_CAPSULE &&
        type != SIM_GEOM_CYLINDER &&
        type != SIM_GEOM_ELLIPSOID &&
        type != SIM_GEOM_BOX) {
      throw sim_builder_error_t(this, "fromto requires capsule, cylinder, box or ellipsoid in geom");
    }

    // make sure pos is not defined; cannot use sim_math_internal_defined because default is (0,0,0)
    if (pos[0] || pos[1] || pos[2]) {
      throw sim_builder_error_t(this, "both pos and fromto defined in geom");
    }

    // size[1] = length (for capsule and cylinder)
    double vec[3] = {fromto[0]-fromto[3], fromto[1]-fromto[4], fromto[2]-fromto[5]};
    size[1] = sim_math_internal_normvec(vec, 3)/2;
    if (size[1] < SIM_EPS) {
      throw sim_builder_error_t(this, "fromto points too close in geom");
    }

    // adjust size for ellipsoid and box
    if (type == SIM_GEOM_ELLIPSOID || type == SIM_GEOM_BOX) {
      size[2] = size[1];
      size[1] = size[0];
    }

    // compute position
    pos[0] = (fromto[0]+fromto[3])/2;
    pos[1] = (fromto[1]+fromto[4])/2;
    pos[2] = (fromto[2]+fromto[5])/2;

    // compute orientation
    sim_math_internal_z2quat(quat, vec);
  }

  // alternative orientation
  else {
    const char* err = ResolveOrientation(quat, compiler->degree, compiler->eulerseq, alt);
    if (err) {
      throw sim_builder_error_t(this, "orientation specification error '%s' in site %d", err, id);
    }
  }

  // frame
  if (frame) {
    sim_math_internal_frameaccumChild(frame->pos, frame->quat, pos, quat);
  }

  // normalize quaternion
  sim_math_internal_normvec(quat, 4);

  // check size parameters
  checksize(size, type, this, name.c_str(), id);
}



//------------------ class SIM_CCamera implementation ------------------------------------------------

// initialize defaults
SIM_CCamera::SIM_CCamera(sim_builder_model_t* _model, sim_builder_default_t* _def) {
  sim_spec_defaultCamera(&spec);
  elemtype = SIM_OBJ_CAMERA;

  // clear private variables
  body = 0;
  targetbodyid = -1;
  spec_targetbody_.clear();

  // reset to default if given
  if (_def) {
    *this = _def->Camera();
  }

  // set model, def
  model = _model;
  if (_model) compiler = &_model->spec.compiler;
  classname = _def ? _def->name : "main";

  // point to local
  PointToLocal();

  // in case this camera is not compiled
  CopyFromSpec();
}



SIM_CCamera::SIM_CCamera(const SIM_CCamera& other) {
  *this = other;
}



SIM_CCamera& SIM_CCamera::operator=(const SIM_CCamera& other) {
  if (this != &other) {
    this->spec = other.spec;
    *static_cast<SIM_CCamera_*>(this) = static_cast<const SIM_CCamera_&>(other);
    *static_cast<SIM_sCamera*>(this) = static_cast<const SIM_sCamera&>(other);
  }
  PointToLocal();
  return *this;
}



void SIM_CCamera::PointToLocal() {
  spec.element = static_cast<sim_spec_element_t*>(this);
  spec.userdata = &spec_userdata_;
  spec.targetbody = &spec_targetbody_;
  spec.info = &info;
  userdata = nullptr;
  targetbody = nullptr;
}



void SIM_CCamera::NameSpace(const sim_builder_model_t* m) {
  sim_builder_base_t::NameSpace(m);
  if (!spec_targetbody_.empty()) {
    spec_targetbody_ = m->prefix + spec_targetbody_ + m->suffix;
  }
}



void SIM_CCamera::CopyFromSpec() {
  *static_cast<SIM_sCamera*>(this) = spec;
  userdata_ = spec_userdata_;
  targetbody_ = spec_targetbody_;
}



void SIM_CCamera::ResolveReferences(const sim_builder_model_t* m) {
  if (!targetbody_.empty()) {
    sim_builder_body_t* tb = (sim_builder_body_t*)m->FindObject(SIM_OBJ_BODY, targetbody_);
    if (tb) {
      targetbodyid = tb->id;
    } else {
      throw sim_builder_error_t(this, "unknown target body in camera");
    }
  }
}



// compiler
void SIM_CCamera::Compile(void) {
  CopyFromSpec();

  // resize userdata
  if (userdata_.size() > model->nuser_cam) {
    throw sim_builder_error_t(this, "user has more values than nuser_cam in camera");
  }
  userdata_.resize(model->nuser_cam);

  // process orientation specifications
  const char* err = ResolveOrientation(quat, compiler->degree, compiler->eulerseq, alt);
  if (err) {
    throw sim_builder_error_t(this, "orientation specification error '%s' in camera %d", err, id);
  }

  // frame
  if (frame) {
    sim_math_internal_frameaccumChild(frame->pos, frame->quat, pos, quat);
  }

  // normalize quaternion
  sim_math_internal_normvec(quat, 4);

  // get targetbodyid
  ResolveReferences(model);

  // make sure the image size is finite
  if (fovy >= 180) {
    throw sim_builder_error_t(this, "fovy too large in camera '%s' (id = %d, value = %d)",
                   name.c_str(), id, fovy);
  }

  // check for advanced camera intrinsic parameters
  bool has_intrinsic = focal_length[0]     || focal_length[1]     ||
                       focal_pixel[0]      || focal_pixel[1]      ||
                       principal_length[0] || principal_length[1] ||
                       principal_pixel[0]  || principal_pixel[1];
  bool has_sensorsize = sensor_size[0] > 0 && sensor_size[1] > 0;

  // intrinsic params require sensorsize
  if (has_intrinsic && !has_sensorsize) {
    throw sim_builder_error_t(this, "focal/principal require sensorsize in camera '%s' (id = %d)",
                   name.c_str(), id);
  }

  // sensorsize requires resolution
  if (has_sensorsize && (resolution[0] <= 0 || resolution[1] <= 0)) {
    throw sim_builder_error_t(this, "sensorsize requires positive resolution in camera '%s' (id = %d)",
                   name.c_str(), id);
  }

  // compute number of pixels per unit length
  if (sensor_size[0] > 0 && sensor_size[1] > 0) {
    float pixel_density[2] = {
      (float)resolution[0] / sensor_size[0],
      (float)resolution[1] / sensor_size[1],
    };

    // pixel values override length values when both are specified
    intrinsic[0] = focal_pixel[0] ? focal_pixel[0] / pixel_density[0] : focal_length[0];
    intrinsic[1] = focal_pixel[1] ? focal_pixel[1] / pixel_density[1] : focal_length[1];
    intrinsic[2] = principal_pixel[0] ? principal_pixel[0] / pixel_density[0] : principal_length[0];
    intrinsic[3] = principal_pixel[1] ? principal_pixel[1] / pixel_density[1] : principal_length[1];

    // fovy with principal point at (0, 0)
    fovy = std::atan2(sensor_size[1]/2, intrinsic[1]) * 360.0 / SIM_PI;
  } else {
    intrinsic[0] = 0.01f;
    intrinsic[1] = 0.01f;
  }
}



//------------------ class SIM_CLight implementation -------------------------------------------------

// initialize defaults
SIM_CLight::SIM_CLight(sim_builder_model_t* _model, sim_builder_default_t* _def) {
  sim_spec_defaultLight(&spec);
  elemtype = SIM_OBJ_LIGHT;

  // clear private variables
  body = 0;
  targetbodyid = -1;
  texid = -1;
  spec_targetbody_.clear();
  spec_texture_.clear();


  // reset to default if given
  if (_def) {
    *this = _def->Light();
  }

  // set model, def
  model = _model;
  if (_model) compiler = &_model->spec.compiler;
  classname = _def ? _def->name : "main";

  PointToLocal();
  CopyFromSpec();
}



SIM_CLight::SIM_CLight(const SIM_CLight& other) {
  *this = other;
}



SIM_CLight& SIM_CLight::operator=(const SIM_CLight& other) {
  if (this != &other) {
    this->spec = other.spec;
    *static_cast<SIM_CLight_*>(this) = static_cast<const SIM_CLight_&>(other);
    *static_cast<SIM_sLight*>(this) = static_cast<const SIM_sLight&>(other);
  }
  PointToLocal();
  return *this;
}



void SIM_CLight::PointToLocal() {
  spec.element = static_cast<sim_spec_element_t*>(this);
  spec.targetbody = &spec_targetbody_;
  spec.texture = &spec_texture_;
  spec.info = &info;
  targetbody = nullptr;
}



void SIM_CLight::NameSpace(const sim_builder_model_t* m) {
  sim_builder_base_t::NameSpace(m);
  if (!spec_targetbody_.empty()) {
    spec_targetbody_ = m->prefix + spec_targetbody_ + m->suffix;
  }
  if (!spec_texture_.empty()) {
    spec_texture_ = m->prefix + spec_texture_ + m->suffix;
  }
}



void SIM_CLight::CopyFromSpec() {
  *static_cast<SIM_sLight*>(this) = spec;
  targetbody_ = spec_targetbody_;
  texture_ = spec_texture_;
}



void SIM_CLight::ResolveReferences(const sim_builder_model_t* m) {
  if (!targetbody_.empty()) {
    sim_builder_body_t* tb = (sim_builder_body_t*)m->FindObject(SIM_OBJ_BODY, targetbody_);
    if (tb) {
      targetbodyid = tb->id;
    } else {
      throw sim_builder_error_t(this, "unknown target body in light");
    }
  }
  if (!texture_.empty()) {
    sim_builder_texture_t* tex = (sim_builder_texture_t*)m->FindObject(SIM_OBJ_TEXTURE, texture_);
    if (tex) {
      texid = tex->id;
    } else {
      throw sim_builder_error_t(this, "unknown texture in light");
    }
  }
}



// compiler
void SIM_CLight::Compile(void) {
  CopyFromSpec();

  // frame
  if (frame) {
    // apply frame transform to pos, qunit is unused
    double qunit[4]= {1, 0, 0, 0};
    sim_math_internal_frameaccumChild(frame->pos, frame->quat, pos, qunit);

    // rotate dir
    sim_math_internal_rotVecQuat(dir, dir, frame->quat);
  }

  // normalize direction, make sure it is not zero
  if (sim_math_internal_normvec(dir, 3) < SIM_EPS) {
    throw sim_builder_error_t(this, "zero direction in light");
  }

  // get targetbodyid and texid
  ResolveReferences(model);
}



//------------------------- class SIM_CHField --------------------------------------------------------

// constructor
SIM_CHField::SIM_CHField(sim_builder_model_t* _model) {
  sim_spec_defaultHField(&spec);
  elemtype = SIM_OBJ_HFIELD;

  // set model pointer
  model = _model;
  if (_model) compiler = &_model->spec.compiler;

  // clear variables
  data.clear();
  spec_file_.clear();
  spec_userdata_.clear();

  // point to local
  PointToLocal();

  // copy from spec
  CopyFromSpec();
}



SIM_CHField::SIM_CHField(const SIM_CHField& other) {
  *this = other;
}



SIM_CHField& SIM_CHField::operator=(const SIM_CHField& other) {
  if (this != &other) {
    this->spec = other.spec;
    *static_cast<SIM_CHField_*>(this) = static_cast<const SIM_CHField_&>(other);
    *static_cast<SIM_sHField*>(this) = static_cast<const SIM_sHField&>(other);
  }
  PointToLocal();
  return *this;
}



void SIM_CHField::PointToLocal() {
  spec.element = static_cast<sim_spec_element_t*>(this);
  spec.file = &spec_file_;
  spec.content_type = &spec_content_type_;
  spec.userdata = &spec_userdata_;
  spec.info = &info;
  file = nullptr;
  content_type = nullptr;
  userdata = nullptr;
}



void SIM_CHField::CopyFromSpec() {
  *static_cast<SIM_sHField*>(this) = spec;
  file_ = spec_file_;
  content_type_ = spec_content_type_;
  userdata_ = spec_userdata_;

  // clear precompiled asset. TODO: use asset cache
  data.clear();
  if (!file_.empty()) {
    nrow = 0;
    ncol = 0;
  }

  // use filename if name is missing
  if (name.empty()) {
    std::string stripped = sim_math_internal_strippath(file_);
    name = sim_math_internal_stripext(stripped);
  }
}



void SIM_CHField::NameSpace(const sim_builder_model_t* m) {
  // use filename if name is missing
  if (name.empty()) {
    std::string stripped = sim_math_internal_strippath(spec_file_);
    name = sim_math_internal_stripext(stripped);
  }
  sim_builder_base_t::NameSpace(m);
  if (modelfiledir_.empty()) {
    modelfiledir_ = FilePath(m->spec_modelfiledir_);
  }
}



// destructor
SIM_CHField::~SIM_CHField() {
  data.clear();
  userdata_.clear();
  spec_userdata_.clear();
}


std::string SIM_CHField::GetCacheId(const SIM_Resource* resource, const std::string& asset_type) {
  std::stringstream ss;
  ss << "SIM_CHField:" << resource->name << ";ARGS:content_type=" << asset_type;
  return ss.str();
}


// load elevation data from custom format
void SIM_CHField::LoadCustom(SIM_Resource* resource) {
  // get file data in buffer
  const void* buffer = 0;
  int buffer_sz = sim_math_readResource(resource, &buffer);

  if (buffer_sz < 1) {
    throw sim_builder_error_t(this, "could not read hfield file '%s'", resource->name);
  } else if (!buffer_sz) {
    throw sim_builder_error_t(this, "empty hfield file '%s'", resource->name);
  }


  if (buffer_sz < 2*sizeof(int)) {
    throw sim_builder_error_t(this, "hfield missing header '%s'", resource->name);
  }

  // read dimensions
  int* pint = (int*)buffer;
  nrow = pint[0];
  ncol = pint[1];

  // check dimensions
  if (nrow < 1 || ncol < 1) {
    throw sim_builder_error_t(this, "non-positive hfield dimensions in file '%s'", resource->name);
  }

  // check buffer size
  if (buffer_sz != nrow*ncol*sizeof(float)+8) {
    throw sim_builder_error_t(this, "unexpected file size in file '%s'", resource->name);
  }

  // allocate
  data.assign(nrow*ncol, 0);
  if (data.empty()) {
    throw sim_builder_error_t(this, "could not allocate buffers in hfield");
  }

  // copy data
  memcpy(data.data(), (void*)(pint+2), nrow*ncol*sizeof(float));
}



// load elevation data from PNG format
void SIM_CHField::LoadPNG(SIM_Resource* resource) {
  PNGImage image = PNGImage::Load(this, resource, LCT_GREY);

  ncol = image.Width();
  nrow = image.Height();

  // copy image data over with rows reversed
  data.reserve(nrow * ncol);
  for (int r = 0; r < nrow; r++) {
    for (int c = 0; c < ncol; c++) {
      data.push_back((float) image[c + (nrow - 1 - r)*ncol]);
    }
  }
}



// compiler
void SIM_CHField::Compile(const SIM_VFS* vfs) {
  CopyFromSpec();

  // copy userdata into data
  if (!userdata_.empty()) {
    if (nrow*ncol != userdata_.size()) {
      throw sim_builder_error_t(this, "elevation data length must match nrow*ncol");
    }
    data.assign(nrow*ncol, 0);
    if (data.empty()) {
      throw sim_builder_error_t(this, "could not allocate buffers in hfield");
    }
    memcpy(data.data(), userdata_.data(), nrow*ncol*sizeof(float));
  }

  // check size parameters
  for (int i=0; i < 4; i++)
    if (size[i] <= 0)
      throw sim_builder_error_t(this, "size parameter is not positive in hfield");

  // remove path from file if necessary
  if (model->strippath) {
    file_ = sim_math_internal_strippath(file_);
  }

  // load from file if specified
  if (!file_.empty()) {
    // make sure hfield was not already specified manually
    if (nrow || ncol || !data.empty()) {
      throw sim_builder_error_t(this, "hfield specified from file and manually");
    }

    SIM_CCache* cache = reinterpret_cast<SIM_CCache*>(sim_getCache()->impl_);

    std::string asset_type = GetAssetContentType(file_, content_type_);

    // fallback to custom
    if (asset_type.empty()) {
      asset_type = "image/vnd.simcore.hfield";
    }

    if (asset_type != "image/png" &&
        asset_type != "image/vnd.simcore.hfield") {
      throw sim_builder_error_t(this, "unsupported content type: '%s'", asset_type.c_str());
    }

    // copy paths from model if not already defined
    if (modelfiledir_.empty()) {
      modelfiledir_ = FilePath(model->modelfiledir_);
    }
    simcore::user::FilePath meshdir_;
    meshdir_ = FilePath(sim_spec_getString(compiler->meshdir));

    FilePath filename = meshdir_ + FilePath(file_);
    SIM_Resource* resource = LoadResource(modelfiledir_.Str(), filename.Str(), vfs);

    struct CachedHField {
      int nrow, ncol;
      std::vector<float> data;
    };

    // cache callback
    auto callback = [&](const void* cached_data) {
      const CachedHField* cached_hfield =
          static_cast<const CachedHField*>(cached_data);
      nrow = cached_hfield->nrow;
      ncol = cached_hfield->ncol;
      this->data = cached_hfield->data;
      return true;
    };

    // try loading from cache
    if (cache && cache->PopulateData(GetCacheId(resource, asset_type), resource, callback)) {
      sim_math_closeResource(resource);
    } else {
      try {
        if (asset_type == "image/png") {
          LoadPNG(resource);
        } else {
          LoadCustom(resource);
        }
      } catch(sim_builder_error_t err) {
        sim_math_closeResource(resource);
        throw err;
      }

      if (cache) {
        CachedHField* cached_hfield = new CachedHField;
        cached_hfield->nrow = nrow;
        cached_hfield->ncol = ncol;
        cached_hfield->data = this->data;
        std::size_t size = sizeof(CachedHField) + this->data.size() * sizeof(float);
        std::shared_ptr<CachedHField> cached_data{cached_hfield};
        cache->Insert("", GetCacheId(resource, asset_type), resource, cached_data, size);
      }
      sim_math_closeResource(resource);
    }
  }

  // make sure hfield was specified (from file or manually)
  if (nrow < 1 || ncol < 1 || data.empty()) {
    throw sim_builder_error_t(this, "hfield not specified");
  }

  // set elevation data to [0-1] range
  float emin = 1E+10, emax = -1E+10;
  for (int i = 0; i < nrow*ncol; i++) {
    emin = std::min(emin, data[i]);
    emax = std::max(emax, data[i]);
  }
  if (emin > emax) {
    throw sim_builder_error_t(this, "invalid data range in hfield '%s'", file_.c_str());
  }
  for (int i=0; i < nrow*ncol; i++) {
    data[i] -= emin;
    if (emax-emin > SIM_EPS) {
      data[i] /= (emax - emin);
    }
  }
}



//------------------ class sim_builder_texture_t implementation -----------------------------------------------

// initialize defaults
sim_builder_texture_t::sim_builder_texture_t(sim_builder_model_t* _model) {
  sim_spec_defaultTexture(&spec);
  elemtype = SIM_OBJ_TEXTURE;

  // set model pointer
  model = _model;
  if (_model) compiler = &_model->spec.compiler;

  // clear user settings: single file
  spec_file_.clear();
  spec_content_type_.clear();

  // clear user settings: separate file
  spec_cubefiles_.assign(6, "");

  // clear internal variables
  data_.clear();
  clear_data_ = false;

  // point to local
  PointToLocal();

  // in case this texture is not compiled
  CopyFromSpec();
}



sim_builder_texture_t::sim_builder_texture_t(const sim_builder_texture_t& other) {
  *this = other;
}



sim_builder_texture_t& sim_builder_texture_t::operator=(const sim_builder_texture_t& other) {
  if (this != &other) {
    this->spec = other.spec;
    *static_cast<SIM_CTexture_*>(this) = static_cast<const SIM_CTexture_&>(other);
    clear_data_ = other.clear_data_;
  }
  PointToLocal();
  return *this;
}



void sim_builder_texture_t::PointToLocal() {
  spec.element = static_cast<sim_spec_element_t*>(this);
  spec.file = &spec_file_;
  spec.data = &data_;
  spec.content_type = &spec_content_type_;
  spec.cubefiles = &spec_cubefiles_;
  spec.info = &info;
  file = nullptr;
  content_type = nullptr;
  cubefiles = nullptr;
}



void sim_builder_texture_t::CopyFromSpec() {
  *static_cast<SIM_sTexture*>(this) = spec;
  file_ = spec_file_;
  content_type_ = spec_content_type_;
  cubefiles_ = spec_cubefiles_;

  if (clear_data_) {
    // clear precompiled asset. TODO: use asset cache
    data_.clear();
  }

  // use filename if name is missing
  if (name.empty()) {
    std::string stripped = sim_math_internal_strippath(file_);
    name = sim_math_internal_stripext(stripped);
  }
}



void sim_builder_texture_t::NameSpace(const sim_builder_model_t* m) {
  // use filename if name is missing
  if (name.empty()) {
    std::string stripped = sim_math_internal_strippath(spec_file_);
    name = sim_math_internal_stripext(stripped);
  }
  sim_builder_base_t::NameSpace(m);
  if (modelfiledir_.empty()) {
    modelfiledir_ = FilePath(m->spec_modelfiledir_);
  }
}



// free data storage allocated by lodepng
sim_builder_texture_t::~sim_builder_texture_t() {
  data_.clear();
}



// insert random dots
static void randomdot(std::byte* rgb, const double* markrgb,
                      int width, int height, double probability) {
  // make distribution using fixed seed
  std::mt19937_64 rng;
  rng.seed(42);
  std::uniform_real_distribution<double> dist(0, 1);

  // sample
  for (int r=0; r < height; r++) {
    for (int c=0; c < width; c++) {
      if (dist(rng) < probability) {
        for (int j=0; j < 3; j++) {
          rgb[3*(r*width+c)+j] = (std::byte)(255*markrgb[j]);
        }
      }
    }
  }
}



// interpolate between colors based on value in (-1, +1)
static void interp(std::byte* rgb, const double* rgb1, const double* rgb2, double pos) {
  const double correction = 1.0/sqrt(2);
  double alpha = 0.5*(1 + pos/sqrt(1+pos*pos)/correction);
  if (alpha < 0) {
    alpha = 0;
  } else if (alpha > 1) {
    alpha = 1;
  }

  for (int j=0; j < 3; j++) {
    rgb[j] = (std::byte)(255*(alpha*rgb1[j] + (1-alpha)*rgb2[j]));
  }
}



// make checker pattern for one side
static void checker(std::byte* rgb, const std::byte* RGB1, const std::byte* RGB2,
                    int width, int height) {
  for (int r=0; r < height/2; r++) {
    for (int c=0; c < width/2; c++) {
      memcpy(rgb+3*(r*width+c), RGB1, 3);
    }
  }
  for (int r=height/2; r < height; r++) {
    for (int c=width/2; c < width; c++) {
      memcpy(rgb+3*(r*width+c), RGB1, 3);
    }
  }
  for (int r=0; r < height/2; r++) {
    for (int c=width/2; c < width; c++) {
      memcpy(rgb+3*(r*width+c), RGB2, 3);
    }
  }
  for (int r=height/2; r < height; r++) {
    for (int c=0; c < width/2; c++) {
      memcpy(rgb+3*(r*width+c), RGB2, 3);
    }
  }
}



// make builtin: 2D
void sim_builder_texture_t::Builtin2D(void) {
  std::byte RGB1[3], RGB2[3], RGBm[3];
  // convert fixed colors
  for (int j=0; j < 3; j++) {
    RGB1[j] = (std::byte)(255*rgb1[j]);
    RGB2[j] = (std::byte)(255*rgb2[j]);
    RGBm[j] = (std::byte)(255*markrgb[j]);
  }

  //------------------ face

  // gradient
  if (builtin == SIM_BUILTIN_GRADIENT) {
    for (int r=0; r < height; r++) {
      for (int c=0; c < width; c++) {
        // compute normalized coordinates and radius
        double x = 2*c/((double)(width-1)) - 1;
        double y = 1 - 2*r/((double)(height-1));
        double pos = 2*sqrt(x*x+y*y) - 1;

        // interpolate through sigmoid
        interp(data_.data() + 3*(r*width+c), rgb2, rgb1, pos);
      }
    }
  }

  // checker
  else if (builtin == SIM_BUILTIN_CHECKER) {
    checker(data_.data(), RGB1, RGB2, width, height);
  }

  // flat
  else if (builtin == SIM_BUILTIN_FLAT) {
    for (int r=0; r < height; r++) {
      for (int c=0; c < width; c++) {
        memcpy(data_.data()+3*(r*width+c), RGB1, 3);
      }
    }
  }

  //------------------ marks

  // edge
  if (mark == SIM_MARK_EDGE) {
    for (int r=0; r < height; r++) {
      memcpy(data_.data()+3*(r*width+0), RGBm, 3);
      memcpy(data_.data()+3*(r*width+width-1), RGBm, 3);
    }
    for (int c=0; c < width; c++) {
      memcpy(data_.data()+3*(0*width+c), RGBm, 3);
      memcpy(data_.data()+3*((height-1)*width+c), RGBm, 3);
    }
  }

  // cross
  else if (mark == SIM_MARK_CROSS) {
    for (int r=0; r < height; r++) {
      memcpy(data_.data()+3*(r*width+width/2), RGBm, 3);
    }
    for (int c=0; c < width; c++) {
      memcpy(data_.data()+3*(height/2*width+c), RGBm, 3);
    }
  }

  // random dots
  else if (mark == SIM_MARK_RANDOM && random > 0) {
    randomdot(data_.data(), markrgb, width, height, random);
  }
}



// make builtin: Cube
void sim_builder_texture_t::BuiltinCube(void) {
  std::byte RGB1[3], RGB2[3], RGBm[3], RGBi[3];
  int w = width;
  if (w > std::numeric_limits<int>::max() / w) {
    throw sim_builder_error_t(this, "Cube texture width is too large.");
  }
  sim_size_t ww = width*width;

  // convert fixed colors
  for (int j = 0; j < 3; j++) {
    RGB1[j] = (std::byte)(255 * rgb1[j]);
    RGB2[j] = (std::byte)(255 * rgb2[j]);
    RGBm[j] = (std::byte)(255 * markrgb[j]);
  }

  //------------------ faces

  // gradient
  if (builtin == SIM_BUILTIN_GRADIENT) {
    if (ww > std::numeric_limits<std::int64_t>::max() / 18) {
      throw sim_builder_error_t(this, "Gradient texture width is too large.");
    }
    for (int r = 0; r < w; r++) {
      for (int c = 0; c < w; c++) {
        // compute normalized pixel coordinates
        double x = 2 * c / ((double)(w - 1)) - 1;
        double y = 1 - 2 * r / ((double)(w - 1));

        // compute normalized elevation for sides and up/down
        double elside = asin(y / sqrt(1 + x * x + y * y)) / (0.5 * SIM_PI);
        double elup = 1 - acos(1.0 / sqrt(1 + x * x + y * y)) / (0.5 * SIM_PI);

        // set sides
        interp(RGBi, rgb1, rgb2, elside);
        memcpy(data_.data() + 0 * 3 * ww + 3 * (r * w + c), RGBi, 3);  // 0: right
        memcpy(data_.data() + 1 * 3 * ww + 3 * (r * w + c), RGBi, 3);  // 1: left
        memcpy(data_.data() + 4 * 3 * ww + 3 * (r * w + c), RGBi, 3);  // 4: front
        memcpy(data_.data() + 5 * 3 * ww + 3 * (r * w + c), RGBi, 3);  // 5: back

        // set up and down
        interp(data_.data() + 2 * 3 * ww + 3 * (r * w + c), rgb1, rgb2, elup);  // 2: up
        interp(data_.data() + 3 * 3 * ww + 3 * (r * w + c), rgb1, rgb2, -elup);  // 3: down
      }
    }
  }

  // checker
  else if (builtin == SIM_BUILTIN_CHECKER) {
    checker(data_.data() + 0 * 3 * ww, RGB1, RGB2, w, w);
    checker(data_.data() + 1 * 3 * ww, RGB1, RGB2, w, w);
    checker(data_.data() + 2 * 3 * ww, RGB1, RGB2, w, w);
    checker(data_.data() + 3 * 3 * ww, RGB1, RGB2, w, w);
    checker(data_.data() + 4 * 3 * ww, RGB2, RGB1, w, w);
    checker(data_.data() + 5 * 3 * ww, RGB2, RGB1, w, w);
  }

  // flat
  else if (builtin == SIM_BUILTIN_FLAT) {
    for (int r = 0; r < w; r++) {
      for (int c = 0; c < w; c++) {
        // set sides and up
        memcpy(data_.data() + 0 * 3 * ww + 3 * (r * w + c), RGB1, 3);
        memcpy(data_.data() + 1 * 3 * ww + 3 * (r * w + c), RGB1, 3);
        memcpy(data_.data() + 2 * 3 * ww + 3 * (r * w + c), RGB1, 3);
        memcpy(data_.data() + 4 * 3 * ww + 3 * (r * w + c), RGB1, 3);
        memcpy(data_.data() + 5 * 3 * ww + 3 * (r * w + c), RGB1, 3);

        // set down
        memcpy(data_.data() + 3 * 3 * ww + 3 * (r * w + c), RGB2, 3);
      }
    }
  }

  //------------------ marks

  // edge
  if (mark == SIM_MARK_EDGE) {
    for (int j = 0; j < 6; j++) {
      for (int r = 0; r < w; r++) {
        memcpy(data_.data() + j * 3 * ww + 3 * (r * w + 0), RGBm, 3);
        memcpy(data_.data() + j * 3 * ww + 3 * (r * w + w - 1), RGBm, 3);
      }
      for (int c = 0; c < w; c++) {
        memcpy(data_.data() + j * 3 * ww + 3 * (0 * w + c), RGBm, 3);
        memcpy(data_.data() + j * 3 * ww + 3 * ((w - 1) * w + c), RGBm, 3);
      }
    }
  }

  // cross
  else if (mark == SIM_MARK_CROSS) {
    for (int j = 0; j < 6; j++) {
      for (int r = 0; r < w; r++) {
        memcpy(data_.data() + j * 3 * ww + 3 * (r * w + w / 2), RGBm, 3);
      }
      for (int c = 0; c < w; c++) {
        memcpy(data_.data() + j * 3 * ww + 3 * (w / 2 * w + c), RGBm, 3);
      }
    }
  }

  // random dots
  else if (mark == SIM_MARK_RANDOM && random > 0) {
    randomdot(data_.data(), markrgb, w, height, random);
  }
}

// load PNG file
void sim_builder_texture_t::LoadPNG(SIM_Resource* resource,
                         std::vector<std::byte>& image,
                         unsigned int& w, unsigned int& h, bool& is_srgb) {
  LodePNGColorType color_type;
  if (nchannel == 4) {
    color_type = LCT_RGBA;
  } else if (nchannel == 3) {
    color_type = LCT_RGB;
  } else if (nchannel == 1) {
    color_type = LCT_GREY;
  } else {
    throw sim_builder_error_t(this, "Unsupported number of channels: %s",
                   std::to_string(nchannel).c_str());
  }
  PNGImage png_image = PNGImage::Load(this, resource, color_type);
  w = png_image.Width();
  h = png_image.Height();
  is_srgb = png_image.IsSRGB();

  // Move data into image.
  image = std::move(png_image).MoveData();
}

// load KTX file
void sim_builder_texture_t::LoadKTX(SIM_Resource* resource, std::vector<std::byte>& image,
                         unsigned int& w, unsigned int& h, bool& is_srgb) {
  const void* buffer = 0;
  int buffer_sz = sim_math_readResource(resource, &buffer);

  // still not found
  if (buffer_sz < 0) {
    throw sim_builder_error_t(this, "could not read texture file '%s'", resource->name);
  } else if (!buffer_sz) {
    throw sim_builder_error_t(this, "texture file is empty: '%s'", resource->name);
  }

  w = buffer_sz;
  h = 1;
  is_srgb = false;

  image.resize(buffer_sz);
  memcpy(image.data(), buffer, buffer_sz);
}

// load custom file
void sim_builder_texture_t::LoadCustom(SIM_Resource* resource, std::vector<std::byte>& image,
                            unsigned int& w, unsigned int& h, bool& is_srgb) {
  const void* buffer = 0;
  int buffer_sz = sim_math_readResource(resource, &buffer);

  // still not found
  if (buffer_sz < 0) {
    throw sim_builder_error_t(this, "could not read texture file '%s'", resource->name);
  } else if (!buffer_sz) {
    throw sim_builder_error_t(this, "texture file is empty: '%s'", resource->name);
  }


  // read dimensions
  int* pint = (int*)buffer;
  w = pint[0];
  h = pint[1];

  // assume linear color space
  is_srgb = false;

  // check dimensions
  if (w < 1 || h < 1) {
    throw sim_builder_error_t(this, "Non-PNG texture, assuming custom binary file format,\n"
                         "non-positive texture dimensions in file '%s'", resource->name);
  }

  // check buffer size
  if (buffer_sz != 2*sizeof(int) + w*h*3*sizeof(char)) {
    throw sim_builder_error_t(this, "Non-PNG texture, assuming custom binary file format,\n"
                         "unexpected file size in file '%s'", resource->name);
  }

  // allocate and copy
  image.resize(w*h*3);
  memcpy(image.data(), (void*)(pint+2), w*h*3*sizeof(char));
}

void sim_builder_texture_t::FlipIfNeeded(std::vector<std::byte>& image, unsigned int w,
                              unsigned int h) {
  // horizontal flip
  if (hflip) {
    for (int r = 0; r < h; r++) {
      for (int c = 0; c < w / 2; c++) {
        int c1 = w - 1 - c;
        auto val1 = nchannel * (r * w + c);
        auto val2 = nchannel * (r * w + c1);
        for (int ch = 0; ch < nchannel; ch++) {
          auto tmp = image[val1 + ch];
          image[val1 + ch] = image[val2 + ch];
          image[val2 + ch] = tmp;
        }
      }
    }
  }

  // vertical flip
  if (vflip) {
    for (int r = 0; r < h / 2; r++) {
      for (int c = 0; c < w; c++) {
        int r1 = h - 1 - r;
        auto val1 = nchannel * (r * w + c);
        auto val2 = nchannel * (r1 * w + c);
        for (int ch = 0; ch < nchannel; ch++) {
          auto tmp = image[val1 + ch];
          image[val1 + ch] = image[val2 + ch];
          image[val2 + ch] = tmp;
        }
      }
    }
  }
}

std::string sim_builder_texture_t::GetCacheId(const SIM_Resource* resource, const std::string& asset_type) {
  std::stringstream ss;
  ss << resource->name << ";ARGS:content_type=" << asset_type << ",nchannel=" << nchannel
     << ",hflip=" << hflip << ",vflip=" << vflip << ";";
  return ss.str();
}

// load from PNG or custom file, flip if specified
void sim_builder_texture_t::LoadFlip(std::string filename, const SIM_VFS* vfs,
                          std::vector<std::byte>& image,
                          unsigned int& w, unsigned int& h, bool& is_srgb) {
  SIM_CCache* cache = reinterpret_cast<SIM_CCache*>(sim_getCache()->impl_);

  struct CachedImage {
    unsigned int w, h, n_ch;
    bool is_srgb;
    std::vector<std::byte> image;
  };

  // cache callback
  auto callback = [&](const void* data) {
    const CachedImage* cached_image =
        static_cast<const CachedImage*>(data);
    w = cached_image->w;
    h = cached_image->h;
    is_srgb = cached_image->is_srgb;
    image = cached_image->image;
    return true;
  };

  std::string asset_type = GetAssetContentType(filename, content_type_);

  // fallback to custom
  if (asset_type.empty()) {
    asset_type = "image/vnd.simcore.texture";
  }

  if (asset_type != "image/png" &&
      asset_type != "image/ktx" &&
      asset_type != "image/vnd.simcore.texture") {
    throw sim_builder_error_t(this, "unsupported content type: '%s'", asset_type.c_str());
  }

  // try loading from cache
  SIM_Resource* resource = LoadResource(modelfiledir_.Str(), filename, vfs);
  if (cache && cache->PopulateData(GetCacheId(resource, asset_type), resource, callback)) {
    sim_math_closeResource(resource);
    return;
  }

  try {
    if (asset_type == "image/png") {
      LoadPNG(resource, image, w, h, is_srgb);
    } else if (asset_type == "image/ktx") {
      if (hflip || vflip) {
        throw sim_builder_error_t(this, "cannot flip KTX textures");
      }
      LoadKTX(resource, image, w, h, is_srgb);
    } else {
      LoadCustom(resource, image, w, h, is_srgb);
    }
  } catch(sim_builder_error_t err) {
    sim_math_closeResource(resource);
    throw err;
  }

  FlipIfNeeded(image, w, h);
  if (cache) {
    CachedImage* cached_texture = new CachedImage;
    cached_texture->w = w;
    cached_texture->h = h;
    cached_texture->is_srgb = is_srgb;
    cached_texture->image = image;
    std::size_t size = sizeof(CachedImage) + image.size();
    std::shared_ptr<CachedImage> cached_data{cached_texture};
    cache->Insert("", GetCacheId(resource, asset_type), resource, cached_data, size);
  }
  sim_math_closeResource(resource);
}

// load 2D
void sim_builder_texture_t::Load2D(std::string filename, const SIM_VFS* vfs) {
  // load PNG or custom
  unsigned int w, h;
  bool is_srgb;

  LoadFlip(filename, vfs, data_, w, h, is_srgb);

  // assign size
  width = w;
  height = h;
  if (colorspace == SIM_COLORSPACE_AUTO) {
    colorspace = is_srgb ? SIM_COLORSPACE_SRGB : SIM_COLORSPACE_LINEAR;
  }
}

// load cube or skybox from single file (repeated or grid)
void sim_builder_texture_t::LoadCubeSingle(std::string filename, const SIM_VFS* vfs) {
  // check gridsize
  if (gridsize[0] < 1 || gridsize[1] < 1 || gridsize[0]*gridsize[1] > 12) {
    throw sim_builder_error_t(this, "gridsize must be non-zero and no more than 12 squares in texture");
  }

  // load PNG or custom
  unsigned int w, h;
  bool is_srgb;
  std::vector<std::byte> image;
  LoadFlip(filename, vfs, image, w, h, is_srgb);

  if (colorspace == SIM_COLORSPACE_AUTO) {
    colorspace = is_srgb ? SIM_COLORSPACE_SRGB : SIM_COLORSPACE_LINEAR;
  }

  // check gridsize for compatibility
  if (w/gridsize[1] != h/gridsize[0] || (w%gridsize[1]) || (h%gridsize[0])) {
    throw sim_builder_error_t(this,
                   "PNG size must be integer multiple of gridsize in texture '%s' (id %d)",
                   (const char*)file_.c_str(), id);
  }

  // assign size: repeated or full
  if (gridsize[0] == 1 && gridsize[1] == 1) {
    width = height = w;
  } else {
    width = w/gridsize[1];
    if (width >= std::numeric_limits<int>::max()/6) {
      throw sim_builder_error_t(this, "Invalid width of cube texture");
    }
    height = 6*width;
  }

  // allocate data
  std::int64_t size = static_cast<std::int64_t>(width)*height;
  if (size >= std::numeric_limits<std::int64_t>::max() / 3 || size <= 0) {
    throw sim_builder_error_t(this, "Cube texture too large");
  }
  try {
    data_.assign(3*size, std::byte(0));
  } catch (const std::bad_alloc& e) {
    throw sim_builder_error_t(this,
                   "Could not allocate memory for texture '%s' (id %d)",
                   (const char*)file_.c_str(), id);
  }

  // copy: repeated
  if (gridsize[0] == 1 && gridsize[1] == 1) {
    memcpy(data_.data(), image.data(), 3*width*width);
  }

  // copy: grid
  else {
    // keep track of which faces were defined
    int loaded[6] = {0, 0, 0, 0, 0, 0};

    // process grid
    for (int k=0; k < gridsize[0]*gridsize[1]; k++) {
      // decode face symbol
      int i = -1;
      if (gridlayout[k] == 'R') {
        i = 0;
      } else if (gridlayout[k] == 'L') {
        i = 1;
      } else if (gridlayout[k] == 'U') {
        i = 2;
      } else if (gridlayout[k] == 'D') {
        i = 3;
      } else if (gridlayout[k] == 'F') {
        i = 4;
      } else if (gridlayout[k] == 'B') {
        i = 5;
      } else if (gridlayout[k] != '.')
        throw sim_builder_error_t(this, "gridlayout symbol is not among '.RLUDFB' in texture");

      // load if specified
      if (i >= 0) {
        // extract sub-image
        int rstart = width*(k/gridsize[1]);
        int cstart = width*(k%gridsize[1]);
        for (int j=0; j < width; j++) {
          memcpy(data_.data()+i*3*width*width+j*3*width, image.data()+(j+rstart)*3*w+3*cstart, 3*width);
        }

        // mark as defined
        loaded[i] = 1;
      }
    }

    // set undefined faces to rgb1
    for (int i=0; i < 6; i++) {
      if (!loaded[i]) {
        for (int k=0; k < width; k++) {
          for (int s=0; s < width; s++) {
            for (int j=0; j < 3; j++) {
              data_[i*3*width*width + 3*(k*width+s) + j] = (std::byte)(255*rgb1[j]);
            }
          }
        }
      }
    }
  }

  image.clear();
}



// load cube or skybox from separate file
void sim_builder_texture_t::LoadCubeSeparate(const SIM_VFS* vfs) {
  // keep track of which faces were defined
  int loaded[6] = {0, 0, 0, 0, 0, 0};

  // process nonempty files
  for (int i=0; i < 6; i++) {
    if (!cubefiles_[i].empty()) {
      // remove path from file if necessary
      if (model->strippath) {
        cubefiles_[i] = sim_math_internal_strippath(cubefiles_[i]);
      }

      // make filename
      simcore::user::FilePath texturedir_;
      texturedir_ = FilePath(sim_spec_getString(compiler->texturedir));
      FilePath filename = texturedir_ + FilePath(cubefiles_[i]);

      // load PNG or custom
      unsigned int w, h;
      bool is_srgb;
      std::vector<std::byte> image;
      LoadFlip(filename.Str(), vfs, image, w, h, is_srgb);

      // assume all faces have the same colorspace
      if (colorspace == SIM_COLORSPACE_AUTO) {
        colorspace = is_srgb ? SIM_COLORSPACE_SRGB : SIM_COLORSPACE_LINEAR;
      }

      // PNG must be square
      if (w != h) {
        throw sim_builder_error_t(this,
                       "Non-square PNG file '%s' in cube or skybox id %d",
                       (const char*)cubefiles_[i].c_str(), id);
      }

      // first file: set size and allocate data
      if (data_.empty()) {
        width = w;
        if (width >= std::numeric_limits<int>::max()/6) {
          throw sim_builder_error_t(this, "Invalid width of builtin texture");
        }
        height = 6*width;
        std::int64_t size = static_cast<std::int64_t>(width)*height;
        if (size >= std::numeric_limits<sim_size_t>::max() / 3 || size <= 0) {
          throw sim_builder_error_t(this, "PNG texture too large");
        }
        try {
          data_.assign(3*size, std::byte(0));
        } catch (const std::bad_alloc& e) {
          throw sim_builder_error_t(this, "Could not allocate memory for texture");
        }
      }

      // otherwise check size
      else if (width != w) {
        throw sim_builder_error_t(this,
                       "PNG file '%s' has incompatible size in texture id %d",
                       (const char*)cubefiles_[i].c_str(), id);
      }

      // copy data
      memcpy(data_.data()+i*3*width*width, image.data(), 3*width*width);
      image.clear();

      // mark as defined
      loaded[i] = 1;
    }
  }

  // set undefined faces to rgb1
  for (int i=0; i < 6; i++) {
    if (!loaded[i]) {
      for (int k=0; k < width; k++) {
        for (int s=0; s < width; s++) {
          for (int j=0; j < 3; j++) {
            data_[i*3*width*width + 3*(k*width+s) + j] = (std::byte)(255*rgb1[j]);
          }
        }
      }
    }
  }
}



// compiler
void sim_builder_texture_t::Compile(const SIM_VFS* vfs) {
  CopyFromSpec();

  // copy paths from model if not already defined
  if (modelfiledir_.empty()) {
    modelfiledir_ = FilePath(model->modelfiledir_);
  }
  simcore::user::FilePath texturedir_;
  texturedir_ = FilePath(sim_spec_getString(compiler->texturedir));

  // buffer from user
  if (!data_.empty()) {
    if (data_.size() != nchannel*width*height) {
      throw sim_builder_error_t(this, "Texture buffer has incorrect size, given %d expected %d", nullptr,
                     data_.size(), nchannel * width * height);
    }

    // Flip if specified.
    FlipIfNeeded(data_, width, height);
    return;
  }

  // builtin
  else if (builtin != SIM_BUILTIN_NONE) {
    // check width
    if (width < 1) {
      throw sim_builder_error_t(this, "Invalid width of builtin texture");
    }

    // adjust height of cube texture
    if (type != SIM_TEXTURE_2D) {
      if (width >= std::numeric_limits<int>::max()/6) {
        throw sim_builder_error_t(this, "Invalid width of builtin texture");
      }
      height = 6*width;
    } else {
      if (height < 1) {
        throw sim_builder_error_t(this, "Invalid height of builtin texture");
      }
    }

    std::int64_t size = static_cast<std::int64_t>(width)*height;
    if (size >= std::numeric_limits<int64_t>::max() / nchannel || size <= 0) {
      throw sim_builder_error_t(this, "Builtin texture too large");
    }
    // allocate data
    try {
      data_.assign(nchannel*size, std::byte(0));
    } catch (const std::bad_alloc& e) {
      throw sim_builder_error_t(this, "Could not allocate memory for texture");
    }

    // dispatch
    if (type == SIM_TEXTURE_2D) {
      Builtin2D();
    } else {
      BuiltinCube();
    }
  }

  // single file
  else if (!file_.empty()) {
    // remove path from file if necessary
    if (model->strippath) {
      file_ = sim_math_internal_strippath(file_);
    }

    // make filename
    FilePath filename = texturedir_ + FilePath(file_);

    // dispatch
    if (type == SIM_TEXTURE_2D) {
      Load2D(filename.Str(), vfs);
    } else {
      LoadCubeSingle(filename.Str(), vfs);
    }
  }

  // separate files
  else {
    // 2D not allowed
    if (type == SIM_TEXTURE_2D) {
      throw sim_builder_error_t(this,
                     "Cannot load 2D texture from separate files, texture");
    }

    // at least one cubefile must be defined
    bool defined = false;
    for (int i=0; i < 6; i++) {
      if (!cubefiles_[i].empty()) {
        defined = true;
        break;
      }
    }
    if (!defined) {
      throw sim_builder_error_t(this,
                     "No cubefiles_ defined in cube or skybox texture");
    }

    // only cube and skybox
    LoadCubeSeparate(vfs);
  }

  // make sure someone allocated data; SHOULD NOT OCCUR
  if (data_.empty()) {
    throw sim_builder_error_t(this, "texture '%s' (id %d) was not specified", name.c_str(), id);
  }

  // if recompiled is called, clear data_ first
  clear_data_ = true;
}



//------------------ class SIM_CMaterial implementation ----------------------------------------------

// initialize defaults
SIM_CMaterial::SIM_CMaterial(sim_builder_model_t* _model, sim_builder_default_t* _def) {
  sim_spec_defaultMaterial(&spec);
  elemtype = SIM_OBJ_MATERIAL;
  textures_.assign(SIM_NTEXROLE, "");
  spec_textures_.assign(SIM_NTEXROLE, "");

  // clear internal
  for (int i=0; i < SIM_NTEXROLE; i++) {
    texid[i] = -1;
  }

  // reset to default if given
  if (_def) {
    *this = _def->Material();
  }

  model = _model;
  if (_model) compiler = &_model->spec.compiler;
  classname = _def ? _def->name : "main";

  PointToLocal();

  // in case this material is not compiled
  CopyFromSpec();
}



SIM_CMaterial::SIM_CMaterial(const SIM_CMaterial& other) {
  *this = other;
}



SIM_CMaterial& SIM_CMaterial::operator=(const SIM_CMaterial& other) {
  if (this != &other) {
    this->spec = other.spec;
    *static_cast<SIM_CMaterial_*>(this) = static_cast<const SIM_CMaterial_&>(other);
    *static_cast<SIM_sMaterial*>(this) = static_cast<const SIM_sMaterial&>(other);
  }
  PointToLocal();
  return *this;
}



void SIM_CMaterial::PointToLocal() {
  spec.element = static_cast<sim_spec_element_t*>(this);
  spec.textures = &spec_textures_;
  spec.info = &info;
  textures = nullptr;
}



void SIM_CMaterial::CopyFromSpec() {
  *static_cast<SIM_sMaterial*>(this) = spec;
  textures_ = spec_textures_;
}



void SIM_CMaterial::NameSpace(const sim_builder_model_t* m) {
  sim_builder_base_t::NameSpace(m);
  for (int i=0; i < SIM_NTEXROLE; i++) {
    if (!spec_textures_[i].empty()) {
      spec_textures_[i] = m->prefix + spec_textures_[i] + m->suffix;
    }
  }
}



// compiler
void SIM_CMaterial::Compile(void) {
  CopyFromSpec();
}



//------------------ class SIM_CPair implementation --------------------------------------------------

// constructor
SIM_CPair::SIM_CPair(sim_builder_model_t* _model, sim_builder_default_t* _def) {
  sim_spec_defaultPair(&spec);
  elemtype = SIM_OBJ_PAIR;

  // set defaults
  spec_geomname1_.clear();
  spec_geomname2_.clear();

  // clear internal variables
  geom1 = nullptr;
  geom2 = nullptr;
  signature = -1;

  // reset to default if given
  if (_def) {
    *this = _def->Pair();
  }

  // set model, def
  model = _model;
  if (_model) compiler = &_model->spec.compiler;
  classname = _def ? _def->name : "main";

  // point to local
  PointToLocal();

  // in case this camera is not compiled
  CopyFromSpec();
}



SIM_CPair::SIM_CPair(const SIM_CPair& other) {
  *this = other;
}



SIM_CPair& SIM_CPair::operator=(const SIM_CPair& other) {
  if (this != &other) {
    this->spec = other.spec;
    *static_cast<SIM_CPair_*>(this) = static_cast<const SIM_CPair_&>(other);
    *static_cast<SIM_sPair*>(this) = static_cast<const SIM_sPair&>(other);
    this->geom1 = nullptr;
    this->geom2 = nullptr;
  }
  PointToLocal();
  return *this;
}



void SIM_CPair::PointToLocal() {
  spec.element = static_cast<sim_spec_element_t*>(this);
  spec.geomname1 = &spec_geomname1_;
  spec.geomname2 = &spec_geomname2_;
  geomname1 = nullptr;
  geomname2 = nullptr;
  spec.info = &info;
}



void SIM_CPair::NameSpace(const sim_builder_model_t* m) {
  sim_builder_base_t::NameSpace(m);
  prefix = m->prefix;
  suffix = m->suffix;
}



void SIM_CPair::CopyFromSpec() {
  *static_cast<SIM_sPair*>(this) = spec;
  geomname1_ = spec_geomname1_;
  geomname2_ = spec_geomname2_;
}



void SIM_CPair::ResolveReferences(const sim_builder_model_t* m) {
  geomname1_ = prefix + geomname1_ + suffix;
  geomname2_ = prefix + geomname2_ + suffix;
  geom1 = (sim_builder_geom_t*)m->FindObject(SIM_OBJ_GEOM, geomname1_);
  geom2 = (sim_builder_geom_t*)m->FindObject(SIM_OBJ_GEOM, geomname2_);

  if (!geom1 && geom2) {
    geomname1_ = spec_geomname1_;
    geom1 = (sim_builder_geom_t*)m->FindObject(SIM_OBJ_GEOM, geomname1_);
  }
  if (geom1 && !geom2) {
    geomname2_ = spec_geomname2_;
    geom2 = (sim_builder_geom_t*)m->FindObject(SIM_OBJ_GEOM, geomname2_);
  }

  if (!geom1) {
    throw sim_builder_error_t(this, "geom '%s' not found in collision %d", geomname1_.c_str(), id);
  }
  if (!geom2) {
    throw sim_builder_error_t(this, "geom '%s' not found in collision %d", geomname2_.c_str(), id);
  }

  spec_geomname1_ = geomname1_;
  spec_geomname2_ = geomname2_;
  prefix.clear();
  suffix.clear();

  // swap if body1 > body2
  if (geom1->body->id > geom2->body->id) {
    std::string nametmp = geomname1_;
    geomname1_ = geomname2_;
    geomname2_ = nametmp;

    sim_builder_geom_t* geomtmp = geom1;
    geom1 = geom2;
    geom2 = geomtmp;
  }

  // get geom ids and body signature
  signature = ((geom1->body->id)<<16) + geom2->body->id;
}



// compiler
void SIM_CPair::Compile(void) {
  CopyFromSpec();

  // check condim
  if (condim != 1 && condim != 3 && condim != 4 && condim != 6) {
    throw sim_builder_error_t(this, "invalid condim in contact pair");
  }

  // find geoms
  ResolveReferences(model);

  // mark geoms as not visual
  geom1->SetNotVisual();
  geom2->SetNotVisual();

  // set undefined margin: max
  if (!sim_math_internal_defined(margin)) {
    margin = std::max(geom1->margin, geom2->margin);
  }

  // set undefined gap: max
  if (!sim_math_internal_defined(gap)) {
    gap = std::max(geom1->gap, geom2->gap);
  }

  // set undefined condim, friction, solref, solimp: different priority
  if (geom1->priority != geom2->priority) {
    sim_builder_geom_t* pgh = (geom1->priority > geom2->priority ? geom1 : geom2);

    // condim
    if (condim < 0) {
      condim = pgh->condim;
    }

    // friction
    if (!sim_math_internal_defined(friction[0])) {
      friction[0] = friction[1] = pgh->friction[0];
      friction[2] =               pgh->friction[1];
      friction[3] = friction[4] = pgh->friction[2];
    }

    // reference
    if (!sim_math_internal_defined(solref[0])) {
      for (int i=0; i < SIM_NREF; i++) {
        solref[i] = pgh->solref[i];
      }
    }

    // impedance
    if (!sim_math_internal_defined(solimp[0])) {
      for (int i=0; i < SIM_NIMP; i++) {
        solimp[i] = pgh->solimp[i];
      }
    }
  }

  // set undefined condim, friction, solref, solimp: same priority
  else {
    // condim: max
    if (condim < 0) {
      condim = std::max(geom1->condim, geom2->condim);
    }

    // friction: max
    if (!sim_math_internal_defined(friction[0])) {
      friction[0] = friction[1] = std::max(geom1->friction[0], geom2->friction[0]);
      friction[2] =               std::max(geom1->friction[1], geom2->friction[1]);
      friction[3] = friction[4] = std::max(geom1->friction[2], geom2->friction[2]);
    }

    // solver mix factor
    double mix;
    if (geom1->solmix >= SIM_EPS && geom2->solmix >= SIM_EPS) {
      mix = geom1->solmix / (geom1->solmix + geom2->solmix);
    } else if (geom1->solmix < SIM_EPS && geom2->solmix < SIM_EPS) {
      mix = 0.5;
    } else if (geom1->solmix < SIM_EPS) {
      mix = 0.0;
    } else {
      mix = 1.0;
    }

    // reference
    if (!sim_math_internal_defined(solref[0])) {
      // standard: mix
      if (solref[0] > 0) {
        for (int i=0; i < SIM_NREF; i++) {
          solref[i] = mix*geom1->solref[i] + (1-mix)*geom2->solref[i];
        }
      }

      // direct: min
      else {
        for (int i=0; i < SIM_NREF; i++) {
          solref[i] = std::min(geom1->solref[i], geom2->solref[i]);
        }
      }
    }

    // impedance
    if (!sim_math_internal_defined(solimp[0])) {
      for (int i=0; i < SIM_NIMP; i++) {
        solimp[i] = mix*geom1->solimp[i] + (1-mix)*geom2->solimp[i];
      }
    }
  }
}



//------------------ class SIM_CBodyPair implementation ----------------------------------------------

// constructor
SIM_CBodyPair::SIM_CBodyPair(sim_builder_model_t* _model) {
  // set model pointer
  model = _model;
  if (_model) compiler = &_model->spec.compiler;
  elemtype = SIM_OBJ_EXCLUDE;

  // set defaults
  spec_bodyname1_.clear();
  spec_bodyname2_.clear();

  // clear internal variables
  body1 = body2 = signature = -1;

  PointToLocal();
  CopyFromSpec();
}



SIM_CBodyPair::SIM_CBodyPair(const SIM_CBodyPair& other) {
  *this = other;
}



SIM_CBodyPair& SIM_CBodyPair::operator=(const SIM_CBodyPair& other) {
  if (this != &other) {
    this->spec = other.spec;
    *static_cast<SIM_CBodyPair_*>(this) = static_cast<const SIM_CBodyPair_&>(other);
    *static_cast<SIM_sExclude*>(this) = static_cast<const SIM_sExclude&>(other);
  }
  PointToLocal();
  return *this;
}



void SIM_CBodyPair::PointToLocal() {
  spec.element = static_cast<sim_spec_element_t*>(this);
  spec.bodyname1 = &spec_bodyname1_;
  spec.bodyname2 = &spec_bodyname2_;
  spec.info = &info;
  bodyname1 = nullptr;
  bodyname2 = nullptr;
}



void SIM_CBodyPair::NameSpace(const sim_builder_model_t* m) {
  if (!name.empty()) {
    name = m->prefix + name + m->suffix;
  }
  prefix = m->prefix;
  suffix = m->suffix;
}



void SIM_CBodyPair::CopyFromSpec() {
  *static_cast<SIM_sExclude*>(this) = spec;
  bodyname1_ = spec_bodyname1_;
  bodyname2_ = spec_bodyname2_;
}



void SIM_CBodyPair::ResolveReferences(const sim_builder_model_t* m) {
  bodyname1_ = prefix + bodyname1_ + suffix;
  bodyname2_ = prefix + bodyname2_ + suffix;
  sim_builder_body_t* pb1 = (sim_builder_body_t*)m->FindObject(SIM_OBJ_BODY, bodyname1_);
  sim_builder_body_t* pb2 = (sim_builder_body_t*)m->FindObject(SIM_OBJ_BODY, bodyname2_);

  if (!pb1 && pb2) {
    bodyname1_ = spec_bodyname1_;
    pb1 = (sim_builder_body_t*)m->FindObject(SIM_OBJ_BODY, bodyname1_);
  }
  if (pb1 && !pb2) {
    bodyname2_ = spec_bodyname2_;
    pb2 = (sim_builder_body_t*)m->FindObject(SIM_OBJ_BODY, bodyname2_);
  }

  if (!pb1) {
    throw sim_builder_error_t(this, "body '%s' not found in bodypair %d", bodyname1_.c_str(), id);
  }
  if (!pb2) {
    throw sim_builder_error_t(this, "body '%s' not found in bodypair %d", bodyname2_.c_str(), id);
  }

  spec_bodyname1_ = bodyname1_;
  spec_bodyname2_ = bodyname2_;
  prefix.clear();
  suffix.clear();

  // swap if body1 > body2
  if (pb1->id > pb2->id) {
    std::string nametmp = bodyname1_;
    bodyname1_ = bodyname2_;
    bodyname2_ = nametmp;

    sim_builder_body_t* bodytmp = pb1;
    pb1 = pb2;
    pb2 = bodytmp;
  }

  // get body ids and body signature
  body1 = pb1->id;
  body2 = pb2->id;
  signature = (body1<<16) + body2;
}



// compiler
void SIM_CBodyPair::Compile(void) {
  CopyFromSpec();

  // find bodies
  ResolveReferences(model);
}



//------------------ class SIM_CEquality implementation ----------------------------------------------

// initialize default constraint
SIM_CEquality::SIM_CEquality(sim_builder_model_t* _model, sim_builder_default_t* _def) {
  sim_spec_defaultEquality(&spec);
  elemtype = SIM_OBJ_EQUALITY;

  // clear internal variables
  spec_name1_.clear();
  spec_name2_.clear();
  obj1id = obj2id = -1;

  // reset to default if given
  if (_def) {
    *this = _def->Equality();
  }

  // set model, def
  model = _model;
  if (_model) compiler = &_model->spec.compiler;
  classname = _def ? _def->name : "main";

  // point to local
  PointToLocal();

  // in case this camera is not compiled
  CopyFromSpec();
}



SIM_CEquality::SIM_CEquality(const SIM_CEquality& other) {
  *this = other;
}



SIM_CEquality& SIM_CEquality::operator=(const SIM_CEquality& other) {
  if (this != &other) {
    this->spec = other.spec;
    *static_cast<SIM_CEquality_*>(this) = static_cast<const SIM_CEquality_&>(other);
    *static_cast<SIM_sEquality*>(this) = static_cast<const SIM_sEquality&>(other);
  }
  PointToLocal();
  return *this;
}



void SIM_CEquality::PointToLocal() {
  spec.element = static_cast<sim_spec_element_t*>(this);
  spec.name1 = &spec_name1_;
  spec.name2 = &spec_name2_;
  spec.info = &info;
  name1 = nullptr;
  name2 = nullptr;
}



void SIM_CEquality::NameSpace(const sim_builder_model_t* m) {
  sim_builder_base_t::NameSpace(m);
  if (!spec_name1_.empty()) {
    spec_name1_ = m->prefix + spec_name1_ + m->suffix;
  }
  if (!spec_name2_.empty()) {
    spec_name2_ = m->prefix + spec_name2_ + m->suffix;
  }
}



void SIM_CEquality::CopyFromSpec() {
  *static_cast<SIM_sEquality*>(this) = spec;
  name1_ = spec_name1_;
  name2_ = spec_name2_;
}



void SIM_CEquality::ResolveReferences(const sim_builder_model_t* m) {
  sim_obj_t object_type;
  sim_builder_base_t *px1, *px2;
  SIM_tJoint jt1, jt2;

  // determine object type
  if (type == SIM_EQ_WELD) {
    if (objtype != SIM_OBJ_SITE && objtype != SIM_OBJ_BODY) {
      throw sim_builder_error_t(this, "weld constraint supports only sites and bodies");
    }
    object_type = objtype;
  } else if (type == SIM_EQ_CONNECT) {
    if (objtype != SIM_OBJ_SITE && objtype != SIM_OBJ_BODY) {
      throw sim_builder_error_t(this, "connect constraint supports only sites and bodies");
    }
    object_type = objtype;
  } else if (type == SIM_EQ_JOINT) {
    object_type = SIM_OBJ_JOINT;
  } else if (type == SIM_EQ_TENDON) {
    object_type = SIM_OBJ_TENDON;
  } else if (type == SIM_EQ_FLEX || type == SIM_EQ_FLEXVERT) {
    object_type = SIM_OBJ_FLEX;
  } else {
    throw sim_builder_error_t(this, "invalid type in equality constraint");
  }

  // find object 1, get id
  px1 = m->FindObject(object_type, name1_);
  if (!px1) {
    throw sim_builder_error_t(this, "unknown element '%s' in equality constraint", name1_.c_str());
  }
  obj1id = px1->id;

  // find object 2, get id
  if (!name2_.empty()) {
    px2 = m->FindObject(object_type, name2_);
    if (!px2) {
      throw sim_builder_error_t(this, "unknown element '%s' in equality constraint %d", name2_.c_str(), id);
    }
    obj2id = px2->id;
  } else {
    // object 2 unspecified: set to -1
    obj2id = -1;
    px2 = nullptr;
  }

  // set missing body = world
  if (object_type == SIM_OBJ_BODY && obj2id == -1) {
    obj2id = 0;
  }

  // make sure the two objects are different
  if (obj1id == obj2id) {
    throw sim_builder_error_t(this, "element '%s' is repeated in equality constraint %d", name1_.c_str(), id);
  }

  // make sure joints are scalar
  if (type == SIM_EQ_JOINT) {
    jt1 = ((sim_builder_joint_t*)px1)->type;
    jt2 = (px2 ? ((sim_builder_joint_t*)px2)->type : SIM_JNT_HINGE);
    if ((jt1 != SIM_JNT_HINGE && jt1 != SIM_JNT_SLIDE) ||
        (jt2 != SIM_JNT_HINGE && jt2 != SIM_JNT_SLIDE)) {
      throw sim_builder_error_t(this, "only HINGE and SLIDE joint allowed in constraint");
    }
  }
}



// compiler
void SIM_CEquality::Compile(void) {
  CopyFromSpec();

  // find objects
  ResolveReferences(model);

  // make sure flex is not rigid
  if ((type == SIM_EQ_FLEX || type == SIM_EQ_FLEXVERT) && model->Flexes()[obj1id]->rigid) {
    throw sim_builder_error_t(this, "rigid flex '%s' in equality constraint %d", name1_.c_str(), id);
  }
}



//------------------ class sim_builder_tendon_t implementation ------------------------------------------------

// constructor
sim_builder_tendon_t::sim_builder_tendon_t(sim_builder_model_t* _model, sim_builder_default_t* _def) {
  sim_spec_defaultTendon(&spec);
  elemtype = SIM_OBJ_TENDON;

  // clear internal variables
  spec_material_.clear();
  spec_userdata_.clear();
  path.clear();
  matid = -1;

  // reset to default if given
  if (_def) {
    *this = _def->Tendon();
  }

  // set model, def
  model = _model;
  if (_model) compiler = &_model->spec.compiler;
  classname = _def ? _def->name : "main";

  // point to local
  PointToLocal();

  // in case this camera is not compiled
  CopyFromSpec();
}



sim_builder_tendon_t::sim_builder_tendon_t(const sim_builder_tendon_t& other) {
  *this = other;
}



sim_builder_tendon_t& sim_builder_tendon_t::operator=(const sim_builder_tendon_t& other) {
  if (this != &other) {
    this->spec = other.spec;
    *static_cast<SIM_CTendon_*>(this) = static_cast<const SIM_CTendon_&>(other);
    *static_cast<SIM_sTendon*>(this) = static_cast<const SIM_sTendon&>(other);
    for (int i=0; i < other.path.size(); i++) {
      path.push_back(new SIM_CWrap(*other.path[i]));
      path.back()->tendon = this;
    }
  }
  PointToLocal();
  return *this;
}



bool sim_builder_tendon_t::is_limited() const {
  return islimited(limited, range);
}
bool sim_builder_tendon_t::is_actfrclimited() const {
  return islimited(actfrclimited, actfrcrange);
}

void sim_builder_tendon_t::PointToLocal() {
  spec.element = static_cast<sim_spec_element_t*>(this);
  spec.material = &spec_material_;
  spec.userdata = &spec_userdata_;
  spec.info = &info;
  material = nullptr;
  userdata = nullptr;
}



void sim_builder_tendon_t::NameSpace(const sim_builder_model_t* m) {
  sim_builder_base_t::NameSpace(m);
  prefix = m->prefix;
  suffix = m->suffix;
}



void sim_builder_tendon_t::CopyFromSpec() {
  *static_cast<SIM_sTendon*>(this) = spec;
  material_ = spec_material_;
  userdata_ = spec_userdata_;

  // clear precompiled
  for (int i=0; i < path.size(); i++) {
    if (path[i]->Type() == SIM_WRAP_CYLINDER) {
      path[i]->spec.type = SIM_WRAP_SPHERE;
    }
  }
}



// desctructor
sim_builder_tendon_t::~sim_builder_tendon_t() {
  // delete objects allocated here
  for (unsigned int i=0; i < path.size(); i++) {
    delete path[i];
  }

  path.clear();
}



void sim_builder_tendon_t::SetModel(sim_builder_model_t* _model) {
  model = _model;
  if (_model) compiler = &_model->spec.compiler;
  for (int i=0; i < path.size(); i++) {
    path[i]->model = _model;
  }
}



// add site as wrap object
void sim_builder_tendon_t::WrapSite(std::string wrapname, std::string_view wrapinfo) {
  // create wrap object
  SIM_CWrap* wrap = new SIM_CWrap(model, this);
  wrap->info = wrapinfo;

  // set parameters, add to path
  wrap->spec.type = SIM_WRAP_SITE;
  wrap->name = wrapname;
  wrap->id = (int)path.size();
  path.push_back(wrap);
}



// add geom (with side site) as wrap object
void sim_builder_tendon_t::WrapGeom(std::string wrapname, std::string sidesite, std::string_view wrapinfo) {
  // create wrap object
  SIM_CWrap* wrap = new SIM_CWrap(model, this);
  wrap->info = wrapinfo;

  // set parameters, add to path
  wrap->spec.type = SIM_WRAP_SPHERE;         // replace with cylinder later if needed
  wrap->name = wrapname;
  wrap->sidesite = sidesite;
  wrap->id = (int)path.size();
  path.push_back(wrap);
}



// add joint as wrap object
void sim_builder_tendon_t::WrapJoint(std::string wrapname, double coef, std::string_view wrapinfo) {
  // create wrap object
  SIM_CWrap* wrap = new SIM_CWrap(model, this);
  wrap->info = wrapinfo;

  // set parameters, add to path
  wrap->spec.type = SIM_WRAP_JOINT;
  wrap->name = wrapname;
  wrap->prm = coef;
  wrap->id = (int)path.size();
  path.push_back(wrap);
}



// add pulley
void sim_builder_tendon_t::WrapPulley(double divisor, std::string_view wrapinfo) {
  // create wrap object
  SIM_CWrap* wrap = new SIM_CWrap(model, this);
  wrap->info = wrapinfo;

  // set parameters, add to path
  wrap->spec.type = SIM_WRAP_PULLEY;
  wrap->prm = divisor;
  wrap->id = (int)path.size();
  path.push_back(wrap);
}



// get number of wraps
int sim_builder_tendon_t::NumWraps() const {
  return (int)path.size();
}



// get pointer to specified wrap
const SIM_CWrap* sim_builder_tendon_t::GetWrap(int i) const {
  if (i >= 0 && i < (int)path.size()) {
    return path[i];
  }
  return nullptr;
}



void sim_builder_tendon_t::ResolveReferences(const sim_builder_model_t* m) {
  int nfailure = 0;
  int npulley = 0;
  for (int i=0; i < path.size(); i++) {
    std::string pname = path[i]->name;
    std::string psidesite = path[i]->sidesite;
    if (path[i]->Type() == SIM_WRAP_PULLEY) {
      npulley++;
    }
    try {
      // look for wrapped element with namespace
      path[i]->name = prefix + pname + suffix;
      path[i]->sidesite = prefix + psidesite + suffix;
      path[i]->ResolveReferences(m);
    } catch(sim_builder_error_t) {
      // remove namespace from wrap names
      path[i]->name = pname;
      path[i]->sidesite = psidesite;
      path[i]->ResolveReferences(m);
      nfailure++;
    }
  }
  if (nfailure == path.size()-npulley) {
    throw sim_builder_error_t(this, "tendon '%s' (id = %d): no attached reference found", name.c_str(), id);
  }
  prefix.clear();
  suffix.clear();
}



// compiler
void sim_builder_tendon_t::Compile(void) {
  // compile all wraps in the path
  for (SIM_CWrap* wrap : path) {
    wrap->Compile();
  }

  CopyFromSpec();

  // resize userdata
  if (userdata_.size() > model->nuser_tendon) {
    throw sim_builder_error_t(this, "user has more values than nuser_tendon in tendon");
  }
  userdata_.resize(model->nuser_tendon);

  // check for empty path
  int sz = (int)path.size();
  if (!sz) {
    throw sim_builder_error_t(this,
                   "tendon '%s' (id = %d): path cannot be empty",
                   name.c_str(), id);
  }

  // determine type
  bool spatial = (path[0]->Type() != SIM_WRAP_JOINT);

  // require at least two objects in spatial path
  if (spatial && sz < 2) {
    throw sim_builder_error_t(this, "tendon '%s' (id = %d): spatial path must contain at least two objects",
                   name.c_str(), id);
  }

  // require positive width
  if (spatial && width <= 0) {
    throw sim_builder_error_t(this, "tendon '%s' (id = %d) must have positive width", name.c_str(), id);
  }

  // compile objects in path
  ResolveReferences(model);

  // check path
  for (int i=0; i < sz; i++) {
    // fixed
    if (!spatial) {
      // make sure all objects are joints
      if (path[i]->Type() != SIM_WRAP_JOINT) {
        throw sim_builder_error_t(this, "tendon '%s' (id = %d): spatial object found in fixed path at pos %d",
                       name.c_str(), id, i);
      }
    }

    // spatial path
    else {
      if (armature < 0) {
        throw sim_builder_error_t(this,
                       "tendon '%s' (id = %d): tendon armature cannot be negative",
                        name.c_str(), id);
      }

      switch (path[i]->Type()) {
        case SIM_WRAP_PULLEY:
          // pulley should not follow other pulley
          if (i > 0 && path[i-1]->Type() == SIM_WRAP_PULLEY) {
            throw sim_builder_error_t(this, "tendon '%s' (id = %d): consecutive pulleys (pos %d)",
                           name.c_str(), id, i);
          }

          // pulley should not be last
          if (i == sz-1) {
            throw sim_builder_error_t(this, "tendon '%s' (id = %d): path ends with pulley", name.c_str(), id);
          }
          break;

        case SIM_WRAP_SITE:
          // site needs a neighbor that is not a pulley
          if ((i == 0 || path[i-1]->Type() == SIM_WRAP_PULLEY) &&
              (i == sz-1 || path[i+1]->Type() == SIM_WRAP_PULLEY)) {
            throw sim_builder_error_t(this,
                           "tendon '%s' (id = %d): site %d needs a neighbor that is not a pulley",
                           name.c_str(), id, i);
          }

          // site cannot be repeated
          if (i < sz-1 && path[i+1]->Type() == SIM_WRAP_SITE && path[i]->obj->id == path[i+1]->obj->id) {
            throw sim_builder_error_t(this,
                           "tendon '%s' (id = %d): site %d is repeated",
                           name.c_str(), id, i);
          }

          break;

        case SIM_WRAP_SPHERE:
        case SIM_WRAP_CYLINDER:
          // geom must be bracketed by sites
          if (i == 0 || i == sz-1 || path[i-1]->Type() != SIM_WRAP_SITE || path[i+1]->Type() != SIM_WRAP_SITE) {
            throw sim_builder_error_t(this,
                           "tendon '%s' (id = %d): geom at pos %d not bracketed by sites",
                           name.c_str(), id, i);
          }

          if (armature > 0) {
            throw sim_builder_error_t(this,
                           "tendon '%s' (id = %d): geom wrapping not supported by tendon armature",
                           name.c_str(), id);
          }

          // mark geoms as non visual
          model->Geoms()[path[i]->obj->id]->SetNotVisual();
          break;

        case SIM_WRAP_JOINT:
          throw sim_builder_error_t(this,
                         "tendon '%s (id = %d)': joint wrap found in spatial path at pos %d",
                         name.c_str(), id, i);

        default:
          throw sim_builder_error_t(this,
                         "tendon '%s (id = %d)': invalid wrap object at pos %d",
                         name.c_str(), id, i);
      }
    }
  }

  // if limited is auto, set to 1 if range is specified, otherwise unlimited
  if (limited == SIM_LIMITED_AUTO) {
    bool hasrange = !(range[0] == 0 && range[1] == 0);
    checklimited(this, compiler->autolimits, "tendon", "", limited, hasrange);
  }

  // check limits
  if (range[0] >= range[1] && is_limited()) {
    throw sim_builder_error_t(this, "invalid limits in tendon");
  }

  // if limited is auto, set to 1 if range is specified, otherwise unlimited
  if (actfrclimited == SIM_LIMITED_AUTO) {
    bool hasactfrcrange = !(actfrcrange[0] == 0 && actfrcrange[1] == 0);
    checklimited(this, compiler->autolimits, "tendon", "", actfrclimited,
                 hasactfrcrange);
  }

  // check actfrclimits
  if (actfrcrange[0] >= actfrcrange[1] && is_actfrclimited()) {
    throw sim_builder_error_t(this, "invalid actuatorfrcrange in tendon");
  }
  if ((actfrcrange[0] > 0 || actfrcrange[1] < 0) && is_actfrclimited()) {
    throw sim_builder_error_t(this, "invalid actuatorfrcrange in tendon");
  }

  // check springlength
  if (springlength[0] > springlength[1]) {
    throw sim_builder_error_t(this, "invalid springlength in tendon");
  }
}



//------------------ class SIM_CWrap implementation --------------------------------------------------

// constructor
SIM_CWrap::SIM_CWrap(sim_builder_model_t* _model, sim_builder_tendon_t* _tendon) {
  elemtype = SIM_OBJ_UNKNOWN;

  // set model and tendon pointer
  model = _model;
  if (_model) compiler = &_model->spec.compiler;
  tendon = _tendon;

  // clear variables
  spec.type = SIM_WRAP_NONE;
  obj = nullptr;
  sideid = -1;
  prm = 0;
  sidesite.clear();

  // point to local
  PointToLocal();
  CopyFromSpec();
}



SIM_CWrap::SIM_CWrap(const SIM_CWrap& other) {
  *this = other;
}



SIM_CWrap& SIM_CWrap::operator=(const SIM_CWrap& other) {
  if (this != &other) {
    this->spec = other.spec;
    *static_cast<SIM_CWrap_*>(this) = static_cast<const SIM_CWrap_&>(other);
    *static_cast<SIM_sWrap*>(this) = static_cast<const SIM_sWrap&>(other);
    obj = nullptr;
  }
  PointToLocal();
  return *this;
}



void SIM_CWrap::PointToLocal() {
  spec.element = static_cast<sim_spec_element_t*>(this);
  spec.info = &info;
}

void SIM_CWrap::CopyFromSpec() {
  *static_cast<SIM_sWrap*>(this) = spec;
}

void SIM_CWrap::NameSpace(const sim_builder_model_t* m) {
  name = m->prefix + name + m->suffix;
  if (!sidesite.empty()) {
    sidesite = m->prefix + sidesite + m->suffix;
  }
}

void SIM_CWrap::Compile(void) {
  CopyFromSpec();
}

void SIM_CWrap::ResolveReferences(const sim_builder_model_t* m) {
  sim_builder_base_t *pside;

  // handle wrap object types
  switch (spec.type) {
    case SIM_WRAP_JOINT:                        // joint
      // find joint by name
      obj = m->FindObject(SIM_OBJ_JOINT, name);
      if (!obj) {
        throw sim_builder_error_t(this,
                       "joint '%s' not found in tendon %d, wrap %d",
                       name.c_str(), tendon->id, id);
      }

      break;

    case SIM_WRAP_SPHERE:                       // geom (cylinder type set here)
      // find geom by name
      obj = m->FindObject(SIM_OBJ_GEOM, name);
      if (!obj) {
        throw sim_builder_error_t(this,
                       "geom '%s' not found in tendon %d, wrap %d",
                       name.c_str(), tendon->id, id);
      }

      // set/check geom type
      if (((sim_builder_geom_t*)obj)->type == SIM_GEOM_CYLINDER) {
        spec.type = SIM_WRAP_CYLINDER;
      } else if (((sim_builder_geom_t*)obj)->type != SIM_GEOM_SPHERE) {
        throw sim_builder_error_t(this,
                       "geom '%s' in tendon %d, wrap %d is not sphere or cylinder",
                       name.c_str(), tendon->id, id);
      }

      // process side site
      if (!sidesite.empty()) {
        // find site by name
        pside = m->FindObject(SIM_OBJ_SITE, sidesite);
        if (!pside) {
          throw sim_builder_error_t(this,
                         "side site '%s' not found in tendon %d, wrap %d",
                         sidesite.c_str(), tendon->id, id);
        }

        // save side site id
        sideid = pside->id;
      }
      break;

    case SIM_WRAP_PULLEY:                       // pulley
      // make sure divisor is non-negative
      if (prm < 0) {
        throw sim_builder_error_t(this,
                       "pulley has negative divisor in tendon %d, wrap %d",
                       0, tendon->id, id);
      }

      break;

    case SIM_WRAP_SITE:                         // site
      // find site by name
      obj = m->FindObject(SIM_OBJ_SITE, name);
      if (!obj) {
        throw sim_builder_error_t(this, "site '%s' not found in wrap %d", name.c_str(), id);
      }
      break;

    default:                                  // SHOULD NOT OCCUR
      throw sim_builder_error_t(this, "unknown wrap type in tendon %d, wrap %d", 0, tendon->id, id);
  }
}



//------------------ class SIM_CActuator implementation ----------------------------------------------

// initialize defaults
SIM_CActuator::SIM_CActuator(sim_builder_model_t* _model, sim_builder_default_t* _def) {
  sim_spec_defaultActuator(&spec);
  elemtype = SIM_OBJ_ACTUATOR;

  // clear private variables
  ptarget = nullptr;
  spec_target_.clear();
  spec_slidersite_.clear();
  spec_refsite_.clear();
  spec_userdata_.clear();
  trnid[0] = trnid[1] = -1;

  // reset to default if given
  if (_def) {
    *this = _def->Actuator();
  }

  // set model, def
  model = _model;
  if (_model) compiler = &_model->spec.compiler;
  classname = _def ? _def->name : "main";

  // in case this actuator is not compiled
  CopyFromSpec();

  // point to local
  PointToLocal();

  // no previous state when an actuator is created
  actadr_ = -1;
  actdim_ = -1;
}



SIM_CActuator::SIM_CActuator(const SIM_CActuator& other) {
  *this = other;
}



SIM_CActuator& SIM_CActuator::operator=(const SIM_CActuator& other) {
  if (this != &other) {
    this->spec = other.spec;
    *static_cast<SIM_CActuator_*>(this) = static_cast<const SIM_CActuator_&>(other);
    *static_cast<sim_spec_actuator_t*>(this) = static_cast<const sim_spec_actuator_t&>(other);
    ptarget = nullptr;
  }
  PointToLocal();
  return *this;
}



void SIM_CActuator::ForgetKeyframes() {
  act_.clear();
  ctrl_.clear();
}



bool SIM_CActuator::is_ctrllimited() const {
  return islimited(ctrllimited, ctrlrange);
}
bool SIM_CActuator::is_forcelimited() const {
  return islimited(forcelimited, forcerange);
}
bool SIM_CActuator::is_actlimited() const {
  return islimited(actlimited, actrange);
}



std::vector<sim_scalar_t>& SIM_CActuator::act(const std::string& state_name) {
  if (act_.find(state_name) == act_.end()) {
    act_[state_name] = std::vector<sim_scalar_t>(model->nu, SIM_NAN);
  }
  return act_.at(state_name);
}



sim_scalar_t& SIM_CActuator::ctrl(const std::string& state_name) {
  if (ctrl_.find(state_name) == ctrl_.end()) {
    ctrl_[state_name] = SIM_NAN;
  }
  return ctrl_.at(state_name);
}



void SIM_CActuator::PointToLocal() {
  spec.element = static_cast<sim_spec_element_t*>(this);
  spec.userdata = &spec_userdata_;
  spec.target = &spec_target_;
  spec.refsite = &spec_refsite_;
  spec.slidersite = &spec_slidersite_;
  spec.plugin.plugin_name = &plugin_name;
  spec.plugin.name = &plugin_instance_name;
  spec.info = &info;
  userdata = nullptr;
  target = nullptr;
  refsite = nullptr;
  slidersite = nullptr;
}



void SIM_CActuator::NameSpace(const sim_builder_model_t* m) {
  sim_builder_base_t::NameSpace(m);
  if (!plugin_instance_name.empty()) {
    plugin_instance_name = m->prefix + plugin_instance_name + m->suffix;
  }
  if (!spec_target_.empty()) {
    spec_target_ = m->prefix + spec_target_ + m->suffix;
  }
  if (!spec_refsite_.empty()) {
    spec_refsite_ = m->prefix + spec_refsite_ + m->suffix;
  }
  if (!spec_slidersite_.empty()) {
    spec_slidersite_ = m->prefix + spec_slidersite_ + m->suffix;
  }
}



void SIM_CActuator::CopyFromSpec() {
  *static_cast<sim_spec_actuator_t*>(this) = spec;
  userdata_ = spec_userdata_;
  target_ = spec_target_;
  refsite_ = spec_refsite_;
  slidersite_ = spec_slidersite_;
  plugin.active = spec.plugin.active;
  plugin.element = spec.plugin.element;
  plugin.plugin_name = spec.plugin.plugin_name;
  plugin.name = spec.plugin.name;
}



void SIM_CActuator::CopyPlugin() {
  model->CopyExplicitPlugin(this);
}



void SIM_CActuator::ResolveReferences(const sim_builder_model_t* m) {
  switch (trntype) {
    case SIM_TRN_JOINT:
    case SIM_TRN_JOINTINPARENT:
      // get joint
      ptarget = m->FindObject(SIM_OBJ_JOINT, target_);
      if (!ptarget) {
        throw sim_builder_error_t(this,
                       "unknown transmission target '%s' for actuator id = %d", target_.c_str(), id);
      }
      break;

    case SIM_TRN_SLIDERCRANK:
      // get slidersite, copy in trnid[1]
      if (slidersite_.empty()) {
        throw sim_builder_error_t(this, "missing base site for slider-crank '%s' (id = %d)", name.c_str(), id);
      }
      ptarget = m->FindObject(SIM_OBJ_SITE, slidersite_);
      if (!ptarget) {
        throw sim_builder_error_t(this, "base site '%s' not found for actuator %d", slidersite_.c_str(), id);
      }
      trnid[1] = ptarget->id;

      // check cranklength
      if (cranklength <= 0) {
        throw sim_builder_error_t(this,
                       "crank length must be positive in actuator '%s' (id = %d)", name.c_str(), id);
      }

      // proceed with regular target
      ptarget = m->FindObject(SIM_OBJ_SITE, target_);
      break;

    case SIM_TRN_TENDON:
      // get tendon
      ptarget = m->FindObject(SIM_OBJ_TENDON, target_);
      break;

    case SIM_TRN_SITE:
      // get refsite, copy into trnid[1]
      if (!refsite_.empty()) {
        ptarget = m->FindObject(SIM_OBJ_SITE, refsite_);
        if (!ptarget) {
          throw sim_builder_error_t(this, "reference site '%s' not found for actuator %d", refsite_.c_str(), id);
        }
        trnid[1] = ptarget->id;
      }

      // proceed with regular site target
      ptarget = m->FindObject(SIM_OBJ_SITE, target_);
      break;

    case SIM_TRN_BODY:
      // get body
      ptarget = m->FindObject(SIM_OBJ_BODY, target_);
      break;

    default:
      throw sim_builder_error_t(this, "invalid transmission type in actuator");
  }

  // assign and check
  if (!ptarget) {
    throw sim_builder_error_t(this, "transmission target '%s' not found in actuator %d", target_.c_str(), id);
  } else {
    trnid[0] = ptarget->id;
  }
}



// compiler
void SIM_CActuator::Compile(void) {
  CopyFromSpec();

  // resize userdata
  if (userdata_.size() > model->nuser_actuator) {
    throw sim_builder_error_t(this, "user has more values than nuser_actuator in actuator '%s' (id = %d)",
                   name.c_str(), id);
  }
  userdata_.resize(model->nuser_actuator);

  // check for missing target name
  if (target_.empty()) {
    throw sim_builder_error_t(this,
                   "missing transmission target for actuator");
  }

  // find transmission target in object arrays
  ResolveReferences(model);

  // handle inheritrange
  if (gaintype == SIM_GAIN_FIXED && biastype == SIM_BIAS_AFFINE &&
      gainprm[0] == -biasprm[1] && inheritrange > 0) {
    // semantic of actuator is the same as transmission, inheritrange is applicable
    double* range;
    if (dyntype == SIM_DYN_NONE || dyntype == SIM_DYN_FILTEREXACT) {
      // position actuator
      range = ctrlrange;
    } else if (dyntype == SIM_DYN_INTEGRATOR) {
      // intvelocity actuator
      range = actrange;
    } else {
      throw sim_builder_error_t(this, "inheritrange only available for position "
                     "and intvelocity actuators");
    }

    const double* target_range;
    if (trntype == SIM_TRN_JOINT) {
      sim_builder_joint_t* pjnt = (sim_builder_joint_t*) ptarget;
      if (pjnt->spec.type != SIM_JNT_HINGE && pjnt->spec.type != SIM_JNT_SLIDE) {
        throw sim_builder_error_t(this, "inheritrange can only be used with hinge and slide joints, "
                       "actuator");
      }
      target_range = pjnt->get_range();
    } else if (trntype == SIM_TRN_TENDON) {
      sim_builder_tendon_t* pten = (sim_builder_tendon_t*) ptarget;
      target_range = pten->get_range();
    } else {
      throw sim_builder_error_t(this, "inheritrange can only be used with joint and tendon transmission, "
                     "actuator");
    }

    if (target_range[0] == target_range[1]) {
      throw sim_builder_error_t(this, "inheritrange used but target '%s' has no range defined in actuator %d",
                     target_.c_str(), id);
    }

    // set range automatically
    double mean   = 0.5*(target_range[1] + target_range[0]);
    double radius = 0.5*(target_range[1] - target_range[0]) * inheritrange;
    range[0] = mean - radius;
    range[1] = mean + radius;
  }

  // if limited is auto, check for inconsistency wrt to autolimits
  if (forcelimited == SIM_LIMITED_AUTO) {
    bool hasrange = !(forcerange[0] == 0 && forcerange[1] == 0);
    checklimited(this, compiler->autolimits, "actuator", "force", forcelimited, hasrange);
  }
  if (ctrllimited == SIM_LIMITED_AUTO) {
    bool hasrange = !(ctrlrange[0] == 0 && ctrlrange[1] == 0);
    checklimited(this, compiler->autolimits, "actuator", "ctrl", ctrllimited, hasrange);
  }
  if (actlimited == SIM_LIMITED_AUTO) {
    bool hasrange = !(actrange[0] == 0 && actrange[1] == 0);
    checklimited(this, compiler->autolimits, "actuator", "act", actlimited, hasrange);
  }

  // check limits
  if (forcerange[0] >= forcerange[1] && is_forcelimited()) {
    throw sim_builder_error_t(this, "invalid force range for actuator");
  }
  if (ctrlrange[0] >= ctrlrange[1] && is_ctrllimited()) {
    throw sim_builder_error_t(this, "invalid control range for actuator");
  }
  if (actrange[0] >= actrange[1] && is_actlimited()) {
    throw sim_builder_error_t(this, "invalid actrange for actuator");
  }
  if (is_actlimited() && dyntype == SIM_DYN_NONE) {
    throw sim_builder_error_t(this, "actrange specified but dyntype is 'none' in actuator");
  }

  // check and set actdim
  if (!plugin.active) {
    if (actdim > 1 && dyntype != SIM_DYN_USER) {
      throw sim_builder_error_t(this, "actdim > 1 is only allowed for dyntype 'user' in actuator");
    }
    if (actdim == 1 && dyntype == SIM_DYN_NONE) {
      throw sim_builder_error_t(this, "invalid actdim 1 in stateless actuator");
    }
    if (actdim == 0 && dyntype != SIM_DYN_NONE) {
      throw sim_builder_error_t(this, "invalid actdim 0 in stateful actuator");
    }
  }

  // set actdim
  if (actdim < 0) {
    actdim = (dyntype != SIM_DYN_NONE);
  }

  // check muscle parameters
  for (int i=0; i < 2; i++) {
    // select gain or bias
    double* prm = NULL;
    if (i == 0 && gaintype == SIM_GAIN_MUSCLE) {
      prm = gainprm;
    } else if (i == 1 && biastype == SIM_BIAS_MUSCLE) {
      prm = biasprm;
    }

    // nothing to check
    if (!prm) {
      continue;
    }

    // range
    if (prm[0] >= prm[1]) {
      throw sim_builder_error_t(this, "range[0]<range[1] required in muscle");
    }

    // lmin<1<lmax
    if (prm[4] >= 1 || prm[5] <= 1) {
      throw sim_builder_error_t(this, "lmin<1<lmax required in muscle");
    }

    // scale, vmax, fpmax, fvmax>0
    if (prm[3] <= 0 || prm[6] <= 0 || prm[7] <= 0 || prm[8] <= 0) {
      throw sim_builder_error_t(this,
                     "positive scale, vmax, fpmax, fvmax required in muscle '%s' (id = %d)",
                     name.c_str(), id);
    }
  }

  // plugin
  if (plugin.active) {
    if (plugin_name.empty() && plugin_instance_name.empty()) {
      throw sim_builder_error_t(
              this, "neither 'plugin' nor 'instance' is specified for actuator '%s', (id = %d)",
              name.c_str(), id);
    }

    sim_builder_plugin_t* plugin_instance = static_cast<sim_builder_plugin_t*>(plugin.element);
    model->ResolvePlugin(this, plugin_name, plugin_instance_name, &plugin_instance);
    plugin.element = plugin_instance;
    const SIM_pPlugin* pplugin = sim_plugin_getPluginAtSlot(plugin_instance->plugin_slot);
    if (!(pplugin->capabilityflags & SIM_PLUGIN_ACTUATOR)) {
      throw sim_builder_error_t(this, "plugin '%s' does not support actuators", pplugin->name);
    }
  }

  // validate delay
  if (delay > 0 && nsample <= 0) {
    throw sim_builder_error_t(this, "setting delay > 0 without a history buffer");
  }

  // nsample is limited to 2^24 because the cursor is stored as an sim_scalar_t, which may be a float
  // single-precision floats can represent all integers up to 2^24 exactly
  if (nsample > 16777216) {
    throw sim_builder_error_t(this, "at most 2^24 samples in history buffer, got %d", nullptr, nsample);
  }
}



//------------------ class SIM_CSensor implementation ------------------------------------------------

// initialize defaults
SIM_CSensor::SIM_CSensor(sim_builder_model_t* _model) {
  sim_spec_defaultSensor(&spec);
  elemtype = SIM_OBJ_SENSOR;

  // set model
  model = _model;
  if (_model) compiler = &_model->spec.compiler;

  // clear private variables
  spec_objname_.clear();
  spec_refname_.clear();
  spec_userdata_.clear();
  obj = nullptr;
  ref = nullptr;

  // in case this sensor is not compiled
  CopyFromSpec();

  // point to local
  PointToLocal();
}



SIM_CSensor::SIM_CSensor(const SIM_CSensor& other) {
  *this = other;
}



SIM_CSensor& SIM_CSensor::operator=(const SIM_CSensor& other) {
  if (this != &other) {
    this->spec = other.spec;
    *static_cast<SIM_CSensor_*>(this) = static_cast<const SIM_CSensor_&>(other);
    *static_cast<SIM_sSensor*>(this) = static_cast<const SIM_sSensor&>(other);
    obj = nullptr;
    ref = nullptr;
  }
  PointToLocal();
  return *this;
}



void SIM_CSensor::PointToLocal() {
  spec.element = static_cast<sim_spec_element_t*>(this);
  spec.userdata = &spec_userdata_;
  spec.objname = &spec_objname_;
  spec.refname = &spec_refname_;
  spec.plugin.plugin_name = &plugin_name;
  spec.plugin.name = &plugin_instance_name;
  spec.info = &info;
  userdata = nullptr;
  objname = nullptr;
  refname = nullptr;
}



void SIM_CSensor::NameSpace(const sim_builder_model_t* m) {
  if (!name.empty()) {
    name = m->prefix + name + m->suffix;
  }
  if (!plugin_instance_name.empty()) {
    plugin_instance_name = m->prefix + plugin_instance_name + m->suffix;
  }
  prefix = m->prefix;
  suffix = m->suffix;
}



void SIM_CSensor::CopyFromSpec() {
  *static_cast<SIM_sSensor*>(this) = spec;
  userdata_ = spec_userdata_;
  objname_ = spec_objname_;
  refname_ = spec_refname_;
  plugin.active = spec.plugin.active;
  plugin.element = spec.plugin.element;
  plugin.plugin_name = spec.plugin.plugin_name;
  plugin.name = spec.plugin.name;
}



void SIM_CSensor::CopyPlugin() {
  model->CopyExplicitPlugin(this);
}



void SIM_CSensor::ResolveReferences(const sim_builder_model_t* m) {
  obj = nullptr;
  ref = nullptr;
  objname_ = prefix + objname_ + suffix;
  refname_ = prefix + refname_ + suffix;

  // get references using the namespace
  if (objtype != SIM_OBJ_UNKNOWN) {
    obj = m->FindObject(objtype, objname_);
  }
  if (reftype != SIM_OBJ_UNKNOWN) {
    ref = m->FindObject(reftype, refname_);
  }

  // if failure and both were requested, use namespace only on one
  if (objtype != SIM_OBJ_UNKNOWN && reftype != SIM_OBJ_UNKNOWN && !obj && ref) {
    objname_ = spec_objname_;
    obj = m->FindObject(objtype, objname_);
  }
  if (objtype != SIM_OBJ_UNKNOWN && reftype != SIM_OBJ_UNKNOWN && obj && !ref) {
    refname_ = spec_refname_;
    ref = m->FindObject(reftype, refname_);
  }

  // check object
  if (objtype != SIM_OBJ_UNKNOWN) {
    // check for missing object name
    if (objname_.empty()) {
      throw sim_builder_error_t(this, "missing name of sensorized object in sensor");
    }

    // find name
    if (!obj) {
      throw sim_builder_error_t(this, "unrecognized name '%s' of sensorized object", objname_.c_str());
    }

    // if geom or mesh, mark it as non visual
    if (objtype == SIM_OBJ_GEOM) {
      static_cast<sim_builder_geom_t*>(obj)->SetNotVisual();
    }
    if (objtype == SIM_OBJ_MESH) {
      static_cast<sim_builder_mesh_t*>(obj)->SetNotVisual();
    }

  } else if (type != SIM_SENS_E_POTENTIAL &&
             type != SIM_SENS_E_KINETIC   &&
             type != SIM_SENS_CLOCK       &&
             type != SIM_SENS_PLUGIN      &&
             type != SIM_SENS_CONTACT     &&
             type != SIM_SENS_USER) {
    throw sim_builder_error_t(this, "invalid type in sensor");
  }

  // check reference object
  if (reftype != SIM_OBJ_UNKNOWN) {
    // check for missing object name
    if (refname_.empty()) {
      throw sim_builder_error_t(this, "missing name of reference frame object in sensor");
    }

    // find name
    if (!ref) {
      throw sim_builder_error_t(this, "unrecognized name '%s' of object", refname_.c_str());
    }

    // if geom or mesh, mark it as non visual
    if (reftype == SIM_OBJ_GEOM) {
      static_cast<sim_builder_geom_t*>(ref)->SetNotVisual();
    }
    if (reftype == SIM_OBJ_MESH) {
      static_cast<sim_builder_mesh_t*>(ref)->SetNotVisual();
    }

    // must be attached to object with spatial frame
    if (reftype != SIM_OBJ_BODY && reftype != SIM_OBJ_XBODY &&
        reftype != SIM_OBJ_GEOM && reftype != SIM_OBJ_SITE && reftype != SIM_OBJ_CAMERA) {
      throw sim_builder_error_t(this,
                     "reference frame object must be (x)body, geom, site or camera in sensor");
    }
  }

  spec_objname_ = objname_;
  spec_refname_ = refname_;
  prefix.clear();
  suffix.clear();
}

// return sensor datatype
SIM_tDataType sensorDatatype(SIM_tSensor type) {
  switch (type) {
  case SIM_SENS_TOUCH:
  case SIM_SENS_INSIDESITE:
    return SIM_DATATYPE_POSITIVE;

  case SIM_SENS_FRAMEXAXIS:
  case SIM_SENS_FRAMEYAXIS:
  case SIM_SENS_FRAMEZAXIS:
  case SIM_SENS_GEOMNORMAL:
    return SIM_DATATYPE_AXIS;

  case SIM_SENS_BALLQUAT:
  case SIM_SENS_FRAMEQUAT:
    return SIM_DATATYPE_QUATERNION;

  case SIM_SENS_ACCELEROMETER:
  case SIM_SENS_VELOCIMETER:
  case SIM_SENS_GYRO:
  case SIM_SENS_FORCE:
  case SIM_SENS_TORQUE:
  case SIM_SENS_MAGNETOMETER:
  case SIM_SENS_CAMPROJECTION:
  case SIM_SENS_JOINTPOS:
  case SIM_SENS_JOINTVEL:
  case SIM_SENS_TENDONPOS:
  case SIM_SENS_TENDONVEL:
  case SIM_SENS_ACTUATORPOS:
  case SIM_SENS_ACTUATORVEL:
  case SIM_SENS_ACTUATORFRC:
  case SIM_SENS_JOINTACTFRC:
  case SIM_SENS_TENDONACTFRC:
  case SIM_SENS_BALLANGVEL:
  case SIM_SENS_JOINTLIMITPOS:
  case SIM_SENS_JOINTLIMITVEL:
  case SIM_SENS_JOINTLIMITFRC:
  case SIM_SENS_TENDONLIMITPOS:
  case SIM_SENS_TENDONLIMITVEL:
  case SIM_SENS_TENDONLIMITFRC:
  case SIM_SENS_FRAMEPOS:
  case SIM_SENS_FRAMELINVEL:
  case SIM_SENS_FRAMEANGVEL:
  case SIM_SENS_FRAMELINACC:
  case SIM_SENS_FRAMEANGACC:
  case SIM_SENS_SUBTREECOM:
  case SIM_SENS_SUBTREELINVEL:
  case SIM_SENS_SUBTREEANGMOM:
  case SIM_SENS_GEOMDIST:
  case SIM_SENS_GEOMFROMTO:
  case SIM_SENS_RANGEFINDER:
  case SIM_SENS_CONTACT:
  case SIM_SENS_TACTILE:
  case SIM_SENS_E_POTENTIAL:
  case SIM_SENS_E_KINETIC:
  case SIM_SENS_CLOCK:
  case SIM_SENS_PLUGIN:
  case SIM_SENS_USER:
    return SIM_DATATYPE_REAL;
  }

  return SIM_DATATYPE_REAL;  // all cases are covered but GCC is extra persnickety
}

// return sensor needstage
SIM_tStage sensorNeedstage(SIM_tSensor type) {
  switch (type) {
  case SIM_SENS_TOUCH:
  case SIM_SENS_ACCELEROMETER:
  case SIM_SENS_FORCE:
  case SIM_SENS_TORQUE:
  case SIM_SENS_ACTUATORFRC:
  case SIM_SENS_JOINTACTFRC:
  case SIM_SENS_TENDONACTFRC:
  case SIM_SENS_JOINTLIMITFRC:
  case SIM_SENS_TENDONLIMITFRC:
  case SIM_SENS_FRAMELINACC:
  case SIM_SENS_FRAMEANGACC:
  case SIM_SENS_CONTACT:
  case SIM_SENS_TACTILE:
    return SIM_STAGE_ACC;

  case SIM_SENS_VELOCIMETER:
  case SIM_SENS_GYRO:
  case SIM_SENS_JOINTVEL:
  case SIM_SENS_TENDONVEL:
  case SIM_SENS_ACTUATORVEL:
  case SIM_SENS_BALLANGVEL:
  case SIM_SENS_JOINTLIMITVEL:
  case SIM_SENS_TENDONLIMITVEL:
  case SIM_SENS_FRAMELINVEL:
  case SIM_SENS_FRAMEANGVEL:
  case SIM_SENS_SUBTREELINVEL:
  case SIM_SENS_SUBTREEANGMOM:
    return SIM_STAGE_VEL;

  case SIM_SENS_MAGNETOMETER:
  case SIM_SENS_RANGEFINDER:
  case SIM_SENS_CAMPROJECTION:
  case SIM_SENS_JOINTPOS:
  case SIM_SENS_TENDONPOS:
  case SIM_SENS_ACTUATORPOS:
  case SIM_SENS_BALLQUAT:
  case SIM_SENS_JOINTLIMITPOS:
  case SIM_SENS_TENDONLIMITPOS:
  case SIM_SENS_FRAMEPOS:
  case SIM_SENS_FRAMEQUAT:
  case SIM_SENS_FRAMEXAXIS:
  case SIM_SENS_FRAMEYAXIS:
  case SIM_SENS_FRAMEZAXIS:
  case SIM_SENS_SUBTREECOM:
  case SIM_SENS_INSIDESITE:
  case SIM_SENS_GEOMDIST:
  case SIM_SENS_GEOMNORMAL:
  case SIM_SENS_GEOMFROMTO:
  case SIM_SENS_E_POTENTIAL:
  case SIM_SENS_E_KINETIC:
  case SIM_SENS_CLOCK:
  case SIM_SENS_PLUGIN:
  case SIM_SENS_USER:
    return SIM_STAGE_POS;
  }

  return SIM_STAGE_POS;  // all cases are covered but GCC is extra persnickety
}

// compiler
void SIM_CSensor::Compile(void) {
  CopyFromSpec();

  // resize userdata
  if (userdata_.size() > model->nuser_sensor) {
    throw sim_builder_error_t(this, "user has more values than nuser_sensor in sensor");
  }
  userdata_.resize(model->nuser_sensor);

  // require non-negative noise
  if (noise < 0) {
    throw sim_builder_error_t(this, "negative noise in sensor");
  }

  // require non-negative cutoff
  if (cutoff < 0) {
    throw sim_builder_error_t(this, "negative cutoff in sensor");
  }

  // require non-negative interval
  if (interval[0] < 0) {
    throw sim_builder_error_t(this, "negative interval in sensor");
  }

  // require non-positive phase
  if (interval[1] > 0) {
    throw sim_builder_error_t(this, "positive phase in sensor");
  }

  // require phase > -period (values outside this are equivalent modulo period)
  if (interval[0] > 0 && interval[1] <= -interval[0]) {
    throw sim_builder_error_t(this, "phase must be greater than -period in sensor");
  }

  // require nsample for delay
  if (delay > 0 && nsample <= 0) {
    throw sim_builder_error_t(this, "setting delay > 0 without a history buffer");
  }

  // validate nsample size (max 2^24)
  if (nsample > 16777216) {
    throw sim_builder_error_t(this, "at most 2^24 samples in sensor history buffer, got %d", nullptr, nsample);
  }

  // Find referenced object
  ResolveReferences(model);

  // set datatype for non-user sensors
  if (type != SIM_SENS_USER) {
    datatype = sensorDatatype(type);
  }

  // set needstage for non-user and non-plugin sensors
  if (type != SIM_SENS_USER && type != SIM_SENS_PLUGIN) {
    needstage = sensorNeedstage(type);
  }

  // process according to sensor type
  switch (type) {
    case SIM_SENS_TOUCH:
    case SIM_SENS_ACCELEROMETER:
    case SIM_SENS_VELOCIMETER:
    case SIM_SENS_GYRO:
    case SIM_SENS_FORCE:
    case SIM_SENS_TORQUE:
    case SIM_SENS_MAGNETOMETER:
    case SIM_SENS_CAMPROJECTION:
      // must be attached to site
      if (objtype != SIM_OBJ_SITE) {
        throw sim_builder_error_t(this, "sensor must be attached to site");
      }

      // check for camera resolution for camera projection sensor
      if (type == SIM_SENS_CAMPROJECTION) {
        SIM_CCamera* camref = (SIM_CCamera*)ref;
        if (!camref->resolution[0] || !camref->resolution[1]) {
          throw sim_builder_error_t(this, "camera projection sensor requires camera resolution");
        }
      }
      break;

    case SIM_SENS_RANGEFINDER:
      {
        // must be attached to site or camera
        if (objtype != SIM_OBJ_SITE && objtype != SIM_OBJ_CAMERA) {
          throw sim_builder_error_t(this, "sensor must be attached to site or camera");
        }

        // check for dataspec correctness
        int dataspec = intprm[0];
        if (dataspec <= 0) {
          throw sim_builder_error_t(this, "data spec (intprm[0]) must be positive, got %d", nullptr, dataspec);
        }
        int mask = (1 << SIM_NRAYDATA) - 1;
        if (!(dataspec & mask)) {
          throw sim_builder_error_t(this, "data spec intprm[0]=%d must have at least one bit set of the first "
                         "SIM_NRAYDATA bits", nullptr, dataspec);
        }
        if (dataspec & ~mask) {
          throw sim_builder_error_t(this, "data spec intprm[0]=%d has bits set beyond the first "
                         "SIM_NRAYDATA bits", nullptr, dataspec);
        }
      }
      break;

    case SIM_SENS_JOINTPOS:
    case SIM_SENS_JOINTVEL:
    case SIM_SENS_JOINTACTFRC:
      // must be attached to joint
      if (objtype != SIM_OBJ_JOINT) {
        throw sim_builder_error_t(this, "sensor must be attached to joint");
      }

      // make sure joint is slide or hinge
      if (((sim_builder_joint_t*)obj)->type != SIM_JNT_SLIDE && ((sim_builder_joint_t*)obj)->type != SIM_JNT_HINGE) {
        throw sim_builder_error_t(this, "joint must be slide or hinge in sensor");
      }
      break;

  case SIM_SENS_TENDONACTFRC:
    // must be attached to tendon
    if (objtype != SIM_OBJ_TENDON) {
      throw sim_builder_error_t(this, "sensor must be attached to tendon");
    }
    break;

    case SIM_SENS_TENDONPOS:
    case SIM_SENS_TENDONVEL:
      // must be attached to tendon
      if (objtype != SIM_OBJ_TENDON) {
        throw sim_builder_error_t(this, "sensor must be attached to tendon");
      }
      break;

    case SIM_SENS_ACTUATORPOS:
    case SIM_SENS_ACTUATORVEL:
    case SIM_SENS_ACTUATORFRC:
      // must be attached to actuator
      if (objtype != SIM_OBJ_ACTUATOR) {
        throw sim_builder_error_t(this, "sensor must be attached to actuator");
      }
      break;

    case SIM_SENS_BALLQUAT:
    case SIM_SENS_BALLANGVEL:
      // must be attached to joint
      if (objtype != SIM_OBJ_JOINT) {
        throw sim_builder_error_t(this, "sensor must be attached to joint");
      }

      // make sure joint is ball
      if (((sim_builder_joint_t*)obj)->type != SIM_JNT_BALL) {
        throw sim_builder_error_t(this, "joint must be ball in sensor");
      }
      break;

    case SIM_SENS_JOINTLIMITPOS:
    case SIM_SENS_JOINTLIMITVEL:
    case SIM_SENS_JOINTLIMITFRC:
      // must be attached to joint
      if (objtype != SIM_OBJ_JOINT) {
        throw sim_builder_error_t(this, "sensor must be attached to joint");
      }

      // make sure joint has limit
      if (!((sim_builder_joint_t*)obj)->is_limited()) {
        throw sim_builder_error_t(this, "joint must be limited in sensor");
      }
      break;

    case SIM_SENS_TENDONLIMITPOS:
    case SIM_SENS_TENDONLIMITVEL:
    case SIM_SENS_TENDONLIMITFRC:
      // must be attached to tendon
      if (objtype != SIM_OBJ_TENDON) {
        throw sim_builder_error_t(this, "sensor must be attached to tendon");
      }

      // make sure tendon has limit
      if (!((sim_builder_tendon_t*)obj)->is_limited()) {
        throw sim_builder_error_t(this, "tendon must be limited in sensor");
      }
      break;

    case SIM_SENS_FRAMEPOS:
    case SIM_SENS_FRAMEQUAT:
    case SIM_SENS_FRAMEXAXIS:
    case SIM_SENS_FRAMEYAXIS:
    case SIM_SENS_FRAMEZAXIS:
    case SIM_SENS_FRAMELINVEL:
    case SIM_SENS_FRAMEANGVEL:
    case SIM_SENS_FRAMELINACC:
    case SIM_SENS_FRAMEANGACC:
      // must be attached to object with spatial frame
      if (objtype != SIM_OBJ_BODY && objtype != SIM_OBJ_XBODY &&
          objtype != SIM_OBJ_GEOM && objtype != SIM_OBJ_SITE && objtype != SIM_OBJ_CAMERA) {
        throw sim_builder_error_t(this, "sensor must be attached to (x)body, geom, site or camera");
      }
      break;

    case SIM_SENS_SUBTREECOM:
    case SIM_SENS_SUBTREELINVEL:
    case SIM_SENS_SUBTREEANGMOM:
      // must be attached to body
      if (objtype != SIM_OBJ_BODY) {
        throw sim_builder_error_t(this, "sensor must be attached to body");
      }
      break;

    case SIM_SENS_INSIDESITE:
      if (objtype != SIM_OBJ_BODY && objtype != SIM_OBJ_XBODY &&
          objtype != SIM_OBJ_GEOM && objtype != SIM_OBJ_SITE && objtype != SIM_OBJ_CAMERA) {
        throw sim_builder_error_t(this, "sensor must be attached to (x)body, geom, site or camera");
      }
      if (reftype != SIM_OBJ_SITE) {
        throw sim_builder_error_t(this, "sensor must be associated with a site");
      }
      break;

    case SIM_SENS_GEOMDIST:
    case SIM_SENS_GEOMNORMAL:
    case SIM_SENS_GEOMFROMTO:
      // must be attached to body or geom
      if ((objtype != SIM_OBJ_BODY && objtype != SIM_OBJ_GEOM) ||
          (reftype != SIM_OBJ_BODY && reftype != SIM_OBJ_GEOM)) {
        throw sim_builder_error_t(this, "sensor must be attached to body or geom");
      }

      // objects must be different
      if (objtype == reftype && obj == ref) {
        throw sim_builder_error_t(this, "1st body/geom must be different from 2nd body/geom");
      }

      // height fields are not necessarily convex and are not yet supported
      if ((objtype == SIM_OBJ_GEOM && static_cast<sim_builder_geom_t*>(obj)->Type() == SIM_GEOM_HFIELD) ||
          (reftype == SIM_OBJ_GEOM && static_cast<sim_builder_geom_t*>(ref)->Type() == SIM_GEOM_HFIELD)) {
        throw sim_builder_error_t(this, "height fields are not supported in geom distance sensors");
      }
      break;

    case SIM_SENS_CONTACT:
      {
        // check first matching criterion
        if (objtype != SIM_OBJ_SITE &&
            objtype != SIM_OBJ_BODY &&
            objtype != SIM_OBJ_XBODY &&
            objtype != SIM_OBJ_GEOM &&
            objtype != SIM_OBJ_UNKNOWN) {
          throw sim_builder_error_t(this, "first matching criterion: if set, must be (x)body, geom or site");
        }

        // check second matching criterion
        if (reftype != SIM_OBJ_BODY &&
            reftype != SIM_OBJ_XBODY &&
            reftype != SIM_OBJ_GEOM &&
            reftype != SIM_OBJ_UNKNOWN) {
          throw sim_builder_error_t(this, "second matching criterion: if set, must be (x)body or geom");
        }

        // check for dataspec correctness
        int dataspec = intprm[0];
        if (dataspec <= 0) {
          throw sim_builder_error_t(this, "data spec (intprm[0]) must be positive, got %d", nullptr, dataspec);
        }
        int mask = (1 << SIM_NCONDATA) - 1;
        if (!(dataspec & mask)) {
          throw sim_builder_error_t(this, "data spec intprm[0]=%d must have at least one bit set of the first "
                         "SIM_NCONDATA bits", nullptr, dataspec);
        }
        if (dataspec & ~mask) {
          throw sim_builder_error_t(this, "data spec intprm[0]=%d has bits set beyond the first "
                         "SIM_NCONDATA bits", nullptr, dataspec);
        }

        // check for reduce correctness
        int reduce = intprm[1];
        if (reduce < 0 || reduce > 3) {
          throw sim_builder_error_t(this, "unknown reduction criterion. got %d, "
                         "expected one of {0, 1, 2, 3}", nullptr, reduce);
        }

        // check for non-positive num
        if (intprm[2] <= 0) {
          throw sim_builder_error_t(this, "num (intprm[2]) must be positive in sensor, got %d", nullptr, dim);
        }
      }
      break;

    case SIM_SENS_E_POTENTIAL:
    case SIM_SENS_E_KINETIC:
    case SIM_SENS_CLOCK:
      break;

    case SIM_SENS_USER:
      // check for negative dim
      if (dim < 0) {
        throw sim_builder_error_t(this, "sensor dim must be non-negative in sensor");
      }

      // make sure dim is consistent with datatype
      if (datatype == SIM_DATATYPE_AXIS && dim != 3) {
        throw sim_builder_error_t(this, "datatype AXIS requires dim=3 in sensor");
      }
      if (datatype == SIM_DATATYPE_QUATERNION && dim != 4) {
        throw sim_builder_error_t(this, "datatype QUATERNION requires dim=4 in sensor");
      }
      break;

    case SIM_SENS_TACTILE:
      if (objtype != SIM_OBJ_MESH) {
        throw sim_builder_error_t(this, "sensor must be associated with a mesh");
      }
      if (reftype != SIM_OBJ_GEOM) {
        throw sim_builder_error_t(this, "sensor must be associated with a geom");
      }
      break;

    case SIM_SENS_PLUGIN:
      if (plugin_name.empty() && plugin_instance_name.empty()) {
        throw sim_builder_error_t(this, "neither 'plugin' nor 'instance' is specified for sensor");
      }

      // resolve plugin instance, or create one if using the "plugin" attribute shortcut
      {
        sim_builder_plugin_t* plugin_instance = static_cast<sim_builder_plugin_t*>(plugin.element);
        model->ResolvePlugin(this, plugin_name, plugin_instance_name, &plugin_instance);
        plugin.element = plugin_instance;
        const SIM_pPlugin* pplugin = sim_plugin_getPluginAtSlot(plugin_instance->plugin_slot);
        if (!(pplugin->capabilityflags & SIM_PLUGIN_SENSOR)) {
          throw sim_builder_error_t(this, "plugin '%s' does not support sensors", pplugin->name);
        }
        needstage = static_cast<SIM_tStage>(pplugin->needstage);
      }

      break;

    default:
      throw sim_builder_error_t(this, "invalid type in sensor '%s' (id = %d)", name.c_str(), id);
  }

  dim = sim_spec_sensorDim(this);

  // check cutoff for incompatible data types
  if (cutoff > 0 && (datatype == SIM_DATATYPE_QUATERNION ||
                     (datatype == SIM_DATATYPE_AXIS && type != SIM_SENS_GEOMNORMAL))) {
    throw sim_builder_error_t(this, "cutoff applied to axis or quaternion datatype in sensor");
  }
}



//------------------ class SIM_CNumeric implementation -----------------------------------------------

// constructor
SIM_CNumeric::SIM_CNumeric(sim_builder_model_t* _model) {
  sim_spec_defaultNumeric(&spec);
  elemtype = SIM_OBJ_NUMERIC;

  // set model pointer
  model = _model;
  if (_model) compiler = &_model->spec.compiler;

  // clear variables
  spec_data_.clear();

  // point to local
  PointToLocal();

  // in case this numeric is not compiled
  CopyFromSpec();
}



SIM_CNumeric::SIM_CNumeric(const SIM_CNumeric& other) {
  *this = other;
}



SIM_CNumeric& SIM_CNumeric::operator=(const SIM_CNumeric& other) {
  if (this != &other) {
    this->spec = other.spec;
    *static_cast<SIM_CNumeric_*>(this) = static_cast<const SIM_CNumeric_&>(other);
    *static_cast<SIM_sNumeric*>(this) = static_cast<const SIM_sNumeric&>(other);
  }
  PointToLocal();
  return *this;
}



void SIM_CNumeric::PointToLocal() {
  spec.element = static_cast<sim_spec_element_t*>(this);
  spec.data = &spec_data_;
  spec.info = &info;
  data = nullptr;
}



void SIM_CNumeric::CopyFromSpec() {
  *static_cast<SIM_sNumeric*>(this) = spec;
  data_ = spec_data_;
}



// destructor
SIM_CNumeric::~SIM_CNumeric() {
  spec_data_.clear();
  data_.clear();
}



// compiler
void SIM_CNumeric::Compile(void) {
  CopyFromSpec();

  // check for size conflict
  if (size && !data_.empty() && size < (int)data_.size()) {
    throw sim_builder_error_t(this,
                   "numeric '%s' (id = %d): specified size smaller than initialization array",
                   name.c_str(), id);
  }

  // set size if left unspecified
  if (!size) {
    size = (int)data_.size();
  }

  // size cannot be zero
  if (!size) {
    throw sim_builder_error_t(this, "numeric '%s' (id = %d): size cannot be zero", name.c_str(), id);
  }
}



//------------------ class SIM_CText implementation --------------------------------------------------

// constructor
SIM_CText::SIM_CText(sim_builder_model_t* _model) {
  sim_spec_defaultText(&spec);
  elemtype = SIM_OBJ_TEXT;

  // set model pointer
  model = _model;
  if (_model) compiler = &_model->spec.compiler;

  // clear variables
  spec_data_.clear();

  // point to local
  PointToLocal();

  // in case this text is not compiled
  CopyFromSpec();
}



SIM_CText::SIM_CText(const SIM_CText& other) {
  *this = other;
}



SIM_CText& SIM_CText::operator=(const SIM_CText& other) {
  if (this != &other) {
    this->spec = other.spec;
    *static_cast<SIM_CText_*>(this) = static_cast<const SIM_CText_&>(other);
    *static_cast<SIM_sText*>(this) = static_cast<const SIM_sText&>(other);
  }
  PointToLocal();
  return *this;
}



void SIM_CText::PointToLocal() {
  spec.element = static_cast<sim_spec_element_t*>(this);
  spec.data = &spec_data_;
  spec.info = &info;
  data = nullptr;
}



void SIM_CText::CopyFromSpec() {
  *static_cast<SIM_sText*>(this) = spec;
  data_ = spec_data_;
}



// destructor
SIM_CText::~SIM_CText() {
  data_.clear();
  spec_data_.clear();
}



// compiler
void SIM_CText::Compile(void) {
  CopyFromSpec();

  // size cannot be zero
  if (data_.empty()) {
    throw sim_builder_error_t(this, "text '%s' (id = %d): size cannot be zero", name.c_str(), id);
  }
}



//------------------ class SIM_CTuple implementation -------------------------------------------------

// constructor
SIM_CTuple::SIM_CTuple(sim_builder_model_t* _model) {
  sim_spec_defaultTuple(&spec);
  elemtype = SIM_OBJ_TUPLE;

  // set model pointer
  model = _model;
  if (_model) compiler = &_model->spec.compiler;

  // clear variables
  spec_objtype_.clear();
  spec_objname_.clear();
  spec_objprm_.clear();
  obj.clear();

  // point to local
  PointToLocal();

  // in case this tuple is not compiled
  CopyFromSpec();
}



SIM_CTuple::SIM_CTuple(const SIM_CTuple& other) {
  *this = other;
}



SIM_CTuple& SIM_CTuple::operator=(const SIM_CTuple& other) {
  if (this != &other) {
    this->spec = other.spec;
    *static_cast<SIM_CTuple_*>(this) = static_cast<const SIM_CTuple_&>(other);
    *static_cast<SIM_sTuple*>(this) = static_cast<const SIM_sTuple&>(other);
  }
  PointToLocal();
  return *this;
}



void SIM_CTuple::PointToLocal() {
  spec.element = static_cast<sim_spec_element_t*>(this);
  spec.objtype = (SIM_IntVec*)&spec_objtype_;
  spec.objname = &spec_objname_;
  spec.objprm = &spec_objprm_;
  spec.info = &info;
  objname = nullptr;
  objprm = nullptr;
}



void SIM_CTuple::NameSpace(const sim_builder_model_t* m) {
  if (!name.empty()) {
    name = m->prefix + name + m->suffix;
  }
  for (int i=0; i < spec_objname_.size(); i++) {
    spec_objname_[i] = m->prefix + spec_objname_[i] + m->suffix;
  }
}



void SIM_CTuple::CopyFromSpec() {
  *static_cast<SIM_sTuple*>(this) = spec;
  objtype_ = spec_objtype_;
  objname_ = spec_objname_;
  objprm_ = spec_objprm_;
  objtype = (SIM_IntVec*)&objtype_;
}



// destructor
SIM_CTuple::~SIM_CTuple() {
  objtype_.clear();
  objname_.clear();
  objprm_.clear();
  spec_objtype_.clear();
  spec_objname_.clear();
  spec_objprm_.clear();
  obj.clear();
}



void SIM_CTuple::ResolveReferences(const sim_builder_model_t* m) {
  // check for empty tuple
  if (objtype_.empty()) {
    throw sim_builder_error_t(this, "tuple '%s' (id = %d) is empty", name.c_str(), id);
  }

  // check for size conflict
  if (objtype_.size() != objname_.size() || objtype_.size() != objprm_.size()) {
    throw sim_builder_error_t(this,
                   "tuple '%s' (id = %d) has object arrays with different sizes", name.c_str(), id);
  }

  // resize objid to correct size
  obj.resize(objtype_.size());

  // find objects, fill in ids
  for (int i=0; i < objtype_.size(); i++) {
    // find object by type and name
    sim_builder_base_t* res = m->FindObject(objtype_[i], objname_[i]);
    if (!res) {
      throw sim_builder_error_t(this, "unrecognized object '%s' in tuple %d", objname_[i].c_str(), id);
    }

    // if geom mark it as non visual
    if (objtype_[i] == SIM_OBJ_GEOM) {
      ((sim_builder_geom_t*)res)->SetNotVisual();
    }

    // assign id
    obj[i] = res;
  }
}



// compiler
void SIM_CTuple::Compile(void) {
  CopyFromSpec();
  ResolveReferences(model);
}



//------------------ class SIM_CKey implementation ---------------------------------------------------

// constructor
SIM_CKey::SIM_CKey(sim_builder_model_t* _model) {
  sim_spec_defaultKey(&spec);
  elemtype = SIM_OBJ_KEY;

  // set model pointer
  model = _model;
  if (_model) compiler = &_model->spec.compiler;

  // clear variables
  spec_qpos_.clear();
  spec_qvel_.clear();
  spec_act_.clear();
  spec_mpos_.clear();
  spec_mquat_.clear();
  spec_ctrl_.clear();

  // point to local
  PointToLocal();

  // in case this keyframe is not compiled
  CopyFromSpec();
}



SIM_CKey::SIM_CKey(const SIM_CKey& other) {
  *this = other;
}



SIM_CKey& SIM_CKey::operator=(const SIM_CKey& other) {
  if (this != &other) {
    this->spec = other.spec;
    *static_cast<SIM_CKey_*>(this) = static_cast<const SIM_CKey_&>(other);
    *static_cast<SIM_sKey*>(this) = static_cast<const SIM_sKey&>(other);
  }
  PointToLocal();
  return *this;
}



void SIM_CKey::PointToLocal() {
  spec.element = static_cast<sim_spec_element_t*>(this);
  spec.qpos = &spec_qpos_;
  spec.qvel = &spec_qvel_;
  spec.act = &spec_act_;
  spec.mpos = &spec_mpos_;
  spec.mquat = &spec_mquat_;
  spec.ctrl = &spec_ctrl_;
  spec.info = &info;
  qpos = nullptr;
  qvel = nullptr;
  act = nullptr;
  mpos = nullptr;
  mquat = nullptr;
  ctrl = nullptr;
}



void SIM_CKey::CopyFromSpec() {
  *static_cast<SIM_sKey*>(this) = spec;
  qpos_ = spec_qpos_;
  qvel_ = spec_qvel_;
  act_ = spec_act_;
  mpos_ = spec_mpos_;
  mquat_ = spec_mquat_;
  ctrl_ = spec_ctrl_;
}



// destructor
SIM_CKey::~SIM_CKey() {
  qpos_.clear();
  qvel_.clear();
  act_.clear();
  mpos_.clear();
  mquat_.clear();
  ctrl_.clear();
  spec_qpos_.clear();
  spec_qvel_.clear();
  spec_act_.clear();
  spec_mpos_.clear();
  spec_mquat_.clear();
  spec_ctrl_.clear();
}



// compiler
void SIM_CKey::Compile(const sim_model_t* m) {
  CopyFromSpec();

  // qpos: allocate or check size
  if (qpos_.empty()) {
    qpos_.resize(m->nq);
    for (int i=0; i < m->nq; i++) {
      qpos_[i] = (double)m->qpos0[i];
    }
  } else if (qpos_.size() != m->nq) {
    throw sim_builder_error_t(this, "keyframe '%s': invalid qpos size, expected %d, got %d",
                   name.c_str(), m->nq, qpos_.size());
  }

  // qvel: allocate or check size
  if (qvel_.empty()) {
    qvel_.resize(m->nv);
    for (int i=0; i < m->nv; i++) {
      qvel_[i] = 0;
    }
  } else if (qvel_.size() != m->nv) {
    throw sim_builder_error_t(this, "keyframe '%s': invalid qvel size, expected %d, got %d",
                   name.c_str(), m->nv, qvel_.size());
  }

  // act: allocate or check size
  if (act_.empty()) {
    act_.resize(m->na);
    for (int i=0; i < m->na; i++) {
      act_[i] = 0;
    }
  } else if (act_.size() != m->na) {
    throw sim_builder_error_t(this, "keyframe '%s': invalid act size, expected %d, got %d",
                   name.c_str(), m->na, act_.size());
  }

  // mpos: allocate or check size
  if (mpos_.empty()) {
    mpos_.resize(3*m->nmocap);
    if (m->nmocap) {
      for (int i=0; i < m->nbody; i++) {
        if (m->body_mocapid[i] >= 0) {
          int mocapid = m->body_mocapid[i];
          mpos_[3*mocapid]   = m->body_pos[3*i];
          mpos_[3*mocapid+1] = m->body_pos[3*i+1];
          mpos_[3*mocapid+2] = m->body_pos[3*i+2];
        }
      }
    }
  } else if (mpos_.size() != 3*m->nmocap) {
    throw sim_builder_error_t(this, "keyframe %d: invalid mpos size, expected length %d", nullptr, id, 3*m->nmocap);
  }

  // mquat: allocate or check size
  if (mquat_.empty()) {
    mquat_.resize(4*m->nmocap);
    if (m->nmocap) {
      for (int i=0; i < m->nbody; i++) {
        if (m->body_mocapid[i] >= 0) {
          int mocapid = m->body_mocapid[i];
          mquat_[4*mocapid]   = m->body_quat[4*i];
          mquat_[4*mocapid+1] = m->body_quat[4*i+1];
          mquat_[4*mocapid+2] = m->body_quat[4*i+2];
          mquat_[4*mocapid+3] = m->body_quat[4*i+3];
        }
      }
    }
  } else if (mquat_.size() != 4*m->nmocap) {
    throw sim_builder_error_t(this, "keyframe %d: invalid mquat size, expected length %d", nullptr, id, 4*m->nmocap);
  }

  // ctrl: allocate or check size
  if (ctrl_.empty()) {
    ctrl_.resize(m->nu);
    for (int i=0; i < m->nu; i++) {
      ctrl_[i] = 0;
    }
  } else if (ctrl_.size() != m->nu) {
    throw sim_builder_error_t(this, "keyframe %d: invalid ctrl size, expected length %d", nullptr, id, m->nu);
  }
}


//------------------ class sim_builder_plugin_t implementation ------------------------------------------------

// initialize defaults
sim_builder_plugin_t::sim_builder_plugin_t(sim_builder_model_t* _model) {
  name = "";
  nstate = -1;
  plugin_slot = -1;
  parent = this;
  model = _model;
  if (_model) compiler = &_model->spec.compiler;
  name.clear();
  plugin_name.clear();

  // public interface
  sim_spec_defaultPlugin(&spec);
  elemtype = SIM_OBJ_PLUGIN;
  spec.plugin_name = &plugin_name;
  spec.info = &info;

  PointToLocal();
}



sim_builder_plugin_t::sim_builder_plugin_t(const sim_builder_plugin_t& other) {
  *this = other;
  id = -1;
}



sim_builder_plugin_t& sim_builder_plugin_t::operator=(const sim_builder_plugin_t& other) {
  if (this != &other) {
    this->spec = other.spec;
    *static_cast<SIM_CPlugin_*>(this) = static_cast<const SIM_CPlugin_&>(other);
    parent = this;
    plugin_slot = other.plugin_slot;
  }
  PointToLocal();
  return *this;
}



void sim_builder_plugin_t::PointToLocal() {
  spec.element = static_cast<sim_spec_element_t*>(this);
  spec.name = &name;
  spec.info = &info;
}



// compiler
void sim_builder_plugin_t::Compile(void) {
  sim_builder_plugin_t* plugin_instance = this;
  model->ResolvePlugin(this, plugin_name, name, &plugin_instance);
  const SIM_pPlugin* plugin = sim_plugin_getPluginAtSlot(plugin_slot);

  // clear precompiled
  flattened_attributes.clear();
  std::map<std::string, std::string, std::less<> > config_attribs_copy = config_attribs;

  // concatenate all of the plugin's attribute values (as null-terminated strings) into
  // flattened_attributes, in the order declared in the SIM_pPlugin
  // each valid attribute found is appended to flattened_attributes and removed from xml_attributes
  for (int i = 0; i < plugin->nattribute; ++i) {
    std::string_view attr(plugin->attributes[i]);
    auto it = config_attribs_copy.find(attr);
    if (it == config_attribs_copy.end()) {
      flattened_attributes.push_back('\0');
    } else {
      auto original_size = flattened_attributes.size();
      flattened_attributes.resize(original_size + it->second.size() + 1);
      std::memcpy(&flattened_attributes[original_size], it->second.c_str(),
                  it->second.size() + 1);
      config_attribs_copy.erase(it);
    }
  }

  // if there are no attributes, add a null terminator
  if (plugin->nattribute == 0) {
    flattened_attributes.push_back('\0');
  }

  // anything left in xml_attributes at this stage is not a valid attribute
  if (!config_attribs_copy.empty()) {
    std::string error =
      "unrecognized attribute 'plugin:" + config_attribs_copy.begin()->first +
      "' for plugin " + std::string(plugin->name) + "'";
    throw sim_builder_error_t(parent, "%s", error.c_str());
  }
}
