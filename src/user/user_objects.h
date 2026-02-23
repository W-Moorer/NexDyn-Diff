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

#ifndef SIMCORE_SRC_USER_USER_OBJECTS_H_
#define SIMCORE_SRC_USER_USER_OBJECTS_H_

#include <stdbool.h>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <array>
#include <deque>
#include <functional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

#include <simcore/SIM_model.h>
#include <simcore/SIM_plugin.h>
#include <simcore/SIM_spec.h>
#include <simcore/SIM_tnum.h>
#include "user/user_cache.h"
#include "user/user_util.h"
#include <tiny_obj_loader.h>

using face_vertices_type =
    decltype(tinyobj::mesh_t::num_face_vertices)::value_type;

// forward declarations of all simC/X classes
class sim_builder_error_t;
class sim_builder_base_t;
class sim_builder_body_t;
class sim_builder_frame_t;
class sim_builder_joint_t;
class sim_builder_geom_t;
class sim_builder_site_t;
class SIM_CCamera;
class SIM_CLight;
class SIM_CHField;
class SIM_CFlex;
class sim_builder_mesh_t;
class SIM_CSkin;
class sim_builder_texture_t;
class SIM_CMaterial;
class SIM_CPair;
class SIM_CBodyPair;
class SIM_CEquality;
class sim_builder_tendon_t;
class SIM_CWrap;
class SIM_CActuator;
class SIM_CSensor;
class SIM_CNumeric;
class SIM_CText;
class SIM_CTuple;
class sim_builder_default_t;
class sim_builder_model_t;        // defined in user_model.h
class sim_xml_writer_t;       // defined in xml_native.h
class SIM_XURDF;         // defined in xml_urdf.h


//------------------------- helper constants, classes and functions --------------------------------

// number of positive size parameters for each geom type
const int SIM_GEOMINFO[SIM_NGEOMTYPES] = {3, 0, 1, 2, 3, 2, 3, 0};

// error information
class [[nodiscard]] sim_builder_error_t {
 public:
  sim_builder_error_t(const sim_builder_base_t* obj = 0,
           const char* msg = 0,
           const char* str = 0,
           int pos1 = 0,
           int pos2 = 0);

  char message[500];              // error message
  bool warning;                   // is this a warning instead of error
};

// alternative specifications of frame orientation
const char* ResolveOrientation(double* quat,             // set frame quat
                               bool degree,              // angle format: degree/radian
                               const char* sequence,     // euler sequence format: "xyz"
                               const SIM_sOrientation& orient);


//------------------------- class SIM_CBoundingVolumeHierarchy ---------------------------------------

// bounding volume
class SIM_CBoundingVolume {
 public:
  SIM_CBoundingVolume(int id, int contype, int conaffinity, const double pos[3],
                    const double quat[4], const double aabb[6]) : contype_(contype),
                      conaffinity_(conaffinity), idval_(id) {
                        std::copy(pos, pos + 3, pos_.begin());
                        std::copy(aabb, aabb + 6, aabb_.begin());
                        quat_set_ = quat != nullptr;
                        if (quat_set_) {
                          std::copy(quat, quat + 4, quat_.begin());
                        }
                      }

  SIM_CBoundingVolume(const int* id, int contype, int conaffinity, const double pos[3],
                    const double quat[4], const double aabb[6]) : contype_(contype),
                      conaffinity_(conaffinity), id_(id) {
                        std::copy(pos, pos + 3, pos_.begin());
                        std::copy(aabb, aabb + 6, aabb_.begin());
                        quat_set_ = quat != nullptr;
                        if (quat_set_) {
                          std::copy(quat, quat + 4, quat_.begin());
                        }
                      }

  int Contype() const { return contype_; }
  int Conaffinity() const { return conaffinity_; }
  const double* AABB() const { return aabb_.data(); }
  double AABB(int i) const { return aabb_[i]; }
  const double* Pos() const { return pos_.data(); }
  double Pos(int i) const { return pos_[i]; }
  const double* Quat() const { return  quat_set_ ? quat_.data() : nullptr; }
  const int* Id() const { return id_ ? id_ : &idval_; }

  void SetContype(int val) { contype_ = val; }
  void SetConaffinity(int val) { conaffinity_ = val; }
  void SetAABB(const double* aabb) { std::copy(aabb, aabb + 6, aabb_.begin()); }
  void SetPos(const double* pos) { std::copy(pos, pos + 3, pos_.begin()); }
  void SetQuat(const double* quat) {
    quat_set_ = true;
    std::copy(quat, quat + 4, quat_.begin());
  }
  void SetId(const int* id) { id_ = id; }
  void SetId(int val) { idval_ = val; }

 private:
  int contype_;                 // contact type
  int conaffinity_;             // contact affinity
  std::array<double, 6> aabb_;  // axis-aligned bounding box (center, size)
  std::array<double, 3> pos_;   // position (set by user or Compiler)
  std::array<double, 4> quat_;  // orientation (set by user or Compiler)
  bool quat_set_;               // boolean flag is quat_ has been set
  int idval_;                   // local id copy for nodes not storing their id's (e.g. faces)

  // pointer to object id
  const int* id_ = nullptr;
};


// bounding volume hierarchy
struct SIM_CBoundingVolumeHierarchy_ {
 protected:
  int nbvh_ = 0;
  std::vector<sim_scalar_t> bvh_;           // bounding boxes                          (nbvh x 6)
  std::vector<int> child_;            // children of each node                   (nbvh x 2)
  std::vector<int> nodeid_;           // id of elem contained by the node        (nbvh x 1)
  std::vector<int*> nodeidptr_;       // ptr to id of elem contained by the node (nbvh x 1)
  std::vector<int> level_;            // levels of each node                     (nbvh x 1)
  std::vector<SIM_CBoundingVolume> bvleaf_;
  std::string name_;
  double ipos_[3] = {0, 0, 0};
  double iquat_[4] = {1, 0, 0, 0};
};

class SIM_CBoundingVolumeHierarchy : public SIM_CBoundingVolumeHierarchy_ {
 public:
  // make bounding volume hierarchy
  void CreateBVH();
  void Set(double ipos_element[3], double iquat_element[4]);
  void AllocateBoundingVolumes(int nleaf);
  void RemoveInactiveVolumes(int nmax);
  const SIM_CBoundingVolume*
      AddBoundingVolume(int id, int contype, int conaffinity, const double pos[3],
                                             const double quat[4], const double aabb[6]);
  const SIM_CBoundingVolume*
      AddBoundingVolume(const int* id, int contype, int conaffinity, const double pos[3],
                                             const double quat[4], const double aabb[6]);

  // public accessors
  int Nbvh() const { return nbvh_; }
  const std::vector<sim_scalar_t>& Bvh() const { return bvh_; }
  const std::vector<int>& Child() const { return child_; }
  const std::vector<int>& Nodeid() const { return nodeid_; }
  int Nodeid(int id) const { return nodeid_[id]; }
  const int* Nodeidptr(int id) const { return nodeidptr_[id]; }
  const std::vector<int>& Level() const { return level_; }
  int Size() const {
    return sizeof(SIM_CBoundingVolume) * bvleaf_.size()
        + sizeof(sim_scalar_t) * bvh_.size() + sizeof(int) * child_.size()
        + sizeof(int) * nodeid_.size() + sizeof(int) * level_.size();
  }

 private:
  // internal class used during BVH construction, for partial sorting of bounding volumes
  struct BVElement {
    const SIM_CBoundingVolume* e;
    // position of the element in the BVH axes
    double lpos[3];
  };
  void Make(std::vector<BVElement>& elements);
  int MakeBVH(std::vector<BVElement>::iterator elements_begin,
              std::vector<BVElement>::iterator elements_end, int lev = 0);
};



//------------------------- class SIM_COctree --------------------------------------------------------

struct Point {
  std::array<double, 3> p;

  double& operator[](size_t i) { return p[i]; }
  const double& operator[](size_t i) const { return p[i]; }

  bool operator==(const Point& other) const {
    constexpr double kEpsilon = 1e-9;
    return std::abs(this->p[0] - other.p[0]) < kEpsilon &&
           std::abs(this->p[1] - other.p[1]) < kEpsilon &&
           std::abs(this->p[2] - other.p[2]) < kEpsilon;
  }
};

namespace std {
template <>
struct hash<Point> {
  size_t operator()(const Point& pt) const {
    size_t h1 = hash<double>()(pt.p[0]);
    size_t h2 = hash<double>()(pt.p[1]);
    size_t h3 = hash<double>()(pt.p[2]);
    // combine hashes
    size_t seed = h1;
    seed ^= h2 + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= h3 + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    return seed;
  }
};
}  // namespace std

typedef std::array<std::array<double, 3>, 3> Triangle;

struct OctNode {
  int level = 0;                       // level of the node
  int parent_index = -1;               // index of the parent node
  int child_slot = -1;                 // slot of the child node in the parent node
  std::array<int, 8> child = {-1};     // children nodes
  std::array<int, 8> vertid = {-1};    // vertex id's
  std::array<double, 6> aamm = {0};    // bounding box
  std::array<double, 8> coeff = {0};   // interpolation coefficients
};

struct OctreeTask {
  std::vector<Triangle*> elements;
  int lev;
  int parent_index;
  int child_slot;
  int node_index;
};

struct SIM_COctree_ {
  int nnode_ = 0;
  int nvert_ = 0;
  std::vector<OctNode> node_;
  std::vector<Triangle> face_;          // mesh faces                (nmeshface x 3)
  std::vector<Point> vert_;             // octree vertices           (nvert x 3)
  std::vector<std::vector<int>> hang_;  // hanging nodes status      (nvert x 1)
  double ipos_[3] = {0, 0, 0};
  double iquat_[4] = {1, 0, 0, 0};
};

class SIM_COctree : public SIM_COctree_ {
 public:
  void CreateOctree(const double aamm[6]);

  int NumNodes() const { return nnode_; }
  int NumVerts() const { return nvert_; }
  void CopyLevel(int* level) const;
  void CopyChild(int* child) const;
  void CopyAabb(sim_scalar_t* aabb) const;
  void CopyCoeff(sim_scalar_t* coeff) const;
  const double* Vert(int i) const { return vert_[i].p.data(); }
  const std::vector<int>& Hang(int i) const { return hang_[i]; }
  int VertId(int n, int v) const { return node_[n].vertid[v]; }
  const std::array<int, 8>& Children(int i) const { return node_[i].child; }
  void SetFace(const std::vector<double>& vert, const std::vector<int>& face);
  int Size() const {
    return sizeof(OctNode) * node_.size() + sizeof(Triangle) * face_.size() +
           sizeof(Point) * vert_.size();
  }
  void Clear() {
    node_.clear();
    face_.clear();
  }
  void AddCoeff(int n, int v, double coeff) { node_[n].coeff[v] = coeff; }

 private:
  void Make(std::vector<Triangle>& elements);
  void MakeOctree(const std::vector<Triangle*>& elements, const double aamm[6],
                  std::unordered_map<Point, int>& vert_map);
  void TaskToNode(const OctreeTask& task, OctNode& node, std::unordered_map<Point, int>& vert_map);
  void Subdivide(const OctreeTask& task, std::unordered_map<Point, int>& vert_map,
                 std::deque<OctreeTask>* queue = nullptr,
                 const std::vector<Triangle*>& colliding = {});
  int FindNeighbor(int node_idx, int dir);
  int FindCoarseNeighbor(int node_idx, int dir);
  void BalanceOctree(std::unordered_map<Point, int>& vert_map);
  void MarkHangingNodes();
};



//------------------------- class sim_builder_base_t ----------------------------------------------------------
// Generic functionality for all derived classes

class SIM_CBase_ : public sim_spec_element_t {
 public:
  int id;                 // object id
  std::string name;       // object name
  std::string classname;  // defaults class name
  std::string info;       // error message info set by the user
  std::string prefix;     // prefix for model operations
  std::string suffix;     // suffix for model operations
};

class sim_builder_base_t : public SIM_CBase_ {
  friend class sim_builder_default_t;

 public:
  // load resource if found (fallback to OS filesystem)
  static SIM_Resource* LoadResource(const std::string& modelfiledir,
                                  const std::string& filename, const SIM_VFS* vfs);

  // Get and sanitize content type from raw_text if not empty, otherwise parse
  // content type from resource_name; throw on failure
  static std::string GetAssetContentType(std::string_view resource_name, std::string_view raw_text);

  // Add frame transformation
  void SetFrame(sim_builder_frame_t* _frame);

  // Copy spec into private attributes
  virtual void CopyFromSpec() {}

  // Throws an error if any of the references is missing
  virtual void ResolveReferences(const sim_builder_model_t* m) {}

  // Appends prefix and suffix to reference
  virtual void NameSpace(const sim_builder_model_t* m);

  // Copy plugins instantiated in this object
  virtual void CopyPlugin() {}

  // Returns parent of this object
  virtual sim_builder_base_t* GetParent() const { return nullptr; }

  // Returns the model of this object
  SIM_sCompiler* FindCompiler(const SIM_sCompiler* compiler) const;

  // Copy assignment
  sim_builder_base_t& operator=(const sim_builder_base_t& other);

  sim_builder_frame_t* frame;                // pointer to frame transformation
  sim_builder_model_t* model;                // pointer to model that owns object
  SIM_sCompiler* compiler;          // pointer to the compiler options

  virtual ~sim_builder_base_t() = default;   // destructor

  // reset keyframe references for allowing self-attach
  virtual void ForgetKeyframes() {}
  virtual void ForgetKeyframes() const {}

  // increment and decrement reference count
  // release uses the argument to delete the plugin
  // which may be still owned by the source spec during shallow attach
  virtual void AddRef() { ++refcount; }
  virtual int GetRef() { return refcount; }
  virtual void Release() {
    if (--refcount == 0) {
      delete this;
    }
  }

  // Set and get user payload
  void SetUserValue(std::string_view key, const void* data,
                    void (*cleanup)(const void*));
  const void* GetUserValue(std::string_view key);
  void DeleteUserValue(std::string_view key);

 protected:
  sim_builder_base_t();                                 // constructor
  sim_builder_base_t(const sim_builder_base_t& other);             // copy constructor

  // reference count for allowing deleting an attached object
  int refcount = 1;

  // Arbitrary user value that cleans up the data when destroyed.
  struct UserValue {
    const void* value = nullptr;
    void (*cleanup)(const void*) = nullptr;

    UserValue() {}
    UserValue(const void* value, void (*cleanup)(const void*))
        : value(value), cleanup(cleanup) {}
    UserValue(const UserValue& other) = delete;
    UserValue& operator=(const UserValue& other) = delete;

    UserValue(UserValue&& other) : value(other.value), cleanup(other.cleanup) {
      other.value = nullptr;
      other.cleanup = nullptr;
    }

    UserValue& operator=(UserValue&& other) {
      if (this != &other) {
        if (cleanup && value) {
          cleanup(value);
        }
        value = other.value;
        cleanup = other.cleanup;
        other.value = nullptr;
        other.cleanup = nullptr;
      }
      return *this;
    }

    ~UserValue() {
      if (cleanup && value) {
        cleanup(value);
      }
    }
  };

  // user payload
  std::unordered_map<std::string, UserValue> user_payload_;
};



//------------------------- class sim_builder_body_t -----------------------------------------------
// Describes a rigid body

class SIM_CBody_ : public sim_builder_base_t {
 protected:
  sim_builder_body_t* parent;

  // variables computed by 'Compile' and 'AddXXX'
  int weldid;                     // top index of body we are welded to
  int dofnum;                     // number of motion dofs for body
  int mocapid;                    // mocap id, -1: not mocap

  int contype;                    // OR over geom contypes
  int conaffinity;                // OR over geom conaffinities
  double margin;                  // MAX over geom margins
  double xpos0[3];                // global position in qpos0
  double xquat0[4];               // global orientation in qpos0

  // used internally by compiler
  int lastdof;                    // id of last dof
  int subtreedofs;                // number of dofs in subtree, including self

  SIM_CBoundingVolumeHierarchy tree;  // bounding volume hierarchy

  // variable-size data
  std::string plugin_name;
  std::string plugin_instance_name;
  std::vector<double> userdata_;
  std::vector<double> spec_userdata_;

  // variables used for temporarily storing the state of the mocap bodies
  std::map<std::string, std::array<sim_scalar_t, 3>> mpos_;   // saved mocap_pos
  std::map<std::string, std::array<sim_scalar_t, 4>> mquat_;  // saved mocap_quat
};

class sim_builder_body_t : public SIM_CBody_, private sim_spec_body_t {
  friend class sim_builder_joint_t;
  friend class sim_builder_geom_t;
  friend class sim_builder_site_t;
  friend class SIM_CCamera;
  friend class SIM_CComposite;
  friend class sim_builder_frame_t;
  friend class SIM_CLight;
  friend class SIM_CFlex;
  friend class SIM_CFlexcomp;
  friend class SIM_CEquality;
  friend class SIM_CPair;
  friend class sim_builder_model_t;
  friend class sim_xml_reader_t;
  friend class sim_xml_writer_t;
  friend class SIM_XURDF;

 public:
  explicit sim_builder_body_t(sim_builder_model_t*);
  ~sim_builder_body_t();

  // API for adding objects to body
  sim_builder_body_t*    AddBody(sim_builder_default_t* = 0);
  sim_builder_frame_t*   AddFrame(sim_builder_frame_t* = 0);
  sim_builder_joint_t*   AddJoint(sim_builder_default_t* = 0);
  sim_builder_joint_t*   AddFreeJoint();
  sim_builder_geom_t*    AddGeom(sim_builder_default_t* = 0);
  sim_builder_site_t*    AddSite(sim_builder_default_t* = 0);
  SIM_CCamera*  AddCamera(sim_builder_default_t* = 0);
  SIM_CLight*   AddLight(sim_builder_default_t* = 0);

  // API for adding/removing objects to body
  sim_builder_body_t& operator+=(const sim_builder_body_t& other);
  sim_builder_body_t& operator+=(const sim_builder_frame_t& other);
  sim_builder_body_t& operator-=(const sim_builder_body_t& subtree);

  // API for accessing objects
  int NumObjects(sim_obj_t type);
  sim_builder_base_t* GetObject(sim_obj_t type, int id);
  sim_builder_base_t* FindObject(sim_obj_t type, std::string name, bool recursive = true);

  // Propagate suffix and prefix to the whole tree
  void NameSpace(const sim_builder_model_t* m);

  // set explicitinertial to true
  void MakeInertialExplicit();

  // compute the bounding volume hierarchy of the body.
  void ComputeBVH();

  // variables set by user
  sim_spec_body_t spec;

  // inherited
  using sim_builder_base_t::info;

  // used by sim_xml_writer_t and sim_builder_model_t
  const std::vector<double>& get_userdata() { return userdata_; }

  // get next child of given type recursively; if `child` is found while traversing the tree,
  // then `found` is set to true and the next element encountered is returned;
  // returns nullptr if the next child is not found or if `child` is the last element, returns
  // the next child after the input `child` otherwise
  sim_spec_element_t* NextChild(const sim_spec_element_t* child, sim_obj_t type = SIM_OBJ_UNKNOWN,
                        bool recursive = false);

  // reset keyframe references for allowing self-attach
  void ForgetKeyframes() const;

  // create a frame and move all contents of this body into it
  sim_builder_frame_t* ToFrame();

  // get mocap position and quaternion
  sim_scalar_t* mpos(const std::string& state_name);
  sim_scalar_t* mquat(const std::string& state_name);

  SIM_sFrame* last_attached;  // last attached frame to this body

  // set parent of this body
  void SetParent(sim_builder_body_t* _body) { parent = _body; }
  sim_builder_body_t* GetParent() const { return parent; }

  // set model of this body
  void SetModel(sim_builder_model_t* _model);

  // reset ids of all objects in this body
  void ResetId();

  // getters
  std::vector<sim_builder_body_t*> Bodies() const { return bodies; }

  // get list of a given type
  template <class T>
  const std::vector<T*>& GetList() const;

  // accumulate inertia of another body into this body, if `result` is not nullptr, the accumulated
  // inertia will be stored in `result`, otherwise the body's private spec will be used.
  void AccumulateInertia(const sim_spec_body_t* other, sim_spec_body_t* result = nullptr);

 private:
  sim_builder_body_t(const sim_builder_body_t& other, sim_builder_model_t* _model);  // copy constructor
  sim_builder_body_t& operator=(const sim_builder_body_t& other);         // copy assignment

  void Compile(void);             // compiler
  void InertiaFromGeom(void);     // get inertial info from geoms

  // objects allocated by Add functions
  std::vector<sim_builder_body_t*>    bodies;     // child bodies
  std::vector<sim_builder_geom_t*>    geoms;      // geoms attached to this body
  std::vector<sim_builder_frame_t*>   frames;     // frames attached to this body
  std::vector<sim_builder_joint_t*>   joints;     // joints allowing motion relative to parent
  std::vector<sim_builder_site_t*>    sites;      // sites attached to this body
  std::vector<SIM_CCamera*>  cameras;    // cameras attached to this body
  std::vector<SIM_CLight*>   lights;     // lights attached to this body

  void CopyFromSpec();                 // copy spec into attributes
  void PointToLocal(void);
  void NameSpace_(const sim_builder_model_t* m, bool propagate = true);
  void CopyPlugin();

  // copy src list of elements into dst; set body, model and frame
  template <typename T>
  void CopyList(std::vector<T*>& dst, const std::vector<T*>& src,
                std::map<sim_builder_frame_t*, int>& fmap, const sim_builder_frame_t* pframe = nullptr);
};



//------------------------- class sim_builder_frame_t ---------------------------------------------------------
// Describes a coordinate transformation relative to its parent

class SIM_CFrame_ : public sim_builder_base_t {
 protected:
  bool compiled;                           // frame already compiled
};

class sim_builder_frame_t : public SIM_CFrame_, private SIM_sFrame {
  friend class sim_builder_base_t;
  friend class sim_builder_body_t;
  friend class sim_builder_geom_t;
  friend class sim_builder_joint_t;
  friend class sim_builder_site_t;
  friend class SIM_CCamera;
  friend class SIM_CLight;
  friend class sim_builder_model_t;

 public:
  sim_builder_frame_t(sim_builder_model_t* = 0, sim_builder_frame_t* = 0);
  sim_builder_frame_t(const sim_builder_frame_t& other);
  sim_builder_frame_t& operator=(const sim_builder_frame_t& other);

  SIM_sFrame spec;
  using sim_builder_base_t::info;

  void CopyFromSpec(void);
  void PointToLocal(void);
  void SetParent(sim_builder_body_t* _body) { body = _body; }
  sim_builder_body_t* GetParent() const { return body; }

  sim_builder_frame_t& operator+=(const sim_builder_body_t& other);

  bool IsAncestor(const sim_builder_frame_t* child) const;  // true if child is contained in this frame

  sim_spec_body_t* last_attached;  // last attached body to this frame

 private:
  void Compile(void);                          // compiler

  sim_builder_body_t* body;  // body that owns the frame
};



//------------------------- class sim_builder_joint_t ---------------------------------------------------------
// Describes a motion degree of freedom of a body relative to its parent

class SIM_CJoint_ : public sim_builder_base_t {
 protected:
  sim_builder_body_t* body;                   // joint's body

  // variable used for temporarily storing the state of the joint
  std::map<std::string, std::array<sim_scalar_t, 7>> qpos_;  // qpos at the previous step
  std::map<std::string, std::array<sim_scalar_t, 6>> qvel_;  // qvel at the previous step

  // variable-size data
  std::vector<double> userdata_;
  std::vector<double> spec_userdata_;
};

class sim_builder_joint_t : public SIM_CJoint_, private SIM_sJoint {
  friend class sim_builder_default_t;
  friend class SIM_CEquality;
  friend class sim_builder_body_t;
  friend class sim_builder_model_t;
  friend class SIM_CSensor;
  friend class sim_xml_writer_t;
  friend class SIM_XURDF;

 public:
  explicit sim_builder_joint_t(sim_builder_model_t* = nullptr, sim_builder_default_t* = nullptr);
  sim_builder_joint_t(const sim_builder_joint_t& other);
  sim_builder_joint_t& operator=(const sim_builder_joint_t& other);

  SIM_sJoint spec;
  using sim_builder_base_t::info;

  void CopyFromSpec(void);
  void SetParent(sim_builder_body_t* _body) { body = _body; }
  sim_builder_body_t* GetParent() const { return body; }

  // used by sim_xml_writer_t and sim_builder_model_t
  const std::vector<double>& get_userdata() const { return userdata_; }
  const double* get_range() const { return range; }

  bool is_limited() const;
  bool is_actfrclimited() const;

  static int nq(SIM_tJoint joint_type);
  static int nv(SIM_tJoint joint_type);
  int nq() const { return nq(spec.type); }
  int nv() const { return nv(spec.type); }

  sim_scalar_t* qpos(const std::string& state_name);
  sim_scalar_t* qvel(const std::string& state_name);

 private:
  int Compile(void);               // compiler; return dofnum
  void PointToLocal(void);

  // variables that should not be copied during copy assignment
  int qposadr_;                                        // address of dof in data->qpos
  int dofadr_;                                         // address of dof in data->qvel
};



//------------------------- class sim_builder_geom_t ----------------------------------------------------------
// Describes a geometric shape belonging to a body

class SIM_CGeom_ : public sim_builder_base_t {
 public:
  bool inferinertia;           // true if inertia should be computed from geom

 protected:
  bool visual_;                       // true: geom does not collide and is unreferenced
  int matid;                          // id of geom's material
  sim_builder_mesh_t* mesh;                      // geom's mesh
  SIM_CHField* hfield;                  // geom's hfield
  double mass_;                       // mass
  double inertia[3];                  // local diagonal inertia
  double aabb[6];                     // axis-aligned bounding box (center, size)
  sim_builder_body_t* body;                      // geom's body
  sim_scalar_t fluid[SIM_NFLUID];             // compile-time fluid-interaction parameters

  // variable-size data
  std::string plugin_name;
  std::string plugin_instance_name;
  std::string hfieldname_;
  std::string meshname_;
  std::string material_;
  std::vector<double> userdata_;
  std::string spec_hfieldname_;
  std::string spec_meshname_;
  std::string spec_material_;
  std::vector<double> spec_userdata_;
};

class sim_builder_geom_t : public SIM_CGeom_, private sim_spec_geom_t {
  friend class sim_builder_default_t;
  friend class sim_builder_mesh_t;
  friend class SIM_CPair;
  friend class sim_builder_body_t;
  friend class sim_builder_model_t;
  friend class SIM_CWrap;
  friend class sim_xml_writer_t;
  friend class SIM_XURDF;

 public:
  explicit sim_builder_geom_t(sim_builder_model_t* = nullptr, sim_builder_default_t* = nullptr);
  sim_builder_geom_t(const sim_builder_geom_t& other);
  sim_builder_geom_t& operator=(const sim_builder_geom_t& other);

  sim_spec_geom_t spec;                       // variables set by user
  double GetVolume() const;           // compute geom volume
  void SetInertia(void);              // compute and set geom inertia
  bool IsVisual(void) const { return visual_; }
  void SetNotVisual(void) { visual_ = false; }
  void SetParent(sim_builder_body_t* _body) { body = _body; }
  sim_builder_body_t* GetParent() const { return body; }
  SIM_tGeom Type() const { return type; }

  // Compute all coefs modeling the interaction with the surrounding fluid.
  void SetFluidCoefs(void);
  // Compute the kappa coefs of the added inertia due to the surrounding fluid.
  double GetAddedMassKappa(double dx, double dy, double dz);

  // used by sim_xml_writer_t and sim_builder_model_t
  const std::vector<double>& get_userdata() const { return userdata_; }
  const std::string& get_hfieldname() const { return spec_hfieldname_; }
  const std::string& get_meshname() const { return spec_meshname_; }
  const std::string& get_material() const;
  void del_material() { spec_material_.clear(); }

 private:
  void Compile(void);                 // compiler
  double GetRBound(void);             // compute bounding sphere radius
  void ComputeAABB(void);             // compute axis-aligned bounding box
  void CopyFromSpec(void);
  void PointToLocal(void);
  void NameSpace(const sim_builder_model_t* m);
  void CopyPlugin();

  // inherited
  using sim_builder_base_t::info;
};



//------------------------- class sim_builder_site_t ----------------------------------------------------------
// Describes a site on a body

class SIM_CSite_ : public sim_builder_base_t {
 protected:
  // variable-size data
  std::string material_;
  std::vector<double> userdata_;
  std::string spec_material_;
  std::vector<double> spec_userdata_;

  // variables computed by 'compile' and 'sim_builder_body_t::addSite'
  sim_builder_body_t* body;                  // site's body
  int matid;                      // material id for rendering
};

class sim_builder_site_t : public SIM_CSite_, private SIM_sSite {
  friend class sim_builder_default_t;
  friend class sim_builder_body_t;
  friend class sim_builder_model_t;
  friend class sim_xml_writer_t;
  friend class SIM_XURDF;

 public:
  explicit sim_builder_site_t(sim_builder_model_t* = nullptr, sim_builder_default_t* = nullptr);
  sim_builder_site_t(const sim_builder_site_t& other);
  sim_builder_site_t& operator=(const sim_builder_site_t& other);

  SIM_sSite spec;                   // variables set by user

  // site's body
  sim_builder_body_t* Body() const { return body; }
  void SetParent(sim_builder_body_t* _body) { body = _body; }
  sim_builder_body_t* GetParent() const { return body; }

  // use strings from sim_builder_base_t rather than SIM_Strings from SIM_sSite
  using sim_builder_base_t::info;

  // used by sim_xml_writer_t and sim_builder_model_t
  const std::vector<double>& get_userdata() const { return userdata_; }
  const std::string& get_material() const { return material_; }
  void del_material() { material_.clear(); }

 private:
  void Compile(void);                     // compiler
  void CopyFromSpec();                    // copy spec into attributes
  void PointToLocal(void);
  void NameSpace(const sim_builder_model_t* m);
};



//------------------------- class SIM_CCamera --------------------------------------------------------
// Describes a camera, attached to a body

class SIM_CCamera_ : public sim_builder_base_t {
 protected:
  sim_builder_body_t* body;                  // camera's body
  int targetbodyid;               // id of target body; -1: none
  std::string targetbody_;
  std::string spec_targetbody_;
  std::vector<double> userdata_;
  std::vector<double> spec_userdata_;
};

class SIM_CCamera : public SIM_CCamera_, private SIM_sCamera {
  friend class sim_builder_default_t;
  friend class sim_builder_body_t;
  friend class sim_builder_model_t;
  friend class SIM_CSensor;
  friend class sim_xml_writer_t;

 public:
  explicit SIM_CCamera(sim_builder_model_t* = nullptr, sim_builder_default_t* = nullptr);
  SIM_CCamera(const SIM_CCamera& other);
  SIM_CCamera& operator=(const SIM_CCamera& other);

  SIM_sCamera spec;
  using sim_builder_base_t::info;

  // used by sim_xml_writer_t and sim_builder_model_t
  const std::string& get_targetbody() const { return targetbody_; }
  const std::vector<double>& get_userdata() const { return userdata_; }

  void SetParent(sim_builder_body_t* _body) { body = _body; }
  sim_builder_body_t* GetParent() const { return body; }

 private:
  void Compile(void);                     // compiler
  void CopyFromSpec(void);
  void PointToLocal(void);
  void NameSpace(const sim_builder_model_t* m);
  void ResolveReferences(const sim_builder_model_t* m);
};



//------------------------- class SIM_CLight ---------------------------------------------------------
// Describes a light, attached to a body

class SIM_CLight_ : public sim_builder_base_t {
 protected:
  sim_builder_body_t* body;                  // light's body
  int targetbodyid;               // id of target body; -1: none
  int texid;                      // id of texture; -1: none
  std::string texture_;
  std::string spec_texture_;
  std::string targetbody_;
  std::string spec_targetbody_;
};

class SIM_CLight : public SIM_CLight_, private SIM_sLight {
  friend class sim_builder_default_t;
  friend class sim_builder_body_t;
  friend class sim_builder_model_t;
  friend class sim_xml_writer_t;

 public:
  explicit SIM_CLight(sim_builder_model_t* = nullptr, sim_builder_default_t* = nullptr);
  SIM_CLight(const SIM_CLight& other);
  SIM_CLight& operator=(const SIM_CLight& other);

  SIM_sLight spec;
  using sim_builder_base_t::info;

  // used by sim_xml_writer_t and sim_builder_model_t
  const std::string& get_targetbody() const { return targetbody_; }
  const std::string& get_texture() const { return texture_; }

  void SetParent(sim_builder_body_t* _body) { body = _body; }
  sim_builder_body_t* GetParent() const { return body; }

 private:
  void Compile(void);                     // compiler
  void CopyFromSpec(void);
  void PointToLocal(void);
  void NameSpace(const sim_builder_model_t* m);
  void ResolveReferences(const sim_builder_model_t* m);
};



//------------------------- class SIM_CFlex ----------------------------------------------------------
// Describes a flex

struct StencilFlap {
  static constexpr int kNumVerts = 4;
  int vertices[kNumVerts];
};

class SIM_CFlex_ : public sim_builder_base_t {
 protected:
  int nvert;                              // number of vertices
  int nnode;                              // number of nodes
  int nedge;                              // number of edges
  int nelem;                              // number of elements
  int matid;                              // material id
  bool rigid;                             // all vertices attached to the same body
  bool centered;                          // all vertices coordinates (0,0,0)
  bool interpolated;                      // vertices are interpolated from nodes
  std::vector<int> vertbodyid;            // vertex body ids
  std::vector<int> nodebodyid;            // node body ids
  std::vector<std::pair<int, int>> edge;  // edge vertex ids
  std::vector<int> shell;                 // shell fragment vertex ids (dim per fragment)
  std::vector<int> elemlayer;             // element layer (distance from border)
  std::vector<int> evpair;                // element-vertex pairs
  std::vector<StencilFlap> flaps;         // adjacent triangles
  std::vector<double> vertxpos;           // global vertex positions
  SIM_CBoundingVolumeHierarchy tree;        // bounding volume hierarchy
  std::vector<double> elemaabb_;          // element bounding volume
  std::vector<int> edgeidx_;              // element edge ids
  std::vector<double> stiffness;          // elasticity stiffness matrix
  std::vector<double> bending;            // bending stiffness matrix

  // variable-size data
  std::vector<std::string> vertbody_;     // vertex body names
  std::vector<std::string> nodebody_;     // node body names
  std::vector<double> vert_;              // vertex positions
  std::vector<double> node_;              // node positions
  std::vector<int> elem_;                 // element vertex ids
  std::vector<float> texcoord_;           // vertex texture coordinates
  std::vector<int> elemtexcoord_;         // face texture coordinates (OBJ only)
  std::string material_;                  // name of material used for rendering

  std::string spec_material_;
  std::vector<std::string> spec_vertbody_;
  std::vector<std::string> spec_nodebody_;
  std::vector<double> spec_vert_;
  std::vector<double> spec_node_;
  std::vector<int> spec_elem_;
  std::vector<float> spec_texcoord_;
  std::vector<int> spec_elemtexcoord_;
};

class SIM_CFlex: public SIM_CFlex_, private SIM_sFlex {
  friend class sim_builder_default_t;
  friend class sim_builder_model_t;
  friend class SIM_CFlexcomp;
  friend class SIM_CEquality;
  friend class sim_xml_writer_t;

 public:
  explicit SIM_CFlex(sim_builder_model_t* = nullptr);
  SIM_CFlex(const SIM_CFlex& other);
  SIM_CFlex& operator=(const SIM_CFlex& other);

  SIM_sFlex spec;
  using sim_builder_base_t::info;

  void CopyFromSpec(void);
  void PointToLocal(void);
  void ResolveReferences(const sim_builder_model_t* m);
  void NameSpace(const sim_builder_model_t* m);

  // used by sim_xml_writer_t and sim_builder_model_t
  const std::string& get_material() const { return material_; }
  const std::vector<std::string>& get_vertbody() const { return vertbody_; }
  const std::vector<double>& get_vert() const { return vert_; }
  const std::vector<double>& get_elemaabb() const { return elemaabb_; }
  const std::vector<int>& get_elem() const { return elem_; }
  const std::vector<float>& get_texcoord() const { return texcoord_; }
  const std::vector<int>& get_elemtexcoord() const { return elemtexcoord_; }
  const std::vector<std::string>& get_nodebody() const { return nodebody_; }

  bool HasTexcoord() const;               // texcoord not null
  void DelTexcoord();                     // delete texcoord

  static constexpr int kNumEdges[3] = {1, 3, 6};  // number of edges per element indexed by dim

  void SetOrder(int order) { order_ = order; }  // set interpolation order

 private:
  void Compile(const SIM_VFS* vfs);         // compiler
  void CreateBVH(void);                   // create flex BVH
  void CreateShellPair(void);             // create shells and evpairs

  std::vector<double> vert0_;             // vertex positions in [0, 1]^d in the bounding box
  std::vector<double> node0_;             // node Cartesian positions

  int order_ = 0;                         // interpolation order
};



//------------------------- class sim_builder_mesh_t ----------------------------------------------------------
// Describes a mesh

class SIM_CMesh_ : public sim_builder_base_t {
 protected:
  // variable size attributes
  std::string plugin_name;
  std::string plugin_instance_name;

  std::string content_type_ = "";                // content type of file
  std::string file_;                             // mesh file
  SIM_Resource* resource_ = nullptr;               // resource for mesh file
  std::vector<double> vert_;                      // vertex data
  std::vector<float> normal_;                    // normal data
  std::vector<float> texcoord_;                  // texcoord data
  std::vector<int> face_;                        // vertex indices
  std::vector<int> facenormal_;                  // normal indices
  std::vector<int> facetexcoord_;                // texcoord indices
  std::string material_;                         // mesh fallback material

  std::string spec_content_type_;
  std::string spec_file_;
  std::vector<float> spec_vert_;
  std::vector<float> spec_normal_;
  std::vector<float> spec_texcoord_;
  std::vector<int> spec_face_;
  std::vector<int> spec_facenormal_;
  std::vector<int> spec_facetexcoord_;
  std::string spec_material_;

  // used by the compiler
  bool needreorient_;                            // needs reorientation
  bool visual_;                                  // true: the mesh is only visual
  std::vector< std::pair<int, int> > halfedge_;  // half-edge data

  // mesh processed flags
  bool processed_;                  // has the mesh been processed yet
  bool transformed_;                // has the mesh been transformed to CoM and inertial frame

  // mesh properties computed by Compile
  double pos_[3];                     // CoM position
  double quat_[4];                    // inertia orientation
  double boxsz_[3];                   // half-sizes of equivalent inertia box
  double aamm_[6];                    // axis-aligned bounding box in (min, max) format
  double volume_;                     // volume of the mesh
  double surface_;                    // surface of the mesh

  // size of mesh data to be copied into sim_model_t
  int szgraph_ = 0;                   // size of graph data in ints
  bool needhull_;                     // needs convex hull for collisions
  int maxhullvert_;                   // max vertex count of convex hull

  // bounding volume hierarchy tree
  SIM_CBoundingVolumeHierarchy tree_;   // bounding volume hierarchy
  std::vector<double> face_aabb_;     // bounding boxes of all faces

  // octree
  SIM_COctree octree_;                  // octree of the mesh

  // paths stored during model attachment
  simcore::user::FilePath modelfiledir_;
};

class sim_builder_mesh_t: public SIM_CMesh_, private SIM_sMesh {
  friend class sim_builder_model_t;

 public:
  explicit sim_builder_mesh_t(sim_builder_model_t* = nullptr, sim_builder_default_t* = nullptr);
  sim_builder_mesh_t(const sim_builder_mesh_t& other);
  sim_builder_mesh_t& operator=(const sim_builder_mesh_t& other);
  ~sim_builder_mesh_t();

  SIM_sMesh spec;
  using sim_builder_base_t::info;

  void CopyFromSpec(void);
  void PointToLocal(void);
  void NameSpace(const sim_builder_model_t* m);

  // make a mesh of a predefined shape
  void MakeHemisphere(int res, bool make_faces, bool make_cap);
  void MakeSphere(int subdiv, bool make_faces);
  void MakeTorus(int res, double radius);
  void MakeSupertorus(int res, double radius, double s, double t);
  void MakeSupersphere(int res, double e, double n);
  void MakeWedge(int resolution[2], double fov[2], double gamma);
  void MakeRect(int resolution[2]);
  void MakeCone(int nedge, double radius);

  // accessors
  const SIM_sPlugin& Plugin() const { return plugin; }
  const std::string& ContentType() const { return content_type_; }
  const std::string& File() const { return file_; }
  const double* Refpos() const { return refpos; }
  const double* Refquat() const { return refquat; }
  const double* Scale() const { return scale; }
  bool SmoothNormal() const { return smoothnormal; }
  const std::vector<double>& Vert() const { return vert_; }
  double Vert(int i) const { return vert_[i]; }
  const std::vector<float>& UserVert() const { return spec_vert_; }
  const std::vector<float>& UserNormal() const { return spec_normal_; }
  const std::vector<float>& Texcoord() const { return texcoord_; }
  const std::vector<int>& FaceTexcoord() const { return facetexcoord_; }
  const std::vector<float>& UserTexcoord() const { return spec_texcoord_; }
  const std::vector<int>& Face() const { return face_; }
  const std::vector<int>& UserFace() const { return spec_face_; }
  SIM_tMeshInertia Inertia() const { return spec.inertia; }
  const std::string& Material() const { return material_; }

  // setters
  void SetNeedHull(bool needhull) { needhull_ = needhull; }

  // mesh properties computed by Compile
  const double* aamm() const { return aamm_; }

  // number of vertices, normals, texture coordinates, and faces
  int nvert() const { return vert_.size()/3; }
  int nnormal() const { return normal_.size()/3; }
  int ntexcoord() const { return texcoord_.size()/2; }
  int nface() const { return face_.size()/3; }
  int npolygon() const { return polygons_.size(); }
  int npolygonvert() const {
    int acc = 0;
    for (const auto& polygon : polygons_) {
      acc += polygon.size();
    }
    return acc;
  }
  int npolygonmap() const {
    int acc = 0;
    for (const auto& polygon : polygon_map_) {
      acc += polygon.size();
    }
    return acc;
  }

  // return size of graph data in ints
  int szgraph() const { return szgraph_; }

  // bounding volume hierarchy tree
  const SIM_CBoundingVolumeHierarchy& tree() { return tree_; }

  // octree
  const SIM_COctree& octree() { return octree_; }

  void Compile(const SIM_VFS* vfs);                   // compiler
  double* GetPosPtr();                              // get position
  double* GetQuatPtr();                             // get orientation
  double* GetInertiaBoxPtr();                       // get inertia box
  double GetVolumeRef() const;                      // get volume
  void FitGeom(sim_builder_geom_t* geom, double center[3]);    // approximate mesh with simple geom
  bool HasTexcoord() const;                         // texcoord not null
  void DelTexcoord();                               // delete texcoord
  bool IsVisual(void) const { return visual_; }     // is geom visual
  void SetNotVisual(void) { visual_ = false; }      // mark mesh as not visual

  void CopyVert(float* arr) const;                  // copy vert data into array
  void CopyNormal(float* arr) const;                // copy normal data into array
  void CopyFace(int* arr) const;                    // copy face data into array
  void CopyFaceNormal(int* arr) const;              // copy face normal data into array
  void CopyFaceTexcoord(int* arr) const;            // copy face texcoord data into array
  void CopyTexcoord(float* arr) const;              // copy texcoord data into array
  void CopyGraph(int* arr) const;                   // copy graph data into array

  // copy polygon data into array
  void CopyPolygons(int* verts, int* adr, int* num, int poly_adr) const;

  // copy polygon map data into array
  void CopyPolygonMap(int *faces, int* adr, int* num, int poly_adr) const;

  // copy polygon normal data into array
  void CopyPolygonNormals(sim_scalar_t* arr);

  // sets properties of a bounding volume given a face id
  void SetBoundingVolume(int faceid);

  // load from OBJ, STL, or MSH file; throws sim_builder_error_t on failure
  void LoadFromResource(SIM_Resource* resource, bool remove_repeated = false);

  static bool IsObj(std::string_view filename, std::string_view ct = "");
  static bool IsSTL(std::string_view filename, std::string_view ct = "");
  static bool IsMSH(std::string_view filename, std::string_view ct = "");

  bool IsObj() const;
  bool IsSTL() const;
  bool IsMSH() const;

 private:
  void TryCompile(const SIM_VFS* vfs);

  // load mesh from cache asset, return true on success (OBJ files are only supported)
  bool LoadCachedMesh(SIM_CCache *cache, const SIM_Resource* resource);

  // store mesh into asset cache (OBJ files are only supported)
  void CacheMesh(SIM_CCache *cache, const SIM_Resource* resource);

  // convert vertices to double precision and remove repeated vertices if requested
  void ProcessVertices(const std::vector<float>& vert, bool remove_repeated = false);


  void LoadOBJ(SIM_Resource* resource, bool remove_repeated);  // load mesh in wavefront OBJ format
  void LoadSTL(SIM_Resource* resource);                        // load mesh in STL BIN format
  void LoadMSH(SIM_Resource* resource, bool remove_repeated);  // load mesh in MSH BIN format

  void LoadSDF();                               // generate mesh using marching cubes
  void MakeGraph();                             // make graph of convex hull
  void CopyGraph();                             // copy graph into face data
  void MakeNormal();                            // compute vertex normals
  void MakeCenter();                            // compute face circumcircle data
  void Process();                               // compute inertial properties
  void ApplyTransformations();                  // apply user transformations
  double ComputeFaceCentroid(double[3]) const;  // compute centroid of all faces
  void CheckInitialMesh() const;                // check if initial mesh is valid
  void CopyPlugin();
  void Rotate(double quat[4]);                      // rotate mesh by quaternion
  void Transform(double pos[3], double quat[4]);    // transform mesh by position and quaternion
  void MakePolygons();                              // compute the polygon sides of the mesh
  void MakePolygonNormals();                        // compute the normals of the polygons

  // computes the inertia matrix of the mesh given the type of inertia
  double ComputeInertia(double inert[6], const double CoM[3]) const;

  int* GraphFaces() const {
    return graph_ + 2 + 3*(graph_[0] + graph_[1]);
  }

  // mesh data to be copied into sim_model_t
  double* center_;                    // face circumcenter data (3*nface)
  int* graph_;                        // convex graph data

  // mesh data for collision detection
  std::vector<std::vector<int>> polygons_;      // polygons of the mesh
  std::vector<double> polygon_normals_;         // normals of the polygons
  std::vector<std::vector<int>> polygon_map_;   // map from vertex to polygon

  // compute the volume and center-of-mass of the mesh given the face centroid
  double ComputeVolume(double CoM[3], const double facecen[3]) const;
  // compute the surface area and center-of-mass of the mesh given the face centroid
  double ComputeSurfaceArea(double CoM[3], const double facecen[3]) const;
};



//------------------------- class SIM_CSkin ---------------------------------------------------------
// Describes a skin

class SIM_CSkin_ : public sim_builder_base_t {
 protected:
  // variable size attributes
  std::string file_;
  std::string material_;
  std::vector<float> vert_;
  std::vector<float> texcoord_;
  std::vector<int> face_;
  std::vector<std::string> bodyname_;
  std::vector<float> bindpos_;
  std::vector<float> bindquat_;
  std::vector<std::vector<int>> vertid_;
  std::vector<std::vector<float>> vertweight_;

  std::string spec_file_;
  std::string spec_material_;
  std::vector<float> spec_vert_;
  std::vector<float> spec_texcoord_;
  std::vector<int> spec_face_;
  std::vector<std::string> spec_bodyname_;
  std::vector<float> spec_bindpos_;
  std::vector<float> spec_bindquat_;
  std::vector<std::vector<int>> spec_vertid_;
  std::vector<std::vector<float>> spec_vertweight_;

  int matid;                          // material id
  std::vector<int> bodyid;            // body ids

  // paths stored during model attachment
  simcore::user::FilePath modelfiledir_;
};

class SIM_CSkin: public SIM_CSkin_, private SIM_sSkin {
  friend class sim_builder_model_t;
  friend class sim_xml_writer_t;

 public:
  explicit SIM_CSkin(sim_builder_model_t* = nullptr);
  SIM_CSkin(const SIM_CSkin& other);
  SIM_CSkin& operator=(const SIM_CSkin& other);
  ~SIM_CSkin();

  SIM_sSkin spec;
  using sim_builder_base_t::info;

  const std::string& File() const { return file_; }
  const std::string& get_material() const { return material_; }
  const std::vector<float>& get_vert() const { return vert_; }
  const std::vector<float>& get_texcoord() const { return texcoord_; }
  const std::vector<int>& get_face() const { return face_; }
  const std::vector<std::string>& get_bodyname() const { return bodyname_; }
  const std::vector<float>& get_bindpos() const { return bindpos_; }
  const std::vector<float>& get_bindquat() const { return bindquat_; }
  const std::vector<std::vector<int>>& get_vertid() const { return vertid_; }
  const std::vector<std::vector<float>>& get_vertweight() const { return vertweight_; }
  void del_material() { material_.clear(); }

  void CopyFromSpec();
  void PointToLocal();

 private:
  void ResolveReferences(const sim_builder_model_t* m);
  void NameSpace(const sim_builder_model_t* m);
  void Compile(const SIM_VFS* vfs);             // compiler
  void LoadSKN(SIM_Resource* resource);         // load skin in SKN BIN format
};



//------------------------- class SIM_CHField --------------------------------------------------------
// Describes a height field

class SIM_CHField_ : public sim_builder_base_t {
 protected:
  std::vector<float> data;  // elevation data, row-major format

  std::string file_;
  std::string content_type_;
  std::vector<float> userdata_;
  std::string spec_file_;
  std::string spec_content_type_;
  std::vector<float> spec_userdata_;

  // paths stored during model attachment
  simcore::user::FilePath modelfiledir_;
};

class SIM_CHField : public SIM_CHField_, private SIM_sHField {
  friend class sim_builder_geom_t;
  friend class sim_builder_model_t;
  friend class sim_xml_writer_t;

 public:
  explicit SIM_CHField(sim_builder_model_t* model);
  SIM_CHField(const SIM_CHField& other);
  SIM_CHField& operator=(const SIM_CHField& other);
  ~SIM_CHField();

  SIM_sHField spec;
  using sim_builder_base_t::info;

  void CopyFromSpec(void);
  void PointToLocal(void);
  void NameSpace(const sim_builder_model_t* m);

  std::string File() const { return file_; }

  // getter for user data
  std::vector<float>& get_userdata() { return userdata_; }

 private:
  void Compile(const SIM_VFS* vfs);         // compiler

  std::string GetCacheId(const SIM_Resource* resource, const std::string& asset_type);
  void LoadCustom(SIM_Resource* resource);  // load from custom format
  void LoadPNG(SIM_Resource* resource);     // load from PNG format
};



//------------------------- class sim_builder_texture_t -------------------------------------------------------
// Describes a texture

class SIM_CTexture_ : public sim_builder_base_t {
 protected:
  std::vector<std::byte> data_;  // texture data (rgb, roughness, etc.)

  std::string file_;
  std::string content_type_;
  std::vector<std::string> cubefiles_;
  std::string spec_file_;
  std::string spec_content_type_;
  std::vector<std::string> spec_cubefiles_;

  // paths stored during model attachment
  simcore::user::FilePath modelfiledir_;
};

class sim_builder_texture_t : public SIM_CTexture_, private SIM_sTexture {
  friend class sim_builder_model_t;
  friend class sim_xml_reader_t;
  friend class sim_xml_writer_t;

 public:
  explicit sim_builder_texture_t(sim_builder_model_t*);
  sim_builder_texture_t(const sim_builder_texture_t& other);
  sim_builder_texture_t& operator=(const sim_builder_texture_t& other);
  ~sim_builder_texture_t();

  SIM_sTexture spec;
  using sim_builder_base_t::info;

  void CopyFromSpec(void);
  void PointToLocal(void);
  void NameSpace(const sim_builder_model_t* m);
  void Compile(const SIM_VFS* vfs);

  std::string File() const { return file_; }
  std::string get_content_type() const { return content_type_; }
  std::vector<std::string> get_cubefiles() const { return cubefiles_; }

 private:
  // store texture into asset cache
  std::string GetCacheId(const SIM_Resource* resource, const std::string& asset_type);
  void Builtin2D(void);                                 // make builtin 2D
  void BuiltinCube(void);                               // make builtin cube
  void Load2D(std::string filename, const SIM_VFS* vfs);  // load 2D from file
  void LoadCubeSingle(std::string filename,
                      const SIM_VFS* vfs);    // load cube from single file
  void LoadCubeSeparate(const SIM_VFS* vfs);  // load cube from separate files

  void FlipIfNeeded(std::vector<std::byte>& image, unsigned int w, unsigned int h);

  void LoadFlip(std::string filename, const SIM_VFS* vfs,  // load and flip
                std::vector<std::byte>& image, unsigned int& w, unsigned int& h,
                bool& is_srgb);

  void LoadPNG(SIM_Resource* resource, std::vector<std::byte>& image,
               unsigned int& w, unsigned int& h, bool& is_srgb);
  void LoadKTX(SIM_Resource* resource, std::vector<std::byte>& image,
               unsigned int& w, unsigned int& h, bool& is_srgb);
  void LoadCustom(SIM_Resource* resource, std::vector<std::byte>& image,
                  unsigned int& w, unsigned int& h, bool& is_srgb);

  bool clear_data_;  // if true, data_ is empty and should be filled by Compile
};



//------------------------- class SIM_CMaterial ------------------------------------------------------
// Describes a material for rendering

class SIM_CMaterial_ : public sim_builder_base_t {
 protected:
  int texid[SIM_NTEXROLE];                    // id of material's textures
  std::vector<std::string> textures_;
  std::vector<std::string> spec_textures_;
};

class SIM_CMaterial : public SIM_CMaterial_, private SIM_sMaterial {
  friend class sim_builder_default_t;
  friend class sim_builder_model_t;
  friend class sim_xml_writer_t;

 public:
  explicit SIM_CMaterial(sim_builder_model_t* = nullptr, sim_builder_default_t* = nullptr);
  SIM_CMaterial(const SIM_CMaterial& other);
  SIM_CMaterial& operator=(const SIM_CMaterial& other);

  SIM_sMaterial spec;
  using sim_builder_base_t::info;

  void CopyFromSpec();
  void PointToLocal();
  void NameSpace(const sim_builder_model_t* m);

  const std::string& get_texture(int i) const { return textures_[i]; }
  void del_textures() { for (auto& t : textures_) t.clear(); }

 private:
  void Compile(void);                       // compiler
};



//------------------------- class SIM_CPair ----------------------------------------------------------
// Predefined geom pair for collision detection

class SIM_CPair_ : public sim_builder_base_t {
 protected:
  int signature;                  // body1<<16 + body2
  std::string geomname1_;
  std::string geomname2_;
  std::string spec_geomname1_;
  std::string spec_geomname2_;
};

class SIM_CPair : public SIM_CPair_, private SIM_sPair {
  friend class sim_builder_default_t;
  friend class sim_builder_body_t;
  friend class sim_builder_model_t;
  friend class sim_xml_writer_t;

 public:
  explicit SIM_CPair(sim_builder_model_t* = nullptr, sim_builder_default_t* = nullptr);
  SIM_CPair(const SIM_CPair& other);
  SIM_CPair& operator=(const SIM_CPair& other);

  SIM_sPair spec;
  using sim_builder_base_t::info;

  void CopyFromSpec();
  void PointToLocal();
  void ResolveReferences(const sim_builder_model_t* m);
  void NameSpace(const sim_builder_model_t* m);

  const std::string& get_geomname1() const { return geomname1_; }
  const std::string& get_geomname2() const { return geomname2_; }

  int GetSignature(void) {
    return signature;
  }

 private:
  void Compile(void);                   // compiler

  sim_builder_geom_t* geom1;                 // geom1
  sim_builder_geom_t* geom2;                 // geom2
};



//------------------------- class SIM_CBodyPair ------------------------------------------------------
// Body pair specification, use to exclude pairs

class SIM_CBodyPair_ : public sim_builder_base_t {
 protected:
  int body1;                       // id of body1
  int body2;                       // id of body2
  int signature;                   // body1<<16 + body2

  std::string bodyname1_;          // name of geom 1
  std::string bodyname2_;          // name of geom 2
  std::string spec_bodyname1_;
  std::string spec_bodyname2_;
};

class SIM_CBodyPair : public SIM_CBodyPair_, private SIM_sExclude {
  friend class sim_builder_body_t;
  friend class sim_builder_model_t;

 public:
  explicit SIM_CBodyPair(sim_builder_model_t*);
  SIM_CBodyPair(const SIM_CBodyPair& other);
  SIM_CBodyPair& operator=(const SIM_CBodyPair& other);

  SIM_sExclude spec;
  using sim_builder_base_t::info;

  void CopyFromSpec();
  void PointToLocal();
  void ResolveReferences(const sim_builder_model_t* m);
  void NameSpace(const sim_builder_model_t* m);

  std::string get_bodyname1() const { return bodyname1_; }
  std::string get_bodyname2() const { return bodyname2_; }

  int GetSignature() {
    return signature;
  }

 private:
  void Compile();              // compiler
};



//------------------------- class SIM_CEquality ------------------------------------------------------
// Describes an equality constraint

class SIM_CEquality_ : public sim_builder_base_t {
 protected:
  int obj1id;
  int obj2id;
  std::string name1_;
  std::string name2_;
  std::string spec_name1_;
  std::string spec_name2_;
};

class SIM_CEquality : public SIM_CEquality_, private SIM_sEquality {
  friend class sim_builder_default_t;
  friend class sim_builder_body_t;
  friend class sim_builder_model_t;
  friend class sim_xml_writer_t;

 public:
  explicit SIM_CEquality(sim_builder_model_t* = 0, sim_builder_default_t* = 0);
  SIM_CEquality(const SIM_CEquality& other);
  SIM_CEquality& operator=(const SIM_CEquality& other);

  SIM_sEquality spec;
  using sim_builder_base_t::info;

  void CopyFromSpec();
  void PointToLocal();
  void ResolveReferences(const sim_builder_model_t* m);
  void NameSpace(const sim_builder_model_t* m);

 private:
  void Compile(void);                       // compiler
};



//------------------------- class sim_builder_tendon_t --------------------------------------------------------
// Describes a tendon

class SIM_CTendon_ : public sim_builder_base_t {
 protected:
  int matid;  // material id for rendering

  // variable-size data
  std::string material_;
  std::string spec_material_;
  std::vector<double> userdata_;
  std::vector<double> spec_userdata_;
};

class sim_builder_tendon_t : public SIM_CTendon_, private SIM_sTendon {
  friend class sim_builder_default_t;
  friend class sim_builder_model_t;
  friend class sim_xml_writer_t;

 public:
  explicit sim_builder_tendon_t(sim_builder_model_t* = nullptr, sim_builder_default_t* = nullptr);
  sim_builder_tendon_t(const sim_builder_tendon_t& other);
  sim_builder_tendon_t& operator=(const sim_builder_tendon_t& other);
  ~sim_builder_tendon_t();

  SIM_sTendon spec;
  using sim_builder_base_t::info;

  void set_material(std::string _material) { material_ = _material; }
  const std::string& get_material() const { return material_; }
  void del_material() { material_.clear(); }

  // API for adding wrapping objects
  void WrapSite(std::string wrapname, std::string_view wrapinfo = "");                    // site
  void WrapGeom(std::string wrapname, std::string side, std::string_view wrapinfo = "");  // geom
  void WrapJoint(std::string wrapname, double coef, std::string_view wrapinfo = "");      // joint
  void WrapPulley(double divisor, std::string_view wrapinfo = "");                        // pulley

  // API for access to wrapping objects
  int NumWraps() const;                       // number of wraps
  const SIM_CWrap* GetWrap(int i) const;        // pointer to wrap
  std::vector<SIM_CWrap*> path;                 // wrapping objects

  // used by sim_xml_writer_t and sim_builder_model_t
  const std::vector<double>& get_userdata() const { return userdata_; }
  const double* get_range() { return range; }

  void CopyFromSpec();
  void PointToLocal();
  void ResolveReferences(const sim_builder_model_t* m);
  void NameSpace(const sim_builder_model_t* m);
  void SetModel(sim_builder_model_t* _model);

  bool is_limited() const;
  bool is_actfrclimited() const;

 private:
  void Compile(void);                         // compiler
};



//------------------------- class SIM_CWrap ----------------------------------------------------------
// Describes a tendon wrap object

class SIM_CWrap_ : public sim_builder_base_t {
 public:
  int sideid;                     // side site id; -1 if not applicable
  double prm;                     // parameter: divisor, coefficient
  std::string sidesite;           // name of side site
};

class SIM_CWrap : public SIM_CWrap_, private SIM_sWrap {
  friend class sim_builder_tendon_t;
  friend class sim_builder_model_t;

 public:
  SIM_sWrap spec;
  using sim_builder_base_t::info;

  void CopyFromSpec();
  void PointToLocal();
  void ResolveReferences(const sim_builder_model_t* m);
  void NameSpace(const sim_builder_model_t* m);
  SIM_tWrap Type() const { return spec.type; }

  sim_builder_base_t* obj;                   // wrap object pointer

 private:
  SIM_CWrap(sim_builder_model_t*, sim_builder_tendon_t*);            // constructor
  SIM_CWrap(const SIM_CWrap& other);             // copy constructor
  SIM_CWrap& operator=(const SIM_CWrap& other);  // copy assignment

  void Compile(void);              // compiler

  sim_builder_tendon_t* tendon;              // tendon owning this wrap
};



//------------------------- class sim_builder_plugin_t --------------------------------------------------------
// Describes an instance of a plugin

class SIM_CPlugin_ : public sim_builder_base_t {
 public:
  int nstate;        // state size for the plugin instance
  std::map<std::string, std::string, std::less<>> config_attribs;  // raw config attributes from XML
  std::vector<char> flattened_attributes;  // config attributes flattened in plugin-declared order;

 protected:
  std::string plugin_name;
};

class sim_builder_plugin_t : public SIM_CPlugin_ {
  friend class sim_builder_model_t;
  friend class sim_xml_writer_t;

 public:
  explicit sim_builder_plugin_t(sim_builder_model_t*);
  sim_builder_plugin_t(const sim_builder_plugin_t& other);
  sim_builder_plugin_t& operator=(const sim_builder_plugin_t& other);

  void PointToLocal();

  SIM_sPlugin spec;
  sim_builder_base_t* parent;  // parent object (only used when generating error message)
  int plugin_slot;  // global registered slot number of the plugin

 private:
  void Compile(void);              // compiler
};



//------------------------- class SIM_CActuator ------------------------------------------------------
// Describes an actuator

class SIM_CActuator_ : public sim_builder_base_t {
 protected:
  int trnid[2];                   // id of transmission target

  // variable used for temporarily storing the state of the actuator
  int actadr_;                                      // address of dof in data->act
  int actdim_;                                      // number of dofs in data->act
  std::map<std::string, std::vector<sim_scalar_t>> act_;  // act at the previous step
  std::map<std::string, sim_scalar_t> ctrl_;              // ctrl at the previous step

  // variable-size data
  std::string plugin_name;
  std::string plugin_instance_name;
  std::string target_;
  std::string slidersite_;
  std::string refsite_;
  std::vector<double> userdata_;
  std::string spec_target_;
  std::string spec_slidersite_;
  std::string spec_refsite_;
  std::vector<double> spec_userdata_;
};

class SIM_CActuator : public SIM_CActuator_, private sim_spec_actuator_t {
  friend class sim_builder_default_t;
  friend class sim_builder_model_t;
  friend class sim_xml_writer_t;

 public:
  explicit SIM_CActuator(sim_builder_model_t* = nullptr, sim_builder_default_t* = nullptr);
  SIM_CActuator(const SIM_CActuator& other);
  SIM_CActuator& operator=(const SIM_CActuator& other);

  sim_spec_actuator_t spec;
  using sim_builder_base_t::info;

  // used by sim_xml_writer_t and sim_builder_model_t
  const std::vector<double>& get_userdata() const { return userdata_; }
  const std::string& get_target() const { return spec_target_; }
  const std::string& get_slidersite() const { return spec_slidersite_; }
  const std::string& get_refsite() const { return spec_refsite_; }

  bool is_ctrllimited() const;
  bool is_forcelimited() const;
  bool is_actlimited() const;

  std::vector<sim_scalar_t>& act(const std::string& state_name);
  sim_scalar_t& ctrl(const std::string& state_name);

 private:
  void Compile(void);                       // compiler
  void CopyFromSpec();
  void PointToLocal();
  void ResolveReferences(const sim_builder_model_t* m);
  void NameSpace(const sim_builder_model_t* m);
  void CopyPlugin();

  // reset keyframe references for allowing self-attach
  void ForgetKeyframes();

  sim_builder_base_t* ptarget;  // transmission target
};



//------------------------- class SIM_CSensor --------------------------------------------------------
// Describes a sensor

class SIM_CSensor_ : public sim_builder_base_t {
 protected:
  // variable-size data
  std::string plugin_name;
  std::string plugin_instance_name;
  std::string objname_;
  std::string refname_;
  std::vector<double> userdata_;
  std::string spec_objname_;
  std::string spec_refname_;
  std::vector<double> spec_userdata_;
};

class SIM_CSensor : public SIM_CSensor_, private SIM_sSensor {
  friend class sim_builder_default_t;
  friend class sim_builder_model_t;
  friend class sim_xml_writer_t;

 public:
  explicit SIM_CSensor(sim_builder_model_t*);
  SIM_CSensor(const SIM_CSensor& other);
  SIM_CSensor& operator=(const SIM_CSensor& other);

  SIM_sSensor spec;
  using sim_builder_base_t::info;

  // used by sim_xml_writer_t and sim_builder_model_t
  const std::vector<double>& get_userdata() { return userdata_; }
  const std::string& get_objname() { return spec_objname_; }
  const std::string& get_refname() { return spec_refname_; }

  const sim_builder_base_t* get_obj() { return obj; }
  const sim_builder_base_t* get_ref() { return ref; }

 private:
  void Compile(void);             // compiler
  void CopyFromSpec();
  void PointToLocal();
  void ResolveReferences(const sim_builder_model_t* m);
  void NameSpace(const sim_builder_model_t* m);
  void CopyPlugin();

  sim_builder_base_t* obj;                   // sensorized object
  sim_builder_base_t* ref;                   // sensorized reference
};



//------------------------- class SIM_CNumeric -------------------------------------------------------
// Describes a custom data field

class SIM_CNumeric_ : public sim_builder_base_t {
 protected:
  std::vector<double> data_;
  std::vector<double> spec_data_;
};

class SIM_CNumeric : public SIM_CNumeric_, private SIM_sNumeric {
  friend class sim_builder_model_t;
  friend class sim_xml_writer_t;

 public:
  explicit SIM_CNumeric(sim_builder_model_t*);
  SIM_CNumeric(const SIM_CNumeric& other);
  SIM_CNumeric& operator=(const SIM_CNumeric& other);
  ~SIM_CNumeric();

  SIM_sNumeric spec;
  using sim_builder_base_t::info;

  void PointToLocal();
  void CopyFromSpec();

 private:
  void Compile(void);                 // compiler
};



//------------------------- class SIM_CText ----------------------------------------------------------
// Describes a custom text field

class SIM_CText_ : public sim_builder_base_t {
 protected:
  std::string data_;
  std::string spec_data_;
};

class SIM_CText : public SIM_CText_, private SIM_sText {
  friend class sim_builder_model_t;
  friend class sim_xml_writer_t;

 public:
  explicit SIM_CText(sim_builder_model_t*);
  SIM_CText(const SIM_CText& other);
  SIM_CText& operator=(const SIM_CText& other);
  ~SIM_CText();

  SIM_sText spec;
  using sim_builder_base_t::info;

  void PointToLocal();
  void CopyFromSpec();

 private:
  void Compile(void);                 // compiler
};



//------------------------- class SIM_CTuple ---------------------------------------------------------
// Describes a custom tuple field

class SIM_CTuple_ : public sim_builder_base_t {
 protected:
  std::vector<sim_builder_base_t*> obj;  // object pointers
  std::vector<sim_obj_t> objtype_;
  std::vector<std::string> objname_;
  std::vector<double> objprm_;
  std::vector<sim_obj_t> spec_objtype_;
  std::vector<std::string> spec_objname_;
  std::vector<double> spec_objprm_;
};

class SIM_CTuple : public SIM_CTuple_, private SIM_sTuple {
  friend class sim_builder_model_t;
  friend class sim_xml_writer_t;

 public:
  explicit SIM_CTuple(sim_builder_model_t*);
  SIM_CTuple(const SIM_CTuple& other);
  SIM_CTuple& operator=(const SIM_CTuple& other);
  ~SIM_CTuple();

  SIM_sTuple spec;
  using sim_builder_base_t::info;

  void PointToLocal();
  void CopyFromSpec();
  void ResolveReferences(const sim_builder_model_t* m);
  void NameSpace(const sim_builder_model_t* m);

 private:
  void Compile(void);             // compiler
};



//------------------------- class SIM_CKey -----------------------------------------------------------
// Describes a keyframe

class SIM_CKey_ : public sim_builder_base_t {
 protected:
  std::vector<double> qpos_;
  std::vector<double> qvel_;
  std::vector<double> act_;
  std::vector<double> mpos_;
  std::vector<double> mquat_;
  std::vector<double> ctrl_;
  std::vector<double> spec_qpos_;
  std::vector<double> spec_qvel_;
  std::vector<double> spec_act_;
  std::vector<double> spec_mpos_;
  std::vector<double> spec_mquat_;
  std::vector<double> spec_ctrl_;
};

class SIM_CKey : public SIM_CKey_, private SIM_sKey {
  friend class sim_builder_model_t;
  friend class sim_xml_writer_t;

 public:
  explicit SIM_CKey(sim_builder_model_t*);
  SIM_CKey(const SIM_CKey& other);
  SIM_CKey& operator=(const SIM_CKey& other);
  ~SIM_CKey();

  SIM_sKey spec;
  using sim_builder_base_t::info;

  void PointToLocal();
  void CopyFromSpec();

 private:
  void Compile(const sim_model_t* m);  // compiler
};



//------------------------- class sim_builder_default_t -----------------------------------------------------------
// Describes one set of defaults

class sim_builder_default_t : public sim_spec_element_t {
  friend class sim_xml_writer_t;

 public:
  sim_builder_default_t();
  explicit sim_builder_default_t(sim_builder_model_t*);
  sim_builder_default_t(const sim_builder_default_t& other);
  sim_builder_default_t& operator=(const sim_builder_default_t& other);
  sim_builder_default_t& operator+=(const sim_builder_default_t& other);

  void CopyWithoutChildren(const sim_builder_default_t& other);
  void PointToLocal(void);
  void CopyFromSpec(void);
  void NameSpace(const sim_builder_model_t* m);

  void Compile(const sim_builder_model_t* model);

  // accessors
  sim_builder_joint_t& Joint() { return joint_; }
  sim_builder_geom_t& Geom() { return geom_; }
  sim_builder_site_t& Site() { return site_; }
  SIM_CCamera& Camera() { return camera_; }
  SIM_CLight& Light() { return light_; }
  SIM_CFlex& Flex() { return flex_; }
  sim_builder_mesh_t& Mesh() { return mesh_; }
  SIM_CMaterial& Material() { return material_; }
  SIM_CPair& Pair() { return pair_; }
  SIM_CEquality& Equality() { return equality_; }
  sim_builder_tendon_t& Tendon() { return tendon_; }
  SIM_CActuator& Actuator() { return actuator_; }

  // identifiers
  std::string name;               // class name
  int id;                         // id of this default
  sim_builder_default_t* parent;                 // id of parent class
  std::vector<sim_builder_default_t*> child;     // child classes

  sim_spec_default_t spec;
  sim_builder_model_t* model;                // pointer to model that owns object

 private:
  sim_builder_joint_t joint_;
  sim_builder_geom_t geom_;
  sim_builder_site_t site_;
  SIM_CCamera camera_;
  SIM_CLight light_;
  SIM_CFlex flex_;
  sim_builder_mesh_t mesh_;
  SIM_CMaterial material_;
  SIM_CPair pair_;
  SIM_CEquality equality_;
  sim_builder_tendon_t tendon_;
  SIM_CActuator actuator_;
};

#endif  // SIMCORE_SRC_USER_USER_OBJECTS_H_
