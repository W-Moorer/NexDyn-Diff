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

#ifndef SIMCORE_SRC_USER_USER_MODEL_H_
#define SIMCORE_SRC_USER_USER_MODEL_H_

#include <array>
#include <cstdint>
#include <functional>
#include <map>
#include <sstream>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

#include <simcore/SIM_data.h>
#include <simcore/SIM_model.h>
#include <simcore/SIM_plugin.h>
#include <simcore/SIM_spec.h>
#include <simcore/SIM_tnum.h>
#include "user/user_objects.h"

typedef std::map<std::string, int, std::less<> > SIM_KeyMap;
typedef std::array<SIM_KeyMap, SIM_NOBJECT> SIM_ListKeyMap;

typedef struct SIM_KeyInfo_ {
  std::string name;
  double time;
  bool qpos;
  bool qvel;
  bool act;
  bool ctrl;
  bool mpos;
  bool mquat;
} SIM_KeyInfo;

class SIM_CModel_ : public sim_spec_element_t {
 public:
  // attach namespaces
  std::string prefix;
  std::string suffix;

 protected:
  bool compiled;     // already compiled flag

  // sizes set from object list lengths
  sim_size_t nbody;     // number of bodies
  sim_size_t njnt;      // number of joints
  sim_size_t ngeom;     // number of geoms
  sim_size_t nsite;     // number of sites
  sim_size_t ncam;      // number of cameras
  sim_size_t nlight;    // number of lights
  sim_size_t nflex;     // number of flexes
  sim_size_t nmesh;     // number of meshes
  sim_size_t nskin;     // number of skins
  sim_size_t nhfield;   // number of height fields
  sim_size_t ntex;      // number of textures
  sim_size_t nmat;      // number of materials
  sim_size_t npair;     // number of geom pairs in pair array
  sim_size_t nexclude;  // number of excluded body pairs
  sim_size_t neq;       // number of equality constraints
  sim_size_t ntendon;   // number of tendons
  sim_size_t nsensor;   // number of sensors
  sim_size_t nnumeric;  // number of numeric fields
  sim_size_t ntext;     // number of text fields
  sim_size_t ntuple;    // number of tuple fields
  sim_size_t nmocap;    // number of mocap bodies
  sim_size_t nplugin;   // number of plugin instances

  // sizes computed by Compile
  sim_size_t nq;              // number of generalized coordinates = dim(qpos)
  sim_size_t nv;              // number of degrees of freedom = dim(qvel)
  sim_size_t nu;              // number of actuators/controls
  sim_size_t na;              // number of activation variables
  sim_size_t ntree;           // number of trees
  sim_size_t nbvh;            // number of total boundary volume hierarchies
  sim_size_t nbvhstatic;      // number of static boundary volume hierarchies
  sim_size_t nbvhdynamic;     // number of dynamic boundary volume hierarchies
  sim_size_t noct;            // number of total octree cells
  sim_size_t nflexnode;       // number of nodes in all flexes
  sim_size_t nflexvert;       // number of vertices in all flexes
  sim_size_t nflexedge;       // number of edges in all flexes
  sim_size_t nflexelem;       // number of elements in all flexes
  sim_size_t nflexelemdata;   // number of element vertex ids in all flexes
  sim_size_t nflexelemedge;   // number of element edges in all flexes
  sim_size_t nflexshelldata;  // number of shell fragment vertex ids in all flexes
  sim_size_t nflexevpair;     // number of element-vertex pairs in all flexes
  sim_size_t nflextexcoord;   // number of vertex texture coordinates in all flexes
  sim_size_t nJfe;            // number of non-zeros in sparse flex edge constraint Jacobian
  sim_size_t nJfv;            // number of non-zeros in sparse flex vertex constraint Jacobian
  sim_size_t nmeshvert;       // number of vertices in all meshes
  sim_size_t nmeshnormal;     // number of normals in all meshes
  sim_size_t nmeshtexcoord;   // number of texture coordinates in all meshes
  sim_size_t nmeshface;       // number of triangular faces in all meshes
  sim_size_t nmeshpoly;       // number of polygon faces in all meshes
  sim_size_t nmeshgraph;      // number of ints in mesh auxiliary data
  sim_size_t nmeshpolyvert;   // number of vertices in all polygon faces
  sim_size_t nmeshpolymap;    // number of polygons in vertex map
  sim_size_t nskinvert;       // number of vertices in all skins
  sim_size_t nskintexvert;    // number of vertices with texcoord in all skins
  sim_size_t nskinface;       // number of faces in all skins
  sim_size_t nskinbone;       // number of bones in all skins
  sim_size_t nskinbonevert;   // number of vertices in all skins
  sim_size_t nhfielddata;     // number of data points in all hfields
  sim_size_t ntexdata;        // number of texture bytes
  sim_size_t nwrap;           // number of wrap objects in all tendon paths
  sim_size_t nsensordata;     // number of SIM_tNums in sensor data vector
  sim_size_t nhistory;        // number of SIM_tNums in history buffer
  sim_size_t nnumericdata;    // number of SIM_tNums in all custom fields
  sim_size_t ntextdata;       // number of chars in all text fields, including 0
  sim_size_t ntupledata;      // number of objects in all tuple fields
  sim_size_t npluginattr;     // number of chars in all plugin config attributes
  sim_size_t nnames;          // number of chars in all names
  sim_size_t npaths;          // number of chars in all paths
  sim_size_t nM;              // number of non-zeros in sparse inertia matrix
  sim_size_t nB;              // number of non-zeros in sparse body-dof matrix
  sim_size_t nC;              // number of non-zeros in reduced sparse dof-dof matrix
  sim_size_t nD;              // number of non-zeros in sparse dof-dof matrix
  sim_size_t nJmom;           // number of non-zeros in sparse actuator_moment matrix
  sim_size_t nJten;           // number of non-zeros in sparse ten_J matrix

  // statistics, as computed by sim_setConst
  double meaninertia_auto;  // mean diagonal inertia, as computed by sim_setConst
  double meanmass_auto;     // mean body mass, as computed by sim_setConst
  double meansize_auto;     // mean body size, as computed by sim_setConst
  double extent_auto;       // spatial extent, as computed by sim_setConst
  double center_auto[3];    // center of model, as computed by sim_setConst

  // save qpos0, to recognize changed key_qpos in write
  std::vector<sim_scalar_t> qpos0;
  std::vector<sim_scalar_t> body_pos0;
  std::vector<sim_scalar_t> body_quat0;

  // variable-size attributes
  std::string comment_;           // comment at top of XML
  std::string modelfiledir_;      // path to model file
  std::string modelname_;
  std::string meshdir_;
  std::string texturedir_;
  std::string spec_comment_;
  std::string spec_modelfiledir_;
  std::string spec_modelname_;
};

// sim_builder_model_t contains everything needed to generate the low-level model.
// It can be constructed manually by calling 'Add' functions and setting
// the public fields of the various objects.  Alternatively it can constructed
// by loading an XML file via SIM_CXML.  Once an sim_builder_model_t object is
// constructed, 'Compile' can be called to generate the corresponding sim_model_t object
// (which is the low-level model).  The sim_builder_model_t object can then be deleted.
class sim_builder_model_t : public SIM_CModel_, private sim_spec_t {
  friend class sim_builder_base_t;
  friend class sim_builder_body_t;
  friend class SIM_CCamera;
  friend class sim_builder_geom_t;
  friend class SIM_CFlex;
  friend class SIM_CHField;
  friend class sim_builder_frame_t;
  friend class sim_builder_joint_t;
  friend class SIM_CEquality;
  friend class sim_builder_mesh_t;
  friend class SIM_CSkin;
  friend class sim_builder_site_t;
  friend class sim_builder_tendon_t;
  friend class sim_builder_texture_t;
  friend class SIM_CActuator;
  friend class SIM_CSensor;
  friend class sim_builder_default_t;
  friend class sim_xml_reader_t;
  friend class sim_xml_writer_t;

 public:
  sim_builder_model_t();
  sim_builder_model_t(const sim_builder_model_t& other);
  ~sim_builder_model_t();
  void CopyFromSpec();  // copy spec to private attributes
  void PointToLocal();

  sim_builder_model_t& operator=(const sim_builder_model_t& other);     // copy other into this, if they are not the same
  sim_builder_model_t& operator+=(const sim_builder_model_t& other);    // add other into this, even if they are the same
  sim_builder_model_t& operator-=(const sim_builder_body_t& subtree);   // remove subtree and all references from model
  sim_builder_model_t& operator+=(sim_builder_default_t& subtree);          // add default tree to this model
  sim_builder_model_t& operator-=(const sim_builder_default_t& subtree);    // remove default tree from this model

  sim_spec_t spec;

  sim_model_t* Compile(const SIM_VFS* vfs = nullptr, sim_model_t** m = nullptr);  // construct sim_model_t
  bool CopyBack(const sim_model_t*);                 // DECOMPILER: copy numeric back
  void FuseStatic();                             // fuse static bodies with parent
  void FuseReindex(sim_builder_body_t* body);               // reindex elements during fuse

  // API for adding model elements
  SIM_CFlex* AddFlex();
  sim_builder_mesh_t* AddMesh(sim_builder_default_t* def = nullptr);
  SIM_CSkin* AddSkin();
  SIM_CHField* AddHField();
  sim_builder_texture_t* AddTexture();
  SIM_CMaterial* AddMaterial(sim_builder_default_t* def = nullptr);
  SIM_CPair* AddPair(sim_builder_default_t* def = nullptr);          // geom pair for inclusion
  SIM_CBodyPair* AddExclude();                        // body pair for exclusion
  SIM_CEquality* AddEquality(sim_builder_default_t* def = nullptr);  // equality constraint
  sim_builder_tendon_t* AddTendon(sim_builder_default_t* def = nullptr);
  SIM_CActuator* AddActuator(sim_builder_default_t* def = nullptr);
  SIM_CSensor* AddSensor();
  SIM_CNumeric* AddNumeric();
  SIM_CText* AddText();
  SIM_CTuple* AddTuple();
  SIM_CKey* AddKey();
  sim_builder_plugin_t* AddPlugin();

  // append spec to this model, optionally map compiler options to the appended spec
  void AppendSpec(sim_spec_t* spec, const SIM_sCompiler* compiler = nullptr);

  // delete elements marked as discard=true
  template <class T> void Delete(std::vector<T*>& elements,
                                 const std::vector<bool>& discard);

  // delete all elements
  template <class T> void DeleteAll(std::vector<T*>& elements);

  // delete object from the corresponding list
  void operator-=(sim_spec_element_t* el);

  // delete default and all descendants
  void RemoveDefault(sim_builder_default_t* def);

  // API for access to model elements (outside tree)
  int NumObjects(sim_obj_t type);              // number of objects in specified list
  sim_builder_base_t* GetObject(sim_obj_t type, int id);  // pointer to specified object
  sim_spec_element_t* NextObject(sim_spec_element_t* object, sim_obj_t type = SIM_OBJ_UNKNOWN);  // next object of specified type

  // API for access to other variables
  bool IsCompiled() const;                                          // is model already compiled
  const sim_builder_error_t& GetError() const;                                 // get reference of error object
  void SetError(const sim_builder_error_t& error) { errInfo = error; }         // set value of error object
  sim_builder_body_t* GetWorld();                                              // pointer to world body
  sim_builder_default_t* FindDefault(std::string name);                            // find defaults class name
  sim_builder_default_t* AddDefault(std::string name, sim_builder_default_t* parent = nullptr);   // add defaults class to array
  sim_builder_base_t* FindObject(sim_obj_t type, std::string name) const;         // find object given type and name
  sim_builder_base_t* FindTree(sim_builder_body_t* body, sim_obj_t type, std::string name);  // find tree object given name
  sim_spec_t* FindSpec(std::string name) const;                         // find spec given name
  sim_spec_t* FindSpec(const SIM_sCompiler* compiler_);                   // find spec given SIM_sCompiler
  void ActivatePlugin(const SIM_pPlugin* plugin, int slot);           // activate plugin

  // find asset given name checking both name and filename
  template <class T>
  sim_builder_base_t* FindAsset(std::string_view name, const std::vector<T*>& list) const;

  // accessors
  std::string get_meshdir() const { return meshdir_; }
  std::string get_texturedir() const { return texturedir_; }

  sim_builder_default_t* Default() const { return defaults_[0]; }
  int NumDefaults() const { return defaults_.size(); }

  const std::vector<std::pair<const SIM_pPlugin*, int>>& ActivePlugins() const {
    return active_plugins_;
  };

  const std::vector<SIM_CFlex*>& Flexes() const { return flexes_; }
  const std::vector<sim_builder_mesh_t*>& Meshes() const {return meshes_; }
  const std::vector<SIM_CSkin*>& Skins() const { return skins_; }
  const std::vector<SIM_CHField*>& HFields() const { return hfields_; }
  const std::vector<sim_builder_texture_t*>& Textures() const { return textures_; }
  const std::vector<SIM_CMaterial*>& Materials() const { return materials_; }
  const std::vector<SIM_CPair*>& Pairs() const { return pairs_; }
  const std::vector<SIM_CBodyPair*>& Excludes() const { return excludes_; }
  const std::vector<SIM_CEquality*>& Equalities() const { return equalities_; }
  const std::vector<sim_builder_tendon_t*>& Tendons() const { return tendons_; }
  const std::vector<SIM_CActuator*>& Actuators() const { return actuators_; }
  const std::vector<SIM_CSensor*>& Sensors() const { return sensors_; }
  const std::vector<SIM_CNumeric*>& Numerics() const { return numerics_; }
  const std::vector<SIM_CText*>& Texts() const { return texts_; }
  const std::vector<SIM_CTuple*>& Tuples() const { return tuples_; }
  const std::vector<SIM_CKey*>& Keys() const { return keys_; }
  const std::vector<sim_builder_plugin_t*>& Plugins() const { return plugins_; }
  const std::vector<sim_builder_body_t*>& Bodies() const { return bodies_; }
  const std::vector<sim_builder_geom_t*>& Geoms() const { return geoms_; }

  // resolve plugin instance, create a new one if needed
  void ResolvePlugin(sim_builder_base_t* obj, const std::string& plugin_name,
                     const std::string& plugin_instance_name,
                     sim_builder_plugin_t** plugin_instance);

  // clear objects allocated by Compile
  void Clear();

  // delete material from object
  template <class T> void DeleteMaterial(std::vector<T*>& list,
                                         std::string_view name = "");

  // save the current state
  template <class T>
  void SaveState(const std::string& state_name, const T* qpos, const T* qvel, const T* act,
                 const T* ctrl, const T* mpos, const T* mquat);

  // restore the previously saved state
  template <class T>
  void RestoreState(const std::string& state_name, const sim_scalar_t* pos0, const sim_scalar_t* mpos0,
                    const sim_scalar_t* mquat0, T* qpos, T* qvel, T* act, T* ctrl, T* mpos, T* mquat);

  // clear existing data
  void MakeData(const sim_model_t* m, sim_data_t** dest);

  // resolve keyframe references
  void StoreKeyframes(sim_builder_model_t* dest);

  // map from default class name to default class pointer
  std::unordered_map<std::string, sim_builder_default_t*> def_map;

  // set deepcopy flag
  void SetDeepCopy(bool deepcopy) { deepcopy_ = deepcopy; }

  // set attached flag
  void SetAttached(bool deepcopy) { attached_ |= !deepcopy; }

  // check for repeated names in list
  void CheckRepeat(sim_obj_t type);

  // increment and decrement reference count
  void AddRef() { ++refcount; }
  int GetRef() const { return refcount; }
  void Release() {
    if (--refcount == 0) {
      delete this;
    }
  }

 private:
  int refcount = 1;

  // settings for each defaults class
  std::vector<sim_builder_default_t*> defaults_;

  // list of active plugins
  std::vector<std::pair<const SIM_pPlugin*, int>> active_plugins_;

  // make lists of bodies and children
  void MakeTreeLists(sim_builder_body_t* body = nullptr);

  // compile phases
  void TryCompile(sim_model_t*& m, sim_data_t*& d, const SIM_VFS* vfs);
  void CompileMeshesAndTextures(const SIM_VFS* vfs);

  void SetNuser();                      // set nuser fields
  void IndexAssets(bool discard);       // convert asset names into indices
  void CheckEmptyNames();               // check empty names
  void SetSizes();                      // compute sizes
  void ComputeSparseSizes();            // compute nM, nD, nB, nC
  void AutoSpringDamper(sim_model_t*);      // automatic stiffness and damping computation
  void LengthRange(sim_model_t*, sim_data_t*);  // compute actuator lengthrange
  void CopyNames(sim_model_t*);             // copy names, compute name addresses
  void CopyPaths(sim_model_t*);             // copy paths, compute path addresses
  void CopyObjects(sim_model_t*);           // copy objects outside kinematic tree
  void CopyTree(sim_model_t*);              // copy objects inside kinematic tree
  void FinalizeSimple(sim_model_t* m);      // finalize simple bodies/dofs including tendon information
  void CopyPlugins(sim_model_t*);           // copy plugin data
  int CountNJmom(const sim_model_t* m);     // compute number of non-zeros in actuator_moment matrix
  int CountNJten(const sim_model_t* m);     // compute number of non-zeros in ten_J matrix

  // remove plugins that are not referenced by any object
  void RemovePlugins();

  // objects created here
  std::vector<SIM_CFlex*>     flexes_;      // list of flexes
  std::vector<sim_builder_mesh_t*>     meshes_;      // list of meshes
  std::vector<SIM_CSkin*>     skins_;       // list of skins
  std::vector<SIM_CHField*>   hfields_;     // list of height fields
  std::vector<sim_builder_texture_t*>  textures_;    // list of textures
  std::vector<SIM_CMaterial*> materials_;   // list of materials
  std::vector<SIM_CPair*>     pairs_;       // list of geom pairs to include
  std::vector<SIM_CBodyPair*> excludes_;    // list of body pairs to exclude
  std::vector<SIM_CEquality*> equalities_;  // list of equality constraints
  std::vector<sim_builder_tendon_t*>   tendons_;     // list of tendons
  std::vector<SIM_CActuator*> actuators_;   // list of actuators
  std::vector<SIM_CSensor*>   sensors_;     // list of sensors
  std::vector<SIM_CNumeric*>  numerics_;    // list of numeric fields
  std::vector<SIM_CText*>     texts_;       // list of text fields
  std::vector<SIM_CTuple*>    tuples_;      // list of tuple fields
  std::vector<SIM_CKey*>      keys_;        // list of keyframe fields
  std::vector<sim_builder_plugin_t*>   plugins_;     // list of plugin instances
  std::vector<sim_spec_t*>      specs_;       // list of attached specs

  // pointers to objects created inside kinematic tree
  std::vector<sim_builder_body_t*>   bodies_;   // list of bodies
  std::vector<sim_builder_joint_t*>  joints_;   // list of joints allowing motion relative to parent
  std::vector<sim_builder_geom_t*>   geoms_;    // list of geoms attached to this body
  std::vector<sim_builder_site_t*>   sites_;    // list of sites attached to this body
  std::vector<SIM_CCamera*> cameras_;  // list of cameras
  std::vector<SIM_CLight*>  lights_;   // list of lights
  std::vector<sim_builder_frame_t*>  frames_;   // list of frames

  // array of pointers to each object list (enumerated by type)
  std::array<std::vector<sim_builder_base_t*>*, SIM_NOBJECT> object_lists_;

  // add object of any type
  template <class T> T* AddObject(std::vector<T*>& list, std::string type);

  // add object of any type, with defaults parameter
  template <class T> T* AddObjectDefault(std::vector<T*>& list, std::string type,
                                         sim_builder_default_t* def);

  // copy vector of elements to this model
  template <class T> void CopyList(std::vector<T*>& dest,
                                   const std::vector<T*>& sources);

  // copy plugins that are explicitly instantiated by the argument object to this model
  template <class T> void CopyExplicitPlugin(T* obj);

  // copy vector of plugins to this model
  template <class T> void CopyPlugin(const std::vector<sim_builder_plugin_t*>& sources,
                                     const std::vector<T*>& list);

  // delete from list the elements that cause an error
  template <class T> void RemoveFromList(std::vector<T*>& list, const sim_builder_model_t& other);

  // create sim_builder_base_t lists from children lists
  void CreateObjectLists();

  // populate objects ids
  void ProcessLists(bool checkrepeat = true);

  // process list of objects
  template <class T> void ProcessList_(SIM_ListKeyMap& ids, std::vector<T*>& list,
                                       sim_obj_t type, bool checkrepeat = true);

  // reset lists of kinematic tree
  void ResetTreeLists();

  // save dof offsets in joints and actuators
  void SaveDofOffsets(bool computesize = false);

  // convert pending keyframes info to actual keyframes
  void ResolveKeyframes(const sim_model_t* m);

  // expand a keyframe, filling in missing values
  void ExpandKeyframe(SIM_CKey* key, const sim_scalar_t* qpos0_, const sim_scalar_t* bpos, const sim_scalar_t* bquat);

  // compute qpos0
  void ComputeReference();

  // return true if body has valid mass and inertia
  bool CheckBodyMassInertia(sim_builder_body_t* body);

  // Mark plugin instances mentioned in the list
  template <class T>
  void MarkPluginInstance(std::unordered_map<std::string, bool>& instances,
                          const std::vector<T*>& list);

  // print the tree of a body
  void PrintTree(std::stringstream& tree, const sim_builder_body_t* body, int depth = 0);

  // generate a signature for the model
  uint64_t Signature();

  // reassign children of a body to a new parent
  template <class T>
  void ReassignChild(std::vector<T*>& dest, std::vector<T*>& list, sim_builder_body_t* parent, sim_builder_body_t* body);

  // resolve references in a list of objects
  template <class T>
  void ResolveReferences(std::vector<T*>& list, sim_builder_body_t* body = nullptr);

  // delete all plugins created by the subtree
  void DeleteSubtreePlugin(sim_builder_body_t* subtree);

  // expand all keyframes in the model
  void ExpandAllKeyframes();

  SIM_ListKeyMap ids;   // map from object names to ids
  sim_builder_error_t errInfo;   // last error info
  std::vector<SIM_KeyInfo> key_pending_;  // attached keyframes
  bool deepcopy_;     // copy objects when attaching
  bool attached_ = false;  // true if model is attached to a parent model
  std::unordered_map<const SIM_sCompiler*, sim_spec_t*> compiler2spec_;  // map from compiler to spec
  std::vector<sim_builder_base_t*> detached_;  // list of detached objects
};
#endif  // SIMCORE_SRC_USER_USER_MODEL_H_
