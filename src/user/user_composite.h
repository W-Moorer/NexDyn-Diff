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

#ifndef SIMCORE_SRC_USER_USER_COMPOSITE_H_
#define SIMCORE_SRC_USER_USER_COMPOSITE_H_

#include <string>
#include <unordered_map>
#include <vector>

#include <simcore/SIM_model.h>
#include <simcore/SIM_spec.h>
#include "user/user_model.h"
#include "user/user_objects.h"

typedef enum _mjtCompType {
  SIM_COMPTYPE_PARTICLE = 0,
  SIM_COMPTYPE_GRID,
  SIM_COMPTYPE_CABLE,
  SIM_COMPTYPE_ROPE,
  SIM_COMPTYPE_LOOP,
  SIM_COMPTYPE_CLOTH,

  SIM_NCOMPTYPES
} SIM_tCompType;


typedef enum _mjtCompKind {
  SIM_COMPKIND_JOINT = 0,

  SIM_NCOMPKINDS
} SIM_tCompKind;


typedef enum _mjtCompShape {
  SIM_COMPSHAPE_INVALID = -1,
  SIM_COMPSHAPE_LINE,
  SIM_COMPSHAPE_COS,
  SIM_COMPSHAPE_SIN,
  SIM_COMPSHAPE_ZERO,

  SIM_NCOMPSHAPES
} SIM_tCompShape;


class SIM_CComposite {
 public:
  SIM_CComposite(void);

  void SetDefault(void);
  bool AddDefaultJoint(char* error = NULL, int error_sz = 0);

  bool Make(sim_spec_t* spec, sim_spec_body_t* body, char* error, int error_sz);
  bool MakeCable(sim_builder_model_t* model, sim_spec_body_t* body, char* error, int error_sz);

  void MakeSkin2(sim_builder_model_t* model, sim_scalar_t inflate);
  void MakeSkin2Subgrid(sim_builder_model_t* model, sim_scalar_t inflate);
  void MakeCableBones(sim_builder_model_t* model, SIM_sSkin* skin);
  void MakeCableBonesSubgrid(sim_builder_model_t* model, SIM_sSkin* skin);

  // common properties
  std::string prefix;             // name prefix
  SIM_tCompType type;               // composite type
  int count[3];                   // geom count in each dimension
  double offset[3];               // position offset
  double quat[4];                 // quaternion offset

  // currently used only for cable
  std::string initial;            // root boundary type
  std::vector<float> uservert;    // user-specified vertex positions
  double size[3];                 // rope size (meaning depends on the shape)
  SIM_tCompShape curve[3];          // geometric shape
  SIM_sFrame* frame;                // frame where the composite is defined

  // body names used in the skin
  std::vector<std::string> username;

  // plugin support
  std::string plugin_name;
  std::string plugin_instance_name;
  SIM_sPlugin plugin;

  // skin
  bool skin;                      // generate skin
  bool skintexcoord;              // generate texture coordinates
  std::string skinmaterial;       // skin material
  float skinrgba[4];              // skin rgba
  float skininflate;              // inflate skin
  int skinsubgrid;                // number of skin subgrid points; 0: none (2D only)
  int skingroup;                  // skin group of the composite object

  // element options
  bool add[SIM_NCOMPKINDS];                                          // add element
  sim_builder_default_t def[SIM_NCOMPKINDS];                                        // default geom, site, tendon
  std::unordered_map<SIM_tCompKind, std::vector<sim_builder_default_t> > defjoint;  // default joints

  // computed internally
  int dim;                        // dimensionality

 private:
  sim_spec_body_t* AddCableBody(sim_builder_model_t* model, sim_spec_body_t* body, int ix, double normal[3], double prev_quat[4]);

  // temporary skin vectors
  void CopyIntoSkin(SIM_sSkin* skin);
  std::vector<int> face;
  std::vector<float> vert;
  std::vector<float> bindpos;
  std::vector<float> bindquat;
  std::vector<float> texcoord;
  std::vector<std::vector<int>> vertid;
  std::vector<std::vector<float>> vertweight;
};

#endif  // SIMCORE_SRC_USER_USER_COMPOSITE_H_
