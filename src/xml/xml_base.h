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

#ifndef SIMCORE_SRC_XML_XML_BASE_H_
#define SIMCORE_SRC_XML_XML_BASE_H_

#include <cstdlib>
#include <string>

#include <simcore/SIM_model.h>
#include <simcore/SIM_spec.h>
#include "xml/xml_util.h"
#include "tinyxml2.h"


// keyword maps (defined in implementation files)
extern const int joint_sz;
extern const int bodysleep_sz;
extern const int projection_sz;
extern const int camlight_sz;
extern const int lighttype_sz;
extern const int integrator_sz;
extern const int collision_sz;
extern const int cone_sz;
extern const int jac_sz;
extern const int solver_sz;
extern const int equality_sz;
extern const int texture_sz;
extern const int colorspace_sz;
extern const int builtin_sz;
extern const int mark_sz;
extern const int dyn_sz;
extern const int gain_sz;
extern const int bias_sz;
extern const int interp_sz;
extern const int stage_sz;
extern const int datatype_sz;
extern const int camout_sz;
extern const int reduce_sz;
extern const sim_map_t angle_map[];
extern const sim_map_t enable_map[];
extern const sim_map_t bool_map[];
extern const sim_map_t fluid_map[];
extern const sim_map_t TFAuto_map[];
extern const sim_map_t joint_map[];
extern const sim_map_t bodysleep_map[];
extern const sim_map_t geom_map[];
extern const sim_map_t projection_map[];
extern const sim_map_t camlight_map[];
extern const sim_map_t lighttype_map[];
extern const sim_map_t integrator_map[];
extern const sim_map_t collision_map[];
extern const sim_map_t impedance_map[];
extern const sim_map_t reference_map[];
extern const sim_map_t cone_map[];
extern const sim_map_t jac_map[];
extern const sim_map_t solver_map[];
extern const sim_map_t equality_map[];
extern const sim_map_t texture_map[];
extern const sim_map_t colorspace_map[];
extern const sim_map_t texrole_map[];
extern const sim_map_t builtin_map[];
extern const sim_map_t mark_map[];
extern const sim_map_t dyn_map[];
extern const sim_map_t gain_map[];
extern const sim_map_t bias_map[];
extern const sim_map_t interp_map[];
extern const sim_map_t stage_map[];
extern const sim_map_t datatype_map[];
extern const sim_map_t condata_map[];
extern const sim_map_t raydata_map[];
extern const sim_map_t camout_map[];
extern const sim_map_t reduce_map[];
extern const sim_map_t meshtype_map[];
extern const sim_map_t meshinertia_map[];
extern const sim_map_t flexself_map[];
extern const sim_map_t elastic2d_map[];


//---------------------------------- Base XML class ------------------------------------------------

class SIM_XBase : public sim_xml_util_t {
 public:
  SIM_XBase();
  virtual ~SIM_XBase() = default;

  // parse: implemented in derived parser classes
  virtual void Parse(tinyxml2::XMLElement* root, const SIM_VFS* vfs = nullptr) {};

  // write: implemented in derived writer class
  virtual std::string Write(char *error, std::size_t error_sz) {
    return "";
  };

  // set the model allocated externally
  virtual void SetModel(sim_spec_t*, const sim_model_t* = nullptr);

  // read alternative orientation specification
  static int ReadAlternative(tinyxml2::XMLElement* elem, SIM_sOrientation& alt);

 protected:
  sim_spec_t* spec;                    // internally-allocated model
};

#endif  // SIMCORE_SRC_XML_XML_BASE_H_
