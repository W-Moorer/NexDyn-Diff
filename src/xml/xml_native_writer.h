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

#ifndef SIMCORE_SRC_XML_XML_NATIVE_WRITER_H_
#define SIMCORE_SRC_XML_XML_NATIVE_WRITER_H_

#include <cstdlib>
#include <string>
#include <string_view>

#include <simcore/SIM_model.h>
#include <simcore/SIM_spec.h>
#include "user/user_objects.h"
#include "xml/xml_base.h"
#include "tinyxml2.h"

class sim_xml_writer_t : public SIM_XBase {
 public:
  sim_xml_writer_t();                                        // constructor
  virtual ~sim_xml_writer_t() = default;                     // destructor
  void SetModel(sim_spec_t* _spec, const sim_model_t* m = nullptr);

  // write XML document to string
  std::string Write(char *error, std::size_t error_sz);

 private:
  // insert end child with given name, return child
  tinyxml2::XMLElement* InsertEnd(tinyxml2::XMLElement* parent, const char* name);

  // compiled model
  sim_builder_model_t* model = 0;

  // XML section writers
  void Compiler(tinyxml2::XMLElement* root);                              // compiler section
  void Option(tinyxml2::XMLElement* root);                                // option section
  void Size(tinyxml2::XMLElement* root);                                  // size section
  void Statistic(tinyxml2::XMLElement* root);                             // statistic section
  void Default(tinyxml2::XMLElement* root, sim_builder_default_t* def);                  // default section
  void Extension(tinyxml2::XMLElement* root);                             // extension section
  void Custom(tinyxml2::XMLElement* root);                                // custom section
  void Asset(tinyxml2::XMLElement* root);                                 // asset section
  void Contact(tinyxml2::XMLElement* root);                               // contact section
  void Deformable(tinyxml2::XMLElement* root);                            // deformable section
  void Equality(tinyxml2::XMLElement* root);                              // equality section
  void Tendon(tinyxml2::XMLElement* root);                                // tendon section
  void Actuator(tinyxml2::XMLElement* root);                              // actuator section
  void Sensor(tinyxml2::XMLElement* root);                                // sensor section
  void Keyframe(tinyxml2::XMLElement* root);                              // keyframe section

  // body/world section
  void Body(tinyxml2::XMLElement* elem, sim_builder_body_t* body, sim_builder_frame_t* frame, std::string_view childclass = "");

  // single element writers, used in defaults and main body
  void OneFlex(tinyxml2::XMLElement* elem, const SIM_CFlex* pflex);
  void OneMesh(tinyxml2::XMLElement* elem, const sim_builder_mesh_t* pmesh,         sim_builder_default_t* def);
  void OneSkin(tinyxml2::XMLElement* elem, const SIM_CSkin* pskin);
  void OneMaterial(tinyxml2::XMLElement* elem, const SIM_CMaterial* pmaterial, sim_builder_default_t* def);
  void OneJoint(tinyxml2::XMLElement* elem, const sim_builder_joint_t* pjoint, sim_builder_default_t* def,
                std::string_view classname = "");
  void OneGeom(tinyxml2::XMLElement* elem, const sim_builder_geom_t* pgeom, sim_builder_default_t* def,
               std::string_view classname = "");
  void OneSite(tinyxml2::XMLElement* elem, const sim_builder_site_t* psite, sim_builder_default_t* def,
               std::string_view classname = "");
  void OneCamera(tinyxml2::XMLElement* elem, const SIM_CCamera* pcamera,
                 sim_builder_default_t* def, std::string_view classname = "");
  void OneLight(tinyxml2::XMLElement* elem, const SIM_CLight* plight, sim_builder_default_t* def,
                std::string_view classname = "");
  void OnePair(tinyxml2::XMLElement* elem, const SIM_CPair* ppair,         sim_builder_default_t* def);
  void OneEquality(tinyxml2::XMLElement* elem, const SIM_CEquality* pequality, sim_builder_default_t* def);
  void OneTendon(tinyxml2::XMLElement* elem, const sim_builder_tendon_t* ptendon,     sim_builder_default_t* def);
  void OneActuator(tinyxml2::XMLElement* elem, const SIM_CActuator* pactuator, sim_builder_default_t* def);
  void OnePlugin(tinyxml2::XMLElement* elem, const SIM_sPlugin* plugin);
  tinyxml2::XMLElement* OneFrame(tinyxml2::XMLElement* elem, sim_builder_frame_t* frame);

  bool writingdefaults;                       // true during defaults write
};

#endif  // SIMCORE_SRC_XML_XML_NATIVE_WRITER_H_
