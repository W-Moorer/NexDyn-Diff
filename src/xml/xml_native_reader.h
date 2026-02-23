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

#ifndef SIMCORE_SRC_XML_XML_NATIVE_READER_H_
#define SIMCORE_SRC_XML_XML_NATIVE_READER_H_

#include <sstream>
#include <string>
#include <vector>

#include <simcore/core_api.h>
#include <simcore/SIM_spec.h>
#include "user/user_util.h"
#include "xml/xml_base.h"
#include "xml/xml_util.h"
#include "tinyxml2.h"

class sim_xml_reader_t : public SIM_XBase {
 public:
  sim_xml_reader_t();                                                         // constructor
  virtual ~sim_xml_reader_t() = default;                                      // destructor

  void Parse(tinyxml2::XMLElement* root, const SIM_VFS* vfs = nullptr);  // parse XML document
  void PrintSchema(std::stringstream& str, bool html, bool pad);       // print text or HTML schema

  void SetModelFileDir(const std::string& modelfiledir);
  const simcore::user::FilePath& ModelFileDir() const { return modelfiledir_; }

  // setters for directory defaults
  void SetAssetDir(const std::string& assetdir);
  void SetMeshDir(const std::string& meshdir);
  void SetTextureDir(const std::string& texturedir);

  // XML sections embedded in all formats
  static void Compiler(tinyxml2::XMLElement* section, sim_spec_t* s);    // compiler section
  static void Option(tinyxml2::XMLElement* section, SIM_Option* opt);  // option section
  static void Size(tinyxml2::XMLElement* section, sim_spec_t* s);        // size section

 private:
  // XML section specific to SIMCF
  void Default(tinyxml2::XMLElement* section, const sim_spec_default_t* def,
               const SIM_VFS* vfs);                                      // default section
  void Extension(tinyxml2::XMLElement* section);                       // extension section
  void Custom(tinyxml2::XMLElement* section);                          // custom section
  void Visual(tinyxml2::XMLElement* section);                          // visual section
  void Statistic(tinyxml2::XMLElement* section);                       // statistic section
  void Asset(tinyxml2::XMLElement* section, const SIM_VFS* vfs);         // asset section
  void Body(tinyxml2::XMLElement* section, sim_spec_body_t* pbody,
            SIM_sFrame* pframe, const SIM_VFS* vfs);                       // body/world section
  void Contact(tinyxml2::XMLElement* section);                         // contact section
  void Deformable(tinyxml2::XMLElement* section, const SIM_VFS* vfs);    // deformable section
  void Equality(tinyxml2::XMLElement* section);                        // equality section
  void Tendon(tinyxml2::XMLElement* section);                          // tendon section
  void Actuator(tinyxml2::XMLElement* section);                        // actuator section
  void Sensor(tinyxml2::XMLElement* section);                          // sensor section
  void Keyframe(tinyxml2::XMLElement* section);                        // keyframe section

  // single element parsers, used in defaults and main body
  void OneFlex(tinyxml2::XMLElement* elem, SIM_sFlex* pflex);
  void OneMesh(tinyxml2::XMLElement* elem, SIM_sMesh* pmesh, const SIM_VFS* vfs);
  void OneSkin(tinyxml2::XMLElement* elem, SIM_sSkin* pskin, const SIM_VFS* vfs);
  void OneMaterial(tinyxml2::XMLElement* elem, SIM_sMaterial* pmaterial);
  void OneJoint(tinyxml2::XMLElement* elem, SIM_sJoint* pjoint);
  void OneGeom(tinyxml2::XMLElement* elem, sim_spec_geom_t* pgeom);
  void OneSite(tinyxml2::XMLElement* elem, SIM_sSite* site);
  void OneCamera(tinyxml2::XMLElement* elem, SIM_sCamera* pcamera);
  void OneLight(tinyxml2::XMLElement* elem, SIM_sLight* plight);
  void OnePair(tinyxml2::XMLElement* elem, SIM_sPair* ppair);
  void OneEquality(tinyxml2::XMLElement* elem, SIM_sEquality* pequality);
  void OneTendon(tinyxml2::XMLElement* elem, SIM_sTendon* ptendon);
  void OneActuator(tinyxml2::XMLElement* elem, sim_spec_actuator_t* pactuator);
  void OneComposite(tinyxml2::XMLElement* elem, sim_spec_body_t* pbody, SIM_sFrame* pframe,
                    const sim_spec_default_t* def);
  void OneFlexcomp(tinyxml2::XMLElement* elem, sim_spec_body_t* pbody, const SIM_VFS* vfs);
  void OnePlugin(tinyxml2::XMLElement* elem, SIM_sPlugin* plugin);

  SIM_XSchema schema;                                     // schema used for validation
  const sim_spec_default_t* GetClass(tinyxml2::XMLElement* section);  // get default class name

  bool readingdefaults;  // true while reading defaults

  // accessors for directory defaults
  simcore::user::FilePath AssetDir() const;
  simcore::user::FilePath MeshDir() const;
  simcore::user::FilePath TextureDir() const;

  simcore::user::FilePath modelfiledir_;
  simcore::user::FilePath assetdir_;
  simcore::user::FilePath meshdir_;
  simcore::user::FilePath texturedir_;
};

// SIMCF schema
#define nSIMCF 245
extern std::vector<const char*> SIMCF[nSIMCF];

#endif  // SIMCORE_SRC_XML_XML_NATIVE_READER_H_
