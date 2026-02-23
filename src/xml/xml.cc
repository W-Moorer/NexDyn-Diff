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

#include "xml/xml.h"

#include <locale.h>
#include <cstring>

#if defined(__APPLE__) || defined(__FreeBSD__)
#include <xlocale.h>
#endif

#include <array>
#include <cstdio>
#include <string>
#include <string_view>
#include <unordered_set>

#include <simcore/core_api.h>
#include <simcore/SIM_model.h>
#include <simcore/SIM_plugin.h>
#include "cc/array_safety.h"
#include "engine/engine_crossplatform.h"
#include <simcore/SIM_spec.h>
#include "user/user_resource.h"
#include "user/user_util.h"
#include "user/user_vfs.h"
#include "xml/xml_native_reader.h"
#include "xml/xml_native_writer.h"
#include "xml/xml_urdf.h"
#include "xml/xml_util.h"
#include "tinyxml2.h"

namespace {

using tinyxml2::XMLDocument;
using tinyxml2::XMLElement;
using tinyxml2::XMLNode;
using simcore::user::FilePath;

namespace sim_math = ::simcore::util;


// We are using "locale-sensitive" sprintf to read and write XML.
// When SimCore is being used as a plug-in for an application that respects the system locale
// (e.g. Unity), the user's locale setting can affect the formatting of numbers into strings.
// Specifically, a number of European locales (e.g. de_DE) uses commas to as decimal separators.
// In order to ensure that XMLs are locale-inpendent, we temporarily switch to the "C" locale
// when handling. Since the standard C `setlocale` is not thread-safe, we instead use
// platform-specific extensions to override the locale only in the calling thread.
// See also upstream issue tracker discussion #131.
#ifdef _WIN32
class LocaleOverride {
 public:
  LocaleOverride()
      : old_per_thread_locale_type_(_configthreadlocale(0)),
        old_locale_(setlocale(LC_ALL, nullptr)) {
    _configthreadlocale(_ENABLE_PER_THREAD_LOCALE);
    setlocale(LC_ALL, "C");
  }

  ~LocaleOverride() {
    setlocale(LC_ALL, old_locale_.c_str());
    _configthreadlocale(old_per_thread_locale_type_);
  }

 private:
  int old_per_thread_locale_type_;
  std::string old_locale_;
};
#else
class LocaleOverride {
 public:
  static locale_t PosixLocale() {
    static locale_t posix_locale = newlocale(LC_ALL_MASK, "C", 0);
    return posix_locale;
  }

  LocaleOverride() : old_locale_(uselocale(PosixLocale())) {}

  ~LocaleOverride() {
    uselocale(old_locale_);
  }

 private:
  locale_t old_locale_;
};
#endif

// find include elements recursively, replace them with subtree from xml file
void IncludeXML(sim_xml_reader_t& reader, XMLElement* elem,
                const FilePath& dir, const SIM_VFS* vfs,
                std::unordered_set<std::string>& included) {
  // capture directory defaults on first pass of XML tree
  if (!strcasecmp(elem->Value(), "compiler")) {
    auto assetdir_attr = sim_xml_util_t::ReadAttrStr(elem, "assetdir");
    if (assetdir_attr.has_value()) {
      reader.SetAssetDir(assetdir_attr.value());
    }

    auto texturedir_attr = sim_xml_util_t::ReadAttrStr(elem, "texturedir");
    if (texturedir_attr.has_value()) {
      reader.SetTextureDir(texturedir_attr.value());
    }

    auto meshdir_attr = sim_xml_util_t::ReadAttrStr(elem, "meshdir");
    if (meshdir_attr.has_value()) {
      reader.SetMeshDir(meshdir_attr.value());
    }
  }

  //  not an include, recursively go through all children
  if (strcasecmp(elem->Value(), "include")) {
    XMLElement* child = elem->FirstChildElement();
    for (; child; child = child->NextSiblingElement()) {
      IncludeXML(reader, child, dir, vfs, included);
    }
    return;
  }

  // make sure include has no children
  if (!elem->NoChildren()) {
    throw sim_xml_error_t(elem, "Include element cannot have children");
  }

  // get filename
  auto file_attr = sim_xml_util_t::ReadAttrFile(elem, "file", vfs,
                                         reader.ModelFileDir(), true);
  if (!file_attr.has_value()) {
    throw sim_xml_error_t(elem, "Include element missing file attribute");
  }
  FilePath filename = file_attr.value();


  // block repeated include files
  if (included.find(filename.Str()) != included.end()) {
    throw sim_xml_error_t(elem, "File '%s' already included", filename.c_str());
  }

  // TODO: b/325905702 - We have a messy wrapper here to remain backwards
  // compatible, which will be removed in the near future.
  // legacy behavior: try to load in top level directory
  std::array<char, 1024> error;
  SIM_Resource *resource = sim_math_openResource(reader.ModelFileDir().c_str(),
                                          filename.c_str(), vfs,
                                          error.data(), error.size());
  if (resource == nullptr) {
    // new behavior: try to load in relative directory
    if (!filename.IsAbs()) {
      FilePath fullname = dir + filename;
      resource = sim_math_openResource(reader.ModelFileDir().c_str(),
                                  fullname.c_str(), vfs, error.data(), error.size());
    }
  }

  if (resource == nullptr) {
    throw sim_xml_error_t(elem, "%s", error.data());
  }

  filename = dir + filename;

  const char* include_dir = nullptr;
  int ninclude_dir = 0;
  sim_math_getResourceDir(resource, &include_dir, &ninclude_dir);
  FilePath next_dir = FilePath(std::string(include_dir, ninclude_dir));
  elem->SetAttribute("dir", next_dir.c_str());

  const char* xmlstring = nullptr;
  int buffer_size = sim_math_readResource(resource, (const void**) &xmlstring);
  if (buffer_size < 0) {
    sim_math_closeResource(resource);
    throw sim_xml_error_t(elem, "Error reading file '%s'", filename.c_str());
  } else if (!buffer_size) {
    sim_math_closeResource(resource);
    throw sim_xml_error_t(elem, "Empty file '%s'", filename.c_str());
  }

  // load XML file or parse string
  XMLDocument doc;
  doc.Parse(xmlstring, buffer_size);

  // close resource
  sim_math_closeResource(resource);

  // check error
  if (doc.Error()) {
    char err[1000];
    sim_math::sprintf_arr(err, "XML parse error %d:\n%s\n", doc.ErrorID(), doc.ErrorStr());
    throw sim_xml_error_t(elem, "Include error: '%s'", err);
  }

  // remember that file was included
  included.insert(filename.Str());

  // get and check root element
  XMLElement* docroot = doc.RootElement();
  if (!docroot) {
    throw sim_xml_error_t(elem, "Root element missing in file '%s'", filename.c_str());
  }

  // get and check first child
  XMLElement* eleminc = docroot->FirstChildElement();
  if (!eleminc) {
    throw sim_xml_error_t(elem, "Empty include file '%s'", filename.c_str());
  }

  // get <include> element
  XMLElement* include = elem->ToElement();
  XMLDocument* include_doc = include->GetDocument();

  // clone first child of included document
  XMLNode* first = include->InsertFirstChild(eleminc->DeepClone(include_doc));

  // point to first
  XMLElement* child = first->ToElement();

  // insert remaining elements from included document as siblings
  eleminc = eleminc->NextSiblingElement();
  while (eleminc) {
    child = include->InsertAfterChild(child, eleminc->DeepClone(include_doc))->ToElement();
    eleminc = eleminc->NextSiblingElement();
  }

  // recursively run include
  child = include->FirstChildElement();
  for (; child; child = child->NextSiblingElement()) {
    IncludeXML(reader, child, next_dir, vfs, included);
  }
}

// Main parser function
sim_spec_t* SpecFromXML(std::string_view xml, std::string_view dir,
                    std::string_view filename, const SIM_VFS* vfs, char* error,
                    int nerror) {
  LocaleOverride locale_override;

  // clear
  sim_spec_t* spec = nullptr;
  if (error) {
    error[0] = '\0';
  }

  // load XML file or parse string
  XMLDocument doc;
  doc.Parse(xml.data(), xml.size());

  // error checking
  if (doc.Error()) {
    if (error) {
      snprintf(error, nerror, "XML parse error %d:\n%s\n",
               doc.ErrorID(), doc.ErrorStr());
    }
    return nullptr;
  }

  // get top-level element
  XMLElement* root = doc.RootElement();
  if (!root) {
    SIM_CopyError(error, "XML root element not found", nerror);
    return nullptr;
  }

  // create model, set filedir
  spec = sim_makeSpec();
  sim_spec_set_string(spec->modelfiledir, std::string(dir).c_str());


  // parse with exceptions
  try {
    if (!strcasecmp(root->Value(), "simcore")) {
      // find include elements, replace them with subtree from xml file
      std::unordered_set<std::string> included = {std::string(filename)};
      sim_xml_reader_t parser;
      parser.SetModelFileDir(sim_spec_getString(spec->modelfiledir));
      IncludeXML(parser, root, FilePath(), vfs, included);

      // parse SimCore model
      parser.SetModel(spec);
      parser.Parse(root, vfs);
    }

    else if (!strcasecmp(root->Value(), "robot")) {
      // parse URDF model
      SIM_XURDF parser;

      // set reasonable default for parsing a URDF
      // this is separate from the Parser to allow multiple URDFs to be loaded.
      spec->strippath = true;
      spec->compiler.fusestatic = true;
      spec->compiler.discardvisual = true;

      parser.SetModel(spec);
      parser.Parse(root);
    }

    else {
      throw sim_xml_error_t(0, "Unrecognized XML model type: '%s'", root->Value());
    }
  }

  // catch known errors
  catch (sim_xml_error_t err) {
    SIM_CopyError(error, err.message, nerror);
    sim_deleteSpec(spec);
    return nullptr;
  }

  return spec;
}
}  // namespace

sim_spec_t* ParseXML(const char* filename, const SIM_VFS* vfs, char* error,
                 int nerror) {
  // check arguments
  if (!filename) {
    if (error) {
      std::snprintf(error, nerror, "ParseXML: filename argument required\n");
    }
    return nullptr;
  }

  // get data source
  const char* xml = nullptr;
  std::array<char, 1024> rerror;
  SIM_Resource* resource = sim_math_openResource("", filename, vfs,
                                          rerror.data(), rerror.size());
  if (resource == nullptr) {
    std::snprintf(error, nerror, "ParseXML: %s", rerror.data());
    return nullptr;
  }

  int buffer_size = sim_math_readResource(resource, (const void**) &xml);
  if (buffer_size < 0) {
    if (error) {
      std::snprintf(error, nerror,
                    "ParseXML: error reading file '%s'", filename);
    }
    sim_math_closeResource(resource);
    return nullptr;
  } else if (!buffer_size) {
    if (error) {
      std::snprintf(error, nerror, "ParseXML: empty file '%s'", filename);
    }
    sim_math_closeResource(resource);
    return nullptr;
  }

  const char* dir;
  int ndir = 0;
  sim_math_getResourceDir(resource, &dir, &ndir);
  std::string_view directory(dir, ndir);

  sim_spec_t* spec = SpecFromXML({xml, xml + buffer_size}, directory,
                             filename, vfs, error, nerror);

  sim_math_closeResource(resource);
  return spec;
}

sim_spec_t* ParseSpecFromString(std::string_view xml, const SIM_VFS* vfs, char* error,
                            int nerror) {
  return SpecFromXML(xml, "", "", vfs, error, nerror);
}

// Main writer function - calls SIM_XWrite
std::string WriteXML(const sim_model_t* m, sim_spec_t* spec, char* error, int nerror) {
  LocaleOverride locale_override;

  // check for empty model
  if (!spec) {
    SIM_CopyError(error, "Cannot write empty model", nerror);
    return "";
  }

  sim_xml_writer_t writer;
  writer.SetModel(spec, m);

  try {
    return writer.Write(error, nerror);
  } catch (sim_xml_error_t err) {
    SIM_CopyError(error, err.message, nerror);
    return "";
  }
}
