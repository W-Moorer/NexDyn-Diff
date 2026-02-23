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

#ifndef SIMCORE_SRC_XML_XML_API_H_
#define SIMCORE_SRC_XML_XML_API_H_

#include <simcore/SIM_export.h>
#include <simcore/SIM_model.h>
#include <simcore/SIM_spec.h>

#ifdef __cplusplus
extern "C" {
#endif



// parse XML file in SIMCF or URDF format, compile it, return low-level model
//  if vfs is not NULL, look up files in vfs before reading from disk
//  error can be NULL; otherwise assumed to have size error_sz
SIM_API sim_model_t* sim_loadXML(const char* filename, const SIM_VFS* vfs, char* error, int error_sz);

// update XML data structures with info from low-level model, save as SIMCF
SIM_API int sim_saveLastXML(const char* filename, const sim_model_t* m, char* error, int error_sz);

// free last XML model if loaded; called internally at each load
SIM_API void sim_freeLastXML(void);

// print internal XML schema as plain text or HTML, with style-padding or &nbsp;
SIM_API int sim_printSchema(const char* filename, char* buffer, int buffer_sz,
                         int flg_html, int flg_pad);

// load model from binary SIMB file
// if vfs is not NULL, look up file in vfs before reading from disk
SIM_API sim_model_t* sim_loadModel(const char* filename, const SIM_VFS* vfs);

// parse spec from file or XML string.
SIM_API sim_spec_t* sim_parseXML(const char* filename, const SIM_VFS* vfs, char* error, int error_sz);
SIM_API sim_spec_t* sim_parseXMLString(const char* xml, const SIM_VFS* vfs, char* error, int error_sz);

// Save spec to XML file and/or string, return 0 on success, -1 otherwise.
SIM_API int sim_saveXML(const sim_spec_t* s, const char* filename, char* error, int error_sz);
SIM_API int sim_saveXMLString(const sim_spec_t* s, char* xml, int xml_sz, char* error, int error_sz);

#ifdef __cplusplus
}
#endif

#endif  // SIMCORE_SRC_XML_XML_API_H_
