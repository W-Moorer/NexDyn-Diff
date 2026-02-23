// Copyright 2026 DeepMind Technologies Limited
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


#ifndef SIMCORE_SRC_XML_XML_GLOBAL_H_
#define SIMCORE_SRC_XML_XML_GLOBAL_H_

#include <string>

#include <simcore/core_api.h>

SIM_API void SetGlobalXmlSpec(sim_spec_t* spec = nullptr);

std::string GetGlobalXmlSpec(const sim_model_t* m, char* error, int error_sz);

#endif  // SIMCORE_SRC_XML_XML_GLOBAL_H_
