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

#ifndef SIMCORE_SRC_XML_XML_NUMERIC_FORMAT_H_
#define SIMCORE_SRC_XML_XML_NUMERIC_FORMAT_H_

#include <simcore/SIM_export.h>

namespace simcore {

extern "C" {
  SIM_API int _simPRIVATE__get_xml_precision();
  SIM_API void _simPRIVATE__set_xml_precision(const int precision);
}

// Full precision printing of floating point numbers in saved XMLs, useful for testing
class FullFloatPrecision {
 public:
  FullFloatPrecision() { _simPRIVATE__set_xml_precision(17);}
  ~FullFloatPrecision() { _simPRIVATE__set_xml_precision(6);}
};

}  // namespace simcore

#endif  // SIMCORE_SRC_XML_NUMERIC_FORMAT_H_

