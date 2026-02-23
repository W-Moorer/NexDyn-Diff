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

#include "xml/xml_base.h"

#include <cfloat>
#include <cstddef>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <simcore/SIM_model.h>
#include <simcore/SIM_spec.h>
#include "xml/xml_util.h"
#include "tinyxml2.h"

namespace {

using std::string;
using tinyxml2::XMLElement;

}  // namespace


//--------------------------------- Base class, helper functions -----------------------------------

// base constructor
SIM_XBase::SIM_XBase() {
  spec = NULL;
}



// set model field
void SIM_XBase::SetModel(sim_spec_t* _model, const sim_model_t* m) {
  spec = _model;
}



// read alternative orientation specification
int SIM_XBase::ReadAlternative(XMLElement* elem, SIM_sOrientation& alt) {
  string text;
  int numspec = (int)(elem->Attribute("quat") != 0);
  if (ReadAttr(elem, "axisangle", 4, alt.axisangle, text)) {
    numspec++;
    alt.type = SIM_ORIENTATION_AXISANGLE;
  }
  if (ReadAttr(elem, "xyaxes", 6, alt.xyaxes, text)) {
    numspec++;
    alt.type = SIM_ORIENTATION_XYAXES;
  }
  if (ReadAttr(elem, "zaxis", 3, alt.zaxis, text)) {
    numspec++;
    alt.type = SIM_ORIENTATION_ZAXIS;
  }
  if (ReadAttr(elem, "euler", 3, alt.euler, text)) {
    numspec++;
    alt.type = SIM_ORIENTATION_EULER;
  }
  if (numspec > 1) {
    throw sim_xml_error_t(elem, "multiple orientation specifiers are not allowed");
  }
  return numspec;
}
