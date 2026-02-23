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

#ifndef SIMCORE_INCLUDE_SIM_TNUM_H_
#define SIMCORE_INCLUDE_SIM_TNUM_H_

#include <stdint.h>


//---------------------------------- floating-point definition -------------------------------------

// floating point data type and minval
#ifndef SIM_USESINGLE
  typedef double sim_scalar_t;
  #define SIM_MINVAL    1E-15       // minimum value in any denominator
#else
  typedef float sim_scalar_t;
  #define SIM_MINVAL    1E-15f
#endif



//---------------------------------- byte definition -----------------------------------------------

typedef unsigned char sim_byte_t;    // used for true/false



//---------------------------------- size definition -----------------------------------------------

typedef int64_t sim_size_t;          // used for buffer sizes



#endif  // SIMCORE_INCLUDE_SIM_TNUM_H_
