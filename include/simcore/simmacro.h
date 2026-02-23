// Copyright 2023 DeepMind Technologies Limited
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

#ifndef SIMCORE_SIM_MACRO_H_
#define SIMCORE_SIM_MACRO_H_

// max and min (use only for primitive types)
#define SIM_MAX(a, b) (((a) > (b)) ? (a) : (b))
#define SIM_MIN(a, b) (((a) < (b)) ? (a) : (b))

// return current value of SIM_Option enable/disable flags
#define SIM_DISABLED(x) (m->opt.disableflags & (x))
#define SIM_ENABLED(x)  (m->opt.enableflags & (x))

// is actuator disabled
#define SIM_ACTUATORDISABLED(i) (m->opt.disableactuator & (1 << m->actuator_group[i]))

// annotation for functions that accept printf-like variadic arguments
#ifndef SIM_PRINTFLIKE
  #if defined(__GNUC__)
    #define SIM_PRINTFLIKE(n, m) __attribute__((format(printf, n, m)))
  #else
    #define SIM_PRINTFLIKE(n, m)
  #endif
#endif

#endif  // SIMCORE_SIM_MACRO_H_
