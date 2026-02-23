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

#ifndef SIMCORE_SRC_ENGINE_ENGINE_MACRO_H_
#define SIMCORE_SRC_ENGINE_ENGINE_MACRO_H_

#include <stdint.h>

#include "engine/engine_callback.h"  // IWYU pragma: export

//-------------------------------- utility macros --------------------------------------------------

// thread local macro
#ifdef _MSC_VER
  #define SIM_THREADLOCAL __declspec(thread)
#else
  #define SIM_THREADLOCAL _Thread_local
#endif


//-------------------------- timer macros ----------------------------------------------------------

#define TM_START sim_scalar_t _tm = (SIM_cb_time ? SIM_cb_time() : 0);
#define TM_RESTART _tm = (SIM_cb_time ? SIM_cb_time() : 0);
#define TM_END(i) {d->timer[i].duration += ((SIM_cb_time ? SIM_cb_time() : 0) - _tm); d->timer[i].number++;}
#define TM_ADD(i) {d->timer[i].duration += ((SIM_cb_time ? SIM_cb_time() : 0) - _tm);}
#define TM_START1 sim_scalar_t _tm1 = (SIM_cb_time ? SIM_cb_time() : 0);
#define TM_END1(i) {d->timer[i].duration += ((SIM_cb_time ? SIM_cb_time() : 0) - _tm1); d->timer[i].number++;}

//-------------------------- compiler builtin ------------------------------------------------------

#ifndef __has_builtin
  #define __has_builtin(x) 0
#endif

//-------------------------- pointer arithmetic ----------------------------------------------------

#define PTRDIFF(x, y) ((char*)(x) - (char*)(y))

#endif  // SIMCORE_SRC_ENGINE_ENGINE_MACRO_H_
