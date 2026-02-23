// Copyright 2025 DeepMind Technologies Limited
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

#ifndef SIMCORE_SRC_ENGINE_ENGINE_MEMORY_H_
#define SIMCORE_SRC_ENGINE_ENGINE_MEMORY_H_

#include <simcore/SIM_data.h>
#include <simcore/SIM_export.h>
#include <simcore/SIM_xmacro.h>

#ifdef __cplusplus
#include <cstddef>
extern "C" {
#else
#include <stddef.h>
#endif

// internal hash map size factor (2 corresponds to a load factor of 0.5)
#define SIM_LOAD_MULTIPLE 2

// sim_data_t arena allocate
SIM_API void* sim_arenaAllocByte(sim_data_t* d, size_t bytes, size_t alignment);

#ifndef ADDRESS_SANITIZER

// sim_data_t mark stack frame
SIM_API void sim_markStack(sim_data_t* d);

// sim_data_t free stack frame
SIM_API void sim_freeStack(sim_data_t* d);

#else

void sim__markStack(sim_data_t* d) __attribute__((noinline));
void sim__freeStack(sim_data_t* d) __attribute__((noinline));

#endif  // ADDRESS_SANITIZER

// returns the number of bytes available on the stack
SIM_API size_t sim_stackBytesAvailable(sim_data_t* d);

// allocate bytes on the stack
SIM_API void* sim_stackAllocByte(sim_data_t* d, size_t bytes, size_t alignment);

// allocate bytes on the stack, with added caller information
SIM_API void* sim_stackAllocInfo(sim_data_t* d, size_t bytes, size_t alignment,
                              const char* caller, int line);

// macro to allocate a stack array of given type, adds caller information
#define SIM_STACK_ALLOC(d, num, type) \
(type*) sim_stackAllocInfo(d, (num) * sizeof(type), _Alignof(type), __func__, __LINE__)

// sim_data_t stack allocate for array of SIM_tNums
SIM_API sim_scalar_t* sim_stackAllocNum(sim_data_t* d, size_t size);

// sim_data_t stack allocate for array of ints
SIM_API int* sim_stackAllocInt(sim_data_t* d, size_t size);

// clear arena pointers in sim_data_t
static inline void sim_clearEfc(sim_data_t* d) {
#define X(type, name, nr, nc) d->name = NULL;
  SIMDATA_ARENA_POINTERS
#undef X
  d->nefc = 0;
  d->nisland = 0;
  d->contact = (sim_contact_t*) d->arena;

  // if any contacts are allocated, clear their efc_address
  for (int i=0; i < d->ncon; i++) {
    d->contact[i].efc_address = -1;
  }
}

#ifdef __cplusplus
}
#endif

#endif  // SIMCORE_SRC_ENGINE_ENGINE_MEMORY_H_
