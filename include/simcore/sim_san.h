// Copyright 2024 DeepMind Technologies Limited
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

#ifndef SIMCORE_INCLUDE_SIM_SAN_H_
#define SIMCORE_INCLUDE_SIM_SAN_H_

// Include asan interface header, or provide stubs for poison/unpoison macros when not using asan.
#ifdef ADDRESS_SANITIZER
  #include <sanitizer/asan_interface.h>
#elif defined(_MSC_VER)
  #define ASAN_POISON_MEMORY_REGION(addr, size)
  #define ASAN_UNPOISON_MEMORY_REGION(addr, size)
#else
  #define ASAN_POISON_MEMORY_REGION(addr, size) ((void)(addr), (void)(size))
  #define ASAN_UNPOISON_MEMORY_REGION(addr, size) ((void)(addr), (void)(size))
#endif

// When built and run under address sanitizer (asan), sim_markStack and sim_freeStack are instrumented
// to detect leakage of sim_data_t stack frames. When the compiler inlines several callees that call
// into mark/free into the same function, this instrumentation requires that the compiler retains
// separate mark/free calls for each original callee. The memory-clobbered asm blocks act as a
// barrier to prevent mark/free calls from being combined under optimization.
#ifdef ADDRESS_SANITIZER
#ifdef __cplusplus
extern "C" {
#endif

void sim__markStack(sim_data_t*) __attribute__((noinline));
static inline void sim_markStack(sim_data_t* d) __attribute__((always_inline)) {
  asm volatile("" ::: "memory");
  sim__markStack(d);
  asm volatile("" ::: "memory");
}

void sim__freeStack(sim_data_t*) __attribute__((noinline));
static inline void sim_freeStack(sim_data_t* d) __attribute__((always_inline)) {
  asm volatile("" ::: "memory");
  sim__freeStack(d);
  asm volatile("" ::: "memory");
}

#ifdef __cplusplus
}
#endif  // __cplusplus
#endif  // ADDRESS_SANITIZER

#endif  // SIMCORE_INCLUDE_SIM_SAN_H_
