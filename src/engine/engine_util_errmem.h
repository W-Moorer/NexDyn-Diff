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

#ifndef SIMCORE_SRC_ENGINE_ENGINE_UTIL_ERRMEM_H_
#define SIMCORE_SRC_ENGINE_ENGINE_UTIL_ERRMEM_H_

#include <stdarg.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include <simcore/SIM_export.h>
#include <simcore/SIM_macro.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef SIM_PRINTFLIKE
  #if defined(__GNUC__)
    #define SIM_PRINTFLIKE(n, m) __attribute__((format(printf, n, m)))
  #else
    #define SIM_PRINTFLIKE(n, m)
  #endif  // __GNUC__
#endif  // SIM_PRINTFLIKE


//------------------------------ user handlers -----------------------------------------------------

SIM_API extern void (*sim_math_user_error)(const char*);
SIM_API extern void (*sim_math_user_warning)(const char*);
SIM_API extern void* (*sim_math_user_malloc)(size_t);
SIM_API extern void (*sim_math_user_free)(void*);

// clear user handlers; restore default processing
SIM_API void sim_math_clearHandlers(void);

// gets/sets thread-local error/warning handlers for internal use
SIM_API void (*_mjPRIVATE__get_tls_error_fn(void))(const char*);
SIM_API void _mjPRIVATE__set_tls_error_fn(void (*h)(const char*));
SIM_API void (*_mjPRIVATE__get_tls_warning_fn(void))(const char*);
SIM_API void _mjPRIVATE__set_tls_warning_fn(void (*h)(const char*));

//------------------------------ errors and warnings -----------------------------------------------

// errors
SIM_API void sim_math_error_raw(const char* msg);
SIM_API void sim_error(const char* msg, ...) SIM_PRINTFLIKE(1, 2);
SIM_API void sim_math_error_v(const char* msg, va_list args);
SIM_API void sim_math_error_i(const char* msg, int i);
SIM_API void sim_math_error_s(const char* msg, const char* text);

// warnings
SIM_API void sim_warning(const char* msg, ...) SIM_PRINTFLIKE(1, 2);
SIM_API void sim_math_warning_i(const char* msg, int i);
SIM_API void sim_math_warning_s(const char* msg, const char* text);

// write [datetime, type: message] to SIMCORE_LOG.TXT
SIM_API void sim_math_writeLog(const char* type, const char* msg);

//------------------------------ internal error macros --------------------------------------------

// internal macro to prepend the calling function name to the error message
#pragma warning(disable : 4996)  // needed to use strncpy with Visual Studio
#define SIM_ERROR(...)                                                          \
{                                                                             \
  char _errbuf[1024];                                                         \
  size_t _funclen = strlen(__func__);                                         \
  strncpy(_errbuf, __func__, sizeof(_errbuf));                                \
  snprintf(_errbuf + _funclen, sizeof(_errbuf) - _funclen, ": " __VA_ARGS__); \
  sim_math_error_raw(_errbuf);                                                     \
}

//------------------------------ malloc and free ---------------------------------------------------

// allocate memory; byte-align on 8; pad size to multiple of 8
SIM_API void* sim_malloc(size_t size);

// free memory with free() by default
SIM_API void sim_free(void* ptr);

#ifdef __cplusplus
}
#endif
#endif  // SIMCORE_SRC_ENGINE_ENGINE_UTIL_ERRMEM_H_
