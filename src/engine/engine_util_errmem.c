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

#include "engine/engine_util_errmem.h"

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <time.h>

#if defined (__unix__) || (defined (__APPLE__) && defined (__MACH__))
#include <unistd.h>
#endif

#include "engine/engine_array_safety.h"
#include "engine/engine_macro.h"

//------------------------- cross-platform aligned malloc/free -------------------------------------

static inline void* sim_math_alignedMalloc(size_t size, size_t align) {
#ifdef _WIN32
  return _aligned_malloc(size, align);
#elif defined(__STDC_VERSION__) && __STDC_VERSION__ >= 201112L
  return aligned_alloc(align, size);
#endif
}

static inline void sim_math_alignedFree(void* ptr) {
#ifdef _WIN32
  _aligned_free(ptr);
#else
  free(ptr);
#endif
}


//------------------------- default user handlers --------------------------------------------------

// define and clear handlers
void (*sim_math_user_error) (const char*) = 0;
void (*sim_math_user_warning) (const char*) = 0;
void* (*sim_math_user_malloc) (size_t) = 0;
void (*sim_math_user_free) (void*) = 0;


// restore default processing
void sim_math_clearHandlers(void) {
  sim_math_user_error = 0;
  sim_math_user_warning = 0;
  sim_math_user_malloc = 0;
  sim_math_user_free = 0;
}

//------------------------- internal-only handlers -------------------------------------------------

typedef void (*callback_fn)(const char*);

static SIM_THREADLOCAL callback_fn _mjPRIVATE_tls_error_fn = NULL;
static SIM_THREADLOCAL callback_fn _mjPRIVATE_tls_warning_fn = NULL;

callback_fn _mjPRIVATE__get_tls_error_fn(void) {
  return _mjPRIVATE_tls_error_fn;
}

void _mjPRIVATE__set_tls_error_fn(callback_fn h) {
  _mjPRIVATE_tls_error_fn = h;
}

callback_fn _mjPRIVATE__get_tls_warning_fn(void) {
  return _mjPRIVATE_tls_warning_fn;
}

void _mjPRIVATE__set_tls_warning_fn(callback_fn h) {
  _mjPRIVATE_tls_warning_fn = h;
}

//------------------------------ error hadling -----------------------------------------------------

// write datetime, type: message to SIMCORE_LOG.TXT
void sim_math_writeLog(const char* type, const char* msg) {
  time_t rawtime;
  struct tm timeinfo;
  FILE* fp = fopen("SIMCORE_LOG.TXT", "a+t");
  if (fp) {
    // get time
    time(&rawtime);

#if defined(_POSIX_C_SOURCE) || defined(__APPLE__) || defined(__STDC_VERSION_TIME_H__) || defined(__EMSCRIPTEN__)
    localtime_r(&rawtime, &timeinfo);
#elif _MSC_VER
    localtime_s(&timeinfo, &rawtime);
#elif __STDC_LIB_EXT1__
    localtime_s(&rawtime, &timeinfo);
#else
    #error "Thread-safe version of `localtime` is not present in the standard C library"
#endif

    // write to log file
    fprintf(fp, "%s%s: %s\n\n", asctime(&timeinfo), type, msg);
    fclose(fp);
  }
}


void sim_math_error_raw(const char* msg) {
  if (_mjPRIVATE_tls_error_fn) {
    _mjPRIVATE_tls_error_fn(msg);
  } else if (sim_math_user_error) {
    sim_math_user_error(msg);
  } else {
    // write to log and console
    sim_math_writeLog("ERROR", msg);
    printf("ERROR: %s\n\n", msg);

    // exit
    exit(EXIT_FAILURE);
  }
}


void sim_math_error_v(const char* msg, va_list args) {
  // Format msg into errmsg
  char errmsg[1024];
  vsnprintf(errmsg, SIM_SIZEOFARRAY(errmsg), msg, args);
  sim_math_error_raw(errmsg);
}


// write message to logfile and console, pause and exit
void sim_error(const char* msg, ...) {
  va_list args;
  va_start(args, msg);
  sim_math_error_v(msg, args);
  va_end(args);
}


// write message to logfile and console
void sim_warning(const char* msg, ...) {
  char wrnmsg[1024];

  // Format msg into wrnmsg
  va_list args;
  va_start(args, msg);
  vsnprintf(wrnmsg, SIM_SIZEOFARRAY(wrnmsg), msg, args);
  va_end(args);

  if (_mjPRIVATE_tls_warning_fn) {
    _mjPRIVATE_tls_warning_fn(wrnmsg);
  } else if (sim_math_user_warning) {
    sim_math_user_warning(wrnmsg);
  } else {
    // write to log file and console
    sim_math_writeLog("WARNING", wrnmsg);
    printf("WARNING: %s\n\n", wrnmsg);
  }
}


// error with int argument
void sim_math_error_i(const char* msg, int i) {
  sim_error(msg, i);
}


// warning with int argument
void sim_math_warning_i(const char* msg, int i) {
  sim_warning(msg, i);
}


// error string argument
void sim_math_error_s(const char* msg, const char* text) {
  sim_error(msg, text);
}


// warning string argument
void sim_math_warning_s(const char* msg, const char* text) {
  sim_warning(msg, text);
}


//------------------------------ malloc and free ---------------------------------------------------

// allocate memory; byte-align on 64; pad size to multiple of 64
void* sim_malloc(size_t size) {
  void* ptr = 0;

  // user allocator
  if (sim_math_user_malloc) {
    ptr = sim_math_user_malloc(size);
  }

  // default allocator
  else {
    // pad size to multiple of 64
    if (size > 0 && (size % 64)) {
      size += 64 - (size % 64);
    }

    // allocate
    if (size > 0) {
      ptr = sim_math_alignedMalloc(size, 64);
    }
  }

  // error if null pointer
  if (!ptr && size > 0) {
    sim_error("Could not allocate memory");
  }

  return ptr;
}


// free memory
void sim_free(void* ptr) {
  // return if null
  if (!ptr) {
    return;
  }

  // free with user or built-in function
  if (sim_math_user_free) {
    sim_math_user_free(ptr);
  } else {
    sim_math_alignedFree(ptr);
  }
}
