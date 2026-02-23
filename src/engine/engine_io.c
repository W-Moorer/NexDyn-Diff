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

#include "engine/engine_io.h"

#include <inttypes.h>  // IWYU pragma: keep
#include <limits.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <simcore/SIM_model.h>
#include <simcore/SIM_plugin.h>
#include <simcore/SIM_san.h>  // IWYU pragma: keep
#include <simcore/SIM_xmacro.h>
#include "engine/engine_core_smooth.h"
#include "engine/engine_forward.h"
#include "engine/engine_init.h"
#include "engine/engine_macro.h"
#include "engine/engine_memory.h"
#include "engine/engine_plugin.h"
#include "engine/engine_sleep.h"
#include "engine/engine_util_blas.h"
#include "engine/engine_util_errmem.h"
#include "engine/engine_util_misc.h"

#ifdef ADDRESS_SANITIZER
  #include <sanitizer/asan_interface.h>
  #include <sanitizer/common_interface_defs.h>
#endif

#ifdef MEMORY_SANITIZER
  #include <sanitizer/msan_interface.h>
#endif

#ifdef _MSC_VER
  #pragma warning (disable: 4305)  // disable MSVC warning: truncation from 'double' to 'float'
#endif

static const int MAX_ARRAY_SIZE = INT_MAX;


//----------------------------------- static utility functions -------------------------------------

// id used to identify binary sim_model_t file/buffer
static const int ID = 54321;


// number of ints in the simb header
#define NHEADER 5


// macro for referring to a sim_model_t member in generic expressions
#define SIMMODEL_MEMBER(name) (((sim_model_t*) NULL)->name)


// count sim_size_t members in sim_model_t
static int getnsize(void) {
  int cnt = 0;

#define X(name) cnt += _Generic(SIMMODEL_MEMBER(name), sim_size_t: 1, default: 0);
  SIMMODEL_SIZES
#undef X

  return cnt;
}


// count pointers in sim_model_t
static int getnptr(void) {
  int cnt = 0;

#define X(type, name, nr, nc) cnt++;
  SIMMODEL_POINTERS
#undef X

  return cnt;
}


// write to memory buffer
static void bufwrite(const void* src, int num, sim_size_t szbuf, void* buf, sim_size_t* ptrbuf) {
  // check pointers
  if (!src || !buf || !ptrbuf) {
    SIM_ERROR("NULL pointer passed to bufwrite");
  }

  // check size
  if (*ptrbuf+num > szbuf) {
    SIM_ERROR("attempting to write outside model buffer");
  }

  // write, advance pointer
  memcpy((char*)buf + *ptrbuf, src, num);
  *ptrbuf += num;
}


// read from memory buffer
static void bufread(void* dest, int num, sim_size_t szbuf, const void* buf, sim_size_t* ptrbuf) {
  // check pointers
  if (!dest || !buf || !ptrbuf) {
    SIM_ERROR("NULL pointer passed to bufread");
  }

  // check size
  if (*ptrbuf+num > szbuf) {
    SIM_ERROR("attempting to read outside model buffer");
  }

  // read, advance pointer
  memcpy(dest, (char*)buf + *ptrbuf, num);
  *ptrbuf += num;
}


// number of bytes to be skipped to achieve 64-byte alignment
static inline unsigned int SKIP(intptr_t offset) {
  const unsigned int align = 64;
  // compute skipped bytes
  return (align - (offset % align)) % align;
}


//----------------------------------- sim_model_t construction -----------------------------------------

// set pointers in sim_model_t buffer
static void sim_setPtrModel(sim_model_t* m) {
  char* ptr = (char*)m->buffer;

  // prepare symbols needed by xmacro
  SIMMODEL_POINTERS_PREAMBLE(m);

  // assign pointers with padding
#define X(type, name, nr, nc)                             \
  m->name = (type*)(ptr + SKIP((intptr_t)ptr));           \
  ASAN_POISON_MEMORY_REGION(ptr, PTRDIFF(m->name, ptr));  \
  ptr += SKIP((intptr_t)ptr) + sizeof(type)*(m->nr)*(nc);

  SIMMODEL_POINTERS
#undef X

  // check size
  ptrdiff_t sz = ptr - (char*)m->buffer;
  if (m->nbuffer != sz) {
    SIM_ERROR(
        "sim_model_t buffer size mismatch, "
        "expected size: %" PRIu64 ",  actual size: %td",
        m->nbuffer, sz);
  }
}


// increases buffer size without causing integer overflow, returns 0 if
// operation would cause overflow
// performs the following operations:
// *nbuffer += SKIP(*offset) + type_size*nr*nc;
// *offset += SKIP(*offset) + type_size*nr*nc;
static sim_size_t safeAddToBufferSize(intptr_t* offset, sim_size_t* nbuffer,
                                   size_t type_size, sim_size_t nr, sim_size_t nc) {
  if (type_size < 0 || nr < 0 || nc < 0) {
    return 0;
  }
#if (__has_builtin(__builtin_add_overflow) && __has_builtin(__builtin_mul_overflow)) \
  || (defined(__GNUC__) && __GNUC__ >= 5)
  // supported by GCC and Clang
  size_t to_add = 0;
  if (__builtin_mul_overflow(nc, nr, &to_add)) return 0;
  if (__builtin_mul_overflow(to_add, type_size, &to_add)) return 0;
  if (__builtin_add_overflow(to_add, SKIP(*offset), &to_add)) return 0;
  if (__builtin_add_overflow(*nbuffer, to_add, nbuffer)) return 0;
  if (__builtin_add_overflow(*offset, to_add, offset)) return 0;
#else
  // TODO: offer a safe implementation for MSVC or other compilers that don't have the builtins
  *nbuffer += SKIP(*offset) + type_size*nr*nc;
  *offset += SKIP(*offset) + type_size*nr*nc;
#endif

  return 1;
}


// free model memory without destroying the struct
static void freeModelBuffers(sim_model_t* m) {
  sim_free(m->buffer);
}


// allocate and initialize sim_model_t structure
void sim_makeModel(sim_model_t** dest,
    sim_size_t nq, sim_size_t nv, sim_size_t nu, sim_size_t na, sim_size_t nbody, sim_size_t nbvh, sim_size_t nbvhstatic,
    sim_size_t nbvhdynamic, sim_size_t noct, sim_size_t njnt, sim_size_t ntree, sim_size_t nM, sim_size_t nB,
    sim_size_t nC, sim_size_t nD, sim_size_t ngeom, sim_size_t nsite, sim_size_t ncam, sim_size_t nlight,
    sim_size_t nflex, sim_size_t nflexnode, sim_size_t nflexvert, sim_size_t nflexedge, sim_size_t nflexelem,
    sim_size_t nflexelemdata, sim_size_t nflexelemedge, sim_size_t nflexshelldata, sim_size_t nflexevpair,
    sim_size_t nflextexcoord, sim_size_t nJfe, sim_size_t nJfv, sim_size_t nmesh, sim_size_t nmeshvert,
    sim_size_t nmeshnormal, sim_size_t nmeshtexcoord, sim_size_t nmeshface, sim_size_t nmeshgraph,
    sim_size_t nmeshpoly, sim_size_t nmeshpolyvert, sim_size_t nmeshpolymap, sim_size_t nskin,
    sim_size_t nskinvert, sim_size_t nskintexvert, sim_size_t nskinface, sim_size_t nskinbone,
    sim_size_t nskinbonevert, sim_size_t nhfield, sim_size_t nhfielddata, sim_size_t ntex, sim_size_t ntexdata,
    sim_size_t nmat, sim_size_t npair, sim_size_t nexclude, sim_size_t neq, sim_size_t ntendon, sim_size_t nwrap,
    sim_size_t nsensor, sim_size_t nnumeric, sim_size_t nnumericdata, sim_size_t ntext, sim_size_t ntextdata,
    sim_size_t ntuple, sim_size_t ntupledata, sim_size_t nkey, sim_size_t nmocap, sim_size_t nplugin,
    sim_size_t npluginattr, sim_size_t nuser_body, sim_size_t nuser_jnt, sim_size_t nuser_geom,
    sim_size_t nuser_site, sim_size_t nuser_cam, sim_size_t nuser_tendon, sim_size_t nuser_actuator,
    sim_size_t nuser_sensor, sim_size_t nnames, sim_size_t npaths) {
  intptr_t offset = 0;
  int allocate = *dest ? 0 : 1;
  sim_model_t* m = NULL;

  // CHECK SIZE PARAMETERS
  {
    // dummy variables for SIMMODEL_SIZES set after sim_model_t construction
    int nnames_map = 0, nJmom = 0, nJten = 0, ngravcomp = 0, nemax = 0, njmax = 0, nconmax=0;
    int nuserdata=0, nsensordata=0, npluginstate=0, nhistory=0, narena=0, nbuffer=0;

    // sizes must be non-negative and fit in int, except for the byte arrays texdata and textdata
    #define X(name)                                                                   \
      if (name < 0) {                                                                 \
        sim_warning("Invalid model: %s is negative (%lld).", #name, (long long)name); \
        return;                                                                       \
      }                                                                               \
      if (name >= MAX_ARRAY_SIZE &&                                                   \
          strcmp(#name, "ntexdata") != 0 && strcmp(#name, "ntextdata") != 0) {        \
        sim_warning("Invalid model: %s is too large. Expected < %d. Got %lld.",       \
                    #name, MAX_ARRAY_SIZE, (long long)name);                          \
        return;                                                                       \
      }
    SIMMODEL_SIZES
#undef X

    // suppress unused variable warnings
    (void)nnames_map; (void)nJmom; (void)nJten; (void)ngravcomp; (void)nemax; (void)njmax; (void)nconmax;
    (void)nuserdata; (void)nsensordata; (void)npluginstate; (void)nhistory; (void)narena;
    (void)nbuffer;
  }

  // nbody should always be positive
  if (nbody == 0) {
    sim_warning("Invalid model: nbody == 0");
    return;
  }

  // allocate sim_model_t
  if (!allocate) {
    m = *dest;
    freeModelBuffers(m);
  } else {
    m = (sim_model_t*)sim_malloc(sizeof(sim_model_t));
  }

  if (!m) {
    SIM_ERROR("could not allocate sim_model_t");
  }
  memset(m, 0, sizeof(sim_model_t));

  // set size parameters
  m->nq = nq;
  m->nv = nv;
  m->nu = nu;
  m->na = na;
  m->nbody = nbody;
  m->nbvh = nbvh;
  m->nbvhstatic = nbvhstatic;
  m->nbvhdynamic = nbvhdynamic;
  m->noct = noct;
  m->njnt = njnt;
  m->ntree = ntree;
  m->nM = nM;
  m->nB = nB;
  m->nC = nC;
  m->nD = nD;
  m->ngeom = ngeom;
  m->nsite = nsite;
  m->ncam = ncam;
  m->nlight = nlight;
  m->nflex = nflex;
  m->nflexnode = nflexnode;
  m->nflexvert = nflexvert;
  m->nflexedge = nflexedge;
  m->nflexelem = nflexelem;
  m->nflexelemdata = nflexelemdata;
  m->nflexelemedge = nflexelemedge;
  m->nflexshelldata = nflexshelldata;
  m->nflexevpair = nflexevpair;
  m->nflextexcoord = nflextexcoord;
  m->nJfe = nJfe;
  m->nJfv = nJfv;
  m->nmesh = nmesh;
  m->nmeshvert = nmeshvert;
  m->nmeshnormal = nmeshnormal;
  m->nmeshtexcoord = nmeshtexcoord;
  m->nmeshface = nmeshface;
  m->nmeshgraph = nmeshgraph;
  m->nmeshpoly = nmeshpoly;
  m->nmeshpolyvert = nmeshpolyvert;
  m->nmeshpolymap = nmeshpolymap;
  m->nskin = nskin;
  m->nskinvert = nskinvert;
  m->nskintexvert = nskintexvert;
  m->nskinface = nskinface;
  m->nskinbone = nskinbone;
  m->nskinbonevert = nskinbonevert;
  m->nhfield = nhfield;
  m->nhfielddata = nhfielddata;
  m->ntex = ntex;
  m->ntexdata = ntexdata;
  m->nmat = nmat;
  m->npair = npair;
  m->nexclude = nexclude;
  m->neq = neq;
  m->ntendon = ntendon;
  m->nwrap = nwrap;
  m->nsensor = nsensor;
  m->nnumeric = nnumeric;
  m->nnumericdata = nnumericdata;
  m->ntext = ntext;
  m->ntextdata = ntextdata;
  m->ntuple = ntuple;
  m->ntupledata = ntupledata;
  m->nkey = nkey;
  m->nmocap = nmocap;
  m->nplugin = nplugin;
  m->npluginattr = npluginattr;
  m->nuser_body = nuser_body;
  m->nuser_jnt = nuser_jnt;
  m->nuser_geom = nuser_geom;
  m->nuser_site = nuser_site;
  m->nuser_cam = nuser_cam;
  m->nuser_tendon = nuser_tendon;
  m->nuser_actuator = nuser_actuator;
  m->nuser_sensor = nuser_sensor;
  m->nnames = nnames;
  long nnames_map = (long)nbody + njnt + ngeom + nsite + ncam + nlight + nflex + nmesh + nskin +
                    nhfield + ntex + nmat + npair + nexclude + neq + ntendon + nu + nsensor +
                    nnumeric + ntext + ntuple + nkey + nplugin;
  if (nnames_map >= INT_MAX / SIM_LOAD_MULTIPLE) {
    if (allocate) sim_free(m);
    sim_warning("Invalid model: size of nnames_map is larger than INT_MAX");
    return;
  }
  m->nnames_map = SIM_LOAD_MULTIPLE * nnames_map;
  m->npaths = npaths;

  // compute buffer size
  m->nbuffer = 0;
#define X(type, name, nr, nc)                                                \
  if (!safeAddToBufferSize(&offset, &m->nbuffer, sizeof(type), m->nr, nc)) { \
    if (allocate) sim_free(m);                                               \
    sim_warning("Invalid model: " #name " too large.");                      \
    return;                                                                  \
  }

  SIMMODEL_POINTERS
#undef X

  // allocate buffer
  m->buffer = sim_malloc(m->nbuffer);
  if (!m->buffer) {
    if (allocate) sim_free(m);
    SIM_ERROR("could not allocate sim_model_t buffer");
  }

  // clear, set pointers in buffer
  memset(m->buffer, 0, m->nbuffer);
#ifdef MEMORY_SANITIZER
  // tell msan to treat the entire buffer as uninitialized
  __msan_allocated_memory(m->buffer, m->nbuffer);
#endif
  sim_setPtrModel(m);

  // set default options
  sim_defaultOption(&m->opt);
  sim_defaultStatistic(&m->stat);

  // copy pointer if allocated here
  if (allocate) {
    *dest = m;
  }
}


// copy sim_model_t, if dest==NULL create new model
sim_model_t* sim_copyModel(sim_model_t* dest, const sim_model_t* src) {
  // allocate new model if needed
  if (!dest) {
    sim_makeModel(
        &dest, src->nq, src->nv, src->nu, src->na, src->nbody, src->nbvh,
        src->nbvhstatic, src->nbvhdynamic, src->noct, src->njnt, src->ntree,
        src->nM, src->nB, src->nC, src->nD, src->ngeom, src->nsite, src->ncam,
        src->nlight, src->nflex, src->nflexnode, src->nflexvert, src->nflexedge,
        src->nflexelem, src->nflexelemdata, src->nflexelemedge, src->nflexshelldata,
        src->nflexevpair, src->nflextexcoord, src->nJfe, src->nJfv, src->nmesh,
        src->nmeshvert, src->nmeshnormal, src->nmeshtexcoord, src->nmeshface,
        src->nmeshgraph, src->nmeshpoly, src->nmeshpolyvert, src->nmeshpolymap,
        src->nskin, src->nskinvert, src->nskintexvert, src->nskinface,
        src->nskinbone, src->nskinbonevert, src->nhfield, src->nhfielddata,
        src->ntex, src->ntexdata, src->nmat, src->npair, src->nexclude,
        src->neq, src->ntendon, src->nwrap, src->nsensor, src->nnumeric,
        src->nnumericdata, src->ntext, src->ntextdata, src->ntuple,
        src->ntupledata, src->nkey, src->nmocap, src->nplugin, src->npluginattr,
        src->nuser_body, src->nuser_jnt, src->nuser_geom, src->nuser_site,
        src->nuser_cam, src->nuser_tendon, src->nuser_actuator,
        src->nuser_sensor, src->nnames, src->npaths);
  }
  if (!dest) {
    SIM_ERROR("failed to make sim_model_t. Invalid sizes.");
  }

  // check sizes
  if (dest->nbuffer != src->nbuffer) {
    sim_deleteModel(dest);
    SIM_ERROR("dest and src models have different buffer size");
  }

  // save buffer ptr, copy struct, restore buffer and other pointers
  void* save_bufptr = dest->buffer;
  *dest = *src;
  dest->buffer = save_bufptr;
  sim_setPtrModel(dest);

  // copy buffer contents
  {
    SIMMODEL_POINTERS_PREAMBLE(src)
    #define X(type, name, nr, nc)  \
      memcpy((char*)dest->name, (const char*)src->name, sizeof(type)*(src->nr)*nc);
    SIMMODEL_POINTERS
    #undef X
  }

  return dest;
}


// save model to binary file, or memory buffer of szbuf>0
void sim_saveModel(const sim_model_t* m, const char* filename, void* buffer, int buffer_sz) {
  FILE* fp = 0;
  sim_size_t ptrbuf = 0;

  // standard header
  int header[NHEADER] = {ID, sizeof(sim_scalar_t), getnsize(), sim_version(), getnptr()};

  // open file for writing if no buffer
  if (!buffer) {
    fp = fopen(filename, "wb");
    if (!fp) {
      sim_warning("Could not open file '%s'", filename);
      return;
    }
  }

  // write standard header, info, options, buffer (omit pointers)
  if (fp) {
    fwrite(header, sizeof(int), NHEADER, fp);
    #define X(name) fwrite(&m->name, sizeof(m->name), 1, fp);
    SIMMODEL_SIZES
    #undef X
    fwrite((void*)&m->opt, sizeof(SIM_Option), 1, fp);
    fwrite((void*)&m->stat, sizeof(SIM_Statistic), 1, fp);
    {
      SIMMODEL_POINTERS_PREAMBLE(m)
      #define X(type, name, nr, nc)  \
        fwrite((void*)m->name, sizeof(type), (m->nr)*(nc), fp);
      SIMMODEL_POINTERS
      #undef X
    }
  } else {
    bufwrite(header, sizeof(header), buffer_sz, buffer, &ptrbuf);
    #define X(name) bufwrite(&m->name, sizeof(m->name), buffer_sz, buffer, &ptrbuf);
    SIMMODEL_SIZES
    #undef X
    bufwrite((void*)&m->opt, sizeof(SIM_Option), buffer_sz, buffer, &ptrbuf);
    bufwrite((void*)&m->stat, sizeof(SIM_Statistic), buffer_sz, buffer, &ptrbuf);
    {
      SIMMODEL_POINTERS_PREAMBLE(m)
      #define X(type, name, nr, nc)  \
        bufwrite((void*)m->name, sizeof(type)*(m->nr)*(nc), buffer_sz, buffer, &ptrbuf);
      SIMMODEL_POINTERS
      #undef X
    }
  }

  if (fp) {
    fclose(fp);
  }
}


// load binary SIMB model
sim_model_t* sim_loadModelBuffer(const void* buffer, int buffer_sz) {
  sim_size_t ptrbuf = 0;
  sim_model_t *m = 0;

  if (buffer_sz < NHEADER*sizeof(int)) {
    sim_warning("Model file has an incomplete header");
    return NULL;
  }

  int header[NHEADER] = {0};
  bufread(header, NHEADER*sizeof(int), buffer_sz, buffer, &ptrbuf);

  // check header
  int expected_header[NHEADER] = {ID, sizeof(sim_scalar_t), getnsize(), sim_version(), getnptr()};
  for (int i=0; i < NHEADER; i++) {
    if (header[i] != expected_header[i]) {
      switch (i) {
      case 0:
        sim_warning("Model missing header ID");
        return NULL;

      case 1:
        sim_warning("Model and executable have different floating point precision");
        return NULL;

      case 2:
        sim_warning("Model and executable have different number of sizes in sim_model_t");
        return NULL;

      case 3:
        sim_warning("Model and executable use different SimCore version");
        return NULL;

      default:
        sim_warning("Model and executable have different number of pointers in sim_model_t");
        return NULL;
      }
    }
  }

  int nsize = getnsize();  // number of sim_size_t fields in sim_model_t

  if (ptrbuf + sizeof(sim_size_t)*nsize > buffer_sz) {
    sim_warning("Truncated model file - ran out of data while reading sizes");
    return NULL;
  }

  // read sim_model_t construction fields
  sim_size_t sizes[256];
  bufread(sizes, sizeof(sim_size_t)*nsize, buffer_sz, buffer, &ptrbuf);

  // allocate new sim_model_t
  sim_makeModel(&m,
               sizes[0],  sizes[1],  sizes[2],  sizes[3],  sizes[4],  sizes[5],  sizes[6],
               sizes[7],  sizes[8],  sizes[9],  sizes[10], sizes[11], sizes[12], sizes[13],
               sizes[14], sizes[15], sizes[16], sizes[17], sizes[18], sizes[19], sizes[20],
               sizes[21], sizes[22], sizes[23], sizes[24], sizes[25], sizes[26], sizes[27],
               sizes[28], sizes[29], sizes[30], sizes[31], sizes[32], sizes[33], sizes[34],
               sizes[35], sizes[36], sizes[37], sizes[38], sizes[39], sizes[40], sizes[41],
               sizes[42], sizes[43], sizes[44], sizes[45], sizes[46], sizes[47], sizes[48],
               sizes[49], sizes[50], sizes[51], sizes[52], sizes[53], sizes[54], sizes[55],
               sizes[56], sizes[57], sizes[58], sizes[59], sizes[60], sizes[61], sizes[62],
               sizes[63], sizes[64], sizes[65], sizes[66], sizes[67], sizes[68], sizes[69],
               sizes[70], sizes[71], sizes[72], sizes[73], sizes[74], sizes[75], sizes[76]);

  // sim_makeModel may fail if the input buffer has invalid sizes
  if (!m) {
    sim_warning("Invalid sizes, unable to load model");
    return NULL;
  }

  // check buffer size (last sim_size_t field is nbuffer)
  if (m->nbuffer != sizes[nsize-1]) {
    sim_warning("Corrupted model, wrong nbuffer field");
    sim_deleteModel(m);
    return NULL;
  }

  // set integer fields
  {
    int int_idx = 0;
    #define X(name) \
        m->name = sizes[int_idx++];
    SIMMODEL_SIZES
    #undef X
  }

  // read options and buffer
  if (ptrbuf + sizeof(SIM_Option) + sizeof(SIM_Statistic) > buffer_sz) {
    sim_warning("Truncated model file - ran out of data while reading structs");
    return NULL;
  }
  bufread((void*)&m->opt, sizeof(SIM_Option), buffer_sz, buffer, &ptrbuf);
  bufread((void*)&m->stat, sizeof(SIM_Statistic), buffer_sz, buffer, &ptrbuf);
  {
    SIMMODEL_POINTERS_PREAMBLE(m)
    #define X(type, name, nr, nc)                                           \
      if (ptrbuf + sizeof(type) * (m->nr) * (nc) > buffer_sz) {             \
        sim_warning(                                                        \
            "Truncated model file - ran out of data while reading " #name); \
        sim_deleteModel(m);                                                  \
        return NULL;                                                        \
      }                                                                     \
      bufread(m->name, sizeof(type)*(m->nr)*(nc), buffer_sz, buffer, &ptrbuf);

    SIMMODEL_POINTERS
    #undef X
  }

  // make sure buffer is the correct size
  if (ptrbuf != buffer_sz) {
    sim_warning("Model file is too large");
    sim_deleteModel(m);
    return NULL;
  }

  const char* validationError = sim_validateReferences(m);
  if (validationError) {
    sim_warning("%s", validationError);
    sim_deleteModel(m);
    return NULL;
  }

  return m;
}


// de-allocate sim_model_t
void sim_deleteModel(sim_model_t* m) {
  if (m) {
    freeModelBuffers(m);
    sim_free(m);
  }
}


// size of buffer needed to hold model
sim_size_t sim_sizeModel(const sim_model_t* m) {
  sim_size_t size = (
    sizeof(int)*NHEADER
    + sizeof(sim_size_t)*getnsize()
    + sizeof(SIM_Option)
    + sizeof(SIM_Statistic));

  SIMMODEL_POINTERS_PREAMBLE(m)
#define X(type, name, nr, nc)         \
  size += sizeof(type)*(m->nr)*(nc);
  SIMMODEL_POINTERS
#undef X

  return size;
}


//-------------------------- sparse system matrix construction -------------------------------------

// construct sparse representation of dof-dof matrix
void sim_makeDofDofSparse(int nv, int nC, int nD, int nM,
                         const int* dof_parentid, const int* dof_simplenum,
                         int* rownnz, int* rowadr, int* diag, int* colind,
                         int reduced, int upper,
                         int* remaining) {
  // no dofs, nothing to do
  if (!nv) {
    return;
  }

  // compute rownnz
  sim_math_zeroInt(rownnz, nv);
  for (int i = nv - 1; i >= 0; i--) {
    // init at diagonal
    int j = i;
    rownnz[i]++;

    // process below diagonal unless reduced and dof is simple
    if (!(reduced && dof_simplenum[i])) {
      while ((j = dof_parentid[j]) >= 0) {
        // both reduced and non-reduced have lower triangle
        rownnz[i]++;

        // add upper triangle if requested
        if (upper) rownnz[j]++;
      }
    }
  }

  // accumulate rowadr
  rowadr[0] = 0;
  for (int i = 1; i < nv; i++) {
    rowadr[i] = rowadr[i - 1] + rownnz[i - 1];
  }

  // populate colind
  sim_math_copyInt(remaining, rownnz, nv);
  for (int i = nv - 1; i >= 0; i--) {
    // init at diagonal
    remaining[i]--;
    colind[rowadr[i] + remaining[i]] = i;

    // process below diagonal unless reduced and dof is simple
    if (!(reduced && dof_simplenum[i])) {
      int j = i;
      while ((j = dof_parentid[j]) >= 0) {
        remaining[i]--;
        colind[rowadr[i] + remaining[i]] = j;

        // add upper triangle if requested
        if (upper) {
          remaining[j]--;
          colind[rowadr[j] + remaining[j]] = i;
        }
      }
    }
  }

  // check for remaining; SHOULD NOT OCCUR
  for (int i = 0; i < nv; i++) {
    if (remaining[i] != 0) {
      SIM_ERROR("unexpected remaining");
    }
  }

  // check total nnz; SHOULD NOT OCCUR
  int expected_nnz = upper ? nD : (reduced ? nC : nM);
  if (rowadr[nv - 1] + rownnz[nv - 1] != expected_nnz) {
    SIM_ERROR("sum of rownnz different from expected");
  }

  // find diagonal indices
  if (diag) {
    for (int i = 0; i < nv; i++) {
      int adr = rowadr[i];
      int j = 0;
      while (colind[adr + j] < i && j < rownnz[i]) {
        j++;
      }
      if (colind[adr + j] != i) {
        SIM_ERROR("diagonal index not found");
      }
      diag[i] = j;
    }
  }
}

// construct sparse representation of body-dof matrix
void sim_makeBSparse(int nv, int nbody, int nB,
                    const int* body_dofnum, const int* body_parentid, const int* body_dofadr,
                    int* B_rownnz, int* B_rowadr, int* B_colind,
                    int* count) {
  // set rownnz to subtree dofs counts, including self
  sim_math_zeroInt(B_rownnz, nbody);
  for (int i = nbody - 1; i > 0; i--) {
    B_rownnz[i] += body_dofnum[i];
    B_rownnz[body_parentid[i]] += B_rownnz[i];
  }

  // check if rownnz[0] != nv; SHOULD NOT OCCUR
  if (B_rownnz[0] != nv) {
    SIM_ERROR("rownnz[0] different from nv");
  }

  // add dofs in ancestors bodies
  for (int i = 0; i < nbody; i++) {
    int j = body_parentid[i];
    while (j > 0) {
      B_rownnz[i] += body_dofnum[j];
      j = body_parentid[j];
    }
  }

  // compute rowadr
  B_rowadr[0] = 0;
  for (int i = 1; i < nbody; i++) {
    B_rowadr[i] = B_rowadr[i - 1] + B_rownnz[i - 1];
  }

  // check if total nnz != nB; SHOULD NOT OCCUR
  if (nB != B_rowadr[nbody - 1] + B_rownnz[nbody - 1]) {
    SIM_ERROR("sum of rownnz different from nB");
  }

  // clear incremental row counts
  sim_math_zeroInt(count, nbody);

  // add subtree dofs to colind
  for (int i = nbody - 1; i > 0; i--) {
    // add this body's dofs to subtree
    for (int n = 0; n < body_dofnum[i]; n++) {
      B_colind[B_rowadr[i] + count[i]] = body_dofadr[i] + n;
      count[i]++;
    }

    // add body subtree to parent
    int par = body_parentid[i];
    for (int n = 0; n < count[i]; n++) {
      B_colind[B_rowadr[par] + count[par]] = B_colind[B_rowadr[i] + n];
      count[par]++;
    }
  }

  // add all ancestor dofs
  for (int i = 0; i < nbody; i++) {
    int par = body_parentid[i];
    while (par > 0) {
      // add ancestor body dofs
      for (int n = 0; n < body_dofnum[par]; n++) {
        B_colind[B_rowadr[i] + count[i]] = body_dofadr[par] + n;
        count[i]++;
      }

      // advance to parent
      par = body_parentid[par];
    }
  }

  // process all bodies
  for (int i = 0; i < nbody; i++) {
    // make sure cnt = rownnz; SHOULD NOT OCCUR
    if (B_rownnz[i] != count[i]) {
      SIM_ERROR("cnt different from rownnz");
    }

    // sort colind in each row
    if (count[i] > 1) {
      sim_math_insertionSortInt(B_colind + B_rowadr[i], count[i]);
    }
  }
}


// check D and B sparsity for consistency
static void checkDBSparse(const sim_model_t* m) {
  // process all dofs
  for (int j = 0; j < m->nv; j++) {
    // get body for this dof
    int i = m->dof_bodyid[j];

    // D[row j] and B[row i] should be identical
    if (m->D_rownnz[j] != m->B_rownnz[i]) {
      SIM_ERROR("rows have different nnz");
    }
    for (int k = 0; k < m->D_rownnz[j]; k++) {
      if (m->D_colind[m->D_rowadr[j] + k] != m->B_colind[m->B_rowadr[i] + k]) {
        SIM_ERROR("rows have different colind");
      }
    }
  }
}


// integer valued dst[D or C or M] = src[M (legacy)], handle different sparsity representations
static void copyM2Sparse(int nv,
                         const int* dof_Madr, const int* dof_simplenum, const int* dof_parentid,
                         const int* rownnz, const int* rowadr, const int* src,
                         int* dst,
                         int reduced, int upper, int* remaining) {
  // init remaining
  sim_math_copyInt(remaining, rownnz, nv);

  // copy data
  for (int i = nv - 1; i >= 0; i--) {
    // init at diagonal
    int adr = dof_Madr[i];
    remaining[i]--;
    dst[rowadr[i] + remaining[i]] = src[adr];
    adr++;

    // process below diagonal unless reduced and dof is simple
    if (!(reduced && dof_simplenum[i])) {
      int j = i;
      while ((j = dof_parentid[j]) >= 0) {
        remaining[i]--;
        dst[rowadr[i] + remaining[i]] = src[adr];

        // add upper triangle if requested
        if (upper) {
          remaining[j]--;
          dst[rowadr[j] + remaining[j]] = src[adr];
        }

        adr++;
      }
    }
  }

  // check that none remaining
  for (int i=0; i < nv; i++) {
    if (remaining[i]) {
      SIM_ERROR("unassigned index");
    }
  }
}


// construct index mappings between M <-> D, M -> C, M (legacy) -> M (CSR)
void sim_makeDofDofMaps(int nv, int nM, int nC, int nD,
                       const int* dof_Madr, const int* dof_simplenum, const int* dof_parentid,
                       const int* D_rownnz, const int* D_rowadr, const int* D_colind,
                       const int* M_rownnz, const int* M_rowadr, const int* M_colind,
                       int* mapM2D, int* mapD2M, int* mapM2M,
                       int* M, int* scratch) {
  // make mapM2D: M -> D (lower to symmetric)
  sim_math_lower2SymMap(mapM2D, nv, D_rowadr, D_rownnz, D_colind, M_rowadr, M_rownnz, M_colind, scratch);

  // make mapD2M: D -> M (symmetric to lower)
  sim_math_sparseMap(mapD2M, nv, M_rowadr, M_rownnz, M_colind, D_rowadr, D_rownnz, D_colind);

  // make mapM2M
  for (int i=0; i < nM; i++) M[i] = i;
  sim_math_fillInt(mapM2M, -1, nC);
  copyM2Sparse(nv, dof_Madr, dof_simplenum, dof_parentid, M_rownnz,
               M_rowadr, M, mapM2M, /*reduced=*/1, /*upper=*/0, scratch);

  // check that all indices are filled in
  for (int i=0; i < nC; i++) {
    if (mapM2M[i] < 0) {
      SIM_ERROR("unassigned index in mapM2M");
    }
  }
}


//----------------------------------- sim_data_t construction ------------------------------------------

// set pointers into sim_data_t buffer
static void sim_setPtrData(const sim_model_t* m, sim_data_t* d) {
  char* ptr = (char*)d->buffer;

  // assign pointers with padding
#define X(type, name, nr, nc)                             \
  d->name = (type*)(ptr + SKIP((intptr_t)ptr));           \
  ASAN_POISON_MEMORY_REGION(ptr, PTRDIFF(d->name, ptr));  \
  ptr += SKIP((intptr_t)ptr) + sizeof(type)*(m->nr)*(nc);

  SIMDATA_POINTERS
#undef X

  // check size
  ptrdiff_t sz = ptr - (char*)d->buffer;
  if (d->nbuffer != sz) {
    SIM_ERROR(
        "sim_data_t buffer size mismatch, "
        "expected size: %" PRIu64 ",  actual size: %td",
        d->nbuffer, sz);
  }

  // zero-initialize arena pointers
#define X(type, name, nr, nc) d->name = NULL;
  SIMDATA_ARENA_POINTERS
#undef X

  d->contact = d->arena;
}


// initialize plugins, copy into d (required for deletion)
void sim_initPlugin(const sim_model_t* m, sim_data_t* d) {
  d->nplugin = m->nplugin;
  for (int i = 0; i < m->nplugin; ++i) {
    d->plugin[i] = m->plugin[i];
    const SIM_pPlugin* plugin = sim_plugin_getPluginAtSlot(m->plugin[i]);
    if (plugin->init && plugin->init(m, d, i) < 0) {
      sim_free(d->buffer);
      sim_free(d->arena);
      sim_free(d);
      SIM_ERROR("plugin->init failed for plugin id %d", i);
    }
  }
}


// free sim_data_t memory without destroying the struct
static void freeDataBuffers(sim_data_t* d) {
#ifdef ADDRESS_SANITIZER
    // raise an error if there's a dangling stack frame
    sim_freeStack(d);
#endif

    // destroy plugin instances
    for (int i = 0; i < d->nplugin; ++i) {
      const SIM_pPlugin* plugin = sim_plugin_getPluginAtSlot(d->plugin[i]);
      if (plugin->destroy) {
        plugin->destroy(d, i);
      }
    }
    sim_free(d->buffer);
    sim_free(d->arena);
}


// allocate and initialize raw sim_data_t structure
void sim_makeRawData(sim_data_t** dest, const sim_model_t* m) {
  intptr_t offset = 0;
  int allocate = *dest ? 0 : 1;
  sim_data_t* d = NULL;

  // allocate sim_data_t
  if (!allocate) {
    d = *dest;
    freeDataBuffers(d);
  } else {
    d = (sim_data_t*) sim_malloc(sizeof(sim_data_t));
  }

  if (!d) {
    SIM_ERROR("could not allocate sim_data_t");
  }

  // compute buffer size
  d->nbuffer = 0;
  d->buffer = d->arena = NULL;
#define X(type, name, nr, nc)                                                \
  if (!safeAddToBufferSize(&offset, &d->nbuffer, sizeof(type), m->nr, nc)) { \
    if (allocate) sim_free(d);                                               \
    sim_warning("Invalid data: " #name " too large.");                       \
    return;                                                                  \
  }

  SIMDATA_POINTERS
#undef X

  // copy stack size from model
  d->narena = m->narena;

  // allocate buffer
  d->buffer = sim_malloc(d->nbuffer);
  if (!d->buffer) {
    if (allocate) sim_free(d);
    SIM_ERROR("could not allocate sim_data_t buffer");
  }

  // allocate arena
  d->arena = sim_malloc(d->narena);
  if (!d->arena) {
    sim_free(d->buffer);
    if (allocate) sim_free(d);
    SIM_ERROR("could not allocate sim_data_t arena");
  }

  // set pointers into buffer
  sim_setPtrData(m, d);

  // clear threadpool
  d->threadpool = 0;

  // clear nplugin (overwritten by _initPlugin)
  d->nplugin = 0;

  // set awake array sizes to default (all awake)
  d->ntree_awake = m->ntree;
  d->nbody_awake = d->nparent_awake = m->nbody;
  d->nv_awake = m->nv;

  // copy pointer if allocated here
  if (allocate) {
    *dest = d;
  }
}


// allocate and initialize sim_data_t structure
sim_data_t* sim_makeData(const sim_model_t* m) {
  sim_data_t* d = NULL;
  sim_makeRawData(&d, m);
  if (d) {
    sim_initPlugin(m, d);
    sim_resetData(m, d);
  }
  return d;
}


// copy sim_data_t, if dest==NULL create new data;
// flg_all  1: copy all fields,  0: skip optional large arrays
sim_data_t* sim_copyDataInternal(sim_data_t* dest, const sim_model_t* m, const sim_data_t* src, int flg_all) {
  void* save_buffer;
  void* save_arena;

  // allocate new data if needed
  if (!dest) {
    sim_makeRawData(&dest, m);
    sim_initPlugin(m, dest);
  }

  // check sizes
  if (dest->nbuffer != src->nbuffer) {
    SIM_ERROR("dest and src data buffers have different size");
  }
  if (dest->narena != src->narena) {
    SIM_ERROR("dest and src stacks have different size");
  }

  // stack is in use
  if (src->pstack) {
    SIM_ERROR("attempting to copy sim_data_t while stack is in use");
  }

  // save pointers, copy everything, restore pointers
  save_buffer = dest->buffer;
  save_arena = dest->arena;
  *dest = *src;
  dest->buffer = save_buffer;
  dest->arena = save_arena;
  sim_setPtrData(m, dest);

  // save plugin_data, since the X macro copying block below will override it
  const size_t plugin_data_size = sizeof(*dest->plugin_data) * dest->nplugin;
  uintptr_t* save_plugin_data = NULL;
  if (plugin_data_size) {
    save_plugin_data = (uintptr_t*)sim_malloc(plugin_data_size);
    if (!save_plugin_data) {
      SIM_ERROR("failed to allocate temporary memory for plugin_data");
    }
    memcpy(save_plugin_data, dest->plugin_data, plugin_data_size);
  }

  // copy buffer
  {
    if (flg_all) {
      #define X(type, name, nr, nc)  \
        memcpy((char*)dest->name, (const char*)src->name, sizeof(type)*(m->nr)*nc);
      SIMDATA_POINTERS
      #undef X
    } else {
      // redefine XNV to nothing
      #undef XNV
      #define XNV(type, name, nr, nc)

      #define X(type, name, nr, nc)  \
        memcpy((char*)dest->name, (const char*)src->name, sizeof(type)*(m->nr)*nc);
      SIMDATA_POINTERS
      #undef X

      // redefine XNV to be the same as X
      #undef XNV
      #define XNV X
    }
  }


  // copy arena memory
  #undef SIM_D
  #define SIM_D(n) (src->n)
  #undef SIM_M
  #define SIM_M(n) (m->n)

  if (flg_all) {
    #define X(type, name, nr, nc)                                                \
    if (src->name) {                                                             \
      dest->name = (type*)((char*)dest->arena + PTRDIFF(src->name, src->arena)); \
      ASAN_UNPOISON_MEMORY_REGION(dest->name, sizeof(type) * nr * nc);           \
      memcpy((char*)dest->name, (const char*)src->name, sizeof(type) * nr * nc); \
    } else {                                                                     \
      dest->name = NULL;                                                         \
    }
    SIMDATA_ARENA_POINTERS
    #undef X
  } else {
    // redefine XNV to nothing
    #undef XNV
    #define XNV(type, name, nr, nc)

    #define X(type, name, nr, nc)                                                \
    if (src->name) {                                                             \
      dest->name = (type*)((char*)dest->arena + PTRDIFF(src->name, src->arena)); \
      ASAN_UNPOISON_MEMORY_REGION(dest->name, sizeof(type) * nr * nc);           \
      memcpy((char*)dest->name, (const char*)src->name, sizeof(type) * nr * nc); \
    } else {                                                                     \
      dest->name = NULL;                                                         \
    }
    SIMDATA_ARENA_POINTERS
    #undef X

    // redefine XNV to be the same as X
    #undef XNV
    #define XNV X
  }

  #undef SIM_M
  #define SIM_M(n) n
  #undef SIM_D
  #define SIM_D(n) n

  // restore contact pointer
  dest->contact = dest->arena;

  // restore plugin_data
  if (plugin_data_size) {
    memcpy(dest->plugin_data, save_plugin_data, plugin_data_size);
    sim_free(save_plugin_data);
    save_plugin_data = NULL;
  }

  // copy plugin instances
  dest->nplugin = m->nplugin;
  for (int i = 0; i < m->nplugin; ++i) {
    const SIM_pPlugin* plugin = sim_plugin_getPluginAtSlot(m->plugin[i]);
    if (plugin->copy) {
      plugin->copy(dest, m, src, i);
    }
  }

  dest->threadpool = src->threadpool;

  return dest;
}


sim_data_t* sim_copyData(sim_data_t* dest, const sim_model_t* m, const sim_data_t* src) {
  return sim_copyDataInternal(dest, m, src, /*flg_all=*/1);
}


// clear data, set defaults
static void _resetData(const sim_model_t* m, sim_data_t* d, unsigned char debug_value) {
  // error early if history buffers cannot be initialized
  sim_scalar_t dt = m->opt.timestep;
  if (m->nhistory && dt <= 0) {
    SIM_ERROR("history buffers require positive timestep, got %g", dt);
  }

  //------------------------------ save plugin state and data
  sim_scalar_t* plugin_state;
  uintptr_t* plugindata;
  if (d->nplugin) {
    plugin_state = sim_malloc(sizeof(sim_scalar_t) * m->npluginstate);
    memcpy(plugin_state, d->plugin_state, sizeof(sim_scalar_t) * m->npluginstate);
    plugindata = sim_malloc(sizeof(uintptr_t) * m->nplugin);
    memcpy(plugindata, d->plugin_data, sizeof(uintptr_t) * m->nplugin);
  }

  //------------------------------ clear header

  // clear stack pointer
  if (!d->threadpool) {
    d->pstack = 0;
  }
  d->pbase = 0;

  // clear arena pointers
  d->parena = 0;

  // poison the entire arena+stack memory region when built with asan
#ifdef ADDRESS_SANITIZER
  ASAN_POISON_MEMORY_REGION(d->arena, d->narena);
#endif

#ifdef MEMORY_SANITIZER
  __msan_allocated_memory(d->arena, d->narena);
#endif

#define X(type, name, nr, nc) d->name = NULL;
  SIMDATA_ARENA_POINTERS
#undef X
  d->contact = d->arena;

  // clear memory utilization stats
  d->maxuse_stack = 0;
  memset(d->maxuse_threadstack, 0, SIM_MAXTHREAD*sizeof(sim_size_t));
  d->maxuse_arena = 0;
  d->maxuse_con = 0;
  d->maxuse_efc = 0;

  // clear solver diagnostics
  memset(d->warning, 0, SIM_NWARNING*sizeof(SIM_WarningStat));
  memset(d->timer, 0, SIM_NTIMER*sizeof(SIM_TimerStat));
  memset(d->solver, 0, SIM_NSOLVER*SIM_NISLAND*sizeof(SIM_SolverStat));
  sim_math_zeroInt(d->solver_niter, SIM_NISLAND);
  sim_math_zeroInt(d->solver_nnz, SIM_NISLAND);
  sim_math_zero(d->solver_fwdinv, 2);

  // clear variable sizes
  d->ncon = 0;
  d->ne = 0;
  d->nf = 0;
  d->nl = 0;
  d->nefc = 0;
  d->nJ = 0;
  d->nA = 0;
  d->nisland = 0;
  d->nidof = 0;

  // clear global properties
  d->time = 0;
  sim_math_zero(d->energy, 2);

  // clear lazy evaluation flags
  d->flg_energypos = 0;
  d->flg_energyvel = 0;
  d->flg_subtreevel = 0;
  d->flg_rnepost = 0;

  //------------------------------ clear buffer, set defaults

  // fill buffer with debug_value (normally 0)
#ifdef ADDRESS_SANITIZER
  {
    #define X(type, name, nr, nc) memset(d->name, (int)debug_value, sizeof(type)*(m->nr)*(nc));
    SIMDATA_POINTERS
    #undef X
  }
#else
  memset(d->buffer, (int)debug_value, d->nbuffer);
#endif

#ifdef MEMORY_SANITIZER
  // under MSAN, mark the entire buffer as uninitialized
  __msan_allocated_memory(d->buffer, d->nbuffer);
#endif

  // zero out user-settable state and input arrays (MSAN: mark as initialized)
  sim_math_zero(d->qpos, m->nq);
  sim_math_zero(d->qvel, m->nv);
  sim_math_zero(d->act, m->na);
  sim_math_zero(d->ctrl, m->nu);
  for (int i=0; i < m->neq; i++) d->eq_active[i] = m->eq_active0[i];
  sim_math_zero(d->qfrc_applied, m->nv);
  sim_math_zero(d->xfrc_applied, 6*m->nbody);
  sim_math_zero(d->qacc, m->nv);  // input to inverse dynamics
  sim_math_zero(d->qacc_warmstart, m->nv);
  sim_math_zero(d->act_dot, m->na);
  sim_math_zero(d->userdata, m->nuserdata);
  sim_math_zero(d->mocap_pos, 3*m->nmocap);
  sim_math_zero(d->mocap_quat, 4*m->nmocap);

  // initialize ctrl history buffers: timestamps at [-n*dt, ..., -dt]
  for (int i = 0; i < m->nu; i++) {
    int n = m->actuator_history[2*i];
    if (n > 0) {
      sim_scalar_t* buf = d->history + m->actuator_historyadr[i];
      buf[0] = 0;          // user slot
      buf[1] = n - 1;      // cursor: newest at logical index n-1
      sim_scalar_t* times = buf + 2;
      for (int j = 0; j < n; j++) {
        times[j] = -(n-j)*dt;
      }

      // clear values
      sim_scalar_t* values = buf + 2 + n;
      sim_math_zero(values, n);
    }
  }

  // initialize sensor history buffers
  for (int i = 0; i < m->nsensor; i++) {
    int n = m->sensor_history[2*i];
    if (n > 0) {
      int dim = m->sensor_dim[i];
      sim_scalar_t period = m->sensor_interval[2*i];
      sim_scalar_t phase = m->sensor_interval[2*i+1];
      sim_scalar_t* buf = d->history + m->sensor_historyadr[i];

      // user slot: last compute time (phase=0 means -period, i.e. first compute at t=0)
      buf[0] = (period > 0) ? (phase == 0 ? -period : phase) : -dt;
      buf[1] = n - 1;  // cursor: newest at logical index n-1

      sim_scalar_t* times = buf + 2;
      if (period > 0) {
        // samples spaced at period intervals, rounded up to dt grid
        sim_scalar_t t0 = (phase == 0) ? -period : phase;
        for (int j = 0; j < n; j++) {
          sim_scalar_t continuous_t = t0 - (n-1-j)*period;
          times[j] = sim_math_ceil(continuous_t / dt) * dt;
        }
      } else {
        // no period: timestamps at [-n*dt, ..., -dt]
        for (int j = 0; j < n; j++) {
          times[j] = -(n-j)*dt;
        }
      }

      // clear values
      sim_scalar_t* values = buf + 2 + n;
      sim_math_zero(values, n*dim);
    }
  }

  // zero out qM, special case because scattering from M skips simple body off-diagonals
  sim_math_zero(d->qM, m->nM);

  // copy qpos0 from model
  if (m->qpos0) {
    sim_math_copy(d->qpos, m->qpos0, m->nq);
  }

  static int kAwake = -(1+SIM_MINAWAKE);  // tree_asleep value for fully awake tree

  // set all trees to awake
  sim_math_fillInt(d->tree_asleep, kAwake, m->ntree);

  // sleep enabled: handle static bodies and trees marked as SIM_SLEEP_INIT
  if (SIM_ENABLED(SIM_ENBL_SLEEP)) {
    // count trees initialized as asleep
    int num_asleep_init = 0;
    for (int i=0; i < m->ntree; i++) {
      num_asleep_init += (m->tree_sleep_policy[i] == SIM_SLEEP_INIT);
    }

    // update sleep arrays, treat static bodies as awake
    sim_updateSleepInit(m, d, /*flg_staticawake*/ 1);

    // partial sim_fwdPosition, functions that update STATIC values
    if (!num_asleep_init) {
      sim_kinematics(m, d);
      sim_comPos(m, d);
      sim_camlight(m, d);
      sim_tendon(m, d);
    }

    // if any trees initialized as sleeping call entire sim_forward, put them to sleep
    else {
      sim_forward(m, d);

      // mark asleep-init trees as ready to sleep
      for (int i=0; i < m->ntree; i++) {
        int init = m->tree_sleep_policy[i] == SIM_SLEEP_INIT;
        d->tree_asleep[i] = init ? -1 : kAwake;
      }

      int nslept = sim_sleep(m, d);

      // raise error if any failed to sleep
      if (nslept != num_asleep_init) {
        // find root body of the first tree that could not be slept
        int root = -1;
        for (int i=0; i < m->ntree; i++) {
          if (m->tree_sleep_policy[i] == SIM_SLEEP_INIT && d->tree_asleep[i] < 0) {
            root = m->tree_bodyadr[i];
            break;
          }
        }

        // free all memory held by d just before aborting
        sim_deleteData(d);

        // raise error and abort
        const char* hasname = sim_id2name(m, SIM_OBJ_BODY, root);
        const char* name = hasname ? hasname : "";
        SIM_ERROR("%d trees were marked as sleep='init' but only %d could be slept.\n"
                "Body '%s' (id=%d) is the root of the first tree that could not be slept.",
                num_asleep_init, nslept, name, root);
      }

      // clear arrays to avoid MSAN errors upon mid-step wake
      sim_math_zero(d->qacc_smooth, m->nv);
      sim_math_zero(d->qfrc_smooth, m->nv);

      // clear arena
      sim_clearEfc(d);
    }
  }

  // update sleep arrays and counters
  sim_updateSleep(m, d);

  // set mocap_pos/quat = body_pos/quat for mocap bodies
  if (m->body_mocapid) {
    for (int i=0; i < m->nbody; i++) {
      int id = m->body_mocapid[i];
      if (id >= 0) {
        sim_math_copy_3(d->mocap_pos+3*id, m->body_pos+3*i);
        sim_math_copy4(d->mocap_quat+4*id, m->body_quat+4*i);
      }
    }
  } else {
    // set the mocap_quats to {1, 0, 0, 0}
    for (int i=0; i < m->nmocap; i++) {
      d->mocap_quat[4*i] = 1.0;
    }
  }

  // check consistency of sparse matrix representations
  checkDBSparse(m);

  // restore pluginstate and plugindata
  if (d->nplugin) {
    memcpy(d->plugin_state, plugin_state, sizeof(sim_scalar_t) * m->npluginstate);
    sim_free(plugin_state);
    memcpy(d->plugin_data, plugindata, sizeof(uintptr_t) * m->nplugin);
    sim_free(plugindata);

    // restore the plugin array back into d and reset the instances
    for (int i = 0; i < m->nplugin; ++i) {
      d->plugin[i] = m->plugin[i];
      const SIM_pPlugin* plugin = sim_plugin_getPluginAtSlot(m->plugin[i]);
      if (plugin->reset) {
        plugin->reset(m, &d->plugin_state[m->plugin_stateadr[i]],
                      (void*)(d->plugin_data[i]), i);
      }
    }
  }

  // copy signature from model
  d->signature = m->signature;
}


// clear data, set data->qpos = model->qpos0
void sim_resetData(const sim_model_t* m, sim_data_t* d) {
  _resetData(m, d, 0);
}


// clear data, set data->qpos = model->qpos0, fill with debug_value
void sim_resetDataDebug(const sim_model_t* m, sim_data_t* d, unsigned char debug_value) {
  _resetData(m, d, debug_value);
}


// Reset data. If 0 <= key < nkey, set fields from specified keyframe.
void sim_resetDataKeyframe(const sim_model_t* m, sim_data_t* d, int key) {
  _resetData(m, d, 0);

  // copy keyframe data if key is valid
  if (key >= 0 && key < m->nkey) {
    d->time = m->key_time[key];
    sim_math_copy(d->qpos, m->key_qpos+key*m->nq, m->nq);
    sim_math_copy(d->qvel, m->key_qvel+key*m->nv, m->nv);
    sim_math_copy(d->act,  m->key_act+ key*m->na, m->na);
    sim_math_copy(d->mocap_pos,  m->key_mpos+key*3*m->nmocap, 3*m->nmocap);
    sim_math_copy(d->mocap_quat, m->key_mquat+key*4*m->nmocap, 4*m->nmocap);
    sim_math_copy(d->ctrl, m->key_ctrl+key*m->nu, m->nu);
  }
}


// de-allocate sim_data_t
void sim_deleteData(sim_data_t* d) {
  if (d) {
    freeDataBuffers(d);
    sim_free(d);
  }
}


// number of position and velocity coordinates for each joint type
const int nPOS[4] = {7, 4, 1, 1};
const int nVEL[4] = {6, 3, 1, 1};

static int sensorSize(SIM_tSensor sensor_type, int sensor_dim) {
  switch (sensor_type) {
  case SIM_SENS_TOUCH:
  case SIM_SENS_RANGEFINDER:
  case SIM_SENS_JOINTPOS:
  case SIM_SENS_JOINTVEL:
  case SIM_SENS_TENDONPOS:
  case SIM_SENS_TENDONVEL:
  case SIM_SENS_ACTUATORPOS:
  case SIM_SENS_ACTUATORVEL:
  case SIM_SENS_ACTUATORFRC:
  case SIM_SENS_JOINTACTFRC:
  case SIM_SENS_TENDONACTFRC:
  case SIM_SENS_JOINTLIMITPOS:
  case SIM_SENS_JOINTLIMITVEL:
  case SIM_SENS_JOINTLIMITFRC:
  case SIM_SENS_TENDONLIMITPOS:
  case SIM_SENS_TENDONLIMITVEL:
  case SIM_SENS_TENDONLIMITFRC:
  case SIM_SENS_GEOMDIST:
  case SIM_SENS_INSIDESITE:
  case SIM_SENS_E_POTENTIAL:
  case SIM_SENS_E_KINETIC:
  case SIM_SENS_CLOCK:
    return 1;

  case SIM_SENS_CAMPROJECTION:
    return 2;

  case SIM_SENS_ACCELEROMETER:
  case SIM_SENS_VELOCIMETER:
  case SIM_SENS_GYRO:
  case SIM_SENS_FORCE:
  case SIM_SENS_TORQUE:
  case SIM_SENS_MAGNETOMETER:
  case SIM_SENS_BALLANGVEL:
  case SIM_SENS_FRAMEPOS:
  case SIM_SENS_FRAMEXAXIS:
  case SIM_SENS_FRAMEYAXIS:
  case SIM_SENS_FRAMEZAXIS:
  case SIM_SENS_FRAMELINVEL:
  case SIM_SENS_FRAMEANGVEL:
  case SIM_SENS_FRAMELINACC:
  case SIM_SENS_FRAMEANGACC:
  case SIM_SENS_SUBTREECOM:
  case SIM_SENS_SUBTREELINVEL:
  case SIM_SENS_SUBTREEANGMOM:
  case SIM_SENS_GEOMNORMAL:
    return 3;

  case SIM_SENS_GEOMFROMTO:
    return 6;

  case SIM_SENS_BALLQUAT:
  case SIM_SENS_FRAMEQUAT:
    return 4;

  case SIM_SENS_CONTACT:
  case SIM_SENS_TACTILE:
  case SIM_SENS_USER:
    return sensor_dim;

  case SIM_SENS_PLUGIN:
    return -1;

    // don't use a 'default' case, so compiler warns about missing values
  }
  return -1;
}


// returns the number of objects of the given type
//   -1: SIM_OBJ_UNKNOWN
//   -2: invalid objtype
static int numObjects(const sim_model_t* m, sim_obj_t objtype) {
  switch (objtype) {
  case SIM_OBJ_DEFAULT:
  case SIM_OBJ_FRAME:
  case SIM_OBJ_UNKNOWN:
  case SIM_OBJ_MODEL:
    return -1;
  case SIM_OBJ_BODY:
  case SIM_OBJ_XBODY:
    return m->nbody;
  case SIM_OBJ_JOINT:
    return m->njnt;
  case SIM_OBJ_DOF:
    return m->nv;
  case SIM_OBJ_GEOM:
    return m->ngeom;
  case SIM_OBJ_SITE:
    return m->nsite;
  case SIM_OBJ_CAMERA:
    return m->ncam;
  case SIM_OBJ_LIGHT:
    return m->nlight;
  case SIM_OBJ_FLEX:
    return m->nflex;
  case SIM_OBJ_MESH:
    return m->nmesh;
  case SIM_OBJ_SKIN:
    return m->nskin;
  case SIM_OBJ_HFIELD:
    return m->nhfield;
  case SIM_OBJ_TEXTURE:
    return m->ntex;
  case SIM_OBJ_MATERIAL:
    return m->nmat;
  case SIM_OBJ_PAIR:
    return m->npair;
  case SIM_OBJ_EXCLUDE:
    return m->nexclude;
  case SIM_OBJ_EQUALITY:
    return m->neq;
  case SIM_OBJ_TENDON:
    return m->ntendon;
  case SIM_OBJ_ACTUATOR:
    return m->nu;
  case SIM_OBJ_SENSOR:
    return m->nsensor;
  case SIM_OBJ_NUMERIC:
    return m->nnumeric;
  case SIM_OBJ_TEXT:
    return m->ntext;
  case SIM_OBJ_TUPLE:
    return m->ntuple;
  case SIM_OBJ_KEY:
    return m->nkey;
  case SIM_OBJ_PLUGIN:
    return m->nplugin;
  case SIM_NOBJECT:
    return -2;
  }
  return -2;
}


// validate reference fields in a model; return null if valid, error message otherwise
const char* sim_validateReferences(const sim_model_t* m) {
  // for each field in sim_model_t that refers to another field, call X with:
  //   adrarray: array containing the references
  //   nadrs:    number of elements in refarray
  //   ntarget:  number of elements in array where references are pointing
  //   numarray: if refarray is an adr array, numarray is the corresponding num array, otherwise 0
#define SIMMODEL_REFERENCES                                                       \
  X(body_parentid,      nbody,          nbody         , 0                      ) \
  X(body_rootid,        nbody,          nbody         , 0                      ) \
  X(body_weldid,        nbody,          nbody         , 0                      ) \
  X(body_mocapid,       nbody,          nmocap        , 0                      ) \
  X(body_jntadr,        nbody,          njnt          , m->body_jntnum         ) \
  X(body_dofadr,        nbody,          nv            , m->body_dofnum         ) \
  X(body_geomadr,       nbody,          ngeom         , m->body_geomnum        ) \
  X(body_bvhadr,        nbody,          nbvh          , m->body_bvhnum         ) \
  X(body_plugin,        nbody,          nplugin       , 0                      ) \
  X(jnt_qposadr,        njnt,           nq            , 0                      ) \
  X(jnt_dofadr,         njnt,           nv            , 0                      ) \
  X(jnt_bodyid,         njnt,           nbody         , 0                      ) \
  X(dof_bodyid,         nv,             nbody         , 0                      ) \
  X(dof_jntid,          nv,             njnt          , 0                      ) \
  X(dof_parentid,       nv,             nv            , 0                      ) \
  X(dof_Madr,           nv,             nM            , 0                      ) \
  X(tree_bodyadr,       ntree,          nbody         , m->tree_bodynum        ) \
  X(tree_dofadr,        ntree,          nv            , m->tree_dofnum         ) \
  X(geom_bodyid,        ngeom,          nbody         , 0                      ) \
  X(geom_matid,         ngeom,          nmat          , 0                      ) \
  X(site_bodyid,        nsite,          nbody         , 0                      ) \
  X(site_matid,         nsite,          nmat          , 0                      ) \
  X(cam_bodyid,         ncam,           nbody         , 0                      ) \
  X(cam_targetbodyid,   ncam,           nbody         , 0                      ) \
  X(light_bodyid,       nlight,         nbody         , 0                      ) \
  X(light_targetbodyid, nlight,         nbody         , 0                      ) \
  X(mesh_vertadr,       nmesh,          nmeshvert     , m->mesh_vertnum        ) \
  X(mesh_normaladr,     nmesh,          nmeshnormal   , m->mesh_normalnum      ) \
  X(mesh_texcoordadr,   nmesh,          nmeshtexcoord , m->mesh_texcoordnum    ) \
  X(mesh_faceadr,       nmesh,          nmeshface     , m->mesh_facenum        ) \
  X(mesh_bvhadr,        nmesh,          nbvh          , m->mesh_bvhnum         ) \
  X(mesh_graphadr,      nmesh,          nmeshgraph    , 0                      ) \
  X(mesh_polyadr,       nmesh,          nmeshpoly     , m->mesh_polynum        ) \
  X(mesh_polyvertadr,   nmeshpoly,      nmeshpolyvert , m->mesh_polyvertnum    ) \
  X(mesh_polymapadr,    nmeshvert,      nmeshpolymap  , m->mesh_polymapnum     ) \
  X(flex_vertadr,       nflex,          nflexvert     , m->flex_vertnum        ) \
  X(flex_edgeadr,       nflex,          nflexedge     , m->flex_edgenum        ) \
  X(flex_elemadr,       nflex,          nflexelem     , m->flex_elemnum        ) \
  X(flex_evpairadr,     nflex,          nflexevpair   , m->flex_evpairnum      ) \
  X(flex_texcoordadr,   nflex,          nflextexcoord , 0                      ) \
  X(flex_elemdataadr,   nflex,          nflexelemdata , 0                      ) \
  X(flex_elemedgeadr,   nflex,          nflexelemedge , 0                      ) \
  X(flex_shelldataadr,  nflex,          nflexshelldata, 0                      ) \
  X(flex_edge,          nflexedge*2,    nflexvert     , 0                      ) \
  X(flex_elem,          nflexelemdata,  nflexvert     , 0                      ) \
  X(flex_elemedge,      nflexelemedge,  nflexedge     , 0                      ) \
  X(flex_shell,         nflexshelldata, nflexvert     , 0                      ) \
  X(flex_bvhadr,        nflex,          nbvh          , m->flex_bvhnum         ) \
  X(skin_matid,         nskin,          nmat          , 0                      ) \
  X(skin_vertadr,       nskin,          nskinvert     , m->skin_vertnum        ) \
  X(skin_texcoordadr,   nskin,          nskintexvert  , 0                      ) \
  X(skin_faceadr,       nskin,          nskinface     , m->skin_facenum        ) \
  X(skin_boneadr,       nskin,          nskinbone     , m->skin_bonenum        ) \
  X(skin_bonevertadr,   nskinbone,      nskinbonevert , m->skin_bonevertnum    ) \
  X(skin_bonebodyid,    nskinbone,      nbody         , 0                      ) \
  X(skin_bonevertid,    nskinbonevert,  nskinvert     , 0                      ) \
  X(pair_geom1,         npair,          ngeom         , 0                      ) \
  X(pair_geom2,         npair,          ngeom         , 0                      ) \
  X(actuator_plugin,    nu,             nplugin       , 0                      ) \
  X(actuator_actadr,    nu,             na            , m->actuator_actnum     ) \
  X(sensor_plugin,      nsensor,        nplugin       , 0                      ) \
  X(plugin_stateadr,    nplugin,        npluginstate  , m->plugin_statenum     ) \
  X(plugin_attradr,     nplugin,        npluginattr   , 0                      ) \
  X(tendon_adr,         ntendon,        nwrap         , m->tendon_num          ) \
  X(tendon_matid,       ntendon,        nmat          , 0                      ) \
  X(tendon_treeid,      ntendon*2,      ntree         , 0                      ) \
  X(numeric_adr,        nnumeric,       nnumericdata  , m->numeric_size        ) \
  X(text_adr,           ntext,          ntextdata     , m->text_size           ) \
  X(tuple_adr,          ntuple,         ntupledata    , m->tuple_size          ) \
  X(name_bodyadr,       nbody,          nnames        , 0                      ) \
  X(name_jntadr,        njnt,           nnames        , 0                      ) \
  X(name_geomadr,       ngeom,          nnames        , 0                      ) \
  X(name_siteadr,       nsite,          nnames        , 0                      ) \
  X(name_camadr,        ncam,           nnames        , 0                      ) \
  X(name_lightadr,      nlight,         nnames        , 0                      ) \
  X(name_meshadr,       nmesh,          nnames        , 0                      ) \
  X(name_skinadr,       nskin,          nnames        , 0                      ) \
  X(name_hfieldadr,     nhfield,        nnames        , 0                      ) \
  X(name_texadr,        ntex,           nnames        , 0                      ) \
  X(name_matadr,        nmat,           nnames        , 0                      ) \
  X(name_pairadr,       npair,          nnames        , 0                      ) \
  X(name_excludeadr,    nexclude,       nnames        , 0                      ) \
  X(name_eqadr,         neq,            nnames        , 0                      ) \
  X(name_tendonadr,     ntendon,        nnames        , 0                      ) \
  X(name_actuatoradr,   nu,             nnames        , 0                      ) \
  X(name_sensoradr,     nsensor,        nnames        , 0                      ) \
  X(name_numericadr,    nnumeric,       nnames        , 0                      ) \
  X(name_textadr,       ntext,          nnames        , 0                      ) \
  X(name_tupleadr,      ntuple,         nnames        , 0                      ) \
  X(name_keyadr,        nkey,           nnames        , 0                      ) \
  X(hfield_pathadr,     nhfield,        npaths        , 0                      ) \
  X(mesh_pathadr,       nmesh,          npaths        , 0                      ) \
  X(skin_pathadr,       nskin,          npaths        , 0                      ) \
  X(tex_pathadr,        ntex,           npaths        , 0                      )

  #define X(adrarray, nadrs, ntarget, numarray) {             \
    int *nums = (numarray);                                   \
    for (int i=0; i<m->nadrs; i++) {                          \
      int adrsmin = m->adrarray[i];                           \
      int num = (nums ? nums[i] : 1);                         \
      if (num < 0) {                                          \
        return "Invalid model: " #numarray " is negative.";   \
      }                                                       \
      if (num > MAX_ARRAY_SIZE) {                             \
        return "Invalid model: " #numarray " is too large.";  \
      }                                                       \
      int adrsmax = m->adrarray[i] + num;                     \
      if (adrsmax > m->ntarget || adrsmin < -1) {             \
        return "Invalid model: " #adrarray " out of bounds."; \
      }                                                       \
    }                                                         \
  }

  SIMMODEL_REFERENCES;
  #undef X
#undef SIMMODEL_REFERENCES

  // special logic that doesn't fit in the macro:
  for (int i=0; i < m->nbody; i++) {
    if (i > 0 && m->body_parentid[i] >= i) {
      return "Invalid model: bad body_parentid.";
    }
    if (m->body_rootid[i] > i) {
      return "Invalid model: bad body_rootid.";
    }
    if (m->body_weldid[i] > i) {
      return "Invalid model: bad body_weldid.";
    }
  }
  for (int i=0; i < m->njnt; i++) {
    if (m->jnt_type[i] >= 4 || m->jnt_type[i] < 0) {
      return "Invalid model: jnt_type out of bounds.";
    }
    int jnt_qposadr = m->jnt_qposadr[i] + nPOS[m->jnt_type[i]];
    if (jnt_qposadr > m->nq || m->jnt_qposadr[i] < 0) {
      return "Invalid model: jnt_qposadr out of bounds.";
    }
    int jnt_dofadr = m->jnt_dofadr[i] + nVEL[m->jnt_type[i]];
    if (jnt_dofadr > m->nv || m->jnt_dofadr[i] < 0) {
      return "Invalid model: jnt_dofadr out of bounds.";
    }
  }
  for (int i=0; i < m->nv; i++) {
    if (m->dof_parentid[i] >= i) {
      return "Invalid model: bad dof_parentid.";
    }
  }
  for (int i=0; i < m->ngeom; i++) {
    if (m->geom_condim[i] > 6 || m->geom_condim[i] < 0) {
      return "Invalid model: geom_condim out of bounds.";
    }
    if (m->geom_type[i] == SIM_GEOM_HFIELD) {
      if (m->geom_dataid[i] >= m->nhfield || m->geom_dataid[i] < -1) {
        return "Invalid model: geom_dataid out of bounds.";
      }
    } else if ((m->geom_type[i] == SIM_GEOM_MESH) || (m->geom_type[i] == SIM_GEOM_SDF)) {
      if (m->geom_dataid[i] >= m->nmesh || m->geom_dataid[i] < -1) {
        return "Invalid model: geom_dataid out of bounds.";
      }
    }
  }
  for (int i=0; i < m->nhfield; i++) {
    sim_size_t hfield_adr = m->hfield_adr[i] + ((sim_size_t) m->hfield_nrow[i]) * m->hfield_ncol[i];
    if (hfield_adr > m->nhfielddata || m->hfield_adr[i] < 0) {
      return "Invalid model: hfield_adr out of bounds.";
    }
  }
  for (int i=0; i < m->ntex; i++) {
    sim_size_t nbytes = ((sim_size_t) m->tex_nchannel[i]) * m->tex_height[i] * m->tex_width[i];
    if (m->tex_adr[i] + nbytes > m->ntexdata || m->tex_adr[i] < 0) {
      return "Invalid model: tex_adr out of bounds.";
    }
  }
  for (int i=0; i < m->npair; i++) {
    int pair_body1 = (m->pair_signature[i] & 0xFFFF);
    if (pair_body1 >= m->nbody || pair_body1 < 0) {
      return "Invalid model: pair_body1 out of bounds.";
    }
    int pair_body2 = (m->pair_signature[i] >> 16);
    if (pair_body2 >= m->nbody || pair_body2 < 0) {
      return "Invalid model: pair_body2 out of bounds.";
    }
  }
  for (int i=0; i < m->neq; i++) {
    int obj1id = m->eq_obj1id[i];
    int obj2id = m->eq_obj2id[i];
    int objtype = m->eq_objtype[i];
    switch ((SIM_tEq) m->eq_type[i]) {
    case SIM_EQ_JOINT:
      if (obj1id >= m->njnt || obj1id < 0) {
        return "Invalid model: eq_obj1id out of bounds.";
      }
      // -1 is the value used if second object is omitted.
      if (obj2id >= m->njnt || obj2id < -1) {
        return "Invalid model: eq_obj2id out of bounds.";
      }
      break;

    case SIM_EQ_TENDON:
      if (obj1id >= m->ntendon || obj1id < 0) {
        return "Invalid model: eq_obj1id out of bounds.";
      }
      // -1 is the value used if second object is omitted.
      if (obj2id >= m->ntendon || obj2id < -1) {
        return "Invalid model: eq_obj2id out of bounds.";
      }
      break;

    case SIM_EQ_WELD:
    case SIM_EQ_CONNECT:
      if (objtype == SIM_OBJ_BODY) {
        if (obj1id >= m->nbody || obj1id < 0) {
          return "Invalid model: eq_obj1id out of bounds.";
        }
        if (obj2id >= m->nbody || obj2id < 0) {
          return "Invalid model: eq_obj2id out of bounds.";
        }
      } else if (objtype == SIM_OBJ_SITE) {
        if (obj1id >= m->nsite || obj1id < 0) {
          return "Invalid model: eq_obj1id out of bounds.";
        }
        if (obj2id >= m->nsite || obj2id < 0) {
          return "Invalid model: eq_obj2id out of bounds.";
        }
      } else {
        return "Invalid model: eq_objtype is not body or site.";
      }
      break;

    case SIM_EQ_FLEX:
    case SIM_EQ_FLEXVERT:
      if (obj1id >= m->nflex || obj1id < 0) {
        return "Invalid model: eq_obj1id out of bounds.";
      }

      // -1 is the value used if second object is omitted
      if (obj2id != -1) {
        return "Invalid model: eq_obj2id must be -1.";
      }
      break;

    default:
      // might occur in case of the now-removed distance equality constraint
      SIM_ERROR("unknown equality constraint type.");
    }
  }
  for (int i=0; i < m->nwrap; i++) {
    int wrap_objid = m->wrap_objid[i];
    switch ((SIM_tWrap) m->wrap_type[i]) {
    case SIM_WRAP_NONE:
    case SIM_WRAP_PULLEY:
      // wrap_objid not used.
      break;
    case SIM_WRAP_JOINT:
      if (wrap_objid >= m->njnt || wrap_objid < 0) {
        return "Invalid model: wrap_objid out of bounds.";
      }
      break;
    case SIM_WRAP_SITE:
      if (wrap_objid >= m->nsite || wrap_objid < 0) {
        return "Invalid model: wrap_objid out of bounds.";
      }
      break;
    case SIM_WRAP_SPHERE:
    case SIM_WRAP_CYLINDER:
      if (wrap_objid >= m->ngeom || wrap_objid < 0) {
        return "Invalid model: wrap_objid out of bounds.";
      }
      break;
    }
  }
  for (int i=0; i < m->nu; i++) {
    int actuator_trntype = m->actuator_trntype[i];
    int id = m->actuator_trnid[2*i];
    int idslider = m->actuator_trnid[2*i+1];
    switch ((SIM_tTrn) actuator_trntype) {
    case SIM_TRN_JOINT:
    case SIM_TRN_JOINTINPARENT:
      if (id < 0 || id >= m->njnt) {
        return "Invalid model: actuator_trnid out of bounds.";
      }
      break;
    case SIM_TRN_TENDON:
      if (id < 0 || id >= m->ntendon) {
        return "Invalid model: actuator_trnid out of bounds.";
      }
      break;
    case SIM_TRN_SITE:
      if (id < 0 || id >= m->nsite) {
        return "Invalid model: actuator_trnid out of bounds.";
      }
      break;
    case SIM_TRN_SLIDERCRANK:
      if (id < 0 || id >= m->nsite) {
        return "Invalid model: actuator_trnid out of bounds.";
      }
      if (idslider < 0 || idslider >= m->nsite) {
        return "Invalid model: actuator_trnid out of bounds.";
      }
      break;
    case SIM_TRN_BODY:
      if (id < 0 || id >= m->nbody) {
        return "Invalid model: actuator_trnid out of bounds.";
      }
      break;
    case SIM_TRN_UNDEFINED:
      // actuator_trnid not used.
      break;
    }
  }
  for (int i=0; i < m->nsensor; i++) {
    SIM_tSensor sensor_type = m->sensor_type[i];
    int sensor_size;
    if (sensor_type == SIM_SENS_PLUGIN) {
      const SIM_pPlugin* plugin = sim_plugin_getPluginAtSlot(m->plugin[m->sensor_plugin[i]]);
      if (!plugin->nsensordata) {
        SIM_ERROR("`nsensordata` is a null function pointer for plugin at slot %d",
                m->plugin[m->sensor_plugin[i]]);
      }
      sensor_size = plugin->nsensordata(m, m->sensor_plugin[i], i);
    } else {
      sensor_size = sensorSize(sensor_type, m->sensor_dim[i]);
    }
    if (sensor_size < 0) {
      return "Invalid model: Bad sensor_type.";
    }
    int sensor_adr = m->sensor_adr[i];
    if (sensor_adr < 0 || sensor_adr + sensor_size > m->nsensordata) {
      return "Invalid model: sensor_adr out of bounds.";
    }
    int nobj = numObjects(m, m->sensor_objtype[i]);
    if (nobj == -2) {
      return "Invalid model: invalid sensor_objtype";
    }
    if (nobj != -1 && (m->sensor_objid[i] < 0 || m->sensor_objid[i] >= nobj)) {
      return "Invalid model: invalid sensor_objid";
    }
    nobj = numObjects(m, m->sensor_reftype[i]);
    if (nobj == -2) {
      return "Invalid model: invalid sensor_reftype";
    }
    if (nobj != -1 && (m->sensor_refid[i] < -1 || m->sensor_refid[i] >= nobj)) {
      return "Invalid model: invalid sensor_refid";
    }
    if (sensor_type == SIM_SENS_TACTILE) {
      int obj_id = m->sensor_objid[i];
      int parent_body = m->geom_bodyid[obj_id];
      int collision_geoms = 0;
      for (int b = 0; b < m->body_geomnum[parent_body]; ++b) {
        int geom_id = m->body_geomadr[parent_body]+b;
        if (m->geom_contype[geom_id] || m->geom_conaffinity[geom_id]) {
          collision_geoms++;
        }
      }
      if (collision_geoms == 0) {
        return "Touch sensor requires a body with at least one collision geom";
      }
    }
  }
  for (int i=0; i < m->nexclude; i++) {
    int exclude_body1 = (m->exclude_signature[i] & 0xFFFF);
    if (exclude_body1 >= m->nbody || exclude_body1 < 0) {
      return "Invalid model: exclude_body1 out of bounds.";
    }
    int exclude_body2 = (m->exclude_signature[i] >> 16);
    if (exclude_body2 >= m->nbody || exclude_body2 < 0) {
      return "Invalid model: exclude_body2 out of bounds.";
    }
  }
  for (int i=0; i < m->ntuple; i++) {
    for (int j=0; j < m->tuple_size[i]; j++) {
      int adr = m->tuple_adr[i] + j;
      int nobj = numObjects(m, m->tuple_objtype[adr]);
      if (nobj == -2) {
        return "Invalid model: invalid tuple_objtype";
      }
      if (nobj != -1 && (m->tuple_objid[adr] < 0 || m->tuple_objid[adr] >= nobj)) {
        return "Invalid model: invalid tuple_objid";
      }
    }
  }

  return NULL;
}
