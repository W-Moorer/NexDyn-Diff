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

#include "engine/engine_name.h"

#include <stdint.h>
#include <string.h>

#include <simcore/SIM_model.h>
#include "engine/engine_crossplatform.h"
#include "engine/engine_io.h"



//-------------------------- name functions --------------------------------------------------------
// get number of objects and name addresses for given object type
static int _getnumadr(const sim_model_t* m, sim_obj_t type, int** padr, int* mapadr) {
  int num = -1;
  // map address starts at the end, subtract with explicit switch fallthrough below
  *mapadr = m->nnames_map;

  // get address list and size for object type
  switch (type) {
  case SIM_OBJ_BODY:
  case SIM_OBJ_XBODY:
    *mapadr -= SIM_LOAD_MULTIPLE*m->nbody;
    *padr = m->name_bodyadr;
    num = m->nbody;
    SIM_FALLTHROUGH;

  case SIM_OBJ_JOINT:
    *mapadr -= SIM_LOAD_MULTIPLE*m->njnt;
    if (num < 0) {
      *padr = m->name_jntadr;
      num = m->njnt;
    }
    SIM_FALLTHROUGH;

  case SIM_OBJ_GEOM:
    *mapadr -= SIM_LOAD_MULTIPLE*m->ngeom;
    if (num < 0) {
      *padr = m->name_geomadr;
      num = m->ngeom;
    }
    SIM_FALLTHROUGH;

  case SIM_OBJ_SITE:
    *mapadr -= SIM_LOAD_MULTIPLE*m->nsite;
    if (num < 0) {
      *padr = m->name_siteadr;
      num = m->nsite;
    }
    SIM_FALLTHROUGH;

  case SIM_OBJ_CAMERA:
    *mapadr -= SIM_LOAD_MULTIPLE*m->ncam;
    if (num < 0) {
      *padr = m->name_camadr;
      num = m->ncam;
    }
    SIM_FALLTHROUGH;

  case SIM_OBJ_LIGHT:
    *mapadr -= SIM_LOAD_MULTIPLE*m->nlight;
    if (num < 0) {
      *padr = m->name_lightadr;
      num = m->nlight;
    }
    SIM_FALLTHROUGH;

  case SIM_OBJ_FLEX:
    *mapadr -= SIM_LOAD_MULTIPLE*m->nflex;
    if (num < 0) {
      *padr = m->name_flexadr;
      num =  m->nflex;
    }
    SIM_FALLTHROUGH;

  case SIM_OBJ_MESH:
    *mapadr -= SIM_LOAD_MULTIPLE*m->nmesh;
    if (num < 0) {
      *padr = m->name_meshadr;
      num =  m->nmesh;
    }
    SIM_FALLTHROUGH;

  case SIM_OBJ_SKIN:
    *mapadr -= SIM_LOAD_MULTIPLE*m->nskin;
    if (num < 0) {
      *padr = m->name_skinadr;
      num = m->nskin;
    }
    SIM_FALLTHROUGH;

  case SIM_OBJ_HFIELD:
    *mapadr -= SIM_LOAD_MULTIPLE*m->nhfield;
    if (num < 0) {
      *padr = m->name_hfieldadr;
      num = m->nhfield;
    }
    SIM_FALLTHROUGH;

  case SIM_OBJ_TEXTURE:
    *mapadr -= SIM_LOAD_MULTIPLE*m->ntex;
    if (num < 0) {
      *padr = m->name_texadr;
      num = m->ntex;
    }
    SIM_FALLTHROUGH;

  case SIM_OBJ_MATERIAL:
    *mapadr -= SIM_LOAD_MULTIPLE*m->nmat;
    if (num < 0) {
      *padr = m->name_matadr;
      num = m->nmat;
    }
    SIM_FALLTHROUGH;

  case SIM_OBJ_PAIR:
    *mapadr -= SIM_LOAD_MULTIPLE*m->npair;
    if (num < 0) {
      *padr = m->name_pairadr;
      num = m->npair;
    }
    SIM_FALLTHROUGH;

  case SIM_OBJ_EXCLUDE:
    *mapadr -= SIM_LOAD_MULTIPLE*m->nexclude;
    if (num < 0) {
      *padr = m->name_excludeadr;
      num = m->nexclude;
    }
    SIM_FALLTHROUGH;

  case SIM_OBJ_EQUALITY:
    *mapadr -= SIM_LOAD_MULTIPLE*m->neq;
    if (num < 0) {
      *padr = m->name_eqadr;
      num = m->neq;
    }
    SIM_FALLTHROUGH;

  case SIM_OBJ_TENDON:
    *mapadr -= SIM_LOAD_MULTIPLE*m->ntendon;
    if (num < 0) {
      *padr = m->name_tendonadr;
      num = m->ntendon;
    }
    SIM_FALLTHROUGH;

  case SIM_OBJ_ACTUATOR:
    *mapadr -= SIM_LOAD_MULTIPLE*m->nu;
    if (num < 0) {
      *padr = m->name_actuatoradr;
      num = m->nu;
    }
    SIM_FALLTHROUGH;

  case SIM_OBJ_SENSOR:
    *mapadr -= SIM_LOAD_MULTIPLE*m->nsensor;
    if (num < 0) {
      *padr = m->name_sensoradr;
      num = m->nsensor;
    }
    SIM_FALLTHROUGH;

  case SIM_OBJ_NUMERIC:
    *mapadr -= SIM_LOAD_MULTIPLE*m->nnumeric;
    if (num < 0) {
      *padr = m->name_numericadr;
      num = m->nnumeric;
    }
    SIM_FALLTHROUGH;

  case SIM_OBJ_TEXT:
    *mapadr -= SIM_LOAD_MULTIPLE*m->ntext;
    if (num < 0) {
      *padr = m->name_textadr;
      num = m->ntext;
    }
    SIM_FALLTHROUGH;

  case SIM_OBJ_TUPLE:
    *mapadr -= SIM_LOAD_MULTIPLE*m->ntuple;
    if (num < 0) {
      *padr = m->name_tupleadr;
      num = m->ntuple;
    }
    SIM_FALLTHROUGH;

  case SIM_OBJ_KEY:
    *mapadr -= SIM_LOAD_MULTIPLE*m->nkey;
    if (num < 0) {
      *padr = m->name_keyadr;
      num = m->nkey;
    }
    SIM_FALLTHROUGH;

  case SIM_OBJ_PLUGIN:
    *mapadr -= SIM_LOAD_MULTIPLE*m->nplugin;
    if (num < 0) {
      *padr = m->name_pluginadr;
      num = m->nplugin;
    }
    SIM_FALLTHROUGH;

  default:
    if (num < 0) {
      *padr = 0;
      num = 0;
    }
  }

  return num;
}

// get string hash, see http://www.cse.yorku.ca/~oz/hash.html
uint64_t sim_hashString(const char* s, uint64_t n) {
  uint64_t h = 5381;
  int c;
  while ((c = *s++)) {
    h = ((h << 5) + h) ^ c;
  }
  return h % n;
}

// get id of object with the specified sim_obj_t type and name,
// returns -1 if id not found
int sim_name2id(const sim_model_t* m, int type, const char* name) {
  int mapadr;
  int* adr = 0;

  // get number of objects and name addresses
  int num = SIM_LOAD_MULTIPLE*_getnumadr(m, type, &adr, &mapadr);

  // search
  if (num) {    // look up at hash address
    uint64_t hash = sim_hashString(name, num);
    uint64_t i = hash;

    do {
      int j = m->names_map[mapadr + i];
      if (j < 0) {
        return -1;
      }

      if (!strncmp(name, m->names+adr[j], m->nnames-adr[j])) {
        return j;
      }
      if ((++i) == num)i = 0;
    } while (i != hash);
  }
  return -1;
}


// get name of object with the specified sim_obj_t type and id,
// returns NULL if name not found
const char* sim_id2name(const sim_model_t* m, int type, int id) {
  int mapadr;
  int* adr = 0;

  // get number of objects and name addresses
  int num = _getnumadr(m, type, &adr, &mapadr);

  // id is in [0, num) and the found name is not the empty string "\0"
  if (id >= 0 && id < num && m->names[adr[id]]) {
    return m->names+adr[id];
  }

  return NULL;
}
