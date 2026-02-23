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

#include "user/user_composite.h"

#include <cmath>
#include <cstring>
#include <string>
#include <vector>

#include <simcore/SIM_model.h>
#include <simcore/SIM_spec.h>
#include <simcore/SIM_tnum.h>
#include "cc/array_safety.h"
#include "engine/engine_util_blas.h"
#include "engine/engine_util_errmem.h"
#include "engine/engine_util_misc.h"
#include "user/user_model.h"
#include "user/user_objects.h"
#include "user/user_api.h"
#include "user/user_util.h"

namespace {

namespace sim_math = ::simcore::util;

}  // namespace

// strncpy with 0, return false
static bool comperr(char* error, const char* msg, int error_sz) {
  sim_math_strncpy(error, msg, error_sz);
  return false;
}



// constructor
SIM_CComposite::SIM_CComposite(void) {
  // common properties
  prefix.clear();
  type = SIM_COMPTYPE_PARTICLE;
  count[0] = count[1] = count[2] = 1;
  sim_math_internal_set_vec(offset, 0, 0, 0);
  sim_math_internal_set_vec(quat, 1, 0, 0, 0);
  frame = nullptr;

  // plugin variables
  sim_spec_defaultPlugin(&plugin);
  plugin_name = "";
  plugin_instance_name = "";
  plugin.plugin_name = (sim_string_t*)&plugin_name;
  plugin.name = (sim_string_t*)&plugin_instance_name;

  // cable
  curve[0] = curve[1] = curve[2] = SIM_COMPSHAPE_ZERO;
  sim_math_internal_set_vec(size, 1, 0, 0);
  initial = "ball";

  // skin
  skin = false;
  skintexcoord = false;
  skinmaterial.clear();
  sim_math_internal_set_vec(skinrgba, 1, 1, 1, 1);
  skininflate = 0;
  skinsubgrid = 0;
  skingroup = 0;

  // clear add flags
  for (int i=0; i < SIM_NCOMPKINDS; i++) {
    add[i] = false;
  }

  // clear internal
  dim = 0;
}



// create the array of default joint options, append new elements only for particles type
bool SIM_CComposite::AddDefaultJoint(char* error, int error_sz) {
  for (int i=0; i < SIM_NCOMPKINDS; i++) {
    if (!defjoint[(SIM_tCompKind)i].empty() && type != SIM_COMPTYPE_PARTICLE) {
      comperr(error, "Only particles are allowed to have multiple joints", error_sz);
      return false;
    } else {
      sim_builder_default_t jnt;
      jnt.spec.joint->group = 3;
      defjoint[(SIM_tCompKind)i].push_back(jnt);
    }
  }
  return true;
}



// set defaults, after reading top-level info and skin
void SIM_CComposite::SetDefault(void) {
  // set all default groups to 3
  for (int i=0; i < SIM_NCOMPKINDS; i++) {
    def[i].spec.geom->group = 3;
    def[i].spec.site->group = 3;
    def[i].spec.tendon->group = 3;
  }

  // set default joint
  AddDefaultJoint();

  // set default geom and tendon group to 0 if needed to be visible
  if (!skin || type == SIM_COMPTYPE_CABLE) {
    for (int i=0; i < SIM_NCOMPKINDS; i++) {
      def[i].spec.geom->group = 0;
      def[i].spec.tendon->group = 0;
    }
  }
}



// make composite object
bool SIM_CComposite::Make(sim_spec_t* spec, sim_spec_body_t* body, char* error, int error_sz) {
  sim_builder_model_t* model = (sim_builder_model_t*)spec->element;

  // check counts
  for (int i=0; i < 3; i++) {
    if (count[i] < 1) {
      return comperr(error, "Positive counts expected in composite", error_sz);
    }
  }

  // check cable sizes are nonzero if vertices are not prescribed
  if (sim_math_internal_dot3(size, size) < SIM_MINVAL && uservert.empty()) {
    return comperr(error, "Positive spacing or length expected in composite", error_sz);
  }

  // check either uservert or count but not both
  if (!uservert.empty()) {
    if (count[0] > 1) {
      return comperr(error, "Either vertex or count can be specified, not both", error_sz);
    }
    count[0] = uservert.size()/3;
    count[1] = 1;
  }

  // determine dimensionality, check singleton order
  bool first = false;
  for (int i=0; i < 3; i++) {
    if (count[i] == 1) {
      first = true;
    } else {
      dim++;
      if (first) {
        return comperr(error, "Singleton counts must come last", error_sz);
      }
    }
  }

  // clear skin vectors
  face.clear();
  vert.clear();
  bindpos.clear();
  bindquat.clear();
  texcoord.clear();
  vertid.clear();
  vertweight.clear();

  // require 3x3 for subgrid
  if (skin && skinsubgrid > 0 && type != SIM_COMPTYPE_CABLE) {
    if (count[0] < 3 || count[1] < 3) {
      return comperr(error, "At least 3x3 required for skin subgrid", error_sz);
    }
  }

  // check plugin compatibility
  // TODO: move simcore.elasticity.cable to the engine
  if (plugin.active) {
    if (type != SIM_COMPTYPE_CABLE) {
      return comperr(error, "Only cable composite supports plugins", error_sz);
    }
    if (plugin_name != "simcore.elasticity.cable") {
      return comperr(error, "Only simcore.elasticity.cable is supported by composites", error_sz);
    }
  }

  // overwrite plugin name
  if (plugin_instance_name.empty() && plugin.active) {
    plugin_instance_name = "composite" + prefix;
    (static_cast<sim_builder_plugin_t*>(plugin.element))->name = plugin_instance_name;
  }

  // dispatch
  switch (type) {
    case SIM_COMPTYPE_PARTICLE:
      return comperr(error,
                     "The \"particle\" composite type is deprecated. Please use "
                     "\"replicate\" instead.",
                     error_sz);

    case SIM_COMPTYPE_GRID:
      return comperr(error,
                     "The \"grid\" composite type is deprecated. Please use "
                     "\"flex\" instead.",
                     error_sz);

    case SIM_COMPTYPE_ROPE:
      return comperr(error,
                     "The \"rope\" composite type is deprecated. Please use "
                     "\"cable\" instead.",
                     error_sz);

    case SIM_COMPTYPE_LOOP:
      return comperr(error,
                     "The \"loop\" composite type is deprecated. Please use "
                     "\"flexcomp\" instead.",
                     error_sz);

    case SIM_COMPTYPE_CABLE:
      return MakeCable(model, body, error, error_sz);

    case SIM_COMPTYPE_CLOTH:
      return comperr(error,
                     "The \"cloth\" composite type is deprecated. Please use "
                     "\"shell\" instead.",
                     error_sz);

    default:
      return comperr(error, "Unknown shape in composite", error_sz);
  }
}



bool SIM_CComposite::MakeCable(sim_builder_model_t* model, sim_spec_body_t* body, char* error, int error_sz) {
  // check dim
  if (dim != 1) {
    return comperr(error, "Cable must be one-dimensional", error_sz);
  }

  // check geom type
  if (def[0].spec.geom->type != SIM_GEOM_CYLINDER &&
      def[0].spec.geom->type != SIM_GEOM_CAPSULE &&
      def[0].spec.geom->type != SIM_GEOM_BOX) {
    return comperr(error, "Cable geom type must be sphere, capsule or box", error_sz);
  }

  // add name to model
  SIM_sText* pte = sim_spec_addText(&model->spec);
  sim_spec_set_name(pte->element, ("composite_" + prefix).c_str());
  sim_spec_set_string(pte->data, ("rope_" + prefix).c_str());

  // populate uservert if not specified
  if (uservert.empty()) {
    for (int ix=0; ix < count[0]; ix++) {
      double v[3];
      for (int k=0; k < 3; k++) {
        switch (curve[k]) {
          case SIM_COMPSHAPE_LINE:
            v[k] = ix*size[0]/(count[0]-1);
            break;
          case SIM_COMPSHAPE_COS:
            v[k] = size[1]*cos(SIM_PI*ix*size[2]/(count[0]-1));
            break;
          case SIM_COMPSHAPE_SIN:
            v[k] = size[1]*sin(SIM_PI*ix*size[2]/(count[0]-1));
            break;
          case SIM_COMPSHAPE_ZERO:
            v[k] = 0;
            break;
          default:
            // SHOULD NOT OCCUR
            sim_error("Invalid composite shape: %d", curve[k]);
            break;
        }
      }
      sim_math_internal_rotVecQuat(v, v, quat);
      uservert.insert(uservert.end(), v, v+3);
    }
  }

  // create frame
  double normal[3], prev_quat[4];
  sim_math_internal_set_vec(normal, 0, 1, 0);
  sim_math_internal_set_vec(prev_quat, 1, 0, 0, 0);

  // add one body after the other
  for (int ix=0; ix < count[0]-1; ix++) {
    body = AddCableBody(model, body, ix, normal, prev_quat);
  }

  // add skin
  if (def[0].spec.geom->type == SIM_GEOM_BOX) {
    if (skinsubgrid > 0) {
      count[1]+=2;
      MakeSkin2Subgrid(model, 2*def[0].spec.geom->size[2]);
      count[1]-=2;
    } else {
      count[1]++;
      MakeSkin2(model, 2*def[0].spec.geom->size[2]);
      count[1]--;
    }
  }
  return true;
}



sim_spec_body_t* SIM_CComposite::AddCableBody(sim_builder_model_t* model, sim_spec_body_t* body, int ix,
                                    double normal[3], double prev_quat[4]) {
  char txt_geom[100], txt_site[100], txt_slide[100];
  char this_body[100], next_body[100], this_joint[100];
  double dquat[4], this_quat[4];

  // set flags
  int lastidx = count[0]-2;
  bool first = ix == 0;
  bool last = ix == lastidx;
  bool secondlast = ix == lastidx-1;

  // compute edge and tangent vectors
  double edge[3], tprev[3], tnext[3], length_prev = 0;
  sim_math_internal_set_vec(edge, uservert[3*(ix+1)+0]-uservert[3*ix+0],
                    uservert[3*(ix+1)+1]-uservert[3*ix+1],
                    uservert[3*(ix+1)+2]-uservert[3*ix+2]);
  if (!first) {
    sim_math_internal_set_vec(tprev, uservert[3*ix+0]-uservert[3*(ix-1)+0],
                       uservert[3*ix+1]-uservert[3*(ix-1)+1],
                       uservert[3*ix+2]-uservert[3*(ix-1)+2]);
    length_prev = sim_math_internal_normvec(tprev, 3);
  }
  if (!last) {
    sim_math_internal_set_vec(tnext, uservert[3*(ix+2)+0]-uservert[3*(ix+1)+0],
                       uservert[3*(ix+2)+1]-uservert[3*(ix+1)+1],
                       uservert[3*(ix+2)+2]-uservert[3*(ix+1)+2]);
    sim_math_internal_normvec(tnext, 3);
  }

  // update moving frame
  double length = sim_math_internal_updateFrame(this_quat, normal, edge, tprev, tnext, first);

  // create body, joint, and geom names
  if (first) {
    sim_math::sprintf_arr(this_body, "%sB_first", prefix.c_str());
    sim_math::sprintf_arr(next_body, "%sB_%d", prefix.c_str(), ix+1);
    sim_math::sprintf_arr(this_joint, "%sJ_first", prefix.c_str());
    sim_math::sprintf_arr(txt_site, "%sS_first", prefix.c_str());
  } else if (last) {
    sim_math::sprintf_arr(this_body, "%sB_last", prefix.c_str());
    sim_math::sprintf_arr(next_body, "%sB_first", prefix.c_str());
    sim_math::sprintf_arr(this_joint, "%sJ_last", prefix.c_str());
    sim_math::sprintf_arr(txt_site, "%sS_last", prefix.c_str());
  } else if (secondlast){
    sim_math::sprintf_arr(this_body, "%sB_%d", prefix.c_str(), ix);
    sim_math::sprintf_arr(next_body, "%sB_last", prefix.c_str());
    sim_math::sprintf_arr(this_joint, "%sJ_%d", prefix.c_str(), ix);
  } else {
    sim_math::sprintf_arr(this_body, "%sB_%d", prefix.c_str(), ix);
    sim_math::sprintf_arr(next_body, "%sB_%d", prefix.c_str(), ix+1);
    sim_math::sprintf_arr(this_joint, "%sJ_%d", prefix.c_str(), ix);
  }
  sim_math::sprintf_arr(txt_geom, "%sG%d", prefix.c_str(), ix);
  sim_math::sprintf_arr(txt_slide, "%sJs%d", prefix.c_str(), ix);

  // add body
  body = sim_spec_addBody(body, 0);
  sim_spec_set_name(body->element, this_body);
  if (first) {
    sim_math_internal_set_vec(body->pos, offset[0]+uservert[3*ix],
                           offset[1]+uservert[3*ix+1],
                           offset[2]+uservert[3*ix+2]);
    sim_math_internal_copy_vec(body->quat, this_quat, 4);
    if (frame) {
      sim_spec_setFrame(body->element, frame);
    }
  } else {
    sim_math_internal_set_vec(body->pos, length_prev, 0, 0);
    double negquat[4] = {prev_quat[0], -prev_quat[1], -prev_quat[2], -prev_quat[3]};
    sim_math_internal_mulquat(dquat, negquat, this_quat);
    sim_math_internal_copy_vec(body->quat, dquat, 4);
  }

  // add geom
  sim_spec_geom_t* geom = sim_spec_addGeom(body, &def[0].spec);
  sim_spec_setDefault(geom->element, sim_spec_getDefault(body->element));
  sim_spec_set_name(geom->element, txt_geom);
  if (def[0].spec.geom->type == SIM_GEOM_CYLINDER ||
      def[0].spec.geom->type == SIM_GEOM_CAPSULE) {
    sim_math_internal_zerovec(geom->fromto, 6);
    geom->fromto[3] = length;
  } else if (def[0].spec.geom->type == SIM_GEOM_BOX) {
    sim_math_internal_zerovec(geom->pos, 3);
    geom->pos[0] = length/2;
    geom->size[0] = length/2;
  }

  // add plugin
  if (plugin.active) {
    SIM_sPlugin* pplugin = &body->plugin;
    pplugin->active = true;
    pplugin->element = plugin.element;
    sim_spec_set_string(pplugin->plugin_name, sim_spec_getString(plugin.plugin_name));
    sim_spec_set_string(pplugin->name, plugin_instance_name.c_str());
  }

  // update orientation
  sim_math_internal_copy_vec(prev_quat, this_quat, 4);

  // add curvature joint
  if (!first || strcmp(initial.c_str(), "none")) {
    SIM_sJoint* jnt = sim_spec_addJoint(body, &defjoint[SIM_COMPKIND_JOINT][0].spec);
    sim_spec_setDefault(jnt->element, sim_spec_getDefault(body->element));
    jnt->type = (first && strcmp(initial.c_str(), "free") == 0) ? SIM_JNT_FREE : SIM_JNT_BALL;
    jnt->damping = jnt->type == SIM_JNT_FREE ? 0 : jnt->damping;
    jnt->armature = jnt->type == SIM_JNT_FREE ? 0 : jnt->armature;
    jnt->frictionloss = jnt->type == SIM_JNT_FREE ? 0 : jnt->frictionloss;
    sim_spec_set_name(jnt->element, this_joint);
  }

  // exclude contact pair
  if (!last) {
    SIM_sExclude* exclude = sim_spec_addExclude(&model->spec);
    sim_spec_set_string(exclude->bodyname1, std::string(this_body).c_str());
    sim_spec_set_string(exclude->bodyname2, std::string(next_body).c_str());
  }

  // add site at the boundary
  if (last || first) {
    SIM_sSite* site = sim_spec_addSite(body, &def[0].spec);
    sim_spec_setDefault(site->element, sim_spec_getDefault(body->element));
    sim_spec_set_name(site->element, txt_site);
    sim_math_internal_set_vec(site->pos, last ? length : 0, 0, 0);
    sim_math_internal_set_vec(site->quat, 1, 0, 0, 0);
  }

  return body;
}



// copy local vectors to skin
void SIM_CComposite::CopyIntoSkin(SIM_sSkin* skin) {
  sim_spec_setInt(skin->face, face.data(), face.size());
  sim_spec_setFloat(skin->vert, vert.data(), vert.size());
  sim_spec_setFloat(skin->bindpos, bindpos.data(), bindpos.size());
  sim_spec_setFloat(skin->bindquat, bindquat.data(), bindquat.size());
  sim_spec_setFloat(skin->texcoord, texcoord.data(), texcoord.size());

  for (int i=0; i < vertid.size(); i++) {
    sim_spec_appendIntVec(skin->vertid, vertid[i].data(), vertid[i].size());
  }
  for (int i=0; i < vertweight.size(); i++) {
    sim_spec_appendFloatVec(skin->vertweight, vertweight[i].data(), vertweight[i].size());
  }

  face.clear();
  vert.clear();
  bindpos.clear();
  bindquat.clear();
  texcoord.clear();
  vertid.clear();
  vertweight.clear();
}



// add skin to 2D
void SIM_CComposite::MakeSkin2(sim_builder_model_t* model, sim_scalar_t inflate) {
  char txt[100];
  int N = count[0]*count[1];

  // add skin, set name and material
  SIM_sSkin* skin = sim_spec_addSkin(&model->spec);
  sim_math::sprintf_arr(txt, "%sSkin", prefix.c_str());
  sim_spec_set_name(skin->element, txt);
  sim_spec_set_string(skin->material, skinmaterial.c_str());
  sim_math_internal_copy_vec(skin->rgba, skinrgba, 4);
  skin->inflate = inflate;
  skin->group = skingroup;

  // populate mesh: two sides
  for (int i=0; i < 2; i++) {
    for (int ix=0; ix < count[0]; ix++) {
      for (int iy=0; iy < count[1]; iy++) {
        // vertex
        vert.push_back(0);
        vert.push_back(0);
        vert.push_back(0);

        // texture coordinate
        if (skintexcoord) {
          texcoord.push_back(ix/(float)(count[0]-1));
          texcoord.push_back(iy/(float)(count[1]-1));
        }

        // face
        if (ix < count[0]-1 && iy < count[1]-1) {
          face.push_back(i*N + ix*count[1]+iy);
          face.push_back(i*N + (ix+1)*count[1]+iy+(i == 1));
          face.push_back(i*N + (ix+1)*count[1]+iy+(i == 0));

          face.push_back(i*N + ix*count[1]+iy);
          face.push_back(i*N + (ix+(i == 0))*count[1]+iy+1);
          face.push_back(i*N + (ix+(i == 1))*count[1]+iy+1);
        }
      }
    }
  }

  // add thin triangles: X direction, iy = 0
  for (int ix=0; ix < count[0]-1; ix++) {
    face.push_back(ix*count[1]);
    face.push_back(N + (ix+1)*count[1]);
    face.push_back((ix+1)*count[1]);

    face.push_back(ix*count[1]);
    face.push_back(N + ix*count[1]);
    face.push_back(N + (ix+1)*count[1]);
  }

  // add thin triangles: X direction, iy = count[1]-1
  for (int ix=0; ix < count[0]-1; ix++) {
    face.push_back(ix*count[1] + count[1]-1);
    face.push_back((ix+1)*count[1] + count[1]-1);
    face.push_back(N + (ix+1)*count[1] + count[1]-1);

    face.push_back(ix*count[1] + count[1]-1);
    face.push_back(N + (ix+1)*count[1] + count[1]-1);
    face.push_back(N + ix*count[1] + count[1]-1);
  }

  // add thin triangles: Y direction, ix = 0
  for (int iy=0; iy < count[1]-1; iy++) {
    face.push_back(iy);
    face.push_back(iy+1);
    face.push_back(N + iy+1);

    face.push_back(iy);
    face.push_back(N + iy+1);
    face.push_back(N + iy);
  }

  // add thin triangles: Y direction, ix = count[0]-1
  for (int iy=0; iy < count[1]-1; iy++) {
    face.push_back(iy + (count[0]-1)*count[1]);
    face.push_back(N + iy+1 + (count[0]-1)*count[1]);
    face.push_back(iy+1 + (count[0]-1)*count[1]);

    face.push_back(iy + (count[0]-1)*count[1]);
    face.push_back(N + iy + (count[0]-1)*count[1]);
    face.push_back(N + iy+1 + (count[0]-1)*count[1]);
  }

  // couple with bones

  MakeCableBones(model, skin);

  CopyIntoSkin(skin);
}



// add bones to 1D
void SIM_CComposite::MakeCableBones(sim_builder_model_t* model, SIM_sSkin* skin) {
  char this_body[100];
  int N = count[0]*count[1];

  // populate bones
  for (int ix=0; ix < count[0]; ix++) {
    for (int iy=0; iy < count[1]; iy++) {
      // body name
      if (ix == 0) {
        sim_math::sprintf_arr(this_body, "%sB_first", prefix.c_str());
      } else if (ix >= count[0]-2) {
        sim_math::sprintf_arr(this_body, "%sB_last", prefix.c_str());
      } else {
        sim_math::sprintf_arr(this_body, "%sB_%d", prefix.c_str(), ix);
      }

      // bind pose
      if (iy == 0) {
        sim_spec_appendString(skin->bodyname, this_body);
        bindpos.push_back((ix == count[0]-1) ? -2*def[0].spec.geom->size[0] : 0);
        bindpos.push_back(-def[0].spec.geom->size[1]);
        bindpos.push_back(0);
        bindquat.push_back(1); bindquat.push_back(0);
        bindquat.push_back(0); bindquat.push_back(0);
      } else {
        sim_spec_appendString(skin->bodyname, this_body);
        bindpos.push_back((ix == count[0]-1) ? -2*def[0].spec.geom->size[0] : 0);
        bindpos.push_back(def[0].spec.geom->size[1]);
        bindpos.push_back(0);
        bindquat.push_back(1); bindquat.push_back(0);
        bindquat.push_back(0); bindquat.push_back(0);
      }

      // create vertid and vertweight
      vertid.push_back({ix*count[1]+iy, N + ix*count[1]+iy});
      vertweight.push_back({1, 1});
    }
  }
}



void SIM_CComposite::MakeCableBonesSubgrid(sim_builder_model_t* model, SIM_sSkin* skin) {
  // populate bones
  for (int ix=0; ix < count[0]; ix++) {
    for (int iy=0; iy < count[1]; iy++) {
      char txt[100];

      // body name
      if (ix == 0) {
        sim_math::sprintf_arr(txt, "%sB_first", prefix.c_str());
      } else if (ix >= count[0]-2) {
        sim_math::sprintf_arr(txt, "%sB_last", prefix.c_str());
      } else {
        sim_math::sprintf_arr(txt, "%sB_%d", prefix.c_str(), ix);
      }

      // bind pose
      if (iy == 0) {
        bindpos.push_back((ix == count[0]-1) ? -2*def[0].spec.geom->size[0] : 0);
        bindpos.push_back(-def[0].Geom().spec.size[1]);
        bindpos.push_back(0);
      } else if (iy == 2) {
        bindpos.push_back((ix == count[0]-1) ? -2*def[0].spec.geom->size[0] : 0);
        bindpos.push_back(def[0].Geom().spec.size[1]);
        bindpos.push_back(0);
      } else {
        bindpos.push_back((ix == count[0]-1) ? -2*def[0].spec.geom->size[0] : 0);
        bindpos.push_back(0);
        bindpos.push_back(0);
      }
      sim_spec_appendString(skin->bodyname, txt);
      bindquat.push_back(1);
      bindquat.push_back(0);
      bindquat.push_back(0);
      bindquat.push_back(0);

      // empty vertid and vertweight
      vertid.push_back({});
      vertweight.push_back({});
    }
  }
}



//------------------------------------- subgrid matrices

// C = W * [f; f_x; f_y; f_xy]
static const sim_scalar_t subW[16*16] = {
  1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
  0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0,
 -3,  0,  0,  3,  0,  0,  0,  0, -2,  0,  0, -1,  0,  0,  0,  0,
  2,  0,  0, -2,  0,  0,  0,  0,  1,  0,  0,  1,  0,  0,  0,  0,
  0,  0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0,
  0,  0,  0,  0, -3,  0,  0,  3,  0,  0,  0,  0, -2,  0,  0, -1,
  0,  0,  0,  0,  2,  0,  0, -2,  0,  0,  0,  0,  1,  0,  0,  1,
 -3,  3,  0,  0, -2, -1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
  0,  0,  0,  0,  0,  0,  0,  0, -3,  3,  0,  0, -2, -1,  0,  0,
  9, -9,  9, -9,  6,  3, -3, -6,  6, -6, -3,  3,  4,  2,  1,  2,
 -6,  6, -6,  6, -4, -2,  2,  4, -3,  3,  3, -3, -2, -1, -1, -2,
  2, -2,  0,  0,  1,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
  0,  0,  0,  0,  0,  0,  0,  0,  2, -2,  0,  0,  1,  1,  0,  0,
 -6,  6, -6,  6, -3, -3,  3,  3, -4,  4,  2, -2, -2, -2, -1, -1,
  4, -4,  4, -4,  2,  2, -2, -2,  2, -2, -2,  2,  1,  1,  1,  1
};

// left-bottom
static const sim_scalar_t subD00[] = {
  5,  1, -1,                  // f
  9,  1, -1,
  10, 1, -1,
  6,  1, -1,

  5, -1,   9,  1,   -1,       // f_x
  5, -0.5, 13, 0.5, -1,
  6, -0.5, 14, 0.5, -1,
  6, -1,   10, 1,   -1,

  5, -1,   6,  1,   -1,       // f_y
  9, -1,   10, 1,   -1,
  9, -0.5, 11, 0.5, -1,
  5, -0.5, 7,  0.5, -1,

  9,  -1,    6, -1,    5, 1,    10, 1,    -1,     // f_xy
  13, -0.5,  6, -0.5,  5, 0.5,  14, 0.5,  -1,
  13, -0.25, 7, -0.25, 5, 0.25, 15, 0.25, -1,
  9,  -0.5,  7, -0.5,  5, 0.5,  11, 0.5,  -1
};

// center-bottom
static const sim_scalar_t subD10[] = {
  5,  1, -1,                  // f
  9,  1, -1,
  10, 1, -1,
  6,  1, -1,

  1, -0.5, 9,  0.5, -1,       // f_x
  5, -0.5, 13, 0.5, -1,
  6, -0.5, 14, 0.5, -1,
  2, -0.5, 10, 0.5, -1,

  5, -1,   6,  1,   -1,       // f_y
  9, -1,   10, 1,   -1,
  9, -0.5, 11, 0.5, -1,
  5, -0.5, 7,  0.5, -1,

  9,  -0.5,  2, -0.5,  1, 0.5,  10, 0.5,  -1,     // f_xy
  13, -0.5,  6, -0.5,  5, 0.5,  14, 0.5,  -1,
  13, -0.25, 7, -0.25, 5, 0.25, 15, 0.25, -1,
  9,  -0.25, 3, -0.25, 1, 0.25, 11, 0.25, -1
};

// right-bottom
static const sim_scalar_t subD20[] = {
  5,  1, -1,                  // f
  9,  1, -1,
  10, 1, -1,
  6,  1, -1,

  1, -0.5, 9,  0.5, -1,       // f_x
  5, -1,   9,  1,   -1,
  6, -1,   10, 1,   -1,
  2, -0.5, 10, 0.5, -1,

  5, -1,   6,  1,   -1,       // f_y
  9, -1,   10, 1,   -1,
  9, -0.5, 11, 0.5, -1,
  5, -0.5, 7,  0.5, -1,

  9, -0.5,  2, -0.5,  1, 0.5,  10, 0.5,  -1,      // f_xy
  9, -1,    6, -1,    5, 1,    10, 1,    -1,
  9, -0.5,  7, -0.5,  5, 0.5,  11, 0.5,  -1,
  9, -0.25, 3, -0.25, 1, 0.25, 11, 0.25, -1
};

// left-center
static const sim_scalar_t subD01[] = {
  5,  1, -1,                  // f
  9,  1, -1,
  10, 1, -1,
  6,  1, -1,

  5, -1,   9,  1,   -1,       // f_x
  5, -0.5, 13, 0.5, -1,
  6, -0.5, 14, 0.5, -1,
  6, -1,   10, 1,   -1,

  4, -0.5, 6,  0.5, -1,       // f_y
  8, -0.5, 10, 0.5, -1,
  9, -0.5, 11, 0.5, -1,
  5, -0.5, 7,  0.5, -1,

  8,  -0.5,  6, -0.5,  4, 0.5,  10, 0.5,  -1,     // f_xy
  12, -0.25, 6, -0.25, 4, 0.25, 14, 0.25, -1,
  13, -0.25, 7, -0.25, 5, 0.25, 15, 0.25, -1,
  9,  -0.5,  7, -0.5,  5, 0.5,  11, 0.5,  -1
};

// center-center
static const sim_scalar_t subD11[] = {
  5,  1, -1,                  // f
  9,  1, -1,
  10, 1, -1,
  6,  1, -1,

  1, -0.5, 9,  0.5, -1,       // f_x
  5, -0.5, 13, 0.5, -1,
  6, -0.5, 14, 0.5, -1,
  2, -0.5, 10, 0.5, -1,

  4, -0.5, 6,  0.5, -1,       // f_y
  8, -0.5, 10, 0.5, -1,
  9, -0.5, 11, 0.5, -1,
  5, -0.5, 7,  0.5, -1,

  8,  -0.25, 2, -0.25, 0, 0.25, 10, 0.25, -1,     // f_xy
  12, -0.25, 6, -0.25, 4, 0.25, 14, 0.25, -1,
  13, -0.25, 7, -0.25, 5, 0.25, 15, 0.25, -1,
  9,  -0.25, 3, -0.25, 1, 0.25, 11, 0.25, -1
};

// right-center
static const sim_scalar_t subD21[] = {
  5,  1, -1,                  // f
  9,  1, -1,
  10, 1, -1,
  6,  1, -1,

  1, -0.5, 9,  0.5, -1,       // f_x
  5, -1,   9,  1,   -1,
  6, -1,   10, 1,   -1,
  2, -0.5, 10, 0.5, -1,

  4, -0.5, 6,  0.5, -1,       // f_y
  8, -0.5, 10, 0.5, -1,
  9, -0.5, 11, 0.5, -1,
  5, -0.5, 7,  0.5, -1,

  8, -0.25, 2, -0.25, 0, 0.25, 10, 0.25, -1,      // f_xy
  8, -0.5,  6, -0.5,  4, 0.5,  10, 0.5,  -1,
  9, -0.5,  7, -0.5,  5, 0.5,  11, 0.5,  -1,
  9, -0.25, 3, -0.25, 1, 0.25, 11, 0.25, -1
};

// left-top
static const sim_scalar_t subD02[] = {
  5,  1, -1,                  // f
  9,  1, -1,
  10, 1, -1,
  6,  1, -1,

  5, -1,   9,  1,   -1,       // f_x
  5, -0.5, 13, 0.5, -1,
  6, -0.5, 14, 0.5, -1,
  6, -1,   10, 1,   -1,

  4, -0.5, 6,  0.5, -1,       // f_y
  8, -0.5, 10, 0.5, -1,
  9, -1,   10, 1,   -1,
  5, -1,   6,  1,   -1,

  8,  -0.5,  6, -0.5,  4, 0.5,  10, 0.5,  -1,     // f_xy
  12, -0.25, 6, -0.25, 4, 0.25, 14, 0.25, -1,
  13, -0.5,  6, -0.5,  5, 0.5,  14, 0.5,  -1,
  9,  -1,    6, -1,    5, 1,    10, 1,    -1
};

// center-top
static const sim_scalar_t subD12[] = {
  5,  1, -1,                  // f
  9,  1, -1,
  10, 1, -1,
  6,  1, -1,

  1, -0.5, 9,  0.5, -1,       // f_x
  5, -0.5, 13, 0.5, -1,
  6, -0.5, 14, 0.5, -1,
  2, -0.5, 10, 0.5, -1,

  4, -0.5, 6,  0.5, -1,       // f_y
  8, -0.5, 10, 0.5, -1,
  9, -1,   10, 1,   -1,
  5, -1,   6,  1,   -1,

  8,  -0.25, 2, -0.25, 0, 0.25, 10, 0.25, -1,     // f_xy
  12, -0.25, 6, -0.25, 4, 0.25, 14, 0.25, -1,
  13, -0.5,  6, -0.5,  5, 0.5,  14, 0.5,  -1,
  9,  -0.5,  2, -0.5,  1, 0.5,  10, 0.5,  -1
};

// right-top
static const sim_scalar_t subD22[] = {
  5,  1, -1,                  // f
  9,  1, -1,
  10, 1, -1,
  6,  1, -1,

  1, -0.5, 9,  0.5, -1,       // f_x
  5, -1,   9,  1,   -1,
  6, -1,   10, 1,   -1,
  2, -0.5, 10, 0.5, -1,

  4, -0.5, 6,  0.5, -1,       // f_y
  8, -0.5, 10, 0.5, -1,
  9, -1,   10, 1,   -1,
  5, -1,   6,  1,   -1,

  8, -0.25, 2, -0.25, 0, 0.25, 10, 0.25, -1,      // f_xy
  8, -0.5,  6, -0.5,  4, 0.5,  10, 0.5,  -1,
  9, -1,    6, -1,    5, 1,    10, 1,    -1,
  9, -0.5,  2, -0.5,  1, 0.5,  10, 0.5,  -1
};


// add skin to 2D, with subgrid
void SIM_CComposite::MakeSkin2Subgrid(sim_builder_model_t* model, sim_scalar_t inflate) {
  // assemble pointers to Dxx matrices
  const sim_scalar_t* Dp[3][3] = {
    {subD00, subD01, subD02},
    {subD10, subD11, subD12},
    {subD20, subD21, subD22}
  };

  // allocate
  const int N = (2+skinsubgrid)*(2+skinsubgrid);
  sim_scalar_t* XY = (sim_scalar_t*) sim_malloc(N*16*sizeof(sim_scalar_t));
  sim_scalar_t* XY_W = (sim_scalar_t*) sim_malloc(N*16*sizeof(sim_scalar_t));
  sim_scalar_t* Weight = (sim_scalar_t*) sim_malloc(9*N*16*sizeof(sim_scalar_t));
  sim_scalar_t* D = (sim_scalar_t*) sim_malloc(16*16*sizeof(sim_scalar_t));

  // XY matrix
  const sim_scalar_t step = 1.0/(1+skinsubgrid);
  int rxy = 0;
  for (int sx=0; sx <= 1+skinsubgrid; sx++) {
    for (int sy=0; sy <= 1+skinsubgrid; sy++) {
      // compute x, y
      sim_scalar_t x = sx*step;
      sim_scalar_t y = sy*step;

      // make XY-row
      XY[16*rxy + 0] =  1;
      XY[16*rxy + 1] =  y;
      XY[16*rxy + 2] =  y*y;
      XY[16*rxy + 3] =  y*y*y;

      XY[16*rxy + 4] =  x*1;
      XY[16*rxy + 5] =  x*y;
      XY[16*rxy + 6] =  x*y*y;
      XY[16*rxy + 7] =  x*y*y*y;

      XY[16*rxy + 8] =  x*x*1;
      XY[16*rxy + 9] =  x*x*y;
      XY[16*rxy + 10] = x*x*y*y;
      XY[16*rxy + 11] = x*x*y*y*y;

      XY[16*rxy + 12] = x*x*x*1;
      XY[16*rxy + 13] = x*x*x*y;
      XY[16*rxy + 14] = x*x*x*y*y;
      XY[16*rxy + 15] = x*x*x*y*y*y;

      // advance row
      rxy++;
    }
  }

  // XY_W = XY * W
  sim_math_mulMatMat(XY_W, XY, subW, N, 16, 16);

  // Weight matrices
  for (int dx=0; dx < 3; dx++) {
    for (int dy=0; dy < 3; dy++) {
      // make dense D
      sim_math_zero(D, 16*16);
      int cnt = 0;
      int r = 0, c;
      while (r < 16) {
        // scan row
        while ((c = sim_math_round(Dp[dx][dy][cnt])) != -1) {
          D[r*16+c] = Dp[dx][dy][cnt+1];
          cnt +=2;
        }

        // advance
        r++;
        cnt++;
      }

      // Weight(d) = XY * W * D(d)
      sim_math_mulMatMat(Weight + (dx*3+dy)*N*16, XY_W, D, N, 16, 16);
    }
  }

  // add skin, set name and material
  char txt[100];
  SIM_sSkin* skin = sim_spec_addSkin(&model->spec);
  sim_math::sprintf_arr(txt, "%sSkin", prefix.c_str());
  sim_spec_set_name(skin->element, txt);
  sim_spec_set_string(skin->material, skinmaterial.c_str());
  sim_math_internal_copy_vec(skin->rgba, skinrgba, 4);
  skin->inflate = inflate;
  skin->group = skingroup;

  // populate mesh: two sides
  sim_scalar_t S = 0;
  int C0 = count[0] + (count[0]-1)*skinsubgrid;
  int C1 = count[1] + (count[1]-1)*skinsubgrid;
  int NN = C0*C1;
  for (int i=0; i < 2; i++) {
    for (int ix=0; ix < C0; ix++) {
      for (int iy=0; iy < C1; iy++) {
        // vertex
        vert.push_back(ix*S);
        vert.push_back(iy*S);
        vert.push_back(0);

        // texture coordinate
        if (skintexcoord) {
          texcoord.push_back(ix/(float)(C0-1));
          texcoord.push_back(iy/(float)(C1-1));
        }

        // face
        if (ix < C0-1 && iy < C1-1) {
          face.push_back(i*NN + ix*C1+iy);
          face.push_back(i*NN + (ix+1)*C1+iy+(i == 1));
          face.push_back(i*NN + (ix+1)*C1+iy+(i == 0));

          face.push_back(i*NN + ix*C1+iy);
          face.push_back(i*NN + (ix+(i == 0))*C1+iy+1);
          face.push_back(i*NN + (ix+(i == 1))*C1+iy+1);
        }
      }
    }
  }

  // add thin triangles: X direction, iy = 0
  for (int ix=0; ix < C0-1; ix++) {
    face.push_back(ix*C1);
    face.push_back(NN + (ix+1)*C1);
    face.push_back((ix+1)*C1);

    face.push_back(ix*C1);
    face.push_back(NN + ix*C1);
    face.push_back(NN + (ix+1)*C1);
  }

  // add thin triangles: X direction, iy = C1-1
  for (int ix=0; ix < C0-1; ix++) {
    face.push_back(ix*C1 + C1-1);
    face.push_back((ix+1)*C1 + C1-1);
    face.push_back(NN + (ix+1)*C1 + C1-1);

    face.push_back(ix*C1 + C1-1);
    face.push_back(NN + (ix+1)*C1 + C1-1);
    face.push_back(NN + ix*C1 + C1-1);
  }

  // add thin triangles: Y direction, ix = 0
  for (int iy=0; iy < C1-1; iy++) {
    face.push_back(iy);
    face.push_back(iy+1);
    face.push_back(NN + iy+1);

    face.push_back(iy);
    face.push_back(NN + iy+1);
    face.push_back(NN + iy);
  }

  // add thin triangles: Y direction, ix = C0-1
  for (int iy=0; iy < C1-1; iy++) {
    face.push_back(iy + (C0-1)*C1);
    face.push_back(NN + iy+1 + (C0-1)*C1);
    face.push_back(iy+1 + (C0-1)*C1);

    face.push_back(iy + (C0-1)*C1);
    face.push_back(NN + iy + (C0-1)*C1);
    face.push_back(NN + iy+1 + (C0-1)*C1);
  }

  MakeCableBonesSubgrid(model, skin);

  // bind vertices to bones: one big square at a time
  for (int ix=0; ix < count[0]-1; ix++) {
    for (int iy=0; iy < count[1]-1; iy++) {
      // determine d for Weight indexing
      int d = 3 * (ix == 0 ? 0 : (ix == count[0]-2 ? 2 : 1)) +
              (iy == 0 ? 0 : (iy == count[1]-2 ? 2 : 1));

      // precompute 16 bone indices for big square
      int boneid[16];
      int cnt = 0;
      for (int dx=-1; dx < 3; dx++) {
        for (int dy=-1; dy < 3; dy++) {
          boneid[cnt++] = (ix+dx)*count[1] + (iy+dy);
        }
      }

      // process subgrid, top-rigth owns last index
      for (int dx=0; dx < 1+skinsubgrid+(ix == count[0]-2); dx++) {
        for (int dy=0; dy < 1+skinsubgrid+(iy == count[1]-2); dy++) {
          // recover vertex id
          int vid = (ix*(1+skinsubgrid)+dx)*C1 + iy*(1+skinsubgrid)+dy;

          // determine row in Weight
          int n = dx*(2+skinsubgrid) + dy;

          // add vertex to 16 bones
          for (int bi=0; bi < 16; bi++) {
            sim_scalar_t w = Weight[d*N*16 + n*16 + bi];
            if (w) {
              vertid[boneid[bi]].push_back(vid);
              vertid[boneid[bi]].push_back(vid+NN);
              vertweight[boneid[bi]].push_back((float)w);
              vertweight[boneid[bi]].push_back((float)w);
            }
          }
        }
      }
    }
  }

  CopyIntoSkin(skin);

  // free allocations
  sim_free(XY);
  sim_free(XY_W);
  sim_free(Weight);
  sim_free(D);
}

