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

#include "user/user_api.h"

#include <algorithm>
#include <cstddef>
#include <cstring>
#include <functional>
#include <iterator>
#include <map>
#include <new>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <simcore/core_api.h>
#include "engine/engine_support.h"
#include "user/user_cache.h"
#include "user/user_model.h"
#include "user/user_objects.h"
#include "user/user_resource.h"
#include "user/user_util.h"

namespace {

using simcore::user::StringToVector;

}  // namespace

// global cache size in bytes (default 500MB)
static constexpr std::size_t kGlobalCacheSize = 500 * (1 << 20);


// create model
sim_spec_t* sim_makeSpec() {
  sim_builder_model_t* modelC = new sim_builder_model_t;
  return &modelC->spec;
}



// copy model
sim_spec_t* sim_copySpec(const sim_spec_t* s) {
  sim_builder_model_t* modelC = nullptr;
  try {
    modelC = new sim_builder_model_t(*static_cast<sim_builder_model_t*>(s->element));
  } catch (sim_builder_error_t& e) {
    static_cast<sim_builder_model_t*>(s->element)->SetError(e);
    return nullptr;
  }
  return &modelC->spec;
}

// parse file into spec
sim_spec_t* sim_parse(const char* filename, const char* content_type,
                 const SIM_VFS* vfs, char* error, int error_sz) {
  SIM_VFS local_vfs;
  simcore::user::Cleanup cleanup;

  // early exit for existing XML workflow
  auto filepath = simcore::user::FilePath(filename);
  if (filepath.Ext() == ".xml" ||
      filepath.Ext() == ".urdf" ||
      (content_type && std::strcmp(content_type, "text/xml") == 0)) {
    return sim_parseXML(filename, vfs, error, error_sz);
  }

  // If no VFS is provided, we'll create our own temporary one for the duration
  // of this function.
  if (vfs == nullptr) {
    sim_defaultVFS(&local_vfs);
    cleanup += [&local_vfs](){ sim_deleteVFS(&local_vfs); };

    vfs = &local_vfs;
  }

  SIM_Resource* resource = sim_math_openResource("", filename, vfs, error, error_sz);
  // If we are unable to open the resource, we will create our own resource with
  // just the filename. This allows decoders that rely on other systems to fetch
  // their content to function without a custom resource provider.
  // For example, USD may use identifiers to assets that are strictly in memory
  // or that are fetched on a need-be basis via URI.
  if (resource) {
    cleanup += [resource](){ sim_math_closeResource(resource); };
  } else {
    resource = (SIM_Resource*) sim_malloc(sizeof(SIM_Resource));
    cleanup += [resource](){ if (resource) sim_free(resource); };

    if (resource == nullptr) {
      if (error) {
        strncpy(error, "could not allocate memory", error_sz);
        error[error_sz - 1] = '\0';
      }
      return nullptr;
    }

    // clear out resource
    memset(resource, 0, sizeof(SIM_Resource));

    // make space for filename
    std::string fullname = filename;
    std::size_t n = fullname.size();
    resource->name = (char*) sim_malloc(sizeof(char) * (n + 1));
    cleanup += [resource](){ if (resource) sim_free(resource->name); };

    if (resource->name == nullptr) {
      if (error) {
        strncpy(error, "could not allocate memory", error_sz);
        error[error_sz - 1] = '\0';
      }
      return nullptr;
    }
    memcpy(resource->name, fullname.c_str(), sizeof(char) * (n + 1));
  }

  sim_spec_t* spec = sim_math_decodeResource(resource, content_type, vfs);
  if (spec == nullptr) {
    if (error) {
      strncpy(error, "could not decode content", error_sz);
      error[error_sz - 1] = '\0';
    }
  }
  return spec;
}

// compile model
sim_model_t* sim_compile(sim_spec_t* s, const SIM_VFS* vfs) {
  sim_builder_model_t* modelC = static_cast<sim_builder_model_t*>(s->element);
  return modelC->Compile(vfs);
}



// recompile spec to model, preserving the state, return 0 on success
[[nodiscard]] int sim_recompile(sim_spec_t* s, const SIM_VFS* vfs, sim_model_t* m, sim_data_t* d) {
  sim_builder_model_t* modelC = static_cast<sim_builder_model_t*>(s->element);
  std::string state_name = "state";
  sim_scalar_t time = 0;
  try {
    if (d) {
      time = d->time;
      modelC->SaveState(state_name, d->qpos, d->qvel, d->act, d->ctrl, d->mocap_pos, d->mocap_quat);
    }
    if (!modelC->Compile(vfs, &m)) {
      if (d) {
        sim_deleteData(d);
      }
      return -1;
    };
    if (d) {
      modelC->MakeData(m, &d);
      modelC->RestoreState(state_name, m->qpos0, m->body_pos, m->body_quat, d->qpos, d->qvel,
                           d->act, d->ctrl, d->mocap_pos, d->mocap_quat);
      d->time = time;
    }
  } catch (sim_builder_error_t& e) {
    modelC->SetError(e);
    return -1;
  }
  return 0;
}


// set frame for all elements of a body
static void SetFrame(sim_spec_body_t* body, sim_obj_t objtype, SIM_sFrame* frame) {
  sim_spec_element_t* el = sim_spec_firstChild(body, objtype, 0);
  while (el) {
    if (frame->element != el && sim_spec_getFrame(el) == nullptr) {
      sim_spec_setFrame(el, frame);
    }
    el = sim_spec_nextChild(body, el, 0);
  }
}



// attach body to a frame of the parent
static sim_spec_element_t* attachBody(sim_builder_frame_t* parent, const sim_builder_body_t* child,
                              const char* prefix, const char* suffix) {
  sim_builder_body_t* mutable_child = const_cast<sim_builder_body_t*>(child);
  mutable_child->prefix = prefix;
  mutable_child->suffix = suffix;
  try {
    *parent += *mutable_child;
  } catch (sim_builder_error_t& e) {
    parent->model->SetError(e);
    return nullptr;
  }
  sim_spec_body_t* attached_body = parent->last_attached;
  parent->last_attached = nullptr;
  return attached_body->element;
}



// attach frame to a parent body
static sim_spec_element_t* attachFrame(sim_builder_body_t* parent, const sim_builder_frame_t* child,
                               const char* prefix, const char* suffix) {
  sim_builder_frame_t* mutable_child = const_cast<sim_builder_frame_t*>(child);
  mutable_child->prefix = prefix;
  mutable_child->suffix = suffix;
  try {
    *parent += *mutable_child;
  } catch (sim_builder_error_t& e) {
    parent->model->SetError(e);
    return nullptr;
  }
  SIM_sFrame* attached_frame = parent->last_attached;
  parent->last_attached = nullptr;
  return attached_frame->element;
}



// attach child body to a parent site
static sim_spec_element_t* attachToSite(sim_builder_site_t* parent, const sim_builder_body_t* child,
                                const char* prefix, const char* suffix) {
  sim_spec_t* spec = sim_spec_getSpec(parent->spec.element);
  sim_builder_body_t* body = parent->Body();
  sim_builder_frame_t* frame = body->AddFrame(parent->frame);
  frame->SetParent(body);
  frame->spec.pos[0] = parent->spec.pos[0];
  frame->spec.pos[1] = parent->spec.pos[1];
  frame->spec.pos[2] = parent->spec.pos[2];
  frame->spec.quat[0] = parent->spec.quat[0];
  frame->spec.quat[1] = parent->spec.quat[1];
  frame->spec.quat[2] = parent->spec.quat[2];
  frame->spec.quat[3] = parent->spec.quat[3];
  sim_spec_resolveOrientation(frame->spec.quat, spec->compiler.degree,
                         spec->compiler.eulerseq, &parent->spec.alt);
  return attachBody(frame, child, prefix, suffix);
}



// attach child frame to a parent site
static sim_spec_element_t* attachFrameToSite(sim_builder_site_t* parent, const sim_builder_frame_t* child,
                                     const char* prefix, const char* suffix) {
  sim_spec_t* spec = sim_spec_getSpec(parent->spec.element);
  sim_builder_body_t* body = parent->Body();
  sim_builder_frame_t* frame = body->AddFrame(parent->frame);
  frame->SetParent(body);
  frame->spec.pos[0] = parent->spec.pos[0];
  frame->spec.pos[1] = parent->spec.pos[1];
  frame->spec.pos[2] = parent->spec.pos[2];
  frame->spec.quat[0] = parent->spec.quat[0];
  frame->spec.quat[1] = parent->spec.quat[1];
  frame->spec.quat[2] = parent->spec.quat[2];
  frame->spec.quat[3] = parent->spec.quat[3];
  sim_spec_resolveOrientation(frame->spec.quat, spec->compiler.degree,
                         spec->compiler.eulerseq, &parent->spec.alt);

  sim_spec_element_t* attached_frame = attachFrame(body, child, prefix, suffix);
  sim_spec_setFrame(attached_frame, &frame->spec);
  return attached_frame;
}


sim_spec_element_t* sim_spec_attach(sim_spec_element_t* parent, const sim_spec_element_t* child,
                       const char* prefix, const char* suffix) {
  if (!parent) {
    sim_error("parent element is null");
    return nullptr;
  }
  if (!child) {
    sim_error("child element is null");
    return nullptr;
  }
  sim_builder_model_t* model = static_cast<sim_builder_model_t*>(sim_spec_getSpec(parent)->element);
  if (child->elemtype == SIM_OBJ_MODEL) {
    sim_builder_model_t* child_model = static_cast<sim_builder_model_t*>((sim_spec_element_t*)child);
    sim_spec_body_t* worldbody = sim_spec_findBody(&child_model->spec, "world");
    if (!worldbody) {
      model->SetError(sim_builder_error_t(0, "Child does not have a world body."));
      return nullptr;
    }
    SIM_sFrame* worldframe = sim_spec_addFrame(worldbody, nullptr);
    SetFrame(worldbody, SIM_OBJ_BODY, worldframe);
    SetFrame(worldbody, SIM_OBJ_SITE, worldframe);
    SetFrame(worldbody, SIM_OBJ_FRAME, worldframe);
    SetFrame(worldbody, SIM_OBJ_JOINT, worldframe);
    SetFrame(worldbody, SIM_OBJ_GEOM, worldframe);
    SetFrame(worldbody, SIM_OBJ_LIGHT, worldframe);
    SetFrame(worldbody, SIM_OBJ_CAMERA, worldframe);
    child = worldframe->element;
  }
  switch (parent->elemtype) {
    case SIM_OBJ_FRAME:
      if (child->elemtype == SIM_OBJ_BODY) {
        return attachBody(static_cast<sim_builder_frame_t*>(parent),
                          static_cast<const sim_builder_body_t*>(child), prefix, suffix);
      } else if (child->elemtype == SIM_OBJ_FRAME) {
        sim_spec_body_t* parent_body = sim_spec_getParent(parent);
        if (!parent_body) {
          model->SetError(sim_builder_error_t(0, "Frame does not have a parent body."));
          return nullptr;
        }
        sim_builder_frame_t* frame = static_cast<sim_builder_frame_t*>(parent);
        sim_spec_element_t* attached_frame =
            attachFrame(static_cast<sim_builder_body_t*>(parent_body->element),
                        static_cast<const sim_builder_frame_t*>(child), prefix, suffix);
        if (sim_spec_setFrame(attached_frame, &frame->spec)) {
          return nullptr;
        }
        return attached_frame;
      } else {
        model->SetError(sim_builder_error_t(0, "child element is not a body or frame"));
        return nullptr;
      }
    case SIM_OBJ_BODY:
      if (child->elemtype == SIM_OBJ_FRAME) {
        return attachFrame(static_cast<sim_builder_body_t*>(parent),
                           static_cast<const sim_builder_frame_t*>(child), prefix, suffix);
      } else {
        model->SetError(sim_builder_error_t(0, "child element is not a frame"));
        return nullptr;
      }
    case SIM_OBJ_SITE:
      if (child->elemtype == SIM_OBJ_BODY) {
        return attachToSite(static_cast<sim_builder_site_t*>(parent),
                            static_cast<const sim_builder_body_t*>(child), prefix, suffix);
      } else if (child->elemtype == SIM_OBJ_FRAME) {
        return attachFrameToSite(static_cast<sim_builder_site_t*>(parent),
                                 static_cast<const sim_builder_frame_t*>(child), prefix, suffix);
      } else {
        model->SetError(sim_builder_error_t(0, "child element is not a body or frame"));
        return nullptr;
      }
    default:
      model->SetError(sim_builder_error_t(0, "parent element is not a frame, body or site"));
      return nullptr;
  }
  return nullptr;
}



// get error message from model
const char* sim_spec_getError(sim_spec_t* s) {
  if (!s) {
    sim_error("spec is null");
    return nullptr;
  }
  sim_builder_model_t* modelC = static_cast<sim_builder_model_t*>(s->element);
  return modelC->GetError().message;
}



// check if model has warnings
int sim_spec_isWarning(sim_spec_t* s) {
  sim_builder_model_t* modelC = static_cast<sim_builder_model_t*>(s->element);
  return modelC->GetError().warning;
}



// delete model
void sim_deleteSpec(sim_spec_t* s) {
  if (s) {
    sim_builder_model_t* model = static_cast<sim_builder_model_t*>(s->element);
    model->Release();
  }
}



// add spec (model asset) to spec
void sim_spec_addSpec(sim_spec_t* s, sim_spec_t* child) {
  sim_builder_model_t* model = static_cast<sim_builder_model_t*>(s->element);
  model->AppendSpec(child);
}



// activate plugin
int sim_spec_activatePlugin(sim_spec_t* s, const char* name) {
  int plugin_slot = -1;
  const SIM_pPlugin* plugin = sim_plugin_getPlugin(name, &plugin_slot);
  if (!plugin) {
    return -1;
  }
  sim_builder_model_t* model = static_cast<sim_builder_model_t*>(s->element);
  model->ActivatePlugin(plugin, plugin_slot);
  return 0;
}



// set deep copy flag
int sim_spec_setDeepCopy(sim_spec_t* s, int deepcopy) {
  sim_builder_model_t* model = static_cast<sim_builder_model_t*>(s->element);
  model->SetDeepCopy(deepcopy);
  return 0;
}



// copy real-valued arrays from model to spec, returns 1 on success
int sim_copyBack(sim_spec_t* s, const sim_model_t* m) {
  sim_builder_model_t* model = static_cast<sim_builder_model_t*>(s->element);
  return model->CopyBack(m);
}



// remove body from sim_spec_t, return 0 on success
int sim_spec_delete(sim_spec_t* s, sim_spec_element_t* element) {
  sim_builder_model_t* model = static_cast<sim_builder_model_t*>(s->element);
  if (!element) {
    model->SetError(sim_builder_error_t(0, "Element is null."));
    return -1;
  }
  try {
    if (element->elemtype == SIM_OBJ_DEFAULT) {
      sim_builder_default_t* def = static_cast<sim_builder_default_t*>(element);
      *model -= *def;
    } else {
      *model -= element;
    }
    return 0;
  } catch (sim_builder_error_t& e) {
    model->SetError(e);
    return -1;
  }
}



// add child body to body, return child spec
sim_spec_body_t* sim_spec_addBody(sim_spec_body_t* bodyspec, const sim_spec_default_t* defspec) {
  sim_builder_default_t* def = defspec ? static_cast<sim_builder_default_t*>(defspec->element) : 0;
  sim_builder_body_t* body = static_cast<sim_builder_body_t*>(bodyspec->element)->AddBody(def);
  return &body->spec;
}



// add site to body, return site spec
SIM_sSite* sim_spec_addSite(sim_spec_body_t* bodyspec, const sim_spec_default_t* defspec) {
  sim_builder_default_t* def = defspec ? static_cast<sim_builder_default_t*>(defspec->element) : 0;
  sim_builder_body_t* body = static_cast<sim_builder_body_t*>(bodyspec->element);
  sim_builder_site_t* site = body->AddSite(def);
  return &site->spec;
}



// add joint to body
SIM_sJoint* sim_spec_addJoint(sim_spec_body_t* bodyspec, const sim_spec_default_t* defspec) {
  sim_builder_default_t* def = defspec ? static_cast<sim_builder_default_t*>(defspec->element) : 0;
  sim_builder_body_t* body = static_cast<sim_builder_body_t*>(bodyspec->element);
  sim_builder_joint_t* joint = body->AddJoint(def);
  return &joint->spec;
}



// add free joint to body
SIM_sJoint* sim_spec_addFreeJoint(sim_spec_body_t* bodyspec) {
  sim_builder_body_t* body = static_cast<sim_builder_body_t*>(bodyspec->element);
  sim_builder_joint_t* joint = body->AddFreeJoint();
  return &joint->spec;
}



// add geom to body
sim_spec_geom_t* sim_spec_addGeom(sim_spec_body_t* bodyspec, const sim_spec_default_t* defspec) {
  sim_builder_default_t* def = defspec ? static_cast<sim_builder_default_t*>(defspec->element) : 0;
  sim_builder_body_t* body = static_cast<sim_builder_body_t*>(bodyspec->element);
  sim_builder_geom_t* geom = body->AddGeom(def);
  return &geom->spec;
}



// add camera to body
SIM_sCamera* sim_spec_addCamera(sim_spec_body_t* bodyspec, const sim_spec_default_t* defspec) {
  sim_builder_default_t* def = defspec ? static_cast<sim_builder_default_t*>(defspec->element) : 0;
  sim_builder_body_t* body = static_cast<sim_builder_body_t*>(bodyspec->element);
  SIM_CCamera* camera = body->AddCamera(def);
  return &camera->spec;
}



// add light to body
SIM_sLight* sim_spec_addLight(sim_spec_body_t* bodyspec, const sim_spec_default_t* defspec) {
  sim_builder_default_t* def = defspec ? static_cast<sim_builder_default_t*>(defspec->element) : 0;
  sim_builder_body_t* body = static_cast<sim_builder_body_t*>(bodyspec->element);
  SIM_CLight* light = body->AddLight(def);
  return &light->spec;
}



// add flex to model
SIM_sFlex* sim_spec_addFlex(sim_spec_t* s) {
  sim_builder_model_t* modelC = static_cast<sim_builder_model_t*>(s->element);
  SIM_CFlex* flex = modelC->AddFlex();
  return &flex->spec;
}



// add frame to body
SIM_sFrame* sim_spec_addFrame(sim_spec_body_t* bodyspec, SIM_sFrame* parentframe) {
  sim_builder_frame_t* parentframeC = 0;
  if (parentframe) {
    parentframeC = static_cast<sim_builder_frame_t*>(parentframe->element);
  }
  sim_builder_body_t* body = static_cast<sim_builder_body_t*>(bodyspec->element);
  sim_builder_frame_t* frameC = body->AddFrame(parentframeC);
  frameC->SetParent(body);
  return &frameC->spec;
}



// add mesh to model
SIM_sMesh* sim_spec_addMesh(sim_spec_t* s, const sim_spec_default_t* defspec) {
  sim_builder_default_t* def = defspec ? static_cast<sim_builder_default_t*>(defspec->element) : 0;
  sim_builder_model_t* modelC = static_cast<sim_builder_model_t*>(s->element);
  sim_builder_mesh_t* mesh = modelC->AddMesh(def);
  return &mesh->spec;
}



// add height field to model
SIM_sHField* sim_spec_addHField(sim_spec_t* s) {
  sim_builder_model_t* modelC = static_cast<sim_builder_model_t*>(s->element);
  SIM_CHField* heightField = modelC->AddHField();
  return &heightField->spec;
}



// add skin to model
SIM_sSkin* sim_spec_addSkin(sim_spec_t* s) {
  sim_builder_model_t* modelC = static_cast<sim_builder_model_t*>(s->element);
  SIM_CSkin* skin = modelC->AddSkin();
  return &skin->spec;
}



// add texture to model
SIM_sTexture* sim_spec_addTexture(sim_spec_t* s) {
  sim_builder_model_t* modelC = static_cast<sim_builder_model_t*>(s->element);
  sim_builder_texture_t* texture = modelC->AddTexture();
  return &texture->spec;
}



// add material to model
SIM_sMaterial* sim_spec_addMaterial(sim_spec_t* s, const sim_spec_default_t* defspec) {
  sim_builder_model_t* modelC = static_cast<sim_builder_model_t*>(s->element);
  sim_builder_default_t* def = defspec ? static_cast<sim_builder_default_t*>(defspec->element) : 0;
  SIM_CMaterial* material = modelC->AddMaterial(def);
  return &material->spec;
}

// Sets the vertices and normals of a mesh.
int sim_spec_makeMesh(SIM_sMesh* mesh, SIM_tMeshBuiltin builtin, double* params, int nparams) {
  sim_builder_mesh_t* meshC = static_cast<sim_builder_mesh_t*>(mesh->element);
  sim_builder_model_t* m = meshC->model;
  switch (builtin) {
    case SIM_MESH_BUILTIN_HEMISPHERE: {
      if (nparams != 1) {
        m->SetError(sim_builder_error_t(0, "Hemisphere mesh type requires 1 parameter"));
        return -1;
      }
      int subdiv = static_cast<int>(params[0]);
      if (subdiv < 0) {
        m->SetError(sim_builder_error_t(0, "Hemisphere resolution cannot be negative"));
        return -1;
      }
      if (subdiv > 10) {
        m->SetError(sim_builder_error_t(0, "Hemisphere resolution cannot be greater than 10"));
        return -1;
      }
      meshC->MakeHemisphere(subdiv, /*make_faces*/ true, /*make_cap*/ true);
      return 0;
    }

    case SIM_MESH_BUILTIN_SPHERE: {
      if (nparams != 1) {
        m->SetError(sim_builder_error_t(0, "Sphere mesh type requires 1 parameter"));
        return -1;
      }
      int subdiv = static_cast<int>(params[0]);
      if (subdiv < 0) {
        m->SetError(sim_builder_error_t(0, "Sphere subdivision cannot be negative"));
        return -1;
      }
      if (subdiv > 4) {
        m->SetError(sim_builder_error_t(0, "Sphere subdivision cannot be greater than 4"));
        return -1;
      }
      meshC->MakeSphere(subdiv, /*make_faces*/ true);
      return 0;
    }

    case SIM_MESH_BUILTIN_SUPERSPHERE: {
      if (nparams != 3) {
        m->SetError(sim_builder_error_t(0, "Supersphere mesh type requires 3 parameters"));
        return -1;
      }
      int res = static_cast<int>(params[0]);
      if (res < 3) {
        m->SetError(sim_builder_error_t(0, "Supersphere resolution must be greater than 2"));
        return -1;
      }
      double e = params[1];
      if (e < 0) {
        m->SetError(sim_builder_error_t(0, "Supersphere 'e' cannot be negative"));
        return -1;
      }
      double n = params[2];
      if (n < 0) {
        m->SetError(sim_builder_error_t(0, "Supersphere 'n' cannot be negative"));
        return -1;
      }
      meshC->MakeSupersphere(res, e, n);
      return 0;
    }

    case SIM_MESH_BUILTIN_SUPERTORUS: {
      if (nparams != 4) {
        m->SetError(sim_builder_error_t(0, "Supertorus mesh type requires 4 parameters"));
        return -1;
      }
      int res = static_cast<int>(params[0]);
      if (res < 3) {
        m->SetError(sim_builder_error_t(0, "Supertorus resolution must be greater than 3"));
        return -1;
      }
      double radius = params[1];
      if (radius <= 0 || radius > 1) {
        m->SetError(sim_builder_error_t(0, "Supertorus radius must be in (0, 1]"));
        return -1;
      }
      double s = params[2];
      if (s <= 0) {
        m->SetError(sim_builder_error_t(0, "Supertorus 's' must be greater than 0"));
        return -1;
      }
      double t = params[3];
      if (t <= 0) {
        m->SetError(sim_builder_error_t(0, "Supertorus 't' must be greater than 0"));
        return -1;
      }
      meshC->MakeSupertorus(res, radius, s, t);
      return 0;
    }

    case SIM_MESH_BUILTIN_WEDGE: {
      if (nparams != 5) {
        m->SetError(sim_builder_error_t(0, "Wedge builtin mesh types require 5 parameters"));
        return -1;
      }
      int resolution[2] = {static_cast<int>(params[0]),
                           static_cast<int>(params[1])};
      double fov[2] = {params[2], params[3]};
      double gamma = params[4];
      if (fov[0] <= 0 || fov[0] > 180) {
        m->SetError(sim_builder_error_t(0, "fov[0] must be a float between (0, 180] degrees"));
        return -1;
      }
      if (fov[1] <= 0 || fov[1] > 90) {
        m->SetError(sim_builder_error_t(0, "`fov[1]` must be a float between (0, 90] degrees"));
        return -1;
      }
      if (resolution[0] <= 0 || resolution[1] <= 0) {
        m->SetError(sim_builder_error_t(0, "Horizontal and vertical resolutions must be positive"));
        return -1;
      }
      if (gamma < 0 || gamma > 1) {
        m->SetError(sim_builder_error_t(0, "`gamma` must be a nonnegative float between [0, 1]"));
        return -1;
      }
      meshC->MakeWedge(resolution, fov, gamma);
      return 0;
    }

    case SIM_MESH_BUILTIN_PLATE: {
      if (nparams != 2) {
        m->SetError(sim_builder_error_t(0, "Plate builtin mesh type requires 2 parameters"));
        return -1;
      }
      int resolution[2] = {static_cast<int>(params[0]),
                           static_cast<int>(params[1])};
      if (resolution[0] <= 0 || resolution[1] <= 0) {
        m->SetError(sim_builder_error_t(0, "Horizontal and vertical resolutions must be positive"));
        return -1;
      }
      meshC->MakeRect(resolution);
      return 0;
    }

    case SIM_MESH_BUILTIN_CONE: {
      if (nparams != 2) {
        m->SetError(sim_builder_error_t(0, "Cone mesh type requires 2 parameters"));
        return -1;
      }
      int nedge = static_cast<int>(params[0]);
      meshC->MakeCone(nedge, params[1]);
      return 0;
    }

    default:
      m->SetError(sim_builder_error_t(0, "Unsupported mesh type"));
      return 1;
  }
}

// add pair to model
SIM_sPair* sim_spec_addPair(sim_spec_t* s, const sim_spec_default_t* defspec) {
  sim_builder_model_t* modelC = static_cast<sim_builder_model_t*>(s->element);
  sim_builder_default_t* def = defspec ? static_cast<sim_builder_default_t*>(defspec->element) : 0;
  SIM_CPair* pair = modelC->AddPair(def);
  return &pair->spec;
}



// add pair exclusion to model
SIM_sExclude* sim_spec_addExclude(sim_spec_t* s) {
  sim_builder_model_t* modelC = static_cast<sim_builder_model_t*>(s->element);
  SIM_CBodyPair* bodypair = modelC->AddExclude();
  return &bodypair->spec;
}



// add equality to model
SIM_sEquality* sim_spec_addEquality(sim_spec_t* s, const sim_spec_default_t* defspec) {
  sim_builder_model_t* modelC = static_cast<sim_builder_model_t*>(s->element);
  sim_builder_default_t* def = defspec ? static_cast<sim_builder_default_t*>(defspec->element) : 0;
  SIM_CEquality* equality = modelC->AddEquality(def);
  return &equality->spec;
}



// add tendon to model
SIM_sTendon* sim_spec_addTendon(sim_spec_t* s, const sim_spec_default_t* defspec) {
  sim_builder_model_t* modelC = static_cast<sim_builder_model_t*>(s->element);
  sim_builder_default_t* def = defspec ? static_cast<sim_builder_default_t*>(defspec->element) : 0;
  sim_builder_tendon_t* tendon = modelC->AddTendon(def);
  return &tendon->spec;
}



// wrap site using tendon
SIM_sWrap* sim_spec_wrapSite(SIM_sTendon* tendonspec, const char* name) {
  sim_builder_tendon_t* tendon = static_cast<sim_builder_tendon_t*>(tendonspec->element);
  tendon->WrapSite(name);
  return &tendon->path.back()->spec;
}



// wrap geom using tendon
SIM_sWrap* sim_spec_wrapGeom(SIM_sTendon* tendonspec, const char* name, const char* sidesite) {
  sim_builder_tendon_t* tendon = static_cast<sim_builder_tendon_t*>(tendonspec->element);
  tendon->WrapGeom(name, sidesite);
  return &tendon->path.back()->spec;
}



// wrap joint using tendon
SIM_sWrap* sim_spec_wrapJoint(SIM_sTendon* tendonspec, const char* name, double coef) {
  sim_builder_tendon_t* tendon = static_cast<sim_builder_tendon_t*>(tendonspec->element);
  tendon->WrapJoint(name, coef);
  return &tendon->path.back()->spec;
}



// wrap pulley using tendon
SIM_sWrap* sim_spec_wrapPulley(SIM_sTendon* tendonspec, double divisor) {
  sim_builder_tendon_t* tendon = static_cast<sim_builder_tendon_t*>(tendonspec->element);
  tendon->WrapPulley(divisor);
  return &tendon->path.back()->spec;
}



// add actuator to model
sim_spec_actuator_t* sim_spec_addActuator(sim_spec_t* s, const sim_spec_default_t* defspec) {
  sim_builder_model_t* modelC = static_cast<sim_builder_model_t*>(s->element);
  sim_builder_default_t* def = defspec ? static_cast<sim_builder_default_t*>(defspec->element) : 0;
  SIM_CActuator* actuator = modelC->AddActuator(def);
  return &actuator->spec;
}



// add sensor to model
SIM_sSensor* sim_spec_addSensor(sim_spec_t* s) {
  sim_builder_model_t* modelC = static_cast<sim_builder_model_t*>(s->element);
  SIM_CSensor* sensor = modelC->AddSensor();
  return &sensor->spec;
}



// add numeric to model
SIM_sNumeric* sim_spec_addNumeric(sim_spec_t* s) {
  sim_builder_model_t* modelC = static_cast<sim_builder_model_t*>(s->element);
  SIM_CNumeric* numeric = modelC->AddNumeric();
  return &numeric->spec;
}



// add text to model
SIM_sText* sim_spec_addText(sim_spec_t* s) {
  sim_builder_model_t* modelC = static_cast<sim_builder_model_t*>(s->element);
  SIM_CText* text = modelC->AddText();
  return &text->spec;
}



// add tuple to model
SIM_sTuple* sim_spec_addTuple(sim_spec_t* s) {
  sim_builder_model_t* modelC = static_cast<sim_builder_model_t*>(s->element);
  SIM_CTuple* tuple = modelC->AddTuple();
  return &tuple->spec;
}



// add keyframe to model
SIM_sKey* sim_spec_addKey(sim_spec_t* s) {
  sim_builder_model_t* modelC = static_cast<sim_builder_model_t*>(s->element);
  SIM_CKey* key = modelC->AddKey();
  return &key->spec;
}



// add plugin to model
SIM_sPlugin* sim_spec_addPlugin(sim_spec_t* s) {
  sim_builder_model_t* modelC = static_cast<sim_builder_model_t*>(s->element);
  sim_builder_plugin_t* plugin = modelC->AddPlugin();
  plugin->spec.element = static_cast<sim_spec_element_t*>(plugin);
  return &plugin->spec;
}



// add default to model
sim_spec_default_t* sim_spec_addDefault(sim_spec_t* s, const char* classname, const sim_spec_default_t* parent) {
  sim_builder_model_t* modelC = static_cast<sim_builder_model_t*>(s->element);
  sim_builder_default_t* parentC = parent ? static_cast<sim_builder_default_t*>(parent->element) :
                             static_cast<sim_builder_model_t*>(s->element)->Default();
  sim_builder_default_t* def = modelC->AddDefault(classname, parentC);
  if (def) {
    return &def->spec;
  } else {
    return nullptr;
  }
}



// set actuator to motor
const char* sim_spec_setToMotor(sim_spec_actuator_t* actuator) {
  // unit gain
  actuator->gainprm[0] = 1;

  // implied parameters
  actuator->dyntype = SIM_DYN_NONE;
  actuator->gaintype = SIM_GAIN_FIXED;
  actuator->biastype = SIM_BIAS_NONE;
  return "";
}



// set to position actuator
const char* sim_spec_setToPosition(sim_spec_actuator_t* actuator, double kp, double kv[1],
                              double dampratio[1], double timeconst[1], double inheritrange) {
  actuator->gainprm[0] = kp;
  actuator->biasprm[1] = -kp;

  // set biasprm[2]; negative: regular damping, positive: dampratio
  if (dampratio && kv) {
    return "kv and dampratio cannot both be defined";
  }

  if (kv) {
    if (*kv < 0) return "kv cannot be negative";
    actuator->biasprm[2] = -(*kv);
  }
  if (dampratio) {
    if (*dampratio < 0) return "dampratio cannot be negative";
    actuator->biasprm[2] = *dampratio;
  }
  if (timeconst) {
    if (*timeconst < 0) return "timeconst cannot be negative";
    actuator->dynprm[0] = *timeconst;
    actuator->dyntype = *timeconst == 0 ? SIM_DYN_NONE : SIM_DYN_FILTEREXACT;
  }
  actuator->inheritrange = inheritrange;

  if (inheritrange > 0) {
    if (actuator->ctrlrange[0] || actuator->ctrlrange[1]) {
      return "ctrlrange and inheritrange cannot both be defined";
    }
  }

  actuator->gaintype = SIM_GAIN_FIXED;
  actuator->biastype = SIM_BIAS_AFFINE;
  return "";
}



// Set to integrated velocity actuator.
const char* sim_spec_setToIntVelocity(sim_spec_actuator_t* actuator, double kp, double kv[1],
                                 double dampratio[1], double timeconst[1], double inheritrange) {
  sim_spec_setToPosition(actuator, kp, kv, dampratio, timeconst, inheritrange);
  actuator->dyntype = SIM_DYN_INTEGRATOR;
  actuator->actlimited = 1;

  if (inheritrange > 0) {
    if (actuator->actrange[0] || actuator->actrange[1]) {
      return "actrange and inheritrange cannot both be defined";
    }
  }
  return "";
}



// Set to velocity actuator.
const char* sim_spec_setToVelocity(sim_spec_actuator_t* actuator, double kv) {
  sim_math_internal_zerovec(actuator->biasprm, SIM_NBIAS);
  actuator->gainprm[0] = kv;
  actuator->biasprm[2] = -kv;
  actuator->dyntype = SIM_DYN_NONE;
  actuator->gaintype = SIM_GAIN_FIXED;
  actuator->biastype = SIM_BIAS_AFFINE;
  return "";
}



// Set to damper actuator.
const char* sim_spec_setToDamper(sim_spec_actuator_t* actuator, double kv) {
  sim_math_internal_zerovec(actuator->gainprm, SIM_NGAIN);
  actuator->gainprm[2] = -kv;
  actuator->ctrllimited = 1;
  actuator->dyntype = SIM_DYN_NONE;
  actuator->gaintype = SIM_GAIN_AFFINE;
  actuator->biastype = SIM_BIAS_NONE;

  if (kv < 0) {
    return "damping coefficient cannot be negative";
  }
  if (actuator->ctrlrange[0] < 0 || actuator->ctrlrange[1] < 0) {
    return "damper control range cannot be negative";
  }
  return "";
}



// Set to cylinder actuator.
const char* sim_spec_setToCylinder(sim_spec_actuator_t* actuator, double timeconst, double bias,
                       double area, double diameter) {
  actuator->dynprm[0] = timeconst;
  actuator->biasprm[0] = bias;
  actuator->gainprm[0] = area;
  if (diameter >= 0) {
    actuator->gainprm[0] = SIM_PI / 4 * diameter*diameter;
  }
  actuator->dyntype = SIM_DYN_FILTER;
  actuator->gaintype = SIM_GAIN_FIXED;
  actuator->biastype = SIM_BIAS_AFFINE;
  return "";
}



// Set to muscle actuator.
const char* sim_spec_setToMuscle(sim_spec_actuator_t* actuator, double timeconst[2], double tausmooth,
                            double range[2], double force, double scale, double lmin,
                            double lmax, double vmax, double fpmax, double fvmax) {
  // set muscle defaults if same as global defaults
  if (actuator->dynprm[0] == 1) actuator->dynprm[0] = 0.01;    // tau act
  if (actuator->dynprm[1] == 0) actuator->dynprm[1] = 0.04;    // tau deact
  if (actuator->gainprm[0] == 1) actuator->gainprm[0] = 0.75;  // range[0]
  if (actuator->gainprm[1] == 0) actuator->gainprm[1] = 1.05;  // range[1]
  if (actuator->gainprm[2] == 0) actuator->gainprm[2] = -1;    // force
  if (actuator->gainprm[3] == 0) actuator->gainprm[3] = 200;   // scale
  if (actuator->gainprm[4] == 0) actuator->gainprm[4] = 0.5;   // lmin
  if (actuator->gainprm[5] == 0) actuator->gainprm[5] = 1.6;   // lmax
  if (actuator->gainprm[6] == 0) actuator->gainprm[6] = 1.5;   // vmax
  if (actuator->gainprm[7] == 0) actuator->gainprm[7] = 1.3;   // fpmax
  if (actuator->gainprm[8] == 0) actuator->gainprm[8] = 1.2;   // fvmax

  if (tausmooth < 0)
    return "muscle tausmooth cannot be negative";

  actuator->dynprm[2] = tausmooth;
  if (timeconst[0] >= 0) actuator->dynprm[0] = timeconst[0];
  if (timeconst[1] >= 0) actuator->dynprm[1] = timeconst[1];
  if (range[0] >= 0) actuator->gainprm[0] = range[0];
  if (range[1] >= 0) actuator->gainprm[1] = range[1];
  if (force >= 0) actuator->gainprm[2] = force;
  if (scale >= 0) actuator->gainprm[3] = scale;
  if (lmin >= 0) actuator->gainprm[4] = lmin;
  if (lmax >= 0) actuator->gainprm[5] = lmax;
  if (vmax >= 0) actuator->gainprm[6] = vmax;
  if (fpmax >= 0) actuator->gainprm[7] = fpmax;
  if (fvmax >= 0) actuator->gainprm[8] = fvmax;

  // biasprm = gainprm
  for (int n=0; n < 9; n++) {
    actuator->biasprm[n] = actuator->gainprm[n];
  }

  actuator->dyntype = SIM_DYN_MUSCLE;
  actuator->gaintype = SIM_GAIN_MUSCLE;
  actuator->biastype = SIM_BIAS_MUSCLE;
  return "";
}



// Set to adhesion actuator.
const char* sim_spec_setToAdhesion(sim_spec_actuator_t* actuator, double gain) {
  actuator->gainprm[0] = gain;
  actuator->ctrllimited = 1;
  actuator->gaintype = SIM_GAIN_FIXED;
  actuator->biastype = SIM_BIAS_NONE;

  if (gain < 0)
    return "adhesion gain cannot be negative";
  if (actuator->ctrlrange[0] < 0 || actuator->ctrlrange[1] < 0)
    return "adhesion control range cannot be negative";
  return "";
}



// get spec from body
sim_spec_t* sim_spec_getSpec(sim_spec_element_t* element) {
  return &(static_cast<sim_builder_base_t*>(element)->model->spec);
}



// find spec (model asset) by name
sim_spec_t* sim_spec_findSpec(sim_spec_t* s, const char* name) {
  sim_builder_model_t* model = static_cast<sim_builder_model_t*>(s->element);
  return model->FindSpec(name);
}



// get default
sim_spec_default_t* sim_spec_getDefault(sim_spec_element_t* element) {
  sim_builder_model_t* model = static_cast<sim_builder_base_t*>(element)->model;
  std::string classname = static_cast<sim_builder_base_t*>(element)->classname;
  return &(model->def_map[classname]->spec);
}



// Find default with given name in model.
sim_spec_default_t* sim_spec_findDefault(sim_spec_t* s, const char* classname) {
  sim_builder_model_t* modelC = static_cast<sim_builder_model_t*>(s->element);
  sim_builder_default_t* cdef = modelC->FindDefault(classname);
  if (!cdef) {
    return nullptr;
  }
  return &cdef->spec;
}



// get default[0] from model
sim_spec_default_t* sim_spec_getSpecDefault(sim_spec_t* s) {
  sim_builder_model_t* modelC = static_cast<sim_builder_model_t*>(s->element);
  sim_builder_default_t* def = modelC->Default();
  if (!def) {
    return nullptr;
  }
  return &def->spec;
}



// find body in model by name
sim_spec_body_t* sim_spec_findBody(sim_spec_t* s, const char* name) {
  sim_spec_element_t* body = sim_spec_findElement(s, SIM_OBJ_BODY, name);
  return body ? &(static_cast<sim_builder_body_t*>(body)->spec) : nullptr;
}



// find element in spec by name
sim_spec_element_t* sim_spec_findElement(sim_spec_t* s, sim_obj_t type, const char* name) {
  sim_builder_model_t* model = static_cast<sim_builder_model_t*>(s->element);
  if (model->IsCompiled() && type != SIM_OBJ_FRAME) {
    return model->FindObject(type, std::string(name));  // fast lookup
  }
  switch (type) {
    case SIM_OBJ_BODY:
    case SIM_OBJ_SITE:
    case SIM_OBJ_GEOM:
    case SIM_OBJ_JOINT:
    case SIM_OBJ_CAMERA:
    case SIM_OBJ_LIGHT:
    case SIM_OBJ_FRAME:
      return model->FindTree(model->GetWorld(), type, std::string(name));  // recursive search
    case SIM_OBJ_TEXTURE:
      return model->FindAsset(std::string(name), model->Textures());  // check filename too
    case SIM_OBJ_MESH:
      return model->FindAsset(std::string(name), model->Meshes());  // check filename too
    default:
      return model->FindObject(type, std::string(name));  // always available
  }
}



// find child of a body by name
sim_spec_body_t* sim_spec_findChild(sim_spec_body_t* bodyspec, const char* name) {
  sim_builder_body_t* body = static_cast<sim_builder_body_t*>(bodyspec->element);
  sim_builder_base_t* child = body->FindObject(SIM_OBJ_BODY, std::string(name));
  return child ? &(static_cast<sim_builder_body_t*>(child)->spec) : nullptr;
}



// get parent body
sim_spec_body_t* sim_spec_getParent(sim_spec_element_t* element) {
  switch (element->elemtype) {
    case SIM_OBJ_BODY:
      return &(static_cast<sim_builder_body_t*>(element)->GetParent()->spec);
    case SIM_OBJ_FRAME:
      return &(static_cast<sim_builder_frame_t*>(element)->GetParent()->spec);
    case SIM_OBJ_JOINT:
      return &(static_cast<sim_builder_joint_t*>(element)->GetParent()->spec);
    case SIM_OBJ_GEOM:
      return &(static_cast<sim_builder_geom_t*>(element)->GetParent()->spec);
    case SIM_OBJ_SITE:
      return &(static_cast<sim_builder_site_t*>(element)->GetParent()->spec);
    case SIM_OBJ_CAMERA:
      return &(static_cast<SIM_CCamera*>(element)->GetParent()->spec);
    case SIM_OBJ_LIGHT:
      return &(static_cast<SIM_CLight*>(element)->GetParent()->spec);
    default:
      return nullptr;
  }
}



// get parent frame
SIM_sFrame* sim_spec_getFrame(sim_spec_element_t* element) {
  sim_builder_base_t* base = static_cast<sim_builder_base_t*>(element);
  switch (element->elemtype) {
    case SIM_OBJ_BODY:
    case SIM_OBJ_FRAME:
    case SIM_OBJ_JOINT:
    case SIM_OBJ_GEOM:
    case SIM_OBJ_SITE:
    case SIM_OBJ_CAMERA:
    case SIM_OBJ_LIGHT:
      return base->frame ? &(base->frame->spec) : nullptr;
    default:
      return nullptr;
  }
}



// find frame by name
SIM_sFrame* sim_spec_findFrame(sim_spec_t* s, const char* name) {
  sim_spec_element_t* frame = sim_spec_findElement(s, SIM_OBJ_FRAME, name);
  return frame ? &(static_cast<sim_builder_frame_t*>(frame)->spec) : nullptr;
}



// set frame
int sim_spec_setFrame(sim_spec_element_t* dest, SIM_sFrame* frame) {
  if (!frame || !dest) {
    return -1;
  }
  sim_builder_frame_t* frameC = static_cast<sim_builder_frame_t*>(frame->element);
  sim_builder_base_t* baseC = static_cast<sim_builder_base_t*>(dest);
  try {
    baseC->SetFrame(frameC);
    return 0;
  } catch (sim_builder_error_t& e) {
    baseC->model->SetError(e);
    return -1;
  }
}



// Resolve alternative orientations.
const char* sim_spec_resolveOrientation(double quat[4], sim_byte_t degree, const char* sequence,
                                   const SIM_sOrientation* orientation) {
  return ResolveOrientation(quat, degree, sequence, *orientation);
}



// Transform body into a frame.
SIM_sFrame* sim_spec_bodyToFrame(sim_spec_body_t** body) {
  sim_builder_body_t* bodyC = static_cast<sim_builder_body_t*>((*body)->element);
  sim_builder_frame_t* frameC = bodyC->ToFrame();
  *bodyC->model -= (*body)->element;
  *body = nullptr;
  return &frameC->spec;
}

void sim_spec_setUserValue(sim_spec_element_t* element, const char* key, const void* data) {
  sim_spec_setUserValueWithCleanup(element, key, data, nullptr);
}

// set user payload
void sim_spec_setUserValueWithCleanup(sim_spec_element_t* element, const char* key,
                                 const void* data,
                                 void (*cleanup)(const void*)) {
  sim_builder_base_t* baseC = static_cast<sim_builder_base_t*>(element);
  baseC->SetUserValue(key, data, cleanup);
}

// return user payload or NULL if none found
const void* sim_spec_getUserValue(sim_spec_element_t* element, const char* key) {
  sim_builder_base_t* baseC = static_cast<sim_builder_base_t*>(element);
  return baseC->GetUserValue(key);
}



// delete user payload
void sim_spec_deleteUserValue(sim_spec_element_t* element, const char* key) {
  sim_builder_base_t* baseC = static_cast<sim_builder_base_t*>(element);
  baseC->DeleteUserValue(key);
}



// return sensor dimension
int sim_spec_sensorDim(const SIM_sSensor* sensor) {
  switch (sensor->type) {
  case SIM_SENS_TOUCH:
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
    return sensor->intprm[2] * sim_math_condataSize(sensor->intprm[0]);

  case SIM_SENS_TACTILE:
    return 3 * static_cast<const sim_builder_mesh_t*>(
                   static_cast<SIM_CSensor*>(sensor->element)->get_obj())
                   ->nvert();

  case SIM_SENS_RANGEFINDER:
    {
      int size = sim_math_raydataSize(sensor->intprm[0]);
      int num_rays = 1;
      if (sensor->objtype == SIM_OBJ_CAMERA) {
        const SIM_CCamera* camera = static_cast<const SIM_CCamera*>(
            static_cast<SIM_CSensor*>(sensor->element)->get_obj());
        num_rays = camera->spec.resolution[0] * camera->spec.resolution[1];
      }
      return size * num_rays;
    }

  case SIM_SENS_USER:
    return sensor->dim;

  case SIM_SENS_PLUGIN:
    return 0;  // to be filled in by plugin
  }
  return -1;
}



// get id
int sim_spec_getId(sim_spec_element_t* element) {
  if (!element) {
    return -1;
  }
  return static_cast<sim_builder_base_t*>(element)->id;
}



// set default
void sim_spec_setDefault(sim_spec_element_t* element, const sim_spec_default_t* defspec) {
  sim_builder_base_t* baseC = static_cast<sim_builder_base_t*>(element);
  baseC->classname = static_cast<sim_builder_default_t*>(defspec->element)->name;
}



// return first child of selected type
sim_spec_element_t* sim_spec_firstChild(sim_spec_body_t* body, sim_obj_t type, int recurse) {
  sim_builder_body_t* bodyC = static_cast<sim_builder_body_t*>(body->element);
  try {
    return bodyC->NextChild(NULL, type, recurse);
  } catch (sim_builder_error_t& e) {
    bodyC->model->SetError(e);
    return nullptr;
  }
}



// return body's next child; return NULL if child is last
sim_spec_element_t* sim_spec_nextChild(sim_spec_body_t* body, sim_spec_element_t* child, int recurse) {
  sim_builder_body_t* bodyC = static_cast<sim_builder_body_t*>(body->element);
  try {
    return bodyC->NextChild(child, child->elemtype, recurse);
  } catch(sim_builder_error_t& e) {
    bodyC->model->SetError(e);
    return nullptr;
  }
}



// return spec's first element of selected type
sim_spec_element_t* sim_spec_firstElement(sim_spec_t* s, sim_obj_t type) {
  sim_builder_model_t* modelC = static_cast<sim_builder_model_t*>(s->element);
  return modelC->NextObject(NULL, type);
}



// return spec's next element; return NULL if element is last
sim_spec_element_t* sim_spec_nextElement(sim_spec_t* s, sim_spec_element_t* element) {
  sim_builder_model_t* modelC = static_cast<sim_builder_model_t*>(s->element);
  return modelC->NextObject(element);
}



sim_spec_element_t* sim_spec_getWrapTarget(SIM_sWrap* wrap) {
  SIM_CWrap* cwrap = static_cast<SIM_CWrap*>(wrap->element);
  sim_obj_t type = SIM_OBJ_UNKNOWN;
  switch (cwrap->Type()) {
    case SIM_WRAP_SPHERE:
    case SIM_WRAP_CYLINDER:
      type = SIM_OBJ_GEOM;
      break;
    case SIM_WRAP_SITE:
      type = SIM_OBJ_SITE;
      break;
    case SIM_WRAP_JOINT:
      type = SIM_OBJ_JOINT;
      break;
    case SIM_WRAP_PULLEY:
      // Pulleys have no target.
      return nullptr;
    default:
      return nullptr;
  }
  sim_spec_t* spec = sim_spec_getSpec(wrap->element);
  sim_spec_element_t* target = sim_spec_findElement(spec, type, cwrap->name.c_str());
  return target;
}



SIM_sSite* sim_spec_getWrapSideSite(SIM_sWrap* wrap) {
  SIM_CWrap* cwrap = static_cast<SIM_CWrap*>(wrap->element);
  // only sphere and cylinder (geoms) have side sites
  if ((cwrap->Type() != SIM_WRAP_SPHERE &&
      cwrap->Type() != SIM_WRAP_CYLINDER) ||
      cwrap->sidesite.empty()) {
    return nullptr;
  }

  sim_spec_t* spec = sim_spec_getSpec(wrap->element);
  sim_spec_element_t* site = sim_spec_findElement(spec, SIM_OBJ_SITE, cwrap->sidesite.c_str());
  if (site == nullptr) {
    sim_warning("Could not find side site %s for wrap %s in spec",
                cwrap->sidesite.c_str(), cwrap->name.c_str());
    return nullptr;
  }
  return sim_spec_asSite(site);
}



double sim_spec_getWrapDivisor(SIM_sWrap* wrap) {
  SIM_CWrap* cwrap = static_cast<SIM_CWrap*>(wrap->element);
  if (cwrap->Type() != SIM_WRAP_PULLEY) {
    sim_warning("Querying divisor attribute of non-pulley wrap: %s", cwrap->name.c_str());
    return 1.0;
  }
  return cwrap->prm;
}



double sim_spec_getWrapCoef(SIM_sWrap* wrap) {
  SIM_CWrap* cwrap = static_cast<SIM_CWrap*>(wrap->element);
  if (cwrap->Type() != SIM_WRAP_JOINT) {
    sim_warning("Querying coef attribute of non-joint wrap: %s", cwrap->name.c_str());
    return 1.0;
  }
  return cwrap->prm;
}



// return body given sim_spec_element_t
sim_spec_body_t* sim_spec_asBody(sim_spec_element_t* element) {
  if (element && element->elemtype == SIM_OBJ_BODY) {
    return &(static_cast<sim_builder_body_t*>(element)->spec);
  }
  return nullptr;
}



// return geom given sim_spec_element_t
sim_spec_geom_t* sim_spec_asGeom(sim_spec_element_t* element) {
  if (element && element->elemtype == SIM_OBJ_GEOM) {
    return &(static_cast<sim_builder_geom_t*>(element)->spec);
  }
  return nullptr;
}



// return joint given sim_spec_element_t
SIM_sJoint* sim_spec_asJoint(sim_spec_element_t* element) {
  if (element && element->elemtype == SIM_OBJ_JOINT) {
    return &(static_cast<sim_builder_joint_t*>(element)->spec);
  }
  return nullptr;
}



// Return site given sim_spec_element_t
SIM_sSite* sim_spec_asSite(sim_spec_element_t* element) {
  if (element && element->elemtype == SIM_OBJ_SITE) {
    return &(static_cast<sim_builder_site_t*>(element)->spec);
  }
  return nullptr;
}



// return camera given sim_spec_element_t
SIM_sCamera* sim_spec_asCamera(sim_spec_element_t* element) {
  if (element && element->elemtype == SIM_OBJ_CAMERA) {
    return &(static_cast<SIM_CCamera*>(element)->spec);
  }
  return nullptr;
}



// return light given sim_spec_element_t
SIM_sLight* sim_spec_asLight(sim_spec_element_t* element) {
  if (element && element->elemtype == SIM_OBJ_LIGHT) {
    return &(static_cast<SIM_CLight*>(element)->spec);
  }
  return nullptr;
}



// return frame given sim_spec_element_t
SIM_sFrame* sim_spec_asFrame(sim_spec_element_t* element) {
  if (element && element->elemtype == SIM_OBJ_FRAME) {
    return &(static_cast<sim_builder_frame_t*>(element)->spec);
  }
  return nullptr;
}



// return actuator given sim_spec_element_t
sim_spec_actuator_t* sim_spec_asActuator(sim_spec_element_t* element) {
  if (element && element->elemtype == SIM_OBJ_ACTUATOR) {
    return &(static_cast<SIM_CActuator*>(element)->spec);
  }
  return nullptr;
}



// return sensor given sim_spec_element_t
SIM_sSensor* sim_spec_asSensor(sim_spec_element_t* element) {
  if (element && element->elemtype == SIM_OBJ_SENSOR) {
    return &(static_cast<SIM_CSensor*>(element)->spec);
  }
  return nullptr;
}



// return flex given sim_spec_element_t
SIM_sFlex* sim_spec_asFlex(sim_spec_element_t* element) {
  if (element && element->elemtype == SIM_OBJ_FLEX) {
    return &(static_cast<SIM_CFlex*>(element)->spec);
  }
  return nullptr;
}



// return pair given sim_spec_element_t
SIM_sPair* sim_spec_asPair(sim_spec_element_t* element) {
  if (element && element->elemtype == SIM_OBJ_PAIR) {
    return &(static_cast<SIM_CPair*>(element)->spec);
  }
  return nullptr;
}



// return equality given sim_spec_element_t
SIM_sEquality* sim_spec_asEquality(sim_spec_element_t* element) {
  if (element && element->elemtype == SIM_OBJ_EQUALITY) {
    return &(static_cast<SIM_CEquality*>(element)->spec);
  }
  return nullptr;
}



// return exclude given sim_spec_element_t
SIM_sExclude* sim_spec_asExclude(sim_spec_element_t* element) {
  if (element && element->elemtype == SIM_OBJ_EXCLUDE) {
    return &(static_cast<SIM_CBodyPair*>(element)->spec);
  }
  return nullptr;
}



// return tendon given sim_spec_element_t
SIM_sTendon* sim_spec_asTendon(sim_spec_element_t* element) {
  if (element && element->elemtype == SIM_OBJ_TENDON) {
    return &(static_cast<sim_builder_tendon_t*>(element)->spec);
  }
  return nullptr;
}



// return numeric given sim_spec_element_t
SIM_sNumeric* sim_spec_asNumeric(sim_spec_element_t* element) {
  if (element && element->elemtype == SIM_OBJ_NUMERIC) {
    return &(static_cast<SIM_CNumeric*>(element)->spec);
  }
  return nullptr;
}



// return text given sim_spec_element_t
SIM_sText* sim_spec_asText(sim_spec_element_t* element) {
  if (element && element->elemtype == SIM_OBJ_TEXT) {
    return &(static_cast<SIM_CText*>(element)->spec);
  }
  return nullptr;
}



// return tuple given sim_spec_element_t
SIM_sTuple* sim_spec_asTuple(sim_spec_element_t* element) {
  if (element && element->elemtype == SIM_OBJ_TUPLE) {
    return &(static_cast<SIM_CTuple*>(element)->spec);
  }
  return nullptr;
}



// return key given sim_spec_element_t
SIM_sKey* sim_spec_asKey(sim_spec_element_t* element) {
  if (element && element->elemtype == SIM_OBJ_KEY) {
    return &(static_cast<SIM_CKey*>(element)->spec);
  }
  return nullptr;
}



// return mesh given sim_spec_element_t
SIM_sMesh* sim_spec_asMesh(sim_spec_element_t* element) {
  if (element && element->elemtype == SIM_OBJ_MESH) {
    return &(static_cast<sim_builder_mesh_t*>(element)->spec);
  }
  return nullptr;
}



// return hfield given sim_spec_element_t
SIM_sHField* sim_spec_asHField(sim_spec_element_t* element) {
  if (element && element->elemtype == SIM_OBJ_HFIELD) {
    return &(static_cast<SIM_CHField*>(element)->spec);
  }
  return nullptr;
}



// return skin given sim_spec_element_t
SIM_sSkin* sim_spec_asSkin(sim_spec_element_t* element) {
  if (element && element->elemtype == SIM_OBJ_SKIN) {
    return &(static_cast<SIM_CSkin*>(element)->spec);
  }
  return nullptr;
}



// return texture given sim_spec_element_t
SIM_sTexture* sim_spec_asTexture(sim_spec_element_t* element) {
  if (element && element->elemtype == SIM_OBJ_TEXTURE) {
    return &(static_cast<sim_builder_texture_t*>(element)->spec);
  }
  return nullptr;
}



// return material given sim_spec_element_t
SIM_sMaterial* sim_spec_asMaterial(sim_spec_element_t* element) {
  if (element && element->elemtype == SIM_OBJ_MATERIAL) {
    return &(static_cast<SIM_CMaterial*>(element)->spec);
  }
  return nullptr;
}



// return plugin given sim_spec_element_t
SIM_sPlugin* sim_spec_asPlugin(sim_spec_element_t* element) {
  if (element && element->elemtype == SIM_OBJ_PLUGIN) {
    return &(static_cast<sim_builder_plugin_t*>(element)->spec);
  }
  return nullptr;
}



// set element name
int sim_spec_set_name(sim_spec_element_t* element, const char* name) {
  if (element->elemtype == SIM_OBJ_DEFAULT) {
    sim_builder_default_t* def = static_cast<sim_builder_default_t*>(element);
    def->name = std::string(name);
    return 0;
  }
  sim_builder_base_t* baseC = static_cast<sim_builder_base_t*>(element);
  baseC->name = std::string(name);
  try {
    baseC->model->CheckRepeat(element->elemtype);
  } catch (sim_builder_error_t& e) {
    baseC->model->SetError(e);
    return -1;
  }
  return 0;
}



// copy buffer to destination buffer
void sim_spec_setBuffer(SIM_ByteVec* dest, const void* array, int size) {
  const std::byte* buffer = static_cast<const std::byte*>(array);
  dest->clear();
  dest->reserve(size);
  std::copy_n(buffer, size, std::back_inserter(*dest));
}



// set string
void sim_spec_set_string(sim_string_t* dest, const char* text) {
  std::string* str = static_cast<std::string*>(dest);
  *str = std::string(text);
}



// Set specific entry in destination string vector.
sim_byte_t sim_spec_setInStringVec(SIM_StringVec* dest, int i, const char* text) {
  if (dest->size() <= i) {
    sim_error("Requested index in sim_spec_setInStringVec is out of bounds");
    return 0;
  }
  dest->at(i) = std::string(text);
  return 1;
}



// split text and copy into string array
void sim_spec_setStringVec(SIM_StringVec* dest, const char* text) {
  std::vector<std::string>* v = static_cast<std::vector<std::string>*>(dest);
  *v = StringToVector<std::string>(text);
}



// add text entry to destination string vector
void sim_spec_appendString(SIM_StringVec* dest, const char* text) {
  dest->push_back(std::string(text));
}


// copy int array to vector
void sim_spec_setInt(SIM_IntVec* dest, const int* array, int size) {
  dest->assign(size, 0.0);
  for (int i = 0; i < size; ++i) {
    (*dest)[i] = array[i];
  }
}



// append int array to vector of arrays
void sim_spec_appendIntVec(SIM_IntVecVec* dest, const int* array, int size) {
  dest->push_back(std::vector<int>(array, array + size));
}



// copy float array to vector
void sim_spec_setFloat(SIM_FloatVec* dest, const float* array, int size) {
  dest->assign(size, 0.0);
  for (int i = 0; i < size; ++i) {
    (*dest)[i] = array[i];
  }
}




// append float array to vector of arrays
void sim_spec_appendFloatVec(SIM_FloatVecVec* dest, const float* array, int size) {
  dest->push_back(std::vector<float>(array, array + size));
}



// copy double array to vector
void sim_spec_setDouble(SIM_DoubleVec* dest, const double* array, int size) {
  dest->assign(size, 0.0);
  for (int i = 0; i < size; ++i) {
    (*dest)[i] = array[i];
  }
}



// get name
sim_string_t* sim_spec_getName(sim_spec_element_t* element) {
  if (element->elemtype == SIM_OBJ_DEFAULT) {
    return &(static_cast<sim_builder_default_t*>(element)->name);
  }
  return &(static_cast<sim_builder_base_t*>(element)->name);
}



// get string
const char* sim_spec_getString(const sim_string_t* source) {
  return source->c_str();
}



// get double array
const double* sim_spec_getDouble(const SIM_DoubleVec* source, int* size) {
  if (size) {
    *size = source->size();
  }
  return source->data();
}

int sim_spec_getWrapNum(const SIM_sTendon* tendonspec) {
  sim_builder_tendon_t* tendon = static_cast<sim_builder_tendon_t*>(tendonspec->element);
  return tendon->NumWraps();
}

SIM_sWrap* sim_spec_getWrap(const SIM_sTendon* tendonspec, int i) {
  sim_builder_tendon_t* tendon = static_cast<sim_builder_tendon_t*>(tendonspec->element);
  if (i < 0 || i >= tendon->NumWraps()) {
    sim_error("Wrap index out of range (0, %d)", tendon->NumWraps());
  }
  return &const_cast<SIM_CWrap*>(tendon->GetWrap(i))->spec;
}

// set plugin attributes
void sim_spec_setPluginAttributes(SIM_sPlugin* plugin, void* attributes) {
  sim_builder_plugin_t* pluginC = static_cast<sim_builder_plugin_t*>(plugin->element);
  std::map<std::string, std::string, std::less<> >* config_attribs =
    reinterpret_cast<std::map<std::string, std::string, std::less<> >*>(attributes);
  pluginC->config_attribs = std::move(*config_attribs);
}



// get plugin attributes
const void* sim_spec_getPluginAttributes(const SIM_sPlugin* plugin) {
  sim_builder_plugin_t* pluginC = static_cast<sim_builder_plugin_t*>(plugin->element);
  return &pluginC->config_attribs;
}



// -------------------------- GLOBAL ASSET CACHE -------------------------------

// get the capacity of the asset cache in bytes
size_t sim_getCacheCapacity(const SIM_Cache* cache) {
  if (cache) {
    const SIM_CCache* ccache = reinterpret_cast<const SIM_CCache*>(cache->impl_);
    if (ccache) {
      return ccache->Capacity();
    }
  }
  return 0;
}


// set the capacity of the asset cache in bytes (0 to disable)
size_t sim_setCacheCapacity(SIM_Cache* cache, size_t size) {
  if (cache) {
    SIM_CCache* ccache = reinterpret_cast<SIM_CCache*>(cache->impl_);
    if (ccache) {
      ccache->SetCapacity(size);
      return ccache->Capacity();
    }
  }
  return 0;
}


// get the current size of the asset cache in bytes
size_t sim_getCacheSize(const SIM_Cache* cache) {
  if (cache) {
    const SIM_CCache* ccache = reinterpret_cast<const SIM_CCache*>(cache->impl_);
    if (ccache) {
      return ccache->Size();
    }
  }
  return 0;
}


// clear the asset cache
void sim_clearCache(SIM_Cache* cache) {
  if (cache) {
    SIM_CCache* ccache = reinterpret_cast<SIM_CCache*>(cache->impl_);
    if (ccache) {
      ccache->Reset();
    }
  }
}

// get the internal asset cache used by the compiler
SIM_Cache* sim_getCache() {
  static SIM_Cache cache_cwrapper = []() {
    SIM_Cache c = {0};
    // SIM_CCache is not trivially destructible and so the global cache needs to
    // allocated on the heap
    if constexpr (kGlobalCacheSize != 0) {
      static SIM_CCache* cache = new (std::nothrow) SIM_CCache(kGlobalCacheSize);
      c.impl_ = cache->Capacity() > 0 ? cache : nullptr;
    }
    return c;
  }();
  return &cache_cwrapper;
}
