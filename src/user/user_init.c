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

#include <string.h>
#include <simcore/SIM_model.h>
#include <simcore/SIM_spec.h>
#include "engine/engine_init.h"
#include "engine/engine_io.h"
#include "user/user_api.h"


// default model attributes
void sim_spec_defaultSpec(sim_spec_t* spec) {
  memset(spec, 0, sizeof(sim_spec_t));

  // default statistics
  spec->stat.meaninertia = SIM_NAN;
  spec->stat.meanmass = SIM_NAN;
  spec->stat.meansize = SIM_NAN;
  spec->stat.extent = SIM_NAN;
  spec->stat.center[0] = SIM_NAN;

  // compiler settings
  spec->compiler.autolimits = 1;
  spec->compiler.settotalmass = -1;
  spec->compiler.degree = 1;
  spec->compiler.eulerseq[0] = 'x';
  spec->compiler.eulerseq[1] = 'y';
  spec->compiler.eulerseq[2] = 'z';
  spec->compiler.usethread = 1;
  spec->compiler.inertiafromgeom = SIM_INERTIAFROMGEOM_AUTO;
  spec->compiler.inertiagrouprange[1] = SIM_NGROUP-1;
  spec->compiler.saveinertial = 0;
  sim_defaultLROpt(&spec->compiler.LRopt);

  // engine data
  sim_defaultOption(&spec->option);
  spec->memory = -1;
  spec->njmax = -1;
  spec->nconmax = -1;
  spec->nstack = -1;

  // user fields
  spec->nuser_body = -1;
  spec->nuser_jnt = -1;
  spec->nuser_geom = -1;
  spec->nuser_site = -1;
  spec->nuser_cam = -1;
  spec->nuser_tendon = -1;
  spec->nuser_actuator = -1;
  spec->nuser_sensor = -1;
}


// default orientation attributes
void sim_spec_defaultOrientation(SIM_sOrientation* orient) {
  memset(orient, 0, sizeof(SIM_sOrientation));
}


// default body attributes
void sim_spec_defaultBody(sim_spec_body_t* body) {
  memset(body, 0, sizeof(sim_spec_body_t));

  // body frame
  body->quat[0] = 1;

  // inertial frame
  body->ipos[0] = SIM_NAN;
  body->iquat[0] = 1;
  body->fullinertia[0] = SIM_NAN;
}


// default frame attributes
void sim_spec_defaultFrame(SIM_sFrame* frame) {
  memset(frame, 0, sizeof(SIM_sFrame));
  frame->quat[0] = 1;
}


// default joint attributes
void sim_spec_defaultJoint(SIM_sJoint* joint) {
  memset(joint, 0, sizeof(SIM_sJoint));
  joint->type = SIM_JNT_HINGE;
  joint->axis[2] = 1;
  joint->limited = SIM_LIMITED_AUTO;
  joint->actfrclimited = SIM_LIMITED_AUTO;
  joint->align = SIM_ALIGNFREE_AUTO;
  sim_defaultSolRefImp(joint->solref_limit, joint->solimp_limit);
  sim_defaultSolRefImp(joint->solref_friction, joint->solimp_friction);
}


// default geom attributes
void sim_spec_defaultGeom(sim_spec_geom_t* geom) {
  memset(geom, 0, sizeof(sim_spec_geom_t));

  // type
  geom->type = SIM_GEOM_SPHERE;

  // frame
  geom->quat[0] = 1;
  geom->fromto[0] = SIM_NAN;

  // contact-related
  geom->contype = 1;
  geom->conaffinity = 1;
  geom->condim = 3;
  geom->friction[0] = 1;
  geom->friction[1] = 0.005;
  geom->friction[2] = 0.0001;
  geom->solmix = 1.0;
  sim_defaultSolRefImp(geom->solref, geom->solimp);

  // inertia-related
  geom->mass = SIM_NAN;
  geom->density = 1000;  // water density (1000 Kg / m^3)
  geom->typeinertia = SIM_INERTIA_VOLUME;

  // color
  geom->rgba[0] = geom->rgba[1] = geom->rgba[2] = 0.5f;
  geom->rgba[3] = 1.0f;

  // fluid forces
  geom->fluid_coefs[0] = 0.5;   // blunt drag
  geom->fluid_coefs[1] = 0.25;  // slender drag
  geom->fluid_coefs[2] = 1.5;   // angular drag
  geom->fluid_coefs[3] = 1.0;   // kutta lift
  geom->fluid_coefs[4] = 1.0;   // magnus lift

  // other
  geom->fitscale = 1;
}


// default site attributes
void sim_spec_defaultSite(SIM_sSite* site) {
  memset(site, 0, sizeof(SIM_sSite));

  // type
  site->type = SIM_GEOM_SPHERE;

  // frame
  site->quat[0] = 1;
  site->size[0] = site->size[1] = site->size[2] = 0.005;
  site->fromto[0] = SIM_NAN;

  // color
  site->rgba[0] = site->rgba[1] = site->rgba[2] = 0.5f;
  site->rgba[3] = 1.0f;
}


// default cam attributes
void sim_spec_defaultCamera(SIM_sCamera* cam) {
  memset(cam, 0, sizeof(SIM_sCamera));

  // mode
  cam->mode = SIM_CAMLIGHT_FIXED;

  // extrinsics
  cam->quat[0] = 1;

  // intrinsics
  cam->fovy = 45;
  cam->ipd = 0.068;
  cam->resolution[0] = cam->resolution[1] = 1;
  cam->output = SIM_CAMOUT_RGB;
}


// default light attributes
void sim_spec_defaultLight(SIM_sLight* light) {
  memset(light, 0, sizeof(SIM_sLight));

  // mode
  light->mode = SIM_CAMLIGHT_FIXED;

  // extrinsics
  light->dir[2] = -1;

  // intrinsics
  light->castshadow = 1;
  light->bulbradius = 0.02;
  light->intensity = 0.0;
  light->range = 10.0;
  light->active = 1;
  light->attenuation[0] = 1;
  light->cutoff = 45;
  light->exponent = 10;
  light->diffuse[0] = light->diffuse[1] = light->diffuse[2] = 0.7;
  light->specular[0] = light->specular[1] = light->specular[2] = 0.3;
}


// default flex attributes
void sim_spec_defaultFlex(SIM_sFlex* flex) {
  memset(flex, 0, sizeof(SIM_sFlex));

  // set contact defaults
  flex->contype = 1;
  flex->conaffinity = 1;
  flex->condim = 3;
  flex->friction[0] = 1;
  flex->friction[1] = 0.005;
  flex->friction[2] = 0.0001;
  flex->solmix = 1.0;
  sim_defaultSolRefImp(flex->solref, flex->solimp);

  // set other defaults
  flex->dim = 2;
  flex->radius = 0.005;
  flex->internal = 0;
  flex->selfcollide = SIM_FLEXSELF_AUTO;
  flex->activelayers = 1;
  flex->rgba[0] = flex->rgba[1] = flex->rgba[2] = 0.5f;
  flex->rgba[3] = 1.0f;
  flex->thickness = -1;
}


// default mesh attributes
void sim_spec_defaultMesh(SIM_sMesh* mesh) {
  memset(mesh, 0, sizeof(SIM_sMesh));
  mesh->refquat[0] = 1;
  mesh->scale[0] = mesh->scale[1] = mesh->scale[2] = 1;
  mesh->maxhullvert = -1;
  mesh->inertia = SIM_MESH_INERTIA_LEGACY;
}


// default height field attributes
void sim_spec_defaultHField(SIM_sHField* hfield) {
  memset(hfield, 0, sizeof(SIM_sHField));
}


// default skin attributes
void sim_spec_defaultSkin(SIM_sSkin* skin) {
  memset(skin, 0, sizeof(SIM_sSkin));
  skin->rgba[0] = skin->rgba[1] = skin->rgba[2] = 0.5f;
  skin->rgba[3] = 1.0f;
}


// default texture attributes
void sim_spec_defaultTexture(SIM_sTexture* texture) {
  memset(texture, 0, sizeof(SIM_sTexture));
  texture->type = SIM_TEXTURE_CUBE;
  texture->colorspace = SIM_COLORSPACE_AUTO;
  texture->rgb1[0] = texture->rgb1[1] = texture->rgb1[2] = 0.8;
  texture->rgb2[0] = texture->rgb2[1] = texture->rgb2[2] = 0.5;
  texture->random = 0.01;
  texture->gridsize[0] = texture->gridsize[1] = 1;
  texture->nchannel = 3;
  char defaultlayout[sizeof(texture->gridlayout)] = "............";
  strncpy(texture->gridlayout, defaultlayout, sizeof(texture->gridlayout));
}


// default material attributes
void sim_spec_defaultMaterial(SIM_sMaterial* material) {
  memset(material, 0, sizeof(SIM_sMaterial));
  material->texrepeat[0] = material->texrepeat[1] = 1;
  material->specular = 0.5;
  material->shininess = 0.5;
  material->metallic = -1.0;
  material->roughness = -1.0;
  material->rgba[0] = material->rgba[1] = material->rgba[2] = material->rgba[3] = 1;
}


// default pair attributes
void sim_spec_defaultPair(SIM_sPair* pair) {
  memset(pair, 0, sizeof(SIM_sPair));
  pair->condim = 3;
  sim_defaultSolRefImp(pair->solref, pair->solimp);
  pair->friction[0] = 1;
  pair->friction[1] = 1;
  pair->friction[2] = 0.005;
  pair->friction[3] = 0.0001;
  pair->friction[4] = 0.0001;
}


// default equality attributes
void sim_spec_defaultEquality(SIM_sEquality* equality) {
  memset(equality, 0, sizeof(SIM_sEquality));
  equality->type = SIM_EQ_CONNECT;
  equality->active = 1;
  sim_defaultSolRefImp(equality->solref, equality->solimp);
  equality->data[1] = 1;
  equality->data[10] = 1;  // torque:force ratio
}


// default tendon attributes
void sim_spec_defaultTendon(SIM_sTendon* tendon) {
  memset(tendon, 0, sizeof(SIM_sTendon));
  tendon->limited = SIM_LIMITED_AUTO;
  tendon->springlength[0] = tendon->springlength[1] = -1;
  sim_defaultSolRefImp(tendon->solref_limit, tendon->solimp_limit);
  sim_defaultSolRefImp(tendon->solref_friction, tendon->solimp_friction);
  tendon->width = 0.003;
  tendon->rgba[0] = tendon->rgba[1] = tendon->rgba[2] = 0.5f;
  tendon->rgba[3] = 1.0f;
}


// default actuator attributes
void sim_spec_defaultActuator(sim_spec_actuator_t* actuator) {
  memset(actuator, 0, sizeof(sim_spec_actuator_t));

  // gain, bias
  actuator->gaintype = SIM_GAIN_FIXED;
  actuator->gainprm[0] = 1;
  actuator->biastype = SIM_BIAS_NONE;

  // activation state
  actuator->dyntype = SIM_DYN_NONE;
  actuator->dynprm[0] = 1;
  actuator->actdim = -1;

  // transmission
  actuator->trntype = SIM_TRN_UNDEFINED;
  actuator->gear[0] = 1;

  // input/output clamping
  actuator->ctrllimited = SIM_LIMITED_AUTO;
  actuator->forcelimited = SIM_LIMITED_AUTO;
  actuator->actlimited = SIM_LIMITED_AUTO;
}


// default sensor attributes
void sim_spec_defaultSensor(SIM_sSensor* sensor) {
  memset(sensor, 0, sizeof(SIM_sSensor));

  sensor->type = SIM_SENS_TOUCH;
  sensor->datatype = SIM_DATATYPE_REAL;
  sensor->needstage = SIM_STAGE_ACC;
}


// Default numeric attributes.
void sim_spec_defaultNumeric(SIM_sNumeric* numeric) {
  memset(numeric, 0, sizeof(SIM_sNumeric));
}


// Default text attributes.
void sim_spec_defaultText(SIM_sText* text) {
  memset(text, 0, sizeof(SIM_sText));
}


// Default tuple attributes.
void sim_spec_defaultTuple(SIM_sTuple* tuple) {
  memset(tuple, 0, sizeof(SIM_sTuple));
}


// Default keyframe attributes.
void sim_spec_defaultKey(SIM_sKey* key) {
  memset(key, 0, sizeof(SIM_sKey));
}


// default plugin attributes
void sim_spec_defaultPlugin(SIM_sPlugin* plugin) {
  memset(plugin, 0, sizeof(SIM_sPlugin));
}
