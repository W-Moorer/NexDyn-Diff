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

#include "xml/xml_native_reader.h"

#include <array>
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <functional>
#include <limits>
#include <map>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <simcore/core_api.h>
#include <simcore/SIM_model.h>
#include <simcore/SIM_plugin.h>
#include <simcore/SIM_tnum.h>
#include "engine/engine_plugin.h"
#include "engine/engine_support.h"
#include "engine/engine_util_errmem.h"
#include "engine/engine_util_misc.h"
#include <simcore/SIM_spec.h>
#include "user/user_api.h"
#include "user/user_composite.h"
#include "user/user_flexcomp.h"
#include "user/user_util.h"
#include "xml/xml_base.h"
#include "xml/xml_util.h"
#include "tinyxml2.h"

namespace {
using std::string;
using std::string_view;
using std::vector;
using simcore::user::FilePath;
using tinyxml2::XMLElement;

void ReadPluginConfigs(tinyxml2::XMLElement* elem, SIM_sPlugin* p) {
  std::map<string, string, std::less<> > config_attribs;
  XMLElement* child = FirstChildElement(elem);
  while (child) {
    string_view name = child->Value();
    if (name == "config") {
      string key, value;
      sim_xml_util_t::ReadAttrTxt(child, "key", key, /* required = */ true);
      if (config_attribs.find(key) != config_attribs.end()) {
        string err = "duplicate config key: " + key;
        throw sim_xml_error_t(child, "%s", err.c_str());
      }
      sim_xml_util_t::ReadAttrTxt(child, "value", value, /* required = */ true);
      config_attribs[key] = value;
    }
    child = NextSiblingElement(child);
  }

  if (!p && !config_attribs.empty()) {
    throw sim_xml_error_t(elem,
                   "plugin configuration attributes cannot be used in an "
                   "element that references a predefined plugin instance");
  } else if (p) {
    sim_spec_setPluginAttributes(p, &config_attribs);
  }
}

static void UpdateString(string& psuffix, int count, int i) {
  int ndigits = std::to_string(count).length();
  string i_string = std::to_string(i);
  string prefix = "";
  while (ndigits-- > i_string.length()) {
    prefix += '0';
  }
  psuffix += prefix + i_string;
}
}  // namespace


//---------------------------------- SIMCF schema ---------------------------------------------------

std::vector<const char*> SIMCF[nSIMCF] = {
{"simcore", "!", "model"},
{"<"},
    {"compiler", "*", "autolimits", "boundmass", "boundinertia", "settotalmass",
        "balanceinertia", "strippath", "coordinate", "angle", "fitaabb", "eulerseq",
        "meshdir", "texturedir", "discardvisual", "usethread", "fusestatic", "inertiafromgeom",
        "inertiagrouprange", "saveinertial", "assetdir", "alignfree"},
    {"<"},
        {"lengthrange", "?", "mode", "useexisting", "uselimit",
            "accel", "maxforce", "timeconst", "timestep",
            "inttotal", "interval", "tolrange"},
    {">"},

    {"option", "*",
        "timestep", "impratio", "tolerance", "ls_tolerance", "noslip_tolerance",
        "ccd_tolerance", "sleep_tolerance", "gravity", "wind", "magnetic", "density", "viscosity",
        "o_margin", "o_solref", "o_solimp", "o_friction",
        "integrator", "cone", "jacobian",
        "solver", "iterations", "ls_iterations", "noslip_iterations", "ccd_iterations",
        "sdf_iterations", "sdf_initpoints", "actuatorgroupdisable"},
    {"<"},
        {"flag", "?", "constraint", "equality", "frictionloss", "limit", "contact",
            "spring", "damper", "gravity", "clampctrl", "warmstart", "filterparent", "actuation",
            "refsafe", "sensor", "midphase", "eulerdamp", "autoreset", "nativeccd", "island",
            "override", "energy", "fwdinv", "invdiscrete", "multiccd", "sleep"},
    {">"},

    {"size", "*", "memory", "njmax", "nconmax", "nstack", "nuserdata", "nkey",
        "nuser_body", "nuser_jnt", "nuser_geom", "nuser_site", "nuser_cam",
        "nuser_tendon", "nuser_actuator", "nuser_sensor"},

    {"visual", "*"},
    {"<"},
        {"global", "?", "cameraid", "orthographic", "fovy", "ipd", "azimuth", "elevation",
            "linewidth", "glow", "offwidth", "offheight", "realtime", "ellipsoidinertia",
            "bvactive"},
        {"quality", "?", "shadowsize", "offsamples", "numslices", "numstacks",
            "numquads"},
        {"headlight", "?", "ambient", "diffuse", "specular", "active"},
        {"map", "?", "stiffness", "stiffnessrot", "force", "torque", "alpha",
            "fogstart", "fogend", "znear", "zfar", "haze", "shadowclip", "shadowscale",
            "actuatortendon"},
        {"scale", "?", "forcewidth", "contactwidth", "contactheight", "connect", "com",
            "camera", "light", "selectpoint", "jointlength", "jointwidth", "actuatorlength",
            "actuatorwidth", "framelength", "framewidth", "constraint", "slidercrank", "frustum"},
        {"rgba", "?", "fog", "haze", "force", "inertia", "joint",
            "actuator", "actuatornegative", "actuatorpositive", "com",
            "camera", "light", "selectpoint", "connect", "contactpoint", "contactforce",
            "contactfriction", "contacttorque", "contactgap", "rangefinder",
            "constraint", "slidercrank", "crankbroken", "frustum", "bv", "bvactive"},
    {">"},

    {"statistic", "*", "meaninertia", "meanmass", "meansize", "extent", "center"},

    {"default", "R", "class"},
    {"<"},
        {"mesh", "?", "scale", "maxhullvert", "inertia"},
        {"material", "?", "texture", "emission", "specular", "shininess",
            "reflectance", "metallic", "roughness", "rgba", "texrepeat", "texuniform"},
        {"<"},
            {"layer", "*", "texture", "role"},
        {">"},
        {"joint", "?", "type", "group", "pos", "axis", "springdamper",
            "limited", "actuatorfrclimited", "solreflimit", "solimplimit",
            "solreffriction", "solimpfriction", "stiffness", "range", "actuatorfrcrange",
            "actuatorgravcomp", "margin", "ref", "springref", "armature", "damping",
            "frictionloss", "user"},
        {"geom", "?", "type", "pos", "quat", "contype", "conaffinity", "condim",
            "group", "priority", "size", "material", "friction", "mass", "density",
            "shellinertia", "solmix", "solref", "solimp",
            "margin", "gap", "fromto", "axisangle", "xyaxes", "zaxis", "euler",
            "hfield", "mesh", "fitscale", "rgba", "fluidshape", "fluidcoef", "user"},
        {"site", "?", "type", "group", "pos", "quat", "material",
            "size", "fromto", "axisangle", "xyaxes", "zaxis", "euler", "rgba", "user"},
        {"camera", "?", "projection", "fovy", "ipd", "resolution", "output", "pos", "quat",
            "axisangle", "xyaxes", "zaxis", "euler", "mode", "focal", "focalpixel",
            "principal", "principalpixel", "sensorsize", "user"},
        {"light", "?", "pos", "dir", "bulbradius", "intensity", "range",
            "directional", "type", "castshadow", "active", "attenuation", "cutoff", "exponent",
            "ambient", "diffuse", "specular", "mode"},
        {"pair", "?", "condim", "friction", "solref", "solreffriction", "solimp",
         "gap", "margin"},
        {"equality", "?", "active", "solref", "solimp"},
        {"tendon", "?", "group", "limited", "range",
            "solreflimit", "solimplimit", "solreffriction", "solimpfriction",
            "frictionloss", "springlength", "width", "material",
            "margin", "stiffness", "damping", "rgba", "user"},
        {"general", "?", "ctrllimited", "forcelimited", "actlimited", "ctrlrange",
            "forcerange", "actrange", "gear", "cranklength", "user", "group", "nsample", "interp", "delay", "actdim",
            "dyntype", "gaintype", "biastype", "dynprm", "gainprm", "biasprm", "actearly"},
        {"motor", "?", "ctrllimited", "forcelimited", "ctrlrange", "forcerange",
            "gear", "cranklength", "user", "group", "nsample", "interp", "delay"},
        {"position", "?", "ctrllimited", "forcelimited", "ctrlrange", "inheritrange",
            "forcerange", "gear", "cranklength", "user", "group", "nsample", "interp", "delay", "kp", "kv", "dampratio", "timeconst"},
        {"velocity", "?", "ctrllimited", "forcelimited", "ctrlrange", "forcerange",
            "gear", "cranklength", "user", "group", "nsample", "interp", "delay", "kv"},
        {"intvelocity", "?", "ctrllimited", "forcelimited",
            "ctrlrange", "forcerange", "actrange", "inheritrange",
            "gear", "cranklength", "user", "group", "nsample", "interp", "delay",
            "kp", "kv", "dampratio"},
        {"damper", "?", "forcelimited", "ctrlrange", "forcerange",
            "gear", "cranklength", "user", "group", "nsample", "interp", "delay", "kv"},
        {"cylinder", "?", "ctrllimited", "forcelimited", "ctrlrange", "forcerange",
            "gear", "cranklength", "user", "group", "nsample", "interp", "delay",
            "timeconst", "area", "diameter", "bias"},
        {"muscle", "?", "ctrllimited", "forcelimited", "ctrlrange", "forcerange",
            "gear", "cranklength", "user", "group", "nsample", "interp", "delay",
            "timeconst", "range", "force", "scale",
            "lmin", "lmax", "vmax", "fpmax", "fvmax"},
        {"adhesion", "?", "forcelimited", "ctrlrange", "forcerange",
            "gain", "user", "group", "nsample", "interp", "delay"},
    {">"},

    {"extension", "*"},
    {"<"},
        {"plugin", "*", "plugin"},
        {"<"},
            {"instance", "*", "name"},
            {"<"},
                {"config", "*", "key", "value"},
            {">"},
        {">"},
    {">"},

    {"custom", "*"},
    {"<"},
        {"numeric", "*",  "name", "size", "data"},
        {"text", "*", "name", "data"},
        {"tuple", "*", "name"},
        {"<"},
            {"element", "*",  "objtype", "objname", "prm"},
        {">"},
    {">"},

    {"asset", "*"},
    {"<"},
        {"mesh", "*", "name", "class", "content_type", "file", "vertex", "normal",
            "texcoord", "face", "refpos", "refquat", "scale", "smoothnormal",
            "maxhullvert", "inertia", "builtin", "params", "material"},
        {"<"},
          {"plugin", "*", "plugin", "instance"},
          {"<"},
            {"config", "*", "key", "value"},
          {">"},
        {">"},
        {"hfield", "*", "name", "content_type", "file", "nrow", "ncol", "size", "elevation"},
        {"skin", "*", "name", "file", "material", "rgba", "inflate",
            "vertex", "texcoord", "face", "group"},
        {"<"},
            {"bone", "*", "body", "bindpos", "bindquat", "vertid", "vertweight"},
        {">"},
        {"texture", "*", "name", "type", "colorspace", "content_type", "file", "gridsize",
            "gridlayout", "fileright", "fileleft", "fileup", "filedown", "filefront", "fileback",
            "builtin", "rgb1", "rgb2", "mark", "markrgb", "random", "width", "height",
            "hflip", "vflip", "nchannel"},
        {"material", "*", "name", "class", "texture",  "texrepeat", "texuniform",
            "emission", "specular", "shininess", "reflectance", "metallic", "roughness", "rgba"},
        {"<"},
            {"layer", "*", "texture", "role"},
        {">"},
        {"model", "*", "name", "file", "content_type"},
    {">"},

    {"body", "R", "name", "childclass", "pos", "quat", "mocap",
        "axisangle", "xyaxes", "zaxis", "euler", "gravcomp", "sleep", "user"},
    {"<"},
        {"inertial", "?", "pos", "quat", "mass", "diaginertia",
            "axisangle", "xyaxes", "zaxis", "euler", "fullinertia"},
        {"joint", "*", "name", "class", "type", "group", "pos", "axis",
            "springdamper", "limited", "actuatorfrclimited",
            "solreflimit", "solimplimit", "solreffriction", "solimpfriction",
            "stiffness", "range", "actuatorfrcrange", "actuatorgravcomp", "margin", "ref",
            "springref", "armature", "damping", "frictionloss", "user"},
        {"freejoint", "*",  "name", "group", "align"},
        {"geom", "*", "name", "class", "type", "contype", "conaffinity", "condim",
            "group", "priority", "size", "material", "friction", "mass", "density",
            "shellinertia", "solmix", "solref", "solimp",
            "margin", "gap", "fromto", "pos", "quat", "axisangle", "xyaxes", "zaxis", "euler",
            "hfield", "mesh", "fitscale", "rgba", "fluidshape", "fluidcoef", "user"},
        {"<"},
            {"plugin", "*", "plugin", "instance"},
            {"<"},
              {"config", "*", "key", "value"},
            {">"},
        {">"},
        {"attach", "*", "model", "body", "prefix"},
        {"site", "*",  "name", "class", "type", "group", "pos", "quat",
            "material", "size", "fromto", "axisangle", "xyaxes", "zaxis", "euler", "rgba", "user"},
        {"camera", "*", "name", "class", "projection", "fovy", "ipd", "resolution", "output", "pos",
            "quat", "axisangle", "xyaxes", "zaxis", "euler", "mode", "target",
            "focal", "focalpixel", "principal", "principalpixel", "sensorsize", "user"},
        {"light", "*", "name", "class", "directional", "type", "castshadow", "active",
            "pos", "dir", "bulbradius", "intensity", "range", "attenuation", "cutoff",
            "exponent", "ambient", "diffuse", "specular", "mode", "target", "texture"},
        {"plugin", "*", "plugin", "instance"},
        {"<"},
          {"config", "*", "key", "value"},
        {">"},
        {"composite", "*", "prefix", "type", "count", "offset",
            "vertex", "initial", "curve", "size", "quat"},
        {"<"},
            {"joint", "*", "kind", "group", "stiffness", "damping", "armature",
                "solreffix", "solimpfix", "type", "axis",
                "limited", "range", "margin", "solreflimit", "solimplimit",
                "frictionloss", "solreffriction", "solimpfriction"},
            {"skin", "?", "texcoord", "material", "group", "rgba", "inflate", "subgrid"},
            {"geom", "?", "type", "contype", "conaffinity", "condim",
                "group", "priority", "size", "material", "rgba", "friction", "mass",
                "density", "solmix", "solref", "solimp", "margin", "gap"},
            {"site", "?", "group", "size", "material", "rgba"},
            {"plugin", "*", "plugin", "instance"},
            {"<"},
              {"config", "*", "key", "value"},
            {">"},
        {">"},
        {"flexcomp", "*", "name", "type", "group", "dim", "dof",
            "count", "spacing", "radius", "rigid", "mass", "inertiabox",
            "scale", "file", "point", "element", "texcoord", "material", "rgba",
            "flatskin", "pos", "quat", "axisangle", "xyaxes", "zaxis", "euler", "origin"},
        {"<"},
            {"edge", "?", "equality", "solref", "solimp", "stiffness", "damping"},
            {"elasticity", "?", "young", "poisson", "damping", "thickness", "elastic2d"},
            {"contact", "?",  "contype", "conaffinity", "condim", "priority",
                "friction", "solmix", "solref", "solimp", "margin", "gap",
                "internal", "selfcollide", "activelayers", "vertcollide", "passive"},
            {"pin", "*", "id", "range", "grid", "gridrange"},
            {"plugin", "*", "plugin", "instance"},
            {"<"},
              {"config", "*", "key", "value"},
            {">"},
        {">"},
    {">"},

    {"deformable", "*"},
    {"<"},
        {"flex", "*", "name", "group", "dim", "radius", "material",
            "rgba", "flatskin", "body", "vertex", "element", "texcoord", "elemtexcoord", "node"},
        {"<"},
            {"contact", "?",  "contype", "conaffinity", "condim", "priority",
                "friction", "solmix", "solref", "solimp", "margin", "gap",
                "internal", "selfcollide", "activelayers", "vertcollide", "passive"},
            {"edge", "?", "stiffness", "damping"},
            {"elasticity", "?", "young", "poisson", "damping", "thickness", "elastic2d"},
        {">"},
        {"skin", "*", "name", "file", "material", "rgba", "inflate",
            "vertex", "texcoord", "face", "group"},
        {"<"},
            {"bone", "*", "body", "bindpos", "bindquat", "vertid", "vertweight"},
        {">"},
    {">"},

    {"contact", "*"},
    {"<"},
        {"pair", "*", "name", "class", "geom1", "geom2", "condim", "friction",
            "solref", "solreffriction", "solimp", "gap", "margin"},
        {"exclude", "*", "name", "body1", "body2"},
    {">"},

    {"equality", "*"},
    {"<"},
        {"connect", "*",  "name", "class", "body1", "body2", "anchor",
            "site1", "site2", "active", "solref", "solimp"},
        {"weld", "*", "name", "class", "body1", "body2", "relpose", "anchor",
            "site1", "site2", "active", "solref", "solimp", "torquescale"},
        {"joint", "*", "name", "class", "joint1", "joint2", "polycoef",
            "active", "solref", "solimp"},
        {"tendon", "*", "name", "class", "tendon1", "tendon2", "polycoef",
            "active", "solref", "solimp"},
        {"flex", "*", "name", "class", "flex",
            "active", "solref", "solimp"},
        {"flexvert", "*", "name", "class", "flex",
            "active", "solref", "solimp"},
    {">"},

    {"tendon", "*"},
    {"<"},
        {"spatial", "*", "name", "class", "group", "limited", "actuatorfrclimited", "range",
            "actuatorfrcrange", "solreflimit", "solimplimit", "solreffriction", "solimpfriction",
            "frictionloss", "springlength", "width", "material",
            "margin", "stiffness", "damping", "armature", "rgba", "user"},
        {"<"},
            {"site", "*", "site"},
            {"geom", "*", "geom", "sidesite"},
            {"pulley", "*", "divisor"},
        {">"},
        {"fixed", "*", "name", "class", "group", "limited", "actuatorfrclimited", "range",
            "actuatorfrcrange", "solreflimit", "solimplimit", "solreffriction", "solimpfriction",
            "frictionloss", "springlength", "margin", "stiffness", "damping", "armature", "user"},
        {"<"},
            {"joint", "*", "joint", "coef"},
        {">"},
    {">"},

    {"actuator", "*"},
    {"<"},
        {"general", "*", "name", "class", "group", "nsample", "interp", "delay",
            "ctrllimited", "forcelimited", "actlimited", "ctrlrange", "forcerange", "actrange",
            "lengthrange", "gear", "cranklength", "user",
            "joint", "jointinparent", "tendon", "slidersite", "cranksite", "site", "refsite",
            "body", "actdim", "dyntype", "gaintype", "biastype", "dynprm", "gainprm", "biasprm",
            "actearly"},
        {"motor", "*", "name", "class", "group", "nsample", "interp", "delay",
            "ctrllimited", "forcelimited", "ctrlrange", "forcerange",
            "lengthrange", "gear", "cranklength", "user",
            "joint", "jointinparent", "tendon", "slidersite", "cranksite", "site", "refsite"},
        {"position", "*", "name", "class", "group", "nsample", "interp", "delay",
            "ctrllimited", "forcelimited", "ctrlrange", "inheritrange", "forcerange",
            "lengthrange", "gear", "cranklength", "user",
            "joint", "jointinparent", "tendon", "slidersite", "cranksite", "site", "refsite",
            "kp", "kv", "dampratio", "timeconst"},
        {"velocity", "*", "name", "class", "group", "nsample", "interp", "delay",
            "ctrllimited", "forcelimited", "ctrlrange", "forcerange",
            "lengthrange", "gear", "cranklength", "user",
            "joint", "jointinparent", "tendon", "slidersite", "cranksite", "site", "refsite",
            "kv"},
        {"intvelocity", "*", "name", "class", "group", "nsample", "interp", "delay",
            "ctrllimited", "forcelimited",
            "ctrlrange", "forcerange", "actrange", "inheritrange", "lengthrange",
            "gear", "cranklength", "user",
            "joint", "jointinparent", "tendon", "slidersite", "cranksite", "site", "refsite",
            "kp", "kv", "dampratio"},
        {"damper", "*", "name", "class", "group", "nsample", "interp", "delay",
            "forcelimited", "ctrlrange", "forcerange",
            "lengthrange", "gear", "cranklength", "user",
            "joint", "jointinparent", "tendon", "slidersite", "cranksite", "site", "refsite",
            "kv"},
        {"cylinder", "*", "name", "class", "group", "nsample", "interp", "delay",
            "ctrllimited", "forcelimited", "ctrlrange", "forcerange",
            "lengthrange", "gear", "cranklength", "user",
            "joint", "jointinparent", "tendon", "slidersite", "cranksite", "site", "refsite",
            "timeconst", "area", "diameter", "bias"},
        {"muscle", "*",  "name", "class", "group", "nsample", "interp", "delay",
            "ctrllimited", "forcelimited", "ctrlrange", "forcerange",
            "lengthrange", "gear", "cranklength", "user",
            "joint", "jointinparent", "tendon", "slidersite", "cranksite",
            "timeconst", "tausmooth", "range", "force", "scale",
            "lmin", "lmax", "vmax", "fpmax", "fvmax"},
        {"adhesion", "*", "name", "class", "group", "nsample", "interp", "delay",
            "forcelimited", "ctrlrange", "forcerange", "user", "body", "gain"},
        {"plugin", "*", "name", "class",  "plugin", "instance", "group", "nsample", "interp", "delay",
            "ctrllimited", "forcelimited", "actlimited", "ctrlrange", "forcerange", "actrange",
            "lengthrange", "gear", "cranklength", "joint", "jointinparent",
            "site", "actdim", "dyntype", "dynprm", "tendon", "cranksite", "slidersite", "user",
            "actearly"},
        {"<"},
          {"config", "*", "key", "value"},
        {">"},
    {">"},

    {"sensor", "*"},
    {"<"},
        {"touch", "*", "name", "site", "nsample", "interp", "delay", "interval", "cutoff", "noise", "user"},
        {"accelerometer", "*", "name", "site", "nsample", "interp", "delay", "interval", "cutoff", "noise", "user"},
        {"velocimeter", "*", "name", "site", "nsample", "interp", "delay", "interval", "cutoff", "noise", "user"},
        {"gyro", "*", "name", "site", "nsample", "interp", "delay", "interval", "cutoff", "noise", "user"},
        {"force", "*", "name", "site", "nsample", "interp", "delay", "interval", "cutoff", "noise", "user"},
        {"torque", "*", "name", "site", "nsample", "interp", "delay", "interval", "cutoff", "noise", "user"},
        {"magnetometer", "*", "name", "site", "nsample", "interp", "delay", "interval", "cutoff", "noise", "user"},
        {"camprojection", "*", "name", "site", "camera", "nsample", "interp", "delay", "interval", "cutoff", "noise", "user"},
        {"rangefinder", "*", "name", "site", "camera", "data", "nsample", "interp", "delay", "interval", "cutoff", "noise", "user"},
        {"jointpos", "*", "name", "joint", "nsample", "interp", "delay", "interval", "cutoff", "noise", "user"},
        {"jointvel", "*", "name", "joint", "nsample", "interp", "delay", "interval", "cutoff", "noise", "user"},
        {"tendonpos", "*", "name", "tendon", "nsample", "interp", "delay", "interval", "cutoff", "noise", "user"},
        {"tendonvel", "*", "name", "tendon", "nsample", "interp", "delay", "interval", "cutoff", "noise", "user"},
        {"actuatorpos", "*", "name", "actuator", "nsample", "interp", "delay", "interval", "cutoff", "noise", "user"},
        {"actuatorvel", "*", "name", "actuator", "nsample", "interp", "delay", "interval", "cutoff", "noise", "user"},
        {"actuatorfrc", "*", "name", "actuator", "nsample", "interp", "delay", "interval", "cutoff", "noise", "user"},
        {"jointactuatorfrc", "*", "name", "joint", "nsample", "interp", "delay", "interval", "cutoff", "noise", "user"},
        {"tendonactuatorfrc", "*", "name", "tendon", "nsample", "interp", "delay", "interval", "cutoff", "noise", "user"},
        {"ballquat", "*", "name", "joint", "nsample", "interp", "delay", "interval", "cutoff", "noise", "user"},
        {"ballangvel", "*", "name", "joint", "nsample", "interp", "delay", "interval", "cutoff", "noise", "user"},
        {"jointlimitpos", "*", "name", "joint", "nsample", "interp", "delay", "interval", "cutoff", "noise", "user"},
        {"jointlimitvel", "*", "name", "joint", "nsample", "interp", "delay", "interval", "cutoff", "noise", "user"},
        {"jointlimitfrc", "*", "name", "joint", "nsample", "interp", "delay", "interval", "cutoff", "noise", "user"},
        {"tendonlimitpos", "*", "name", "tendon", "nsample", "interp", "delay", "interval", "cutoff", "noise", "user"},
        {"tendonlimitvel", "*", "name", "tendon", "nsample", "interp", "delay", "interval", "cutoff", "noise", "user"},
        {"tendonlimitfrc", "*", "name", "tendon", "nsample", "interp", "delay", "interval", "cutoff", "noise", "user"},
        {"framepos", "*", "name", "objtype", "objname", "reftype", "refname", "nsample", "interp", "delay", "interval", "cutoff", "noise", "user"},
        {"framequat", "*", "name", "objtype", "objname", "reftype", "refname", "nsample", "interp", "delay", "interval", "cutoff", "noise", "user"},
        {"framexaxis", "*", "name", "objtype", "objname", "reftype", "refname", "nsample", "interp", "delay", "interval", "cutoff", "noise", "user"},
        {"frameyaxis", "*", "name", "objtype", "objname", "reftype", "refname", "nsample", "interp", "delay", "interval", "cutoff", "noise", "user"},
        {"framezaxis", "*", "name", "objtype", "objname", "reftype", "refname", "nsample", "interp", "delay", "interval", "cutoff", "noise", "user"},
        {"framelinvel", "*", "name", "objtype", "objname", "reftype", "refname", "nsample", "interp", "delay", "interval", "cutoff", "noise", "user"},
        {"frameangvel", "*", "name", "objtype", "objname", "reftype", "refname", "nsample", "interp", "delay", "interval", "cutoff", "noise", "user"},
        {"framelinacc", "*", "name", "objtype", "objname", "nsample", "interp", "delay", "interval", "cutoff", "noise", "user"},
        {"frameangacc", "*", "name", "objtype", "objname", "nsample", "interp", "delay", "interval", "cutoff", "noise", "user"},
        {"subtreecom", "*", "name", "body", "nsample", "interp", "delay", "interval", "cutoff", "noise", "user"},
        {"subtreelinvel", "*", "name", "body", "nsample", "interp", "delay", "interval", "cutoff", "noise", "user"},
        {"subtreeangmom", "*", "name", "body", "nsample", "interp", "delay", "interval", "cutoff", "noise", "user"},
        {"insidesite", "*", "name", "site", "objtype", "objname", "nsample", "interp", "delay", "interval", "cutoff", "noise", "user"},
        {"distance", "*", "name", "geom1", "geom2", "body1", "body2", "nsample", "interp", "delay", "interval", "cutoff", "noise", "user"},
        {"normal", "*", "name", "geom1", "geom2", "body1", "body2", "nsample", "interp", "delay", "interval", "cutoff", "noise", "user"},
        {"fromto", "*", "name", "geom1", "geom2", "body1", "body2", "nsample", "interp", "delay", "interval", "cutoff", "noise", "user"},
        {"contact", "*", "name", "geom1", "geom2", "body1", "body2", "subtree1", "subtree2", "site",
            "num", "data", "reduce", "nsample", "interp", "delay", "interval", "cutoff", "noise", "user"},
        {"e_potential", "*", "name", "nsample", "interp", "delay", "interval", "cutoff", "noise", "user"},
        {"e_kinetic", "*", "name", "nsample", "interp", "delay", "interval", "cutoff", "noise", "user"},
        {"clock", "*", "name", "nsample", "interp", "delay", "interval", "cutoff", "noise", "user"},
        {"tactile", "*", "name", "geom", "mesh", "nsample", "interp", "delay", "interval", "user"},
        {"user", "*", "name", "objtype", "objname", "datatype", "needstage",
            "dim", "cutoff", "noise", "user"},
        {"plugin", "*", "name", "plugin", "instance", "cutoff", "objtype", "objname", "reftype", "refname",
            "user"},
        {"<"},
          {"config", "*", "key", "value"},
        {">"},
    {">"},

    {"keyframe", "*"},
    {"<"},
        {"key", "*", "name", "time", "qpos", "qvel", "act", "mpos", "mquat", "ctrl"},
    {">"},
{">"}
};



//---------------------------------- SIMCF keywords used in attributes ------------------------------

// coordinate type
const sim_map_t coordinate_map[2] = {
  {"local",   0},
  {"global",  1}
};


// angle type
const sim_map_t angle_map[2] = {
  {"radian",  0},
  {"degree",  1}
};


// bool type
const sim_map_t bool_map[2] = {
  {"false",   0},
  {"true",    1}
};


// fluidshape type
const sim_map_t fluid_map[2] = {
  {"none",      0},
  {"ellipsoid", 1}
};


// enable type
const sim_map_t enable_map[2] = {
  {"disable", 0},
  {"enable",  1}
};


// TFAuto type
const sim_map_t TFAuto_map[3] = {
  {"false",   0},
  {"true",    1},
  {"auto",    2}
};


// body sleep type
const int bodysleep_sz = 4;
const sim_map_t bodysleep_map[bodysleep_sz] = {
  {"auto",          SIM_SLEEP_AUTO},
  {"never",         SIM_SLEEP_NEVER},
  {"allowed",       SIM_SLEEP_ALLOWED},
  {"init",          SIM_SLEEP_INIT}
};

// joint type
const int joint_sz = 4;
const sim_map_t joint_map[joint_sz] = {
  {"free",          SIM_JNT_FREE},
  {"ball",          SIM_JNT_BALL},
  {"slide",         SIM_JNT_SLIDE},
  {"hinge",         SIM_JNT_HINGE}
};


// geom type
const sim_map_t geom_map[SIM_NGEOMTYPES] = {
  {"plane",         SIM_GEOM_PLANE},
  {"hfield",        SIM_GEOM_HFIELD},
  {"sphere",        SIM_GEOM_SPHERE},
  {"capsule",       SIM_GEOM_CAPSULE},
  {"ellipsoid",     SIM_GEOM_ELLIPSOID},
  {"cylinder",      SIM_GEOM_CYLINDER},
  {"box",           SIM_GEOM_BOX},
  {"mesh",          SIM_GEOM_MESH},
  {"sdf",           SIM_GEOM_SDF}
};


// projection type
const int projection_sz = 2;
const sim_map_t projection_map[projection_sz] = {
  {"perspective",   SIM_PROJ_PERSPECTIVE},
  {"orthographic",  SIM_PROJ_ORTHOGRAPHIC}
};

// camlight type
const int camlight_sz = 5;
const sim_map_t camlight_map[camlight_sz] = {
  {"fixed",         SIM_CAMLIGHT_FIXED},
  {"track",         SIM_CAMLIGHT_TRACK},
  {"trackcom",      SIM_CAMLIGHT_TRACKCOM},
  {"targetbody",    SIM_CAMLIGHT_TARGETBODY},
  {"targetbodycom", SIM_CAMLIGHT_TARGETBODYCOM}
};


// light type
const int lighttype_sz = 4;
const sim_map_t lighttype_map[lighttype_sz] = {
  {"spot",          SIM_LIGHT_SPOT},
  {"directional",   SIM_LIGHT_DIRECTIONAL},
  {"point",         SIM_LIGHT_POINT},
  {"image",         SIM_LIGHT_IMAGE}
};


// texmat role type
const int texrole_sz = SIM_NTEXROLE - 1;
const sim_map_t texrole_map[texrole_sz] = {
  {"rgb",           SIM_TEXROLE_RGB},
  {"occlusion",     SIM_TEXROLE_OCCLUSION},
  {"roughness",     SIM_TEXROLE_ROUGHNESS},
  {"metallic",      SIM_TEXROLE_METALLIC},
  {"normal",        SIM_TEXROLE_NORMAL},
  {"opacity",       SIM_TEXROLE_OPACITY},
  {"emissive",      SIM_TEXROLE_EMISSIVE},
  {"rgba",          SIM_TEXROLE_RGBA},
  {"orm",           SIM_TEXROLE_ORM},
};


// integrator type
const int integrator_sz = 4;
const sim_map_t integrator_map[integrator_sz] = {
  {"Euler",         SIM_INT_EULER},
  {"RK4",           SIM_INT_RK4},
  {"implicit",      SIM_INT_IMPLICIT},
  {"implicitfast",  SIM_INT_IMPLICITFAST}
};


// cone type
const int cone_sz = 2;
const sim_map_t cone_map[cone_sz] = {
  {"pyramidal",     SIM_CONE_PYRAMIDAL},
  {"elliptic",      SIM_CONE_ELLIPTIC}
};


// Jacobian type
const int jac_sz = 3;
const sim_map_t jac_map[jac_sz] = {
  {"dense",         SIM_JAC_DENSE},
  {"sparse",        SIM_JAC_SPARSE},
  {"auto",          SIM_JAC_AUTO}
};


// solver type
const int solver_sz = 3;
const sim_map_t solver_map[solver_sz] = {
  {"PGS",           SIM_SOL_PGS},
  {"CG",            SIM_SOL_CG},
  {"Newton",        SIM_SOL_NEWTON}
};


// constraint type
const int equality_sz = 7;
const sim_map_t equality_map[equality_sz] = {
  {"connect",       SIM_EQ_CONNECT},
  {"weld",          SIM_EQ_WELD},
  {"joint",         SIM_EQ_JOINT},
  {"tendon",        SIM_EQ_TENDON},
  {"flex",          SIM_EQ_FLEX},
  {"flexvert",      SIM_EQ_FLEXVERT},
  {"distance",      SIM_EQ_DISTANCE}
};


// type for texture
const int texture_sz = 3;
const sim_map_t texture_map[texture_sz] = {
  {"2d",            SIM_TEXTURE_2D},
  {"cube",          SIM_TEXTURE_CUBE},
  {"skybox",        SIM_TEXTURE_SKYBOX}
};


// colorspace for texture
const int colorspace_sz = 3;
const sim_map_t colorspace_map[colorspace_sz] = {
  {"auto",          SIM_COLORSPACE_AUTO},
  {"linear",        SIM_COLORSPACE_LINEAR},
  {"sRGB",          SIM_COLORSPACE_SRGB}
};


// builtin type for texture
const int builtin_sz = 4;
const sim_map_t builtin_map[builtin_sz] = {
  {"none",          SIM_BUILTIN_NONE},
  {"gradient",      SIM_BUILTIN_GRADIENT},
  {"checker",       SIM_BUILTIN_CHECKER},
  {"flat",          SIM_BUILTIN_FLAT}
};


// mark type for texture
const int mark_sz = 4;
const sim_map_t mark_map[mark_sz] = {
  {"none",          SIM_MARK_NONE},
  {"edge",          SIM_MARK_EDGE},
  {"cross",         SIM_MARK_CROSS},
  {"random",        SIM_MARK_RANDOM}
};


// dyn type
const int dyn_sz = 6;
const sim_map_t dyn_map[dyn_sz] = {
  {"none",          SIM_DYN_NONE},
  {"integrator",    SIM_DYN_INTEGRATOR},
  {"filter",        SIM_DYN_FILTER},
  {"filterexact",   SIM_DYN_FILTEREXACT},
  {"muscle",        SIM_DYN_MUSCLE},
  {"user",          SIM_DYN_USER}
};


// gain type
const int gain_sz = 4;
const sim_map_t gain_map[gain_sz] = {
  {"fixed",         SIM_GAIN_FIXED},
  {"affine",        SIM_GAIN_AFFINE},
  {"muscle",        SIM_GAIN_MUSCLE},
  {"user",          SIM_GAIN_USER}
};


// bias type
const int bias_sz = 4;
const sim_map_t bias_map[bias_sz] = {
  {"none",          SIM_BIAS_NONE},
  {"affine",        SIM_BIAS_AFFINE},
  {"muscle",        SIM_BIAS_MUSCLE},
  {"user",          SIM_BIAS_USER}
};


// interpolation type
const int interp_sz = 3;
const sim_map_t interp_map[interp_sz] = {
  {"zoh",           0},
  {"linear",        1},
  {"cubic",         2}
};


// stage type
const int stage_sz = 4;
const sim_map_t stage_map[stage_sz] = {
  {"none",          SIM_STAGE_NONE},
  {"pos",           SIM_STAGE_POS},
  {"vel",           SIM_STAGE_VEL},
  {"acc",           SIM_STAGE_ACC}
};


// data type
const int datatype_sz = 4;
const sim_map_t datatype_map[datatype_sz] = {
  {"real",          SIM_DATATYPE_REAL},
  {"positive",      SIM_DATATYPE_POSITIVE},
  {"axis",          SIM_DATATYPE_AXIS},
  {"quaternion",    SIM_DATATYPE_QUATERNION}
};


// contact data type
const sim_map_t condata_map[SIM_NCONDATA] = {
  {"found",         SIM_CONDATA_FOUND},
  {"force",         SIM_CONDATA_FORCE},
  {"torque",        SIM_CONDATA_TORQUE},
  {"dist",          SIM_CONDATA_DIST},
  {"pos",           SIM_CONDATA_POS},
  {"normal",        SIM_CONDATA_NORMAL},
  {"tangent",       SIM_CONDATA_TANGENT}
};


// rangefinder data type
const sim_map_t raydata_map[SIM_NRAYDATA] = {
  {"dist",          SIM_RAYDATA_DIST},
  {"dir",           SIM_RAYDATA_DIR},
  {"origin",        SIM_RAYDATA_ORIGIN},
  {"point",         SIM_RAYDATA_POINT},
  {"normal",        SIM_RAYDATA_NORMAL},
  {"depth",         SIM_RAYDATA_DEPTH}
};

// camera output type
const int camout_sz = SIM_NCAMOUT;
const sim_map_t camout_map[SIM_NCAMOUT] = {{"rgb", SIM_CAMOUT_RGB},
                                     {"depth", SIM_CAMOUT_DEPTH},
                                     {"distance", SIM_CAMOUT_DIST},
                                     {"normal", SIM_CAMOUT_NORMAL},
                                     {"segmentation", SIM_CAMOUT_SEG}};

// contact reduction type
const int reduce_sz = 4;
const sim_map_t reduce_map[reduce_sz] = {
  {"none",          0},
  {"mindist",       1},
  {"maxforce",      2},
  {"netforce",      3}
};


// LR mode
const int lrmode_sz = 4;
const sim_map_t lrmode_map[lrmode_sz] = {
  {"none",          SIM_LRMODE_NONE},
  {"muscle",        SIM_LRMODE_MUSCLE},
  {"muscleuser",    SIM_LRMODE_MUSCLEUSER},
  {"all",           SIM_LRMODE_ALL}
};


// composite type
const sim_map_t comp_map[SIM_NCOMPTYPES] = {
  {"particle",      SIM_COMPTYPE_PARTICLE},
  {"grid",          SIM_COMPTYPE_GRID},
  {"rope",          SIM_COMPTYPE_ROPE},
  {"loop",          SIM_COMPTYPE_LOOP},
  {"cable",         SIM_COMPTYPE_CABLE},
  {"cloth",         SIM_COMPTYPE_CLOTH}
};


// composite joint kind
const sim_map_t jkind_map[1] = {
  {"main",          SIM_COMPKIND_JOINT}
};


// composite rope shape
const sim_map_t shape_map[SIM_NCOMPSHAPES] = {
  {"s",             SIM_COMPSHAPE_LINE},
  {"cos(s)",        SIM_COMPSHAPE_COS},
  {"sin(s)",        SIM_COMPSHAPE_SIN},
  {"0",             SIM_COMPSHAPE_ZERO}
};


// mesh type
const sim_map_t meshtype_map[2] = {
  {"false",         SIM_INERTIA_VOLUME},
  {"true",          SIM_INERTIA_SHELL},
};


// mesh inertia type
const sim_map_t meshinertia_map[4] = {
  {"convex",        SIM_MESH_INERTIA_CONVEX},
  {"legacy",        SIM_MESH_INERTIA_LEGACY},
  {"exact",         SIM_MESH_INERTIA_EXACT},
  {"shell",         SIM_MESH_INERTIA_SHELL}
};


// mesh builtin type
const int meshbuiltin_sz = 8;
const sim_map_t meshbuiltin_map[meshbuiltin_sz] = {
  {"none",          SIM_MESH_BUILTIN_NONE},
  {"sphere",        SIM_MESH_BUILTIN_SPHERE},
  {"hemisphere",    SIM_MESH_BUILTIN_HEMISPHERE},
  {"cone",          SIM_MESH_BUILTIN_CONE},
  {"supertorus",    SIM_MESH_BUILTIN_SUPERTORUS},
  {"supersphere",   SIM_MESH_BUILTIN_SUPERSPHERE},
  {"wedge",         SIM_MESH_BUILTIN_WEDGE},
  {"plate",         SIM_MESH_BUILTIN_PLATE}
};


// flexcomp type
const sim_map_t fcomp_map[SIM_NFCOMPTYPES] = {
  {"grid",          SIM_FCOMPTYPE_GRID},
  {"box",           SIM_FCOMPTYPE_BOX},
  {"cylinder",      SIM_FCOMPTYPE_CYLINDER},
  {"ellipsoid",     SIM_FCOMPTYPE_ELLIPSOID},
  {"square",        SIM_FCOMPTYPE_SQUARE},
  {"disc",          SIM_FCOMPTYPE_DISC},
  {"circle",        SIM_FCOMPTYPE_CIRCLE},
  {"mesh",          SIM_FCOMPTYPE_MESH},
  {"gmsh",          SIM_FCOMPTYPE_GMSH},
  {"direct",        SIM_FCOMPTYPE_DIRECT}
};


// flexcomp dof type
const sim_map_t fdof_map[SIM_NFCOMPDOFS] = {
  {"full",          SIM_FCOMPDOF_FULL},
  {"radial",        SIM_FCOMPDOF_RADIAL},
  {"trilinear",     SIM_FCOMPDOF_TRILINEAR},
  {"quadratic",     SIM_FCOMPDOF_QUADRATIC}
};


// flex selfcollide type
const sim_map_t flexself_map[5] = {
  {"none",          SIM_FLEXSELF_NONE},
  {"narrow",        SIM_FLEXSELF_NARROW},
  {"bvh",           SIM_FLEXSELF_BVH},
  {"sap",           SIM_FLEXSELF_SAP},
  {"auto",          SIM_FLEXSELF_AUTO},
};


// flex elastic 2d type
const sim_map_t elastic2d_map[5] = {
  {"none",          0},
  {"bend",          1},
  {"stretch",       2},
  {"both",          3},
};


// flex equality type
const sim_map_t flexeq_map[3] = {
  {"false",         0},
  {"true",          1},
  {"vert",          2},
};



//---------------------------------- class sim_xml_reader_t implementation --------------------------------

// constructor
sim_xml_reader_t::sim_xml_reader_t() : schema(SIMCF, nSIMCF) {
  readingdefaults = false;
}



// print schema
void sim_xml_reader_t::PrintSchema(std::stringstream& str, bool html, bool pad) {
  if (html) {
    schema.PrintHTML(str, 0, pad);
  } else {
    schema.Print(str, 0);
  }
}



// main entry point for XML parser
//  sim_builder_model_t is allocated here; caller is responsible for deallocation
void sim_xml_reader_t::Parse(XMLElement* root, const SIM_VFS* vfs) {
  // check schema
  if (!schema.GetError().empty()) {
    throw sim_xml_error_t(0, "XML Schema Construction Error: %s", schema.GetError().c_str());
  }

  // validate
  XMLElement* bad = 0;
  if ((bad = schema.Check(root, 0))) {
    throw sim_xml_error_t(bad, "Schema violation: %s", schema.GetError().c_str());
  }

  // get model name
  string modelname;
  if (ReadAttrTxt(root, "model", modelname)) {
    sim_spec_set_string(spec->modelname, modelname.c_str());
  }

  // get comment
  if (root->FirstChild() && root->FirstChild()->ToComment()) {
    sim_spec_set_string(spec->comment, root->FirstChild()->Value());
  } else {
    sim_spec_set_string(spec->comment, "");
  }

  //------------------- parse SimCore sections embedded in all XML formats

  for (XMLElement* section = FirstChildElement(root, "compiler"); section;
       section = NextSiblingElement(section, "compiler")) {
    Compiler(section, spec);
  }

  for (XMLElement* section = FirstChildElement(root, "option"); section;
       section = NextSiblingElement(section, "option")) {
    Option(section, &spec->option);
  }

  for (XMLElement* section = FirstChildElement(root, "size"); section;
       section = NextSiblingElement(section, "size")) {
    Size(section, spec);
  }

  //------------------ parse SIMCF-specific sections

  for (XMLElement* section = FirstChildElement(root, "visual"); section;
       section = NextSiblingElement(section, "visual")) {
    Visual(section);
  }

  for (XMLElement* section = FirstChildElement(root, "statistic"); section;
       section = NextSiblingElement(section, "statistic")) {
    Statistic(section);
  }

  readingdefaults = true;
  for (XMLElement* section = FirstChildElement(root, "default"); section;
       section = NextSiblingElement(section, "default")) {
    Default(section, nullptr, vfs);
  }
  readingdefaults = false;

  for (XMLElement* section = FirstChildElement(root, "extension"); section;
       section = NextSiblingElement(section, "extension")) {
    Extension(section);
  }

  for (XMLElement* section = FirstChildElement(root, "custom"); section;
       section = NextSiblingElement(section, "custom")) {
    Custom(section);
  }

  for (XMLElement* section = FirstChildElement(root, "asset"); section;
       section = NextSiblingElement(section, "asset")) {
    Asset(section, vfs);
  }

  for (XMLElement* section = FirstChildElement(root, "contact"); section;
       section = NextSiblingElement(section, "contact")) {
    Contact(section);
  }

  for (XMLElement* section = FirstChildElement(root, "deformable"); section;
       section = NextSiblingElement(section, "deformable")) {
    Deformable(section, vfs);
  }

  for (XMLElement* section = FirstChildElement(root, "equality"); section;
       section = NextSiblingElement(section, "equality")) {
    Equality(section);
  }

  for (XMLElement* section = FirstChildElement(root, "tendon"); section;
       section = NextSiblingElement(section, "tendon")) {
    Tendon(section);
  }

  for (XMLElement* section = FirstChildElement(root, "actuator"); section;
       section = NextSiblingElement(section, "actuator")) {
    Actuator(section);
  }

  for (XMLElement* section = FirstChildElement(root, "sensor"); section;
       section = NextSiblingElement(section, "sensor")) {
    Sensor(section);
  }

  for (XMLElement* section = FirstChildElement(root, "keyframe"); section;
       section = NextSiblingElement(section, "keyframe")) {
    Keyframe(section);
  }

  // set deepcopy flag to true to copy child specs during attach calls
  sim_spec_setDeepCopy(spec, true);

  for (XMLElement* section = FirstChildElement(root, "worldbody"); section;
       section = NextSiblingElement(section, "worldbody")) {
    Body(section, sim_spec_findBody(spec, "world"), nullptr, vfs);
  }

  // set deepcopy flag to false to disable copying during attach in all future calls
  sim_spec_setDeepCopy(spec, false);
}



// compiler section parser
void sim_xml_reader_t::Compiler(XMLElement* section, sim_spec_t* s) {
  string text;
  int n;

  // top-level attributes
  if (MapValue(section, "autolimits", &n, bool_map, 2)) {
    s->compiler.autolimits = (n == 1);
  }
  ReadAttr(section, "boundmass", 1, &s->compiler.boundmass, text);
  ReadAttr(section, "boundinertia", 1, &s->compiler.boundinertia, text);
  ReadAttr(section, "settotalmass", 1, &s->compiler.settotalmass, text);
  if (MapValue(section, "balanceinertia", &n, bool_map, 2)) {
    s->compiler.balanceinertia = (n == 1);
  }
  if (MapValue(section, "strippath", &n, bool_map, 2)) {
    s->strippath = (n == 1);
  }
  if (MapValue(section, "fitaabb", &n, bool_map, 2)) {
    s->compiler.fitaabb = (n == 1);
  }
  if (MapValue(section, "coordinate", &n, coordinate_map, 2)) {
    if (n == 1) {
      throw sim_xml_error_t(section, "global coordinates no longer supported. To convert existing models, "
                     "load and save them in SimCore 2.3.3 or older");
    }
  }
  if (MapValue(section, "angle", &n, angle_map, 2)) {
    s->compiler.degree = (n == 1);
  }
  if (ReadAttrTxt(section, "eulerseq", text)) {
    if (text.size() != 3) {
      throw sim_xml_error_t(section, "euler format must have length 3");
    }
    memcpy(s->compiler.eulerseq, text.c_str(), 3);
  }
  if (ReadAttrTxt(section, "assetdir", text)) {
    sim_spec_set_string(s->compiler.meshdir, text.c_str());
  }
  // meshdir takes precedence over assetdir
  string meshdir;
  if (ReadAttrTxt(section, "meshdir", meshdir)) {
    sim_spec_set_string(s->compiler.meshdir, meshdir.c_str());
  };
  string texturedir;
  if (ReadAttrTxt(section, "texturedir", texturedir)) {
    sim_warning("Ignoring deprecated compiler texturedir attribute at line %d.", section->GetLineNum());
  }
  if (MapValue(section, "discardvisual", &n, bool_map, 2)) {
    s->compiler.discardvisual = (n == 1);
  }
  if (MapValue(section, "usethread", &n, bool_map, 2)) {
    s->compiler.usethread = (n == 1);
  }
  if (MapValue(section, "fusestatic", &n, bool_map, 2)) {
    s->compiler.fusestatic = (n == 1);
  }
  MapValue(section, "inertiafromgeom", &s->compiler.inertiafromgeom, TFAuto_map, 3);
  ReadAttr(section, "inertiagrouprange", 2, s->compiler.inertiagrouprange, text);
  if (MapValue(section, "alignfree", &n, bool_map, 2)) {
    s->compiler.alignfree = (n == 1);
  }
  if (MapValue(section, "saveinertial", &n, bool_map, 2)) {
    s->compiler.saveinertial = (n == 1);
  }

  // lengthrange subelement
  XMLElement* elem = FindSubElem(section, "lengthrange");
  if (elem) {
    SIM_LROpt* opt = &(s->compiler.LRopt);

    // flags
    MapValue(elem, "mode", &opt->mode, lrmode_map, lrmode_sz);
    if (MapValue(elem, "useexisting", &n, bool_map, 2)) {
      opt->useexisting = (n == 1);
    }
    if (MapValue(elem, "uselimit", &n, bool_map, 2)) {
      opt->uselimit = (n == 1);
    }

    // algorithm parameters
    ReadAttr(elem, "accel", 1, &opt->accel, text);
    ReadAttr(elem, "maxforce", 1, &opt->maxforce, text);
    ReadAttr(elem, "timeconst", 1, &opt->timeconst, text);
    ReadAttr(elem, "timestep", 1, &opt->timestep, text);
    ReadAttr(elem, "inttotal", 1, &opt->inttotal, text);
    ReadAttr(elem, "interval", 1, &opt->interval, text);
    ReadAttr(elem, "tolrange", 1, &opt->tolrange, text);
  }
}



// option section parser
void sim_xml_reader_t::Option(XMLElement* section, SIM_Option* opt) {
  string text;
  int n;

  // read options
  ReadAttr(section, "timestep", 1, &opt->timestep, text);
  ReadAttr(section, "impratio", 1, &opt->impratio, text);
  ReadAttr(section, "tolerance", 1, &opt->tolerance, text);
  ReadAttr(section, "ls_tolerance", 1, &opt->ls_tolerance, text);
  ReadAttr(section, "noslip_tolerance", 1, &opt->noslip_tolerance, text);
  ReadAttr(section, "ccd_tolerance", 1, &opt->ccd_tolerance, text);
  ReadAttr(section, "sleep_tolerance", 1, &opt->sleep_tolerance, text);
  ReadAttr(section, "gravity", 3, opt->gravity, text);
  ReadAttr(section, "wind", 3, opt->wind, text);
  ReadAttr(section, "magnetic", 3, opt->magnetic, text);
  ReadAttr(section, "density", 1, &opt->density, text);
  ReadAttr(section, "viscosity", 1, &opt->viscosity, text);

  ReadAttr(section, "o_margin", 1, &opt->o_margin, text);
  ReadAttr(section, "o_solref", SIM_NREF, opt->o_solref, text, false, false);
  ReadAttr(section, "o_solimp", SIM_NIMP, opt->o_solimp, text, false, false);
  ReadAttr(section, "o_friction", 5, opt->o_friction, text, false, false);

  MapValue(section, "integrator", &opt->integrator, integrator_map, integrator_sz);
  MapValue(section, "cone", &opt->cone, cone_map, cone_sz);
  MapValue(section, "jacobian", &opt->jacobian, jac_map, jac_sz);
  MapValue(section, "solver", &opt->solver, solver_map, solver_sz);
  ReadAttrInt(section, "iterations", &opt->iterations);
  ReadAttrInt(section, "ls_iterations", &opt->ls_iterations);
  ReadAttrInt(section, "noslip_iterations", &opt->noslip_iterations);
  ReadAttrInt(section, "ccd_iterations", &opt->ccd_iterations);
  ReadAttrInt(section, "sdf_iterations", &opt->sdf_iterations);
  ReadAttrInt(section, "sdf_initpoints", &opt->sdf_initpoints);

  // actuatorgroupdisable
  constexpr int num_bitflags = 31;
  int disabled_act_groups[num_bitflags];
  int num_found = ReadAttr(section, "actuatorgroupdisable", num_bitflags, disabled_act_groups,
                           text, false, false);
  for (int i=0; i < num_found; i++) {
    int group = disabled_act_groups[i];
    if (group < 0) {
      throw sim_xml_error_t(section, "disabled actuator group value must be non-negative");
    }
    if (group > num_bitflags - 1) {
      throw sim_xml_error_t(section, "disabled actuator group value cannot exceed 30");
    }
    opt->disableactuator |= (1 << group);
  }

  // read disable sub-element
  XMLElement* elem = FindSubElem(section, "flag");
  if (elem) {
#define READDSBL(NAME, MASK) \
        if (MapValue(elem, NAME, &n, enable_map, 2)) { \
            opt->disableflags ^= (opt->disableflags & MASK); \
            opt->disableflags |= (n ? 0 : MASK); }

    READDSBL("constraint",   SIM_DSBL_CONSTRAINT)
    READDSBL("equality",     SIM_DSBL_EQUALITY)
    READDSBL("frictionloss", SIM_DSBL_FRICTIONLOSS)
    READDSBL("limit",        SIM_DSBL_LIMIT)
    READDSBL("contact",      SIM_DSBL_CONTACT)
    READDSBL("spring",       SIM_DSBL_SPRING)
    READDSBL("damper",       SIM_DSBL_DAMPER)
    READDSBL("gravity",      SIM_DSBL_GRAVITY)
    READDSBL("clampctrl",    SIM_DSBL_CLAMPCTRL)
    READDSBL("warmstart",    SIM_DSBL_WARMSTART)
    READDSBL("filterparent", SIM_DSBL_FILTERPARENT)
    READDSBL("actuation",    SIM_DSBL_ACTUATION)
    READDSBL("refsafe",      SIM_DSBL_REFSAFE)
    READDSBL("sensor",       SIM_DSBL_SENSOR)
    READDSBL("midphase",     SIM_DSBL_MIDPHASE)
    READDSBL("eulerdamp",    SIM_DSBL_EULERDAMP)
    READDSBL("autoreset",    SIM_DSBL_AUTORESET)
    READDSBL("nativeccd",    SIM_DSBL_NATIVECCD)
    READDSBL("island",       SIM_DSBL_ISLAND)
#undef READDSBL

#define READENBL(NAME, MASK) \
        if (MapValue(elem, NAME, &n, enable_map, 2)) { \
            opt->enableflags ^= (opt->enableflags & MASK); \
            opt->enableflags |= (n ? MASK : 0); }

    READENBL("override",    SIM_ENBL_OVERRIDE)
    READENBL("energy",      SIM_ENBL_ENERGY)
    READENBL("fwdinv",      SIM_ENBL_FWDINV)
    READENBL("invdiscrete", SIM_ENBL_INVDISCRETE)
    READENBL("multiccd",    SIM_ENBL_MULTICCD)
    READENBL("sleep",       SIM_ENBL_SLEEP)
#undef READENBL
  }
}



// size section parser
void sim_xml_reader_t::Size(XMLElement* section, sim_spec_t* s) {
  // read memory bytes
  {
    constexpr char err_msg[] =
      "unsigned integer with an optional suffix {K,M,G,T,P,E} is expected in "
      "attribute 'memory' (or the size specified is too big)";

    auto memory = [&]() -> std::optional<std::size_t> {
      const char* pstr = section->Attribute("memory");
      if (!pstr) {
        return std::nullopt;
      }

      // trim entire string
      string trimmed;
      {
        std::istringstream strm((string(pstr)));
        strm >> trimmed;
        string trailing;
        strm >> trailing;
        if (!trailing.empty() || !strm.eof()) {
          throw sim_xml_error_t(section, "%s", err_msg);
        }

        // allow explicit specification of the default "-1" value
        if (trimmed == "-1") {
          return std::nullopt;
        }
      }

      std::istringstream strm(trimmed);

      // check that the number is not negative
      if (strm.peek() == '-') {
        throw sim_xml_error_t(section, "%s", err_msg);
      }

      std::size_t base_size;
      strm >> base_size;
      if (strm.fail()) {
        // either not an integer or the number without the suffix is already bigger than size_t
        throw sim_xml_error_t(section, "%s", err_msg);
      }

      // parse the multiplier suffix
      int multiplier_bit = 0;
      if (!strm.eof()) {
        char suffix = strm.get();
        if (suffix == 'K' || suffix == 'k') {
          multiplier_bit = 10;
        } else if (suffix == 'M' || suffix == 'm') {
          multiplier_bit = 20;
        } else if (suffix == 'G' || suffix == 'g') {
          multiplier_bit = 30;
        } else if (suffix == 'T' || suffix == 't') {
          multiplier_bit = 40;
        } else if (suffix == 'P' || suffix == 'p') {
          multiplier_bit = 50;
        } else if (suffix == 'E' || suffix == 'e') {
          multiplier_bit = 60;
        }

        // check for invalid suffix, or suffix longer than one character
        strm.get();
        if (!multiplier_bit || !strm.eof()) {
          throw sim_xml_error_t(section, "%s", err_msg);
        }
      }

      // check that the specified suffix isn't bigger than size_t
      if (multiplier_bit + 1 > std::numeric_limits<std::size_t>::digits) {
        throw sim_xml_error_t(section, "%s", err_msg);
      }

      // check that the suffix won't take the total size beyond size_t
      const std::size_t max_base_size =
          (std::numeric_limits<std::size_t>::max() << multiplier_bit) >> multiplier_bit;
      if (base_size > max_base_size) {
        throw sim_xml_error_t(section, "%s", err_msg);
      }

      const std::size_t total_size = base_size << multiplier_bit;
      return total_size;
    }();

    if (memory.has_value()) {
      if (*memory / sizeof(sim_scalar_t) > std::numeric_limits<std::size_t>::max()) {
        throw sim_xml_error_t(section, "%s", err_msg);
      }
      s->memory = *memory;
    }
  }

  // read sizes
  ReadAttrInt(section, "nuserdata", &s->nuserdata);
  ReadAttrInt(section, "nkey", &s->nkey);

  ReadAttrInt(section, "nconmax", &s->nconmax);
  if (s->nconmax < -1) throw sim_xml_error_t(section, "nconmax must be >= -1");

  {
    int nstack = -1;
    const bool has_nstack = ReadAttrInt(section, "nstack", &nstack);
    if (has_nstack) {
      if (s->nstack < -1) {
        throw sim_xml_error_t(section, "nstack must be >= -1");
      }
      if (s->memory != -1 && nstack != -1) {
        throw sim_xml_error_t(section,
                       "either 'memory' and 'nstack' attribute can be specified, not both");
      }
      s->nstack = nstack;
    }
  }
  {
    int njmax = -1;
    const bool has_njmax = ReadAttrInt(section, "njmax", &njmax);
    if (has_njmax) {
      if (s->njmax < -1) {
        throw sim_xml_error_t(section, "njmax must be >= -1");
      }
      if (s->memory != -1 && njmax != -1) {
        throw sim_xml_error_t(section,
                       "either 'memory' and 'njmax' attribute can be specified, not both");
      }
      s->njmax = njmax;
    }
  }

  ReadAttrInt(section, "nuser_body", &s->nuser_body);
  if (s->nuser_body < -1) throw sim_xml_error_t(section, "nuser_body must be >= -1");

  ReadAttrInt(section, "nuser_jnt", &s->nuser_jnt);
  if (s->nuser_jnt < -1) throw sim_xml_error_t(section, "nuser_jnt must be >= -1");

  ReadAttrInt(section, "nuser_geom", &s->nuser_geom);
  if (s->nuser_geom < -1) throw sim_xml_error_t(section, "nuser_geom must be >= -1");

  ReadAttrInt(section, "nuser_site", &s->nuser_site);
  if (s->nuser_site < -1) throw sim_xml_error_t(section, "nuser_site must be >= -1");

  ReadAttrInt(section, "nuser_cam", &s->nuser_cam);
  if (s->nuser_cam < -1) throw sim_xml_error_t(section, "nuser_cam must be >= -1");

  ReadAttrInt(section, "nuser_tendon", &s->nuser_tendon);
  if (s->nuser_tendon < -1) throw sim_xml_error_t(section, "nuser_tendon must be >= -1");

  ReadAttrInt(section, "nuser_actuator", &s->nuser_actuator);
  if (s->nuser_actuator < -1) throw sim_xml_error_t(section, "nuser_actuator must be >= -1");

  ReadAttrInt(section, "nuser_sensor", &s->nuser_sensor);
  if (s->nuser_sensor < -1) throw sim_xml_error_t(section, "nuser_sensor must be >= -1");
}



// statistic section parser
void sim_xml_reader_t::Statistic(XMLElement* section) {
  string text;

  // read statistics
  ReadAttr(section, "meaninertia", 1, &spec->stat.meaninertia, text);
  ReadAttr(section, "meanmass", 1, &spec->stat.meanmass, text);
  ReadAttr(section, "meansize", 1, &spec->stat.meansize, text);
  ReadAttr(section, "extent", 1, &spec->stat.extent, text);
  if (sim_math_internal_defined(spec->stat.extent) && spec->stat.extent <= 0) {
    throw sim_xml_error_t(section, "extent must be strictly positive");
  }
  ReadAttr(section, "center", 3, spec->stat.center, text);
}



//---------------------------------- one-element parsers -------------------------------------------

// flex element parser
void sim_xml_reader_t::OneFlex(XMLElement* elem, SIM_sFlex* flex) {
  string text, name, nodebody;
  int n;

  // read attributes
  if (ReadAttrTxt(elem, "name", name)) {
    if (sim_spec_set_name(flex->element, name.c_str())) {
      throw sim_xml_error_t(elem, "%s", sim_spec_getError(spec));
    }
  }
  string material;
  if (ReadAttrTxt(elem, "material", material)) {
    sim_warning("Ignoring flex material attribute at line %d.", elem->GetLineNum());
  }

  ReadAttr(elem, "radius", 1, &flex->radius, text);
  ReadAttr(elem, "rgba", 4, flex->rgba, text);
  if (MapValue(elem, "flatskin", &n, bool_map, 2)) {
    flex->flatskin = (n == 1);
  }
  ReadAttrInt(elem, "dim", &flex->dim);
  ReadAttrInt(elem, "group", &flex->group);

  // read data vectors
  if (ReadAttrTxt(elem, "body", text, true)) {
    sim_spec_setStringVec(flex->vertbody, text.c_str());
  }
  if (ReadAttrTxt(elem, "node", nodebody)) {
    sim_spec_setStringVec(flex->nodebody, nodebody.c_str());
  }
  auto vert = ReadAttrVec<double>(elem, "vertex");
  if (vert.has_value()) {
    sim_spec_setDouble(flex->vert, vert->data(), vert->size());
  }
  auto element = ReadAttrVec<int>(elem, "element", true);
  if (element.has_value()) {
    sim_spec_setInt(flex->elem, element->data(), element->size());
  }
  auto texcoord = ReadAttrVec<float>(elem, "texcoord");
  if (texcoord.has_value()) {
    sim_spec_setFloat(flex->texcoord, texcoord->data(), texcoord->size());
  }
  auto elemtexcoord = ReadAttrVec<int>(elem, "elemtexcoord");
  if (elemtexcoord.has_value()) {
    sim_spec_setInt(flex->elemtexcoord, elemtexcoord->data(), elemtexcoord->size());
  }

  // contact subelement
  XMLElement* cont = FirstChildElement(elem, "contact");
  if (cont) {
    ReadAttrInt(cont, "contype", &flex->contype);
    ReadAttrInt(cont, "conaffinity", &flex->conaffinity);
    ReadAttrInt(cont, "condim", &flex->condim);
    ReadAttrInt(cont, "priority", &flex->priority);
    ReadAttr(cont, "friction", 3, flex->friction, text, false, false);
    ReadAttr(cont, "solmix", 1, &flex->solmix, text);
    ReadAttr(cont, "solref", SIM_NREF, flex->solref, text, false, false);
    ReadAttr(cont, "solimp", SIM_NIMP, flex->solimp, text, false, false);
    ReadAttr(cont, "margin", 1, &flex->margin, text);
    ReadAttr(cont, "gap", 1, &flex->gap, text);
    if (MapValue(cont, "internal", &n, bool_map, 2)) {
      flex->internal = (n == 1);
    }
    MapValue(cont, "selfcollide", &flex->selfcollide, flexself_map, 5);
    if (MapValue(cont, "vertcollide", &flex->vertcollide, bool_map, 2)) {
      flex->vertcollide = (n == 1);
    }
    if (MapValue(cont, "passive", &flex->passive, bool_map, 2)) {
      flex->passive = (n == 1);
    }
    ReadAttrInt(cont, "activelayers", &flex->activelayers);
  }

  // edge subelement
  XMLElement* edge = FirstChildElement(elem, "edge");
  if (edge) {
    ReadAttr(edge, "stiffness", 1, &flex->edgestiffness, text);
    ReadAttr(edge, "damping", 1, &flex->edgedamping, text);
  }

  // elasticity subelement
  XMLElement* elasticity = FirstChildElement(elem, "elasticity");
  if (elasticity) {
    ReadAttr(elasticity, "young", 1, &flex->young, text);
    ReadAttr(elasticity, "poisson", 1, &flex->poisson, text);
    ReadAttr(elasticity, "thickness", 1, &flex->thickness, text);
    ReadAttr(elasticity, "damping", 1, &flex->damping, text);
    MapValue(elasticity, "elastic2d", &flex->elastic2d, elastic2d_map, 4);
  }

  // write error info
  sim_spec_set_string(flex->info, ("line " + std::to_string(elem->GetLineNum())).c_str());
}



// mesh element parser
void sim_xml_reader_t::OneMesh(XMLElement* elem, SIM_sMesh* mesh, const SIM_VFS* vfs) {
  int n;
  string text, name, content_type;

  // read attributes
  if (ReadAttrTxt(elem, "name", name)) {
    if (sim_spec_set_name(mesh->element, name.c_str())) {
      throw sim_xml_error_t(elem, "%s", sim_spec_getError(spec));
    }
  }
  if (ReadAttrTxt(elem, "content_type", content_type)) {
    *mesh->content_type = content_type;
  }
  auto file = ReadAttrFile(elem, "file", vfs, MeshDir());
  if (file) {
    sim_spec_set_string(mesh->file, file->c_str());
  }
  ReadAttr(elem, "refpos", 3, mesh->refpos, text);
  ReadAttr(elem, "refquat", 4, mesh->refquat, text);
  ReadAttr(elem, "scale", 3, mesh->scale, text);
  if (MapValue(elem, "inertia", &n, meshinertia_map, 4)) {
    mesh->inertia = (SIM_tMeshInertia)n;
  }

  XMLElement* eplugin = FirstChildElement(elem, "plugin");
  if (eplugin) {
    OnePlugin(eplugin, &mesh->plugin);
  }

  if (MapValue(elem, "smoothnormal", &n, bool_map, 2)) {
    mesh->smoothnormal = (n == 1);
  }

  if (ReadAttrInt(elem, "maxhullvert", &n)) {
    if (n != -1 && n < 4) throw sim_xml_error_t(elem, "maxhullvert must be larger than 3");
    mesh->maxhullvert = n;
  }

  // read user vertex data
  if (ReadAttrTxt(elem, "vertex", text)) {
    auto uservert = ReadAttrVec<float>(elem, "vertex");
    if (uservert.has_value()) {
      sim_spec_setFloat(mesh->uservert, uservert->data(), uservert->size());
    }
  }

  // read user normal data
  if (ReadAttrTxt(elem, "normal", text)) {
    auto usernormal = ReadAttrVec<float>(elem, "normal");
    if (usernormal.has_value()) {
      sim_spec_setFloat(mesh->usernormal, usernormal->data(), usernormal->size());
    }
  }

  // read user texcoord data
  if (ReadAttrTxt(elem, "texcoord", text)) {
    auto usertexcoord = ReadAttrVec<float>(elem, "texcoord");
    if (usertexcoord.has_value()) {
      sim_spec_setFloat(mesh->usertexcoord, usertexcoord->data(), usertexcoord->size());
    }
  }

  // read user face data
  if (ReadAttrTxt(elem, "face", text)) {
    auto userface = ReadAttrVec<int>(elem, "face");
    if (userface.has_value()) {
      sim_spec_setInt(mesh->userface, userface->data(), userface->size());
    }
  }

  // read builtin options
  if (MapValue(elem, "builtin", &n, meshbuiltin_map, meshbuiltin_sz)) {
    std::vector<double> params;
    int nparams = ReadVector(elem, "params", params, text, /*required*/ true);
    if (file) {
      throw sim_xml_error_t(elem, "builtin cannot be used with a mesh file");
    }
    if (!mesh->uservert->empty()) {
      throw sim_xml_error_t(elem, "builtin mesh cannot be used with user vertex data");
    }
    if (sim_spec_makeMesh(mesh, (SIM_tMeshBuiltin)n, params.data(), nparams)) {
      throw sim_xml_error_t(elem, "%s", sim_spec_getError(spec));
    }
  }

  std::string material;
  if (ReadAttrTxt(elem, "material", material)) {
    sim_warning("Ignoring mesh material attribute at line %d.", elem->GetLineNum());
  }

  // write error info
  sim_spec_set_string(mesh->info, ("line " + std::to_string(elem->GetLineNum())).c_str());
}



// skin element parser
void sim_xml_reader_t::OneSkin(XMLElement* elem, SIM_sSkin* skin, const SIM_VFS* vfs) {
  string text, name, material;
  float data[4];

  // read attributes
  if (ReadAttrTxt(elem, "name", name)) {
    if (sim_spec_set_name(skin->element, name.c_str())) {
      throw sim_xml_error_t(elem, "%s", sim_spec_getError(spec));
    }
  }
  auto file = ReadAttrFile(elem, "file", vfs, AssetDir());
  if (file.has_value()) {
    sim_spec_set_string(skin->file, file->c_str());
  }
  if (ReadAttrTxt(elem, "material", material)) {
    sim_warning("Ignoring skin material attribute at line %d.", elem->GetLineNum());
  }
  ReadAttrInt(elem, "group", &skin->group);
  if (skin->group < 0 || skin->group >= SIM_NGROUP) {
    throw sim_xml_error_t(elem, "skin group must be between 0 and 5");
  }
  ReadAttr(elem, "rgba", 4, skin->rgba, text);
  ReadAttr(elem, "inflate", 1, &skin->inflate, text);

  // read vertex data
  auto vertex = ReadAttrVec<float>(elem, "vertex");
  if (vertex.has_value()) {
    sim_spec_setFloat(skin->vert, vertex->data(), vertex->size());
  }

  // read texcoord data
  auto texcoord = ReadAttrVec<float>(elem, "texcoord");
  if (texcoord.has_value()) {
    sim_spec_setFloat(skin->texcoord, texcoord->data(), texcoord->size());
  }

  // read user face data
  auto face = ReadAttrVec<int>(elem, "face");
  if (face.has_value()) {
    sim_spec_setInt(skin->face, face->data(), face->size());
  }

  // read bones
  XMLElement* bone = FirstChildElement(elem, "bone");
  std::vector<float> bindpos;
  std::vector<float> bindquat;

  while (bone) {
    // read body
    ReadAttrTxt(bone, "body", text, true);
    sim_spec_appendString(skin->bodyname, text.c_str());

    // read bindpos
    ReadAttr(bone, "bindpos", 3, data, text, true);
    bindpos.push_back(data[0]);
    bindpos.push_back(data[1]);
    bindpos.push_back(data[2]);

    // read bindquat
    ReadAttr(bone, "bindquat", 4, data, text, true);
    bindquat.push_back(data[0]);
    bindquat.push_back(data[1]);
    bindquat.push_back(data[2]);
    bindquat.push_back(data[3]);

    // read vertid
    auto tempid = ReadAttrVec<int>(bone, "vertid", true);
    if (tempid.has_value()) {
      sim_spec_appendIntVec(skin->vertid, tempid->data(), tempid->size());
    }

    // read vertweight
    auto tempweight = ReadAttrVec<float>(bone, "vertweight", true);
    if (tempweight.has_value()) {
      sim_spec_appendFloatVec(skin->vertweight, tempweight->data(), tempweight->size());
    }

    // advance to next bone
    bone = NextSiblingElement(bone, "bone");
  }

  // set bind vectors
  sim_spec_setFloat(skin->bindpos, bindpos.data(), bindpos.size());
  sim_spec_setFloat(skin->bindquat, bindquat.data(), bindquat.size());

  // write error info
  sim_spec_set_string(skin->info, ("line " + std::to_string(elem->GetLineNum())).c_str());
}



// material element parser
void sim_xml_reader_t::OneMaterial(XMLElement* elem, SIM_sMaterial* material) {
  string text, name, texture;
  int n;

  // read attributes
  if (ReadAttrTxt(elem, "name", name)) {
    if (sim_spec_set_name(material->element, name.c_str())) {
      throw sim_xml_error_t(elem, "%s", sim_spec_getError(spec));
    }
  }

  bool tex_attributes_found = false;
  if (ReadAttrTxt(elem, "texture", texture)) {
    sim_spec_setInStringVec(material->textures, SIM_TEXROLE_RGB, texture.c_str());
    tex_attributes_found = true;
  }

  XMLElement* layer = FirstChildElement(elem);
  while (layer) {
    if (tex_attributes_found) {
      throw sim_xml_error_t(layer, "A material with a texture attribute cannot have layer sub-elements");
    }

    // layer sub-element
    ReadAttrTxt(layer, "role", text, true);
    int role = FindKey(texrole_map, texrole_sz, text);
    ReadAttrTxt(layer, "texture", text, true);
    sim_spec_setInStringVec(material->textures, role, text.c_str());
    layer = NextSiblingElement(layer);
  }

  if (MapValue(elem, "texuniform", &n, bool_map, 2)) {
    material->texuniform = (n == 1);
  }
  ReadAttr(elem, "texrepeat", 2, material->texrepeat, text);
  ReadAttr(elem, "emission", 1, &material->emission, text);
  ReadAttr(elem, "specular", 1, &material->specular, text);
  ReadAttr(elem, "shininess", 1, &material->shininess, text);
  ReadAttr(elem, "reflectance", 1, &material->reflectance, text);
  ReadAttr(elem, "metallic", 1, &material->metallic, text);
  ReadAttr(elem, "roughness", 1, &material->roughness, text);
  ReadAttr(elem, "rgba", 4, material->rgba, text);

  // write error info
  sim_spec_set_string(material->info, ("line " + std::to_string(elem->GetLineNum())).c_str());
}



// joint element parser
void sim_xml_reader_t::OneJoint(XMLElement* elem, SIM_sJoint* joint) {
  string text, name;
  std::vector<double> userdata;
  int n;

  // read attributes
  if (ReadAttrTxt(elem, "name", name)) {
    if (sim_spec_set_name(joint->element, name.c_str())) {
      throw sim_xml_error_t(elem, "%s", sim_spec_getError(spec));
    }
  }
  if (MapValue(elem, "type", &n, joint_map, joint_sz)) {
    joint->type = (SIM_tJoint)n;
  }
  MapValue(elem, "limited", &joint->limited, TFAuto_map, 3);
  MapValue(elem, "actuatorfrclimited", &joint->actfrclimited, TFAuto_map, 3);
  ReadAttrInt(elem, "group", &joint->group);
  ReadAttr(elem, "solreflimit", SIM_NREF, joint->solref_limit, text, false, false);
  ReadAttr(elem, "solimplimit", SIM_NIMP, joint->solimp_limit, text, false, false);
  ReadAttr(elem, "solreffriction", SIM_NREF, joint->solref_friction, text, false, false);
  ReadAttr(elem, "solimpfriction", SIM_NIMP, joint->solimp_friction, text, false, false);
  ReadAttr(elem, "pos", 3, joint->pos, text);
  ReadAttr(elem, "axis", 3, joint->axis, text);
  ReadAttr(elem, "springdamper", 2, joint->springdamper, text);
  ReadAttr(elem, "stiffness", 1, &joint->stiffness, text);
  ReadAttr(elem, "range", 2, joint->range, text);
  ReadAttr(elem, "actuatorfrcrange", 2, joint->actfrcrange, text);
  ReadAttr(elem, "margin", 1, &joint->margin, text);
  ReadAttr(elem, "ref", 1, &joint->ref, text);
  ReadAttr(elem, "springref", 1, &joint->springref, text);
  ReadAttr(elem, "armature", 1, &joint->armature, text);
  ReadAttr(elem, "damping", 1, &joint->damping, text);
  ReadAttr(elem, "frictionloss", 1, &joint->frictionloss, text);
  if (MapValue(elem, "actuatorgravcomp", &n, bool_map, 2)) {
    joint->actgravcomp = (n == 1);
  }

  // read userdata
  if (ReadVector(elem, "user", userdata, text)) {
    sim_spec_setDouble(joint->userdata, userdata.data(), userdata.size());
  }

  // write error info
  sim_spec_set_string(joint->info, ("line " + std::to_string(elem->GetLineNum())).c_str());
}



// geom element parser
void sim_xml_reader_t::OneGeom(XMLElement* elem, sim_spec_geom_t* geom) {
  string text, name;
  std::vector<double> userdata;
  string hfieldname, meshname, material;
  int n;

  // read attributes
  if (ReadAttrTxt(elem, "name", name)) {
    if (sim_spec_set_name(geom->element, name.c_str())) {
      throw sim_xml_error_t(elem, "%s", sim_spec_getError(spec));
    }
  }
  if (MapValue(elem, "type", &n, geom_map, SIM_NGEOMTYPES)) {
    geom->type = (SIM_tGeom)n;
  }
  ReadAttr(elem, "size", 3, geom->size, text, false, false);
  ReadAttrInt(elem, "contype", &geom->contype);
  ReadAttrInt(elem, "conaffinity", &geom->conaffinity);
  ReadAttrInt(elem, "condim", &geom->condim);
  ReadAttrInt(elem, "group", &geom->group);
  ReadAttrInt(elem, "priority", &geom->priority);
  ReadAttr(elem, "friction", 3, geom->friction, text, false, false);
  ReadAttr(elem, "solmix", 1, &geom->solmix, text);
  ReadAttr(elem, "solref", SIM_NREF, geom->solref, text, false, false);
  ReadAttr(elem, "solimp", SIM_NIMP, geom->solimp, text, false, false);
  ReadAttr(elem, "margin", 1, &geom->margin, text);
  ReadAttr(elem, "gap", 1, &geom->gap, text);
  if (ReadAttrTxt(elem, "hfield", hfieldname)) {
    sim_spec_set_string(geom->hfieldname, hfieldname.c_str());
  }
  if (ReadAttrTxt(elem, "mesh", meshname)) {
    sim_spec_set_string(geom->meshname, meshname.c_str());
  }
  ReadAttr(elem, "fitscale", 1, &geom->fitscale, text);
  if (ReadAttrTxt(elem, "material", material)) {
    sim_warning("Ignoring geom material attribute at line %d.", elem->GetLineNum());
  }
  ReadAttr(elem, "rgba", 4, geom->rgba, text);
  if (MapValue(elem, "fluidshape", &n, fluid_map, 2)) {
    geom->fluid_ellipsoid = (n == 1);
  }
  ReadAttr(elem, "fluidcoef", 5, geom->fluid_coefs, text, false, false);

  // read userdata
  if (ReadVector(elem, "user", userdata, text)) {
    sim_spec_setDouble(geom->userdata, userdata.data(), userdata.size());
  }

  // plugin sub-element
  XMLElement* eplugin = FirstChildElement(elem, "plugin");
  if (eplugin) {
    OnePlugin(eplugin, &geom->plugin);
  }

  // remaining attributes
  ReadAttr(elem, "mass", 1, &geom->mass, text);
  ReadAttr(elem, "density", 1, &geom->density, text);
  ReadAttr(elem, "fromto", 6, geom->fromto, text);
  ReadAttr(elem, "pos", 3, geom->pos, text);
  ReadQuat(elem, "quat", geom->quat, text);
  ReadAlternative(elem, geom->alt);

  // compute inertia using either solid or shell geometry
  if (MapValue(elem, "shellinertia", &n, meshtype_map, 2)) {
    geom->typeinertia = (SIM_tGeomInertia)n;
  }

  // write error info
  sim_spec_set_string(geom->info, ("line " + std::to_string(elem->GetLineNum())).c_str());
}



// site element parser
void sim_xml_reader_t::OneSite(XMLElement* elem, SIM_sSite* site) {
  int n;
  string text, name;
  std::vector<double> userdata;
  string material;

  // read attributes
  if (ReadAttrTxt(elem, "name", name)) {
    if (sim_spec_set_name(site->element, name.c_str())) {
      throw sim_xml_error_t(elem, "%s", sim_spec_getError(spec));
    }
  }
  if (MapValue(elem, "type", &n, geom_map, SIM_NGEOMTYPES)) {
    site->type = (SIM_tGeom)n;
  }
  ReadAttr(elem, "size", 3, site->size, text, false, false);
  ReadAttrInt(elem, "group", &site->group);
  ReadAttr(elem, "pos", 3, site->pos, text);
  ReadQuat(elem, "quat", site->quat, text);
  if (ReadAttrTxt(elem, "material", material)) {
    sim_warning("Ignoring site material attribute at line %d.", elem->GetLineNum());
  }
  ReadAttr(elem, "rgba", 4, site->rgba, text);
  ReadAttr(elem, "fromto", 6, site->fromto, text);
  ReadAlternative(elem, site->alt);
  if (ReadVector(elem, "user", userdata, text)) {
    sim_spec_setDouble(site->userdata, userdata.data(), userdata.size());
  }

  // write error info
  sim_spec_set_string(site->info, ("line " + std::to_string(elem->GetLineNum())).c_str());
}



// camera element parser
void sim_xml_reader_t::OneCamera(XMLElement* elem, SIM_sCamera* camera) {
  int n;
  string text, name, targetbody;
  std::vector<double> userdata;

  // read attributes
  if (ReadAttrTxt(elem, "name", name)) {
    if (sim_spec_set_name(camera->element, name.c_str())) {
      throw sim_xml_error_t(elem, "%s", sim_spec_getError(spec));
    }
  }
  if (ReadAttrTxt(elem, "target", targetbody)) {
    sim_spec_set_string(camera->targetbody, targetbody.c_str());
  }
  if (MapValue(elem, "mode", &n, camlight_map, camlight_sz)) {
    camera->mode = (SIM_tCamLight)n;
  }
  ReadAttr(elem, "pos", 3, camera->pos, text);
  ReadQuat(elem, "quat", camera->quat, text);
  ReadAlternative(elem, camera->alt);
  ReadAttr(elem, "ipd", 1, &camera->ipd, text);

  if (MapValue(elem, "projection", &n, projection_map, 2)) {
    camera->proj = (SIM_tProjection)n;
  }

  ReadAttr(elem, "principalpixel", 2, camera->principal_pixel, text);
  ReadAttr(elem, "principal", 2, camera->principal_length, text);
  ReadAttr(elem, "focalpixel", 2, camera->focal_pixel, text);
  ReadAttr(elem, "focal", 2, camera->focal_length, text);
  ReadAttr(elem, "resolution", 2, camera->resolution, text);

  // read output attribute as space-separated bitflags
  std::vector<int> outvals(SIM_NCAMOUT);
  int nout = MapValues(elem, "output", outvals.data(), camout_map, SIM_NCAMOUT);
  if (nout) {
    camera->output = 0;
    for (int i = 0; i < nout; ++i) {
      camera->output |= outvals[i];
    }
  }

  bool sensorsize = ReadAttr(elem, "sensorsize", 2, camera->sensor_size, text);
  bool fovy = ReadAttr(elem, "fovy", 1, &camera->fovy, text);
  if (fovy && sensorsize) {
    throw sim_xml_error_t(elem, "either 'fovy' or 'sensorsize' attribute can be specified, not both");
  }

  // read userdata
  ReadVector(elem, "user", userdata, text);
  sim_spec_setDouble(camera->userdata, userdata.data(), userdata.size());

  // write error info
  sim_spec_set_string(camera->info, ("line " + std::to_string(elem->GetLineNum())).c_str());
}



// light element parser
void sim_xml_reader_t::OneLight(XMLElement* elem, SIM_sLight* light) {
  int n;
  bool has_directional = false;
  string text, name, texture, targetbody;

  // read attributes
  if (ReadAttrTxt(elem, "name", name)) {
    if (sim_spec_set_name(light->element, name.c_str())) {
      throw sim_xml_error_t(elem, "%s", sim_spec_getError(spec));
    }
  }
  if (ReadAttrTxt(elem, "texture", texture)) {
    sim_spec_set_string(light->texture, texture.c_str());
  }
  if (ReadAttrTxt(elem, "target", targetbody)) {
    sim_spec_set_string(light->targetbody, targetbody.c_str());
  }
  if (MapValue(elem, "mode", &n, camlight_map, camlight_sz)) {
    light->mode = (SIM_tCamLight)n;
  }
  if (MapValue(elem, "directional", &n, bool_map, 2)) {
    light->type = (n == 1) ? SIM_LIGHT_DIRECTIONAL : SIM_LIGHT_SPOT;
    has_directional = true;
  }
  if (MapValue(elem, "type", &n, lighttype_map, lighttype_sz)) {
    if (has_directional) {
      throw sim_xml_error_t(elem, "type and directional cannot both be defined");
    }
    light->type = (SIM_tLightType)n;
  }
  if (MapValue(elem, "castshadow", &n, bool_map, 2)) {
    light->castshadow = (n == 1);
  }
  if (MapValue(elem, "active", &n, bool_map, 2)) {
    light->active = (n == 1);
  }
  ReadAttr(elem, "pos", 3, light->pos, text);
  ReadAttr(elem, "dir", 3, light->dir, text);
  ReadAttr(elem, "bulbradius", 1, &light->bulbradius, text);
  ReadAttr(elem, "intensity", 1, &light->intensity, text);
  ReadAttr(elem, "range", 1, &light->range, text);
  ReadAttr(elem, "attenuation", 3, light->attenuation, text);
  ReadAttr(elem, "cutoff", 1, &light->cutoff, text);
  ReadAttr(elem, "exponent", 1, &light->exponent, text);
  ReadAttr(elem, "ambient", 3, light->ambient, text);
  ReadAttr(elem, "diffuse", 3, light->diffuse, text);
  ReadAttr(elem, "specular", 3, light->specular, text);

  // write error info
  sim_spec_set_string(light->info, ("line " + std::to_string(elem->GetLineNum())).c_str());
}



// pair element parser
void sim_xml_reader_t::OnePair(XMLElement* elem, SIM_sPair* pair) {
  string text, name, geomname1, geomname2;

  // regular only
  if (!readingdefaults) {
    if (ReadAttrTxt(elem, "geom1", geomname1)) {
      sim_spec_set_string(pair->geomname1, geomname1.c_str());
    }
    if (ReadAttrTxt(elem, "geom2", geomname2)) {
      sim_spec_set_string(pair->geomname2, geomname2.c_str());
    }
  }

  // read other parameters
  if (ReadAttrTxt(elem, "name", name)) {
    if (sim_spec_set_name(pair->element, name.c_str())) {
      throw sim_xml_error_t(elem, "%s", sim_spec_getError(spec));
    }
  }
  ReadAttrInt(elem, "condim", &pair->condim);
  ReadAttr(elem, "solref", SIM_NREF, pair->solref, text, false, false);
  ReadAttr(elem, "solreffriction", SIM_NREF, pair->solreffriction, text, false, false);
  ReadAttr(elem, "solimp", SIM_NIMP, pair->solimp, text, false, false);
  ReadAttr(elem, "margin", 1, &pair->margin, text);
  ReadAttr(elem, "gap", 1, &pair->gap, text);
  ReadAttr(elem, "friction", 5, pair->friction, text, false, false);

  // write error info
  sim_spec_set_string(pair->info, ("line " + std::to_string(elem->GetLineNum())).c_str());
}



// equality element parser
void sim_xml_reader_t::OneEquality(XMLElement* elem, SIM_sEquality* equality) {
  int n;
  string text, name1, name2, name;

  // read type (bad keywords already detected by schema)
  text = elem->Value();
  equality->type = (SIM_tEq)FindKey(equality_map, equality_sz, text);

  // regular only
  if (!readingdefaults) {
    if (ReadAttrTxt(elem, "name", name)) {
      if (sim_spec_set_name(equality->element, name.c_str())) {
        throw sim_xml_error_t(elem, "%s", sim_spec_getError(spec));
      }
    }

    switch (equality->type) {
      case SIM_EQ_CONNECT: {
        auto maybe_site1 = ReadAttrStr(elem, "site1");
        auto maybe_site2 = ReadAttrStr(elem, "site2");
        auto maybe_body1 = ReadAttrStr(elem, "body1");
        auto maybe_body2 = ReadAttrStr(elem, "body2");
        bool has_anchor = ReadAttr(elem, "anchor", 3, equality->data, text);

        bool maybe_site = maybe_site1.has_value() || maybe_site2.has_value();
        bool maybe_body = maybe_body1.has_value() || maybe_body2.has_value() || has_anchor;

        if (maybe_site && maybe_body) {
          throw sim_xml_error_t(elem, "body and site semantics cannot be mixed");
        }

        bool site_semantic = maybe_site1.has_value() && maybe_site2.has_value();
        bool body_semantic = maybe_body1.has_value() && has_anchor;
        if (site_semantic == body_semantic) {
          throw sim_xml_error_t(elem, "either both body1 and anchor must be defined,"
                         " or both site1 and site2 must be defined");
        }

        if (body_semantic) {
          name1 = maybe_body1.value();
          if (maybe_body2.has_value()) {
            name2 = maybe_body2.value();
          }
          equality->objtype = SIM_OBJ_BODY;
        } else {
          name1 = maybe_site1.value();
          name2 = maybe_site2.value();
          equality->objtype = SIM_OBJ_SITE;
        }
      }
      break;

      case SIM_EQ_WELD: {
        auto maybe_site1 = ReadAttrStr(elem, "site1");
        auto maybe_site2 = ReadAttrStr(elem, "site2");
        auto maybe_body1 = ReadAttrStr(elem, "body1");
        auto maybe_body2 = ReadAttrStr(elem, "body2");
        bool has_anchor = ReadAttr(elem, "anchor", 3, equality->data, text);
        bool has_relpose = ReadAttr(elem, "relpose", 7, equality->data+3, text);

        bool maybe_site = maybe_site1.has_value() || maybe_site2.has_value();
        bool maybe_body = maybe_body1.has_value() ||
                          maybe_body2.has_value() ||
                          has_anchor              ||
                          has_relpose;

        if (maybe_site && maybe_body) {
          throw sim_xml_error_t(elem, "body and site semantics cannot be mixed");
        }

        bool site_semantic = maybe_site1.has_value() && maybe_site2.has_value();
        bool body_semantic = maybe_body1.has_value();

        if (site_semantic == body_semantic) {
          throw sim_xml_error_t(
                  elem,
                  "either body1 must be defined and optionally {body2, anchor, relpose},"
                  " or site1 and site2 must be defined");
        }

        if (body_semantic) {
          name1 = maybe_body1.value();
          if (maybe_body2.has_value()) {
            name2 = maybe_body2.value();
          }
          equality->objtype = SIM_OBJ_BODY;
          if (!has_anchor) {
            sim_math_internal_zerovec(equality->data, 3);
          }
        } else {
          name1 = maybe_site1.value();
          name2 = maybe_site2.value();
          equality->objtype = SIM_OBJ_SITE;
        }

        ReadAttr(elem, "torquescale", 1, equality->data+10, text);
      }
      break;

      case SIM_EQ_JOINT:
        ReadAttrTxt(elem, "joint1", name1, true);
        ReadAttrTxt(elem, "joint2", name2);
        ReadAttr(elem, "polycoef", 5, equality->data, text, false, false);
        break;

      case SIM_EQ_TENDON:
        ReadAttrTxt(elem, "tendon1", name1, true);
        ReadAttrTxt(elem, "tendon2", name2);
        ReadAttr(elem, "polycoef", 5, equality->data, text, false, false);
        break;

      case SIM_EQ_FLEX:
      case SIM_EQ_FLEXVERT:
        ReadAttrTxt(elem, "flex", name1, true);
        break;

      case SIM_EQ_DISTANCE:
        throw sim_xml_error_t(elem, "support for distance equality constraints was removed in SimCore 2.2.2");
        break;

      default:                  // SHOULD NOT OCCUR
        throw sim_xml_error_t(elem, "unrecognized equality constraint type");
    }

    sim_spec_set_string(equality->name1, name1.c_str());
    if (!name2.empty()) {
      sim_spec_set_string(equality->name2, name2.c_str());
    }
  }

  // read attributes
  if (MapValue(elem, "active", &n, bool_map, 2)) {
    equality->active = (n == 1);
  }
  ReadAttr(elem, "solref", SIM_NREF, equality->solref, text, false, false);
  ReadAttr(elem, "solimp", SIM_NIMP, equality->solimp, text, false, false);

  // write error info
  sim_spec_set_string(equality->info, ("line " + std::to_string(elem->GetLineNum())).c_str());
}



// tendon element parser
void sim_xml_reader_t::OneTendon(XMLElement* elem, SIM_sTendon* tendon) {
  string text, name, material;
  std::vector<double> userdata;

  // read attributes
  if (ReadAttrTxt(elem, "name", name)) {
    if (sim_spec_set_name(tendon->element, name.c_str())) {
      throw sim_xml_error_t(elem, "%s", sim_spec_getError(spec));
    }
  }
  ReadAttrInt(elem, "group", &tendon->group);
  if (ReadAttrTxt(elem, "material", material)) {
    sim_warning("Ignoring tendon material attribute at line %d.", elem->GetLineNum());
  }
  MapValue(elem, "limited", &tendon->limited, TFAuto_map, 3);
  MapValue(elem, "actuatorfrclimited", &tendon->actfrclimited, TFAuto_map, 3);
  ReadAttr(elem, "width", 1, &tendon->width, text);
  ReadAttr(elem, "solreflimit", SIM_NREF, tendon->solref_limit, text, false, false);
  ReadAttr(elem, "solimplimit", SIM_NIMP, tendon->solimp_limit, text, false, false);
  ReadAttr(elem, "solreffriction", SIM_NREF, tendon->solref_friction, text, false, false);
  ReadAttr(elem, "solimpfriction", SIM_NIMP, tendon->solimp_friction, text, false, false);
  ReadAttr(elem, "range", 2, tendon->range, text);
  ReadAttr(elem, "actuatorfrcrange", 2, tendon->actfrcrange, text);
  ReadAttr(elem, "margin", 1, &tendon->margin, text);
  ReadAttr(elem, "stiffness", 1, &tendon->stiffness, text);
  ReadAttr(elem, "damping", 1, &tendon->damping, text);
  ReadAttr(elem, "armature", 1, &tendon->armature, text);
  ReadAttr(elem, "frictionloss", 1, &tendon->frictionloss, text);
  // read springlength, either one or two values; if one, copy to second value
  if (ReadAttr(elem, "springlength", 2, tendon->springlength, text, false, false) == 1) {
    tendon->springlength[1] = tendon->springlength[0];
  }
  ReadAttr(elem, "rgba", 4, tendon->rgba, text);

  // read userdata
  if (ReadVector(elem, "user", userdata, text)) {
    sim_spec_setDouble(tendon->userdata, userdata.data(), userdata.size());
  }

  // write error info
  sim_spec_set_string(tendon->info, ("line " + std::to_string(elem->GetLineNum())).c_str());
}



// actuator element parser
void sim_xml_reader_t::OneActuator(XMLElement* elem, sim_spec_actuator_t* actuator) {
  string text, type, name, target, slidersite, refsite;

  // common attributes
  if (ReadAttrTxt(elem, "name", name)) {
    if (sim_spec_set_name(actuator->element, name.c_str())) {
      throw sim_xml_error_t(elem, "%s", sim_spec_getError(spec));
    }
  }
  ReadAttrInt(elem, "group", &actuator->group);
  ReadAttrInt(elem, "nsample", &actuator->nsample);
  MapValue(elem, "interp", &actuator->interp, interp_map, interp_sz);
  ReadAttr(elem, "delay", 1, &actuator->delay, text);
  MapValue(elem, "ctrllimited", &actuator->ctrllimited, TFAuto_map, 3);
  MapValue(elem, "forcelimited", &actuator->forcelimited, TFAuto_map, 3);
  MapValue(elem, "actlimited", &actuator->actlimited, TFAuto_map, 3);
  ReadAttr(elem, "ctrlrange", 2, actuator->ctrlrange, text);
  ReadAttr(elem, "forcerange", 2, actuator->forcerange, text);
  ReadAttr(elem, "actrange", 2, actuator->actrange, text);
  ReadAttr(elem, "lengthrange", 2, actuator->lengthrange, text);
  ReadAttr(elem, "gear", 6, actuator->gear, text, false, false);

  // transmission target and type
  int cnt = 0;
  if (ReadAttrTxt(elem, "joint", target)) {
    sim_spec_set_string(actuator->target, target.c_str());
    actuator->trntype = SIM_TRN_JOINT;
    cnt++;
  }
  if (ReadAttrTxt(elem, "jointinparent", target)) {
    sim_spec_set_string(actuator->target, target.c_str());
    actuator->trntype = SIM_TRN_JOINTINPARENT;
    cnt++;
  }
  if (ReadAttrTxt(elem, "tendon", target)) {
    sim_spec_set_string(actuator->target, target.c_str());
    actuator->trntype = SIM_TRN_TENDON;
    cnt++;
  }
  if (ReadAttrTxt(elem, "cranksite", target)) {
    sim_spec_set_string(actuator->target, target.c_str());
    actuator->trntype = SIM_TRN_SLIDERCRANK;
    cnt++;
  }
  if (ReadAttrTxt(elem, "site", target)) {
    sim_spec_set_string(actuator->target, target.c_str());
    actuator->trntype = SIM_TRN_SITE;
    cnt++;
  }
  if (ReadAttrTxt(elem, "body", target)) {
    sim_spec_set_string(actuator->target, target.c_str());
    actuator->trntype = SIM_TRN_BODY;
    cnt++;
  }
  // check for repeated transmission
  if (cnt > 1) {
    throw sim_xml_error_t(elem, "actuator can have at most one of transmission target");
  }

  // slidercrank-specific parameters
  int r1 = ReadAttr(elem, "cranklength", 1, &actuator->cranklength, text);
  int r2 = ReadAttrTxt(elem, "slidersite", slidersite);
  if (r2) {
    sim_spec_set_string(actuator->slidersite, slidersite.c_str());
  }
  if ((r1 || r2) &&
      actuator->trntype != SIM_TRN_SLIDERCRANK &&
      actuator->trntype != SIM_TRN_UNDEFINED) {
    throw sim_xml_error_t(elem, "cranklength and slidersite can only be used in slidercrank transmission");
  }

  // site-specific parameters (refsite)
  int r3 = ReadAttrTxt(elem, "refsite", refsite);
  if (r3) {
    sim_spec_set_string(actuator->refsite, refsite.c_str());
  }
  if (r3 && actuator->trntype != SIM_TRN_SITE && actuator->trntype != SIM_TRN_UNDEFINED) {
    throw sim_xml_error_t(elem, "refsite can only be used with site transmission");
  }

  // get predefined type
  type = elem->Value();

  // explicit attributes
  string err;
  if (type == "general") {
    // explicit attributes
    int n;
    if (MapValue(elem, "dyntype", &n, dyn_map, dyn_sz)) {
      actuator->dyntype = (SIM_tDyn)n;
    }
    if (MapValue(elem, "gaintype", &n, gain_map, gain_sz)) {
      actuator->gaintype = (SIM_tGain)n;
    }
    if (MapValue(elem, "biastype", &n, bias_map, bias_sz)) {
      actuator->biastype = (SIM_tBias)n;
    }
    if (MapValue(elem, "actearly", &n, bool_map, 2)) {
      actuator->actearly = (n == 1);
    }
    ReadAttr(elem, "dynprm", SIM_NDYN, actuator->dynprm, text, false, false);
    ReadAttr(elem, "gainprm", SIM_NGAIN, actuator->gainprm, text, false, false);
    ReadAttr(elem, "biasprm", SIM_NBIAS, actuator->biasprm, text, false, false);
    ReadAttrInt(elem, "actdim", &actuator->actdim);
  }

  // direct drive motor
  else if (type == "motor") {
    err = sim_spec_setToMotor(actuator);
  }

  // position or integrated velocity servo
  else if (type == "position" || type == "intvelocity") {
    double kp = actuator->gainprm[0];
    ReadAttr(elem, "kp", 1, &kp, text);

    // read kv
    double kv_data;
    double *kv = &kv_data;
    if (!ReadAttr(elem, "kv", 1, kv, text)) {
      kv = nullptr;
    }

    // read dampratio
    double dampratio_data;
    double *dampratio = &dampratio_data;
    if (!ReadAttr(elem, "dampratio", 1, dampratio, text)) {
      dampratio = nullptr;
    }

    // read timeconst, set dyntype
    double timeconst_data;
    double *timeconst = &timeconst_data;
    if (!ReadAttr(elem, "timeconst", 1, timeconst, text)) {
      timeconst = nullptr;
    }

    // handle inheritrange
    double inheritrange = actuator->inheritrange;
    ReadAttr(elem, "inheritrange", 1, &inheritrange, text);

    if (type == "position") {
      err = sim_spec_setToPosition(actuator, kp, kv, dampratio, timeconst, inheritrange);
    } else {
      err = sim_spec_setToIntVelocity(actuator, kp, kv, dampratio, timeconst, inheritrange);
    }
  }

  // velocity servo
  else if (type == "velocity") {
    double kv = actuator->gainprm[0];
    ReadAttr(elem, "kv", 1, &kv, text);
    err = sim_spec_setToVelocity(actuator, kv);
  }

  // damper
  else if (type == "damper") {
    double kv = 0;
    ReadAttr(elem, "kv", 1, &kv, text);
    err = sim_spec_setToDamper(actuator, kv);
  }

  // cylinder
  else if (type == "cylinder") {
    double timeconst = actuator->dynprm[0];
    double bias = actuator->biasprm[0];
    double area = actuator->gainprm[0];
    double diameter = -1;
    ReadAttr(elem, "timeconst", 1, &timeconst, text);
    ReadAttr(elem, "bias", 3, &bias, text);
    ReadAttr(elem, "area", 1, &area, text);
    ReadAttr(elem, "diameter", 1, &diameter, text);
    err = sim_spec_setToCylinder(actuator, timeconst, bias, area, diameter);
  }

  // muscle
  else if (type == "muscle") {
    double tausmooth = actuator->dynprm[2];
    double force = -1, scale = -1, lmin = -1, lmax = -1, vmax = -1, fpmax = -1, fvmax = -1;
    double range[2] = {-1, -1}, timeconst[2] = {-1, -1};
    ReadAttr(elem, "timeconst", 2, timeconst, text);
    ReadAttr(elem, "tausmooth", 1, &tausmooth, text);
    ReadAttr(elem, "range", 2, range, text);
    ReadAttr(elem, "force", 1, &force, text);
    ReadAttr(elem, "scale", 1, &scale, text);
    ReadAttr(elem, "lmin", 1, &lmin, text);
    ReadAttr(elem, "lmax", 1, &lmax, text);
    ReadAttr(elem, "vmax", 1, &vmax, text);
    ReadAttr(elem, "fpmax", 1, &fpmax, text);
    ReadAttr(elem, "fvmax", 1, &fvmax, text);
    err = sim_spec_setToMuscle(actuator, timeconst, tausmooth, range, force, scale,
                          lmin, lmax, vmax, fpmax, fvmax);
  }

  // adhesion
  else if (type == "adhesion") {
    double gain = actuator->gainprm[0];
    ReadAttr(elem, "gain", 1, &gain, text);
    ReadAttr(elem, "ctrlrange", 2, actuator->ctrlrange, text);
    err = sim_spec_setToAdhesion(actuator, gain);
  }

  else if (type == "plugin") {
    OnePlugin(elem, &actuator->plugin);
    int n;
    if (MapValue(elem, "dyntype", &n, dyn_map, dyn_sz)) {
      actuator->dyntype = (SIM_tDyn)n;
    }
    if (MapValue(elem, "actearly", &n, bool_map, 2)) {
      actuator->actearly = (n == 1);
    }
    ReadAttr(elem, "dynprm", SIM_NDYN, actuator->dynprm, text, false, false);
    ReadAttrInt(elem, "actdim", &actuator->actdim);
  }

  else {          // SHOULD NOT OCCUR
    throw sim_xml_error_t(elem, "unrecognized actuator type: %s", type.c_str());
  }

  // throw error if any of the above failed
  if (!err.empty()) {
    throw sim_xml_error_t(elem, err.c_str());
  }

  // read userdata
  std::vector<double> userdata;
  if (ReadVector(elem, "user", userdata, text)) {
    sim_spec_setDouble(actuator->userdata, userdata.data(), userdata.size());
  }

  // write info
  sim_spec_set_string(actuator->info, ("line " + std::to_string(elem->GetLineNum())).c_str());
}



// make composite
void sim_xml_reader_t::OneComposite(XMLElement* elem, sim_spec_body_t* body, SIM_sFrame* frame, const sim_spec_default_t* def) {
  string text;
  int n;

  // create out-of-DOM element
  SIM_CComposite comp;

  // common properties
  ReadAttrTxt(elem, "prefix", comp.prefix);
  if (MapValue(elem, "type", &n, comp_map, SIM_NCOMPTYPES, true)) {
    comp.type = (SIM_tCompType)n;
  }
  ReadAttr(elem, "count", 3, comp.count, text, false, false);
  ReadAttr(elem, "offset", 3, comp.offset, text);
  ReadAttr(elem, "quat", 4, comp.quat, text);
  comp.frame = frame;

  // plugin
  XMLElement* eplugin = FirstChildElement(elem, "plugin");
  if (eplugin) {
    OnePlugin(eplugin, &comp.plugin);
  }

  // cable
  string curves;
  ReadAttrTxt(elem, "curve", curves);
  ReadAttrTxt(elem, "initial", comp.initial);
  ReadAttr(elem, "size", 3, comp.size, text, false, false);
  auto uservert = ReadAttrVec<float>(elem, "vertex");
  if (uservert.has_value()) {
    comp.uservert = std::move(uservert.value());
  }

  // process curve string
  std::istringstream iss(curves);
  int i = 0;
  while (iss) {
    if (curves.empty()) {
      break;
    }
    iss >> text;
    if (i > 2) {
      throw sim_xml_error_t(elem, "The curve array must have a maximum of 3 components");
    }
    comp.curve[i++] = (SIM_tCompShape)FindKey(shape_map, SIM_NCOMPSHAPES, text);
    if (comp.curve[i-1] == -1) {
      throw sim_xml_error_t(elem, "The curve array contains an invalid shape");
    }
    if (iss.eof()){
      break;
    }
  };

  // skin
  XMLElement* eskin = FirstChildElement(elem, "skin");
  if (eskin) {
    string skinmaterial;
    comp.skin = true;
    if (MapValue(eskin, "texcoord", &n, bool_map, 2)) {
      comp.skintexcoord = (n == 1);
    }
    if (ReadAttrTxt(eskin, "material", skinmaterial)) {
      sim_warning("Ignoring composite skin material attribute at line %d.", eskin->GetLineNum());
    }
    ReadAttr(eskin, "rgba", 4, comp.skinrgba, text);
    ReadAttr(eskin, "inflate", 1, &comp.skininflate, text);
    ReadAttrInt(eskin, "subgrid", &comp.skinsubgrid);
    ReadAttrInt(eskin, "group", &comp.skingroup, 0);
    if (comp.skingroup < 0 || comp.skingroup >= SIM_NGROUP) {
      throw sim_xml_error_t(eskin, "skin group must be between 0 and 5");
    }
  }

  // set type-specific defaults
  comp.SetDefault();

  // geom
  XMLElement* egeom = FirstChildElement(elem, "geom");
  if (egeom) {
    string material;
    sim_spec_geom_t& dgeom = *comp.def[0].spec.geom;
    if (MapValue(egeom, "type", &n, geom_map, SIM_NGEOMTYPES)) {
      dgeom.type = (SIM_tGeom)n;
    }
    ReadAttr(egeom, "size", 3, dgeom.size, text, false, false);
    ReadAttrInt(egeom, "contype", &dgeom.contype);
    ReadAttrInt(egeom, "conaffinity", &dgeom.conaffinity);
    ReadAttrInt(egeom, "condim", &dgeom.condim);
    ReadAttrInt(egeom, "group", &dgeom.group);
    ReadAttrInt(egeom, "priority", &dgeom.priority);
    ReadAttr(egeom, "friction", 3, dgeom.friction, text, false, false);
    ReadAttr(egeom, "solmix", 1, &dgeom.solmix, text);
    ReadAttr(egeom, "solref", SIM_NREF, dgeom.solref, text, false, false);
    ReadAttr(egeom, "solimp", SIM_NIMP, dgeom.solimp, text, false, false);
    ReadAttr(egeom, "margin", 1, &dgeom.margin, text);
    ReadAttr(egeom, "gap", 1, &dgeom.gap, text);
    if (ReadAttrTxt(egeom, "material", material)) {
      sim_warning("Ignoring composite geom material attribute at line %d.", egeom->GetLineNum());
    }
    ReadAttr(egeom, "rgba", 4, dgeom.rgba, text);
    ReadAttr(egeom, "mass", 1, &dgeom.mass, text);
    ReadAttr(egeom, "density", 1, &dgeom.density, text);
  }

  // site
  XMLElement* esite = FirstChildElement(elem, "site");
  if (esite) {
    string material;
    SIM_sSite& dsite = *comp.def[0].spec.site;
    ReadAttr(esite, "size", 3, dsite.size, text, false, false);
    ReadAttrInt(esite, "group", &dsite.group);
    if (ReadAttrTxt(esite, "material", material)) {
      sim_warning("Ignoring composite site material attribute at line %d.", esite->GetLineNum());
    }
    ReadAttr(esite, "rgba", 4, dsite.rgba, text);
  }

  // joint
  XMLElement* ejnt = FirstChildElement(elem, "joint");
  while (ejnt) {
    // kind
    int kind;
    MapValue(ejnt, "kind", &kind, jkind_map, 1, true);

    // create a new element if this kind already exists
    if (comp.add[kind]) {
      char error[200];
      if (!comp.AddDefaultJoint(error, 200)) {
        throw sim_xml_error_t(elem, "%s", error);
      }
    }
    comp.add[kind] = true;

    // get element
    sim_spec_default_t* dspec = &comp.defjoint[(SIM_tCompKind)kind].back().spec;
    SIM_sJoint& djoint = *dspec->joint;
    SIM_sEquality& dequality = *dspec->equality;

    // particle joint
    if (MapValue(ejnt, "type", &n, joint_map, joint_sz)) {
      djoint.type = (SIM_tJoint)n;
    }
    ReadAttr(ejnt, "axis", 3, djoint.axis, text);

    // solreffix, solimpfix
    ReadAttr(ejnt, "solreffix", SIM_NREF, dequality.solref, text, false, false);
    ReadAttr(ejnt, "solimpfix", SIM_NIMP, dequality.solimp, text, false, false);

    // joint attributes
    MapValue(elem, "limited", &djoint.limited, TFAuto_map, 3);
    ReadAttrInt(ejnt, "group", &djoint.group);
    ReadAttr(ejnt, "solreflimit", SIM_NREF, djoint.solref_limit, text, false, false);
    ReadAttr(ejnt, "solimplimit", SIM_NIMP, djoint.solimp_limit, text, false, false);
    ReadAttr(ejnt,
             "solreffriction", SIM_NREF, djoint.solref_friction, text, false, false);
    ReadAttr(ejnt,
             "solimpfriction", SIM_NIMP, djoint.solimp_friction, text, false, false);
    ReadAttr(ejnt, "stiffness", 1, &djoint.stiffness, text);
    ReadAttr(ejnt, "range", 2, djoint.range, text);
    ReadAttr(ejnt, "margin", 1, &djoint.margin, text);
    ReadAttr(ejnt, "armature", 1, &djoint.armature, text);
    ReadAttr(ejnt, "damping", 1, &djoint.damping, text);
    ReadAttr(ejnt, "frictionloss", 1, &djoint.frictionloss, text);

    // advance
    ejnt = NextSiblingElement(ejnt, "joint");
  }


  // make composite
  char error[200];
  bool res = comp.Make(spec, body, error, 200);

  // throw error
  if (!res) {
    throw sim_xml_error_t(elem, "%s", error);
  }
}



// make flexcomp
void sim_xml_reader_t::OneFlexcomp(XMLElement* elem, sim_spec_body_t* body, const SIM_VFS* vfs) {
  string text;
  int n;

  // create out-of-DOM element
  SIM_CFlexcomp fcomp;
  SIM_sFlex& dflex = *fcomp.def.spec.flex;

  // common properties
  ReadAttrTxt(elem, "name", fcomp.name, true);
  if (MapValue(elem, "type", &n, fcomp_map, SIM_NFCOMPTYPES)) {
    fcomp.type = (SIM_tFcompType)n;
  }
  ReadAttr(elem, "count", 3, fcomp.count, text);
  ReadAttr(elem, "spacing", 3, fcomp.spacing, text);
  ReadAttr(elem, "scale", 3, fcomp.scale, text);
  ReadAttr(elem, "mass", 1, &fcomp.mass, text);
  ReadAttr(elem, "inertiabox", 1, &fcomp.inertiabox, text);
  auto maybe_file = ReadAttrFile(elem, "file", vfs, modelfiledir_);
  if (maybe_file.has_value()) {
    fcomp.file = std::move(maybe_file.value().Str());
  } else {
    fcomp.file = "";
  }
  string material;
  if (ReadAttrTxt(elem, "material", material)) {
    sim_warning("Ignoring flexcomp material attribute at line %d.", elem->GetLineNum());
  }
  ReadAttr(elem, "rgba", 4, dflex.rgba, text);
  if (MapValue(elem, "flatskin", &n, bool_map, 2)) {
    dflex.flatskin = (n == 1);
  }
  ReadAttrInt(elem, "dim", &dflex.dim);
  ReadAttr(elem, "radius", 1, &dflex.radius, text);
  ReadAttrInt(elem, "group", &dflex.group);
  if (!ReadAttr(elem, "origin", 3, fcomp.origin, text) &&
      fcomp.type == SIM_FCOMPTYPE_MESH && dflex.dim == 3) {
    throw sim_xml_error_t(elem, "origin must be specified for mesh flexcomps if dim=3");
  }

  // pose
  ReadAttr(elem, "pos", 3, fcomp.pos, text);
  ReadAttr(elem, "quat", 4, fcomp.quat, text);
  ReadAlternative(elem, fcomp.alt);

  // user or internal
  if (MapValue(elem, "rigid", &n, bool_map, 2)) {
    fcomp.rigid = (n == 1);
  }
  auto point = ReadAttrVec<double>(elem, "point");
  if (point.has_value()) {
    fcomp.point = std::move(point.value());
  }
  auto element = ReadAttrVec<int>(elem, "element");
  if (element.has_value()) {
    fcomp.element = std::move(element.value());
  }
  auto texcoord = ReadAttrVec<float>(elem, "texcoord");
  if (texcoord.has_value()) {
    fcomp.texcoord = std::move(texcoord.value());
  }

  // dof type
  if (MapValue(elem, "dof", &n, fdof_map, SIM_NFCOMPDOFS)) {
    fcomp.doftype = (SIM_tDof)n;
  }

  // edge
  XMLElement* edge = FirstChildElement(elem, "edge");
  if (edge) {
    MapValue(edge, "equality", &fcomp.equality, flexeq_map, 3);
    ReadAttr(edge, "solref", SIM_NREF, fcomp.def.spec.equality->solref, text, false, false);
    ReadAttr(edge, "solimp", SIM_NIMP, fcomp.def.spec.equality->solimp, text, false, false);
    ReadAttr(edge, "stiffness", 1, &dflex.edgestiffness, text);
    ReadAttr(edge, "damping", 1, &dflex.edgedamping, text);
  }

  // elasticity
  XMLElement* elasticity = FirstChildElement(elem, "elasticity");
  if (elasticity) {
    ReadAttr(elasticity, "young", 1, &dflex.young, text);
    ReadAttr(elasticity, "poisson", 1, &dflex.poisson, text);
    ReadAttr(elasticity, "damping", 1, &dflex.damping, text);
    ReadAttr(elasticity, "thickness", 1, &dflex.thickness, text);
    MapValue(elasticity, "elastic2d", &dflex.elastic2d, elastic2d_map, 4);
  }

  // check errors
  if (dflex.elastic2d >= 2 && fcomp.equality) {
    throw sim_xml_error_t(elem, "elasticity and edge constraints cannot both be present");
  }

  // contact
  XMLElement* cont = FirstChildElement(elem, "contact");
  if (cont) {
    ReadAttrInt(cont, "contype", &dflex.contype);
    ReadAttrInt(cont, "conaffinity", &dflex.conaffinity);
    ReadAttrInt(cont, "condim", &dflex.condim);
    ReadAttrInt(cont, "priority", &dflex.priority);
    ReadAttr(cont, "friction", 3, dflex.friction, text, false, false);
    ReadAttr(cont, "solmix", 1, &dflex.solmix, text);
    ReadAttr(cont, "solref", SIM_NREF, dflex.solref, text, false, false);
    ReadAttr(cont, "solimp", SIM_NIMP, dflex.solimp, text, false, false);
    ReadAttr(cont, "margin", 1, &dflex.margin, text);
    ReadAttr(cont, "gap", 1, &dflex.gap, text);
    if (MapValue(cont, "internal", &n, bool_map, 2)) {
      dflex.internal = (n == 1);
    }
    MapValue(cont, "selfcollide", &dflex.selfcollide, flexself_map, 5);
    if (MapValue(cont, "vertcollide", &n, bool_map, 2)) {
      dflex.vertcollide = (n == 1);
    }
    if (MapValue(cont, "passive", &n, bool_map, 2)) {
      dflex.passive = (n == 1);
    }
    ReadAttrInt(cont, "activelayers", &dflex.activelayers);
  }

  // pin
  XMLElement* epin = FirstChildElement(elem, "pin");
  while (epin) {
    auto id = ReadAttrVec<int>(epin, "id");
    if (id.has_value()) {
      fcomp.pinid.insert(fcomp.pinid.end(), id->begin(), id->end());
    }
    auto range = ReadAttrVec<int>(epin, "range");
    if (range.has_value()) {
      fcomp.pinrange.insert(fcomp.pinrange.end(), range->begin(), range->end());
    }
    auto grid = ReadAttrVec<int>(epin, "grid");
    if (grid.has_value()) {
      fcomp.pingrid.insert(fcomp.pingrid.end(), grid->begin(), grid->end());
    }
    auto gridrange = ReadAttrVec<int>(epin, "gridrange");
    if (gridrange.has_value()) {
      fcomp.pingridrange.insert(fcomp.pingridrange.end(),
                                gridrange->begin(), gridrange->end());
    }

    // advance
    epin = NextSiblingElement(epin, "pin");
  }

  // plugin
  XMLElement* eplugin = FirstChildElement(elem, "plugin");
  if (eplugin) {
    OnePlugin(eplugin, &fcomp.plugin);
  }

  // make flexcomp
  char error[200];
  bool res = fcomp.Make(body, error, 200);

  // throw error
  if (!res) {
    throw sim_xml_error_t(elem, "%s", error);
  }
}



// add plugin
void sim_xml_reader_t::OnePlugin(XMLElement* elem, SIM_sPlugin* plugin) {
  plugin->active = true;
  string name = "";
  string instance_name = "";
  ReadAttrTxt(elem, "plugin", name);
  ReadAttrTxt(elem, "instance", instance_name);
  sim_spec_set_string(plugin->plugin_name, name.c_str());
  sim_spec_set_string(plugin->name, instance_name.c_str());
  if (instance_name.empty()) {
    plugin->element = sim_spec_addPlugin(spec)->element;
    ReadPluginConfigs(elem, plugin);
  } else {
    spec->hasImplicitPluginElem = true;
  }
}



//------------------ SIMCF-specific sections --------------------------------------------------------

// default section parser
void sim_xml_reader_t::Default(XMLElement* section, const sim_spec_default_t* def, const SIM_VFS* vfs) {
  XMLElement* elem;
  string text, name;

  // create new default, except at top level (already added in sim_builder_model_t constructor)
  text.clear();
  ReadAttrTxt(section, "class", text);
  if (text.empty()) {
    if (def) {
      throw sim_xml_error_t(section, "empty class name");
    }
  }
  if (def) {
    def = sim_spec_addDefault(spec, text.c_str(), def);
    if (!def) {
      throw sim_xml_error_t(section, "repeated default class name");
    }
  } else {
    def = sim_spec_getSpecDefault(spec);
    if (!text.empty() && text != "main") {
      throw sim_xml_error_t(section, "top-level default class 'main' cannot be renamed");
    }
  }

  // iterate over elements other than nested defaults
  elem = FirstChildElement(section);
  while (elem) {
    // get element name
    name = elem->Value();

    // read mesh
    if (name == "mesh")OneMesh(elem, def->mesh, vfs);

    // read material
    else if (name == "material") {
      sim_warning("Ignoring deprecated default material element at line %d.", elem->GetLineNum());
    }

    // read joint
    else if (name == "joint")OneJoint(elem, def->joint);

    // read geom
    else if (name == "geom")OneGeom(elem, def->geom);

    // read site
    else if (name == "site")OneSite(elem, def->site);

    // read camera
    else if (name == "camera") {
      sim_warning("Ignoring deprecated default camera element at line %d.", elem->GetLineNum());
    }

    // read light
    else if (name == "light") {
      sim_warning("Ignoring deprecated default light element at line %d.", elem->GetLineNum());
    }

    // read pair
    else if (name == "pair")OnePair(elem, def->pair);

    // read equality
    else if (name == "equality")OneEquality(elem, def->equality);

    // read tendon
    else if (name == "tendon")OneTendon(elem, def->tendon);

    // read actuator
    else if (name == "general"     ||
             name == "motor"       ||
             name == "position"    ||
             name == "velocity"    ||
             name == "damper"      ||
             name == "intvelocity" ||
             name == "cylinder"    ||
             name == "muscle"      ||
             name == "adhesion") {
      OneActuator(elem, def->actuator);
    }

    // advance
    elem = NextSiblingElement(elem);
  }

  // iterate over nested defaults
  elem = FirstChildElement(section);
  while (elem) {
    // get element name
    name = elem->Value();

    // read default
    if (name == "default") {
      Default(elem, def, vfs);
    }

    // advance
    elem = NextSiblingElement(elem);
  }
}



// extension section parser
void sim_xml_reader_t::Extension(XMLElement* section) {
  XMLElement* elem = FirstChildElement(section);

  while (elem) {
    // get sub-element name
    string_view name = elem->Value();

    if (name == "plugin") {
      string plugin_name;
      ReadAttrTxt(elem, "plugin", plugin_name, /* required = */ true);
      if (sim_spec_activatePlugin(spec, plugin_name.c_str())) {
        throw sim_xml_error_t(elem, "plugin %s not found", plugin_name.c_str());
      }

      XMLElement* child = FirstChildElement(elem);
      while (child) {
        if (string(child->Value()) == "instance") {
          if (spec->hasImplicitPluginElem) {
            throw sim_xml_error_t(
                    child, "explicit plugin instance must appear before implicit plugin elements");
          }
          string name;
          SIM_sPlugin* p = sim_spec_addPlugin(spec);
          sim_spec_set_string(p->plugin_name, plugin_name.c_str());
          sim_spec_set_string(p->info, ("line " + std::to_string(elem->GetLineNum())).c_str());
          ReadAttrTxt(child, "name", name, /* required = */ true);
          sim_spec_set_string(p->name, name.c_str());
          if (!p->name) {
            throw sim_xml_error_t(child, "plugin instance must have a name");
          }
          ReadPluginConfigs(child, p);
        }
        child = NextSiblingElement(child);
      }
    }

    // advance to next element
    elem = NextSiblingElement(elem);
  }
}



// custom section parser
void sim_xml_reader_t::Custom(XMLElement* section) {
  string str, name;
  XMLElement* elem;
  double data[500];

  // iterate over child elements
  elem = FirstChildElement(section);
  while (elem) {
    // get sub-element name
    name = elem->Value();
    string elname;

    // numeric
    if (name == "numeric") {
      // create custom
      SIM_sNumeric* numeric = sim_spec_addNumeric(spec);

      // write error info
      sim_spec_set_string(numeric->info, ("line " + std::to_string(elem->GetLineNum())).c_str());

      // read attributes
      ReadAttrTxt(elem, "name", elname, true);
      if (sim_spec_set_name(numeric->element, elname.c_str())) {
        throw sim_xml_error_t(elem, "%s", sim_spec_getError(spec));
      }
      if (ReadAttrInt(elem, "size", &numeric->size)) {
        int sz = numeric->size < 500 ? numeric->size : 500;
        for (int i=0; i < sz; i++) {
          data[i] = 0;
        }
      } else {
        numeric->size = 501;
      }
      int len = ReadAttr(elem, "data", numeric->size, data, str, false, false);
      if (numeric->size == 501) {
        numeric->size = len;
      }
      if (numeric->size < 1 || numeric->size > 500) {
        throw sim_xml_error_t(elem, "custom field size must be between 1 and 500");
      }

      // copy data
      sim_spec_setDouble(numeric->data, data, numeric->size);
    }

    // text
    else if (name == "text") {
      // create custom
      SIM_sText* text = sim_spec_addText(spec);

      // write error info
      sim_spec_set_string(text->info, ("line " + std::to_string(elem->GetLineNum())).c_str());

      // read attributes
      ReadAttrTxt(elem, "name", elname, true);
      if (sim_spec_set_name(text->element, elname.c_str())) {
        throw sim_xml_error_t(elem, "%s", sim_spec_getError(spec));
      }
      ReadAttrTxt(elem, "data", str, true);
      if (str.empty()) {
        throw sim_xml_error_t(elem, "text field cannot be empty");
      }

      // copy data
      sim_spec_set_string(text->data, str.c_str());
    }

    // tuple
    else if (name == "tuple") {
      // create custom
      SIM_sTuple* tuple = sim_spec_addTuple(spec);

      // write error info
      sim_spec_set_string(tuple->info, ("line " + std::to_string(elem->GetLineNum())).c_str());

      // read attributes
      ReadAttrTxt(elem, "name", elname, true);
      if (sim_spec_set_name(tuple->element, elname.c_str())) {
        throw sim_xml_error_t(elem, "%s", sim_spec_getError(spec));
      }

      // read objects and add
      XMLElement* obj = FirstChildElement(elem);
      std::vector<int> objtype;
      string objname = "";
      std::vector<double> objprm;

      while (obj) {
        // get sub-element name
        name = obj->Value();

        // new object
        if (name == "element") {
          // read type, check and assign
          ReadAttrTxt(obj, "objtype", str, true);
          sim_obj_t otype = (sim_obj_t)sim_math_str2Type(str.c_str());
          if (otype == SIM_OBJ_UNKNOWN) {
            throw sim_xml_error_t(obj, "unknown object type");
          }
          objtype.push_back(otype);

          // read name and assign
          ReadAttrTxt(obj, "objname", str, true);
          objname += " " + str;

          // read parameter and assign
          double oprm = 0;
          ReadAttr(obj, "prm", 1, &oprm, str);
          objprm.push_back(oprm);
        }

        // advance to next object
        obj = NextSiblingElement(obj);
      }

      sim_spec_setInt(tuple->objtype, objtype.data(), objtype.size());
      sim_spec_setStringVec(tuple->objname, objname.c_str());
      sim_spec_setDouble(tuple->objprm, objprm.data(), objprm.size());
    }

    // advance to next element
    elem = NextSiblingElement(elem);
  }
}



// visual section parser
void sim_xml_reader_t::Visual(XMLElement* section) {
  sim_warning("Ignoring deprecated <visual> section at line %d in XML.", section->GetLineNum());
}



// asset section parser
void sim_xml_reader_t::Asset(XMLElement* section, const SIM_VFS* vfs) {
  string text, name;
  XMLElement* elem;

  // iterate over child elements
  elem = FirstChildElement(section);
  while (elem) {
    // get sub-element name
    name = elem->Value();

    // get class if specified, otherwise use default0
    const sim_spec_default_t* def = GetClass(elem);
    if (!def) {
      def = sim_spec_getSpecDefault(spec);
    }

    // texture sub-element
    if (name == "texture") {
      sim_warning("Ignoring deprecated asset texture element at line %d.", elem->GetLineNum());
    }

    // material sub-element
    else if (name == "material") {
      sim_warning("Ignoring deprecated asset material element at line %d.", elem->GetLineNum());
    }

    // mesh sub-element
    else if (name == "mesh") {
      // create mesh and parse
      SIM_sMesh* mesh = sim_spec_addMesh(spec, def);
      OneMesh(elem, mesh, vfs);
    }

    // skin sub-element... deprecate ???
    else if (name == "skin") {
      // create skin and parse
      SIM_sSkin* skin = sim_spec_addSkin(spec);
      OneSkin(elem, skin, vfs);
    }

    // hfield sub-element
    else if (name == "hfield") {
      // create hfield
      SIM_sHField* hfield = sim_spec_addHField(spec);

      // write error info
      sim_spec_set_string(hfield->info, ("line " + std::to_string(elem->GetLineNum())).c_str());

      // read attributes
      string name, content_type;
      if (ReadAttrTxt(elem, "name", name)) {
        if (sim_spec_set_name(hfield->element, name.c_str())) {
          throw sim_xml_error_t(elem, "%s", sim_spec_getError(spec));
        }
      }
      if (ReadAttrTxt(elem, "content_type", content_type)) {
        sim_spec_set_string(hfield->content_type, content_type.c_str());
      }
      auto file = ReadAttrFile(elem, "file", vfs, AssetDir());
      if (file.has_value()) {
        sim_spec_set_string(hfield->file, file->c_str());
      }
      ReadAttrInt(elem, "nrow", &hfield->nrow);
      ReadAttrInt(elem, "ncol", &hfield->ncol);
      ReadAttr(elem, "size", 4, hfield->size, text, true);

      // allocate buffer for dynamic hfield, copy user data if given
      if (!file.has_value() && hfield->nrow > 0 && hfield->ncol > 0) {
        int nrow = hfield->nrow;
        int ncol = hfield->ncol;

        // read user data
        auto userdata = ReadAttrVec<float>(elem, "elevation");

        // user data given, copy into data
        if (userdata.has_value()) {
          if (userdata->size() != nrow*ncol) {
            throw sim_xml_error_t(elem, "elevation data length must match nrow*ncol");
          }

          // copy in reverse row order, so XML string is top-to-bottom
          std::vector<float> flipped(nrow*ncol);
          for (int i = 0; i < nrow; i++) {
            int flip = nrow-1-i;
            for (int j = 0; j < ncol; j++) {
              flipped[flip*ncol + j] = userdata->data()[i*ncol + j];
            }
          }

          sim_spec_setFloat(hfield->userdata, flipped.data(), flipped.size());
        }

        // user data not given, set to 0
        else {
          std::vector<float> zero(nrow*ncol);
          sim_spec_setFloat(hfield->userdata, zero.data(), zero.size());
        }
      }
    }

    // model sub-element
    else if (name == "model") {
      std::string content_type;
      ReadAttrTxt(elem, "content_type", content_type);

      // parse the child
      std::array<char, 1024> error;
      auto filename = modelfiledir_ + ReadAttrFile(elem, "file", vfs).value();

      sim_spec_t* child = sim_parse(filename.c_str(), content_type.c_str(), vfs,
                               error.data(), error.size());
      if (!child) {
        throw sim_xml_error_t(elem, "could not parse model file with error: %s", error.data());
      }

      // overwrite model name if given
      string modelname = "";
      if (ReadAttrTxt(elem, "name", modelname)) {
        sim_spec_set_string(child->modelname, modelname.c_str());
      }

      // store child spec in model
      sim_spec_addSpec(spec, child);
    }

    // advance to next element
    elem = NextSiblingElement(elem);
  }
}



// body/world section parser; recursive
void sim_xml_reader_t::Body(XMLElement* section, sim_spec_body_t* body, SIM_sFrame* frame,
                     const SIM_VFS* vfs) {
  string text, name;
  XMLElement* elem;
  int n;

  // sanity check
  if (!body) {
    throw sim_xml_error_t(section, "null body pointer");
  }

  // no attributes allowed in world body
  if (sim_spec_getId(body->element) == 0 && section->FirstAttribute() && !frame) {
    throw sim_xml_error_t(section, "World body cannot have attributes");
  }

  // iterate over sub-elements; attributes set while parsing parent body
  elem = FirstChildElement(section);
  while (elem) {
    // get sub-element name
    name = elem->Value();

    // get class if specified, otherwise use body
    const sim_spec_default_t* def = GetClass(elem);
    if (!def) {
      def = sim_spec_getDefault(frame ? frame->element : body->element);
    }

    // inertial sub-element
    if (name == "inertial") {
      // no inertia allowed in world body
      if (sim_spec_getId(body->element) == 0) {
        throw sim_xml_error_t(elem, "World body cannot have inertia");
      }
      body->explicitinertial = true;
      ReadAttr(elem, "pos", 3, body->ipos, text, true);
      ReadQuat(elem, "quat", body->iquat, text);
      ReadAttr(elem, "mass", 1, &body->mass, text, true);
      ReadAttr(elem, "diaginertia", 3, body->inertia, text);
      bool alt = ReadAlternative(elem, body->ialt);
      bool full = ReadAttr(elem, "fullinertia", 6, body->fullinertia, text);
      if (alt && full) {
        throw sim_xml_error_t(elem, "fullinertia and inertial orientation cannot both be specified");
      }
    }

    // joint sub-element
    else if (name == "joint") {
      // no joints allowed in world body
      if (sim_spec_getId(body->element) == 0) {
        throw sim_xml_error_t(elem, "World body cannot have joints");
      }

      // create joint and parse
      SIM_sJoint* joint = sim_spec_addJoint(body, def);
      OneJoint(elem, joint);
      sim_spec_setFrame(joint->element, frame);
    }

    // freejoint sub-element
    else if (name == "freejoint") {
      // no joints allowed in world body
      if (sim_spec_getId(body->element) == 0) {
        throw sim_xml_error_t(elem, "World body cannot have joints");
      }

      // create free joint without defaults
      SIM_sJoint* joint = sim_spec_addFreeJoint(body);
      sim_spec_setFrame(joint->element, frame);

      // save defaults after creation, to make sure writing is ok
      sim_spec_setDefault(joint->element, def);

      // read attributes
      string name;
      if (ReadAttrTxt(elem, "name", name)) {
        if (sim_spec_set_name(joint->element, name.c_str())) {
          throw sim_xml_error_t(elem, "%s", sim_spec_getError(spec));
        }
      }
      ReadAttrInt(elem, "group", &joint->group);
      MapValue(elem, "align", &joint->align, TFAuto_map, 3);
    }

    // geom sub-element
    else if (name == "geom") {
      // create geom and parse
      sim_spec_geom_t* geom = sim_spec_addGeom(body, def);
      OneGeom(elem, geom);
      sim_spec_setFrame(geom->element, frame);
    }

    // site sub-element
    else if (name == "site") {
      // create site and parse
      SIM_sSite* site = sim_spec_addSite(body,  def);
      OneSite(elem, site);
      sim_spec_setFrame(site->element, frame);
    }

    // camera sub-element
    else if (name == "camera") {
      sim_warning("Ignoring deprecated body camera element at line %d.", elem->GetLineNum());
    }

    // light sub-element
    else if (name == "light") {
      sim_warning("Ignoring deprecated body light element at line %d.", elem->GetLineNum());
    }

    // plugin sub-element
    else if (name == "plugin") {
      OnePlugin(elem, &(body->plugin));
    }

    // composite sub-element
    else if (name == "composite") {
      // parse composite
      OneComposite(elem, body, frame, def);
    }

    // flexcomp sub-element
    else if (name == "flexcomp") {
      // parse flexcomp
      OneFlexcomp(elem, body, vfs);
    }

    // frame sub-element
    else if (name == "frame") {
      // read childdef
      bool has_childclass = ReadAttrTxt(elem, "childclass", text);
      const sim_spec_default_t* childdef = has_childclass ? sim_spec_findDefault(spec, text.c_str()) : nullptr;
      if (has_childclass && !childdef) {
        throw sim_xml_error_t(elem, "unknown default childclass");
      }

      // create frame
      SIM_sFrame* pframe = sim_spec_addFrame(body, frame);
      sim_spec_set_string(pframe->info, ("line " + std::to_string(elem->GetLineNum())).c_str());
      sim_spec_setDefault(pframe->element, childdef ? childdef : def);

      // read attributes
      string name, childclass;
      if (ReadAttrTxt(elem, "name", name)) {
        if (sim_spec_set_name(pframe->element, name.c_str())) {
          throw sim_xml_error_t(elem, "%s", sim_spec_getError(spec));
        }
      }
      if (ReadAttrTxt(elem, "childclass", childclass)) {
        sim_spec_set_string(pframe->childclass, childclass.c_str());
      }
      ReadAttr(elem, "pos", 3, pframe->pos, text);
      ReadQuat(elem, "quat", pframe->quat, text);
      ReadAlternative(elem, pframe->alt);

      Body(elem, body, pframe, vfs);
    }

    // replicate sub-element
    else if (name == "replicate") {
      int count;
      double offset[3] = {0, 0, 0};
      double euler[3] = {0, 0, 0};
      string separator = "";
      ReadAttr(elem, "count", 1, &count, text, true);
      ReadAttr(elem, "offset", 3, offset, text);
      ReadAttr(elem, "euler", 3, euler, text);
      ReadAttrTxt(elem, "sep", separator);

      // store rotation difference
      SIM_sOrientation alt;
      sim_spec_defaultOrientation(&alt);
      alt.type = SIM_ORIENTATION_EULER;
      sim_math_internal_copy_vec(alt.euler, euler, 3);
      double rotation[4] = {1, 0, 0, 0};
      sim_spec_resolveOrientation(rotation, spec->compiler.degree, spec->compiler.eulerseq, &alt);

      // read childdef
      bool has_childclass = ReadAttrTxt(elem, "childclass", text);
      const sim_spec_default_t* childdef = has_childclass ? sim_spec_findDefault(spec, text.c_str()) : nullptr;
      if (has_childclass && !childdef) {
        throw sim_xml_error_t(elem, "unknown default childclass");
      }

      // create subtree
      sim_spec_body_t* subtree = sim_spec_addBody(body, childdef);
      double pos[3] = {0, 0, 0};
      double quat[4] = {1, 0, 0, 0};

      // parent frame that will be used to attach the subtree
      SIM_sFrame* pframe = sim_spec_addFrame(subtree, frame);
      sim_spec_setDefault(pframe->element, childdef ? childdef : def);
      sim_spec_set_string(pframe->info, ("line = " + std::to_string(elem->GetLineNum())).c_str());

      // parse subtree
      Body(elem, subtree, pframe, vfs);

      // update pframe and attach
      for (int i = 0; i < count; i++) {
        // overwrite orientation to increase precision
        alt.euler[0] = i*euler[0];
        alt.euler[1] = i*euler[1];
        alt.euler[2] = i*euler[2];
        sim_spec_resolveOrientation(quat, spec->compiler.degree, spec->compiler.eulerseq, &alt);

        // set position and orientation
        sim_math_internal_set_vec(pframe->pos, pos[0], pos[1], pos[2]);
        sim_math_internal_set_vec(pframe->quat, quat[0], quat[1], quat[2], quat[3]);

        // accumulate rotation
        sim_math_internal_frameaccum(pos, quat, offset, rotation);

        // process suffix
        string suffix = separator;
        UpdateString(suffix, count, i);

        // attach to parent
        if (!sim_spec_attach(body->element, pframe->element, /*prefix=*/"", suffix.c_str())) {
          throw sim_xml_error_t(elem, "%s", sim_spec_getError(spec));
        }
      }

      // delete subtree
      if (sim_spec_delete(spec, subtree->element)) {
        throw sim_xml_error_t(elem, "%s", sim_spec_getError(spec));
      }
    }

    // body sub-element
    else if (name == "body") {
      // read childdef
      bool has_childclass = ReadAttrTxt(elem, "childclass", text);
      const sim_spec_default_t* childdef = has_childclass ? sim_spec_findDefault(spec, text.c_str()) : nullptr;
      if (has_childclass && !childdef) {
        throw sim_xml_error_t(elem, "unknown default childclass");
      }

      // create child body
      sim_spec_body_t* child = sim_spec_addBody(body, childdef);
      sim_spec_set_string(child->info, string("line " + std::to_string(elem->GetLineNum())).c_str());

      // set default from class or childclass
      sim_spec_setDefault(child->element, childdef ? childdef : def);

      // read attributes
      string name, childclass;
      if (ReadAttrTxt(elem, "name", name)) {
        if (sim_spec_set_name(child->element, name.c_str())) {
          throw sim_xml_error_t(elem, "%s", sim_spec_getError(spec));
        }
      }
      if (ReadAttrTxt(elem, "childclass", childclass)) {
        sim_spec_set_string(child->childclass, childclass.c_str());
      }
      ReadAttr(elem, "pos", 3, child->pos, text);
      ReadQuat(elem, "quat", child->quat, text);
      if (MapValue(elem, "mocap", &n, bool_map, 2)) {
        child->mocap = (n == 1);
      }
      ReadAlternative(elem, child->alt);

      // gravcomp, sleep policy
      ReadAttr(elem, "gravcomp", 1, &child->gravcomp, text);
      if (MapValue(elem, "sleep", &n, bodysleep_map, bodysleep_sz)) {
        child->sleep = (SIM_tSleepPolicy) n;
      }

      // read userdata
      std::vector<double> userdata;
      ReadVector(elem, "user", userdata, text);
      sim_spec_setDouble(child->userdata, userdata.data(), userdata.size());

      // add frame
      sim_spec_setFrame(child->element, frame);

      // make recursive call
      Body(elem, child, nullptr, vfs);
    }

    // attachment
    else if (name == "attach") {
      string model_name, body_name, prefix;
      ReadAttrTxt(elem, "model", model_name, /*required=*/true);
      ReadAttrTxt(elem, "body", body_name, /*required=*/false);
      ReadAttrTxt(elem, "prefix", prefix, /*required=*/true);

      sim_spec_body_t* child_body = sim_spec_findBody(spec, (prefix+body_name).c_str());
      SIM_sFrame* pframe = frame ? frame : sim_spec_addFrame(body, nullptr);

      if (!child_body) {
        sim_spec_t* asset = sim_spec_findSpec(spec, model_name.c_str());
        if (!asset) {
          throw sim_xml_error_t(elem, "could not find model '%s'", model_name.c_str());
        }
        sim_spec_element_t* child;
        if (body_name.empty()) {
          child = asset->element;
        } else {
          child_body = sim_spec_findBody(asset, body_name.c_str());
          if (!child_body) {
            throw sim_xml_error_t(elem, "could not find body '%s''%s'", body_name.c_str());
          }
          child = child_body->element;
        }
        if (!sim_spec_attach(pframe->element, child, prefix.c_str(), "")) {
          throw sim_xml_error_t(elem, "%s", sim_spec_getError(spec));
        }
      } else {
        // only set frame to existing body
        if (sim_spec_setFrame(child_body->element, pframe)) {
          throw sim_xml_error_t(elem, "%s", sim_spec_getError(spec));
        }
      }
    }

    // no match
    else {
      throw sim_xml_error_t(elem, "unrecognized model element '%s'", name.c_str());
    }

    // advance to next element
    elem = NextSiblingElement(elem);
  }
}



// contact section parser
void sim_xml_reader_t::Contact(XMLElement* section) {
  string text, name;
  XMLElement* elem;

  // iterate over child elements
  elem = FirstChildElement(section);
  while (elem) {
    // get sub-element name
    name = elem->Value();

    // get class if specified, otherwise use default0
    const sim_spec_default_t* def = GetClass(elem);
    if (!def) {
      def = sim_spec_getSpecDefault(spec);
    }

    // geom pair to include
    if (name == "pair") {
      // create pair and parse
      SIM_sPair* pair = sim_spec_addPair(spec, def);
      OnePair(elem, pair);
    }

    // body pair to exclude
    else if (name == "exclude") {
      SIM_sExclude* exclude = sim_spec_addExclude(spec);
      string exname, exbody1, exbody2;

      // write error info
      sim_spec_set_string(exclude->info, ("line " + std::to_string(elem->GetLineNum())).c_str());

      // read name and body names
      if (ReadAttrTxt(elem, "name", exname)) {
        if (sim_spec_set_name(exclude->element, exname.c_str())) {
          throw sim_xml_error_t(elem, "%s", sim_spec_getError(spec));
        }
      }
      ReadAttrTxt(elem, "body1", exbody1, true);
      sim_spec_set_string(exclude->bodyname1, exbody1.c_str());
      ReadAttrTxt(elem, "body2", exbody2, true);
      sim_spec_set_string(exclude->bodyname2, exbody2.c_str());
    }

    // advance to next element
    elem = NextSiblingElement(elem);
  }
}



// constraint section parser
void sim_xml_reader_t::Equality(XMLElement* section) {
  XMLElement* elem;

  // iterate over child elements
  elem = FirstChildElement(section);
  while (elem) {
    // get class if specified, otherwise use default0
    const sim_spec_default_t* def = GetClass(elem);
    if (!def) {
      def = sim_spec_getSpecDefault(spec);
    }

    // create equality constraint and parse
    SIM_sEquality* equality = sim_spec_addEquality(spec, def);
    OneEquality(elem, equality);

    // advance to next element
    elem = NextSiblingElement(elem);
  }
}



// deformable section parser
void sim_xml_reader_t::Deformable(XMLElement* section, const SIM_VFS* vfs) {
  string name;
  XMLElement* elem;

  // iterate over child elements
  elem = FirstChildElement(section);
  while (elem) {
    // get sub-element name
    name = elem->Value();

    // get class if specified, otherwise use default0
    const sim_spec_default_t* def = GetClass(elem);
    if (!def) {
      def = sim_spec_getSpecDefault(spec);
    }

    // flex sub-element
    if (name == "flex") {
      // create flex and parse
      SIM_sFlex* flex = sim_spec_addFlex(spec);
      OneFlex(elem, flex);
    }

    // skin sub-element
    else if (name == "skin") {
      // create skin and parse
      SIM_sSkin* skin = sim_spec_addSkin(spec);
      OneSkin(elem, skin, vfs);
    }

    // advance to next element
    elem = NextSiblingElement(elem);
  }
}



// tendon section parser
void sim_xml_reader_t::Tendon(XMLElement* section) {
  string text, text1;
  XMLElement* elem;
  double data;

  // iterate over child elements
  elem = FirstChildElement(section);
  while (elem) {
    // get class if specified, otherwise use default0
    const sim_spec_default_t* def = GetClass(elem);
    if (!def) {
      def = sim_spec_getSpecDefault(spec);
    }

    // create tendon and parse
    SIM_sTendon* tendon = sim_spec_addTendon(spec, def);
    OneTendon(elem, tendon);

    // process wrap sub-elements
    XMLElement* sub = FirstChildElement(elem);
    while (sub) {
      // get wrap type
      string type = sub->Value();
      SIM_sWrap* wrap;;

      // read attributes depending on type
      if (type == "site") {
        ReadAttrTxt(sub, "site", text, true);
        wrap = sim_spec_wrapSite(tendon, text.c_str());
      }

      else if (type == "geom") {
        ReadAttrTxt(sub, "geom", text, true);
        if (!ReadAttrTxt(sub, "sidesite", text1)) {
          text1.clear();
        }
        wrap = sim_spec_wrapGeom(tendon, text.c_str(), text1.c_str());
      }

      else if (type == "pulley") {
        ReadAttr(sub, "divisor", 1, &data, text, true);
        wrap = sim_spec_wrapPulley(tendon, data);
      }

      else if (type == "joint") {
        ReadAttrTxt(sub, "joint", text, true);
        ReadAttr(sub, "coef", 1, &data, text1, true);
        wrap = sim_spec_wrapJoint(tendon, text.c_str(), data);
      }

      else {
        throw sim_xml_error_t(sub, "unknown wrap type");  // SHOULD NOT OCCUR
      }

      sim_spec_set_string(wrap->info, ("line " + std::to_string(sub->GetLineNum())).c_str());

      // advance to next sub-element
      sub = NextSiblingElement(sub);
    }

    // advance to next element
    elem = NextSiblingElement(elem);
  }
}



// actuator section parser
void sim_xml_reader_t::Actuator(XMLElement* section) {
  XMLElement* elem;

  // iterate over child elements
  elem = FirstChildElement(section);
  while (elem) {
    // get class if specified, otherwise use default0
    const sim_spec_default_t* def = GetClass(elem);
    if (!def) {
      def = sim_spec_getSpecDefault(spec);
    }

    // create actuator and parse
    sim_spec_actuator_t* actuator = sim_spec_addActuator(spec, def);
    OneActuator(elem, actuator);

    // advance to next element
    elem = NextSiblingElement(elem);
  }
}



// sensor section parser
void sim_xml_reader_t::Sensor(XMLElement* section) {
  int n;
  XMLElement* elem = FirstChildElement(section);
  while (elem) {
    string type = elem->Value();
    if (type == "camprojection") {
      sim_warning("Ignoring deprecated camprojection sensor at line %d.", elem->GetLineNum());
      elem = NextSiblingElement(elem);
      continue;
    }
    if (type == "rangefinder") {
      string camera_name;
      if (ReadAttrTxt(elem, "camera", camera_name, false)) {
        sim_warning("Ignoring deprecated camera-based rangefinder sensor at line %d.",
                    elem->GetLineNum());
        elem = NextSiblingElement(elem);
        continue;
      }
    }

    // create sensor after filtering deprecated visualization-bound sensor semantics
    SIM_sSensor* sensor = sim_spec_addSensor(spec);
    string text, name, objname, refname;
    std::vector<double> userdata;

    // read name, noise, userdata
    if (ReadAttrTxt(elem, "name", name)) {
      if (sim_spec_set_name(sensor->element, name.c_str())) {
        throw sim_xml_error_t(elem, "%s", sim_spec_getError(spec));
      }
    }
    ReadAttr(elem, "cutoff", 1, &sensor->cutoff, text);
    ReadAttr(elem, "noise", 1, &sensor->noise, text);
    ReadAttrInt(elem, "nsample", &sensor->nsample);
    MapValue(elem, "interp", &sensor->interp, interp_map, interp_sz);
    ReadAttr(elem, "delay", 1, &sensor->delay, text);
    ReadAttr(elem, "interval", 2, sensor->interval, text, /*required=*/false, /*exact=*/false);
    if (ReadVector(elem, "user", userdata, text)) {
      sim_spec_setDouble(sensor->userdata, userdata.data(), userdata.size());
    }

    // common robotic sensors, attached to a site
    if (type == "touch") {
      sensor->type = SIM_SENS_TOUCH;
      sensor->objtype = SIM_OBJ_SITE;
      ReadAttrTxt(elem, "site", objname, true);
    } else if (type == "accelerometer") {
      sensor->type = SIM_SENS_ACCELEROMETER;
      sensor->objtype = SIM_OBJ_SITE;
      ReadAttrTxt(elem, "site", objname, true);
    } else if (type == "velocimeter") {
      sensor->type = SIM_SENS_VELOCIMETER;
      sensor->objtype = SIM_OBJ_SITE;
      ReadAttrTxt(elem, "site", objname, true);
    } else if (type == "gyro") {
      sensor->type = SIM_SENS_GYRO;
      sensor->objtype = SIM_OBJ_SITE;
      ReadAttrTxt(elem, "site", objname, true);
    } else if (type == "force") {
      sensor->type = SIM_SENS_FORCE;
      sensor->objtype = SIM_OBJ_SITE;
      ReadAttrTxt(elem, "site", objname, true);
    } else if (type == "torque") {
      sensor->type = SIM_SENS_TORQUE;
      sensor->objtype = SIM_OBJ_SITE;
      ReadAttrTxt(elem, "site", objname, true);
    } else if (type == "magnetometer") {
      sensor->type = SIM_SENS_MAGNETOMETER;
      sensor->objtype = SIM_OBJ_SITE;
      ReadAttrTxt(elem, "site", objname, true);
    } else if (type == "rangefinder") {
      sensor->type = SIM_SENS_RANGEFINDER;
      sensor->objtype = SIM_OBJ_SITE;
      ReadAttrTxt(elem, "site", objname, true);

      // process data specification (intprm[0])
      int dataspec = 1 << SIM_RAYDATA_DIST;
      std::vector<int> raydata(SIM_NRAYDATA);
      int nkeys = MapValues(elem, "data", raydata.data(), raydata_map, SIM_NRAYDATA);
      if (nkeys) {
        dataspec = 1 << raydata[0];

        // check ordering while adding bits to dataspec
        for (int i = 1; i < nkeys; ++i) {
          if (raydata[i] <= raydata[i-1]) {
            std::string correct_order;
            for (int j = 0; j < SIM_NRAYDATA; ++j) {
              correct_order += raydata_map[j].key;
              if (j < SIM_NRAYDATA - 1) correct_order += ", ";
            }
            throw sim_xml_error_t(elem, "data attributes must be in order: %s", correct_order.c_str());
          }
          dataspec |= 1 << raydata[i];
        }
      }
      sensor->intprm[0] = dataspec;
    }

    // sensors related to scalar joints, tendons, actuators
    else if (type == "jointpos") {
      sensor->type = SIM_SENS_JOINTPOS;
      sensor->objtype = SIM_OBJ_JOINT;
      ReadAttrTxt(elem, "joint", objname, true);
    } else if (type == "jointvel") {
      sensor->type = SIM_SENS_JOINTVEL;
      sensor->objtype = SIM_OBJ_JOINT;
      ReadAttrTxt(elem, "joint", objname, true);
    } else if (type == "tendonpos") {
      sensor->type = SIM_SENS_TENDONPOS;
      sensor->objtype = SIM_OBJ_TENDON;
      ReadAttrTxt(elem, "tendon", objname, true);
    } else if (type == "tendonvel") {
      sensor->type = SIM_SENS_TENDONVEL;
      sensor->objtype = SIM_OBJ_TENDON;
      ReadAttrTxt(elem, "tendon", objname, true);
    } else if (type == "actuatorpos") {
      sensor->type = SIM_SENS_ACTUATORPOS;
      sensor->objtype = SIM_OBJ_ACTUATOR;
      ReadAttrTxt(elem, "actuator", objname, true);
    } else if (type == "actuatorvel") {
      sensor->type = SIM_SENS_ACTUATORVEL;
      sensor->objtype = SIM_OBJ_ACTUATOR;
      ReadAttrTxt(elem, "actuator", objname, true);
    } else if (type == "actuatorfrc") {
      sensor->type = SIM_SENS_ACTUATORFRC;
      sensor->objtype = SIM_OBJ_ACTUATOR;
      ReadAttrTxt(elem, "actuator", objname, true);
    } else if (type == "jointactuatorfrc") {
      sensor->type = SIM_SENS_JOINTACTFRC;
      sensor->objtype = SIM_OBJ_JOINT;
      ReadAttrTxt(elem, "joint", objname, true);
    } else if (type=="tendonactuatorfrc") {
      sensor->type = SIM_SENS_TENDONACTFRC;
      sensor->objtype = SIM_OBJ_TENDON;
      ReadAttrTxt(elem, "tendon", objname, true);
    }

    // sensors related to ball joints
    else if (type == "ballquat") {
      sensor->type = SIM_SENS_BALLQUAT;
      sensor->objtype = SIM_OBJ_JOINT;
      ReadAttrTxt(elem, "joint", objname, true);
    } else if (type == "ballangvel") {
      sensor->type = SIM_SENS_BALLANGVEL;
      sensor->objtype = SIM_OBJ_JOINT;
      ReadAttrTxt(elem, "joint", objname, true);
    }

    // joint and tendon limit sensors
    else if (type == "jointlimitpos") {
      sensor->type = SIM_SENS_JOINTLIMITPOS;
      sensor->objtype = SIM_OBJ_JOINT;
      ReadAttrTxt(elem, "joint", objname, true);
    } else if (type == "jointlimitvel") {
      sensor->type = SIM_SENS_JOINTLIMITVEL;
      sensor->objtype = SIM_OBJ_JOINT;
      ReadAttrTxt(elem, "joint", objname, true);
    } else if (type == "jointlimitfrc") {
      sensor->type = SIM_SENS_JOINTLIMITFRC;
      sensor->objtype = SIM_OBJ_JOINT;
      ReadAttrTxt(elem, "joint", objname, true);
    } else if (type == "tendonlimitpos") {
      sensor->type = SIM_SENS_TENDONLIMITPOS;
      sensor->objtype = SIM_OBJ_TENDON;
      ReadAttrTxt(elem, "tendon", objname, true);
    } else if (type == "tendonlimitvel") {
      sensor->type = SIM_SENS_TENDONLIMITVEL;
      sensor->objtype = SIM_OBJ_TENDON;
      ReadAttrTxt(elem, "tendon", objname, true);
    } else if (type == "tendonlimitfrc") {
      sensor->type = SIM_SENS_TENDONLIMITFRC;
      sensor->objtype = SIM_OBJ_TENDON;
      ReadAttrTxt(elem, "tendon", objname, true);
    }

    // sensors attached to an object with spatial frame: (x)body, geom, site, camera
    else if (type == "framepos") {
      sensor->type = SIM_SENS_FRAMEPOS;
      ReadAttrTxt(elem, "objtype", text, true);
      sensor->objtype = (sim_obj_t)sim_math_str2Type(text.c_str());
      ReadAttrTxt(elem, "objname", objname, true);
      if (ReadAttrTxt(elem, "reftype", text)) {
        sensor->reftype = (sim_obj_t)sim_math_str2Type(text.c_str());
        ReadAttrTxt(elem, "refname", refname, true);
      } else if (ReadAttrTxt(elem, "refname", text)) {
        throw sim_xml_error_t(elem, "refname '%s' given but reftype is missing", text.c_str());
      }
    } else if (type == "framequat") {
      sensor->type = SIM_SENS_FRAMEQUAT;
      ReadAttrTxt(elem, "objtype", text, true);
      sensor->objtype = (sim_obj_t)sim_math_str2Type(text.c_str());
      ReadAttrTxt(elem, "objname", objname, true);
      if (ReadAttrTxt(elem, "reftype", text)) {
        sensor->reftype = (sim_obj_t)sim_math_str2Type(text.c_str());
        ReadAttrTxt(elem, "refname", refname, true);
      } else if (ReadAttrTxt(elem, "refname", text)) {
        throw sim_xml_error_t(elem, "refname '%s' given but reftype is missing", text.c_str());
      }
    } else if (type == "framexaxis") {
      sensor->type = SIM_SENS_FRAMEXAXIS;
      ReadAttrTxt(elem, "objtype", text, true);
      sensor->objtype = (sim_obj_t)sim_math_str2Type(text.c_str());
      ReadAttrTxt(elem, "objname", objname, true);
      if (ReadAttrTxt(elem, "reftype", text)) {
        sensor->reftype = (sim_obj_t)sim_math_str2Type(text.c_str());
        ReadAttrTxt(elem, "refname", refname, true);
      } else if (ReadAttrTxt(elem, "refname", text)) {
        throw sim_xml_error_t(elem, "refname '%s' given but reftype is missing", text.c_str());
      }
    } else if (type == "frameyaxis") {
      sensor->type = SIM_SENS_FRAMEYAXIS;
      ReadAttrTxt(elem, "objtype", text, true);
      sensor->objtype = (sim_obj_t)sim_math_str2Type(text.c_str());
      ReadAttrTxt(elem, "objname", objname, true);
      if (ReadAttrTxt(elem, "reftype", text)) {
        sensor->reftype = (sim_obj_t)sim_math_str2Type(text.c_str());
        ReadAttrTxt(elem, "refname", refname, true);
      } else if (ReadAttrTxt(elem, "refname", text)) {
        throw sim_xml_error_t(elem, "refname '%s' given but reftype is missing", text.c_str());
      }
    } else if (type == "framezaxis") {
      sensor->type = SIM_SENS_FRAMEZAXIS;
      ReadAttrTxt(elem, "objtype", text, true);
      sensor->objtype = (sim_obj_t)sim_math_str2Type(text.c_str());
      ReadAttrTxt(elem, "objname", objname, true);
      if (ReadAttrTxt(elem, "reftype", text)) {
        sensor->reftype = (sim_obj_t)sim_math_str2Type(text.c_str());
        ReadAttrTxt(elem, "refname", refname, true);
      } else if (ReadAttrTxt(elem, "refname", text)) {
        throw sim_xml_error_t(elem, "refname '%s' given but reftype is missing", text.c_str());
      }
    } else if (type == "framelinvel") {
      sensor->type = SIM_SENS_FRAMELINVEL;
      ReadAttrTxt(elem, "objtype", text, true);
      sensor->objtype = (sim_obj_t)sim_math_str2Type(text.c_str());
      ReadAttrTxt(elem, "objname", objname, true);
      if (ReadAttrTxt(elem, "reftype", text)) {
        sensor->reftype = (sim_obj_t)sim_math_str2Type(text.c_str());
        ReadAttrTxt(elem, "refname", refname, true);
      } else if (ReadAttrTxt(elem, "refname", text)) {
        throw sim_xml_error_t(elem, "refname '%s' given but reftype is missing", text.c_str());
      }
    } else if (type == "frameangvel") {
      sensor->type = SIM_SENS_FRAMEANGVEL;
      ReadAttrTxt(elem, "objtype", text, true);
      sensor->objtype = (sim_obj_t)sim_math_str2Type(text.c_str());
      ReadAttrTxt(elem, "objname", objname, true);
      if (ReadAttrTxt(elem, "reftype", text)) {
        sensor->reftype = (sim_obj_t)sim_math_str2Type(text.c_str());
        ReadAttrTxt(elem, "refname", refname, true);
      } else if (ReadAttrTxt(elem, "refname", text)) {
        throw sim_xml_error_t(elem, "refname '%s' given but reftype is missing", text.c_str());
      }
    } else if (type == "framelinacc") {
      sensor->type = SIM_SENS_FRAMELINACC;
      ReadAttrTxt(elem, "objtype", text, true);
      sensor->objtype = (sim_obj_t)sim_math_str2Type(text.c_str());
      ReadAttrTxt(elem, "objname", objname, true);
    } else if (type == "frameangacc") {
      sensor->type = SIM_SENS_FRAMEANGACC;
      ReadAttrTxt(elem, "objtype", text, true);
      sensor->objtype = (sim_obj_t)sim_math_str2Type(text.c_str());
      ReadAttrTxt(elem, "objname", objname, true);
    } else if (type == "insidesite") {
      sensor->type = SIM_SENS_INSIDESITE;
      sensor->reftype = SIM_OBJ_SITE;
      ReadAttrTxt(elem, "site", refname, true);
      ReadAttrTxt(elem, "objtype", text, true);
      sensor->objtype = (sim_obj_t)sim_math_str2Type(text.c_str());
      ReadAttrTxt(elem, "objname", objname, true);
    }

    // sensors related to kinematic subtrees; attached to a body (which is the subtree root)
    else if (type == "subtreecom") {
      sensor->type = SIM_SENS_SUBTREECOM;
      sensor->objtype = SIM_OBJ_BODY;
      ReadAttrTxt(elem, "body", objname, true);
    } else if (type == "subtreelinvel") {
      sensor->type = SIM_SENS_SUBTREELINVEL;
      sensor->objtype = SIM_OBJ_BODY;
      ReadAttrTxt(elem, "body", objname, true);
    } else if (type == "subtreeangmom") {
      sensor->type = SIM_SENS_SUBTREEANGMOM;
      sensor->objtype = SIM_OBJ_BODY;
      ReadAttrTxt(elem, "body", objname, true);
    }

    // sensors for geometric distance; attached to geoms or bodies
    else if (type == "distance" || type == "normal" || type == "fromto") {
      bool has_body1 = ReadAttrTxt(elem, "body1", objname);
      bool has_geom1 = ReadAttrTxt(elem, "geom1", objname);
      if (has_body1 == has_geom1) {
        throw sim_xml_error_t(elem, "exactly one of (geom1, body1) must be specified");
      }
      sensor->objtype = has_body1 ? SIM_OBJ_BODY : SIM_OBJ_GEOM;
      bool has_body2 = ReadAttrTxt(elem, "body2", refname);
      bool has_geom2 = ReadAttrTxt(elem, "geom2", refname);
      if (has_body2 == has_geom2) {
        throw sim_xml_error_t(elem, "exactly one of (geom2, body2) must be specified");
      }
      sensor->reftype = has_body2 ? SIM_OBJ_BODY : SIM_OBJ_GEOM;
      if (type == "distance") {
        sensor->type = SIM_SENS_GEOMDIST;
      } else if (type == "normal") {
        sensor->type = SIM_SENS_GEOMNORMAL;
      } else {
        sensor->type = SIM_SENS_GEOMFROMTO;
      }
    }

    // sensor for contacts; attached to geoms or bodies or a site
    else if (type == "contact") {
      // first matching criterion
      bool has_site = ReadAttrTxt(elem, "site", objname);
      bool has_body1 = ReadAttrTxt(elem, "body1", objname);
      bool has_subtree1 = ReadAttrTxt(elem, "subtree1", objname);
      bool has_geom1 = ReadAttrTxt(elem, "geom1", objname);
      if (has_site + has_body1 + has_subtree1 + has_geom1 > 1) {
        throw sim_xml_error_t(elem, "at most one of (geom1, body1, subtree1, site) can be specified");
      }
      if (has_site)          { sensor->objtype = SIM_OBJ_SITE; }
      else if (has_body1)    { sensor->objtype = SIM_OBJ_BODY; }
      else if (has_subtree1) { sensor->objtype = SIM_OBJ_XBODY; }
      else if (has_geom1)    { sensor->objtype = SIM_OBJ_GEOM; }
      else                   { sensor->objtype = SIM_OBJ_UNKNOWN; }

      // second matching criterion
      bool has_body2 = ReadAttrTxt(elem, "body2", refname);
      bool has_subtree2 = ReadAttrTxt(elem, "subtree2", refname);
      bool has_geom2 = ReadAttrTxt(elem, "geom2", refname);
      if (has_body2 + has_subtree2 + has_geom2 > 1) {
        throw sim_xml_error_t(elem, "at most one of (geom2, body2, subtree2) can be specified");
      }
      if (has_body2)         { sensor->reftype = SIM_OBJ_BODY; }
      else if (has_subtree2) { sensor->reftype = SIM_OBJ_XBODY; }
      else if (has_geom2)    { sensor->reftype = SIM_OBJ_GEOM; }
      else                   { sensor->reftype = SIM_OBJ_UNKNOWN; }

      // process data specification (intprm[0])
      int dataspec = 1 << SIM_CONDATA_FOUND;
      std::vector<int> condata(SIM_NCONDATA);
      int nkeys = MapValues(elem, "data", condata.data(), condata_map, SIM_NCONDATA);
      if (nkeys) {
        dataspec = 1 << condata[0];

        // check ordering while adding bits to dataspec
        for (int i = 1; i < nkeys; ++i) {
          if (condata[i] <= condata[i-1]) {
            std::string correct_order;
            for (int j = 0; j < SIM_NCONDATA; ++j) {
              correct_order += condata_map[j].key;
              if (j < SIM_NCONDATA - 1) correct_order += ", ";
            }
            throw sim_xml_error_t(elem, "data attributes must be in order: %s", correct_order.c_str());
          }
          dataspec |= 1 << condata[i];
        }
      }
      sensor->intprm[0] = dataspec;

      // reduction type (intprm[1])
      sensor->intprm[1] = 0;
      if (MapValue(elem, "reduce", &n, reduce_map, reduce_sz)) {
        sensor->intprm[1] = n;
      }

      // number of contacts (intprm[2])
      sensor->intprm[2] = 1;
      ReadAttrInt(elem, "num", &sensor->intprm[2]);
      if (sensor->intprm[2] <= 0) {
        throw sim_xml_error_t(elem, "'num' must be positive in sensor");
      }

      // sensor type
      sensor->type = SIM_SENS_CONTACT;
    }

    // global sensors
    else if (type == "e_potential") {
      sensor->type = SIM_SENS_E_POTENTIAL;
      sensor->objtype = SIM_OBJ_UNKNOWN;
    } else if (type == "e_kinetic") {
      sensor->type = SIM_SENS_E_KINETIC;
      sensor->objtype = SIM_OBJ_UNKNOWN;
    } else if (type == "clock") {
      sensor->type = SIM_SENS_CLOCK;
      sensor->objtype = SIM_OBJ_UNKNOWN;
    }

    // user-defined sensor
    else if (type == "user") {
      sensor->type = SIM_SENS_USER;
      bool objname_given = ReadAttrTxt(elem, "objname", objname);
      if (ReadAttrTxt(elem, "objtype", text)) {
        if (!objname_given) {
          throw sim_xml_error_t(elem, "objtype '%s' given but objname is missing", text.c_str());
        }
        sensor->objtype = (sim_obj_t)sim_math_str2Type(text.c_str());
      } else if (objname_given) {
        throw sim_xml_error_t(elem, "objname '%s' given but objtype is missing", objname.c_str());
      }
      ReadAttrInt(elem, "dim", &sensor->dim, true);

      // keywords
      if (MapValue(elem, "needstage", &n, stage_map, stage_sz)) {
        sensor->needstage = (SIM_tStage)n;
      }
      if (MapValue(elem, "datatype", &n, datatype_map, datatype_sz)) {
        sensor->datatype = (SIM_tDataType)n;
      }
    }

    // tactile sensor
    if (type == "tactile") {
      sensor->type = SIM_SENS_TACTILE;
      sensor->reftype = SIM_OBJ_GEOM;
      ReadAttrTxt(elem, "geom", refname, /*required=*/true);

      // associate the sensor with a mesh
      sensor->objtype = SIM_OBJ_MESH;
      ReadAttrTxt(elem, "mesh", objname, /*required=*/true);
      sim_spec_set_string(sensor->objname, objname.c_str());
    }

    else if (type == "plugin") {
      sensor->type = SIM_SENS_PLUGIN;
      OnePlugin(elem, &sensor->plugin);
      ReadAttrTxt(elem, "objtype", text);
      sensor->objtype = (sim_obj_t)sim_math_str2Type(text.c_str());
      ReadAttrTxt(elem, "objname", objname);
      if (sensor->objtype != SIM_OBJ_UNKNOWN && objname.empty()) {
        throw sim_xml_error_t(elem, "objtype is specified but objname is not");
      }
      if (sensor->objtype == SIM_OBJ_UNKNOWN && !objname.empty()) {
        throw sim_xml_error_t(elem, "objname is specified but objtype is not");
      }
      if (ReadAttrTxt(elem, "reftype", text)) {
        sensor->reftype = (sim_obj_t)sim_math_str2Type(text.c_str());
      }
      ReadAttrTxt(elem, "refname", refname);
      if (sensor->reftype != SIM_OBJ_UNKNOWN && refname.empty()) {
        throw sim_xml_error_t(elem, "reftype is specified but refname is not");
      }
      if (sensor->reftype == SIM_OBJ_UNKNOWN && !refname.empty()) {
        throw sim_xml_error_t(elem, "refname is specified but reftype is not");
      }
    }

    if (!objname.empty()) {
      sim_spec_set_string(sensor->objname, objname.c_str());
    }

    if (!refname.empty()) {
      sim_spec_set_string(sensor->refname, refname.c_str());
    }

    // write info
    sim_spec_set_string(sensor->info, ("line " + std::to_string(elem->GetLineNum())).c_str());

    // advance to next element
    elem = NextSiblingElement(elem);
  }
}



// keyframe section parser
void sim_xml_reader_t::Keyframe(XMLElement* section) {
  XMLElement* elem;

  // iterate over child elements
  elem = FirstChildElement(section);
  while (elem) {
    string text, name = "";

    // add keyframe
    SIM_sKey* key = sim_spec_addKey(spec);

    // read name, time
    ReadAttrTxt(elem, "name", name);
    if (sim_spec_set_name(key->element, name.c_str())) {
      throw sim_xml_error_t(elem, "%s", sim_spec_getError(spec));
    }
    ReadAttr(elem, "time", 1, &key->time, text);

    // read qpos
    auto maybe_data = ReadAttrVec<double>(elem, "qpos", false);
    if (maybe_data.has_value()) {
      sim_spec_setDouble(key->qpos, maybe_data->data(), maybe_data->size());
    }

    // read qvel
    maybe_data = ReadAttrVec<double>(elem, "qvel", false);
    if (maybe_data.has_value()) {
      sim_spec_setDouble(key->qvel, maybe_data->data(), maybe_data->size());
    }

    // read act
    maybe_data = ReadAttrVec<double>(elem, "act", false);
    if (maybe_data.has_value()) {
      sim_spec_setDouble(key->act, maybe_data->data(), maybe_data->size());
    }

    // read mpos
    maybe_data = ReadAttrVec<double>(elem, "mpos", false);
    if (maybe_data.has_value()) {
      sim_spec_setDouble(key->mpos, maybe_data->data(), maybe_data->size());
    }

    // read mquat
    maybe_data = ReadAttrVec<double>(elem, "mquat", false);
    if (maybe_data.has_value()) {
      sim_spec_setDouble(key->mquat, maybe_data->data(), maybe_data->size());
    }

    // read ctrl
    maybe_data = ReadAttrVec<double>(elem, "ctrl", false);
    if (maybe_data.has_value()) {
      sim_spec_setDouble(key->ctrl, maybe_data->data(), maybe_data->size());
    }

    // advance to next element
    elem = NextSiblingElement(elem);
  }
}



// get defaults class
const sim_spec_default_t* sim_xml_reader_t::GetClass(XMLElement* section) {
  string text;

  if (!ReadAttrTxt(section, "class", text)) {
    return nullptr;
  }

  const sim_spec_default_t* def = sim_spec_findDefault(spec, text.c_str());
  if (!def) {
    throw sim_xml_error_t(
            section,
            string("unknown default class name '" + text + "'").c_str());
  }
  return def;
}

void sim_xml_reader_t::SetModelFileDir(const string& modelfiledir) {
  modelfiledir_ = FilePath(modelfiledir);
}

void sim_xml_reader_t::SetAssetDir(const string& assetdir) {
  assetdir_ = FilePath(assetdir);
}

void sim_xml_reader_t::SetMeshDir(const string& meshdir) {
  meshdir_ = FilePath(meshdir);
}

void sim_xml_reader_t::SetTextureDir(const string& texturedir) {
  texturedir_ = FilePath(texturedir);
}

FilePath sim_xml_reader_t::AssetDir() const {
  return modelfiledir_ + assetdir_;
}

FilePath sim_xml_reader_t::MeshDir() const {
  if (meshdir_.empty()) {
    return AssetDir();
  }
  return modelfiledir_ + meshdir_;
}
FilePath sim_xml_reader_t::TextureDir() const {
  if (texturedir_.empty()) {
    return AssetDir();
  }
  return modelfiledir_ + texturedir_;
}
