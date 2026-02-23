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

#include <array>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

#include <simcore/SIM_model.h>
#include <simcore/SIM_spec.h>
#include "user/user_api.h"
#include "user/user_util.h"
#include "xml/xml_native_reader.h"
#include "xml/xml_urdf.h"
#include "xml/xml_util.h"
#include "tinyxml2.h"

using tinyxml2::XMLElement;

// URDF joint type
static const int urJoint_sz = 7;
static const sim_map_t urJoint_map[urJoint_sz] = {
  {"revolute",    0},
  {"continuous",  1},
  {"prismatic",   2},
  {"fixed",       3},
  {"floating",    4},
  {"planar",      5},
  {"spherical",   6}  // Bullet physics supports ball joints (non-standard URDF)
};



//---------------------------------- class SIM_XURDF -------------------------------------------------

// constructor
SIM_XURDF::SIM_XURDF() {
  Clear();
}



// destructor
SIM_XURDF::~SIM_XURDF() {
  Clear();
}


// clear internal variables
void SIM_XURDF::Clear(void) {
  spec = 0;

  urName.clear();
  urParent.clear();
  urChildren.clear();
  urGeomNames.clear();
}

std::string SIM_XURDF::GetPrefixedName(const std::string& name) {
  if (name.empty()) {
    return name;
  }
  if (urPrefix.empty()) {
    return name;
  }
  return urPrefix + "/" + name;
}

// actual parser
void SIM_XURDF::Parse(
    XMLElement* root, const std::string& prefix, double* pos, double* quat,
    const bool static_body) {
  std::string name, text;
  XMLElement *elem, *temp;
  int id_parent, id_child;
  urPrefix = prefix;

  // parse SimCore sections (not part of URDF)
  XMLElement* simc = FindSubElem(root, "simcore");
  if (simc) {
    XMLElement *section;
    if ((section = FindSubElem(simc, "compiler"))) {
      sim_xml_reader_t::Compiler(section, spec);
    }

    if ((section = FindSubElem(simc, "option"))) {
      sim_xml_reader_t::Option(section, &spec->option);
    }

    if ((section = FindSubElem(simc, "size"))) {
      sim_xml_reader_t::Size(section, spec);
    }
  }

  // enforce required compiler defaults for URDF
  spec->compiler.degree = false;

  // get model name
  std::string modelname;
  if (ReadAttrTxt(root, "name", modelname)) {
    sim_spec_set_string(spec->modelname, modelname.c_str());
  }

  // find all links/bodies, save names
  elem = root->FirstChildElement();
  while (elem) {
    // identify link elements
    name = elem->Value();
    if (name == "link") {
      ReadAttrTxt(elem, "name", text, true);
      text = GetPrefixedName(text);
      AddBody(text);
    }

    // advance to next element
    elem = elem->NextSiblingElement();
  }

  // find all joints, assign parent and child pointers
  elem = root->FirstChildElement();
  while (elem) {
    // identify joint elements
    name = elem->Value();
    if (name == "joint") {
      // find parent, get name and id
      temp = FindSubElem(elem, "parent", true);
      ReadAttrTxt(temp, "link", text, true);
      text = GetPrefixedName(text);
      id_parent = FindName(text, urName);

      // find child, get name and id
      temp = FindSubElem(elem, "child", true);
      ReadAttrTxt(temp, "link", text, true);
      text = GetPrefixedName(text);
      id_child = FindName(text, urName);

      // make sure parent and child exist
      if (id_parent < 0 || id_child < 0) {
        throw sim_xml_error_t(elem, "URDF joint parent or child missing");
      }

      // check for multiple parents
      if (urParent[id_child] >= 0) {
        throw sim_xml_error_t(elem, "URDF body has multiple parents:", urName[id_child].c_str());
      }

      // add parent and child info
      urParent[id_child] = id_parent;
      urChildren[id_parent].push_back(id_child);
    }

    // advance to next element
    elem = elem->NextSiblingElement();
  }

  // find all top-level bodies, call recursive tree constructor
  for (int i=0; i < (int)urName.size(); i++) {
    if (urParent[i] < 0) {
      AddToTree(i);
    }
  }

  // parse bodies
  elem = root->FirstChildElement();
  while (elem) {
    // identify body/link elements
    name = elem->Value();
    if (name == "link") {
      Body(elem);
    }

    // advance to next element
    elem = elem->NextSiblingElement();
  }

  // parse joints
  elem = root->FirstChildElement();
  while (elem) {
    // identify body/link elements
    name = elem->Value();
    if (name == "joint") {
      Joint(elem);
    }

    // advance to next element
    elem = elem->NextSiblingElement();
  }

  // override the pose for the base link and add a free joint
  for (int i = 0; i < (int)urName.size(); i++) {
    if (urParent[i] < 0) {
      sim_spec_body_t* world = sim_spec_findBody(spec, "world");
      sim_spec_body_t* pbody = sim_spec_findChild(world, urName[i].c_str());
      sim_math_internal_copy_vec(pbody->pos, pos, 3);
      sim_math_internal_copy_vec(pbody->quat, quat, 4);

      // add a free joint to allow motion of the body
      // if the mass is 0, assume the object is static
      if (!static_body && pbody->mass > 0) {
        SIM_sJoint* pjoint = sim_spec_addJoint(pbody, 0);
        sim_spec_set_name(pjoint->element, (urName[i] + "_free_joint").c_str());
        pjoint->type = SIM_JNT_FREE;
      }
    }
  }
}

// parse body/link
void SIM_XURDF::Body(XMLElement* body_elem) {
  std::string name, text;
  XMLElement *elem, *temp;
  sim_spec_body_t *pbody, *world;
  sim_spec_geom_t* pgeom;

  // get body name and pointer to sim_spec_body_t
  ReadAttrTxt(body_elem, "name", name, true);
  name = GetPrefixedName(name);
  world = sim_spec_findBody(spec, "world");
  pbody = sim_spec_findChild(world, name.c_str());
  if (!pbody) {
    throw sim_xml_error_t(body_elem, "URDF body not found");  // SHOULD NOT OCCUR
  }
  // inertial element: copy into alternative body frame
  if ((elem = FindSubElem(body_elem, "inertial"))) {
    pbody->explicitinertial = true;
    // origin- relative to joint frame for now
    Origin(elem, pbody->ipos, pbody->iquat);

    // mass
    temp = FindSubElem(elem, "mass", true);
    ReadAttr(temp, "value", 1, &pbody->mass, text, true);

    // inertia
    temp = FindSubElem(elem, "inertia", true);
    ReadAttr(temp, "ixx", 1, pbody->fullinertia+0, text, true);
    ReadAttr(temp, "iyy", 1, pbody->fullinertia+1, text, true);
    ReadAttr(temp, "izz", 1, pbody->fullinertia+2, text, true);
    ReadAttr(temp, "ixy", 1, pbody->fullinertia+3, text, true);
    ReadAttr(temp, "ixz", 1, pbody->fullinertia+4, text, true);
    ReadAttr(temp, "iyz", 1, pbody->fullinertia+5, text, true);

    // If the inertias are all 0 in a URDF then it is still undefined.
    bool inertia_defined = false;
    for (int i = 0; i < 6; ++i) {
      if (pbody->fullinertia[i] != 0) {
        inertia_defined = true;
        break;
      }
    }
    if (!inertia_defined) {
      pbody->fullinertia[0] = SIM_NAN;
    }

    // process inertia
    //  lquat = rotation from specified to default (joint/body) inertial frame
    double lquat[4] = {1, 0, 0, 0};
    double tmpquat[4] = {1, 0, 0, 0};
    const char* altres = sim_math_internal_fullInertia(lquat, nullptr, pbody->fullinertia);

    // inertias are sometimes 0 in URDF files: ignore error in altres, fix later
    (void) altres;

    // correct for alignment of full inertia matrix
    sim_math_internal_mulquat(tmpquat, pbody->iquat, lquat);
    sim_math_internal_copy_vec(pbody->iquat, tmpquat, 4);
  }

  // clear body frame; set by joint later
  sim_math_internal_set_vec(pbody->pos, 0, 0, 0);
  sim_math_internal_set_vec(pbody->quat, 1, 0, 0, 0);

  // process all visual and geometry elements in order
  std::string geom_name;

  elem = body_elem->FirstChildElement();
  while (elem) {
    name = elem->Value();

    // visual element
    if (name == "visual") {
      // create geom if not discarded
      if (!spec->compiler.discardvisual) {
        pgeom = Geom(elem, pbody, false);

        // save name if it doesn't already exist.
        sim_xml_util_t::ReadAttrTxt(elem, "name", geom_name);
        name = GetPrefixedName(name);
        if (urGeomNames.find(geom_name) == urGeomNames.end()) {
          sim_spec_set_name(pgeom->element, geom_name.c_str());
          urGeomNames.insert(geom_name);
        } else if (!geom_name.empty()) {
          std::cerr << "WARNING: Geom with duplicate name '" << geom_name
                    << "' encountered in URDF, creating an unnamed geom."
                    << std::endl;
        }
      }
    }

    // collision element
    else if (name == "collision") {
      pgeom = Geom(elem, pbody, true);

      // save name if it doesn't already exist.
      sim_xml_util_t::ReadAttrTxt(elem, "name", geom_name);
      geom_name = GetPrefixedName(geom_name);
      if (urGeomNames.find(geom_name) == urGeomNames.end()) {
        sim_spec_set_name(pgeom->element, geom_name.c_str());
        urGeomNames.insert(geom_name);
      } else if (!geom_name.empty()) {
        std::cerr << "WARNING: Geom with duplicate name '" << geom_name
                  << "' encountered in URDF, creating an unnamed geom."
                  << std::endl;
      }
    }
    // advance
    elem = elem->NextSiblingElement();
  }
}

void SIM_XURDF::Parse(XMLElement* root, const SIM_VFS* vfs) {
  double pos[3] = {0};
  sim_math_internal_set_vec(pos, 0, 0, 0);
  double quat[4] = {1, 0, 0, 0};
  sim_math_internal_set_vec(quat, 1, 0, 0, 0);
  Parse(root, /*prefix=*/"", pos, quat, true);
}

// parse joint
void SIM_XURDF::Joint(XMLElement* joint_elem) {
  std::string jntname, name, text;
  XMLElement *elem;
  sim_spec_body_t *pbody, *parent, *world;
  SIM_sJoint *pjoint=0, *pjoint1=0, *pjoint2=0;
  int jointtype;

  // get type and name
  ReadAttrTxt(joint_elem, "type", text, true);
  jointtype = FindKey(urJoint_map, urJoint_sz, text);
  if (jointtype < 0) {
    throw sim_xml_error_t(joint_elem, "invalid joint type in URDF joint definition");
  }
  ReadAttrTxt(joint_elem, "name", jntname, true);
  jntname = GetPrefixedName(jntname);
  // get parent, check
  elem = FindSubElem(joint_elem, "parent", true);
  ReadAttrTxt(elem, "link", name, true);
  name = GetPrefixedName(name);
  world = sim_spec_findBody(spec, "world");
  parent = sim_spec_findChild(world, name.c_str());
  if (!parent) {                      // SHOULD NOT OCCUR
    throw sim_xml_error_t(elem, "invalid parent name in URDF joint definition");
  }

  // get child=this, check
  elem = FindSubElem(joint_elem, "child", true);
  ReadAttrTxt(elem, "link", name, true);
  name = GetPrefixedName(name);
  world = sim_spec_findBody(spec, "world");
  pbody = sim_spec_findChild(world, name.c_str());
  if (!pbody) {                       // SHOULD NOT OCCUR
    throw sim_xml_error_t(elem, "invalid child name in URDF joint definition");
  }

  // read origin and axis
  double axis[3] = {1, 0, 0};
  Origin(joint_elem, pbody->pos, pbody->quat);
  if ((elem = FindSubElem(joint_elem, "axis"))) {
    ReadAttr(elem, "xyz", 3, axis, text, /*required=*/true);
  }

  // create joint (unless fixed)
  double mat[9], quat[4], tmpaxis[3];
  switch (jointtype) {
    case 0:   // revolute
    case 1:   // continuous
      pjoint = sim_spec_addJoint(pbody, 0);
      sim_spec_set_name(pjoint->element, jntname.c_str());
      pjoint->type = SIM_JNT_HINGE;
      sim_math_internal_set_vec(pjoint->pos, 0, 0, 0);
      sim_math_internal_copy_vec(pjoint->axis, axis, 3);
      break;

    case 2:   // prismatic
      pjoint = sim_spec_addJoint(pbody, 0);
      sim_spec_set_name(pjoint->element, jntname.c_str());
      pjoint->type = SIM_JNT_SLIDE;
      sim_math_internal_set_vec(pjoint->pos, 0, 0, 0);
      sim_math_internal_copy_vec(pjoint->axis, axis, 3);
      break;

    case 3:   // fixed- no joint, return
      return;

    case 4:   // floating
      pjoint = sim_spec_addJoint(pbody, 0);
      sim_spec_set_name(pjoint->element, jntname.c_str());
      pjoint->type = SIM_JNT_FREE;
      break;

    case 5:   // planar- construct complex joint
      // make frame with axis = z
      sim_math_internal_z2quat(quat, axis);
      sim_math_internal_quat2mat(mat, quat);

      // construct slider along x
      pjoint = sim_spec_addJoint(pbody, 0);
      sim_spec_set_name(pjoint->element, (jntname + "_TX").c_str());
      pjoint->type = SIM_JNT_SLIDE;
      tmpaxis[0] = mat[0];
      tmpaxis[1] = mat[3];
      tmpaxis[2] = mat[6];
      sim_math_internal_set_vec(pjoint->pos, 0, 0, 0);
      sim_math_internal_copy_vec(pjoint->axis, tmpaxis, 3);

      // construct slider along y
      pjoint1 = sim_spec_addJoint(pbody, 0);
      sim_spec_set_name(pjoint1->element, (jntname + "_TY").c_str());
      pjoint1->type = SIM_JNT_SLIDE;
      tmpaxis[0] = mat[1];
      tmpaxis[1] = mat[4];
      tmpaxis[2] = mat[7];
      sim_math_internal_set_vec(pjoint1->pos, 0, 0, 0);
      sim_math_internal_copy_vec(pjoint1->axis, tmpaxis, 3);

      // construct hinge around z = locaxis
      pjoint2 = sim_spec_addJoint(pbody, 0);
      sim_spec_set_name(pjoint2->element, (jntname + "_RZ").c_str());
      pjoint2->type = SIM_JNT_HINGE;
      sim_math_internal_set_vec(pjoint2->pos, 0, 0, 0);
      sim_math_internal_copy_vec(pjoint2->axis, axis, 3);
      break;

    case 6: // ball joint
      pjoint = sim_spec_addJoint(pbody, 0);
      sim_spec_set_name(pjoint->element, jntname.c_str());
      pjoint->type = SIM_JNT_BALL;
      sim_math_internal_set_vec(pjoint->pos, 0, 0, 0);
      sim_math_internal_copy_vec(pjoint->axis, axis, 3);
  }

  // dynamics element
  if ((elem = FindSubElem(joint_elem, "dynamics"))) {
    ReadAttr(elem, "damping", 1, &pjoint->damping, text);
    ReadAttr(elem, "friction", 1, &pjoint->frictionloss, text);

    // copy parameters to all elements of planar joint
    if (pjoint1) {
      pjoint1->damping = pjoint2->damping = pjoint->damping;
      pjoint1->frictionloss = pjoint2->frictionloss = pjoint->frictionloss;
    }
  }

  // limit element
  if ((elem = FindSubElem(joint_elem, "limit"))) {
    bool haslower = ReadAttr(elem, "lower", 1, pjoint->range, text);
    bool hasupper = ReadAttr(elem, "upper", 1, pjoint->range+1, text);

    // handle range mis-specification, otherwise the default SIM_LIMITED_AUTO will do the right thing
    bool bad_range = (haslower != hasupper) || pjoint->range[0] > pjoint->range[1];
    if (bad_range) {
      pjoint->limited = SIM_LIMITED_FALSE;
    }

    // ReadAttr(elem, "velocity", 1, &pjoint->maxvel, text); // no maxvel in SimCore
    double effort = 0;
    ReadAttr(elem, "effort", 1, &effort, text);
    effort = std::abs(effort);
    if (effort > 0) {
      pjoint->actfrcrange[0] = -effort;
      pjoint->actfrcrange[1] = effort;
    }
  }
}



// parse origin and geometry elements of visual or collision
sim_spec_geom_t* SIM_XURDF::Geom(XMLElement* geom_elem, sim_spec_body_t* pbody, bool collision) {
  XMLElement *elem, *temp;
  std::string text, meshfile;

  // get geometry element
  elem = FindSubElem(geom_elem, "geometry", true);

  // add BOX geom, modify type later
  sim_spec_geom_t* pgeom = sim_spec_addGeom(pbody, 0);
  sim_spec_set_name(pgeom->element, "");
  pgeom->type = SIM_GEOM_BOX;
  if (collision) {
    pgeom->contype = 1;
    pgeom->conaffinity = 1;
  } else {
    pgeom->contype = 0;
    pgeom->conaffinity = 0;
    pgeom->group = 1;
    pgeom->density = 0;
  }

  // box
  if ((temp = FindSubElem(elem, "box"))) {
    ReadAttr(temp, "size", 3, pgeom->size, text, true, true);
    for (int i=0; i < 3; i++) {
      pgeom->size[i] /= 2;  // SimCore uses half-length
    }
  }

  // cylinder
  else if ((temp = FindSubElem(elem, "cylinder"))) {
    pgeom->type = SIM_GEOM_CYLINDER;
    ReadAttr(temp, "radius", 1, pgeom->size, text, true, true);
    ReadAttr(temp, "length", 1, pgeom->size+1, text, true, true);
    pgeom->size[1] /= 2;            // SimCore uses half-length
  }

  // sphere
  else if ((temp = FindSubElem(elem, "sphere"))) {
    pgeom->type = SIM_GEOM_SPHERE;
    ReadAttr(temp, "radius", 1, pgeom->size, text, true, true);
  }

  // capsule
  else if ((temp = FindSubElem(elem, "capsule"))) {
    pgeom->type = SIM_GEOM_CAPSULE;
    ReadAttr(temp, "radius", 1, pgeom->size, text, true, true);
    ReadAttr(temp, "length", 1, pgeom->size+1, text, true, true);
    pgeom->size[1] /= 2;            // SimCore uses half-length
  }

  // mesh
  else if ((temp = FindSubElem(elem, "mesh"))) {
    SIM_sMesh* pmesh = 0;
    bool newmesh = false;

    // set geom type and read mesh attributes
    pgeom->type = SIM_GEOM_MESH;
    meshfile = ReadAttrStr(temp, "filename", true).value();
    std::array<double, 3> default_meshscale = {1, 1, 1};
    std::array<double, 3> meshscale = ReadAttrArr<double, 3>(temp, "scale")
                                      .value_or(default_meshscale);

    // strip file name if necessary
    if (spec->strippath) {
      meshfile = sim_math_internal_strippath(meshfile);
    }

    // construct mesh name: always stripped
    std::string meshname = sim_math_internal_strippath(meshfile);
    meshname = sim_math_internal_stripext(meshname);

    if (meshes.find(meshname) == meshes.end()) {
      // does not exist: create
      pmesh = sim_spec_addMesh(spec, 0);
      meshes[meshname].push_back(pmesh);
      newmesh = true;
    } else {
      int i = 0;

      // find if it exists with the same scale
      for (SIM_sMesh* mesh : meshes[meshname]) {
        if (mesh->scale[0] == meshscale[0] &&
            mesh->scale[1] == meshscale[1] &&
            mesh->scale[2] == meshscale[2]) {
          pmesh = mesh;
          break;
        }
        i++;
      }

      // add a new spec making an incremental new name
      if (i == meshes[meshname].size()) {
        pmesh = sim_spec_addMesh(spec, 0);
        meshes[meshname].push_back(pmesh);
        meshname = meshname + std::to_string(i);
        newmesh = true;
      }
    }

    // set fields
    if (newmesh) {
      sim_spec_set_string(pmesh->file, meshfile.c_str());
      sim_spec_set_name(pmesh->element, meshname.c_str());
      pmesh->scale[0] = meshscale[0];
      pmesh->scale[1] = meshscale[1];
      pmesh->scale[2] = meshscale[2];
    }
    sim_spec_set_string(pgeom->meshname, meshname.c_str());
  }

  else {
    throw sim_xml_error_t(elem, "visual geometry specification not found");
  }

  // origin element
  Origin(geom_elem, pgeom->pos, pgeom->quat);

  return pgeom;
}



// parse origin element
void SIM_XURDF::Origin(XMLElement* origin_elem, double* pos, double* quat) {
  XMLElement* temp;
  std::string text;

  // set defaults
  sim_math_internal_set_vec(pos, 0, 0, 0);
  sim_math_internal_set_vec(quat, 1, 0, 0, 0);

  // read origin element if present
  if ((temp = FindSubElem(origin_elem, "origin"))) {
    // position
    ReadAttr(temp, "xyz", 3, pos, text);

    // orientation
    SIM_sOrientation alt;
    sim_spec_defaultOrientation(&alt);
    if (ReadAttr(temp, "rpy", 3, alt.euler, text)) {
      alt.type = SIM_ORIENTATION_EULER;
      sim_spec_resolveOrientation(quat, 0, "XYZ", &alt);
    }
  }
}



// find body with given name in list, return -1 if not found
int SIM_XURDF::FindName(std::string name, std::vector<std::string>& list) {
  for (unsigned int i=0; i < list.size(); i++)
    if (list[i] == name) {
      return i;
    }

  return -1;
}



// add name to list, error if name already exists
void SIM_XURDF::AddName(std::string name, std::vector<std::string>& list) {
  // make sure name is unique
  if (FindName(name, list) >= 0) {
    throw sim_xml_error_t(0, "repeated URDF name: ", name.c_str());
  }

  list.push_back(name);
}



// add body name to list of URDF bodies, error if name already exists
void SIM_XURDF::AddBody(std::string name) {
  // add body name, make sure it is unique
  AddName(name, urName);

  // add parent and child elements
  urParent.push_back(-1);
  std::vector<int> children;
  children.clear();
  urChildren.push_back(children);
}



// add body with given number to the sim_builder_model_t tree, process children
void SIM_XURDF::AddToTree(int n) {
  // get pointer to parent in sim_builder_model_t tree
  sim_spec_body_t *parent = 0, *child = 0, *world = 0;
  if (urParent[n] >= 0) {
    world = sim_spec_findBody(spec, "world");
    parent = sim_spec_findChild(world, urName[urParent[n]].c_str());

    if (!parent)
      throw sim_xml_error_t(0, "URDF body parent should already be in tree: %s",
                     urName[urParent[n]].c_str());       // SHOULD NOT OCCUR
  } else {
    parent = sim_spec_findBody(spec, "world");
  }

  // add this body
  if (urName[n] != "world") {
    child = sim_spec_addBody(parent, 0);
    sim_spec_set_name(child->element, urName[n].c_str());
  }

  // add children recursively
  for (int i=0; i < (int)urChildren[n].size(); i++) {
    AddToTree(urChildren[n][i]);
  }
}
