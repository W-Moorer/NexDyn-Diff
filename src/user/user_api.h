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

#ifndef SIMCORE_SRC_USER_USER_API_H_
#define SIMCORE_SRC_USER_USER_API_H_

#include <math.h>
#include <stddef.h>
#include <simcore/SIM_data.h>
#include <simcore/SIM_export.h>
#include <simcore/SIM_model.h>
#include <simcore/SIM_spec.h>
#include <simcore/SIM_tnum.h>


// this is a C-API
#ifdef __cplusplus
#include <string>
#include <vector>

extern "C" {
#endif

#define SIM_NAN NAN                  // used to mark undefined fields

//---------------------------------- Top-level spec manipulation -----------------------------------

// Create spec.
SIM_API sim_spec_t* sim_makeSpec(void);

// Compile spec to model.
SIM_API sim_model_t* sim_compile(sim_spec_t* s, const SIM_VFS* vfs);

// Recompile spec to model, preserving the state, return 0 on success.
SIM_API int sim_recompile(sim_spec_t* s, const SIM_VFS* vfs, sim_model_t* m, sim_data_t* d);

// Copy spec.
SIM_API sim_spec_t* sim_copySpec(const sim_spec_t* s);

// Get compiler error message from spec.
SIM_API const char* sim_spec_getError(sim_spec_t* s);

// Return 1 if compiler error is a warning.
SIM_API int sim_spec_isWarning(sim_spec_t* s);

// Delete spec.
SIM_API void sim_deleteSpec(sim_spec_t* s);

// Add spec (model asset) to spec.
SIM_API void sim_spec_addSpec(sim_spec_t* s, sim_spec_t* child);

// Activate plugin, return 0 on success.
SIM_API int sim_spec_activatePlugin(sim_spec_t* s, const char* name);

// Turn deep copy on or off attach. Returns 0 on success.
SIM_API int sim_spec_setDeepCopy(sim_spec_t* s, int deepcopy);

// Copy real-valued arrays from model to spec, returns 1 on success.
SIM_API int sim_copyBack(sim_spec_t* s, const sim_model_t* m);


//---------------------------------- Attachment ----------------------------------------------------

// Attach child to a parent, return the attached element if success or NULL otherwise.
SIM_API sim_spec_element_t* sim_spec_attach(sim_spec_element_t* parent, const sim_spec_element_t* child,
                             const char* prefix, const char* suffix);


//---------------------------------- Add tree elements ---------------------------------------------

// Add child body to body, return child.
SIM_API sim_spec_body_t* sim_spec_addBody(sim_spec_body_t* body, const sim_spec_default_t* def);

// Add site to body, return site spec.
SIM_API SIM_sSite* sim_spec_addSite(sim_spec_body_t* body, const sim_spec_default_t* def);

// Add joint to body.
SIM_API SIM_sJoint* sim_spec_addJoint(sim_spec_body_t* body, const sim_spec_default_t* def);

// Add freejoint to body.
SIM_API SIM_sJoint* sim_spec_addFreeJoint(sim_spec_body_t* body);

// Add geom to body.
SIM_API sim_spec_geom_t* sim_spec_addGeom(sim_spec_body_t* body, const sim_spec_default_t* def);

// Add camera to body.
SIM_API SIM_sCamera* sim_spec_addCamera(sim_spec_body_t* body, const sim_spec_default_t* def);

// Add light to body.
SIM_API SIM_sLight* sim_spec_addLight(sim_spec_body_t* body, const sim_spec_default_t* def);

// Add frame to body.
SIM_API SIM_sFrame* sim_spec_addFrame(sim_spec_body_t* body, SIM_sFrame* parentframe);

// Remove object corresponding to the given element, return 0 on success.
SIM_API int sim_spec_delete(sim_spec_t* s, sim_spec_element_t* element);


//---------------------------------- Add non-tree elements -----------------------------------------

// Add actuator.
SIM_API sim_spec_actuator_t* sim_spec_addActuator(sim_spec_t* s, const sim_spec_default_t* def);

// Add sensor.
SIM_API SIM_sSensor* sim_spec_addSensor(sim_spec_t* s);

// Add flex.
SIM_API SIM_sFlex* sim_spec_addFlex(sim_spec_t* s);

// Add contact pair.
SIM_API SIM_sPair* sim_spec_addPair(sim_spec_t* s, const sim_spec_default_t* def);

// Add excluded body pair.
SIM_API SIM_sExclude* sim_spec_addExclude(sim_spec_t* s);

// Add equality.
SIM_API SIM_sEquality* sim_spec_addEquality(sim_spec_t* s, const sim_spec_default_t* def);

// Add tendon.
SIM_API SIM_sTendon* sim_spec_addTendon(sim_spec_t* s, const sim_spec_default_t* def);

// Wrap site using tendon.
SIM_API SIM_sWrap* sim_spec_wrapSite(SIM_sTendon* tendon, const char* name);

// Wrap geom using tendon.
SIM_API SIM_sWrap* sim_spec_wrapGeom(SIM_sTendon* tendon, const char* name, const char* sidesite);

// Wrap joint using tendon.
SIM_API SIM_sWrap* sim_spec_wrapJoint(SIM_sTendon* tendon, const char* name, double coef);

// Wrap pulley using tendon.
SIM_API SIM_sWrap* sim_spec_wrapPulley(SIM_sTendon* tendon, double divisor);

// Add numeric.
SIM_API SIM_sNumeric* sim_spec_addNumeric(sim_spec_t* s);

// Add text.
SIM_API SIM_sText* sim_spec_addText(sim_spec_t* s);

// Add tuple.
SIM_API SIM_sTuple* sim_spec_addTuple(sim_spec_t* s);

// Add keyframe.
SIM_API SIM_sKey* sim_spec_addKey(sim_spec_t* s);

// Add plugin.
SIM_API SIM_sPlugin* sim_spec_addPlugin(sim_spec_t* s);

// Add default.
SIM_API sim_spec_default_t* sim_spec_addDefault(sim_spec_t* s, const char* classname, const sim_spec_default_t* parent);


//---------------------------------- Set actuator parameters ---------------------------------------

// Set actuator to motor, return error on failure.
SIM_API const char* sim_spec_setToMotor(sim_spec_actuator_t* actuator);

// Set actuator to position, return error on failure.
SIM_API const char* sim_spec_setToPosition(sim_spec_actuator_t* actuator, double kp, double kv[1],
                                    double dampratio[1], double timeconst[1], double inheritrange);

// Set actuator to integrated velocity, return error on failure.
SIM_API const char* sim_spec_setToIntVelocity(sim_spec_actuator_t* actuator, double kp, double kv[1],
                                       double dampratio[1], double timeconst[1], double inheritrange);

// Set actuator to velocity, return error on failure.
SIM_API const char* sim_spec_setToVelocity(sim_spec_actuator_t* actuator, double kv);

// Set actuator to damper, return error on failure.
SIM_API const char* sim_spec_setToDamper(sim_spec_actuator_t* actuator, double kv);

// Set actuator to cylinder actuator, return error on failure.
SIM_API const char* sim_spec_setToCylinder(sim_spec_actuator_t* actuator, double timeconst,
                                    double bias, double area, double diameter);

// Set actuator to muscle, return error on failure.
SIM_API const char* sim_spec_setToMuscle(sim_spec_actuator_t* actuator, double timeconst[2], double tausmooth,
                                  double range[2], double force, double scale, double lmin,
                                  double lmax, double vmax, double fpmax, double fvmax);

// Set actuator to adhesion, return error on failure.
SIM_API const char* sim_spec_setToAdhesion(sim_spec_actuator_t* actuator, double gain);

//---------------------------------- Add assets ----------------------------------------------------

// Add mesh.
SIM_API SIM_sMesh* sim_spec_addMesh(sim_spec_t* s, const sim_spec_default_t* def);

// Add height field.
SIM_API SIM_sHField* sim_spec_addHField(sim_spec_t* s);

// Add skin.
SIM_API SIM_sSkin* sim_spec_addSkin(sim_spec_t* s);

// Add texture.
SIM_API SIM_sTexture* sim_spec_addTexture(sim_spec_t* s);

// Add material.
SIM_API SIM_sMaterial* sim_spec_addMaterial(sim_spec_t* s, const sim_spec_default_t* def);

// Sets the vertices and normals of a mesh.
SIM_API int sim_spec_makeMesh(SIM_sMesh* mesh, SIM_tMeshBuiltin builtin, double* params, int nparams);

//---------------------------------- Find/get utilities --------------------------------------------

// Get spec from body.
SIM_API sim_spec_t* sim_spec_getSpec(sim_spec_element_t* element);

// Find spec (model asset) by name.
SIM_API sim_spec_t* sim_spec_findSpec(sim_spec_t* spec, const char* name);

// Find body in spec by name.
SIM_API sim_spec_body_t* sim_spec_findBody(sim_spec_t* s, const char* name);

// Find element in spec by name.
SIM_API sim_spec_element_t* sim_spec_findElement(sim_spec_t* s, sim_obj_t type, const char* name);

// Find child body by name.
SIM_API sim_spec_body_t* sim_spec_findChild(sim_spec_body_t* body, const char* name);

// Get parent body.
SIM_API sim_spec_body_t* sim_spec_getParent(sim_spec_element_t* element);

// Get parent frame.
SIM_API SIM_sFrame* sim_spec_getFrame(sim_spec_element_t* element);

// Find frame by name.
SIM_API SIM_sFrame* sim_spec_findFrame(sim_spec_t* s, const char* name);

// Get default corresponding to an element.
SIM_API sim_spec_default_t* sim_spec_getDefault(sim_spec_element_t* element);

// Find default in model by class name.
SIM_API sim_spec_default_t* sim_spec_findDefault(sim_spec_t* s, const char* classname);

// Get global default from model.
SIM_API sim_spec_default_t* sim_spec_getSpecDefault(sim_spec_t* s);

// Get element id.
SIM_API int sim_spec_getId(sim_spec_element_t* element);


//---------------------------------- Tree traversal ------------------------------------------------

// Return body's first child of given type. If recurse is nonzero, also search the body's subtree.
SIM_API sim_spec_element_t* sim_spec_firstChild(sim_spec_body_t* body, sim_obj_t type, int recurse);

// Return body's next child of the same type; return NULL if child is last.
// If recurse is nonzero, also search the body's subtree.
SIM_API sim_spec_element_t* sim_spec_nextChild(sim_spec_body_t* body, sim_spec_element_t* child, int recurse);

// Return spec's first element of selected type.
SIM_API sim_spec_element_t* sim_spec_firstElement(sim_spec_t* s, sim_obj_t type);

// Return spec's next element; return NULL if element is last.
SIM_API sim_spec_element_t* sim_spec_nextElement(sim_spec_t* s, sim_spec_element_t* element);

// Get wrapped element in tendon path.
SIM_API sim_spec_element_t* sim_spec_getWrapTarget(SIM_sWrap* wrap);

// Get wrapped element in tendon path.
SIM_API SIM_sSite* sim_spec_getWrapSideSite(SIM_sWrap* wrap);

// Get divisor of SIM_sWrap wrapping a puller.
SIM_API double sim_spec_getWrapDivisor(SIM_sWrap* wrap);

// Get coefficient of SIM_sWrap wrapping a joint.
SIM_API double sim_spec_getWrapCoef(SIM_sWrap* wrap);

// Safely cast an element as sim_spec_body_t, or return NULL if the element is not an sim_spec_body_t.
SIM_API sim_spec_body_t* sim_spec_asBody(sim_spec_element_t* element);

// Safely cast an element as sim_spec_geom_t, or return NULL if the element is not an sim_spec_geom_t.
SIM_API sim_spec_geom_t* sim_spec_asGeom(sim_spec_element_t* element);

// Safely cast an element as SIM_sJoint, or return NULL if the element is not an SIM_sJoint.
SIM_API SIM_sJoint* sim_spec_asJoint(sim_spec_element_t* element);

// Safely cast an element as SIM_sSite, or return NULL if the element is not an SIM_sSite.
SIM_API SIM_sSite* sim_spec_asSite(sim_spec_element_t* element);

// Safely cast an element as SIM_sCamera, or return NULL if the element is not an SIM_sCamera.
SIM_API SIM_sCamera* sim_spec_asCamera(sim_spec_element_t* element);

// Safely cast an element as SIM_sLight, or return NULL if the element is not an SIM_sLight.
SIM_API SIM_sLight* sim_spec_asLight(sim_spec_element_t* element);

// Safely cast an element as SIM_sFrame, or return NULL if the element is not an SIM_sFrame.
SIM_API SIM_sFrame* sim_spec_asFrame(sim_spec_element_t* element);

// Safely cast an element as sim_spec_actuator_t, or return NULL if the element is not an sim_spec_actuator_t.
SIM_API sim_spec_actuator_t* sim_spec_asActuator(sim_spec_element_t* element);

// Safely cast an element as SIM_sSensor, or return NULL if the element is not an SIM_sSensor.
SIM_API SIM_sSensor* sim_spec_asSensor(sim_spec_element_t* element);

// Safely cast an element as SIM_sFlex, or return NULL if the element is not an SIM_sFlex.
SIM_API SIM_sFlex* sim_spec_asFlex(sim_spec_element_t* element);

// Safely cast an element as SIM_sPair, or return NULL if the element is not an SIM_sPair.
SIM_API SIM_sPair* sim_spec_asPair(sim_spec_element_t* element);

// Safely cast an element as SIM_sEquality, or return NULL if the element is not an SIM_sEquality.
SIM_API SIM_sEquality* sim_spec_asEquality(sim_spec_element_t* element);

// Safely cast an element as SIM_sExclude, or return NULL if the element is not an SIM_sExclude.
SIM_API SIM_sExclude* sim_spec_asExclude(sim_spec_element_t* element);

// Safely cast an element as SIM_sTendon, or return NULL if the element is not an SIM_sTendon.
SIM_API SIM_sTendon* sim_spec_asTendon(sim_spec_element_t* element);

// Safely cast an element as SIM_sNumeric, or return NULL if the element is not an SIM_sNumeric.
SIM_API SIM_sNumeric* sim_spec_asNumeric(sim_spec_element_t* element);

// Safely cast an element as SIM_sText, or return NULL if the element is not an SIM_sText.
SIM_API SIM_sText* sim_spec_asText(sim_spec_element_t* element);

// Safely cast an element as SIM_sTuple, or return NULL if the element is not an SIM_sTuple.
SIM_API SIM_sTuple* sim_spec_asTuple(sim_spec_element_t* element);

// Safely cast an element as SIM_sKey, or return NULL if the element is not an SIM_sKey.
SIM_API SIM_sKey* sim_spec_asKey(sim_spec_element_t* element);

// Safely cast an element as SIM_sMesh, or return NULL if the element is not an SIM_sMesh.
SIM_API SIM_sMesh* sim_spec_asMesh(sim_spec_element_t* element);

// Safely cast an element as SIM_sHField, or return NULL if the element is not an SIM_sHField.
SIM_API SIM_sHField* sim_spec_asHField(sim_spec_element_t* element);

// Safely cast an element as SIM_sSkin, or return NULL if the element is not an SIM_sSkin.
SIM_API SIM_sSkin* sim_spec_asSkin(sim_spec_element_t* element);

// Safely cast an element as SIM_sTexture, or return NULL if the element is not an SIM_sTexture.
SIM_API SIM_sTexture* sim_spec_asTexture(sim_spec_element_t* element);

// Safely cast an element as SIM_sMaterial, or return NULL if the element is not an SIM_sMaterial.
SIM_API SIM_sMaterial* sim_spec_asMaterial(sim_spec_element_t* element);

// Safely cast an element as SIM_sPlugin, or return NULL if the element is not an SIM_sPlugin.
SIM_API SIM_sPlugin* sim_spec_asPlugin(sim_spec_element_t* element);


//---------------------------------- Attribute setters ---------------------------------------------

// Set element's name, return 0 on success.
SIM_API int sim_spec_set_name(sim_spec_element_t* element, const char* name);

// Copy buffer.
SIM_API void sim_spec_setBuffer(SIM_ByteVec* dest, const void* array, int size);

// Copy text to string.
SIM_API void sim_spec_set_string(sim_string_t* dest, const char* text);

// Split text to entries and copy to string vector.
SIM_API void sim_spec_setStringVec(SIM_StringVec* dest, const char* text);

// Set entry in string vector.
SIM_API sim_byte_t sim_spec_setInStringVec(SIM_StringVec* dest, int i, const char* text);

// Append text entry to string vector.
SIM_API void sim_spec_appendString(SIM_StringVec* dest, const char* text);

// Copy int array to vector.
SIM_API void sim_spec_setInt(SIM_IntVec* dest, const int* array, int size);

// Append int array to vector of arrays.
SIM_API void sim_spec_appendIntVec(SIM_IntVecVec* dest, const int* array, int size);

// Copy float array to vector.
SIM_API void sim_spec_setFloat(SIM_FloatVec* dest, const float* array, int size);

// Append float array to vector of arrays.
SIM_API void sim_spec_appendFloatVec(SIM_FloatVecVec* dest, const float* array, int size);

// Copy double array to vector.
SIM_API void sim_spec_setDouble(SIM_DoubleVec* dest, const double* array, int size);

// Set plugin attributes.
SIM_API void sim_spec_setPluginAttributes(SIM_sPlugin* plugin, void* attributes);


//---------------------------------- Attribute getters ---------------------------------------------

// Get element's name.
SIM_API sim_string_t* sim_spec_getName(sim_spec_element_t* element);

// Get string contents.
SIM_API const char* sim_spec_getString(const sim_string_t* source);

// Get double array contents and optionally its size.
SIM_API const double* sim_spec_getDouble(const SIM_DoubleVec* source, int* size);

// Get number of elements a tendon wraps.
SIM_API int sim_spec_getWrapNum(const SIM_sTendon* tendonspec);

SIM_API SIM_sWrap* sim_spec_getWrap(const SIM_sTendon* tendonspec, int i);

// Get plugin attributes.
SIM_API const void* sim_spec_getPluginAttributes(const SIM_sPlugin* plugin);


//---------------------------------- Other utilities -----------------------------------------------

// Set element's default.
SIM_API void sim_spec_setDefault(sim_spec_element_t* element, const sim_spec_default_t* def);

// Set element's enclosing frame, return 0 on success.
SIM_API int sim_spec_setFrame(sim_spec_element_t* dest, SIM_sFrame* frame);

// Resolve alternative orientations to quat, return error if any.
SIM_API const char* sim_spec_resolveOrientation(double quat[4], sim_byte_t degree, const char* sequence,
                                         const SIM_sOrientation* orientation);

// Transform body into a frame.
SIM_API SIM_sFrame* sim_spec_bodyToFrame(sim_spec_body_t** body);

// Set user payload.
SIM_API void sim_spec_setUserValue(sim_spec_element_t* element, const char* key, const void* data);

// Set user payload.
SIM_API void sim_spec_setUserValueWithCleanup(sim_spec_element_t* element, const char* key,
                                       const void* data,
                                       void (*cleanup)(const void*));

// Return user payload or NULL if none found.
SIM_API const void* sim_spec_getUserValue(sim_spec_element_t* element, const char* key);

// Delete user payload.
SIM_API void sim_spec_deleteUserValue(sim_spec_element_t* element, const char* key);

// Return sensor dimension.
SIM_API int sim_spec_sensorDim(const SIM_sSensor* sensor);

//---------------------------------- Initialization  -----------------------------------------------

// Default spec attributes.
SIM_API void sim_spec_defaultSpec(sim_spec_t* spec);

// Default orientation attributes.
SIM_API void sim_spec_defaultOrientation(SIM_sOrientation* orient);

// Default body attributes.
SIM_API void sim_spec_defaultBody(sim_spec_body_t* body);

// Default frame attributes.
SIM_API void sim_spec_defaultFrame(SIM_sFrame* frame);

// Default joint attributes.
SIM_API void sim_spec_defaultJoint(SIM_sJoint* joint);

// Default geom attributes.
SIM_API void sim_spec_defaultGeom(sim_spec_geom_t* geom);

// Default site attributes.
SIM_API void sim_spec_defaultSite(SIM_sSite* site);

// Default camera attributes.
SIM_API void sim_spec_defaultCamera(SIM_sCamera* camera);

// Default light attributes.
SIM_API void sim_spec_defaultLight(SIM_sLight* light);

// Default flex attributes.
SIM_API void sim_spec_defaultFlex(SIM_sFlex* flex);

// Default mesh attributes.
SIM_API void sim_spec_defaultMesh(SIM_sMesh* mesh);

// Default height field attributes.
SIM_API void sim_spec_defaultHField(SIM_sHField* hfield);

// Default skin attributes.
SIM_API void sim_spec_defaultSkin(SIM_sSkin* skin);

// Default texture attributes.
SIM_API void sim_spec_defaultTexture(SIM_sTexture* texture);

// Default material attributes.
SIM_API void sim_spec_defaultMaterial(SIM_sMaterial* material);

// Default pair attributes.
SIM_API void sim_spec_defaultPair(SIM_sPair* pair);

// Default equality attributes.
SIM_API void sim_spec_defaultEquality(SIM_sEquality* equality);

// Default tendon attributes.
SIM_API void sim_spec_defaultTendon(SIM_sTendon* tendon);

// Default actuator attributes.
SIM_API void sim_spec_defaultActuator(sim_spec_actuator_t* actuator);

// Default sensor attributes.
SIM_API void sim_spec_defaultSensor(SIM_sSensor* sensor);

// Default numeric attributes.
SIM_API void sim_spec_defaultNumeric(SIM_sNumeric* numeric);

// Default text attributes.
SIM_API void sim_spec_defaultText(SIM_sText* text);

// Default tuple attributes.
SIM_API void sim_spec_defaultTuple(SIM_sTuple* tuple);

// Default keyframe attributes.
SIM_API void sim_spec_defaultKey(SIM_sKey* key);

// Default plugin attributes.
SIM_API void sim_spec_defaultPlugin(SIM_sPlugin* plugin);


//---------------------------------- Compiler cache ------------------------------------------------

// Get the capacity of the asset cache in bytes.
SIM_API size_t sim_getCacheCapacity(const SIM_Cache* cache);

// Set the capacity of the asset cache in bytes (0 to disable); returns the new capacity.
SIM_API size_t sim_setCacheCapacity(SIM_Cache* cache, size_t size);

// Get the current size of the asset cache in bytes.
SIM_API size_t sim_getCacheSize(const SIM_Cache* cache);

// Clear the asset cache.
SIM_API void sim_clearCache(SIM_Cache* cache);

// Get the internal asset cache used by the compiler.
SIM_API SIM_Cache* sim_getCache(void);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // SIMCORE_SRC_USER_USER_API_H_
