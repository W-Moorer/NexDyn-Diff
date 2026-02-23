// Copyright 2023 DeepMind Technologies Limited
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

#include "engine/engine_util_container.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <simcore/core_api.h>
#include "engine/engine_crossplatform.h"
#include "engine/engine_memory.h"

// stack allocate and initialize new SIM_ArrayList
SIM_ArrayList* sim_math_arrayListCreate(sim_data_t* d, size_t element_size, size_t initial_capacity) {
  SIM_ArrayList* array_list = SIM_STACK_ALLOC(d, 1, SIM_ArrayList);
  initial_capacity = SIM_MAX(1, initial_capacity);
  array_list->d = d;
  array_list->element_size = element_size;
  array_list->capacity = initial_capacity;
  array_list->size = 0;
  array_list->next_segment = NULL;

  // allocate array list buffer
  array_list->buffer = (void*) sim_stackAllocByte(
    d, element_size * initial_capacity, _Alignof(SIM_tMaxAlign));
  return array_list;
}


// returns total number of elements in SIM_ArrayList
size_t sim_math_arrayListSize(const SIM_ArrayList* array_list) {
  const SIM_ArrayList* cursor = array_list;
  size_t array_list_size = 0;
  while (cursor) {
    array_list_size += cursor->size;
    cursor = cursor->next_segment;
  }
  return array_list_size;
}


// copies one element into an SIM_ArrayList
void sim_math_arrayListAdd(SIM_ArrayList* array_list, void* element) {
  SIM_ArrayList* cursor = array_list;

  // find a non full segment or add a new segment
  while (cursor->size == cursor->capacity) {
    if (cursor->next_segment == NULL) {
      // add a new segment with twice the capacity of the last segment
      cursor->next_segment = sim_math_arrayListCreate(
        cursor->d, cursor->element_size, 2 * cursor->capacity);
    }
    cursor = cursor->next_segment;
  }
  // copy element into segment
  memcpy((sim_byte_t*)cursor->buffer + cursor->element_size * cursor->size,
         element, cursor->element_size);
  ++cursor->size;
}


// returns pointer to element at index, NULL if out of bounds
void* sim_math_arrayListAt(const SIM_ArrayList* array_list, size_t index) {
  // if the index is larger than the current capacity, then it is in a later segment
  const SIM_ArrayList* cursor = array_list;
  size_t total_capacity = 0;
  while (cursor != NULL && index >= total_capacity + cursor->capacity) {
    total_capacity += cursor->capacity;
    cursor = cursor->next_segment;
  }

  if (!cursor) {
    return NULL;
  }

  if (index - total_capacity >= cursor->size) {
    return NULL;
  }

  return (sim_byte_t*)cursor->buffer + (cursor->element_size * (index - total_capacity));
}

