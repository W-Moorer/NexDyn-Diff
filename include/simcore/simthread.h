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

#ifndef SIMCORE_INCLUDE_SIM_THREAD_H_
#define SIMCORE_INCLUDE_SIM_THREAD_H_

#define SIM_MAXTHREAD 128        // maximum number of threads in a thread pool

typedef enum SIM_tTaskStatus_ {  // status values for SIM_Task
  SIM_TASK_NEW = 0,              // newly created
  SIM_TASK_QUEUED,               // enqueued in a thread pool
  SIM_TASK_COMPLETED             // completed execution
} SIM_tTaskStatus;

// function pointer type for SIM_Task
typedef void* (*SIM_fTask)(void*);

// An opaque type representing a thread pool.
struct SIM_ThreadPool_ {
  int nworker;  // number of workers in the pool
};
typedef struct SIM_ThreadPool_ SIM_ThreadPool;

struct SIM_Task_ {        // a task that can be executed by a thread pool.
  SIM_fTask func;         // pointer to the function that implements the task
  void* args;           // arguments to func
  volatile int status;  // status of the task
};
typedef struct SIM_Task_ SIM_Task;

#endif  // SIMCORE_INCLUDE_SIM_THREAD_H_
