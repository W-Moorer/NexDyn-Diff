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

#include "thread/thread_task.h"

#include <thread>

#include <simcore/SIM_thread.h>

namespace simcore {
void sim_math_defaultTask(SIM_Task* task) {
  task->func = nullptr;
  task->args = nullptr;
  task->status = SIM_TASK_NEW;
}

void sim_math_taskJoin(SIM_Task* task) {
  while (GetAtomicTaskStatus(task) != SIM_TASK_COMPLETED) {
    std::this_thread::yield();
  }
}
}  // namespace simcore
