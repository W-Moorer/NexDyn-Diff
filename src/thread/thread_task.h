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

#ifndef SIMCORE_SRC_THREAD_THREAD_TASK_H_
#define SIMCORE_SRC_THREAD_THREAD_TASK_H_

#include <simcore/SIM_export.h>
#include <simcore/SIM_thread.h>

#ifdef __cplusplus
#include <atomic>
#include <new>
#include <type_traits>
namespace simcore {
extern "C" {
#endif

// Initialize an SIM_Task.
SIM_API void sim_math_defaultTask(SIM_Task* task);

// Wait for a task to complete.
SIM_API void sim_math_taskJoin(SIM_Task* task);

#ifdef __cplusplus
}  // extern "C"

using TaskStatus = std::remove_volatile_t<decltype(SIM_Task::status)>;
inline std::atomic<TaskStatus>& GetAtomicTaskStatus(SIM_Task* task) {
  static_assert(sizeof(std::atomic<TaskStatus>) == sizeof(TaskStatus));
  static_assert(alignof(std::atomic<TaskStatus>) == alignof(TaskStatus));
  static_assert(std::atomic<TaskStatus>::is_always_lock_free);
  return *std::launder(reinterpret_cast<std::atomic<TaskStatus>*>(
      const_cast<TaskStatus*>(&task->status)));
}
}  // namespace simcore
#endif  // __cplusplus

#endif  // SIMCORE_SRC_THREAD_THREAD_TASK_H_
