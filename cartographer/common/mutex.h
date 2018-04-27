/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CARTOGRAPHER_COMMON_MUTEX_H_
#define CARTOGRAPHER_COMMON_MUTEX_H_

#include <condition_variable>
#include <mutex>

#include "absl/synchronization/mutex.h"
#include "cartographer/common/time.h"

namespace cartographer {
namespace common {

// Enable thread safety attributes only with clang.
// The attributes can be safely erased when compiling with other compilers.
#if defined(__SUPPORT_TS_ANNOTATION__) || defined(__clang__)
#define THREAD_ANNOTATION_ATTRIBUTE__(x) __attribute__((x))
#else
#define THREAD_ANNOTATION_ATTRIBUTE__(x)  // no-op
#endif

#define CAPABILITY(x) THREAD_ANNOTATION_ATTRIBUTE__(capability(x))

#define SCOPED_CAPABILITY THREAD_ANNOTATION_ATTRIBUTE__(scoped_lockable)

#define GUARDED_BY(x) THREAD_ANNOTATION_ATTRIBUTE__(guarded_by(x))

#define PT_GUARDED_BY(x) THREAD_ANNOTATION_ATTRIBUTE__(pt_guarded_by(x))

#define REQUIRES(...) \
  THREAD_ANNOTATION_ATTRIBUTE__(requires_capability(__VA_ARGS__))

#define ACQUIRE(...) \
  THREAD_ANNOTATION_ATTRIBUTE__(acquire_capability(__VA_ARGS__))

#define RELEASE(...) \
  THREAD_ANNOTATION_ATTRIBUTE__(release_capability(__VA_ARGS__))

#define EXCLUDES(...) THREAD_ANNOTATION_ATTRIBUTE__(locks_excluded(__VA_ARGS__))

#define NO_THREAD_SAFETY_ANALYSIS \
  THREAD_ANNOTATION_ATTRIBUTE__(no_thread_safety_analysis)

// Defines an annotated mutex that can only be locked through its scoped locker
// implementation.
class CAPABILITY("mutex") Mutex {
 public:
  // A RAII class that acquires a mutex in its constructor, and
  // releases it in its destructor. It also implements waiting functionality on
  // conditions that get checked whenever the mutex is released.
  class SCOPED_CAPABILITY Locker {
   public:
    Locker(Mutex* mutex) ACQUIRE(mutex)
        : mutex_(mutex), lock_(&mutex->mutex_) {}

    ~Locker() RELEASE() {}

    template <typename Predicate>
    void Await(Predicate predicate) REQUIRES(this) {
      mutex_->mutex_.Await(::absl::Condition(&predicate));
    }

    template <typename Predicate>
    bool AwaitWithTimeout(Predicate predicate, common::Duration timeout)
        REQUIRES(this) {
      return mutex_->mutex_.AwaitWithTimeout(::absl::Condition(&predicate),
                                             ::absl::FromChrono(timeout));
    }

   private:
    Mutex* mutex_;
    ::absl::MutexLock lock_;
  };

 private:
  ::absl::Mutex mutex_;
};

using MutexLocker = Mutex::Locker;

}  // namespace common
}  // namespace cartographer

#endif  // CARTOGRAPHER_COMMON_MUTEX_H_
