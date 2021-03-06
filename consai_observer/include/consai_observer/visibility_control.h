// Copyright 2021 Roots
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

#ifndef CONSAI_OBSERVER__VISIBILITY_CONTROL_H_
#define CONSAI_OBSERVER__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define CONSAI_OBSERVER_EXPORT __attribute__ ((dllexport))
    #define CONSAI_OBSERVER_IMPORT __attribute__ ((dllimport))
  #else
    #define CONSAI_OBSERVER_EXPORT __declspec(dllexport)
    #define CONSAI_OBSERVER_IMPORT __declspec(dllimport)
  #endif
  #ifdef CONSAI_OBSERVER_BUILDING_DLL
    #define CONSAI_OBSERVER_PUBLIC CONSAI_VISION_TRACKER_EXPORT
  #else
    #define CONSAI_OBSERVER_PUBLIC CONSAI_VISION_TRACKER_IMPORT
  #endif
  #define CONSAI_OBSERVER_PUBLIC_TYPE CONSAI_VISION_TRACKER_PUBLIC
  #define CONSAI_OBSERVER_LOCAL
#else
  #define CONSAI_OBSERVER_EXPORT __attribute__ ((visibility("default")))
  #define CONSAI_OBSERVER_IMPORT
  #if __GNUC__ >= 4
    #define CONSAI_OBSERVER_PUBLIC __attribute__ ((visibility("default")))
    #define CONSAI_OBSERVER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define CONSAI_OBSERVER_PUBLIC
    #define CONSAI_OBSERVER_LOCAL
  #endif
  #define CONSAI_OBSERVER_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // CONSAI_OBSERVER__VISIBILITY_CONTROL_H_
