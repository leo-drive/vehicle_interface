// Copyright 2022 Leo Drive Teknoloji A.Ş.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/// \copyright Copyright 2022 Leo Drive Teknoloji A.Ş.


#ifndef LEO_VCU_DRIVER__VISIBILITY_CONTROL_HPP_
#define LEO_VCU_DRIVER__VISIBILITY_CONTROL_HPP_

////////////////////////////////////////////////////////////////////////////////
#if defined(__WIN32)
  #if defined(LEO_VCU_DRIVER_BUILDING_DLL) || defined(LEO_VCU_DRIVER_EXPORTS)
    #define LEO_VCU_DRIVER_PUBLIC __declspec(dllexport)
    #define LEO_VCU_DRIVER_LOCAL
  #else  // defined(LEO_VCU_DRIVER_BUILDING_DLL) || defined(LEO_VCU_DRIVER_EXPORTS)
    #define LEO_VCU_DRIVER_PUBLIC __declspec(dllimport)
    #define LEO_VCU_DRIVER_LOCAL
  #endif  // defined(LEO_VCU_DRIVER_BUILDING_DLL) || defined(LEO_VCU_DRIVER_EXPORTS)
#elif defined(__linux__)
  #define LEO_VCU_DRIVER_PUBLIC __attribute__((visibility("default")))
  #define LEO_VCU_DRIVER_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
  #define LEO_VCU_DRIVER_PUBLIC __attribute__((visibility("default")))
  #define LEO_VCU_DRIVER_LOCAL __attribute__((visibility("hidden")))
#else
  #error "Unsupported Build Configuration"
#endif

#endif  // LEO_VCU_DRIVER__VISIBILITY_CONTROL_HPP_
