// 신규 — 26.09.25 _ YSH _ morai_actuate --lockstep 진입점 선언 (구현은 actuate_lockstep.cpp)
//
// Copyright 2026 The Autoware Contributors
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

#ifndef UNIVERSE__AUTOWARE_UNIVERSE__SIMULATOR__MORAI_GRPC_VENDOR__TOOLS__ACTUATE_LOCKSTEP_HPP_
#define UNIVERSE__AUTOWARE_UNIVERSE__SIMULATOR__MORAI_GRPC_VENDOR__TOOLS__ACTUATE_LOCKSTEP_HPP_

#include "actuate_options.hpp"

namespace morai_grpc_vendor::actuate
{

/// morai_actuate --lockstep (options already parsed: confirmed && lockstep). Installs its
/// own signal handlers, runs preflight -> SYNCHRONOUS -> patterns -> restore, returns the
/// exit code (same table as the real-time mode; 6 = restore failed, MORAI may be frozen).
int run_lockstep(const Options & opt);

}  // namespace morai_grpc_vendor::actuate

#endif  // UNIVERSE__AUTOWARE_UNIVERSE__SIMULATOR__MORAI_GRPC_VENDOR__TOOLS__ACTUATE_LOCKSTEP_HPP_
