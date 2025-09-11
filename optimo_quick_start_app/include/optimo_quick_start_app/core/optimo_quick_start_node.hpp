// Copyright 2024 Roboligent, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef OPTIMO_QUICK_START_APP__CORE__OPTIMO_QUICK_START_NODE_HPP_
#define OPTIMO_QUICK_START_APP__CORE__OPTIMO_QUICK_START_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

/** @brief Optimo Quick Start Application */
namespace optimo_quick_start_app
{

/** @brief Optimo Quick Start Application Node */
class OptimoQuickStartNode : public rclcpp::Node
{
public:
  /** @brief Construct a new Optimo Quick Start Node. */
  OptimoQuickStartNode();

  /** @brief Default Destructor */
  virtual ~OptimoQuickStartNode() = default;
};

}  // namespace optimo_quick_start_app

#endif  // OPTIMO_QUICK_START_APP__CORE__OPTIMO_QUICK_START_NODE_HPP_
