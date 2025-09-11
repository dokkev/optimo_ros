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


#ifndef OPTIMO_CONTROLLER_SRC_CONTROLLER_FREEMOTIONCALLBACK_H_
#define OPTIMO_CONTROLLER_SRC_CONTROLLER_FREEMOTIONCALLBACK_H_

#include <rl/controller/AbstractCallback.h>

namespace optimo
{
/**
 * @brief Callback that can be used to start free motion.
 *
 */
class FreeMotionCallback : public roboligent::AbstractCallback
{
public:
  /**
   * @brief Construct a new Free Motion Callback object
   *
   * @param model_
   */
  explicit FreeMotionCallback(roboligent::Model & model_);

  /**
   * @brief Destroy the Free Motion Callback object
   *
   */
  virtual ~FreeMotionCallback() = default;

  /**
   * @brief Does nothing
   *
   * @param torque_
   */
  void calculate_torque(std::vector<int> & torque_) override;
};
}  // namespace optimo

#endif
