/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/prediction/container/container_manager.h"

#include "modules/common/log.h"
#include "modules/prediction/container/obstacles/obstacles_container.h"
#include "modules/prediction/container/pose/pose_container.h"

namespace apollo {
namespace prediction {

using apollo::common::adapter::AdapterConfig;
using apollo::common::adapter::AdapterManagerConfig;

ContainerManager::ContainerManager() {}

void ContainerManager::Init(const AdapterManagerConfig& config) {
  config_.CopyFrom(config);
  RegisterContainers();
}

//adapter.conf文件中的话题类型是接受话题，就用对应的type注册一个容器。
void ContainerManager::RegisterContainers() {
  for (const auto& adapter_config : config_.config()) {
    if (adapter_config.has_type() &&
        (adapter_config.mode() == AdapterConfig::RECEIVE_ONLY ||
         adapter_config.mode() == AdapterConfig::DUPLEX)) {
      RegisterContainer(adapter_config.type());
    }
  }
}

Container* ContainerManager::GetContainer(
    const common::adapter::AdapterConfig::MessageType& type) {
  if (containers_.find(type) != containers_.end()) {
    return containers_[type].get();
  } else {
    return nullptr;
  }
}

//容器类型：ObstaclesContainer和PoseContainer 类。
std::unique_ptr<Container> ContainerManager::CreateContainer(
    const common::adapter::AdapterConfig::MessageType& type) {
  std::unique_ptr<Container> container_ptr(nullptr);
  if (type == AdapterConfig::PERCEPTION_OBSTACLES) {
    container_ptr.reset(new ObstaclesContainer());
  } else if (type == AdapterConfig::LOCALIZATION) {
    container_ptr.reset(new PoseContainer());
  }
  return container_ptr;
}

//注册容器，在这里就是增加containers_[]数组中的一项。
void ContainerManager::RegisterContainer(
    const common::adapter::AdapterConfig::MessageType& type) {
  containers_[type] = CreateContainer(type);
  AINFO << "Container [" << type << "] is registered.";
}

}  // namespace prediction
}  // namespace apollo
