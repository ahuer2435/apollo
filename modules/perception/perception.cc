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

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"
#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/obstacle/base/object.h"
#include "modules/perception/obstacle/onboard/lidar_process.h"
#include "modules/perception/perception.h"
#include "ros/include/ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

namespace apollo {
namespace perception {

using apollo::common::adapter::AdapterManager;
using apollo::common::Status;
using apollo::common::ErrorCode;

//ros node name
std::string Perception::Name() const { return "perception"; }

Status Perception::Init() {
  //读取配置文件，创建node handle和topic，参考文档：adapter设计架构源码分析。
  AdapterManager::Init(FLAGS_adapter_config_filename);

  //启动激光雷达进程
  lidar_process_.reset(new LidarProcess());     //reset()函数是智能指针unique_ptr的属性，通过此使得指针指向对象LidarProcess的实例。
  if (lidar_process_ != nullptr && !lidar_process_->Init()) {     //lidar_process_->Init配置对雷达数据处理的算法
    AERROR << "failed to init lidar_process.";
    return Status(ErrorCode::PERCEPTION_ERROR, "failed to init lidar_process.");
  }

  //检测是否获取到点云数据（from激光雷达）
  CHECK(AdapterManager::GetPointCloud()) << "PointCloud is not initialized.";
  //注册回调函数OnPointCloud
  AdapterManager::AddPointCloudCallback(&Perception::OnPointCloud, this);
  return Status::OK();
}

/*
* 收到点云数据，也就是激光雷达的数据执行此函数。
*/
void Perception::OnPointCloud(const sensor_msgs::PointCloud2& message) {
  ADEBUG << "get point cloud callback";

  if (lidar_process_ != nullptr && lidar_process_->IsInit()) {
    lidar_process_->Process(message);       //处理激光雷达数据，使用roi_filter，segmentor，object builder，tracker算法处理激光雷达数据。

    /// public obstacle message
    PerceptionObstacles obstacles;
    if (lidar_process_->GeneratePbMsg(&obstacles)) {            //生成障碍物数据
      AdapterManager::PublishPerceptionObstacles(obstacles);    //发布障碍物数据
    }
  }
}

Status Perception::Start() { return Status::OK(); }

void Perception::Stop() {}

}  // namespace perception
}  // namespace apollo
