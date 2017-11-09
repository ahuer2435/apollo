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

#include "modules/prediction/prediction.h"

#include "modules/prediction/proto/prediction_obstacle.pb.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/util/file.h"
#include "modules/prediction/common/prediction_gflags.h"
#include "modules/prediction/container/container_manager.h"
#include "modules/prediction/container/obstacles/obstacles_container.h"
#include "modules/prediction/container/pose/pose_container.h"
#include "modules/prediction/evaluator/evaluator_manager.h"
#include "modules/prediction/predictor/predictor_manager.h"

namespace apollo {
namespace prediction {

using apollo::perception::PerceptionObstacles;
using apollo::perception::PerceptionObstacle;
using apollo::localization::LocalizationEstimate;
using apollo::common::adapter::AdapterManager;
using apollo::common::adapter::AdapterConfig;
using apollo::common::Status;
using apollo::common::ErrorCode;

//FLAGS_prediction_xxxx宏在prediction/common/prediction_gflag.c中定义
std::string Prediction::Name() const { return FLAGS_prediction_module_name; }

//定义好节点与话题。
Status Prediction::Init() {
  // Load prediction conf
  prediction_conf_.Clear();
  //FLAGS_prediction_conf_file宏为："modules/prediction/conf/prediction_conf.pb.txt"
  //其中规定的是preditor的产生的通道类型
  if (!common::util::GetProtoFromFile(FLAGS_prediction_conf_file,
                                      &prediction_conf_)) {
    return OnError("Unable to load prediction conf file: " +
                   FLAGS_prediction_conf_file);
  } else {
    ADEBUG << "Prediction config file is loaded into: "
           << prediction_conf_.ShortDebugString();
  }

  //FLAGS_adapter_config_filename宏为：modules/prediction/conf/adapter.conf
  //加载prediction adapter，设置定义话题。
  adapter_conf_.Clear();
  if (!common::util::GetProtoFromFile(FLAGS_adapter_config_filename,
                                      &adapter_conf_)) {
    return OnError("Unable to load adapter conf file: " +
                   FLAGS_adapter_config_filename);
  } else {
    ADEBUG << "Adapter config file is loaded into: "
           << adapter_conf_.ShortDebugString();
  }

  // Initialization of all managers
  AdapterManager::instance()->Init(adapter_conf_);
  ContainerManager::instance()->Init(adapter_conf_);
  EvaluatorManager::instance()->Init(prediction_conf_);
  PredictorManager::instance()->Init(prediction_conf_);

  CHECK(AdapterManager::GetLocalization()) << "Localization is not ready.";
  CHECK(AdapterManager::GetPerceptionObstacles()) << "Perception is not ready.";

  //设置perception obstacle 回调函数为OnPerception
  // Set perception obstacle callback function
  AdapterManager::AddPerceptionObstaclesCallback(&Prediction::OnPerception,
                                                 this);
  //设置localization 回调函数为OnLocalization
  // Set localization callback function
  AdapterManager::AddLocalizationCallback(&Prediction::OnLocalization, this);

  return Status::OK();
}

Status Prediction::Start() { return Status::OK(); }

void Prediction::Stop() {}

void Prediction::OnLocalization(const LocalizationEstimate& localization) {
  //获取障碍物容器
  ObstaclesContainer* obstacles_container = dynamic_cast<ObstaclesContainer*>(
      ContainerManager::instance()->GetContainer(
          AdapterConfig::PERCEPTION_OBSTACLES));
  CHECK_NOTNULL(obstacles_container);

  //获取位置容器
  PoseContainer* pose_container = dynamic_cast<PoseContainer*>(
      ContainerManager::instance()->GetContainer(AdapterConfig::LOCALIZATION));
  CHECK_NOTNULL(pose_container);


  pose_container->Insert(localization);     //将定位信息插入位置容器。

  //将位置信息融合到障碍物容器中。
  PerceptionObstacle* pose_ptr = pose_container->ToPerceptionObstacle();    //重点pose_container->ToPerceptionObstacle()函数。暂放。
  if (pose_ptr != nullptr) {
    obstacles_container->InsertPerceptionObstacle(                          //重点obstacles_container->InsertPerceptionObstacle函数，暂放。
        *(pose_ptr), pose_container->GetTimestamp());
  } else {
    ADEBUG << "Invalid pose found.";
  }

  ADEBUG << "Received a localization message ["
         << localization.ShortDebugString() << "].";
}

void Prediction::OnPerception(const PerceptionObstacles& perception_obstacles) {
  //获取障碍物容器。
  ObstaclesContainer* obstacles_container = dynamic_cast<ObstaclesContainer*>(
      ContainerManager::instance()->GetContainer(
          AdapterConfig::PERCEPTION_OBSTACLES));
  CHECK_NOTNULL(obstacles_container);
  //将障碍物信息加入到障碍物容器。
  obstacles_container->Insert(perception_obstacles);

  EvaluatorManager::instance()->Run(perception_obstacles);  //运行评估器。
  PredictorManager::instance()->Run(perception_obstacles);  //运行预测器。

  //填充发布消息内容，感觉应该用一个新的变量，而不是直接修改形参prediction_obstacles。
  auto prediction_obstacles =
      PredictorManager::instance()->prediction_obstacles();
  AdapterManager::FillPredictionHeader(Name(), &prediction_obstacles);
  AdapterManager::PublishPrediction(prediction_obstacles);
  ADEBUG << "Published a prediction message ["
         << prediction_obstacles.ShortDebugString() << "].";
}

Status Prediction::OnError(const std::string& error_msg) {
  return Status(ErrorCode::PREDICTION_ERROR, error_msg);
}

}  // namespace prediction
}  // namespace apollo
