/*
 * Copyright 2017 The Cartographer Authors
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

#include "cartographer/mapping/map_builder.h"

#include "cartographer/common/config.h"
#include "cartographer/io/proto_stream.h"
#include "cartographer/mapping/internal/testing/test_helpers.h"
#include "gtest/gtest.h"
#include "gmock/gmock.h"

namespace cartographer {
namespace mapping {
namespace {

using SensorId = cartographer::mapping::TrajectoryBuilderInterface::SensorId;

const SensorId kRangeSensorId{SensorId::SensorType::RANGE, "range"};
const SensorId kIMUSensorId{SensorId::SensorType::IMU, "imu"};
constexpr double kDuration = 4.;         // Seconds.
constexpr double kTimeStep = 0.1;        // Seconds.
constexpr double kTravelDistance = 1.2;  // Meters.

class MapBuilderTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Global SLAM optimization is not executed.
    const std::string kMapBuilderLua = R"text(
      include "map_builder.lua"
      MAP_BUILDER.use_trajectory_builder_2d = true
      MAP_BUILDER.pose_graph.optimize_every_n_nodes = 0
      MAP_BUILDER.pose_graph.global_sampling_ratio = 0.05
      MAP_BUILDER.pose_graph.global_constraint_search_after_n_seconds = 0
      return MAP_BUILDER)text";
    auto map_builder_parameters = test::ResolveLuaParameters(kMapBuilderLua);
    map_builder_options_ =
        CreateMapBuilderOptions(map_builder_parameters.get());
    // Multiple submaps are created because of a small 'num_range_data'.
    const std::string kTrajectoryBuilderLua = R"text(
      include "trajectory_builder.lua"
      TRAJECTORY_BUILDER.trajectory_builder_2d.use_imu_data = false
      TRAJECTORY_BUILDER.trajectory_builder_2d.submaps.num_range_data = 4
      TRAJECTORY_BUILDER.trajectory_builder_3d.submaps.num_range_data = 4
      return TRAJECTORY_BUILDER)text";
    auto trajectory_builder_parameters =
        test::ResolveLuaParameters(kTrajectoryBuilderLua);
    trajectory_builder_options_ =
        CreateTrajectoryBuilderOptions(trajectory_builder_parameters.get());
  }

  void BuildMapBuilder() {
    map_builder_ = common::make_unique<MapBuilder>(map_builder_options_);
  }

  void SetOptionsTo3D() {
    map_builder_options_.set_use_trajectory_builder_2d(false);
    map_builder_options_.set_use_trajectory_builder_3d(true);
  }

  void SetOptionsEnableGlobalOptimization() {
    map_builder_options_.mutable_pose_graph_options()
        ->set_optimize_every_n_nodes(3);
    trajectory_builder_options_.mutable_trajectory_builder_2d_options()
        ->mutable_motion_filter_options()
        ->set_max_distance_meters(0);
  }

  MapBuilderInterface::LocalSlamResultCallback GetLocalSlamResultCallback() {
    return [=](const int trajectory_id, const ::cartographer::common::Time time,
               const ::cartographer::transform::Rigid3d local_pose,
               ::cartographer::sensor::RangeData range_data_in_local,
               const std::unique_ptr<
                   const cartographer::mapping::TrajectoryBuilderInterface::
                       InsertionResult>) {
      local_slam_result_poses_.push_back(local_pose);
    };
  }

  int CreateTrajectoryWithFakeData(double timestamps_add_duration = 0.) {
    int trajectory_id = map_builder_->AddTrajectoryBuilder(
        {kRangeSensorId}, trajectory_builder_options_,
        GetLocalSlamResultCallback());
    TrajectoryBuilderInterface* trajectory_builder =
        map_builder_->GetTrajectoryBuilder(trajectory_id);
    auto measurements = test::GenerateFakeRangeMeasurements(
        kTravelDistance, kDuration, kTimeStep);
    for (auto& measurement : measurements) {
      measurement.time += common::FromSeconds(timestamps_add_duration);
      trajectory_builder->AddSensorData(kRangeSensorId.id, measurement);
    }
    map_builder_->FinishTrajectory(trajectory_id);
    return trajectory_id;
  }

  std::unique_ptr<MapBuilderInterface> map_builder_;
  proto::MapBuilderOptions map_builder_options_;
  proto::TrajectoryBuilderOptions trajectory_builder_options_;
  std::vector<::cartographer::transform::Rigid3d> local_slam_result_poses_;
};

TEST_F(MapBuilderTest, TrajectoryAddFinish2D) {
  BuildMapBuilder();
  int trajectory_id = map_builder_->AddTrajectoryBuilder(
      {kRangeSensorId}, trajectory_builder_options_,
      nullptr /* GetLocalSlamResultCallbackForSubscriptions */);
  EXPECT_EQ(1, map_builder_->num_trajectory_builders());
  EXPECT_TRUE(map_builder_->GetTrajectoryBuilder(trajectory_id) != nullptr);
  EXPECT_TRUE(map_builder_->pose_graph() != nullptr);
  map_builder_->FinishTrajectory(trajectory_id);
  map_builder_->pose_graph()->RunFinalOptimization();
  EXPECT_TRUE(map_builder_->pose_graph()->IsTrajectoryFinished(trajectory_id));
}

TEST_F(MapBuilderTest, TrajectoryAddFinish3D) {
  SetOptionsTo3D();
  BuildMapBuilder();
  int trajectory_id = map_builder_->AddTrajectoryBuilder(
      {kRangeSensorId}, trajectory_builder_options_,
      nullptr /* GetLocalSlamResultCallbackForSubscriptions */);
  EXPECT_EQ(1, map_builder_->num_trajectory_builders());
  EXPECT_TRUE(map_builder_->GetTrajectoryBuilder(trajectory_id) != nullptr);
  EXPECT_TRUE(map_builder_->pose_graph() != nullptr);
  map_builder_->FinishTrajectory(trajectory_id);
  map_builder_->pose_graph()->RunFinalOptimization();
  EXPECT_TRUE(map_builder_->pose_graph()->IsTrajectoryFinished(trajectory_id));
}

TEST_F(MapBuilderTest, LocalSlam2D) {
  BuildMapBuilder();
  int trajectory_id = map_builder_->AddTrajectoryBuilder(
      {kRangeSensorId}, trajectory_builder_options_,
      GetLocalSlamResultCallback());
  TrajectoryBuilderInterface* trajectory_builder =
      map_builder_->GetTrajectoryBuilder(trajectory_id);
  const auto measurements = test::GenerateFakeRangeMeasurements(
      kTravelDistance, kDuration, kTimeStep);
  for (const auto& measurement : measurements) {
    trajectory_builder->AddSensorData(kRangeSensorId.id, measurement);
  }
  map_builder_->FinishTrajectory(trajectory_id);
  map_builder_->pose_graph()->RunFinalOptimization();
  EXPECT_EQ(local_slam_result_poses_.size(), measurements.size());
  EXPECT_NEAR(kTravelDistance,
              (local_slam_result_poses_.back().translation() -
               local_slam_result_poses_.front().translation())
                  .norm(),
              0.1 * kTravelDistance);
}

TEST_F(MapBuilderTest, LocalSlam3D) {
  SetOptionsTo3D();
  BuildMapBuilder();
  int trajectory_id = map_builder_->AddTrajectoryBuilder(
      {kRangeSensorId, kIMUSensorId}, trajectory_builder_options_,
      GetLocalSlamResultCallback());
  TrajectoryBuilderInterface* trajectory_builder =
      map_builder_->GetTrajectoryBuilder(trajectory_id);
  const auto measurements = test::GenerateFakeRangeMeasurements(
      kTravelDistance, kDuration, kTimeStep);
  for (const auto& measurement : measurements) {
    trajectory_builder->AddSensorData(kRangeSensorId.id, measurement);
    trajectory_builder->AddSensorData(
        kIMUSensorId.id,
        sensor::ImuData{measurement.time, Eigen::Vector3d(0., 0., 9.8),
                        Eigen::Vector3d::Zero()});
  }
  map_builder_->FinishTrajectory(trajectory_id);
  map_builder_->pose_graph()->RunFinalOptimization();
  EXPECT_EQ(local_slam_result_poses_.size(), measurements.size());
  EXPECT_NEAR(kTravelDistance,
              (local_slam_result_poses_.back().translation() -
               local_slam_result_poses_.front().translation())
                  .norm(),
              0.1 * kTravelDistance);
}

TEST_F(MapBuilderTest, GlobalSlam2D) {
  SetOptionsEnableGlobalOptimization();
  BuildMapBuilder();
  int trajectory_id = map_builder_->AddTrajectoryBuilder(
      {kRangeSensorId}, trajectory_builder_options_,
      GetLocalSlamResultCallback());
  TrajectoryBuilderInterface* trajectory_builder =
      map_builder_->GetTrajectoryBuilder(trajectory_id);
  const auto measurements = test::GenerateFakeRangeMeasurements(
      kTravelDistance, kDuration, kTimeStep);
  for (const auto& measurement : measurements) {
    trajectory_builder->AddSensorData(kRangeSensorId.id, measurement);
  }
  map_builder_->FinishTrajectory(trajectory_id);
  map_builder_->pose_graph()->RunFinalOptimization();
  EXPECT_EQ(local_slam_result_poses_.size(), measurements.size());
  EXPECT_NEAR(kTravelDistance,
              (local_slam_result_poses_.back().translation() -
               local_slam_result_poses_.front().translation())
                  .norm(),
              0.1 * kTravelDistance);
  EXPECT_GE(map_builder_->pose_graph()->constraints().size(), 50);
  EXPECT_THAT(map_builder_->pose_graph()->constraints(),
              testing::Contains(testing::Field(
                  &PoseGraphInterface::Constraint::tag,
                  PoseGraphInterface::Constraint::INTER_SUBMAP)));
  const auto trajectory_nodes =
      map_builder_->pose_graph()->GetTrajectoryNodes();
  EXPECT_GE(trajectory_nodes.SizeOfTrajectoryOrZero(trajectory_id), 20);
  const auto submap_data = map_builder_->pose_graph()->GetAllSubmapData();
  EXPECT_GE(submap_data.SizeOfTrajectoryOrZero(trajectory_id), 5);
  const transform::Rigid3d final_pose =
      map_builder_->pose_graph()->GetLocalToGlobalTransform(trajectory_id) *
      local_slam_result_poses_.back();
  EXPECT_NEAR(kTravelDistance, final_pose.translation().norm(),
              0.1 * kTravelDistance);
}

TEST_F(MapBuilderTest, GlobalSlam3D) {
  SetOptionsTo3D();
  SetOptionsEnableGlobalOptimization();
  BuildMapBuilder();
  int trajectory_id = map_builder_->AddTrajectoryBuilder(
      {kRangeSensorId, kIMUSensorId}, trajectory_builder_options_,
      GetLocalSlamResultCallback());
  TrajectoryBuilderInterface* trajectory_builder =
      map_builder_->GetTrajectoryBuilder(trajectory_id);
  const auto measurements = test::GenerateFakeRangeMeasurements(
      kTravelDistance, kDuration, kTimeStep);
  for (const auto& measurement : measurements) {
    trajectory_builder->AddSensorData(kRangeSensorId.id, measurement);
    trajectory_builder->AddSensorData(
        kIMUSensorId.id,
        sensor::ImuData{measurement.time, Eigen::Vector3d(0., 0., 9.8),
                        Eigen::Vector3d::Zero()});
  }
  map_builder_->FinishTrajectory(trajectory_id);
  map_builder_->pose_graph()->RunFinalOptimization();
  EXPECT_EQ(local_slam_result_poses_.size(), measurements.size());
  EXPECT_NEAR(kTravelDistance,
              (local_slam_result_poses_.back().translation() -
               local_slam_result_poses_.front().translation())
                  .norm(),
              0.1 * kTravelDistance);
  EXPECT_GE(map_builder_->pose_graph()->constraints().size(), 10);
  EXPECT_THAT(map_builder_->pose_graph()->constraints(),
              testing::Contains(testing::Field(
                  &PoseGraphInterface::Constraint::tag,
                  PoseGraphInterface::Constraint::INTER_SUBMAP)));
  const auto trajectory_nodes =
      map_builder_->pose_graph()->GetTrajectoryNodes();
  EXPECT_GE(trajectory_nodes.SizeOfTrajectoryOrZero(trajectory_id), 5);
  const auto submap_data = map_builder_->pose_graph()->GetAllSubmapData();
  EXPECT_GE(submap_data.SizeOfTrajectoryOrZero(trajectory_id), 2);
  const transform::Rigid3d final_pose =
      map_builder_->pose_graph()->GetLocalToGlobalTransform(trajectory_id) *
      local_slam_result_poses_.back();
  EXPECT_NEAR(kTravelDistance, final_pose.translation().norm(),
              0.1 * kTravelDistance);
}

TEST_F(MapBuilderTest, DeleteFinishedTrajectory2D) {
  SetOptionsEnableGlobalOptimization();
  BuildMapBuilder();
  int trajectory_id = CreateTrajectoryWithFakeData();
  map_builder_->pose_graph()->RunFinalOptimization();
  EXPECT_GE(map_builder_->pose_graph()->constraints().size(), 50);
  EXPECT_GE(
      map_builder_->pose_graph()->GetTrajectoryNodes().SizeOfTrajectoryOrZero(
          trajectory_id),
      20);
  EXPECT_GE(
      map_builder_->pose_graph()->GetAllSubmapData().SizeOfTrajectoryOrZero(
          trajectory_id),
      5);
  map_builder_->pose_graph()->DeleteTrajectory(trajectory_id);
  int another_trajectory_id = CreateTrajectoryWithFakeData(100.);
  map_builder_->pose_graph()->RunFinalOptimization();
  EXPECT_EQ(
      map_builder_->pose_graph()->GetTrajectoryNodes().SizeOfTrajectoryOrZero(
          trajectory_id),
      0);
  EXPECT_EQ(
      map_builder_->pose_graph()->GetAllSubmapData().SizeOfTrajectoryOrZero(
          trajectory_id),
      0);
  map_builder_->pose_graph()->DeleteTrajectory(another_trajectory_id);
  map_builder_->pose_graph()->RunFinalOptimization();
  EXPECT_EQ(map_builder_->pose_graph()->constraints().size(), 0);
  EXPECT_EQ(
      map_builder_->pose_graph()->GetTrajectoryNodes().SizeOfTrajectoryOrZero(
          another_trajectory_id),
      0);
  EXPECT_EQ(
      map_builder_->pose_graph()->GetAllSubmapData().SizeOfTrajectoryOrZero(
          another_trajectory_id),
      0);
}

TEST_F(MapBuilderTest, SaveLoadState2D) {
  BuildMapBuilder();
  int trajectory_id = CreateTrajectoryWithFakeData();
  map_builder_->pose_graph()->RunFinalOptimization();
  int num_constraints = map_builder_->pose_graph()->constraints().size();
  int num_nodes =
      map_builder_->pose_graph()->GetTrajectoryNodes().SizeOfTrajectoryOrZero(
          trajectory_id);
  EXPECT_GT(num_constraints, 0);
  EXPECT_GT(num_nodes, 0);
  // TODO(gaschler): Consider using in-memory to avoid side effects.
  const std::string filename = "temp-SaveLoadState2D.pbstream";
  io::ProtoStreamWriter writer(filename);
  map_builder_->SerializeState(&writer);
  writer.Close();

  // Reset MapBuilder.
  BuildMapBuilder();
  io::ProtoStreamReader reader(filename);
  map_builder_->LoadState(&reader, false /* load_frozen_state */);
  // TODO(gaschler): Design better way to find out which new trajectory_ids were
  // created by LoadState.
  map_builder_->pose_graph()->RunFinalOptimization();
  auto nodes = map_builder_->pose_graph()->GetTrajectoryNodes();
  ASSERT_GT(nodes.size(), 0);
  int new_trajectory_id = nodes.begin()->id.trajectory_id;
  EXPECT_EQ(num_constraints, map_builder_->pose_graph()->constraints().size());
  EXPECT_EQ(
      num_nodes,
      map_builder_->pose_graph()->GetTrajectoryNodes().SizeOfTrajectoryOrZero(
          new_trajectory_id));
}

TEST_F(MapBuilderTest, LocalizationOnFrozenTrajectory2D) {
  BuildMapBuilder();
  int temp_trajectory_id = CreateTrajectoryWithFakeData();
  map_builder_->pose_graph()->RunFinalOptimization();
  EXPECT_GT(map_builder_->pose_graph()->constraints().size(), 0);
  EXPECT_GT(
      map_builder_->pose_graph()->GetTrajectoryNodes().SizeOfTrajectoryOrZero(
          temp_trajectory_id),
      0);
  const std::string filename = "temp-LocalizationOnFrozenTrajectory2D.pbstream";
  io::ProtoStreamWriter writer(filename);
  map_builder_->SerializeState(&writer);
  writer.Close();

  // Reset MapBuilder.
  local_slam_result_poses_.clear();
  SetOptionsEnableGlobalOptimization();
  BuildMapBuilder();
  io::ProtoStreamReader reader(filename);
  map_builder_->LoadState(&reader, true /* load_frozen_state */);
  map_builder_->pose_graph()->RunFinalOptimization();
  // TODO(gaschler): Consider loading while SLAMing.
  int trajectory_id = map_builder_->AddTrajectoryBuilder(
      {kRangeSensorId}, trajectory_builder_options_,
      GetLocalSlamResultCallback());
  TrajectoryBuilderInterface* trajectory_builder =
      map_builder_->GetTrajectoryBuilder(trajectory_id);
  // TODO(gaschler): Add some yaw rotation.
  // transform::Rigid3d frozen_trajectory_to_global(Eigen::Vector3d(2.0, 1.0, 0), rotation);
  Eigen::Quaterniond rotation(
      Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitZ()));
  transform::Rigid3d frozen_trajectory_to_global =
      transform::Rigid3d::Translation(Eigen::Vector3d(2.0, 1.0, 0));
  auto measurements = test::GenerateFakeRangeMeasurements(kTravelDistance,
                                                          kDuration, kTimeStep);
  double delay_measurements = 100.;
  for (auto& measurement : measurements) {
    measurement.time += common::FromSeconds(delay_measurements);
    measurement.ranges = sensor::TransformTimedPointCloud(
        measurement.ranges,
        frozen_trajectory_to_global.cast<float>().inverse());
    trajectory_builder->AddSensorData(kRangeSensorId.id, measurement);
  }
  map_builder_->FinishTrajectory(trajectory_id);
  map_builder_->pose_graph()->RunFinalOptimization();
  EXPECT_EQ(local_slam_result_poses_.size(), measurements.size());
  EXPECT_NEAR(kTravelDistance,
              (local_slam_result_poses_.back().translation() -
               local_slam_result_poses_.front().translation())
                  .norm(),
              0.1 * kTravelDistance);
  EXPECT_GE(map_builder_->pose_graph()->constraints().size(), 50);
  auto constraints = map_builder_->pose_graph()->constraints();
  int num_cross_trajectory_constraints = 0;
  for (const auto& constraint : constraints) {
    if (constraint.node_id.trajectory_id !=
        constraint.submap_id.trajectory_id) {
      num_cross_trajectory_constraints++;
    }
  }
  EXPECT_GT(num_cross_trajectory_constraints, 0);
  // TODO(gaschler): Subscribe global slam callback, verify that all nodes are
  // optimized.
  EXPECT_THAT(constraints, testing::Contains(testing::Field(
                               &PoseGraphInterface::Constraint::tag,
                               PoseGraphInterface::Constraint::INTER_SUBMAP)));
  const auto trajectory_nodes =
      map_builder_->pose_graph()->GetTrajectoryNodes();
  EXPECT_GE(trajectory_nodes.SizeOfTrajectoryOrZero(trajectory_id), 20);
  const auto submap_data = map_builder_->pose_graph()->GetAllSubmapData();
  EXPECT_GE(submap_data.SizeOfTrajectoryOrZero(trajectory_id), 5);
  const transform::Rigid3d global_pose =
      map_builder_->pose_graph()->GetLocalToGlobalTransform(trajectory_id) *
      local_slam_result_poses_.back();
  EXPECT_GT(map_builder_->pose_graph()
                ->GetLocalToGlobalTransform(trajectory_id)
                .translation()
                .norm(),
            1.0);
  Eigen::Vector3d travel_translation =
      Eigen::Vector3d(2., 1., 0.).normalized() * kTravelDistance;
  const transform::Rigid3d expected_global_pose =
      frozen_trajectory_to_global *
      transform::Rigid3d::Translation(travel_translation);
  EXPECT_NEAR(
      0.,
      (global_pose.translation() - expected_global_pose.translation()).norm(),
      0.2) << "global_pose: " << global_pose << "expected_global_pose: " << expected_global_pose;
  LOG(INFO) << "num_cross_trajectory_constraints: "
            << num_cross_trajectory_constraints;
}

}  // namespace
}  // namespace mapping
}  // namespace cartographer
