/*
 * Copyright 2018 The Cartographer Authors
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

#include <cmath>
#include <fstream>
#include <string>

#include "cartographer/common/port.h"
#include "cartographer/io/file_writer.h"
#include "cartographer/io/image.h"
#include "cartographer/io/proto_stream.h"
#include "cartographer/io/submap_painter.h"
#include "cartographer/mapping/internal/2d/pose_graph_2d.h"
#include "cartographer/mapping/internal/3d/pose_graph_3d.h"
#include "cartographer/mapping/internal/test_helpers.h"
#include "cartographer/mapping/map_builder.h"
#include "cartographer/mapping/pose_graph.h"
#include "cartographer/mapping/proto/pose_graph.pb.h"
#include "cartographer/mapping/proto/trajectory_builder_options.pb.h"
#include "cartographer/transform/transform.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

DEFINE_bool(use_3d, false, "Use 3D pipeline (default is 2D).");
DEFINE_string(pose_graph_filenames, "",
              "Comma-separated list of pbstream files to read.");
DEFINE_bool(skip_optimization, false, "Skip all optimization.");

namespace cartographer {
namespace mapping {
namespace {

std::vector<std::string> SplitString(const std::string& input,
                                     const char delimiter) {
  std::istringstream stream(input);
  std::string token;
  std::vector<std::string> tokens;
  while (std::getline(stream, token, delimiter)) {
    tokens.push_back(token);
  }
  return tokens;
}

class EmptyOptimizationProblem2D : public pose_graph::OptimizationProblem2D {
 public:
  EmptyOptimizationProblem2D()
      : OptimizationProblem2D(pose_graph::proto::OptimizationProblemOptions{}) {
  }
  ~EmptyOptimizationProblem2D() override = default;
  void Solve(
      const std::vector<Constraint>& constraints,
      const std::set<int>& frozen_trajectories,
      const std::map<std::string, LandmarkNode>& landmark_nodes) override {}
};

class EmptyOptimizationProblem3D : public pose_graph::OptimizationProblem3D {
 public:
  EmptyOptimizationProblem3D()
      : OptimizationProblem3D(pose_graph::proto::OptimizationProblemOptions{}) {
  }
  ~EmptyOptimizationProblem3D() override = default;
  void Solve(
      const std::vector<Constraint>& constraints,
      const std::set<int>& frozen_trajectories,
      const std::map<std::string, LandmarkNode>& landmark_nodes) override {}
};

void AddProtoStreamToPoseGraph(io::ProtoStreamReaderInterface* const reader,
                               mapping::PoseGraph* pose_graph,
                               int* next_trajectory_id) {
  // TODO(gaschler): Reuse this function with MapBuilder.
  proto::PoseGraph pose_graph_proto;
  CHECK(reader->ReadProto(&pose_graph_proto));
  proto::AllTrajectoryBuilderOptions all_builder_options_proto;
  CHECK(reader->ReadProto(&all_builder_options_proto));
  bool is_pose_graph_3d = dynamic_cast<PoseGraph3D*>(pose_graph) != nullptr;
  CHECK(all_builder_options_proto.options_with_sensor_ids_size() > 0);
  auto& o = all_builder_options_proto.options_with_sensor_ids(0);
  CHECK(o.has_trajectory_builder_options());
  if (is_pose_graph_3d) {
    CHECK(o.trajectory_builder_options().has_trajectory_builder_3d_options());
  } else {
    CHECK(o.trajectory_builder_options().has_trajectory_builder_2d_options());
  }
  CHECK_EQ(pose_graph_proto.trajectory_size(),
           all_builder_options_proto.options_with_sensor_ids_size());

  std::map<int, int> trajectory_remapping;
  for (auto& trajectory_proto : *pose_graph_proto.mutable_trajectory()) {
    // TODO: Collect these options for later output.
    const auto& options_with_sensor_ids_proto =
        all_builder_options_proto.options_with_sensor_ids(
            trajectory_proto.trajectory_id());
    CHECK(trajectory_remapping
              .emplace(trajectory_proto.trajectory_id(), *next_trajectory_id)
              .second)
        << "Duplicate trajectory ID: " << trajectory_proto.trajectory_id();
    trajectory_proto.set_trajectory_id(*next_trajectory_id);
    *next_trajectory_id += 1;
  }

  // Apply the calculated remapping to constraints in the pose graph proto.
  for (auto& constraint_proto : *pose_graph_proto.mutable_constraint()) {
    constraint_proto.mutable_submap_id()->set_trajectory_id(
        trajectory_remapping.at(constraint_proto.submap_id().trajectory_id()));
    constraint_proto.mutable_node_id()->set_trajectory_id(
        trajectory_remapping.at(constraint_proto.node_id().trajectory_id()));
  }

  MapById<SubmapId, transform::Rigid3d> submap_poses;
  for (const proto::Trajectory& trajectory_proto :
       pose_graph_proto.trajectory()) {
    for (const proto::Trajectory::Submap& submap_proto :
         trajectory_proto.submap()) {
      submap_poses.Insert(SubmapId{trajectory_proto.trajectory_id(),
                                   submap_proto.submap_index()},
                          transform::ToRigid3(submap_proto.pose()));
    }
  }

  MapById<NodeId, transform::Rigid3d> node_poses;
  for (const proto::Trajectory& trajectory_proto :
       pose_graph_proto.trajectory()) {
    for (const proto::Trajectory::Node& node_proto : trajectory_proto.node()) {
      node_poses.Insert(
          NodeId{trajectory_proto.trajectory_id(), node_proto.node_index()},
          transform::ToRigid3(node_proto.pose()));
    }
  }

  for (;;) {
    proto::SerializedData proto;
    if (!reader->ReadProto(&proto)) {
      break;
    }

    if (proto.has_node()) {
      proto.mutable_node()->mutable_node_id()->set_trajectory_id(
          trajectory_remapping.at(proto.node().node_id().trajectory_id()));
      const transform::Rigid3d node_pose =
          node_poses.at(NodeId{proto.node().node_id().trajectory_id(),
                               proto.node().node_id().node_index()});
      pose_graph->AddNodeFromProto(node_pose, proto.node());
    }
    if (proto.has_submap()) {
      if (is_pose_graph_3d) {
        CHECK(proto.submap().has_submap_3d());
      } else {
        CHECK(proto.submap().has_submap_2d());
      }
      proto.mutable_submap()->mutable_submap_id()->set_trajectory_id(
          trajectory_remapping.at(proto.submap().submap_id().trajectory_id()));
      const transform::Rigid3d submap_pose =
          submap_poses.at(SubmapId{proto.submap().submap_id().trajectory_id(),
                                   proto.submap().submap_id().submap_index()});
      pose_graph->AddSubmapFromProto(submap_pose, proto.submap());
    }
    if (proto.has_trajectory_data()) {
      proto.mutable_trajectory_data()->set_trajectory_id(
          trajectory_remapping.at(proto.trajectory_data().trajectory_id()));
      pose_graph->SetTrajectoryDataFromProto(proto.trajectory_data());
    }
    if (proto.has_imu_data()) {
      pose_graph->AddImuData(
          trajectory_remapping.at(proto.imu_data().trajectory_id()),
          sensor::FromProto(proto.imu_data().imu_data()));
    }
    if (proto.has_odometry_data()) {
      pose_graph->AddOdometryData(
          trajectory_remapping.at(proto.odometry_data().trajectory_id()),
          sensor::FromProto(proto.odometry_data().odometry_data()));
    }
    if (proto.has_fixed_frame_pose_data()) {
      pose_graph->AddFixedFramePoseData(
          trajectory_remapping.at(
              proto.fixed_frame_pose_data().trajectory_id()),
          sensor::FromProto(
              proto.fixed_frame_pose_data().fixed_frame_pose_data()));
    }
    if (proto.has_landmark_data()) {
      pose_graph->AddLandmarkData(
          trajectory_remapping.at(proto.landmark_data().trajectory_id()),
          sensor::FromProto(proto.landmark_data().landmark_data()));
    }
  }

  pose_graph->AddSerializedConstraints(
      FromProto(pose_graph_proto.constraint()));
  CHECK(reader->eof());
}

void WritePng(PoseGraph* pose_graph, io::StreamFileWriter* png_writer) {
  LOG(INFO) << "Loading submap slices from serialized data.";
  std::map<SubmapId, io::SubmapSlice> submap_slices;
  auto all_submap_data = pose_graph->GetAllSubmapData();
  for (const auto& submap_data : all_submap_data) {
    proto::Submap submap_proto;
    submap_data.data.submap->ToProto(&submap_proto, true);
    FillSubmapSlice(submap_data.data.pose, submap_proto,
                    &submap_slices[submap_data.id]);
  }
  LOG(INFO) << "Generating combined map image from submap slices.";
  // TODO(gaschler): Read from flag.
  double resolution = 0.05;
  auto result = io::PaintSubmapSlices(submap_slices, resolution);
  io::Image image(std::move(result.surface));
  image.WritePng(png_writer);
  LOG(INFO) << "Wrote image to " << png_writer->GetFilename();
}

void Run(bool use_3d, const std::string& pose_graph_filenames,
         bool skip_optimization) {
  auto filenames = SplitString(pose_graph_filenames, ',');

  // TODO: Read options from flag rather than using default.
  auto thread_pool = common::make_unique<common::ThreadPool>(16);
  const std::string kPoseGraphLua = R"text(
      include "pose_graph.lua"
      return POSE_GRAPH)text";
  auto pose_graph_parameters = test::ResolveLuaParameters(kPoseGraphLua);
  auto pose_graph_options = CreatePoseGraphOptions(pose_graph_parameters.get());
  // Check if this can be dropped.
  pose_graph_options.mutable_optimization_problem_options()
      ->set_log_solver_summary(true);

  std::unique_ptr<mapping::PoseGraph> pose_graph;
  if (use_3d) {
    std::unique_ptr<pose_graph::OptimizationProblem3D> optimization_problem =
        (skip_optimization)
            ? common::make_unique<EmptyOptimizationProblem3D>()
            : common::make_unique<pose_graph::OptimizationProblem3D>(
                  pose_graph_options.optimization_problem_options());
    pose_graph = common::make_unique<PoseGraph3D>(
        pose_graph_options, std::move(optimization_problem), thread_pool.get());
  } else {
    std::unique_ptr<pose_graph::OptimizationProblem2D> optimization_problem =
        (skip_optimization)
            ? common::make_unique<EmptyOptimizationProblem2D>()
            : common::make_unique<pose_graph::OptimizationProblem2D>(
                  pose_graph_options.optimization_problem_options());
    pose_graph = common::make_unique<PoseGraph2D>(
        pose_graph_options, std::move(optimization_problem), thread_pool.get());
  }

  int next_trajectory_id = 0;
  for (const std::string& pose_graph_filename : filenames) {
    LOG(INFO) << "Reading pose graph from '" << pose_graph_filename << "'...";
    io::ProtoStreamReader reader(pose_graph_filename);
    AddProtoStreamToPoseGraph(&reader, pose_graph.get(), &next_trajectory_id);
  }
  pose_graph->RunFinalOptimization();
  CHECK(!pose_graph->IsTrajectoryFrozen(0));

  pose_graph->FindInterTrajectoryConstraints();
  pose_graph->RunFinalOptimization();
  {
    io::StreamFileWriter writer("merged.png");
    WritePng(pose_graph.get(), &writer);
  }
  {
    std::string output_filename = "merged.pbstream";
    io::ProtoStreamWriter writer(output_filename);
    std::vector<proto::TrajectoryBuilderOptionsWithSensorIds>
        all_trajectory_builder_options;
    // TODO(gaschler): Collect all_trajectory_builder_options.
    MapBuilder::SerializeState(all_trajectory_builder_options, pose_graph.get(),
                               &writer);
    LOG(INFO) << "Wrote merged pbstream " << output_filename;
  }
}

}  // namespace
}  // namespace mapping
}  // namespace cartographer

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = true;
  google::SetUsageMessage(
      "\n\n"
      "This program merge multiple pose graph pbstream files.\n");
  google::ParseCommandLineFlags(&argc, &argv, true);
  if (FLAGS_pose_graph_filenames.empty()) {
    google::ShowUsageWithFlags(argv[0]);
    return EXIT_FAILURE;
  }
  ::cartographer::mapping::Run(FLAGS_use_3d, FLAGS_pose_graph_filenames,
                               FLAGS_skip_optimization);
}
