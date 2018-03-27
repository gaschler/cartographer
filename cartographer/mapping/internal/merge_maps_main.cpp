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
#include "cartographer/mapping/pose_graph.h"
#include "cartographer/mapping/proto/pose_graph.pb.h"
#include "cartographer/mapping/proto/trajectory_builder_options.pb.h"
#include "cartographer/transform/transform.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

DEFINE_string(pose_graph_filenames, "",
              "Comma-separated list of pbstream files to read.");
DEFINE_bool(run_optimization, false,
            "Run optimization on the entire pose graph.");

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

bool Is3D(const proto::AllTrajectoryBuilderOptions& options, bool* success) {
  if (options.options_with_sensor_ids_size() == 0 ||
      !options.options_with_sensor_ids(0).has_trajectory_builder_options()) {
    *success = false;
    return false;
  }
  const auto& o =
      options.options_with_sensor_ids(0).trajectory_builder_options();
  if (o.has_trajectory_builder_2d_options() &&
      !o.has_trajectory_builder_3d_options()) {
    *success = true;
    return false;
  }
  if (!o.has_trajectory_builder_2d_options() &&
      o.has_trajectory_builder_3d_options()) {
    *success = true;
    return true;
  }
  *success = false;
  return false;
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

std::unique_ptr<mapping::PoseGraph> LoadProtoStream(
    io::ProtoStreamReaderInterface* const reader,
    common::ThreadPool* thread_pool) {
  proto::PoseGraph pose_graph_proto;
  CHECK(reader->ReadProto(&pose_graph_proto));
  proto::AllTrajectoryBuilderOptions all_builder_options_proto;
  CHECK(reader->ReadProto(&all_builder_options_proto));
  bool success;
  bool is_3d = Is3D(all_builder_options_proto, &success);
  if (!success) {
    LOG(WARNING)
        << "Could not determine if 2D or 3D from TrajectoryBuilderOptions.";
    return nullptr;
  }
  std::unique_ptr<mapping::PoseGraph> pose_graph;
  proto::PoseGraphOptions pose_graph_options;
  if (is_3d) {
    // TODO(gaschler): Follow run_optimization flag.
    pose_graph = common::make_unique<PoseGraph3D>(
        pose_graph_options, common::make_unique<EmptyOptimizationProblem3D>(),
        thread_pool);
  } else {
    pose_graph = common::make_unique<PoseGraph2D>(
        pose_graph_options, common::make_unique<EmptyOptimizationProblem2D>(),
        thread_pool);
  }
  CHECK_EQ(pose_graph_proto.trajectory_size(),
           all_builder_options_proto.options_with_sensor_ids_size());

  std::map<int, int> trajectory_remapping;
  int new_trajectory_id = 0;
  for (auto& trajectory_proto : *pose_graph_proto.mutable_trajectory()) {
    const auto& options_with_sensor_ids_proto =
        all_builder_options_proto.options_with_sensor_ids(
            trajectory_proto.trajectory_id());
    CHECK(trajectory_remapping
              .emplace(trajectory_proto.trajectory_id(), new_trajectory_id)
              .second)
        << "Duplicate trajectory ID: " << trajectory_proto.trajectory_id();
    trajectory_proto.set_trajectory_id(new_trajectory_id);
    new_trajectory_id++;
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
  }

  // pose_graph->AddSerializedConstraints(
  //    FromProto(pose_graph_proto.constraint()));
  CHECK(reader->eof());
  return pose_graph;
}

void WritePng(PoseGraph* pose_graph, io::StreamFileWriter* png_writer) {
  LOG(INFO) << "Loading submap slices from serialized data.";
  std::map<SubmapId, io::SubmapSlice> submap_slices;
  auto all_submap_data = pose_graph->GetAllSubmapData();
  for (const auto& submap_data : all_submap_data) {
    proto::Submap submap_proto;
    submap_data.data.submap->ToProto(&submap_proto, true);
#if 0
    CHECK(submap_proto.has_submap_2d());
    CHECK(submap_proto.submap_2d().num_range_data() > 0);
    CHECK(submap_proto.submap_2d().probability_grid().cells_size() > 0);
#endif
    FillSubmapSlice(submap_data.data.pose, submap_proto,
                    &submap_slices[submap_data.id]);
  }
  LOG(INFO) << "Generating combined map image from submap slices.";
  double resolution = 0.1;
  auto result = io::PaintSubmapSlices(submap_slices, resolution);
  io::Image image(std::move(result.surface));
  image.WritePng(png_writer);
  LOG(INFO) << "Wrote image to " << png_writer->GetFilename();
}

void Run(const std::string& pose_graph_filenames, bool run_optimization) {
  auto filenames = SplitString(pose_graph_filenames, ',');
  // TODO(gaschler): Read other pose graphs.
  std::string pose_graph_filename = filenames[0];
  LOG(INFO) << "Reading pose graph from '" << pose_graph_filename << "'...";
  auto thread_pool = common::make_unique<common::ThreadPool>(1);
  std::unique_ptr<mapping::PoseGraph> pose_graph;
  {
    io::ProtoStreamReader reader(pose_graph_filename);
    pose_graph = LoadProtoStream(&reader, thread_pool.get());
  }
  if (!pose_graph) {
    LOG(FATAL) << "Failed to read pose graph.";
  }
  pose_graph->RunFinalOptimization();

  io::StreamFileWriter writer("output.png");
  WritePng(pose_graph.get(), &writer);
  // TODO(gaschler): Save pose graph.
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
  ::cartographer::mapping::Run(FLAGS_pose_graph_filenames,
                               FLAGS_run_optimization);
}
