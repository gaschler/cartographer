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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_RANGE_DATA_COLLATOR_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_RANGE_DATA_COLLATOR_H_

#include <memory>

#include "cartographer/mapping/2d/pose_graph_2d.h"
#include "cartographer/mapping/3d/pose_graph_3d.h"
#include "cartographer/mapping/internal/2d/local_trajectory_builder_2d.h"
#include "cartographer/mapping/internal/3d/local_trajectory_builder_3d.h"
#include "cartographer/mapping/local_slam_result_data.h"
#include "cartographer/mapping/trajectory_builder_interface.h"

namespace cartographer {
namespace mapping {

// Sends out a collection of time-cropped range data when enough data is
// buffered. There is enough buffer when we have either one of each sensor or
// two of the same sensor.
class RangeDataCollator {
 public:
  explicit RangeDataCollator(
      const std::vector<std::string>& expected_range_sensor_ids)
      : num_sensors_(expected_range_sensor_ids.size()) {}

  std::vector<std::unique_ptr<sensor::TimedPointCloudData>> AddSensorData(
      const std::string& sensor_id,
      std::unique_ptr<sensor::TimedPointCloudData> timed_point_cloud_data) {
    CHECK(timed_point_cloud_data);
    if (id_to_pending_data_.count(sensor_id) != 0) {
      current_start_ = current_end_;
      // When we have two messages of the same sensor, move forward the older of
      // the two (do not send out current).
      current_end_ = id_to_pending_data_.at(sensor_id)->time;
      std::vector<std::unique_ptr<sensor::TimedPointCloudData>> result =
          SelectAndMerge();
      id_to_pending_data_.emplace(sensor_id, std::move(timed_point_cloud_data));
      return result;
    }
    id_to_pending_data_.emplace(sensor_id, std::move(timed_point_cloud_data));
    if (num_sensors_ == id_to_pending_data_.size()) {
      current_start_ = current_end_;
      // We have messages from all sensors, move forward to oldest.
      common::Time oldest_timestamp = common::Time::max();
      for (const auto& pair : id_to_pending_data_) {
        oldest_timestamp = std::min(oldest_timestamp, pair.second->time);
      }
      return SelectAndMerge();
    }
    return {};
    // TODO(gaschler): Check actual sensor_id string, test-cover.
  }

 private:
  std::vector<std::unique_ptr<sensor::TimedPointCloudData>> SelectAndMerge() {
    std::vector<std::unique_ptr<sensor::TimedPointCloudData>> result;
    for (auto& pair : id_to_pending_data_) {
      common::Time& in_timestamp = pair.second->time;
      sensor::TimedPointCloud& ranges = pair.second->ranges;

      sensor::TimedPointCloud::iterator overlap_begin = ranges.begin();
      while (overlap_begin < ranges.end() &&
             in_timestamp + common::FromSeconds((*overlap_begin)[3]) <
                 current_start_) {
        ++overlap_begin;
      }
      sensor::TimedPointCloud::iterator overlap_end = overlap_begin;
      while (overlap_end < ranges.end() &&
             in_timestamp + common::FromSeconds((*overlap_end)[3]) <
                 current_end_) {
        ++overlap_end;
      }

      // Select range from 'current_start_' to 'current_end_'.
      if (overlap_begin < overlap_end) {
        double time_correction = common::ToSeconds(current_end_ - in_timestamp);
        auto selection = common::make_unique<sensor::TimedPointCloudData>(
            sensor::TimedPointCloudData{
                current_end_, pair.second->origin,
                sensor::TimedPointCloud(overlap_begin, overlap_end)});
        for (Eigen::Vector4f& point : selection->ranges) {
          point[3] += time_correction;
        }
        result.emplace_back(std::move(selection));
      }

      if (overlap_end < ranges.end()) {
        // TODO
      }
    }
    return result;
  }

  const int num_sensors_;

  // Store at most one message for each sensor.
  std::map<std::string, std::unique_ptr<sensor::TimedPointCloudData>>
      id_to_pending_data_;

  common::Time current_start_ = common::Time::min();
  common::Time current_end_ = common::Time::min();
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_RANGE_DATA_COLLATOR_H_
