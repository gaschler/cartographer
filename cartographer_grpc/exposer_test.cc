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

#include "cartographer/metrics/family_factory.h"
#include "cartographer/metrics/histogram.h"
#include "cartographer/metrics/register.h"
#include "cartographer_grpc/metrics/prometheus/family_factory.h"
#include "glog/logging.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "prometheus/exposer.h"

using testing::_;

namespace cartographer_grpc {
namespace {

static auto kScoresMetric = cartographer::metrics::Histogram::Null();

class Algorithm {
 public:
  static void RegisterMetrics(cartographer::metrics::FamilyFactory* factory) {
    auto boundaries = cartographer::metrics::Histogram::FixedWidth(0.05, 20);
    cartographer::metrics::HistogramFamily* scores_family = factory->NewHistogramFamily(
        "/algorithm/scores",
        "Scores achieved",
        boundaries);
    kScoresMetric = scores_family->Add({{"kind", "score"}});
  }
  void Run() {
    kScoresMetric->Observe(-1);
    kScoresMetric->Observe(0.11);
    kScoresMetric->Observe(0.2);
    kScoresMetric->Observe(0.5);
    kScoresMetric->Observe(2);
  }
};

TEST(ExposerTest, StartAndStop) {
  metrics::prometheus::FamilyFactory registry;
  Algorithm::RegisterMetrics(&registry);
  cartographer::metrics::RegisterAllMetrics(&registry);
  ::prometheus::Exposer exposer("0.0.0.0:9100");
  exposer.RegisterCollectable(registry.GetCollectable());

  Algorithm algorithm;
  algorithm.Run();

  LOG(INFO) << "Sleeping so you can inspect http://localhost:9100/metrics";
  sleep(60);
}

}  // namespace
}  // namespace cartographer_grpc
