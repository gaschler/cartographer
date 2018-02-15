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

#include "cartographer_grpc/metrics/prometheus/family_factory.h"

#include "cartographer/common/make_unique.h"
#include "prometheus/family.h"
#include "prometheus/histogram.h"

namespace cartographer_grpc {
namespace metrics {
namespace prometheus {

namespace {

using BucketBoundaries = cartographer::metrics::Histogram::BucketBoundaries;

class Histogram : public cartographer::metrics::Histogram {
 public:
  Histogram(::prometheus::Histogram* prometheus) : prometheus_(prometheus) {}

  void Observe(double value) override { prometheus_->Observe(value); }

 private:
  ::prometheus::Histogram* prometheus_;
};

class HistogramFamily : public cartographer::metrics::HistogramFamily {
 public:
  HistogramFamily(::prometheus::Family<::prometheus::Histogram>* prometheus,
                  const BucketBoundaries& boundaries)
      : prometheus_(prometheus), boundaries_(boundaries) {}

  Histogram* Add(const std::map<std::string, std::string>& labels) override {
    ::prometheus::Histogram* histogram = &prometheus_->Add(labels, boundaries_);
    auto wrapper = new Histogram(histogram);
    wrappers_.emplace_back(wrapper);
    return wrapper;
  }

 private:
  ::prometheus::Family<::prometheus::Histogram>* prometheus_;
  std::vector<std::unique_ptr<Histogram>> wrappers_;
  const BucketBoundaries boundaries_;
};

}  // namespace

FamilyFactory::FamilyFactory()
    : registry_(std::make_shared<::prometheus::Registry>()) {}

cartographer::metrics::HistogramFamily* FamilyFactory::NewHistogramFamily(
    const std::string& name, const std::string& description,
    const BucketBoundaries& boundaries) {
  auto& family = ::prometheus::BuildHistogram()
                     .Name(name)
                     .Help(description)
                     .Register(*registry_);
  auto wrapper = new HistogramFamily(&family, boundaries);
  histograms_.emplace_back(wrapper);
  return wrapper;
}

std::weak_ptr<::prometheus::Collectable> FamilyFactory::GetCollectable() const {
  return registry_;
}

}  // namespace prometheus
}  // namespace metrics
}  // namespace cartographer_grpc
