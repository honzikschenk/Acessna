#include "acessna/telemetry.hpp"

#include <limits>

namespace acessna {

void Telemetry::record(const std::string& name, double value) {
  if (!enabled_) return;
  auto [it, inserted] = index_.try_emplace(name, names_.size());
  if (inserted) {
    names_.push_back(name);
    values_.push_back(value);
  } else {
    values_[it->second] = value;
  }
}

void Telemetry::begin_step() {
  for (double& v : values_) v = std::numeric_limits<double>::quiet_NaN();
}

Telemetry& Telemetry::discard() {
  static Telemetry sink(false);
  return sink;
}

}  // namespace acessna
