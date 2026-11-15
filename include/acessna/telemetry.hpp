// Records signals via controllers each control step
#pragma once

#include <cstddef>
#include <string>
#include <unordered_map>
#include <vector>

namespace acessna {

class Telemetry {
 public:
  Telemetry() = default;

  // Stores `value` under `name` for the current step. Names are registered on first use
  void record(const std::string& name, double value);

  // Marks every signal NaN
  void begin_step();

  // Signal names in registration order
  const std::vector<std::string>& names() const { return names_; }

  // Signal values at the curent step
  const std::vector<double>& values() const { return values_; }

  // Shared sink that ignores everything. Default for components with no telemetry attached
  static Telemetry& discard();

 private:
  explicit Telemetry(bool enabled) : enabled_(enabled) {}

  bool enabled_ = true;
  std::unordered_map<std::string, std::size_t> index_;
  std::vector<std::string> names_;
  std::vector<double> values_;
};

// Mixin for components that record telemetry
class TelemetrySource {
 public:
  void attach_telemetry(Telemetry& t) { tlm_ = &t; }

 protected:
  Telemetry& tlm() const { return *tlm_; }

 private:
  Telemetry* tlm_ = &Telemetry::discard();
};

}  // namespace acessna
