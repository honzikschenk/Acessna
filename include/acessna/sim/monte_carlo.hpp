// Monte Carlo runner
// Run i uses seed base_seed + i
#pragma once

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "acessna/sim/controller.hpp"
#include "acessna/sim/harness.hpp"

namespace acessna {

// Builds a fresh controller for each run
using ControllerFactory = std::function<std::unique_ptr<Controller>()>;

// Builds a run from its seed: draw the dispersions (wind, turbulence, initial conditions)
// from an RNG seeded with `seed`, and set ic.random_seed
using ScenarioGenerator = std::function<RunConfig(std::uint64_t seed)>;

struct MonteCarloConfig {
  int runs = 100;
  std::uint64_t base_seed = 1;
  int threads = 0;  // 0 = hardware concurrency
};

// Runs cfg.runs scenarios and returns summarize(seed, result)
template <class Summary>
std::vector<Summary> run_monte_carlo(
    const MonteCarloConfig& cfg, const ControllerFactory& make_controller,
    const ScenarioGenerator& scenario,
    const std::function<Summary(std::uint64_t seed, const RunResult&)>& summarize) {
  const int runs = std::max(cfg.runs, 0);
  int threads =
      cfg.threads > 0 ? cfg.threads : static_cast<int>(std::thread::hardware_concurrency());
  threads = std::clamp(threads, 1, std::max(runs, 1));

  std::vector<Summary> out(static_cast<std::size_t>(runs));
  std::atomic<int> next{0};
  std::exception_ptr error;
  std::mutex error_mutex;

  auto worker = [&] {
    for (int i = next++; i < runs; i = next++) {
      try {
        const std::uint64_t seed = cfg.base_seed + static_cast<std::uint64_t>(i);
        RunConfig run_cfg = scenario(seed);
        run_cfg.realtime = false;
        run_cfg.flightgear = false;
        auto controller = make_controller();
        out[static_cast<std::size_t>(i)] = summarize(seed, run(*controller, run_cfg));
      } catch (...) {
        std::lock_guard<std::mutex> lock(error_mutex);
        if (!error) error = std::current_exception();
        next = runs;  // stops handing out runs
      }
    }
  };

  std::vector<std::thread> pool;
  for (int t = 1; t < threads; ++t) pool.emplace_back(worker);
  worker();
  for (auto& th : pool) th.join();
  if (error) std::rethrow_exception(error);
  return out;
}

}  // namespace acessna
