// Inner-loop controller
#pragma once

#include "acessna/sim/controller.hpp"

namespace acessna {

class FlightController : public Controller {
 public:
  void reset(const State& initial, const Controls& trim) override;
  Controls update(const State& s, double dt) override;

 private:
  Controls trim_;
};

}  // namespace acessna
