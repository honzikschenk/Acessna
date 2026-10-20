# Acessna

Exploring guidance algorithms on a Cessna 172

## Commands

```sh
# Configure (first time, or after editing CMakeLists.txt)
cmake -S . -B build

# Build
cmake --build build -j

# Run
./build/acessna_run

# Test
ctest --test-dir build

# Plot
uv run analysis/plot_run.py run.csv # interactive window
uv run analysis/plot_run.py run.csv --save run.png
```

## Layout

| Path | Description |
| --- | --- |
| `include/acessna/sim/sim.hpp` | `C172Sim`: JSBSim wrapper, `InitialConditions`, `Controls`, `State` |
| `include/acessna/sim/controller.hpp` | `Controller` interface the harness flies |
| `include/acessna/sim/harness.hpp` | `run(controller, RunConfig)` closed-loop runner + CSV logging |
| `src/main.cpp` | Entrypoint |
| `src/control/` | Inner loops |
| `src/guidance/` | Guidance |
| `analysis/` | Python analysis of run logs |
| `tests/` | ctest tests |

### Conventions

- Surfaces are normalized `[-1, 1]`; throttle/mixture/flaps `[0, 1]`; values are clamped.
- `+elevator` = nose down, `+aileron` = roll right, `+rudder` = yaw right.
- Angles in degrees, rates in deg/s, altitude in ft, speeds in kts, `vs_fpm` in ft/min,
  `north_m`/`east_m` are meters from the start point.
- `control_hz` must evenly divide `sim_hz` (default 60 / 120 Hz).

## Visualizing in FlightGear

```sh
fgfs --fdm=null --native-fdm=socket,in,60,,5550,udp --aircraft=c172p --timeofday=noon --lat=37.325776 --lon=-79.978586 --altitude=3000 --heading=0 --in-air
# wait until the cockpit view is up, then:
./build/acessna_run
```
