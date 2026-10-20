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
ctest --test-dir build --output-on-failure

# Plot
uv run analysis/plot_run.py run.csv # interactive window
uv run analysis/plot_run.py run.csv --save run.png
```

## Layout

| Path | Description |
| --- | --- |
| `src/main.cpp` | Entrypoint: configures the run and flies `Autopilot` |
| `sim/sim` | `C172Sim`: JSBSim wrapper, `InitialConditions`, `Controls`, `State` |
| `sim/harness` | `run(controller, RunConfig)` closed-loop runner, CSV logging |
| `sim/controller` | `Controller` interface the harness flies |
| `autopilot/autopilot` | `Autopilot : Controller`: wires the layers below together |
| `autopilot/mode_manager` | `ModeManager`: state machine, current `Mode` -> `Targets` |
| `guidance/` | Guidance (TBD): will produce `Targets` for the modes |
| `control/setpoints` | `Targets`, `AttitudeSetpoint`, `FlightPathOutput`, `RateSetpoint`, `SurfaceCommands`: the data passed between layers |
| `control/flight_path_controller` | `FlightPathController`: `Targets` -> `AttitudeSetpoint` + throttle |
| `control/attitude_controller` | `AttitudeController`: `AttitudeSetpoint` -> `RateSetpoint` (p, q, r) |
| `control/rate_controller` | `RateController`: `RateSetpoint` -> elevator/aileron/rudder |
| `analysis/` | Python analysis of run logs |
| `tests/` | ctest tests |

`<dir>/<name>` means `include/acessna/<dir>/<name>.hpp` and `src/<dir>/<name>.cpp`

Each control step, `Autopilot` runs the layers top to bottom:

```text
ModeManager --Targets-> FlightPathController --AttitudeSetpoint-> AttitudeController --RateSetpoint-> RateController
```

### Conventions

- Surfaces are normalized `[-1, 1]`; throttle/mixture/flaps `[0, 1]`; values are clamped.
- `+elevator` = nose down, `+aileron` = roll right, `+rudder` = yaw right.
- Angles in degrees, rates in deg/s, altitude in ft, speeds in kts, `vs_fpm` in ft/min,
  `north_m`/`east_m` are meters from the start point.
- `control_hz` must evenly divide `sim_hz` (default 60 / 120 Hz).

## Visualizing in FlightGear

```sh
fgfs --fdm=null --native-fdm=socket,in,60,,5550,udp --aircraft=c172p --timeofday=noon --lat=37.325776 --lon=-79.978586 --altitude=3000 --heading=0 --in-air '--prop:/consumables/fuel/tank[0]/capacity-gal_us=30.8' '--prop:/consumables/fuel/tank[0]/density-ppg=6.0' '--prop:/consumables/fuel/tank[1]/capacity-gal_us=30.8' '--prop:/consumables/fuel/tank[1]/density-ppg=6.0'
# wait until the cockpit view is up, then:
./build/acessna_run
```
