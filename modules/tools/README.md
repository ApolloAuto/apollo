# Apollo Tools

## Prerequisites

The tools are mostly written in Python and relying on compiled proto modules. So
generally you need to do the following steps to make it function well.

`Note that all scripts in this page are referenced from Apollo root directory.`

```bash
# Compile everything including python proto libs.
apollo.sh build

# Setup PYTHONPATH properly.
source scripts/apollo_base.sh
```

## Highlight Tools

* Diagnostics

  `shortcuts: scripts/diagnostics.sh`

  Display input/output protobuf messages for modules.

* Plot_control

  Subscribe control command message and plot recent x steering, throttle, and
  brake values.

* Realtime_plot

  `shortcuts: scripts/realtime_plot.sh`

  Subscribe planning & control messages and plot real time trajectory, speed,
  curvature/ST-graph, and acceleration/heading.

* Record_play

  * Rtk_recorder

    `shortcuts: scripts/run_rtk_recorder.sh`

  Record vehicle trajectory and save it into a file.

  * Rtk_player

    `shortcuts: scripts/run_rtk_player.sh`

  Read recorded trajectory and publish it as a planning trajectory.
