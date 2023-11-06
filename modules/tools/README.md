# tools

## Introduction

tools is an important part of the Apollo. It contains various tools and scripts to support the development and operation of the Apollo system.

Specifically, the tools module contains a number of tools for data processing, system calibration, simulation, and testing. For example, some of the tools in tools can be used to start and stop the Apollo system, or to perform system calibration and alignment. In addition, the tools module contains scripts and programs for simulation and testing, e.g., for generating test data, simulating vehicle behaviors, and so on.

In summary, the tools module in the Apollo modules directory provides great support and convenience for developers, allowing them to develop and test more efficiently.

## Prerequisites
 
The tools are mostly written in Python and relying on compiled proto modules. So
generally you need to do the following steps to make it function well.

`Note that all scripts in this page are referenced from Apollo root directory.`

> In source env:

```bash
# Compile everything including python proto libs.
apollo.sh build

# Setup PYTHONPATH properly.
source scripts/apollo_base.sh
```

> In package management:

```bash
buildtool install --legacy tools scripts
```

## Directory Structure
```shell
modules/tools
|-- BUILD
|-- README.md
|-- adataset
|-- amodel
|-- common
|-- control_info
|-- create_map
|-- cyberfile.xml
|-- data_cleaner
|-- dump_gpsbin
|-- gen_vehicle_protocol
|-- localization
|-- manual_traffic_light
|-- map_datachecker
|-- map_gen
|-- mapshow
|-- mapviewers
|-- mobileye_viewer
|-- mock_routing
|-- navigation
|-- navigator
|-- open_space_visualization
|-- ota
|-- perception
|-- planning
|-- planning_command
|-- plot_control
|-- plot_planning
|-- plot_trace
|-- prediction
|-- realtime_plot
|-- record_analyzer
|-- record_parse_save
|-- record_play
|-- replay
|-- restore_video_record
|-- rosbag
|-- routing
|-- sensor_calibration
|-- tools.BUILD
|-- vehicle_calibration
|-- vehicle_profile
`-- visualizer
```

## Highlight Tools

* Diagnostics

  `shortcuts: /apollo/scripts/diagnostics.sh`

  Display input/output protobuf messages for modules.

* Plot_control

  Subscribe control command message and plot recent x steering, throttle, and
  brake values.

* Realtime_plot

  `shortcuts: /apollo/scripts/realtime_plot.sh`

  Subscribe planning & control messages and plot real time trajectory, speed,
  curvature/ST-graph, and acceleration/heading.

* Record_play

  * Rtk_recorder

    `shortcuts: /apollo/scripts/run_rtk_recorder.sh`

  Record vehicle trajectory and save it into a file.

  * Rtk_player

    `shortcuts: /apollo/scripts/run_rtk_player.sh`

  Read recorded trajectory and publish it as a planning trajectory.
