# Navi Config

Navi Config is a tool to set parameters and flags in various modules for navigation mode.

### usage

```
python navi_config.py default.ini
```

*default.ini* file is the default navigation mode configuration, with following content:

```
[PerceptionConf]
# three perception solutions: MOBILEYE, CAMERA, and VELODYNE64
perception = CAMERA


[LocalizationConf]
utm_zone = 10


[PlanningConf]
# three planners are available: EM, LATTICE, NAVI
planner_type = EM

# highest speed for planning algorithms, unit is meter per second
speed_limit = 5
```

In **PerceptionConf** section, the *perception* parameter is to specify the perception  solution. Currently there are three supported in Apollo Navigation Mode: mobileye based, camera based and lidar based.

In the **LocalizationConf** section, utm_zone need to be specified based on  location of the road test.

In the **PlanningConf** section,   three planner are supported: EM, Lattice, and Navi. Select one for the planner_type parameter. speed_limt, which is the planner upper speed limit, is also configurable in this seciton, which unit is meter per second.

Developers could create differet ini files for  different test scenarios/purposes or modified the default.ini based on needs.