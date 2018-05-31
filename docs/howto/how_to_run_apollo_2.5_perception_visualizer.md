# How to Run Offline Perception LowCost Visualizer (Apollo 2.5)

For Apollo 2.5, we provide an offline visualization tool based OpenGL and PCL libraries to show the obstacle perception results(including lane info) in both image front view and bird view.

We introduce the detailed steps to build and run the offline visualizer in docker as below:

### 1. Build The Perception Lowcost
We use Bazel to build the offline perception visualizer.
```
cd /apollo
./apollo.sh build_opt_gpu
```
In order to invoke visualizer subnode, we need to change the pointing of dag streaming config by modifying /modules/perception/conf/perception_lowcost.conf with either of the following two gflag config.
(The default ones are defined in perception scripts already)

This config is image only:

```
 --dag_config_path=conf/dag_camera_obstacle_lane_motion_vis.config
```

This config contains both image and radar with fusion:

```
 --dag_config_path=conf/dag_camera_obstacle_offline_fusion_sync.config
```

### 2. Run The Visualizer With Collected ROS Bag for Apollo 2.5
Please double check your collected rosbag contains topic including camera images, radar, as well as localization results. The 3 corresponding topics are show below:

```
/apollo/sensor/camera/obstacle/front_6mm
/apollo/sensor/conti_radar
/apollo/localization/pose

```
After that, you can start perception and play bags. As before, bootstrap first:

```
./scripts/bootstrap.sh
```

Then you have 3 options to run:

### 1) Run perception with camera subnode only + visualization
This option requires `/apollo/sensor/camera/obstacle/front_6mm` only.
```
./scripts/perception_offline_visualizer.sh
```
### 2) Run full low-cost perception of camera, radar, lane markings + visualization
```
./scripts/perception_lowcost_vis.sh
```
### 3) Run full low-cost perception of camera, radar, lane markings, and no visualization
```
./scripts/perception_lowcost.sh
```

After starting perception module, run a rosbag with required rostopics:
```
rosbag play <bag file> [-l] [--clock]
```
You can also see obstacles and lane markings in Dreamview with `natigationn` and `simcontrol`


If running with visualization, you will see a pop-up window showing the perception result with images frame-by-frame. Top level panes are showing image detections in both 2D and 3D. Bottom left is the bird view visualization showing image obstacle tracks, radar obstacle tracks as well as fused tracks. You may switch on/off by pressing `O` (image track), `F` (fused track) and `D` (radar track) on the fly.
