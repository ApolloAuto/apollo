# How to Run Offline Perception Visualizer

The DreamView tool, can visualize and simulate the perception module, but it lacks the ability of visualizing point cloud and ROI areas which are helpful for debugging and testing perception algorithms. For this purpose, we provide an offline visualization tool based on OpenGL and PCL libraries to show obstacle perception results with point cloud. Please see [demo videos](http://apollo.auto/platform/perception.html) for this offline perception visualizer on Apollo official website.

We introduce the detailed steps to build and run the offline visualizer in docker as below:
### 1. Prepare PCD and Pose Data
Before running the visualizer, we need to prepare necessary PCD and Pose data which can be extracted from a ROS bag with recorded point cloud data. To facilitate the data extraction, we provide a ROS launch file to dump the PCD frame file and a python script to generate the Pose file for each frame.

#### 1.1 Launching the PCD exporter
 Additional details can be found in the [Velodyne driver doc](https://github.com/ApolloAuto/apollo/tree/master/modules/drivers/velodyne)
```
roslaunch velodyne export_pcd_offline.launch
```

#### 1.2 Playing ROS bag
The default directory of ROS bag is `/apollo/data/bag`. Assume the file name of ROS bag is `example.bag`.
```
cd /apollo/data/bag
rosbag play --clock example.bag
```
When playing the bag, the PCD files will be dumped in the export directory (defalut: `/apollo/data/pcd`). The extracted PCD files are named according to their frame numbers which correspond to the order of playing the recorded messages from the point cloud ROS topic (e.g., `/apollo/sensor/velodyne64/compensator/PointCloud2`). In addition, there are two other files (`stamp.txt` and `pose.txt`) generated in the export directory. They will be used to generate the Pose file for each frame.

#### 1.3 Generating Pose files
We provide a Python script `gen_pose_file.py` to generate the Pose files from `pose.txt`.
```
cd /apollo/modules/perception/tool
python gen_pose_file.py /apollo/data/pcd
```
The names of the generated Pose files correspond to their frame numbers as the PCD files. It means that the names of PCD and Pose files for a frame is the same but have different extension names (i.e., .pcd and .pose respectively).

### 2. Build The Offline Perception Visualizer
We use Bazel to build the offline perception visualizer
```
cd /apollo
bazel build -c opt //modules/perception/tool/offline_visualizer_tool:offline_lidar_visualizer_tool
```
The option `-c opt` is used for building the program with optimized performance, which is important for the offline simulation and visualization of the perception module in real time.
If you'd like to run the perception module with GPU, please use the command below:
```
bazel build -c opt --cxxopt=-DUSE_GPU //modules/perception/tool/offline_visualizer_tool:offline_lidar_visualizer_tool
```

### 3. Run The Visualizer With Offline Perception Simulation
Before running the visualizer, you may setup the data directories and the algorithm module settings in the configuration file `/apollo/modules/perception/tool/offline_visualizer_tool/conf/offline_lidar_perception_test.flag`. The detailed parameter settings for each algorithm module can be set according to the corresponding configuration files in `/apollo/modules/perception/tool/offline_visualizer_tool/conf/config_manager.config`. Then you may run the visualizer with offline perception simulation by the command below:
```
/apollo/bazel-bin/modules/perception/tool/offline_visualizer_tool/offline_lidar_visualizer_tool
```
Now you will see a pop-up window showing the perception result with point cloud frame-by-frame. The obstacles are shown with a purple rectangle bounding boxes. There are three modes to visualize the point cloud with/without the ROI area:
* Showing all the point cloud with grey color;
* Showing the point cloud of ROI area only with green color;
* Showing the point cloud of ROI area with green color and that of other areas with grey color. You may press the `S` key on keyboard to switch the modes in turn.
