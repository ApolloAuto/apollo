# Apollo 8.0 Quick Start Guide

This document is based on four typical scenarios, from simple to complex, to introduce how to use the package in a typical way, so that developers can quickly get started using Apollo.

## Prerequisites

Before reading this document, make sure you have installed the Apollo environment manager tool and launch apollo env container successfully according to the installation documentation.

## Demo

<video src="https://apollo-pkg-beta.cdn.bcebos.com/e2e/QuickStartDemo_v0.4_en.mp4" width="700px" height="400px" controls="controls"></video>

### Scenario 1: Using Dreamview to View Records

This scenario describes how to use Dreamview to play records and provides a foundation for developers to become familiar with the Apollo platform. You can use Dreamview to play the records provided by Apollo to further observe and learn about Apollo autonomous driving.

This article explains the installation and simple usage steps of Dreamview. For the detailed documentation of Dreamview, see the Dreamview documentation.

#### Step 1: Enter the Apollo Docker environment

1. Create workspace

```shell
mkdir application-demo
cd application-demo
```

2. Start Apollo env container

```shell
aem start
```

3. Enter Apollo env container

```shell
aem enter
```

> Note: If you want to force the container to be recreated you can specify the `-f` option and If you are using multiple containers at the same time, it is helpful to specify the container name with the `--name` option

4. Initialize workspace

```shell
aem init
```

#### Step 2: Install DreamView

In the same terminal, enter the following command to install Apollo's DreamView program.

```sudo apt install apollo-neo-dreamview-dev apollo-neo-monitor-dev
sudo apt install apollo-neo-dreamview-dev apollo-neo-monitor-dev
```

#### Step 3: Launch Dreamview

In the same terminal, enter the following command to start Apollo's DreamView program.

```shell
aem bootstrap start
```

#### Step 4: Download the Apollo demo record

A file with .record suffix is what we call a record.

> The Record file is used to record messages sent/received to/from channels in Cyber RT. Reply record files can help reproduce the behavior of previous operations of Cyber RT.

From the command line, type the following command to download the record package.

```shell
wget https://apollo-system.cdn.bcebos.com/dataset/6.0_edu/demo_3.5.record
```

#### Step 5: Play the Apollo demo record

```shell
cyber_recorder play -f demo_3.5.record --loop
```

> Note: The --loop option is used to set the loop playback mode.

#### Step 6: Use DreamView to view the record

Enter https://localhost:8888 in your browser to access Apollo DreamView:

![dv_page][dv_page.png]

If all is well, you can see a car moving forward in DreamView. The car and road conditions you see now are simply a playback of the data from the record by DreamView, just like playing back a recorded video.

#### Step 7: Stop DreamView

Enter the following command to end the DreamView process:

```shell
aem bootstrap stop
```

### Scenario 2: Cyber Component Extensions

This document describes how to develop and compile and run a Cyber component as a simple extension to example-component to provide a foundation for developers to become familiar with the Apollo platform. You can observe and learn more about the Apollo compilation process by compiling and running example-component.

The source code of example-component in the QuickStart Project is a demo based on Cyber RT extending Apollo functional components. If you are interested in how to write a component, dag file and launch file, you can find the full source code of example-component in the QuickStart Project.

The directory structure of demo/example-component is shown below:

```shell
demo/example_components/
|-- src
|   |-- common_component_example.cc
|   |-- common_component_example.h
|   |-- BUILD
|-- proto
|   |--examples.proto
|   |--BUILD
|-- BUILD
|-- cyberfile.xml
|-- example.dag
|-- example.launch
```

#### Step 1: Download quickstart project

Clone demo project

```shell
git clone https://github.com/ApolloAuto/application-demo.git
```

and enter project directory

```shell
cd application-demo
```

#### Step 2: Enter the Apollo Docker environment

Start apollo env container.

```shell
aem start
```

> Note: If you want to force the container to be recreated you can specify the `-f` option and If you are using multiple containers at the same time, it is helpful to specify the container name with the `--name` option.

Enter apollo env container.

```shell
aem enter
```

#### Step 3: Compile the component

Enter the following command to compile the component.

```shell
buildtool build --packages example_components
```

> The --packages argument specifies the path to the package specified in the compilation workspace, in this case example_components.

> For more detailed usage of `buildtool`, please refer to the [Apollo buildtool](../03_Package%20Management/apollo_buildtool.md) documentation

When you call the script build command, the current directory is the workspace directory, so be sure to use the script build command in the workspace.

Apollo's compilation tool will automatically analyze all the required dependencies, download and generate the necessary dependency information files automatically.

![example_building][example_building.png]

If the compiler needs to pass in arguments, you can use --builder_args to specify them.

#### Step 4: Run the component

Run the following command:

```shell
cyber_launch start example_components/example.launch
```

If everything works, the terminal will display as follows:

![example_launched][example_launched.png]

At this point, you can open another terminal, run cyber_monitor, and observe the data in channels:

```shell
cyber_monitor
```

### Scenario 3: Learning and experimenting with the planning module

This scenario describes how to learn and adjust the planning module, compile and debug the planning module, and help developers get familiar with the Apollo 8.0 development model.

The planning_customization module is an End-2-End solution (i.e. you can run through the entire contents of Routing Request within the simulation environment), but it does not contain any source code but a cyberfile.xml file. The cyberfile.xml file describes all the component packages (planning-dev, dreamview-dev, routing-dev, task_manager, and monitor-dev) that are dependent on the scenario and how they have been imported. Since the planning source code needs to be extended, the planning-dev package is introduced as "src", and the planning source code is automatically downloaded and copied to the workspace when the module is compiled.

The cyberfile for planning_customization is shown as follows:

```xml

<package>
  <name>planning-customization</name>
  <version>1.0.0</version>
  <description>
   planning_customization
  </description>
  <maintainer email="apollo-support@baidu.com">apollo-support</maintainer>
  <type>module</type>
  <src_path>//planning_customization</src_path>
  <license>BSD</license>
  <author>Apollo</author>
  <depend>bazel-extend-tools-dev</depend>
  <depend type="binary" repo_name="dreamview">dreamview-dev</depend>
  <depend type="binary" repo_name="routing">routing-dev</depend>
  <depend type="binary" repo_name="task-manager">task-manager-dev</depend>
  <depend type="binary" repo_name="monitor">monitor-dev</depend>
  <depend type="src" repo_name="planning">planning-dev</depend>

  <depend expose="False">3rd-rules-python-dev</depend>
  <depend expose="False">3rd-grpc-dev</depend>
  <depend expose="False">3rd-bazel-skylib-dev</depend>
  <depend expose="False">3rd-rules-proto-dev</depend>
  <depend expose="False">3rd-py-dev</depend>
  <depend expose="False">3rd-gpus-dev</depend>

  <builder>bazel</builder>
</package>
```

#### Step 1: Download quickstart project

Clone demo project

```shell
git clone https://github.com/ApolloAuto/application-demo.git
```

and enter project directory

```shell
cd application-demo
```

#### Step 2: Enter the Apollo Docker environment

Start apollo env container.

```shell
aem start
```

> Note: If you want to force the container to be recreated you can specify the `-f` option and If you are using multiple containers at the same time, it is helpful to specify the container name with the `--name` option.

Enter apollo env container.

```shell
aem enter
```

#### Step 3: Compile the planning source code package

```shell
buildtool build --packages planning_customization
```

The --packages argument specifies the path to the package specified in the compilation workspace, in this case planning_customization, and defaults to all packages in the workspace if not specified.

When script compile command is called, the current directory is the workspace directory, so be sure to use the script compile command under the workspace.

Apollo's compilation tool will automatically analyze all the required dependencies, download and generate the necessary dependency information files automatically. The first time you build, you need to pull some dependency packages from the Internet, which takes about 13 minutes depending on the speed and configuration of your computer.

![planning_build_finish][planning_build_finish.png]

#### Step 4: Debug planning

Enter the following command to run dreamview:

```shell
aem bootstrap start
```

If you have already started Dreamview, enter the following command to restart the Dreamview process:

```shell
aem bootstrap restart
```

At this point, DreamView and monitor will be started automatically, and you can enter localhost:8888 in your browser to open DreamView:

![dv_page_2][dv_page_2.png]

#### Step 5: Enter sim control simulation mode for debugging

1. Select the mode, model and map.
   - Select MKz Standard Debug in the upper menu bar.
   - Select MkzExample for the vehicle model.
   - Select Sunnyvale Big Loop for the map.

2. Click  Tasks, and in the Others module, select Sim Control to enter the simulation control, as shown in the following figure.

![start_simcontrol][start_simcontrol.png]

3. Click the Module Controller column on the left to start the process of the module to be debugged, and select the Planning, Routing modules.

![dv_modules][dv_modules_page.png]

4. Set the vehicle simulation driving path, click Route Editing on the left side, drag and click the mouse to set the vehicle driving path in the map, as shown in the following figure:

![route_request][route_request.png]

5. After you set the point position, click Send Routing Request. As shown in the following figure, the red path is found in the map by the Routing module, and the blue path is the local path planned by the Planning module at real time.

![dv_running_page][dv_running_page.png]

6. At this point, if you want to debug the planning module, you can directly modify the planning source code in the workspace, which is located in the modules/planning directory, and re-run the compilation script after the changes are made:

```shell
buildtool build --packages planning_customization
```

7. Re-run the planning module in Dreamview.

### Scenario 4: Awareness LIDAR Functionality Testing

This scenario describes how to start the Lidar Perception module using a package to help developers get familiar with the Apollo Perception module. You can observe the results of the Lidar Perception run by playing the record provided by Apollo.

#### Prerequisites

This document assumes that you have followed the Installation - Package Method > Installing Apollo environment manager tool to complete Steps 1 and 2. Compared to the three examples above, testing the sensing module functionality requires the use of the GPU, so you should obtain the GPU image of the package for testing and verification.

#### Step 1: Start Apollo Docker environment and enter

1. Create workspace

```shell
mkdir application-demo
cd application-demo
```

2. Enter the following command to start in GPU mode:

```shell
aem start_gpu -f
```

3. Enter the following command to access the container:

```shell
aem enter
```

4. Initialize workspace

```shell
aem init
```

#### Step 2: Download the record

1. Enter the following command to download the record:

```shell
wget https://apollo-system.bj.bcebos.com/dataset/6.0_edu/sensor_rgb.tar.xz
```

2. Create a directory and extract the downloaded record to the directory:

```shell
sudo mkdir -p ./data/bag/
sudo tar -xzvf sensor_rgb.tar.xz -C ./data/bag/
```

#### Step 3: Install DreamView

> Note: Apollo core can only be installed inside the container, do not perform this step on the host!

In the same terminal, enter the following command to install the DreamView program.

```shell
buildtool install --legacy dreamview-dev monitor-dev
```

#### Step 4: Install transform, perception and localization

1. In the same terminal, enter the following command to install the perception program.

```shell
buildtool install --legacy perception-dev
```

2. Enter the following command to install localization, v2x and transform programs.

```shell
buildtool install --legacy localization-dev v2x-dev transform-dev
```

#### Step 5: Run the module

1. Modify the pointcloud_topic flag in `/apollo/modules/common/data/global_flagfile.txt` (or add the pointcloud_topic flag if it doesn't exist) to specify the channel for point cloud data.

```shell
--pointcloud_topic=/apollo/sensor/velodyne64/compensator/PointCloud2
```

![package-configuration2][package-configuration2.png]

2. In the same terminal, enter the following command to start Apollo's DreamView program.

```shell
aem bootstrap
```

Enter localhost:8888 in the browser to open the DreamView page, then select the correct mode, model, and map.

![package-dreamview2][package-dreamview2.png]

Click the Module Controller module in the status bar on the left side of the page to start the transform module.

![package-transform][package-transform.png]

3. Start the LIDAR module using the mainboard method:

```shell
mainboard -d /apollo/modules/perception/production/dag/dag_streaming_perception_lidar.dag
```

#### Step 6: Result verification

1. Play the record: you need to mask out the perception channel data contained in the record with -k parameter.

```shell
cyber_recorder play -f ./data/bag/sensor_rgb.record -k /perception/vehicle/obstacles /apollo/perception/obstacles /apollo/perception/traffic_light /apollo/perception
```

2. Verify the detection result: click the LayerMenu in the left toolbar of DreamView and turn on Point Cloud in Perception.

![package-results1][package-results1.png]

To view the results.

![package-results2][package-results2.png]

#### Step 7: Model Replacement

The following describes the parameter configuration and replacement process for the MASK_PILLARS_DETECTION, CNN_SEGMENTATION, and CENTER_POINT_DETECTION models in the lidar detection process. You can easily replace these in lidar_detection_pipeline.pb.txt configuration to load and run a different model.

**MASK_PILLARS_DETECTION Model Replacement**

Modify the contents of the configuration file in lidar_detection_pipeline.pb.txt

```shell
vim /apollo/modules/perception/pipeline/config/lidar_detection_pipeline.pb.txt
```
Replace stage_type with MASK_PILLARS_DETECTION

```shell
stage_type: MASK_PILLARS_DETECTION
```

![mask1][mask1.png]

and change the content of the configuration file information of the corresponding stage to

```shell
stage_config: {
    stage_type: MASK_PPILLARS_DETECTION
    enabled: true
}
```

![mask2][maske2.png]

After saving the configuration file changes, start the LIDAR module and play the record to verify the detection results.

![mask3][mask3.png]

**CNN_SEGMENTATION model replacement**

Modify the content of the configuration file in lidar_detection_pipeline.pb.txt: replace stage_type with CNN_SEGMENTATION and modify the content of the configuration file of the corresponding stage.

```shell
stage_type: CNN_SEGMENTATION
```

```shell
stage_config: {
  stage_type: CNN_SEGMENTATION
  enabled: true

  cnnseg_config: {
    sensor_name: "velodyne128"
    param_file: "/apollo/modules/perception/production/data/perception/lidar/models/cnnseg/velodyne64/cnnseg_param.conf"
    proto_file: "/apollo/modules/perception/production/data/perception/lidar/models/cnnseg/velodyne64/deploy.prototxt"
    weight_file: "/apollo/modules/perception/production/data/perception/lidar/models/cnnseg/velodyne64/deploy.caffemodel"
    engine_file: "/apollo/modules/perception/production/data/perception/lidar/models/cnnseg/velodyne64/engine.conf"
  }
}
```

Start the lidar module and play the record to verify the detection result.

![cnn1][cnn1.png]

**CENTER_POINT_DETECTION model replacement**

Replace stage_type with CENTER_POINT_DETECTION:

```shell
stage_type: CENTER_POINT_DETECTION
```

and change the content of the configuration file information of the corresponding stage to

```shell
stage_config: {
  stage_type: CENTER_POINT_DETECTION
  enabled: true
}
```

Start the LIDAR module and play the record to verify the detection result:

![centor1][centor1.png]

## Install Apollo module source code

This section describes how to install the Apollo modules' source code under the workspace after installing the package.

### Step 1: Start and enter apollo env container

Download demo project

```shell
git clone https://github.com/ApolloAuto/application-demo.git
```

Enter project directory

```shell
cd application-demo
```

> the folder will be mounted to /apollo_workspace

Start apollo env container

```bash
aem start
```

Enter apollo env container

```shell
aem enter
```

### Step 2: Install source code

Execute the following command to install the source code

```bash
buildtool install cyber-dev audio-dev bridge-dev canbus-dev canbus-vehicle-lincoln-dev common-dev control-dev dreamview-dev drivers-dev guardian-dev localization-dev map-dev monitor-dev perception-dev planning-gpu-dev prediction-dev routing-dev storytelling-dev task-manager-dev third-party-perception-dev transform-dev v2x-dev
```

> A series of warnings may appear during the installation process, which is normal and can be ignored.

![install-warning][install-warning.png]

[dv_page.png]: <https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_8_0/dv_page_6334353.jpeg>
[example_building.png]: <https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_8_0/example_buiding_8617536.png>
[example_launched.png]: <https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_8_0/example_launched_07bcf7b.png>
[planning_build_finish.png]: <https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_8_0/planning_build_finish_1ac5b1e.png>
[dv_page_2.png]: <https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_8_0/dv_page_2_bdc05e7.png>
[start_simcontrol.png]: <https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_8_0/start_simcontrol_ee8eaf7.png>
[dv_modules_page.png]: <https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_8_0/dv_modules_page_cf16a28.png>
[route_request.png]: <https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_8_0/route_request_ba8694d.png>
[dv_running_page.png]: <https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_8_0/dv_running_page_e346da7.png>
[package-configuration2.png]: <https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_8_0/%E5%8C%85-%E9%85%8D%E7%BD%AE2_f5ce968.png>
[package-dreamview2.png]: <https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_8_0/%E5%8C%85-dreamview2_3d6fa5c.png>
[package-transform.png]: <https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_8_0/%E5%8C%85-%20transform_e13afb6.png>
[package-results1.png]: <https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_8_0/%E5%8C%85-%E7%BB%93%E6%9E%9C1_32f3f86.png>
[package-results2.png]: <https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_8_0/%E5%8C%85-%E7%BB%93%E6%9E%9C2_1be66a4.png>
[mask1.png]: <https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_8_0/mask1_f067520.png>
[maske2.png]: <https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_8_0/mask2_f5373a5.png>
[mask3.png]: <https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_8_0/mask3_7d0d0bd.png>
[cnn1.png]: <https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_8_0/cnn1_8d22d9f.png>
[centor1.png]: <https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_8_0/centor1_edea96e.png>
[install-warning.png]: <https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_8_0/%E6%88%AA%E5%B1%8F2022-12-09%20%E4%B8%8A%E5%8D%8811.12.22_1da6ba9.png>
