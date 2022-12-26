# Launch and Run Apollo

This document describes how to install and run Apollo via the package method.

This article assumes that you have followed [Installation - Package Method](../01_Installation%20Instructions/apollo_software_installation_guide_package_method.md) and completed the installation of the apollo environment manager tool.

> Make sure Nvidia GPU is available and that you have installed the appropriate Nvidia driver if you want to run the entire system.

## Step 1. Create an end-2-end project

1. Create workspace.

```shell
mkdir application-urban
```

```shell
cd application-urban
```

> Note: You can also check out the demo code from https://github.com/ApolloAuto/application-urban

2. Start Apollo Env Container.

```shell
aem start_gpu
```

> Note: For more detailed usage of `aem`, please refer to the [Apollo Environment Manager](../03_Package%20Management/apollo_env_manager.md) documentation..

3. Enter Apollo Env Container.

```shell
aem enter
```

4. Initialize workspace.

```shell
aem init
```

## Step 2. Create an end-2-end package

1. Create package path and `cyberfile.xml`

```shell
mkdir mkz
```

```shell
touch mkz/cyberfile.xml
```

2. Add necessary packages.

```shell
vim mkz/cyberfile.xml
```

```xml
<?xml version='1.0' encoding='utf-8'?>
<package>
  <name>mkz</name>
  <version>1.0.0</version>
  <description>
   mkz is end to end project, providing a quick setup for Apollo.
  </description>
  <maintainer email="apollo-support@baidu.com">Apollo</maintainer>
  <type>module</type>
  <src_path>//mkz</src_path>
  <license>BSD</license>
  <author>Apollo</author>
  <builder>bazel</builder>

  <depend repo_name="audio" type="binary">audio-dev</depend>
  <depend repo_name="bridge" type="binary">bridge-dev</depend>
  <depend repo_name="canbus" type="binary">canbus-dev</depend>
  <depend repo_name="control" type="binary">control-dev</depend>
  <depend repo_name="dreamview" type="binary">dreamview-dev</depend>
  <depend repo_name="drivers" type="binary">drivers-dev</depend>
  <depend repo_name="guardian" type="binary">guardian-dev</depend>
  <depend repo_name="localization" lib_names="localization" type="binary">localization-dev</depend>
  <depend repo_name="map" type="binary">map-dev</depend>
  <depend repo_name="monitor" type="binary">monitor-dev</depend>
  <depend repo_name="perception" type="binary">perception-dev</depend>
  <depend repo_name="planning" type="binary">planning-gpu-dev</depend>
  <depend repo_name="prediction" type="binary">prediction-dev</depend>
  <depend repo_name="routing" type="binary">routing-dev</depend>
  <depend repo_name="storytelling" type="binary">storytelling-dev</depend>
  <depend repo_name="task-manager" type="binary">task-manager-dev</depend>
  <depend repo_name="tools" type="binary">tools-dev</depend>
  <depend repo_name="transform" type="binary">transform-dev</depend>
  <depend repo_name="v2x" type="binary">v2x-dev</depend>

  <depend expose="False">3rd-rules-python-dev</depend>
  <depend expose="False">3rd-grpc-dev</depend>
  <depend expose="False">3rd-bazel-skylib-dev</depend>
  <depend expose="False">3rd-rules-proto-dev</depend>
  <depend expose="False">3rd-py-dev</depend>
  <depend expose="False">3rd-gpus-dev</depend>
</package>
```

> Note: For more information about `cyberfile.xml`, please refer to the [Introduction to Pakcage of Apollo](../03_Package%20Management/introduction_to_package_of_apollo.md) documentation.

> Note: if Nvidia GPU is not available, you should remove the CUDA-based module (such as perception).

3. Add compiling description file.

```shell
vim mkz/BUILD
```

```bazel
load("//tools/install:install.bzl", "install", "install_src_files")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

install(
    name = "install",
    data = [":cyberfile.xml"],
    data_dest = "mkz"
)

install_src_files(
    name = "install_src",
    src_dir = ["."],
    dest = "mkz",
    filter = "*",
)
```

> Note: For more information about `mkz/BUILD`, please refer to the [Introduction to Pakcage of Apollo](../03_Package%20Management/introduction_to_package_of_apollo.md) documentation

4. Add dependency description file

```shell
touch mkz/mkz.BUILD
```

> Note: For more information about `mkz.BUILD`, please refer to the [Introduction to Pakcage of Apollo](../03_Package%20Management/introduction_to_package_of_apollo.md) documentation

## Step 3. Build

Use `buildtool` to build the end-2-end package, and it will download and install all required packages automatically.

```shell
buildtool build --packages mkz
```

For more detailed usage of `buildtool`, please refer to the [Apollo buildtool](../03_Package%20Management/apollo_buildtool.md) documentation.

## Step 4. Run Apollo

### 1. Start Apollo

```shell
aem bootstrap
```

This command will start Dreamview backend with the monitor module enabled.

### 2. Access Dreamview Web UI

Open [http://localhost:8888](http://localhost:8888) in your browser, e.g. Chrome, and you can see this screen. However, no module(s) except monitor is running in the background at this moment.

![Access Dreamview](../01_Installation%20Instructions/images/apollo_bootstrap_screen.png)

### 3. Select Driving Mode and Map

From the dropdown box of Mode Setup, select "Mkz Standard Debug" mode. From the
dropdown box of Map, select "Sunnyvale with Two Offices".

![Drive Mode and Map Selection](../01_Installation%20Instructions/images/dreamview_6_0_setup_profile.png)

### 4. Replay Demo Record

To see if the system works, use the demo record to "feed" the system.

```
# You need to download the demo record using the following commands
wget -c https://apollo-system.cdn.bcebos.com/dataset/6.0_edu/demo_3.5.record

# You can now replay this demo "record" in a loop with the '-l' flag
cyber_recorder play -f demo_3.5.record -l
```

Dreamview shows a vehicle running forward. (The following image might be
different due to frontend code changes.)

![Dreamview with Trajectory](../01_Installation%20Instructions/images/dv_trajectory_6.0.png)

### Congrats!

You have successfully built Apollo! Now you can revisit [Apollo Readme](../../README.md) for additional guidelines on the necessary hardware setup.
