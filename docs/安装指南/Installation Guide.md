## Note
Since Apollo 10.0, both host and Docker container deployment modes are supported.

In host deployment mode, we recommend using the Ubuntu 22.04 system with x86_64 architecture or the Ubuntu 20.04 system with aarch64 architecture.

In Docker container deployment mode, you can use Ubuntu 18.04, 20.04, 22.04 system with x86_64 architecture or Ubuntu 20.04 system with aarch64 architecture.


## Step 1: Install basic software

### 1. Install Ubuntu Linux

Currently, Apollo supports Ubuntu 18.04, Ubuntu 20.04, and Ubuntu 22.04. For installation steps, please refer to the [official installation guide](https://ubuntu.com/tutorials/install-ubuntu-desktop).

Update relevant software after installation:

```shell
sudo apt-get update
sudo apt-get upgrade
```

> Attention: To complete the update, it is necessary to ensure a network connection.

### 2. Install Docker Engine

> Tip: If you are using host deployment mode, please skip this step.

Apollo relies on Docker 19.03+. To install the Docker engine, you can follow the official documentation for installation:

- See [Install Docker Engine on Ubuntu](https://docs.docker.com/engine/install/ubuntu/)。
- You can also install it directly through the installation script provided by Apollo:

```shell
wget http://apollo-pkg-beta.bj.bcebos.com/docker_install.sh
bash docker_install.sh
```

This process may run multiple scripts, just follow the script prompts to execute.

## Step 2 (optional): Obtain GPU support

Some modules of Apollo require GPU support to compile and run (such as perception modules). If you need to use such modules, you need to install Nvidia graphics card drivers and Nvidia
Container toolkit to obtain GPU support.

> Note: This tutorial is only applicable to Ubuntu system. Virtual machines cannot install graphics card drivers. Please search online for WSL by yourself
> Note: If you have previously installed Nvidia graphics card drivers and can output Nvidia smi normally when inputting it to the terminal, you can skip the section 1. Installing graphics card drivers

### 1. Install the driver

You can refer to the official website method to install the [official website driver](https://www.nvidia.cn/drivers/unix/linux-amd64-display-archive/).

Compatibility between graphics card drivers and CUDA versions. Due to Nvidia's fast hardware updates, we may encounter situations where graphics card drivers and CUDA versions are incompatible. The following are the smooth links we tested.

> TODO: CUDA has been updated and is no longer applicable here. The recommended version should be installed instead

| Graphics card series  | Tested graphics card   | Driver version            | CUDA version        |
| --------------------- | ---------------------- | ------------------------ | ------------------- |
| GeForce 10 Series     | GeForce GTX 1080       | nvidia-driver-535.183.01 | CUDA Version ：11.8 |
| GeForce RTX 20 Series | GeForce RTX 2070 SUPER | nvidia-driver-550.127.05 | CUDA Version ：11.8 |
| GeForce RTX 30 Series | GeForce RTX 3090       | nvidia-driver-525.147.05 | CUDA Version ：11.8 |
| GeForce RTX 30 Series | GeForce RTX 3060       | nvidia-driver-550.127.05 | CUDA Version ：11.8 |
| GeForce RTX 40 Series | GeForce RTX 4080       | nvidia-driver-550.127.05 | CUDA Version ：11.8 |
| AMD                   | MI100 dGPU             | ROCm™ 3.10 driver       |                     |

The recommended driver for the 10, 20 and 30 series graphics cards is version 470.63.01. You can download the driver from the [Nvidia official website](https://www.nvidia.cn/Download/driverResults.aspx/179605/cn/).

After downloading, find the corresponding folder and open the terminal to enter the installation instructions:

```shell
sudo chmod 777 NVIDIA-Linux-x86_64-470.63.01.run
sudo ./NVIDIA-Linux-x86_64-470.63.01.run
```

After installation, you can use the nvidia smi command to check if the driver has been successfully installed. If everything is normal, you can see a prompt similar to the following:

```shell
Tue Jan  3 12:04:21 2023
+-----------------------------------------------------------------------------+
| NVIDIA-SMI 460.91.03    Driver Version: 470.63.01    CUDA Version: 11.4     |
|-------------------------------+----------------------+----------------------+
| GPU  Name        Persistence-M| Bus-Id        Disp.A | Volatile Uncorr. ECC |
| Fan  Temp  Perf  Pwr:Usage/Cap|         Memory-Usage | GPU-Util  Compute M. |
|                               |                      |               MIG M. |
|===============================+======================+======================|
|   0  GeForce GTX 1080    Off  | 00000000:01:00.0 Off |                  N/A |
|  0%   38C    P8     7W / 198W |    239MiB /  8118MiB |      0%      Default |
|                               |                      |                  N/A |
+-------------------------------+----------------------+----------------------+

+-----------------------------------------------------------------------------+
| Processes:                                                                  |
|  GPU   GI   CI        PID   Type   Process name                  GPU Memory |
|        ID   ID                                                   Usage      |
|=============================================================================|
|    0   N/A  N/A      2566      G   /usr/lib/xorg/Xorg                 18MiB |
|    0   N/A  N/A      2657      G   /usr/bin/gnome-shell               67MiB |
|    0   N/A  N/A      6104      G   /usr/lib/xorg/Xorg                132MiB |
|    0   N/A  N/A      6234      G   /usr/bin/gnome-shell               13MiB |
|    0   N/A  N/A      7440      G   gnome-control-center                1MiB |
+-----------------------------------------------------------------------------+
```

### 2. Install Nvidia container toolkit

> Tip: If you are using host deployment mode, please skip this step.

In order to obtain GPU support within the container, NVIDIA Container Toolkit needs to be installed after installing Docker. You can refer to the official installation documentation or follow the following instructions to install NVIDIA Container Toolkit:

```shell
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
  && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt-get -y update
sudo apt-get install -y nvidia-container-toolkit
```

After installation, configure NVIDIA Container Toolkit

```shell
sudo nvidia-ctk runtime configure --runtime=docker
```

Restart Docker after configuration is complete

```shell
sudo systemctl restart docker
```

### 3. Install cuda-toolkit

If you wish to run Apollo directly on the host instead of in a container, you need to install the CUDA toolkit to support the compilation and operation of Apollo. For detailed information, please refer to [Nvidia cuda Document](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html#network-repo-installation-for-ubuntu)。

Taking Ubuntu 22.04 as an example, install CUDA Toolkit 11.8

```shell
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb

sudo apt-get update

sudo apt-get install -y cuda-toolkit-11-8
```

## Step 3: Install Apollo Environment Management Tool

Apollo Environment Management Tool is a command-line tool that helps manage and launch Apollo environment containers.

### 1. Install dependent software

```shell
sudo apt-get update
sudo apt-get install ca-certificates curl gnupg
```

### 2. Add the gpg key for the Apollo software source on the host computer, and set up the source and update settings

```shell
# add gpg key
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://apollo-pkg-beta.cdn.bcebos.com/neo/beta/key/deb.gpg.key | sudo gpg --dearmor -o /etc/apt/keyrings/apolloauto.gpg
sudo chmod a+r /etc/apt/keyrings/apolloauto.gpg

# set apt source and update
echo \
    "deb [arch="$(dpkg --print-architecture)" signed-by=/etc/apt/keyrings/apolloauto.gpg] https://apollo-pkg-beta.cdn.bcebos.com/apollo/core"\
    $(. /etc/os-release && echo "$VERSION_CODENAME") "main" | \
    sudo tee /etc/apt/sources.list.d/apolloauto.list
sudo apt-get update
```

> Note: If you have installed Apollo 8.0 before, the content like `deb https://apollo-pkg-beta.cdn.bcebos.com/neo/beta bionic main` may be found in the /etc/apt/sources.list file on the host, you can delete it directly. The apollo source configuration on the host only serves to install aem tool.

### 3. Install AEM：

```shell
sudo apt install apollo-neo-env-manager-dev --reinstall
```

After successful installation, it can be used

```shell
aem -h
```

## Step 4: Install Example Project

### 1. Select an example project

Apollo currently offers 3 sample projects, you can choose one according to your needs

- [application-core](https://github.com/ApolloAuto/application-core) , which includes all open source software packages of Apollo, allows you to build your own applications based on this project
- [application-pnc](https://github.com/ApolloAuto/application-pnc) , only includes software packages related to planning and control, suitable for users who only focus on planning and control direction
- [application-perception](https://github.com/ApolloAuto/application-perception), only includes perception related software packages, suitable for users who only focus on perception direction

Additionally, Apollo's full source code project [apollo](https://github.com/ApolloAuto/apollo) can also be used

> Tip: Since Apollo 10.0, the source code mode and package mode use the same compilation tool, we recommend that you use the following command for compilation and operation, this dev_start.sh, dev_into.sh, apollo.sh scripts are no longer recommended.


### 2. Cloning Project

Taking application core as an example

```shell
git clone https://github.com/ApolloAuto/application-core.git application-core
```

### 3. Start and enter the Apollo environment container

```shell
# enter the project directory
cd application-core

# switching the environment configuration: it will automatically recognize your host's environment and change . env and. workspace. json configurations
# if you use apollo source code mode, you don't need to execute this step. aem will automatically recognize the startup environment
bash setup.sh

# start and enter into the apollo environment
aem start

# if you use host mode, execute this command:
aem start -b host
```

> Note: you can use the same operation in the container or host mode. use aem enter command to enter the environment, container mode will enter into a docker container, and host mode will enter into an apollo bash space.

### 4. Install software packages

The example project includes a directory called core, where the core/cyberfile.xml file describes the software packages that the project depends on, which can be installed using the buildtool tool tool

```shell
buildtool build -p core

# Note: If you want to compile and install all the code and packages under the working directory, please execute
buildtool build
```

> The true meaning of this operation is to compile the core package in the project, but the `core` itself does not have any code that needs to be compiled, so this operation will only install dependencies package declared in `core/cyberfile.xml`

> Tip: the latest buildtool will auto add --opt --gpu compile parameters by default, if you want to use gdb debug please execute buildtool with --dbg parameter.

### 5. Select vehicle configuration

The profiles/samples directory in the example project is an official vehicle configuration based on one radar and two cameras. You can refer to the samples in the profiles directory to write your own vehicle configuration. The effective method for configuring the vehicle is as follows:

```shell
# Taking sample as an example
aem profile use sample
```

### 6. Play data record

#### Retrieve data record

```shell
wget https://apollo-system.cdn.bcebos.com/dataset/6.0_edu/demo_3.5.record -P $HOME/.apollo/resources/records/
```

Obtain the map corresponding to the data record

```shell
buildtool map get sunnyvale
```

> Starting from version 9.0.0-rc-r10, map data has been separated and needs to be downloaded separately, and will no longer be released with the map package; You can view all available maps through the buildtool map listcommand

#### Launch Dreamview+

```shell
aem bootstrap start --plus
```

##### Play data packets in Dreamview+

After starting Dreamview+, enter localhost: 8888 in the browser to enter the Dreamview+interface. You can choose the default mode or other modes to play data packets. This section takes the default mode as an example.

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_alpha_doc/image_7deb2d2.png)

1. Select **Default Mode** 。

2. Check **Accept the User Agreement and Privacy Policy/接受用户协议和隐私政策** to enter the **Mode Settings/模式设置** page.

3. On the **Mode Settings/模式设置** page, set the broadcast parameters.

- Select Record in **Operations/操作** .
- In **Environment Resources/环境资源** , click on **Records/数据包** and select the specific package you want to play.
- In **Environment Resources/环境资源** , click on **HDMap/高精地图** and select Sunnyvale Big Loop.

4. Click the play button in the bottom area.

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_9_0/image_45acc2d.png)

You can see the playback of data packets in **Vehicle Visualization/车辆可视化** .

##### Play data packets through the command line

1. Enter the Docker environment,

2. Download the required data package first from **Resource Manager/资源管理** > **Records/数据包** in Dreamview+. Enter the following command to play the data packet:

```shell
cyber_recorder play -f ~/.apollo/resources/records/<packet name> -l
```

> Note: If you want to loop the data packet, add - l; if you don't loop the data packet, you don't need to add -l.

### 8. Installation directory structure instructions

At this point, Apollo installation has been completed

The directory structure of the entire project is as follows

```shell
application-core
├── .aem
│   └── envroot
│       ├── apollo          # will be mounted to the/Apollo directory inside the container
│       └── opt             # will be mounted to the/opt/directory inside the container, and Apollo's software packages will be installed by default to/opt/, so this directory can serve as a cache
├── core                    # Project Dependency Package
│   ├── BUILD
│   └── cyberfile.xml       # Package description file, describing all dependencies of the entire project
├── CPPLINT.cfg
├── data                    # Data directory, will be mounted to/album/data
│   ├── calibration_data    # Calibration configuration directory, will be mounted to/add-on/modules/calibration/data
│   ├── kv_db.sqlite
│   ├── log                 # log directory, will be mounted to/opt/aollo/neo/data/log
│   └── map_data            # Map directory, will be mounted to/pollo/modules/map/data
├── profiles                # New configuration directory
│   ├── current -> sample   # Currently enabled configuration directory
│   └── sample              # Official single lidar and two camera sample vehicle configuration provided
├── third_party
├── tools -> /opt/apollo/neo/packages/bazel-extend-tools/latest/src
├── .vscode                 # default vscode configuration
│   ├── c_cpp_properties.json
│   └── settings.json
├── WORKSPACE               # Bazel Configuration
└── .workspace.json         # Apollo project configuration, where you can specify the software package version
```

Next, you can learn more about how to use Apollo through practical tutorials

- [Apollo Planning Practice](docs/应用实践/开发调试教程/Apollo规划实践/综述.md)
- [Apollo Perception Practice](docs/应用实践/开发调试教程/Apollo感知实践/综述.md)

## Step 5: Delete Project (optional)

This step explains how to delete an installed project

### 1. Delete container

Taking application-core as an example

```shell
# enter the project directory
cd application-core
# delete environment
aem remove
```

### 2. Delete project

```shell
# Return to the previous directory
cd ..
# Delete project directory
rm -r application-core
```
