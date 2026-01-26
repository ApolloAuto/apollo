## Notice

Apollo 11.0 uses Docker containers for deployment, supporting Ubuntu 18.04, 20.04, 22.04 on x86_64 architecture, or Ubuntu 20.04 on aarch64 architecture.

## Step 1: Install Basic Software

### 1. Install Ubuntu Linux

Apollo currently supports Ubuntu 18.04, Ubuntu 20.04, and Ubuntu 22.04. For installation steps, refer to the [Official Installation Guide](https://ubuntu.com/tutorials/install-ubuntu-desktop).

After installation, update the related software:

```shell
sudo apt-get update
sudo apt-get upgrade
```

> Note: Ensure network connection is available to complete the updates.

### 2. Install Docker Engine

Apollo depends on Docker 19.03+. To install Docker Engine, you can follow the official documentation:

- Refer to [Install Docker Engine on Ubuntu](https://docs.docker.com/engine/install/ubuntu/).

- Alternatively, you can install directly using the installation script provided by Apollo:

```shell
wget http://apollo-pkg-beta.bj.bcebos.com/docker_install.sh
bash docker_install.sh
```

This process may require running the script multiple times; follow the script prompts accordingly.

## Step 2: Get GPU Support (Optional)

Some Apollo modules require GPU support for compilation and running (e.g., perception modules). If you need to use such modules, install the Nvidia graphics driver and Nvidia container toolkit to get GPU support.

> Note: This tutorial is only applicable to Ubuntu systems. Virtual machines cannot install graphics drivers. For WSL, please search online. Note: If you have already installed Nvidia graphics drivers (i.e., `nvidia-smi` outputs normally in the terminal), you can skip the `1. Install Graphics Driver` section.

### 1. Install Graphics Driver

**You can refer to the official method to install the driver [Official Driver](https://www.nvidia.cn/drivers/unix/linux-amd64-display-archive/).**

Starting from Apollo 11.0, the minimum CUDA version requirement is `11.8`, so it is recommended to use driver version `520.61.05` or higher. For compatibility information between CUDA version and driver version, refer to [Nvidia Official Documentation](https://docs.nvidia.com/cuda/cuda-toolkit-release-notes/index.html).

Due to rapid hardware updates from Nvidia, incompatibility between graphics drivers and CUDA versions may occur. Below is our tested compatible chain:

| Graphics Card Series  | Tested Graphics Card   | Driver Version           | CUDA Version       |
| --------------------- | ---------------------- | ------------------------ | ------------------ |
| GeForce 10 Series     | GeForce GTX 1080       | nvidia-driver-535.183.01 | CUDA Version: 11.8 |
| GeForce RTX 20 Series | GeForce RTX 2070 SUPER | nvidia-driver-550.127.05 | CUDA Version: 11.8 |
| GeForce RTX 30 Series | GeForce RTX 3090       | nvidia-driver-525.147.05 | CUDA Version: 11.8 |
| GeForce RTX 30 Series | GeForce RTX 3060       | nvidia-driver-550.127.05 | CUDA Version: 11.8 |
| GeForce RTX 40 Series | GeForce RTX 4080       | nvidia-driver-550.127.05 | CUDA Version: 11.8 |
| AMD                   | MI100 dGPU             | ROCm™ 3.10 driver        |                    |

You can [download the driver](https://www.nvidia.cn/drivers/lookup/) from Nvidia's official website [nvidia-driver-535.179](https://www.nvidia.cn/Download/driverResults.aspx/224621/cn/)

After downloading, locate the corresponding folder, open the terminal, and enter the installation command:

Taking `NVIDIA-Linux-x86_64-535.179.run` as an example:

```shell
sudo chmod 777 NVIDIA-Linux-x86_64-535.179.run
sudo ./NVIDIA-Linux-x86_64-535.179.run
```

After installation, you can check if the driver is installed successfully using the `nvidia-smi` command. If everything is normal, you will see output similar to the following:

```shell
Mon Nov 11 16:35:59 2024
+-----------------------------------------------------------------------------------------+
| NVIDIA-SMI 555.42.06              Driver Version: 555.42.06      CUDA Version: 12.5     |
|-----------------------------------------+------------------------+----------------------+
| GPU  Name                 Persistence-M | Bus-Id          Disp.A | Volatile Uncorr. ECC |
| Fan  Temp   Perf          Pwr:Usage/Cap |           Memory-Usage | GPU-Util  Compute M. |
|                                         |                        |               MIG M. |
|=========================================+========================+======================|
|   0  NVIDIA GeForce RTX 2070 ...    Off |   00000000:06:00.0 Off |                  N/A |
| 30%   36C    P8              4W /  215W |     200MiB /   8192MiB |      0%      Default |
|                                         |                        |                  N/A |
+-----------------------------------------+------------------------+----------------------+

+-----------------------------------------------------------------------------------------+
| Processes:                                                                              |
|  GPU   GI   CI        PID   Type   Process name                              GPU Memory |
|        ID   ID                                                               Usage      |
|=========================================================================================|
|    0   N/A  N/A      1095      G   /usr/lib/xorg/Xorg                             35MiB |
|    0   N/A  N/A      1972      G   /usr/lib/xorg/Xorg                             94MiB |
|    0   N/A  N/A      2102      G   /usr/bin/gnome-shell                           59MiB |
+-----------------------------------------------------------------------------------------+
```

### 2. Install Nvidia Container Toolkit

To get GPU support within containers, you need to install the NVIDIA Container Toolkit after installing Docker. Run the following command to install NVIDIA Container Toolkit:

```shell
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
  && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt-get -y update
sudo apt-get install -y nvidia-container-toolkit
```

After installation, configure NVIDIA Container Toolkit:

```shell
sudo nvidia-ctk runtime configure --runtime=docker
```

After configuration, restart Docker:

```shell
sudo systemctl restart docker
```

## Step 3: Install Apollo Environment Manager

Apollo Environment Manager is a command-line tool that helps manage and start Apollo environments.

### 1. Install Dependencies

```shell
sudo apt-get update
sudo apt-get install ca-certificates curl gnupg
```

### 2. Add Apollo Software Repository GPG Key to Host, Set Up Source and Update

```shell
# Add GPG key
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://apollo-pkg-beta.cdn.bcebos.com/neo/beta/key/deb.gpg.key | sudo gpg --dearmor -o /etc/apt/keyrings/apolloauto.gpg
sudo chmod a+r /etc/apt/keyrings/apolloauto.gpg

# Set up source and update
echo \
    "deb [arch="$(dpkg --print-architecture)" signed-by=/etc/apt/keyrings/apolloauto.gpg] https://apollo-pkg-beta.cdn.bcebos.com/apollo/core"\
    $(. /etc/os-release && echo "$VERSION_CODENAME") "main" | \
    sudo tee /etc/apt/sources.list.d/apolloauto.list
sudo apt-get update
```

> Note: If you have previously installed Apollo 8.0, there may be a configuration in the `/etc/apt/sources.list` file on the host similar to `deb https://apollo-pkg-beta.cdn.bcebos.com/neo/beta bionic main`. You can delete it directly. The Apollo source configuration on the host is only used to install the `aem` tool.

### 3. Install AEM:

```shell
sudo apt install apollo-neo-env-manager-dev --reinstall
```

After successful installation, you can use it:

```shell
aem -h
```

## Step 4: Install Example Project

### 1. Choose Example Project

Apollo currently provides 3 example projects. You can choose one based on your needs:

- [application-core](https://github.com/ApolloAuto/application-core) - Contains all Apollo open-source software packages. You can build your own applications based on this project.

- [application-pnc](https://github.com/ApolloAuto/application-pnc) - Contains only planning and control related software packages, suitable for users focusing only on planning and control direction.

- [application-perception](https://github.com/ApolloAuto/application-perception) - Contains only perception-related software packages, suitable for users focusing only on perception direction.

> You can also use the full-source Apollo project [apollo](https://github.com/ApolloAuto/apollo)

> Tip: Starting from Apollo 10.0, source code mode and package mode use the same compilation tools. We recommend you uniformly use the following commands for compilation and running. Scripts like dev_start.sh, dev_into.sh, apollo.sh in source code mode are no longer recommended.

### 2. Clone the Project

Taking application-core as an example:

```shell
git clone https://github.com/ApolloAuto/application-core.git application-core
```

### 3. Start and Enter Apollo Environment

```shell
# First enter the project directory
cd application-core

# Switch environment configuration, automatically identifies your host environment to switch .env and .workspace.json configuration
# If you are using Apollo source code method, you do not need to execute this step, aem will automatically identify and start the environment
bash setup.sh

# Start the container
aem start
```

### 4. Install Software Packages

The example project contains a directory named `core`, where the `core/cyberfile.xml` file describes the software packages that the project depends on. You can install the dependency packages using the buildtool:

```shell
buildtool build -p core

# Note: If you want to compile and install all code and packages in the working directory, execute:
buildtool build
```

> The actual meaning of this operation is to compile the `core` package in the project, but `core` itself does not have code that needs to be compiled, so this operation will only install the dependency packages declared in `core/cyberfile.xml`.

> Tip: The new version of buildtool will compile with --opt --gpu parameters by default (if you use GPU environment). If you need gdb debugging, please compile with --dbg parameter.

### 5. Select Vehicle Configuration

The `profiles/sample` directory in the example project is the official vehicle configuration based on one LiDAR and two cameras. You can refer to the sample under the profiles directory to write your own vehicle configuration. The method to activate vehicle configuration is as follows:

```shell
# Taking sample as an example
aem profile use sample
```

### 6. Play Data Package

#### Get Data Package

```shell
wget https://apollo-system.cdn.bcebos.com/dataset/6.0_edu/demo_3.5.record -P $HOME/.apollo/resources/records/
```

#### Get Map Corresponding to Data Package

```shell
buildtool map get sunnyvale
```

> Starting from version `9.0.0-rc-r10`, map data has been separated and needs to be downloaded separately, no longer released with the map package; you can view all available maps through `buildtool map list`.

#### Start Dreamview+

```shell
aem bootstrap start --plus
```

##### Play Data Package in Dreamview+

After starting Dreamview+, enter `localhost:8888` in the browser to enter the Dreamview+ interface. You can choose the default mode or other modes to play the data package. This section uses the default mode as an example.

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_alpha_doc/image_7deb2d2.png)

1. Select **Default Mode**.

2. Check **Accept the User Agreement and Privacy Policy**, and click **Enter this Mode** to enter the **Mode Settings** page.

3. On the **Mode Settings** page, set the playback parameters.
   
   - Select **Record** under **Operations**.
   - Under **Environment Resources**, click **Records** and select the specific data package you want to play.
   - Under **Environment Resources**, click **HDMap** and select **Sunnyvale Big Loop**.

4. Click the play button at the bottom area.
   
   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_9_0/image_45acc2d.png)
   
   You can see the playback of the data package in **Vehicle Visualization**.

##### Play Data Package via Command Line

1. Enter the docker environment.

2. In Dreamview+ **Resource Manager** > **Records**, first download the required data package. Enter the following command to play the data package:
   
   ```bash
   cyber_recorder play -f ~/.apollo/resources/records/data_package_name -l
   ```

> Note: If you want to loop play the data package, add -l; if you don't want to loop play, don't add -l.

### 7. Directory Structure Explanation

At this point, Apollo installation is complete.

The directory structure of the entire project is as follows:

```shell
application-core
├── .aem
│   └── envroot
│       ├── apollo          # Will be mounted to /apollo directory in container
│       └── opt             # Will be mounted to /opt/ directory in container, and Apollo software packages will be installed to /opt/ by default, so this directory can serve as cache
├── core                    # Project dependency packages
│   ├── BUILD
│   └── cyberfile.xml       # Package description file, describing all dependencies of the entire project
├── CPPLINT.cfg
├── data                    # Data directory, will be mounted to /apollo/data
│   ├── calibration_data    # Calibration configuration directory, will be mounted to /apollo/modules/calibration/data
│   ├── kv_db.sqlite
│   ├── log                 # Log directory, will be mounted to /opt/apollo/neo/data/log
│   └── map_data            # Map directory, will be mounted to /apollo/modules/map/data
├── profiles                # New version configuration directory
│   ├── current -> sample   # Currently enabled configuration directory
│   └── sample              # Official sample vehicle configuration with single LiDAR and two cameras
├── third_party
├── tools -> /opt/apollo/neo/packages/bazel-extend-tools/latest/src
├── .vscode                 # Default VSCode configuration
│   ├── c_cpp_properties.json
│   └── settings.json
├── WORKSPACE               # Bazel configuration
└── .workspace.json         # Apollo project configuration, can specify software package versions here
```

Next, you can learn more about Apollo usage methods through practice tutorials:

- [Apollo Planning Practice (Chinese)](https://vscode-remote+ssh-002dremote-002b172-002e20-002e102-002e48.vscode-resource.vscode-cdn.net/home/apollo/faban/apollo/%E5%BA%94%E7%94%A8%E5%AE%9E%E8%B7%B5/%E5%BC%80%E5%8F%91%E8%B0%83%E8%AF%95%E6%95%99%E7%A8%8B/Apollo%E8%A7%84%E5%88%92%E5%AE%9E%E8%B7%B5/%E8%A7%84%E5%88%92%E6%A8%A1%E5%9D%97%E7%BB%BC%E8%BF%B0.md)

- [Apollo Perception Practice (Chinese)](https://vscode-remote+ssh-002dremote-002b172-002e20-002e102-002e48.vscode-resource.vscode-cdn.net/home/apollo/faban/apollo/%E5%BA%94%E7%94%A8%E5%AE%9E%E8%B7%B5/%E5%BC%80%E5%8F%91%E8%B0%83%E8%AF%95%E6%95%99%E7%A8%8B/Apollo%E6%84%9F%E7%9F%A5%E5%AE%9E%E8%B7%B5/%E6%84%9F%E7%9F%A5%E6%A8%A1%E5%9D%97%E7%BB%BC%E8%BF%B0.md)

## Step 5: Delete Project (Optional)

This step explains how to delete an installed project.

### 1. Delete Container

Taking the `application-core` project as an example:

```shell
# First enter the project directory
cd application-core
# Delete the container
aem remove
```

### 2. Delete Project

```shell
# Go back to the parent directory
cd ..
# Delete the project directory
rm -r application-core
```
