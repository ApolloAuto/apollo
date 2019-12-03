# Software Overview of Apollo

Apollo has been initiated to provide an open, comprehensive, and reliable software platform for its partners in the automotive and autonomous-driving industries. Partners can use the Apollo software platform and the reference hardware that Apollo has certified as a template to customize in the development of their own autonomous vehicles.

# Apollo Software Installation

This section includes:

- [Download the Apollo Release Package](#download-apollo-source)
- [Set up the Docker environment](#Set-up-the-Docker-environment)
- [Support a new Vehicle in DreamView](#Support-a-new-Vehicle-in-DreamView)
- [Install Apollo Kernel](#Install-apollo-kernel)
- [Run Apollo in Ubuntu 16](#Run-Apollo-in-Ubuntu-16)

Before getting started, please make sure you have installed Ubuntu Linux 14.04.3 and the Apollo Kernel following the steps in the [Apollo core Software Installation Guide](https://github.com/ApolloAuto/apollo/blob/master/docs/quickstart/apollo_1_0_hardware_system_installation_guide.md#installing-the-software-for-the-ipc).

## Download Apollo Source

1. Download Apollo source code from the [github source](https://github.com/ApolloAuto/apollo/) and check out the correct branch:

    ```
    git clone git@github.com:ApolloAuto/apollo.git
    cd apollo
    git checkout [release_branch_name]
    ```

2. Set up environment variable `APOLLO_HOME` by the following command:

    ```
    echo "export APOLLO_HOME=$(pwd)" >> ~/.bashrc && source ~/.bashrc
    ```

3. Open a new terminal or run `source ~/.bashrc` in an existing terminal.


![tip](images/tip_icon.png) In the following sections, it is assumed that the Apollo directory is located in  `$APOLLO_HOME`.

## Set Up the Docker Environment

The Docker container is the simplest way to set up the build environment for Apollo.

For more information, see the detailed Docker tutorial [here](https://docs.docker.com/).

1. Please follow the [official guide to install the docker-ce 19.03+](https://docs.docker.com/install/linux/docker-ce/ubuntu).

Don't forget the [post-installation steps for Linux](https://docs.docker.com/install/linux/linux-postinstall).

2. After the installation, log out and then log back into the system to enable Docker.

3. (Optional) If you already have Docker installed (before you installed the Apollo Kernel), add the following line in `/etc/default/docker`:

    ```
    DOCKER_OPTS = "-s overlay"
    ```

4. Install latest nvidia-container-toolkit by following the [official doc](https://github.com/NVIDIA/nvidia-docker).

We encourage you to continue the Build process using [Build the Dev docker environment](https://github.com/ApolloAuto/apollo/blob/master/docs/howto/how_to_build_and_release.md#build_release) if you have not already set it up.

## Support a new Vehicle in DreamView

In order to support a new vehicle in DreamView, please follow the steps below:

1. Create a new folder for your vehicle under `modules/calibration/data`

2. There is already a sample file in the `modules/calibration/data` folder named `mkz_example`. Refer to this structure and include all necessary configuration files in the same file structure as “mkz_example”. Remember to update the configuration files with your own parameters if needed. 

3. Restart DreamView and you will be able to see your new vehicle (name is the same as your newly created folder) in the selected vehicle.

## Install Apollo Kernel

The Apollo runtime in the vehicle requires the [Apollo Kernel](https://github.com/ApolloAuto/apollo-kernel). It is strongly recommended to install the pre-built kernel.

### Use the pre-built Apollo Kernel

You get access to and install the pre-built kernel using the following commands.

1. Download the release packages from the release section on GitHub:

```
https://github.com/ApolloAuto/apollo-kernel/releases
```

2. Install the kernel after having downloaded the release package:

```
tar zxvf linux-4.4.32-apollo-1.5.0.tar.gz
cd install
sudo bash install_kernel.sh
```

3. Reboot your system using the `reboot` command.
4. [Optional - if you are using CAN card for interface] Build the CAN driver source code, according to the vendor's instructions

### Build your own kernel

If you have modified the kernel, or the pre-built kernel is not the best for your platform, you can build your own kernel using the following steps:

1. Clone the code from the repository

```
git clone https://github.com/ApolloAuto/apollo-kernel.git
cd apollo-kernel
```

2. Add the CAN driver source code according to vendor's instruction
3. Build the kernel using the following command:

```
bash build.sh
```

4. Install the kernel using the steps for a pre-built Apollo Kernel as described in the previous section.

## Run Apollo in Ubuntu 16

Please refer to
[How to run Apollo with Ubuntu 16](https://github.com/ApolloAuto/apollo/blob/master/docs/howto/how_to_run_apollo_2.5_with_ubuntu16.md)
