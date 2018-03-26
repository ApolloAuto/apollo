# Overview of Apollo

Apollo has been initiated to provide an open, comprehensive, and reliable software platform for its partners in the automotive and autonomous-driving industries. Partners can use the Apollo software platform and the reference hardware that Apollo has certified as a template to customize in the development of their own autonomous vehicles.

# Apollo Software Installation

This section includes:

- Download the Apollo Release Package
- Set up Docker Support
- Customize Your Release Container

Before getting started, please make sure you have installed the Ubuntu Linux 14.04.3 and the Apollo Kernel following the steps in the [Apollo 1.0 Hardware and System Installation Guide](https://github.com/ApolloAuto/apollo/blob/master/docs/quickstart/apollo_1_0_hardware_system_installation_guide.md#installing-the-software-for-the-ipc).

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

## Set Up Docker Support

The Docker container is the simplest way to set up the build environment for Apollo.

For more information, see the detailed Docker tutorial [here](https://docs.docker.com/).

1. Please follow the
[official guide to install the docker-ce](https://docs.docker.com/install/linux/docker-ce/ubuntu).

Don't forget the
[post-installation steps for Linux](https://docs.docker.com/install/linux/linux-postinstall).

2. After the installation, log out and then log back into the system to enable Docker.

3. (Optional) If you already have Docker installed (before you installed the Apollo Kernel), add the following line in `/etc/default/docker`:

    ```
    DOCKER_OPTS = "-s overlay"
    ```

## Use Your Release Container

1. Download and start the Apollo Release docker image by running the following commands:

    ```
    cd $APOLLO_HOME
    bash docker/scripts/release_start.sh
    ```

2. (Optional) If you want to customize your release container, login into the Apollo Release docker image by running the following commands:

    ```
    bash docker/scripts/release_into.sh
    ```

3. (Skip this if you only want to do the offline simulation in release docker container) Set up the zone number for the Global Navigation Satellite System (GNSS) Driver by modifying the following line in file `./ros/share/gnss_driver/launch/gnss_driver.launch`.

    ```
    <arg name="proj4_text" default="+proj=utm +zone=10 +ellps=WGS84 +towgs84=0,0,0,0,0,0,0 +units=m +no_defs " />
    ```

    You only have to modify the value `+zone=10` in the above line. Please refer to the [Apollo's Coordinate System](https://github.com/ApolloAuto/apollo/blob/master/docs/specs/coordination.pdf) to find your local zone number. For example, if you are in Beijing, China, you have to set `+zone=50`.

4. (Skip this if you only want to do the offline simulation in release docker container) Set up the Real Time Kinematic (RTK) Base Station for the GNSS Driver by modifying the file: `./ros/share/gnss_driver/conf/gnss_conf_mkz.txt`

    Refer to the following example for a typical RTK setup:

    ```
    rtk_from {
	format: RTCM_V3
		ntrip {
		    address: <provide your own value>
		    port: <provide your own value>
		    mount_point: <provide your own value>
		    user: <provide your own username>
		    password: <provide your own password>
		    timeout_s: <provide your own value, e.g., 5>
	    }
    }
    rtk_to {
	    format: RTCM_V3
	    serial {
		    device: <provide your own value, e.g., "/dev/ttyUSB1">
		    baud_rate: <provide your own value, e.g., 115200>
	    }
    }
    ```

    The `rtk_from` is  used for RTK base station information. The `rtk_to` is used to send the RTK differential data to the receiver.

5. (Skip this if you only want to do the offline simulation in release docker container) Add ESD CAN Support

    Please refer to [ESD CAN README](https://github.com/ApolloAuto/apollo/blob/master/third_party/can_card_library/esd_can/README.md) to install the ESD CAN library.

6. (Skip this if you have NOT customized your release docker container) Follow these steps to persist your local changes:

    ```
    # EXIT DOCKER ENV
    # commit your docker local changes to local docker image.
    exit # exit from docker environment
    cd $APOLLO_HOME
    bash docker/scripts/release_commit.sh
    ```

7. Start your favorite browser (i.e. Chrome) and with URL: http://localhost:8888