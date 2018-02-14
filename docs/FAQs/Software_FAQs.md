# Software FAQs

## Can other operating systems besides Ubuntu be used?

We have only tested on Ubuntu which means it's the only operating system we currently officially support. Ubuntu is an ideal operating system as ROS is required to support it. Users are always welcome to try different operating systems and can share their patches with the community if they are successfully able to use them.

## I’m having difficulty connecting to localhost:8888 (Dreamview).

Before running the Dreamview page, you need to build the system within the docker container using 'bash apollo.sh build'. Once built, if you are not accessing the Dreamview page through the host machine, you should use <apollo_host_ip>:8888 for Dreamview, instead of localhost:8888.

## How can I perform step-by-step debugging?

The majority of bugs can be found through logging (using AERROR, AINFO, ADEBUG). If step-by-step debugging is needed, we recommend using GDP.

## How do I run the Offline Perception Visualizer?

Refer to the How-To guide located [here](https://github.com/ApolloAuto/apollo/blob/master/docs/howto/how_to_run_offline_perception_visualizer.md).

## Ubuntu 14.04 login loop problem after installing the pre-built Apollo kernel.

Here is a solution to solve this problem:
1. reboot ubuntu system, and press and hold 'shift' button and then enter grub menu.
2. choose a generic and bootable item(not boot loader with Apollo kernel) to boot up.
3. press 'Ctrl+Alt+F1' and install 'NVIDIA-Linux-x86_64-375.39.run' according to the [link](https://github.com/ApolloAuto/apollo/blob/master/docs/quickstart/apollo_2_0_hardware_system_installation_guide_v1.md).
4. reboot and start computer with the default boot loader with Apollo kernel.


**More Software FAQs to follow.**
