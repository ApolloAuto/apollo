# Software FAQs

## Can other operating systems besides Ubuntu be used?

We have only tested on Ubuntu which means it's the only operating system we currently officially support. Ubuntu is an ideal operating system as ROS is required to support it. Users are always welcome to try different operating systems and can share their patches with the community if they are successfully able to use them.

## I’m having difficulty connecting to localhost:8888 (Dreamview).

The Dreamview web server is provided by the dreamview node(A node is an executable in ROS concept). Before accessing the Dreamview page, you need to build the system(including dreamview node) within the docker container following the [guide](https://github.com/ApolloAuto/apollo/blob/master/README.md). Once built, dreamview node will be started after the step `bash scripts/bootstrap.sh`.

So if you can not access Dreamview, please check:

* Make sure you have dreamview process running correctly. In the latest version, `bash scripts/bootstrap.sh` will report `dreamview: ERROR (spawn error)` if dreamview fails to start. For early version, please check with command: `supervisorctl status dreamview` or `ps aux | grep dreamview`. If dreamview is not running, please refer to [How to Debug a Dreamview Start Problem](https://github.com/ApolloAuto/apollo/blob/master/docs/howto/how_to_debug_dreamview_start_problem.md).
* Make sure the address and port are not blocked by the firewall.
* Make sure you're using <apollo_host_ip>:8888 instead of localhost:8888 if you are not accessing the Dreamview page through the host machine.

## How can I perform step-by-step debugging?

The majority of bugs can be found through logging (using AERROR, AINFO, ADEBUG). If step-by-step debugging is needed, we recommend using GDP.

## How do I run the Offline Perception Visualizer?

Refer to the How-To guide located [here](https://github.com/ApolloAuto/apollo/blob/master/docs/howto/how_to_run_offline_perception_visualizer.md).

## Ubuntu 14.04 login loop problem after installing the pre-built Apollo kernel.

Here is a solution to solve this problem:
* Reboot ubuntu system, and press and hold 'shift' button and then enter grub menu.
* Choose a generic and bootable item(not boot loader with Apollo kernel) to boot up.
* Press 'Ctrl+Alt+F1' and install 'NVIDIA-Linux-x86_64-375.39.run' according to the [link](https://github.com/ApolloAuto/apollo/blob/master/docs/quickstart/apollo_2_0_hardware_system_installation_guide_v1.md).
* Reboot and start computer with the default boot loader with Apollo kernel.


**More Software FAQs to follow.**
