# Software FAQs

## Can other operating systems besides Ubuntu be used?

We have only tested on Ubuntu which means it's the only operating system we currently officially support. Ubuntu is an ideal operating system as ROS is required to support it. Users are always welcome to try different operating systems and can share their patches with the community if they are successfully able to use them.

---
## I’m having difficulty connecting to localhost:8888 (Dreamview).

The Dreamview web server is provided by the dreamview node(A node is an executable in ROS concept). Before accessing the Dreamview page, you need to build the system(including dreamview node) within the docker container following the [guide](https://github.com/ApolloAuto/apollo/blob/master/README.md). Once built, dreamview node will be started after the step `bash scripts/bootstrap.sh`.

So if you can not access Dreamview, please check:

* Make sure you have dreamview process running correctly. In the latest version, `bash scripts/bootstrap.sh` will report `dreamview: ERROR (spawn error)` if dreamview fails to start. For early version, please check with command: `supervisorctl status dreamview` or `ps aux | grep dreamview`. If dreamview is not running, please refer to [How to Debug a Dreamview Start Problem](https://github.com/ApolloAuto/apollo/blob/master/docs/howto/how_to_debug_dreamview_start_problem.md).
* Make sure the address and port are not blocked by the firewall.
* Make sure you're using <apollo_host_ip>:8888 instead of localhost:8888 if you are not accessing the Dreamview page through the host machine.

---
## How can I perform step-by-step debugging?

The majority of bugs can be found through logging (using AERROR, AINFO, ADEBUG). If step-by-step debugging is needed, we recommend using gdb.

---
## How do I run the Offline Perception Visualizer?

Refer to the How-To guide located [here](https://github.com/ApolloAuto/apollo/blob/master/docs/howto/how_to_run_offline_perception_visualizer.md).

---
## Ubuntu 14.04 login loop problem after installing the pre-built Apollo kernel.

Here is a solution to solve this problem:
* Reboot ubuntu system, and press and hold 'shift' button and then enter grub menu.
* Choose a generic and bootable item(not boot loader with Apollo kernel) to boot up.
* Press 'Ctrl+Alt+F1' and install 'NVIDIA-Linux-x86_64-375.39.run' according to the [link](https://github.com/ApolloAuto/apollo/blob/master/docs/quickstart/apollo_2_0_hardware_system_installation_guide_v1.md).
* Reboot and start computer with the default boot loader with Apollo kernel.

---
## Could I build Apollo on Ubuntu 18

**We highly recommend that you use Ubuntu 14.04 to build Apollo.**

Yes, you should be able to make Apollo work on Ubuntu 18. Just follow the document to install docker, then `apollo.sh build` and `apollo.sh lint` should work. But `apollo.sh test` might not, because the nvidia-drivers are generally miss-matched between HOST and Docker container, which will raise test failures.

---
## How do I add a new module
Apollo currently functions as a single system, therefore before adding a module to it, understand that there would be a lot of additional work to be done to ensure that the module functions perfectly with the other modules of Apollo. Simply add your module to the `modules/` folder. You can use `modules/routing` as an example, which is a relatively simple module. Write the BUILD files properly and apollo.sh will build your module automatically

---
## Rosbag: not found - What now?

Should you see this error while building Apollo, please confirm the following:

1. Make sure you are running rosbag when you are inside docker (dev or release). We recommend using the dev docker container.
2. You are probably running it as `root`. If yes, you would need to add:
    ```
    source /apollo/scripts/apollo_base.sh (this script includes the ros environment setup procedure source /home/tmp/ros/setup.bash(for dev docker), or you can only source the ros env setup script as well).
    ```
    Usually, this could be avoid by running Apollo as a normal user, since it will be configured into .bashrc automatically.

3. Do not use `sudo` parameter, when executing `dev_start.sh` and `dev_into.sh`

---

## Build error "docker: Error response from daemon: failed to copy files: userspace copy failed": 

An error message like this means that your system does not have enough space to build Apollo and the build process will fail. To resolve this issue, run the following to free up some space:
```
docker/setup_host/cleanup_resources.sh 
```
If it does not work, delete the Apollo repo, free up some space and then try again.

---
## Bootstrap error: unix:///tmp/supervisor.sock refused connection

There could be a number of reasons why this error occurs. 
Please follow the steps recommended in the [following thread](https://github.com/ApolloAuto/apollo/issues/5344). There are quite a few suggestions. If it still does not work for you, comment on the thread mentioned above.

---
## My OS keeps freezing when building Apollo 3.5?

If you see an error like this, you do not have enough memory to build Apollo. Please ensure that you have at least **16GB** memory available before building Apollo.
You could also find `--jobs=$(nproc)` in apollo.sh file and replace it with `--jobs=2`. This will make build process to use only 2 cores. Building will be longer, but will use less memory.

---
**More Software FAQs to follow.**
