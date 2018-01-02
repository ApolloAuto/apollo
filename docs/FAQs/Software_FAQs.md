# Software FAQs

## Can other operating systems besides Ubuntu be used?

We have only tested on Ubuntu which means it's the only operating system we currently officially support. Ubuntu is an ideal operating system as ROS is required to support it. Users are always welcome to try different operating systems and can share their patches with the community if they are successfully able to use them.

## I’m having difficulty connecting to localhost:8888 (Dreamview).

Before running the Dreamview page, you need to build the system within the docker container using 'bash apollo.sh build'. Once built, if you are not accessing the Dreamview page through the host machine, you should use <apollo_host_ip>:8888 for Dreamview, instead of localhost:8888.

## How can I perform step-by-step debugging?

The majority of bugs can be found through logging (using AERROR, AINFO, ADEBUG). If step-by-step debugging is needed, we recommend using GDP.

## How do I run the Offline Perception Visualizer?

Refer to the How-To guide located [here](https://github.com/ApolloAuto/apollo/blob/master/docs/howto/how_to_run_offline_perception_visualizer.md).

**More Software FAQs to follow.**
