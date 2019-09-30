HOW TO UNDERSTAND ARCHITECTURE AND WORKFLOW
===========================================

## Fundamentals to understand AplloAuto - core

Autonomous vehicles \(AV\) dynamics are controlled by the planning engine through the Controller Area Network bus \(CAN bus\). The software reads data from hardware registers and writes them back just like we would in Assembly language. For high-precision computation, the Localization, Perception and Planning modules function as independent
input sources, while output sources work together though the Peer2Peer (P2P) protocol. P2P is supported by the RPC network application.

ApolloAuto uses ROS1 as the underlying network which means that ApolloAuto borrows the Master-Nodes framework from ROS1. Since xmlRPC from ROS1 is really old \(compared
to the recent brpc and [grpc](https://yiakwy.github.io/blog/2017/10/01/gRPC-C-CORE)\), Baidu has developed its own protobuf version of RPC.

In Baidu ApolloAuto, three stages of development have already been described

1. Dreamviewer Offline Simulation Engine & ApolloAuto core software module
   - Get a first taste on how the algorithms work for a car
   - We don't need to touch a real car or hardware and start development immediately
2. Core modules Integration:
   - Localization
   - Perception \(support third parties' solution like Mobileye ES4 chip based camera for L2 development\) process point cloud data from `Lidar` and return segmented objects info on request
   - Planning: compute the fine-tuned path, car dynamic controlling info for path segments from route service
   - Routine: local implementation of finding path segments through `Navigator` interface; Using A\*star algorithm.
3. HD Maps. One of the key differences from L2 level AV development. L4 AV machine needs Hdmap. Since a robot \(an autonomous vehicle \) needs to rebuild
3d world \(please check OpenCV [SLAM]() chapter\) in its microcomputer, reference object coordinates play a great role in relocating AV both in the map and the real world.
4. Cloud-based Online Simulation Drive Scenario Engine and Data Center.
   - As a partner of Baidu, you will be granted docker credentials to commit new images and replay the algorithm you developed on the cloud.
   - Create and manage complex scenarios to simulate real-world driving experiences

## ROS underlying Subscription and Publication mechanism and ApolloAuto modules structure


#### ROS underlying Subscription and Publication mechanism

So how does ROS1 based system communicate with each other and how does ApolloAuto make use of it? ROS has [tutorials](http://wiki.ros.org/ROS/Tutorials), and I will explain it
quickly before we analyze ApolloAuto modules structure.

ROS is a software, currently exclusively well supported by Ubuntu series. It has master roscore.

> printenv | grep ROS

default ros master uri is "http://localhost:11311. One can create an independent binary by performing ros::init and start it by performing ros::spin \(some kind of Linux event loop\)
using c++ or python. The binary behind the freshly created package is called ***ROS node***. The node will register its name and IP address in Master in case of other nodes querying. Nodes communicate
with each by directly constructing a TCP connection.

If a node wants to read data from others, we call it subscriber. The typical format is

```
... bla bla bla
ros::NodeHandle h;
ros::Subscriber sub = h.subscribe("topic_name", q_size, cb)
.. bla bla bla
```

If a node wants to provide data for subscribers to read, we call it a publisher. The typical format is

```
... bla bla bla
ros::NodeHandle h;
ros::Publisher pub = h.advertise<generated_msg_format_cls>("topic_name", q_size)
... bla bla bla
```

cb here is a callback executed when Linux kernel IO is ready. With these signatures bearing in mind, we can quickly analyze ApolloAuto
module structures before diving deep into core modules implementation.

#### apolloauto modules structure

I have conducted full research about it but I cannot show you all of them. ApolloAuto modules/common/ provide basic micros to control ros::spin for each
module and /modules/common/adaptor contains the most information on how a topic is registered. Every module will be registered from the [point](https://github.com/yiakwy/apollo/blob/master/modules/common/adapters/adapter_manager.cc#L50)
. By reading configuration file defined ${MODULE_NAME}/conf, we can get basic information about topics a module subscribe and publish.

Each module starts by firing "Init" interface and register callbacks. If you want to step by step debug ApolloAuto in gdb, make sure you have added breakpoints in those back. This also
demonstrate that if you don't like what implemented by Baidu, just override the callback.

## Data preprocessing and Extended Kalman Filter

Kalman Filter is mathematical interactive methods to converge to real estimation without knowing the whole real\-time input sequence. No matter what kind of data you need to process, you can
rely on Kalman Filter. Extended Kalman Filter is used for 3d rigid movements in a matrix format. It is not hard. I recommend you a series tutorial from United States F15 director
[Michel van Biezen](https://www.youtube.com/watch?v=CaCcOwJPytQ).

Since it is used in input data preprocessing, you might see it in HD Maps, perception, planning and so on so forth.

## Selected modules analysis

#### HMI & Dreamviewer

There is not too much about hmi interface and dreamviewer but it is a good place to visualize the topics parameters.

HMI is a simply simple python application based on Flask.
Instead of using HTTP, it uses web socket to query ROS modules application. If you have experience on asynchronous HTTP downloaders, it is easy to understand, that an HTTP connection is just a
socket connection file descriptor which we have already write HTTP headers, methods into that buffer. Once hmi flask backend receives a command, it will execute a subprocess
to execute the corresponding binary.

Dreamviewer, in contrast, works a little bit like frontend app written in React, Webpack, and Threejs \( WebGL, see /dreamview/backend/simulation_world, /dreamview/frontend/src/render \),
techniques. It subscribes to messages from ROS nodes and draws it a frame after a frame.

#### Perception

Initially, this module implemented logics exclusively for Lidar and Radar processes. It is registered by AdapterManager as a ros node functioning as an info fusion system to
output observed Obstacles info. In the latest version of the codes, different hardware input handlers of ROS nodes are specified in /perception/obstacles/onboard and implemented in
different parallel locations, which consists of *Lidar, Radar, Traffic lights and GPS*.

1. Lidar:
   - HD Maps: get transformation matrix convert point world coordinates to local coordinates and build map polygons
   - ROI filter: get ROI and perform Kalman Filter on input data
   - Segmentation: A U-Net based \(a lot of variants\) Caffe model will be loaded and perform forward computation based on data from HD Maps and ROI filtering results
   - Object Building: Lidar return points \(x, y, z\). Hence you need to group them into "Obstacles" \(vector or set\)
   - Obstacles Tracker: Baidu is using HM solver from Google. For a large bipartite graph, KM algorithms in Lagrange format is usually deployed since
     SGD is extremely simple for that.

2. Radar:
   - Similar to Lidar with raw\_obstacles info from sensors.
   - ROI filter: get ROI objects and perform Kalman Filter on input data
   - Objects Tracker

3. Probability Fusion\(New in Apollo 1.5!\):
   - As far as I can understand, fusion system in ApolloAuto
   - It is typically one of most important parts: collects all the info and makes a final combination of information from sensors on the motherboard
     for track lists and rule-based cognitive engine
   - The major process is the association, hence HM algorithms here is used again as a bipartite graph.
   - Tracklists are maintained along timestamps and each list will be updated based on a probabilistic rules engine
