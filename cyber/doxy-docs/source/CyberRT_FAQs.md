F.A.Q.
=======

## What is Apollo Cyber RT?

Apollo's Cyber RT is an open source runtime framework designed specifically for autonomous driving scenarios. Based on a centralized computing model, it is highly optimized for performance, latency, and data throughput

---

## Why did we decide to work on a new runtime framework?

- During years of development of autonomous driving technologies, we have learned a lot from our previous experience with Apollo. In autonomous driving scenarious, we need an effective centralized computing model, with demands for high performance,  including high concurrency, low latency and high throughput。

- The industry is evolving, so does Apollo. Going forward, Apollo has already moved from development to productization, with volume deployments in the real world, we see the demands for the highest robustness and high performance. That’s why we spent years of building Apollo Cyber RT, which addresses that requirements of autonomous driving solutions.


---

## What are the advantages of the new runtime framework?

- Accelerate development
  + Well defined task interface with data fusion
  + Array of development tools
  + Large set of sensor drivers
- Simplify deployment
  + Efficient and adaptive message communication
  + Configurable user level scheduler with resource awareness
  + Portable with fewer dependencies
- Empower your own autonomous vehicles
  + The default open source runtime framework
  + Building blocks specifically designed for autonomous driving
  + Plug and play your own AD system

---

## Can we still use the data that we have collected?

-	If the data you have collected is compatible with the previous versions of Apollo, you could use our recommended conversion tools to make the data compliant with our new runtime framework
-	If you created a customized data format, then the previously generated data will not be supported by the new runtime framework

---

## Will you continue to support ROS?

We will continue to support previous Apollo releases (3.0 and before) based on ROS. We do appreciate you continue growing with us and highly encourage you to move to Apollo 3.5. While we know that some of our developers would prefer to work on ROS, we do hope you will understand why Apollo as a team cannot continue to support ROS in our future releases as we strive to work towards developing a more holistic platform that meets automotive standards.

---

## Will Apollo Cyber RT affect regular code development?

If you have not modified anything at runtime framework layer and have only worked on Apollo's module code base, you will not be affected by the introduction of our new runtime framework as most of time you would only need to re-interface the access of the input and output data. Additional documents are under [cyber](https://github.com/ApolloAuto/apollo/tree/master/docs/cyber/) with more details.

---

## Recommended setup for Apollo Cyber RT

- Currently the runtime framework only supports running on Trusty (Ubuntu 14.04)
- The runtime framework also uses apollo's docker environment
- It is recommended to run source setup.bash when opening a new terminal
- Fork and clone the Apollo repo with the new framework code which can be found at [apollo/cyber](https://github.com/ApolloAuto/apollo/tree/master/cyber/)

## How to enable SHM to decrease the latency?

To decrease number of threads, the readable notification mechanism of shared memory was changed in CyberRT. The default mechanism is UDP multicast, and system call(sendto) will cause some latency.

So, to decrease the latency, you can change the mechanism, The steps are listed as following:
1. update the CyberRT to the latest version;
2. uncomment the transport_conf in https://github.com/ApolloAuto/apollo/blob/master/cyber/conf/cyber.pb.conf;
3. change **notifier_type** of **shm_conf** from "multicast" to "condition";
4. build CyberRT with opt like `bazel build -c opt --copt=-fpic //cyber/...`;
5. run talker and listener;

Note:You can select the corresponding transmission method according to the relationship between nodes.For example, the default configuration is **INTRA** in the process, **SHM** between the host process, and **RTPS** across the host.
Of course you can change all three to RTPS. Or change `same_proc` and `diff_proc` to **SHM**;

## How to use the no serialization message?

The message types supported by Cyber RT include both serializable structured data like protobuf and raw sequence of bytes.
You can refer the sample code:
- apollo::cyber::message::RawMessage
- talker: https://github.com/gruminions/apollo/blob/record/cyber/examples/talker.cc
- listener: https://github.com/gruminions/apollo/blob/record/cyber/examples/listener.cc

## How to configure multiple hosts communication?

Make sure the two hosts(or more) are under the same network segment of the local area network, Like `192.168.10.6` and `192.168.10.7`.

You just need to modify `CYBER_IP` of `/apollo/cyber/setup.bash`

```bash
export CYBER_IP=127.0.0.1
```

Suppose you have two hosts A and B，the ip of A is `192.168.10.6`, and the ip of B is `192.168.10.7`. Then set `CYBER_IP` to `192.168.10.6` on host A, and set `CYBER_IP` to `192.168.10.7` on host B. Now host A can communicate with host B.

---
More FAQs to follow...
