# General FAQs

## I am new to the Apollo project, where do I start?
You have several options:

- To build apollo on your computer, start by reviewing the [README.md](https://github.com/ApolloAuto/apollo/blob/master/README.md)

- To run the Apollo demo offline, go to: [Apollo README.md](https://github.com/ApolloAuto/apollo/blob/master/docs/demo_guide/README.md).

- To install and build Apollo on a vehicle, go to: [Apollo 1.0 quick start](https://github.com/ApolloAuto/apollo/blob/master/docs/quickstart/apollo_1_0_quick_start.md).

- To build the Apollo Kernel, the Robot Operating System (ROS), and Apollo, go to: [apollo/docs/quickstart/apollo_1_0_quick_start_developer.md](https://github.com/ApolloAuto/apollo/blob/master/docs/quickstart/apollo_1_0_quick_start_developer.md) and refer to [build kernel](https://github.com/ApolloAuto/apollo/blob/master/docs/quickstart/apollo_1_0_quick_start_developer.md#build-the-apollo-kernel).

---
## How do I send a pull request?
Sending a pull request is simple.
1.	Fork the Apollo Repository into your GitHub.
2.	Create a Developer Branch in your Repository.
3.	Commit your change in your Developer Branch.
4.	Send the pull request from your GitHub Repository Webpage.

---
## Do comments need to be made in Doxygen?
Yes, currently all comments need to be made in Doxygen.

---
## How to debug build problems?
1. Carefully review the instructions in the documentation for the option that you selected to get started with the Apollo project.

2. Make sure that you follow the steps in the document exactly as they are written.

3. Use Ubuntu 14.04 as the build can only be implemented using Linux.

4. Verify that the Internet setting is correct on your computer.

5. Allocate more than 1GB of memory, at the recommended minimum, for your computer.

6. If roscore cannot start in apollo docker, you may need to tune the master start timeout value in ROS. You may want to check a related user-reported [issue](https://github.com/ApolloAuto/apollo/issues/2500) for more details.

---
## If I cannot solve my build problems, what is the most effective way to ask for help?
Many build problems are related to the environment settings.

1. Run the script to get your environment: `bash scripts/env.sh >& env.txt`

2. Post the content of env.txt to our Github issues page and someone from our team will get in touch with you.

---
## Which ports must be whitelisted to run Apollo in a public cloud instance?
Use these ports for HMI and Dreamview:
- 8888: Dreamview

---
## Why there is no ROS environment in dev docker?
The ROS package is downloaded when you start to build apollo:
`bash apollo.sh build`.

1. Run the following command inside Docker to set up the ROS environment after the build is complete:
`source /apollo/scripts/apollo_base.sh`

2. Run ROS-related commands such as rosbag, rostopic and so on.

---
## How do I clean the existing build output?
Follow these steps:

1. Log into Docker using the command:
`bash docker/scripts/dev_into.sh`

2. Run the command:
`bash apollo.sh clean`

---
## How do I delete the downloaded third party dependent packages?
Follow these steps:

1. Log into Docker using the command:
`bash docker/scripts/dev_into.sh`

2. Run the command:
`bazel clean --expunge`
The build command, `bash apollo.sh build`, then downloads all of the dependent packages according to the *WORKSPACE* file.


**More General FAQs to follow.**
