# FAQ
## I am new to this project, where should I start?
### --- If you do not have a vehicle
If you want to build apollo on your computer, please start with [README.md](https://github.com/ApolloAuto/apollo/blob/master/README.md)

If you don't want to build apollo and only want to run it for offline demo, please refer [apollo/docs/demo_guide/README.md](https://github.com/ApolloAuto/apollo/blob/master/docs/demo_guide/README.md).
### --- If you have a vehicle
If you are a user and would like to install and build Apollo, please refer [apollo/docs/quickstart/apollo_1_0_quick_start.md](https://github.com/ApolloAuto/apollo/blob/master/docs/quickstart/apollo_1_0_quick_start.md).

If you are a developer and would like to build the Apollo Kernel, the Robot Operating System (ROS), and Apollo, please refer [apollo/docs/quickstart/apollo_1_0_quick_start_developer.md](https://github.com/ApolloAuto/apollo/blob/master/docs/quickstart/apollo_1_0_quick_start_developer.md).

## How to debug build problems?
1. Please make sure you followed the guidance in the correct help document as listed in the first question.

2. Please make sure you followed exactly the same steps in the document.

3. Currently, build can only be done on Linux, we recommend Ubuntu 14.04.

4. Please double check the internet setting on your computer is correct.

5. Please allocate sufficient memory for your computer. It is recommended to have > 1GB memory.

## I cannot solve my build problems, what is the most effective way to ask for help?
So far, many build problems are related to environment setting. You can run the script to get your environment: `bash scripts/env.sh >& env.txt` and provide the content of env.txt in your post.

## Which ports need be white list to run Apollo in public cloud instance?
8887: HMI
8888: Dreamview
