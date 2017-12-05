# Apollo

[![Build Status](https://travis-ci.org/ApolloAuto/apollo.svg?branch=master)](https://travis-ci.org/ApolloAuto/apollo)

```
We choose to go to the moon in this decade and do the other things,
not because they are easy, but because they are hard.
-- John F. Kennedy, 1962
```

Welcome to the Apollo GitHub.

[Apollo](http://apollo.auto) is an open autonomous driving platform. It is a high performance flexible architecture which supports fully autonomous driving capabilities.
For business contact, please visit http://apollo.auto


## Installation

We strongly recommend building Apollo in our pre-specified Docker environment.
See the following instructions on how to set up the docker environment and build from source.

### The docker environment can be set by the commands below.

```
bash docker/scripts/install_docker.sh
# logout and login the computer to make sure to run docker command without sudo
docker ps  # to verify docker works without sudo
# in case you forgot to logout and login back, do so, remove ~/.docker/config.json
# and check again with `docker ps`
bash docker/scripts/dev_start.sh
bash docker/scripts/dev_into.sh
```
### To build from source

```
bash apollo.sh build
```

## Run Apollo
Follow the steps below to launch Apollo:
### Start Apollo
```
# start module monitor
bash scripts/bootstrap.sh
```
### Access Dreamview
Access Dreamview by opening your favorite browser, e.g. Chrome, go to http://localhost:8888
![Access Dreamview](docs/demo_guide/images/apollo_bootstrap_screen.png)


### Replay demo rosbag
```
# in a different terminal, in the apollo directory
bash docker/scripts/dev_into.sh # jump into the docker container
bash ./docs/demo_guide/rosbag_helper.sh download # download rosbag
rosbag play -l ./docs/demo_guide/demo_1.5.np.bag
```

Dreamview should show a running vehicle now. (The following image might be different due to changes in frontend.)
![Dreamview with Trajectory](docs/demo_guide/images/dv_trajectory_1.5.png)

Advanced users who wish to build outside this Docker container can refer
to the corresponding Docker specification file (`./docker/dev.dockerfile`).

## Documents
Apollo documents can be found under the [docs](https://github.com/ApolloAuto/apollo/blob/master/docs/) repository.
   * [quickstart](https://github.com/ApolloAuto/apollo/blob/master/docs/quickstart/): the quickstart tutorial.
   * [demo_guide](https://github.com/ApolloAuto/apollo/blob/master/docs/demo_guide/): the guide for demonstration.
   * [![Apollo Offline Demo](https://img.youtube.com/vi/Q4BawiLWl8c/0.jpg)](https://www.youtube.com/watch?v=Q4BawiLWl8c)
   * [how to contribute code](https://github.com/ApolloAuto/apollo/blob/master/CONTRIBUTING.md): the guide for contributing code to Apollo.
   * [howto](https://github.com/ApolloAuto/apollo/blob/master/docs/howto/): tutorials on how to build, run and modify codes.
   * [specs](https://github.com/ApolloAuto/apollo/blob/master/docs/specs/): Specification documents of Apollo 1.5.
   * [Doxygen APIs](https://apolloauto.github.io/doxygen/apollo/): Apollo Doxygen pages

## Ask Questions

You are welcome to submit questions and bug reports as [Github Issues](https://github.com/ApolloAuto/apollo/issues).

## Copyright and License
Apollo is provided under the [Apache-2.0 license](LICENSE).

## Disclaimer
Please refer the Disclaimer of Apollo in [Apollo official website](http://apollo.auto/docs/disclaimer.html).
