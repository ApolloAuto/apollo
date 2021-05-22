# How to Develop Cyber RT inside Docker on Both x86_64 and ARM64

To make life easier, Apollo maintains Cyber Docker images and a number of
scripts to help developers build and play with Cyber RT framework.

Cyber Docker images are built upon Ubuntu 18.04, and comes with full support for
Cyber RT framework. For those who are interested in Cyber RT only, this would be
the ideal point to start with.

**Note: Apollo team also maintains ARM64 Cyber Docker images which was tested on
Jetson AGX Xavier. The same set of scripts are provided to run both on x86_64
and ARM64 platforms.**

In the next section, we will show you how to play with Cyber Docker images.

## Build and Test Cyber RT inside Docker

Run the following command to start Cyber Docker container:

```bash
bash docker/scripts/cyber_start.sh
```

Or if you are in China, run:

```bash
bash docker/scripts/cyber_start.sh -g cn
```

A Docker container named `apollo_cyber_$USER` will be started.

**Note**: You will lose all your previous changes in the container if you have
ran this command before. Unless you would like to start a fresh Docker
environment.

To log into the newly started Cyber container:

```bash
bash docker/scripts/cyber_into.sh
```

**Note**: you can login and logout Cyber container multiple times as you wish,
the Docker environment will stay there until the next time you start another
Cyber container.

To build Cyber RT only and run unit tests:

```bash
./apollo.sh build cyber
./apollo.sh test cyber
```

Or run with `--config=opt` for an optimized build/test.

```bash
./apollo.sh build --config=opt cyber
./apollo.sh test --config=opt cyber
```

You should be able to see that all the testcases passed.
