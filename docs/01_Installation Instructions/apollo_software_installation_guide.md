# Apollo Software Installation Guide

This document describes the steps required to install Apollo on Ubuntu 18.04.5
LTS (Bionic Beaver), the recommended Ubuntu release for Apollo 6.0.

## Pre-requisites

Before getting started, please make sure all the pre-requisite steps were
finished as described in the
[Pre-requisite Software Installation Guide](./prerequisite_software_installation_guide.md).

Please also make sure Docker is running. Type `systemctl status docker` to check
the running status of Docker daemon, and type `systemctl start docker` to start
Docker if needed.

## Download Apollo Sources

Run the following commands to clone
[Apollo's GitHub Repo](https://github.com/ApolloAuto/apollo.git).

```bash
# Using SSH
git clone git@github.com:ApolloAuto/apollo.git

# Using HTTPS
git clone https://github.com/ApolloAuto/apollo.git

```

And checkout the latest branch:

```bash
cd apollo
git checkout master
```

For CN users, please refer to
[How to Clone Apollo Repository from China](./how_to_clone_apollo_repo_from_china.md)
if your have difficulty cloning from GitHub.

(Optional) For convenience, you can set up environment variable
`APOLLO_ROOT_DIR` to refer to Apollo root directory by running:

```bash
echo "export APOLLO_ROOT_DIR=$(pwd)" >> ~/.bashrc  && source ~/.bashrc
```

![tip](images/tip_icon.png) In the following sections, we will refer to Apollo
root directory as `$APOLLO_ROOT_DIR`

## Start Apollo Development Docker Container

From the `${APOLLO_ROOT_DIR}` directory, type

```bash
bash docker/scripts/dev_start.sh
```

to start Apollo development Docker container.

If successful, you will see the following messages at the bottom of your screen:

```bash
[ OK ] Congratulations! You have successfully finished setting up Apollo Dev Environment.
[ OK ] To login into the newly created apollo_dev_michael container, please run the following command:
[ OK ]   bash docker/scripts/dev_into.sh
[ OK ] Enjoy!
```

## Enter Apollo Development Docker Container

Run the following command to login into the newly started container:

```bash
bash docker/scripts/dev_into.sh
```

## Build Apollo inside Container

From the `/apollo` directory inside Apollo Docker container, type:

```bash
./apollo.sh build
```

to build the whole Apollo project.

Or type

```bash
./apollo.sh build_opt
```

for an optimized build.

You can refer to
[Apollo Build and Test Explained](./apollo_build_and_test_explained.md)
for a thorough understanding of Apollo builds and tests.

## Launch and Run Apollo

Please refer to the
[Run Apollo](./how_to_launch_and_run_apollo.md#run-apollo) section of
[How to Launch And Run Apollo](./how_to_launch_and_run_apollo.md).

## (Optional) Support a new Vehicle in DreamView

In order to support a new vehicle in DreamView, please follow the steps below:

1. Create a new folder for your vehicle under `modules/calibration/data`

2. There is already a sample file in the `modules/calibration/data` folder named
   `mkz_example`. Refer to this structure and include all necessary
   configuration files in the same file structure as “mkz_example”. Remember to
   update the configuration files with your own parameters if needed.

3. Restart DreamView and you will be able to see your new vehicle (name is the
   same as your newly created folder) in the selected vehicle.
