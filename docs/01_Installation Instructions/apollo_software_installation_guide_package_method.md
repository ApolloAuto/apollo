# Installation - Package Method

This article introduces how to install Apollo using the package (deb package) method. The package method is faster and requires no compilation. However, there is only one environment supported, so if you want to install in a custom environment, please refer to source installation method.

This article assumes that the user has a basic working knowledge of Linux.

## Step 1: Install the base software

### 1. Install Ubuntu Linux

For steps to install Ubuntu 18.04+, see the [official installation guide][official_installation_guide]

> Note: While other distributions of Linux may be fine, we have only tested Apollo on a pure Ubuntu System, Ubuntu 18.0.5 LTS (Bionic Beaver), so we recommend that you use Ubuntu 18.04.5 as the host OS.

To update the software after installation:

```shell
sudo apt-get update
sudo apt-get upgrade
```

> Note: To complete the update, you need to ensure that you can access the network.

### 2. Install Docker Engine

Apollo relies on Docker 19.03+. To install the Docker engine, see [install Docker Engine on Ubuntu][install_docker_engine_on_ubuntu]

## Step 2: Install Apollo environment manager tool

The Apollo environemnt manager tool is a command line tool for developers to manage and launch apollo environment containers.

### 1. Add apt source

Add apt source and key:

```
sudo bash -c "echo 'deb https://apollo-pkg-beta.cdn.bcebos.com/neo/beta bionic main' >> /etc/apt/sources.list"
wget -O - https://apollo-pkg-beta.cdn.bcebos.com/neo/beta/key/deb.gpg.key | sudo apt-key add -
sudo apt update
```

### 2. Install apollo-env-manager

Run the following command to install apollo environment manager tool:

```shell
sudo apt install apollo-neo-env-manager-dev
```

If successful, you can use it directly

```shell
aem -h
```

For more detailed usage of `aem`, please refer to the [Apollo Environment Manager](../03_Package%20Management/apollo_env_manager.md) documentation.

## Step 3: Start and enter apollo env container

### 1. Create workspace

Create a directory as a workspace

```shell
mkdir application-demo
cd application-demo
```

### 2. Start apollo env container

```shell
aem start
```

> Note: The default environment image contains gpu-related dependencies. If you want to start the container as a gpu, you can use the `start_gpu` subcommand.

If everything works, you will see a prompt similar to the one below:

![container started][container_started.png]

### 3. Enter apollo env container

```shell
aem enter
```

After successful execution of the command, the following message will be displayed and you will be in the Apollo runtime container.

```shell
user_name@in-dev-docker:/apollo_workspace$
```

The workspace folder will be mounted to path `/apollo_workspace` of container.

### 4. Initialize workspace

```shell
aem init
```

Now, the apollo environment manager tool installed and apollo env container launched, please follow the [Launch and run Apollo](../03_Package%20Management/launch_and_run_apollo_package_method.md) documentation to run apollo or follow the [QuickStart](../02_Quick%20Start/apollo_8_0_quick_start.md) documentation to install different modules as needed for different usage scenarios

[official_installation_guide]: <https://ubuntu.com/tutorials/install-ubuntu-desktop>
[install_docker_engine_on_ubuntu]: <https://docs.docker.com/engine/install/ubuntu/>
[quickstart_project]: <https://apollo-pkg-beta.cdn.bcebos.com/e2e/apollo_v8.0.tar.gz>
[pass_request.png]: <https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_8_0/pass_request_b228e30.png>
[container_started.png]: <https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_8_0/container_started_4ee24d4.png>
[install_core.png]: <https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_8_0/install_core_0b0d533.png>
[feedback_collection_page]: <https://studio.apollo.auto/community/article/163>
