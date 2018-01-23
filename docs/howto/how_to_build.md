Build
==========================

* [1. Install Docker](#docker)
* [2. Build](#build)
* [3. Test](#test)

## <span id="docker">Install Docker</span>
The system requirement for building Apollo is Ubuntu 14.04. Docker container is the simplest way to set up the build environment for Apollo project. Detailed docker tutorial is [here](https://docs.docker.com/).
```bash
sudo docker/scripts/install_docker.sh
```
## <span id="build">Build in Docker </span>
### Start container
We provide a build image named *dev-latest*. Container will mount your local apollo repo to */apollo*.
```bash
bash docker/scripts/dev_start.sh
```
### Get into container
```bash
bash docker/scripts/dev_into.sh
```
### Build modules
```bash
bash apollo.sh build
```

## Legal Disclaimer
The docker image that you build may contain ESD CAN library files provided by ESD Electronics (hereby referred as ESD), which you should have obtained via a licensing agreement with ESD. The licensing agreement shall have granted you (as an individual or a business entity) the right to use the said software provided by ESD; however, you may (and likely you do) need explicit re-distribution permission from ESD to publish the docker image for any other third party to consume. Such licensing agreement is solely between you and ESD, and is not covered by the license terms of the Apollo project (see file LICENSE under Apollo top directory).
