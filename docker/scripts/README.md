# Docker environment

## Install docker

Please follow the
[official guide to install the docker-ce](https://docs.docker.com/install/linux/docker-ce/ubuntu).
We also provide a bash script to install docker-ce

```bash
bash docker/setup_host/install_docker.sh
```

Don't forget the
[post-installation steps for Linux](https://docs.docker.com/install/linux/linux-postinstall).


## Install nvidia-docker (optional)

To use the host machine's GPUs, you need to install nvidia docker.

```bash
bash docker/setup_host/install_nvidia_docker.sh
```
### Mac support

We always recommend to run Apollo container on Ubuntu with the same version of
the base image, but in case you want to do code development on a Mac, you can
still try the latest
[Docker-for-Mac](https://docs.docker.com/docker-for-mac/install)

It runs on a virtual kernel which is similar to using the Docker Toolkit + VirtualBox.
Make sure you understand the [difference](https://docs.docker.com/docker-for-mac/docker-toolbox).

With Mac, a lot of the Linux bindings are not available, which could cause issues.
So it's basically just an environment for code development, but not for production.

## Development container

We provide a development environment where you can build Apollo from code.

```bash
cd /path/to/apollo
bash docker/scripts/dev_start.sh
bash docker/scripts/dev_into.sh
```

The scripts to build the dev-image are also available at docker/build.

```
Note:
Within the scripts in this directory, only standard tools that are expected to be available in most Linux distributions should be used (e.g., don't use realpath).
```

And then you should be able to see Dreamview at http://localhost:8888.
