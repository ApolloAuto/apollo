# Docker environment

## Install docker

Please follow the
[official guide to install the docker-ce](https://docs.docker.com/install/linux/docker-ce/ubuntu).

Don't forget the
[post-installation steps for Linux](https://docs.docker.com/install/linux/linux-postinstall).

### Mac support

We always recommend to run Apollo container on Ubuntu with the same version of
the base image, but in case you want to do code development on a Mac, you can
still try the latest
[Docker-for-Mac](https://docs.docker.com/docker-for-mac/install)

It runs on a virtual kernel which is similar to use Docker Toolkit + VirtualBox.
Make sure you understand the [difference](https://docs.docker.com/docker-for-mac/docker-toolbox).

With Mac, lots of Linux bindings are not available, which could cause problems.
So it's basicly just an environment for code development, while not production.

## Development container

We provide development environment where you can build Apollo from code.

```bash
cd /path/to/apollo
bash docker/scripts/dev_start.sh
bash docker/scripts/dev_into.sh
```

The scripts to build the dev-image are also available at docker/build.

Note that, within the scripts in this directory, only standard tools that are
expected to be available in most Linux distributions should be used (e.g., don't
use realpath).

## Release container

We also distribute release images with runtime environment and pre-compiled
binaries.

Simply run
```bash
cd /path/to/apollo
bash docker/scripts/release_start.sh
```

And then you should be able to see Dreamview at http://localhost:8888.
