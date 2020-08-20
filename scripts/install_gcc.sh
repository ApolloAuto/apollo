#! /usr/bin/env bash

# This script installs multiple versions of gcc/g++ on ubuntu 18.04.

if [ ! -x "$(which add-apt-repository)" ]; then
  sudo apt-get -y update
  sudo apt-get -y install software-properties-common
fi

ubuntu_release="$(lsb_release -rs)"
if [ "${ubuntu_release}" == "16.04" ] || [ "${ubuntu_release}" == "18.04" ]; then
  sudo add-apt-repository ppa:ubuntu-toolchain-r/test
fi

sudo apt-get -y update \
  && sudo apt-get -y install \
    gcc-7 \
    g++-7 \
    gcc-8 \
    g++-8 \
    gcc-9 \
    g++-9 \
  && sudo apt-get clean

sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-9 90
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-9 90

sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-8 80
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-8 80

sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-7 70
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-7 70

# sudo update-alternatives --config gcc
# sudo update-alternatives --config g++
