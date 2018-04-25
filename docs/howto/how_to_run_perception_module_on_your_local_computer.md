# How to Run Perception Module on Your Local Computer

The perception module requires Nvidia GPU and CUDA installed to run the perception algorithms with Caffe. We have already installed the CUDA and Caffe libraries in the released docker. However, the Nvidia GPU driver is not installed in the released dev docker image. To run the perception module with CUDA acceleration, we suggest to install the exactly same version of Nvidia driver in the docker as the one installed in your host machine, and build Apollo with GPU option.

We provide a step-by-step instruction on running perception module with Nvidia GPU as below:

1. Modify the script `./docker/scripts/dev_start.sh` to mount the library and source code of linux kernel from the host machine, by adding two option lines in command `docker run -it`:
```
-v /usr/src:/usr/src
-v /lib/modules:/lib/modules
```

2. Start the released docker image and get into docker with root authority:
```
./docker/scripts/dev_start.sh
docker exec -it apollo_dev /bin/bash
```

3. Download the official Nvidia driver installation file which should be the exactly same version as the one installed in your host machine. We recommend the version of 375.39:
```
wget http://us.download.nvidia.com/XFree86/Linux-x86_64/375.39/NVIDIA-Linux-x86_64-375.39.run
```

4. Install Nvidia driver in docker (using Linux-x86-375.39 version as example):
```
source /apollo/scripts/install_gcc.sh
ln -s /usr/bin/cc /usr/bin/cc1
chmod +x ./NVIDIA-Linux-x86_64-375.39.run
./NVIDIA-Linux-x86_64-375.39.run --no-opengl-files -a -s
source /apollo/scripts/recover_gcc.sh
rm /usr/bin/cc1
```

5. Install cuDNN
Download cudnn from the following link. You may need to create an Nvidia developer account to proceed.
[cuDNN v7.1.1 Developer Library for Ubuntu14.04 (Deb)](https://developer.nvidia.com/compute/machine-learning/cudnn/secure/v7.1.1/prod/8.0_20180214/Ubuntu14_04-x64/libcudnn7-dev_7.1.1.5-1+cuda8.0_amd64)

After download, install cuDNN by
```
sudo dpkg -i libcudnn7*.deb
```

6. Commit a new docker image (in host):
```
docker commit CONTAINER_ID apolloauto/apollo:NEW_DOCKER_IMAGE_TAG
```

7. Start the new docker image (in host) and get into docker:
```
./docker/scripts/dev_start.sh -l -t NEW_DOCKER_IMAGE_TAG
./docker/scripts/dev_into.sh
```

8. Build Apollo with GPU option (in docker):
```
./apollo.sh build_gpu
```
or
```
./apollo.sh build_opt_gpu
```

Now the perception module can be running in GPU mode with command `./scripts/perception.sh start`. (Note for Apollo 2.5, the command is `./scripts/perception_lowcost.sh start`) Please note that the Nvidia driver should be installed appropriately as shown above even if the perception module is running in Caffe CPU_ONLY mode (i.e., using `./apollo.sh build` or `./apollo.sh build_opt` to build the perception module). Please see the detailed instruction of perception module in [the perception README](https://github.com/ApolloAuto/apollo/blob/master/modules/perception/README.md).
