# How to Run Perception Module on Your Local Computer

The perception module requires Nvidia GPU and CUDA installed to run the perception algorithms with Caffe. However, the Nvidia GPU driver is not installed in the dev docker image. To run the perception module with CUDA acceleration, we suggest to install the exactly same version of Nvidia driver in the docker as the one installed in your host machine, and build Apollo with GPU option.

We provide a step-by-step instruction on running perception module with Nvidia GPU as below:

1. Modify the script `./docker/scripts/dev_start.sh` to mount the library and source code of linux kernel from the host machine, by adding two option lines in command `docker run -it`:
```
-v /usr/src:/usr/src 
-v /lib/modules:/lib/modules
```

2. Start the docker image and get into docker with root authority: 
``` 
bash docker/scripts/dev_start.sh
docker exec -it apollo_dev /bin/bash
```

3. Download the official Nvidia driver installation file which should be the exactly same version as the one installed in your host machine. We recommend the version of 375.39:
```
wget http://us.download.nvidia.com/XFree86/Linux-x86_64/375.39/NVIDIA-Linux-x86_64-375.39.run
```

4. Install Nvidia driver in docker (using Linux-x86-375.39 version as example):
```
chmod +x ./NVIDIA-Linux-x86_64-375.39.run
./NVIDIA-Linux-x86_64-375.39.run --no-opengl-files -a -s
```

5. Commit a new docker image (in host):
```
docker commit CONTAINER_ID apolloauto/apollo:NEW_DOCKER_IMAGE_TAG
```

6. Start the new docker image (in host) and get into docker:
```
./docker/scripts/dev_start.sh NEW_DOCKER_IMAGE_TAG
./docker/scripts/dev_into.sh
```

7. Build Apollo with GPU option (in docker):
```
./apollo.sh build_gpu
```
or 
```
./apollo.sh build_opt_gpu
```

Now the perception module can be running in GPU mode with command `./scripts/perception.sh start`. Please note that the Nvidia driver should be installed appropriately as shown above even if the perception module is running in Caffe CPU_ONLY mode (i.e., using `./apollo.sh build` or `./apollo.sh build_opt` to build the perception module). Please see the detailed instruction of perception module in [the perception README](https://github.com/ApolloAuto/apollo/blob/master/modules/perception/README.md).
