## How to Prepare CENTOS(7) based GPU enabled image for the Perception Module

## Setup docker image in CentOS based system

ApolloAuto uses Ubuntu partially because of the Linux distribution dependency of ROS indigo. There is a script
`${apollo\_root}/docker/scripts/install\_dcoker.sh` inside the docker image involved. Most cloud based servers have already installed docker, hence you can just go ahead without touching it.

In CentOS, to install docker
you can either follow [this installation guide](https://docs.docker.com/engine/installation/linux/docker-ce/centos/#os-requirements)
 to install Docker- CE for CentOS or you could write the following:

> sudo yum update && sudo yum install -y docker

Start docker by

> sudo systemctl start docker

Check its health status by

> sudo systemctl status docker

As a matter of the fact, we recommend you to install `nvidia-docker`. `Nvidia-docker2` is currently
under `alpha` verision, and should not be used in production environment.

As typically sugggested by Docker, you should add your name to the docker group

> sudo usermod -aG docker $(whoami)

You can then build out Apollo based on the steps in the installation guide.

Before executing `dev_start.sh` to build or pull an image from  apolloauto for CentOS, you have to
modify ${apollo\_auto}/scripts/docker\_adduser.sh first by adding `--force-badname` in line 21.

> adduser --force-badname --disable-password ...

## GPU enabled container for perception research

#### Update driver in host

This part is much more tricky because you have to install an nvidia driver in docker container as the same
version of that in your host machine and stop "nvidia.uvm" relevant modules.

If you are using `VirtualBox`, whether the virtual graphic adapter \(VGA\) can pass through hardware depends
on your nvidia driver and gpu versions. Some developlers suggest KVM, VMWare and Xen where hardware passthrough to VM
is well developed.

Check hardware modules from nvidia driver, using

	> lsmod | grep nvidia

To successfully build nvidia driver `375.39` suggested by Baidu, you have to download the compatible linux source version, for example
`3.10.0-693.5.2.el7.x86_64` and pass it to the "NVIDIA.\*run". Whether it
will or not succeed really depends on your Linux Kernel version. Please vim "NVIDIA.\*run" to check correct keyword to pass.

#### Update driver in docker

Although you can succeed in the above solution, it is not recommended as it is time intensive and not very efficient.

A better way is to apply for operation time and update Baidu's driver inside docker. The following problems may arise:

1. gcc version confliction with NVIDIA driver:
   - baidu uses gcc-4.8.4, it is old, check `install_nvidia_driver.sh` to see how to update
2. bazel dependency: after installation of the driver, you would have to roll back the default gcc version to build apollo
3. resintall cuda: it will cost you roughly 2GB, please scale the container to proper size by adjusting paremters in `docker run` statement
   - inspect mounted disk inside docker, `df -h`, replace the directory which you think good in `reinstall_cuda.sh`
     keyword `--tmpdir`

After installation you, can check the situation by issuing

```bash
cat /proc/driver/nvidia/version
nvidia-smi
```

`nvidia-smi` should work normally now.

Finally,

	> ./apollo.sh build_gpu && ./scripts/percetion.sh start

If there is no error records in `data/log/`, you are good to go.

## Prebuild image for download

You can refer to my [DockerHub](https://hub.docker.com/r/yiakwy/apolloautocentos\_gpu) for further instructions.

## Troubleshooting

What if you committed incorrect `\*.pd.txt` files and cannot rebase to the latest developer branch? Try the
following commands:

```
git reset --soft HEAD^ # back to the staging area from previous commit
for f in $UNWANTED_FILES
	git reset HEAD $f
	git git update-index --assume-unchanged $f
git commit -m ${MESSAGE}
```

replace it with the arguments you actually need.

## Contact me

https://yiakwy.github.io
