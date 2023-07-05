# Apollo Docker Image Build Process

## Introduction

As you may already know, Apollo runs inside Docker container. There are
basically two types of Apollo Docker images: CyberRT and Dev. CyberRT images
were for developers who want to play with the CyberRT framework only, while Dev
images were used to build and run the whole Apollo project.

Currently, two CPU architectures are supported: `x86_64` and `aarch64`.

**Note**

> `dev.aarch64` image was still WIP as of today. It is expected to be ready in
> the next few months.

In the section below, I will describe briefly the steps to build these Docker
images.

## Build CyberRT Image

Type `./build_docker.sh -h` for help message:

```
Usage:
    build_docker.sh -f <Dockerfile> [Options]
Available options:
    -c,--clean      Use "--no-cache=true" for docker build
    -m,--mode       "build" for build everything from source if possible, "download" for using prebuilt ones
    -g,--geo        Enable geo-specific mirrors to speed up build. Currently "cn" and "us" are supported.
    -d,--dist       Whether to build stable("stable") or experimental("testing") Docker images
    -t,--timestamp  Specify Cyber image timestamp to build Dev image from it. Format: yyyymmdd_hhmm (e.g 20210205_1520)
    --dry           Dry run (for testing purpose)
    -h,--help       Show this message and exit
E.g.,
    build_docker.sh -f cyber.x86_64.dockerfile -m build -g cn
    build_docker.sh -f dev.aarch64.dockerfile -m download -d testing
```

Here, the `-g/--geo` option is used to enable geo-location based settings (APT &
PYPI mirrors, etc.). Two codes (`us` & `cn`) are supported now, and the default
is `us`.

To build the latest CyberRT image, simply run:

```
./build_docker.sh -f cyber.<TARGET_ARCH>.dockerfile
```

Users of mainland China can specify `-g cn` to speed up build process:

```
./build_docker.sh -f cyber.<TARGET_ARCH>.dockerfile -g cn
```

## Build Apollo Dev Image

Run the following command to build Apollo Dev image:

```
build_docker.sh -f dev.<TARGET_ARCH>.dockerfile
```

On success, output messages like the following will be shown at the bottom of
your screen.

```
Successfully built baca71e567e6
Successfully tagged apolloauto/apollo:dev-x86_64-18.04-20200824_0339
Built new image apolloauto/apollo:dev-x86_64-18.04-20200824_0339
```

## Build Apollo Runtime Docker Image

Apollo Runtime Docker was used in combination with Release Build output for easy
deployment. You can run the following commands to build Apollo Runtime Docker
image on your own.

```
# Generate required APT packages
./apollo.sh release -c -r

cd docker/build
# Copy the generated package list for docker build
cp /apollo/output/syspkgs.txt .

cp runtime.x86_64.dockerfile.sample runtime.x86_64.dockerfile
bash build_docker.sh -f runtime.x86_64.dockerfile
```

> Note: Apollo Runtime Docker supports x86_64 only as Release Build was not
> ready for Aarch64 yet.

## Tips

### Build Log

The build log for CyberRT and Dev Docker images was located at
`/opt/apollo/build.log`, which contains download links and checksums of
dependent packages during Docker build.

### Enable Local HTTP Cache to Speed Up Build

You can enable local HTTP cache to speed up package downloading by performing
the following steps on your **host** running Docker:

1. Download all prerequisite packages to a directory (say, `$HOME/archive`) with
   URLs listed in the build log. Pay attention to their checksum.
2. Change to that archive directory and start your local HTTP cache server at
   port **8388**.

```
cd $HOME/archive
nohup python3 -m http.server 8388 &
```

> Note: Another advantage with the local HTTP cache mechanism is, it has little
> influence on the final image size. Even if the cached package was missing or
> broken, it can still be downloaded from the original URL.

3. Rerun `build_docker.sh`.

## Add New Installer

The best practice of a new installer would be:

1. Well tested.

   Of course. Make it work, and don't break other installers, such as
   incompatible versions of libraries.

1. Standalone.

   Have minimum assumption about the basement, which means, you can depend on
   the base image and `installers/installer_base.sh`. Other than that, you
   should install all the dependencies in your own installer.

1. Thin.

   It will generate a new layer in the final image, so please keep it as thin as
   possible. For example, clean up all intermediate files:

   ```bash
   wget xxx.zip
   # Unzip, make, make install
   rm -fr xxx.zip xxx
   ```

1. Cross-architecture.

   It would be awesome to work perfectly for different architectures such as
   `x86_64` and `aarch64`.
