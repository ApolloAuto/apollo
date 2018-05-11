# Docker Scripts Canary

## Usage

1. Setup host.

   ```bash
   sudo bash ./docker/canary/setup_host.sh
   ```

1. If you are going to use GPU (recommended), you need to install
   [nvidia-driver](http://www.nvidia.com/Download/index.aspx) and
   [nvidia-docker](https://github.com/NVIDIA/nvidia-docker/blob/master/README.md#quickstart)
   on HOST.

   Nvidia-driver provides access to GPU from HOST, which will make the following
   command work properly:

   ```bash
   nvidia-smi
   ```

   Nvidia-docker provides access to GPU from docker container, which will make
   the following command work properly:

   ```bash
   docker run --runtime=nvidia --rm nvidia/cuda nvidia-smi
   ```

1. Start Apollo dev container.

   ```bash
   # Show help message.
   ./docker/canary/start_container.py --help

   # Start with default options.
   ./docker/canary/start_container.py
   ```


## New/Coming Features

1. Rewrite with Python.

   The Bash script becomes messy, especially the environment variables. We need
   cleaner and more extensible infrastructure to manage the logic, considering
   optional features, different arch (x86, ARM, etc), different OS (Linux, OS,
   etc), versions, etc.

1. Unified user.

   Only mount USER_ID and GROUP_ID into container, while consolidate the inside
   USER:GROUP as 'apollo:apollo' to guarantee better certainty.

1. Contained bazel cache.

   Keep bazel cache inside container, instead of mount it from HOST.

   Pros: Keep the container more self-contained.

   Cons: Must rebuild everything after restarting the container. But considering
   that we don't do this often, it should be fine.

1. Unify dev and release image.

   There is no different release image anymore. A release image is a dev image
   plus a pre-built bazel cache layer.
