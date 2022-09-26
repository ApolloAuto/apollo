ARG BASE_IMAGE
FROM ${BASE_IMAGE}

ARG CUDA_LITE
ARG CUDNN_VERSION
ARG TENSORRT_VERSION

# nvidia gpg key error. ref: https://developer.nvidia.com/blog/updating-the-cuda-linux-gpg-repository-key/
RUN apt-key adv --fetch-keys http://developer.download.nvidia.com/compute/cuda/repos/ubuntu1604/x86_64/3bf863cc.pub

RUN M="${CUDNN_VERSION%%.*}" \
    && PATCH="-1+cuda${CUDA_LITE}" \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
    libcudnn${M}="${CUDNN_VERSION}${PATCH}" \
    libcudnn${M}-dev="${CUDNN_VERSION}${PATCH}" \
    && apt-mark hold libcudnn${M} \
    && rm -rf /var/lib/apt/lists/* \
    && echo "Delete static cuDNN libraries..." \
    && find /usr/lib/$(uname -m)-linux-gnu -name "libcudnn_*.a" -delete -print

ENV CUDNN_VERSION ${CUDNN_VERSION}

RUN PATCH="-1+cuda${CUDA_LITE}" && apt-get -y update \
    && apt-get install -y --no-install-recommends \
    libnvinfer7="${TENSORRT_VERSION}${PATCH}" \
    libnvonnxparsers7="${TENSORRT_VERSION}${PATCH}" \
    libnvparsers7="${TENSORRT_VERSION}${PATCH}" \
    libnvinfer-plugin7="${TENSORRT_VERSION}${PATCH}" \
    libnvinfer-dev="${TENSORRT_VERSION}${PATCH}" \
    libnvonnxparsers-dev="${TENSORRT_VERSION}${PATCH}" \
    libnvparsers-dev="${TENSORRT_VERSION}${PATCH}" \
    libnvinfer-plugin-dev="${TENSORRT_VERSION}${PATCH}" \
    && apt-get -y clean \
    && rm -rf /var/lib/apt/lists/* \
    && rm -f /etc/apt/sources.list.d/nvidia-ml.list \
    && rm -f /etc/apt/sources.list.d/cuda.list

ENV TENSORRT_VERSION ${TENSORRT_VERSION}

# Notes:
# 1) Removed Nvidia apt sources.list to speed up build
