# Tailored from https://gitlab.com/nvidia/container-images/cuda:
# dist/10.2/ubuntu18.04-x86_64/devel/cudnn8/Dockerfile
FROM nvidia/cuda:10.2-devel-ubuntu18.04
ENV CUDNN_VERSION 8.0.2.39
LABEL com.nvidia.cudnn.version="${CUDNN_VERSION}"
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        libcudnn8=$CUDNN_VERSION-1+cuda10.2 \
        libcudnn8-dev=$CUDNN_VERSION-1+cuda10.2 \
    && apt-mark hold libcudnn8 \
    && apt-get -y clean \
    && rm -rf /var/lib/apt/lists/* \
    && echo "Stripping off libcudnn8 static libraries..." \
    && find /usr/lib/$(uname -m)-linux-gnu -name "libcudnn_*.a" -delete -print
