ARG BASE_IMAGE
FROM ${BASE_IMAGE}

ARG CUDA_LITE
ARG CUDNN_VERSION
ARG TENSORRT_VERSION

RUN apt-get update && apt-get install -y --no-install-recommends gnupg2 curl ca-certificates \
    && echo "deb [trusted=yes] http://172.17.0.1:8080/arm64/ ./" > /etc/apt/sources.list.d/cuda.list \
    && apt-get purge --autoremove -y curl \
    && rm -rf /var/lib/apt/lists/*

ENV CUDA_VERSION 10.2.89

# For libraries in the cuda-compat-* package: https://docs.nvidia.com/cuda/eula/index.html#attachment-a
RUN M="10-2" \
    && apt-get update && apt-get install -y --no-install-recommends \
        cuda-cudart-${M} \
        cuda-libraries-${M} \
        cuda-npp-${M} \
        cuda-nvtx-${M} \
        libcublas10 \
    && ln -s cuda-${CUDA_LITE} /usr/local/cuda && \
    rm -rf /var/lib/apt/lists/*

ENV PATH /usr/local/cuda/bin:${PATH}

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES compute,utility
ENV NVIDIA_REQUIRE_CUDA "cuda>=${CUDA_LITE}"

RUN M="10-2" && apt-get update && apt-get install -y --no-install-recommends \
    cuda-nvml-dev-${M} \
    cuda-command-line-tools-${M} \
    cuda-nvprof-${M} \
    cuda-npp-dev-${M} \
    cuda-libraries-dev-${M} \
    cuda-minimal-build-${M} \
    libcublas-dev \
    cuda-cusparse-${M} \
    cuda-cusparse-dev-${M} \
    && rm -rf /var/lib/apt/lists/*

ENV LIBRARY_PATH /usr/local/cuda/lib64/stubs

RUN M="${CUDNN_VERSION%%.*}" \
    && PATCH="-1+cuda${CUDA_LITE}" \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        libcudnn${M}="${CUDNN_VERSION}${PATCH}" \
        libcudnn${M}-dev="${CUDNN_VERSION}${PATCH}" \
    && apt-mark hold libcudnn${M} \
    && rm -rf /var/lib/apt/lists/*

RUN echo "Delete static cuDNN libraries..." \
    && find /usr/lib/$(uname -m)-linux-gnu -name "libcudnn_*.a" -delete -print

ENV CUDNN_VERSION ${CUDNN_VERSION}

ENV TENSORRT_PKG_VERSION ${TENSORRT_VERSION}-1+cuda${CUDA_LITE}

RUN PATCH="-1+cuda${CUDA_LITE}" && apt-get -y update \
    && apt-get install -y --no-install-recommends \
    libnvinfer7="${TENSORRT_PKG_VERSION}" \
    libnvonnxparsers7="${TENSORRT_PKG_VERSION}" \
    libnvparsers7="${TENSORRT_PKG_VERSION}" \
    libnvinfer-plugin7="${TENSORRT_PKG_VERSION}" \
    libnvinfer-dev="${TENSORRT_PKG_VERSION}" \
    libnvonnxparsers-dev="${TENSORRT_PKG_VERSION}" \
    libnvparsers-dev="${TENSORRT_PKG_VERSION}" \
    libnvinfer-plugin-dev="${TENSORRT_PKG_VERSION}" \
    && apt-get -y clean \
    && rm -rf /var/lib/apt/lists/* \
    && rm -f /etc/apt/sources.list.d/cuda.list

ENV TENSORRT_VERSION ${TENSORRT_VERSION}
