FROM iregistry.baidu-int.com/apollo-internal/nvidia-cuda:11.8.0-cudnn8-devel-ubuntu18.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt update && apt install -y python3 \
    python3-pip \
    python3-apt \
    wget \
    rsync \
    figlet \
    nethogs \
    sysstat \
    software-properties-common \
    tzdata

ENV TZ=UTC

RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

RUN pip3 install --default-timeout 30 --trusted-host pip.baidu-int.com -i http://pip.baidu-int.com/simple/ --upgrade pip setuptools

ENV LANG C.UTF-8

ENV LC_ALL C.UTF-8

RUN pip3 install --default-timeout 30 --trusted-host pip.baidu-int.com -i http://pip.baidu-int.com/simple/ ansible watchdog yattag bokeh flask

COPY aem/galaxy-requirements.yaml /galaxy-requirements.yaml

COPY aem/ansible /ansible

RUN mkdir -pv /opt/apollo/neo/bin /opt/apollo/neo/src && chmod -R 777 /opt/apollo/neo

RUN add-apt-repository ppa:ubuntu-toolchain-r/test && \
    apt update && \
    apt install -y g++-11 gcc-11 && \
    update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-11 11 --slave /usr/bin/g++ g++ /usr/bin/g++-11

ENV CUDNN_VERSION 8.9.6.50

ENV TENSORRT_VERSION 8.6.1.6

ENV PATH /opt/apollo/sysroot/bin:$PATH

ENV PATH /opt/apollo/neo/bin:$PATH

RUN ansible-galaxy collection install -f -r galaxy-requirements.yaml && \
    ansible-playbook ansible/playbook.yaml && \
    rm -rf /opt/apollo/sysroot/Qt* /usr/lib/x86_64-linux-gnu/libnvinfer_static.a && \
    apt clean && \
    rm -rf /var/cache/apollo/distfiles/* /var/cache/apt/archives/* 

RUN touch .installed && chmod 777 .installed

RUN for i in 71 72 73 74 75;do sed  -i "${i}s/^[[:space:]]*#define/#define/" "/usr/local/cuda/include/cusolver_common.h";done

RUN rm -rf galaxy-requirements.yaml \
           ansible
