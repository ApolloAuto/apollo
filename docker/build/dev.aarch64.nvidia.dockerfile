ARG BASE_IMAGE
FROM ${BASE_IMAGE}

ARG GEOLOC
ARG CLEAN_DEPS
ARG INSTALL_MODE

WORKDIR /apollo

COPY archive /tmp/archive

COPY installers /opt/apollo/installers

RUN bash /opt/apollo/installers/install_geo_adjustment.sh ${GEOLOC}
RUN bash /opt/apollo/installers/install_modules_base.sh
RUN bash /opt/apollo/installers/install_gpu_support.sh
RUN bash /opt/apollo/installers/install_ordinary_modules.sh
RUN bash /opt/apollo/installers/install_drivers_deps.sh ${INSTALL_MODE}
RUN bash /opt/apollo/installers/install_dreamview_deps.sh ${GEOLOC}
RUN bash /opt/apollo/installers/install_contrib_deps.sh

RUN bash /opt/apollo/installers/install_release_deps.sh
RUN bash /opt/apollo/installers/post_install.sh ${BUILD_STAGE}

RUN wget "https://apollo-system.cdn.bcebos.com/patch/libc-bin_2.31-0ubuntu9.9.ubuntu.focal.custom_arm64.deb" \
    && wget "https://apollo-system.cdn.bcebos.com/patch/libc-dev-bin_2.31-0ubuntu9.9.ubuntu.focal.custom_arm64.deb" \
    && wget "https://apollo-system.cdn.bcebos.com/patch/libc6-dev_2.31-0ubuntu9.9.ubuntu.focal.custom_arm64.deb" \
    && wget "https://apollo-system.cdn.bcebos.com/patch/libc6_2.31-0ubuntu9.9.ubuntu.focal.custom_arm64.deb" \
    && dpkg -i libc-bin_2.31-0ubuntu9.9.ubuntu.focal.custom_arm64.deb libc-dev-bin_2.31-0ubuntu9.9.ubuntu.focal.custom_arm64.deb libc6-dev_2.31-0ubuntu9.9.ubuntu.focal.custom_arm64.deb libc6_2.31-0ubuntu9.9.ubuntu.focal.custom_arm64.deb \
    && rm -f libc-bin_2.31-0ubuntu9.9.ubuntu.focal.custom_arm64.deb libc-dev-bin_2.31-0ubuntu9.9.ubuntu.focal.custom_arm64.deb libc6-dev_2.31-0ubuntu9.9.ubuntu.focal.custom_arm64.deb libc6_2.31-0ubuntu9.9.ubuntu.focal.custom_arm64.deb 

RUN RUN wget https://apollo-system.cdn.bcebos.com/archive/9.0/dep_install_aarch64.tar.gz && \
	tar -xzvf dep_install_aarch64.tar.gz && mv dep_install_aarch64/lib/lib* /usr/local/lib/ && \
	mv dep_install_aarch64/include/* /usr/local/include/ && rm -rf dep_install_aarch64*

RUN bash /opt/apollo/installers/install_pkg_repo.sh

COPY rcfiles/setup.sh /opt/apollo/neo/

RUN bash /opt/apollo/installers/install_rsdriver.sh
RUN bash /opt/apollo/installers/install_livox_driver.sh
RUN bash /opt/apollo/installers/install_hesai2_driver.sh
RUN bash /opt/apollo/installers/install_vanjee_driver.sh

RUN pip3 install tensorflow==2.10.0
