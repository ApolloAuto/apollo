FROM apolloauto/apollo:dev-x86_64-18.04-testing-20201104_0348
ARG APOLLO_DIST
COPY installers/installer_base.sh /tmp/installers/
COPY installers/install_libtorch.sh /tmp/installers/
RUN bash /tmp/installers/install_libtorch.sh && rm -rf /tmp/installers
