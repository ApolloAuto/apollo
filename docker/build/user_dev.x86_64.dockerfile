FROM apolloauto/apollo:dev-18.04-x86_64-20200505_0330
ENV DEBIAN_FRONTEND=noninteractive

WORKDIR /apollo
USER root

RUN deluser --remove-home apollo

