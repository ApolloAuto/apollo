# safelist
branches:
  only:
  - master

language: cpp
sudo: required
services:
    - docker
dist: trusty
os: linux
cache:
  directories:
  - $HOME/.cache/bazel
install: skip
before_script:
    - cp scripts/AGREEMENT.txt ${HOME}/.apollo_agreement.txt
jobs:
  include:
    - stage: stage-01
      env:
        - JOB=lint
      script:
        - ./docker/scripts/dev_start.sh -t dev-x86_64-20181210_1500
        - ./apollo_docker.sh ${JOB}
        - rm -rf "${HOME}/.cache/bazel/_bazel_${USER}/install"
        - rm -rf "/apollo/data/core"
    - stage: stage-02
      if: branch = master
      env:
        - JOB=cibuild
      script:
        - ./docker/scripts/dev_start.sh -t dev-x86_64-20181210_1500
        - ./apollo_docker.sh ${JOB}
        - rm -rf "${HOME}/.cache/bazel/_bazel_${USER}/install"
        - rm -rf "/apollo/data/core"
    - # stage: stage-02
      env:
        - JOB=cibuild_extended
      script:
        - ./docker/scripts/dev_start.sh -t dev-x86_64-20181210_1500
        - ./apollo_docker.sh ${JOB}
        - rm -rf "${HOME}/.cache/bazel/_bazel_${USER}/install"
        - rm -rf "/apollo/data/core"
    - # stage: stage-02
      env:
        - JOB=citest_basic
      script:
        - ./docker/scripts/dev_start.sh -t dev-x86_64-20181210_1500
        - ./apollo_docker.sh ${JOB}
        - rm -rf "${HOME}/.cache/bazel/_bazel_${USER}/install"
        - rm -rf "/apollo/data/core"
    - # stage: stage-02
      env:
        - JOB=citest_extended
      script:
        - ./docker/scripts/dev_start.sh -t dev-x86_64-20181210_1500
        - ./apollo_docker.sh ${JOB}
        - rm -rf "${HOME}/.cache/bazel/_bazel_${USER}/install"
        - rm -rf "/apollo/data/core"
notifications:
    email:
        on_success: always
        on_failure: always
