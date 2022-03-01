#! /usr/bin/env bash

APOLLO_REPO="apolloauto/apollo"
EDU_CHALLENGE="edu_challenge_sim"

docker build -t "${APOLLO_REPO}:${EDU_CHALLENGE}" -f dev.x86_64.edu_chllenge_sim.dockerfile .