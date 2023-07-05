# Run Simulation with Offline Record

This document provides a step-by-step guide on how to run simulation with a demo
offline record, in case you don't have the required hardware.

## Preparation Work

Suppose you have followed the
[Apollo Software Installation Guide](../../01_Installation%20Instructions/apollo_software_installation_guide.md).
You have cloned Apollo's GitHub repo, all the software pre-requisites were
installed correctly.

## Start and enter Apollo development Docker container

The following commands are assumed to run from `$APOLLO_ROOT_DIR`.

```
bash docker/scripts/dev_start.sh
bash docker/scripts/dev_into.sh
```

## Build Apollo

Run the following command to build Apollo inside Docker:

```
./apollo.sh build
```

Note:

> The script will auto-detect whether it was a CPU only build or a GPU build.

## Start Dreamview

To start the Monitor module and Dreamview
backend, run:

```
bash scripts/bootstrap.sh
```

## Download and play the demo record

```
python3 docs/demo_guide/record_helper.py demo_3.5.record
cyber_recorder play -f demo_3.5.record --loop
```

Note:

> The `--loop` option enables record to keep playing in a loop playback mode.

## Open <http://127.0.0.1:8888> in your favorite browser (e.g. Chrome) to access Apollo Dreamview

The following screen should be shown to you and the car in Dreamview now moves around!

![](images/dv_trajectory.png)

Congratulations!
