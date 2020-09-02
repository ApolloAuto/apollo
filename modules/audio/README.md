# Audio

## Introduction
  The Audio module detect the siren sound coming from the active emergency vehicle. It's trying to do on/off detection, moving status and the relative position of the siren.

  The Audio module depends on some libtorch model file, which is published as a docker volume and mount into Apollo when starting the Apollo dev docker.


## Input
  * Audio signal data (cyber channel /apollo/sensor/microphone)

## Output
  * Audio detection result, including the siren type, moving status(approaching/departing/stationary), and the relative position.
