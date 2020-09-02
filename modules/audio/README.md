# Audio

## Introduction
  The Audio module detect the siren sound coming from the active emergency vehicles. It trying to do on/off detection, moving status and the relative position of the siren.

  The Audio module depends on some libtorch model file, which published as a docker volume and mount into apollo when starting the apollo dev docker.


## Input
  * Audio signal data (cyber channel /apollo/sensor/microphone)

## Output
  * Audio detection result, including the siren type, moving status(approaching/departing/stationary), and the relative position.
