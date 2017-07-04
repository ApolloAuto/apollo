## Introduction

This is a driver for GNSS devices, such as NovAtel, Applanix, and u-blox. A GNSS device usually
includes a GNSS receiver, an IMU, an interface to a wheel encoder, and a fusion engine that combines
information from those sensors.

## Purpose

We aim to the following in this driver.
- Clear C++ code, better code structure, and better performance (than the open-source NovAtel driver).
- Publish sensor-independent protobuf messages such as sensor/gps, sensor/imu, sensor/ins, sensor/wheel.
- Support various sensors: NovAtel, u-blox, STIM300, wheel encoder, etc.
- Log and replay raw data. The log will be used in IE post-processing.
- Support streaming RTK correction to the GPS receiver.

## Design

The driver has two nodelets: stream nodelet and parser nodelet. The stream nodelet is in charge of
communication between host PC and the device, as well as grabbing RTK data from a NTRIP caster. The
parser nodelet subscribes the raw data from the stream nodelet, parses the data, and publishes
protobuf messages.

## Input

- data generated from gnss devices, such as NovAtel, support tcp/usb/udp/ntrip connect method.

## Output

- gnss status
- ins status
- stream status
- imu data
- localization data

## Configuration
We use a protobuf to store all the configuration the driver needs. Configure file is stored in path
 `share/gnss_driver/conf/` which is quoted by gnss_driver.launch. File gnss_driver.launch is stored in path
 `share/gnss_driver/launch/`.
When use gnss_driver, the following should be attended.
- Now the location use UTM projection, must check zone id configure in gnss_driver.launch.
- Lever arm distance check.
- Confirm imu install method, this affect vehicle frame and orientation compute.
