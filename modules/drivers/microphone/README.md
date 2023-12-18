# drivers-microphone

## Introduction
This package is responsible for receiving the native audio input and parsing it to send it to the audio package for subsequent processing.

## Directory Structure
```shell
modules/drivers/microphone/
├── BUILD
├── conf
├── cyberfile.xml
├── dag
├── drivers-microphone.BUILD
├── launch
├── microphone_component.cc
├── microphone_component.h
├── proto
├── README.md
├── respeaker.cc
└── respeaker.h
```

## Modules

### MicrophoneComponent

apollo::drivers::microphone::MicrophoneComponent


#### Input

| Name   | Type| Description         |
| ------ | --- | ------------------- |
| stream |  -  |          -          |

#### Output

| Name  | Type                                             | Description           |
| ----- | -------------------------------------------------| --------------------- |
| `msg` | `apollo::drivers::microphone::config::AudioDate` |  Raw microphone data  |

#### configs

| file path                                      | type / struct                                           | Description           |
| ---------------------------------------------- | ------------------------------------------------------- | --------------------- |
| `modules/drivers/microphone/respeaker.pb.txt`  | `apollo::drivers::microphone::config::MicrophoneConfig` |    microphone config  |

## Microphone Configuration

* **microphone_model**: currently only support `RESPEAKER`.
* **chunk**: number of frames per buffer from hardware to memory each time.
* **record_seconds**: number of seconds each time.
* **channel_type**
  * May be ASR (Audio Speech Recognition), RAW (Raw Audio Data) or Playback.
* **sample_rate**
  * Number of frames that are recorded per second.
* **sample_width**
  * Number of bytes per sample (1, 2, 3, or 4).

For example, if there are 6 channels with 16000 Hz rate and 2 bytes width, then 4 second recording is
* Number of frames: 16000
* Number of "samples": 16000 * 6
  * I use the term "samples" here just to make it easy for understanding, which is seldom used in this context.
* Total size: 6 × 16,000 x 4 × 2 = 768,000 bytes.

You might see other metrics elsewhere as follows:

* **BIT DEPTH** same to sample_width except that the unit is bit.
* **BIT RATE** number of bits encoded per second (kbps or k) -- for compressed format like mp3.

#### How to Launch

```bash
cyber_launch start modules/drivers/microphone/launch/microphone.launch
```
