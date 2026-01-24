# perception-barrier-recognition

## Introduction

This module is used to recognize the status of barrier gate nearby. The status includes opened, closed, opening and closing. 

## Directory Structure

```
├── barrier_recognition // barrier recognition module
    ├── conf            // module configuration files
    ├── dag             // dag files
    ├── interface       // function interface folder
    ├── launch          // launch files
    ├── detector        // main part for recognition
    ├── proto           // proto files
    ├── tracke          // part for tracker
    ├── barrier_recognition_component.cc // component interface
    ├── barrier_recognition_component.h
    ├── cyberfile.xml   // package management profile
    ├── README.md
    └── BUILD
```

#### How to Launch

```bash
cyber_launch start modules/perception/barrier_recognition/launch/barrier_recognition.launch
```
