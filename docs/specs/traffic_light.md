# Traffic Light Perception
This document describes the details of traffic light perception in Baidu Apollo program.
 
## 1. Introduction
The traffic light perception module is designed to provide accurate and robust traffic light state using cameras. Typically, the traffic light has three states, including red, yellow and green. Sometimes traffic light may keep black if not work or flashing. Occasionally, traffic light cannot be found in camera field of vision and our module failed to recognize its state. Hence, the traffic light perception module outputs five states in total, which are red, yellow, green, black and unknown.
 
Our module query hd-map repeatedly to know whether there are lights in front. The traffic light is represented by the four points on its boundary, which can be obtained by querying hd-map given the car's localization. The light in front will be projected from world coordinates to image coordinates.
 
Considering that the height of lights or the breadth of crossing varies widely, single camera with a constant field of vision, can't see traffic light in perception range which should be above 100 meters. We adopt multi cameras to enlarge perception range. In apollo 2.0, a telephoto camera whose focal length is 25 mm, is installed to observe forward far traffic light. Traffic lights captured in the telephoto camera are very large and easy to be detected. However, the field of vision of telephoto camera is quite limited so that the lights are often out of the image if the lane is not straight enough or the car is near the light. Thus, a wide angle camera whose focal length is 6 mm, is equipped to provide supplementary field of vision. The module will decide which camera to be used adaptively based on the coordinate projection. Although there are only two cameras on Apollo car, our algorithm is able to handle multi-cameras.
 
 ![telephoto camera](images/traffic_light/long.jpg) 

 ![wide angle camera](images/traffic_light/short.jpg)

The pipeline consists of two main parts which are pre-process and process.
## 2. Pre-process
 There is no need to detect lights in every frame of image since the state of traffic light changes in low frequency and the computing resource are limited. Normally, images from different cameras would arrive nearly simultaneously and only one of them is fed to process part. Therefore, the camera selection is necessary.The traffic light is represented by the four points on its boundary, which is stored in hd-map. A typical signal info is like below:
``` protobuf
signal info:
id {
  id: "xxx"
}
boundary {
  point { x: ...  y: ...  z: ...  }
  point { x: ...  y: ...  z: ...  }
  point { x: ...  y: ...  z: ...  }
  point { x: ...  y: ...  z: ...  }
}
```

### 2.1 Input/Output
Input：
- images from different cameras, acquired by subscribing:
    - **/apollo/sensor/camera/traffic/image_long**
    - **/apollo/sensor/camera/traffic/image_short**
- localization, acquired by subscribing 
    - **/tf**
- hd-map
- calibration results
 
Output：
  - image from the selected camera.
  - traffic light bounding box projected from world coordinates to image coordinates

### 2.2 camera selection
 The boundary points are projected to the image coordinates of each camera while driving. In the same position, the longger focal length, the larger projected area and the better for detecting. Hence the camera with the longest focal length that could see all the lights is selected. If lights fail to projected to all cameras, we simply selected the telephoto camera just to go through all pipelines. The selected camera id with timestamp is cached in queue, as described below:
 ``` C++
struct ImageLights {
  CarPose pose;
  CameraId camera_id;
  double timestamp;
  size_t num_signal;
  ... other ...
};
 ```
##### Attention
So far, all the information we need is localization, calibration results and hd-map. The selection can be perform at any time since projection is independent of image content. We perform the selection when images arrive is just for simplicity. Besides, the selection don't need to be done at every image's arrival and a interval for selection is set.

### 2.3 image sync
Images arrive with its timestamp and camera id. The pair of timestamp and camera id is used to find appropriate cached information. If the image can find a cached record with same camera id and small difference between timestamps, the image can be published to process. All inappropriate images are abandoned.
 
## 3. Process
We devide the process task into the following three steps.
### 3.1 Input/Output
Input：
- image from selected camera
- a set of bounding boxes
 
Output：
  - a set of bounding boxes with color labels.

### 3.2 rectifier
The projected position, which is affected by the calibration, localization and hd-map label, is not completely reliable. A larger region of interest (ROI) calculated using projected light's position is used to find the accurate bounding box of traffic light. As is shown below, the blue rectangle means the projected light bounding box, which has large offset to real light. The big yellow rectangle is the ROI. 

![example](images/traffic_light/example.jpg)

The traffic light detection is implemented as a regular CNN detection task, which receive a image with ROI as input and output a serial bounding boxes. There may be more lights in ROI than that in input and we need to select the proper lights according to the detection score, input lights' position and shape. If the CNN network couldn't find any light in ROI, all input lights' state will be marked as unknown and the remain steps will be skipped.
 
### 3.3 recogniser
The traffic light recognization is implemented as a typical CNN classification task. The network receive a image with ROI and a list of bounding boxes as input. The output of network is a $4\times n$ vector, representing four probabilities for each box to be black,red,yellow and green. The class with max probability will be regard as the light's state if and only if the probability is large enough. Otherwise, the light's state will be set to black, means the state is not sure.
 
### 3.4 reviser
Since the light can be flashing or shaded and the recognizer is hard to be perfect, the current state may fail to represent real state. A reviser that could correct state is necessary. If receive a sure state,such as red or green, reviser will save and output the state directly. If received state is black or unknown , reviser will look up the saved map. When there are sure state for this light in some time, reviser output this saved state. Otherwise the black or unknown is outputed. Because of the time sequence, yellow only exists after green and before red. Any yellow after red will be reset to red for the sake of safety until green comes.
 
