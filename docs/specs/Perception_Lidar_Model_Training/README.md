# Open Perception Lidar Model Training Service

## Overview

Open Perception Lidar Model Training Service is a cloud based service to train pointpillars algorithm based perception lidar model using data you provide,to better detect obstacles in your environment.


## Prerequisites

- [Apollo](https://github.com/ApolloAuto/apollo) 6.0 or higher version.

- Cloud Fuel services registered according to [Apollo_Fuel](https://github.com/ApolloAuto/apollo/blob/master/docs/Apollo_Fuel/apply_bos_account_cn.md) on [Apollo Dreamland](http://bce.apollo.auto/)


## Main Steps

- Data collection

- Job submission

- Get trained models


## Data Collection

### Data Recording

Collect sensor data from lidar and cameras in different scenarios covering your antonomous driving environment as much as possible,please make sure the scenarios have different obstacles such as pedestrians and verhicles.Then label these sensor data using kitti data format.

### Data format

- **We use [Kitti data format](http://www.cvlibs.net/datasets/kitti/eval_object.php?obj_benchmark=3d) as training data format**：
    You need to organize your data like this:

```
    INPUT_DATA_PATH:
        training:
            calib
            image_2
            label_2
            velodyne
        testing:
            calib
            image_2
            velodyne
        train.txt
	val.txt
        trainval.txt
        test.txt 
```

- Supported obstacle detection categories:
  
```
    bus, Car, construction_vehicle, Truck, barrier, Cyclist, motorcycle, Pedestrian, traffic_cone
```

    When label your data,`type` must be the one of above categories(please note the uppercase). 


## Job Submission

### Upload data to BOS

Here is the folder structure requirements for job submission：
1. Input Data Path:upload your data as in [Data format](###Data-format) to INPUT_DATA_PATH directory.

1. Output Data Path:If model is trained suscessfully,the onnx file will be saved to OUTPUT_DATA_PATH directory.

### Submit job in Dreamland

Go to [Apollo Dreamland](http://bce.apollo.auto/), login with **Baidu** account, choose `Apollo Fuel --> Jobs`，`New Job`, `Perception Lidar Model Training`，and input the correct BOS path as in [Upload data to BOS](###Upload-data-to-BOS) section.


## Get trained models

- After job is done, you should be expecting one email per job including the results and `Model Path`.

![](images/perception_email.png)
