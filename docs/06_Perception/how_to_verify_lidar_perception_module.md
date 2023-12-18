## Scenario 4: Perceptual lidar function test

This scenario introduces how to use the software package to start the perception lidar module, helping developers to get familiar with the Apollo perception module and lay the foundation. You can observe the detection results during the operation of the perception lidar by playing the record data package provided by Apollo.

### Prerequisites

This document assumes that you have followed [Package Installation](https://apollo.baidu.com/Apollo-Homepage-Document/Apollo_Doc_CN_8_0/%E5%AE%89%E8%A3%85%E8%AF%B4%E6%98%8E/%E8%BD%AF%E4%BB%B6%E5%8C%85%E5%AE%89%E8%A3%85/%E8%BD%AF%E4%BB%B6%E5%8C%85%E5%AE%89%E8%A3%85/#%E5%AE%89%E8%A3%85apollo%E7%8E%AF%E5%A2%83%E7%AE%A1%E7%90%86%E5%B7%A5%E5%85%B7) > Install the Apollo environment management tool to complete step 1 and step 2. Compared with the above three scenarios, the function of the test perception module needs to use the GPU, so the GPU image of the software package must be obtained for test verification .

### Step 1: Start and enter the Apollo Docker environment

1. Create a workspace:

   ```shell
   mkdir apollo_v8.0
   cd apollo_v8.0
   ```

2. Enter the following command to enter the container environment in GPU mode:

   ```bash
   aem start_gpu -f
   ```

3. Enter the following command to enter the container:

   ```bash
   aem enter
   ```

4. Initialize the workspace:

   ```shell
   aem init
   ```

### Step 2: Download the record data package

1. Enter the following command to download the data package:

   ```bash
   wget https://apollo-system.bj.bcebos.com/dataset/6.0_edu/sensor_rgb.tar.xz
   ```

2. Create a directory and extract the downloaded installation package into this directory:

   ```bash
   sudo mkdir -p ./data/bag/
   sudo tar -xzvf sensor_rgb.tar.xz -C ./data/bag/
   ```

### Step 3: Install DreamView

In the same terminal, enter the following command to install the DreamView program.

```bash
buildtool install --legacy dreamview-dev monitor-dev
```

### Step 4: Install transform, perception and localization

1. In the same terminal, enter the following command to install the perception program.

   ```bash
   buildtool install --legacy perception-dev
   ```

2. Enter the following commands to install the localization , v2x and transform programs.

   ```bash
    buildtool install --legacy localization-dev v2x-dev transform-dev
   ```

### Step 5: Module running

1. In the same terminal, enter the following command to start Apollo's DreamView program.

   ```bash
   aem bootstrap start
   ```

   Open the browser and enter the `localhost:8888` address, select the model, vehicle configuration, and map.

   ![包-dreamview2.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_8_0/%E5%8C%85-dreamview2_3d6fa5c.png)

   Click the Module Controller module in the status bar on the left side of the page to enable the Transform module:

   ![包- transform.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_8_0/%E5%8C%85-%20transform_e13afb6.png)

2. Use the mainboard method to enable the Lidar module:

   ```bash
    mainboard -d /apollo/modules/perception/production/dag/dag_streaming_perception_lidar.dag
   ```

### Step 6: Result verification

1. It is necessary to use the -k parameter to mask out the perception channel data contained in the record.

   ```bash
    cyber_recorder play -f ./data/bag/sensor_rgb.record -k /perception/vehicle/obstacles /apollo/perception/obstacles /apollo/perception/traffic_light /apollo/prediction
   ```

2. Verify detection results: View detection results in DreamView. Click LayerMenu in the toolbar on the left side of Dreamview, open the Point Cloud in Perception, and select the corresponding channel to view the point cloud data. Check whether the 3D detection results can correspond to the Lidar sensor data.

   ![结果1.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_8_0/%E7%BB%93%E6%9E%9C1_35771bc.png)

   View Results:

   ![包-结果2.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_8_0/%E5%8C%85-%E7%BB%93%E6%9E%9C2_1be66a4.png)

### Step 7: Model Replacement

The following describes the parameter configuration and replacement process of the MASK_PILLARS_DETECTION, CNN_SEGMENTATION and CENTER_POINT_DETECTION models in the lidar detection process. You can easily replace these configurations in `lidar_detection_pipeline.pb.txt` to load and run different models.

#### MASK_PILLARS_DETECTION model replacement

Modify the configuration file content in `lidar_detection_pipeline.pb.txt`:

```bash
 vim /apollo/modules/perception/pipeline/config/lidar_detection_pipeline.pb.txt
```

Replace stage_type with MASK_PILLARS_DETECTION:

```bash
stage_type: MASK_PILLARS_DETECTION
```

![mask1.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_8_0/mask1_f067520.png)

And modify the configuration file information content of the corresponding stage:

```bash
stage_config: {
  stage_type: MASK_PILLARS_DETECTION
  enabled: true
}
```

![mask2.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_8_0/mask2_f5373a5.png)

After saving the modified configuration file, enable the Lidar module and play the record to verify the detection result:

![mask3.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_8_0/mask3_7d0d0bd.png)

#### CNN_SEGMENTATION model replacement

Modify the configuration file content in `lidar_detection_pipeline.pb.txt`: replace stage_type with CNN_SEGMENTATION, and modify the configuration file content of the corresponding stage accordingly:

```bash
stage_type: CNN_SEGMENTATION
stage_config: {
  stage_type: CNN_SEGMENTATION
  enabled: true

  cnnseg_config: {
    sensor_name: "velodyne128"
    param_file: "/apollo/modules/perception/production/data/perception/lidar/models/cnnseg/cnnseg64_param.conf"
    proto_file: "/apollo/modules/perception/production/data/perception/lidar/models/cnnseg/cnnseg64_caffe/deploy.prototxt"
    weight_file: "/apollo/modules/perception/production/data/perception/lidar/models/cnnseg/cnnseg64_caffe/deploy.caffemodel"
    engine_file: "/apollo/modules/perception/production/data/perception/lidar/models/cnnseg/cnnseg64_caffe/engine.conf"
  }
}
```

Enable the lidar module and play the record verification detection result:

![cnn1.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_8_0/cnn1_8d22d9f.png)

#### CENTER_POINT_DETECTION model replacement

Replace stage_type with CENTER_POINT_DETECTION:

```bash
stage_type: CENTER_POINT_DETECTION
```

Modify the configuration file information content of the corresponding stage:

```bash
stage_config: {
  stage_type: CENTER_POINT_DETECTION
  enabled: true
}
```

Enable the lidar module and play the record verification detection result:

![centor1.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_8_0/centor1_edea96e.png)