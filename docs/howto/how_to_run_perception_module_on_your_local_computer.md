# How to Run Perception Module on Your Local Computer

The perception module requires Nvidia GPU and CUDA installed to run the perception algorithms with Caffe. We have already installed the CUDA and Caffe libraries in the released docker. However, the Nvidia GPU driver is not installed in the released dev docker image. To run the perception module with CUDA acceleration, we suggest to install the exactly same version of Nvidia driver in the docker as the one installed in your host machine, and build Apollo with GPU option.

We provide a step-by-step instruction on running perception module with Nvidia GPU as below:
1. Get into the docker container via: 
    ```bash
    $APOLLO_HOME/docker/scripts/dev_start.sh
    $APOLLO_HOME/docker/scripts/dev_into.sh
    ```
2. Build Apollo
    ```bash
    ./apollo.sh build_opt_gpu
    ```
3. Run bootstrap.sh
    ```bash
    bootstrap.sh
    ```
4. Launch Dreamview from your web browser by typing following address
http://localhost:8888/

5. Select your car and map using the dropdowm options in the top right corner in Dreamview

6. Select the transform button in Dreamview or type the following command in your terminal
    ```bash
    cyber_launch start /apollo/modules/transform/launch/static_transform.launch
    ```
7. If the image is compressed, launch the image decompression module
    ```
    cyber_launch start modules/drivers/tools/image_decompress/launch/image_decompress.launch
    ```

8. Launch the perception modules

    - If you want to launch all modules 
    ```
    cyber_launch start /apollo/modules/perception/production/launch/perception_all.launch
    ```

    - If you want to test camera-based obstacle and lane detection
    ```
    cyber_launch start /apollo/modules/perception/production/launch/perception_camera.launch
    ```

9. If you want to visualize camera-based results overlaid on the captured image and in bird view, mark `enable_visualization: true` in `modules/perception/production/conf/perception/camera/fusion_camera_detection_component`

10. Play your recorded bag
    ```
    cyber_recorder play -f /apollo/data/bag/anybag -r 0.2
    ```

Please note that the Nvidia driver should be installed appropriately even if the perception module is running in Caffe CPU_ONLY mode (i.e., using `./apollo.sh build` or `./apollo.sh build_opt` to build the perception module). Please see the detailed instruction of perception module in [the perception README](https://github.com/ApolloAuto/apollo/blob/master/modules/perception/README.md).
