# Run Offline Demo

Apollo provides a method to run simulation if you do not have the required hardware.

Set up the docker release environment by following the instructions in the Install docker section of the Build and Release from Sources page.

Setup steps:

1. Start the docker release environment using the command:

    ```
    bash docker/scripts/release_start.sh
    ```

2. Enter the docker release environment:

    ```
    bash docker/scripts/release_into.sh
    ```

3. Now you can play the rosbag in `docs/demo_guide/demo.bag` using the command:

    ```
    rosbag play docs/demo_guide/demo.bag --loop
    # or 
    bash docs/demo_guide/rosbag_helper.sh download #download rosbag
    rosbag play docs/demo_guide/demo_1.5.np.bag --loop
    ```

    The `--loop` option enables rosbag to keep playing the bag in a loop playback mode.

4. Open Chrome and go to **localhost:8887** to access Apollo HMI, which opens the screen below.
    ![](images/start_hmi.png)

5. Click 'Dreamview' switch in the right middle panel.
    ![](images/dreamview_enable.png)

6. Click upper-right "Dreamview" button to open Dreamview.
    ![](images/dreamview_launch.png)

7. Dreamview is loaded in browser with address **localhost:8888**.
    ![](images/dv_trajectory.png)The car in Dreamview is happy to move around in the screen!

Congratulations!
