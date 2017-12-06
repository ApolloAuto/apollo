# Run Offline Demo

Apollo provides a method to run simulation if you do not have the required
hardware.

Set up the docker release environment by following the instructions in the
[Install docker](https://github.com/ApolloAuto/apollo/blob/master/docs/howto/how_to_build_and_release.md#docker)
section of the
[Build and Release](https://github.com/ApolloAuto/apollo/blob/master/docs/howto/how_to_build_and_release.md)
page.

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

    The `--loop` option enables rosbag to keep playing the bag in a loop
    playback mode.

4. Open Chrome and go to **localhost:8888** to access Apollo Dreamview, which
   opens the screen below.
    ![](images/dv_trajectory.png)
   The car in Dreamview is happy to move around!

Congratulations!
