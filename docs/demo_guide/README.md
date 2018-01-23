# Run Offline Demo

Apollo provides a method to run simulation if you do not have the required
hardware.

Set up the docker development environment by following the instructions in the
[Install docker](https://github.com/ApolloAuto/apollo/blob/master/docs/howto/how_to_build.md#docker)
section of the
[Build](https://github.com/ApolloAuto/apollo/blob/master/docs/howto/how_to_build.md)
page.

Setup steps:

1. Start the docker development environment using the command:

    ```
    bash docker/scripts/dev_start.sh
    ```

2. Enter the docker development environment:

    ```
    bash docker/scripts/dev_into.sh
    ```
3. Build Apollo inside docker

    ```
    bash apollo.sh build
    ```

4. Now you can play the rosbag in command:

    ```
    sudo bash docs/demo_guide/rosbag_helper.sh download #download rosbag
    rosbag play docs/demo_guide/demo_2.0.bag --loop
    ```

    The `--loop` option enables rosbag to keep playing the bag in a loop
    playback mode.

5. Open Chrome and go to **localhost:8888** to access Apollo Dreamview, which
   opens the screen below.
    ![](images/dv_trajectory.png)
   The car in Dreamview is happy to move around!

Congratulations!
