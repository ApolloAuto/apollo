# How to run the Map Data Verification Tool

The Map Data Verification tool is designed to help Apollo developers detect any issues in their map data collection process before the data is used for HD Map creation. The benefit of using this tool is to ensure that all issues are detected prior to map creation and also ensures that data can be efficiently recollected based on the suggestions in the tool.

## Running the tool

In order to run your data on this tool, please follow the steps below:

1. Build Apollo as recommended in the [Build Guide](https://github.com/ApolloAuto/apollo/blob/master/docs/howto/how_to_build_and_release.md) until the `./apollo.sh build` step.
2. Once instde dev docker and after running `./apollo.sh build` please go to the folder `modules/tools/map_datachecker/`
3. Starting the server:
    ```bash
    bash server.sh start
    ```
    Note: You should see the following message displayed `Server has been started successfully`. If not, please resolve the errors before trying again

4. Start the record verification:
    ```bash
    bash client.sh --stage record_check --cmd start --record_path path_to_record
    ```
    Note: Do not close your terminal window throughout this process.
    If the verification is successful, you will not see any error messages.

5. Static alignment verification:
    ```bash
    bash client.sh --stage static_align --cmd start
    ```
    Note: The window will display the progress of the static alignment.

6. Figure 8 verification:
    ```bash
    bash client.sh --stage eight_route --cmd start
    ```
    Note: Your window displays the progress of the Figure 8 verification

7. Laps verification:
    ```bash
    bash client.sh --stage loops_check --cmd start
    ```
    Note: Run this command post data collection to check the number of laps

8. Figure 8 verification: This is repeated to verify that the extrinsic parameters have not been changed

9. Static alignment verification: Similar to 5, it is preferable to repeat this verification to ensure high accuracy

10. Stop record verification:
    ```bash
    bash client.sh --stage record_check --cmd stop
    ```
11. Clean the intermediate results
    ```bash
    bash client.sh --stage clean
    ```

## Tips

1. The default value of `cmd` is `start`
2. All error messages will be printed to help better prepare your map data. Please follow the error messages exactly as recommended
