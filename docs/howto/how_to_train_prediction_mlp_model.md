## How to train the MLP Deep Learning Model

### Prerequisites
There are 2 prerequisites to training the MLP Deep Learning Model:
#### Download and Install Anaconda
* Please download and install Anaconda from its [website](https://www.anaconda.com/download)

#### Install Dependencies
Run the following commands to install the necessary dependencies:
* **Install numpy**: `conda install numpy`
* **Install tensorflow**: `conda install tensorflow`
* **Install keras (version 1.2.2)**: `conda install -c conda-forge keras=1.2.2`
* **Install h5py**: `conda install h5py`
* **Install protobuf**: `conda install -c conda-forge protobuf`

### Train the Model
The following steps are to be followed in order to train the MLP model using the released demo data. For convenience, we denote `APOLLO` as the path of the local apollo repository, for example, `/home/username/apollo`

1. Create a folder to store offline prediction data using the command `mkdir APOLLO/data/prediction` if it does not exist

2. Open `apollo/modules/prediction/conf/prediction.conf`. Turn on the off-line mode by changing `--noprediction_offline_mode` to `--prediction_offline_mode`

3. Start dev docker using `bash docker/scripts/dev_start.sh` under the apollo folder

4. Enter dev docker using `bash docker/scripts/dev_into.sh` under apollo folder

5. In docker, under `/apollo/`, run `bash apollo.sh build` to compile

6. In docker, under `/apollo/`, download the demo ROSbag by `python docs/demo_guide/rosbag_helper.py demo_2.0.bag`

7. In docker, under `/apollo/`, run prediction module by `bash scripts/prediction.sh start_fe`

8. Open a new terminal window, enter the apollo dev docker using Step 4

9. In the new terminal window, under `/apollo/`, play the demo rosbag using `rosbag play demo_2.0.bag`

10. After the demo ROSbag finishes running in the new terminal window, go to the old terminal window and stop the prediction module by pressing `Ctrl + C`

11. Checkout if there is a file called `feature.0.bin` under the folder `/apollo/data/prediction/`

12. In docker, go to `/apollo/modules/tools/prediction/mlp_train/`, label the data using
`python generate_labels.py -f /apollo/data/prediction/feature.0.bin`. Then check if there is a file called `feature.0.label.bin` under the folder `/apollo/data/prediction/`

13. In docker, under `/apollo/modules/tools/prediction/mlp_train/`, generate H5 files using `python generate_h5.py -f /apollo/data/prediction/feature.0.label.bin`. Then check if there is a file called `feature.0.label.h5` created

14. Exit dev docker

15. Go to the folder `APOLLO/modules/tools/prediction/mlp_train/proto/` run `protoc --python_out=./ fnn_model.proto` to generate fnn_model_pb2.py

16. Go to the folder `APOLLO/modules/tools/prediction/mlp_train/`, run the training model using `python mlp_train.py APOLLO/data/prediction/feature.0.label.h5`

17. The model's evaluation report will be in the file `APOLLO/modules/tools/prediction/mlp_train/evaluation_report.log`

18. The model will be stored in the binary file `APOLLO/modules/tools/prediction/mlp_train/mlp_model.bin`, which can replace the old model in `APOLLO/modules/prediction/data/mlp_vehicle_model.bin` if you think that's better
