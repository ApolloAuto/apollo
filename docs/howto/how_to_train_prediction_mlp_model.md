## How to train the MLP Deep Learning Model

### Prerequisites
There are 2 prerequisites to training the MLP Deep Learning Model:
#### Download and Install Anaconda
* Please download and install Anaconda from its [website](https://www.anaconda.com/download)

#### Install Dependencies
Run the following commands to install the necessary dependencies:
* **Install numpy**: `conda install numpy`
* **Install tensorflow**: `conda install tensorflow`
* **Install keras**: `conda install -c conda-forge keras`
* **Install h5py**: `conda install h5py`
* **Install protobuf**: `conda install -c conda-forge protobuf`
* **Install PyTorch**: `conda install -c pytorch pytorch`

### Train the Model
The following steps are to be followed in order to train the MLP model using the released demo data. For convenience, we denote `APOLLO` as the path of the local apollo repository, for example, `/home/username/apollo`

1. Create a folder to store offline prediction data using the command `mkdir APOLLO/data/prediction` if it does not exist

1. Start dev docker using `bash docker/scripts/dev_start.sh` under the apollo folder

1. Enter dev docker using `bash docker/scripts/dev_into.sh` under apollo folder

1. In docker, under `/apollo/`, run `bash apollo.sh build` to compile

1. In docker, under `/apollo/`, copy the demo record into `/apollo/data/prediction` by the command: `cp /apollo/docs/demo_guide/demo_3.5.record /apollo/data/prediction/`

1. In docker, under `/apollo/`, run the bash script for feature extraction: `bash modules/tools/prediction/mlp_train/feature_extraction.sh /apollo/data/prediction/ apollo/data/prediction/`, then the feature files will be generated in the folder `/apollo/data/prediction/`.

1. Exit docker, train the cruise model and junction model according to `APOLLO/modules/tools/prediction/mlp_train/cruiseMLP_train.py` and `APOLLO/modules/tools/prediction/mlp_train/junctionMLP_train.py`
