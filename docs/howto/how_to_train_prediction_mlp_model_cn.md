## 训练预测MLP深度学习模型

### 前提条件
训练MLP深度学习模式有2个前提条件:
#### 下载并安装Anaconda
* 请从官网下载并安装Anaconda [website](https://www.anaconda.com/download)

#### 安装依赖库
* **安装 numpy**: `conda install numpy`
* **安装 tensorflow**: `conda install tensorflow`
* **安装 keras**: `conda install -c conda-forge keras`
* **安装 h5py**: `conda install h5py`
* **安装 protobuf**: `conda install -c conda-forge protobuf`
* **安装 PyTorch**: `conda install -c pytorch pytorch`

### 训练模型
请按照以下步骤使用演示cyber record来训练MLP模型。为了方便起见，我们把`Apollo`作为本地Apollo的路径，例如，`/home/username/apollo`。

1. 创建存储数据文件夹 `mkdir APOLLO/data/prediction`， 如果它不存在的话。

1. 在APOLLO文件夹下，启动docker `bash docker/scripts/dev_start.sh`。

1. 在APOLLO文件夹下，进入docker `bash docker/scripts/dev_into.sh`。

1. 在docker中，`/apollo/`路径下, 运行 `bash apollo.sh build` 编译代码。

1. 在docker中，`/apollo/`路径下, 从`/apollo/data/prediction` 拷贝演示cyber record到数据文件夹下： `cp /apollo/docs/demo_guide/demo_3.5.record /apollo/data/prediction/`。

1. 在docker中，`/apollo/`路径下, 运行bash脚本进行特征抽取: `bash modules/tools/prediction/mlp_train/feature_extraction.sh /apollo/data/prediction/ apollo/data/prediction/`, 运行完了以后，特征数据文件会出现在 `/apollo/data/prediction/`路径下.

1. 退出docker, 根据 `APOLLO/modules/tools/prediction/mlp_train/cruiseMLP_train.py` 和 `APOLLO/modules/tools/prediction/mlp_train/junctionMLP_train.py` 训练模型。
