# Quick Start Guide
**<font color=##FF0000 size=3>The model_demo directory provides a demo of the model, which is intended for developers to quickly understand the open source data format and how to use the open source data, and we will continue to open end-to-end models and algorithms.</font>**
## Train Model

The train model

	sh run_train.sh
Download the trainsets to the current project.  See `./trainsets` directory.

## Model predict

The predict model

	sh run_predict.sh path/to/steering_model  path/to/acc_model  path/to/test_img  path/to/result

## About
* Data Set Download: [Data Download](https://console.bce.baidu.com/apollo/task/download#/apollo/home)
* Data Set Summary：See  [Data Overview](https://github.com/ApolloAuto/apollo/blob/master/modules/e2e/model_demo/data_overview.md)
* Model Summary：See  [Model Overview](https://github.com/ApolloAuto/apollo/blob/master/modules/e2e/model_demo/model_overview.md)
