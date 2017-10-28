# Quick Start Guide

*	**<font size=4>The data_usage directory provides a demo which helps developers quickly use the open data which are collected through Baidu’s map collection vehicles.</font>**

*	**<font size=4>The model need to be transfered to the online_system, and the transfered model is not open yet.</font>**


## Train Model

The train model

	sh run_train.sh
Download the trainsets to the current project.  See `./trainsets` directory.

## Model predict

The predict model

	sh run_predict.sh path/to/steering_model  path/to/acc_model  path/to/test_img  path/to/result

## About
* Data Set Download: [Data Download](https://console.bce.baidu.com/apollo/task/download#/apollo/home)
* Data Set Summary：See  [Data Overview](https://github.com/ApolloAuto/apollo/blob/master/modules/e2e/model/data_overview.md)
* Model Summary：See  [Model Overview](https://github.com/ApolloAuto/apollo/blob/master/modules/e2e/model/model_overview.md)