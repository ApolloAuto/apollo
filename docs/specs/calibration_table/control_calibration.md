Control calibration system automatically generates calibration table for different vehicle models. It includes three parts: a frontend data collection monitor system, a data pipeline upload/download tool for uploading collected data and downloading generated calibration tables, and a visualization tool for performance evaluation.

<!-- # Table of Contents 1\. [Frontend](#frontend) 2\. [Data](#data) - [Upload Tool](#upload) - [Download Tool](#download) 3\. [Visulization](#visulization) -->

 # Frontend

In DreamView, a data collection monitor is presented for monitoring the data calibration process. In vehicle calibration mode, collected data frames are visualized in the data calibration monitor. Data frames are categorized into different driving conditions according to their chassis information. The amount of collected data frames are indicated as progress bars.

## Setup

In the on-vehicle DreamView environment,

1. Choose `vehicle calibration` in `--setup mode--`,
2. Choose `Data Collection Monitor` at `Others` panel.

![](images/calibration_table.png)

The data collection monitor is displayed in DreamView.

## Data collection

When driving, data frames are automatically processed by reading their chassis messages. When a data frame satisfy the speed criterion (speed equal or larger than 0.2 mps), the data frame is categorized by its steering, speed and throttle/brake information. The data collection process is presented by bars in data collection monitor.

There are 21 bars in total in data collection monitor. The overall process is indicated by the top bar. The rest 20 bars indicate 20 driving conditions, including

- Six brake conditions at different speed level

  - low speed (<10 mps) brake pulse
  - middle speed (10 mps ~ 20 mps ) brake pulse
  - high speed (>=20 mps) brake pulsing
  - low speed ( <10 mps) brake tap
  - middle speed (10 mps ~ 20 mps ) brake tap
  - high speed (>=20 mps) brake tap

- Six throttle conditions at different speed level

  - low speed (<10 mps) under throttle
  - middle speed (10 mps ~ 20 mps ) under throttle
  - high speed (>=20 mps) under throttle
  - low speed ( <10 mps) harsh throttle
  - middle speed (10 mps ~ 20 mps ) harsh throttle
  - high speed (>=20 mps) harsh throttle

- Eight steering angle conditions

  - left 0% ~ 20%
  - left 20% ~ 40%
  - left 40% ~ 60%
  - left 60% ~ 100%
  - right 0% ~ 20%
  - right 20% ~ 40%
  - right 40% ~ 60%
  - right 60% ~ 100%

For each bar, there is a blue ribbon indicating collected data frames. When the blue ribbon fills the whole bar, the number of collected frames reaches the target number. There is also a number at right end of each bar indicating the completion percentage.

For calibration table data collection, when the first 13 bars (total progress bar and 12 brake/throttle condition bars) reaches 100% the data collection process is considered as completed. For dynamic model data collection, the data collection process is completed when all bars reaches 100%.

All data are saved in `nvme dirve` or `data/record/`

## Vehicle Configuration

The brake and throttle specs are different between vehicle models. Therefore, the criteria for brake pulsing/tap and hash/under throttle depend on vehicle models. The default setting is based on Lincoln MKZ model. For different vehicle model, these parameters is configurable at

```
/apollo/modules/dreamview/conf/mkz7_data_collection_table.pb.txt
```

(description)

# Data Upload/Download

## Prerequest

Firstly, please make sure you have already finished setting up the **Apollo Fuel Proxy** following documents available at:

https://github.com/ApolloAuto/apollo/blob/master/modules/tools/fuel_proxy/README.md

This is **essential** before you can get enjoy control calibration or other Apollo Fuel based cloud service.

## Folder Structure Requirement

Before uploading your data, take a note of:
1. The folder structure to be maintained is:
   ![](images/file_system1.png)

1. As seen above, the file structure to be maintained is
   ```
   Origin Folder -> Vehicle Folder -> Records + Configuration files
   ```
1. A vehicle folder needs to be created for your vehicle. The name of the folder
   should be the same as seen in Dreamview
1. Inside your folder, create a **Records** folder to hold the data
1. Store all the **Configuration files** along with the Records folder, within
   the **Vehicle** folder

## Upload

Use [bosfs](https://cloud.baidu.com/doc/BOS/BOSCLI/8.5CBOS.20FS.html) to mount
your bucket to local, for example,

```
BUCKET=<bucket>
AK=<access key>
SK=<secret key>
MOUNT=/mnt/bos
# It's required to provide correct BOS region. Please read the document
# https://cloud.baidu.com/doc/BOS/S3.html#.E6.9C.8D.E5.8A.A1.E5.9F.9F.E5.90.8D
REGION=bj

mkdir -p "${MOUNT}"
bosfs "${BUCKET}" "${MOUNT}" -o allow_other,logfile=/tmp/bos-${BUCKET}.log,endpoint=http://su.${REGION}.bcebos.com,ak=${AK},sk=${SK}
```

Then you can copy the prepared data folder to somewhere under /mnt/bos.

## Download

No download needed, the results will be sent to your email associated with your BOS bucket.


# Result Visualization

The docker environment does not support Matplotlib. Thus, result are visualized outside of the docker environment. The following two figures show the visualization result of PC training results.

![](images/throttle.png)

![](images/brake.png)
