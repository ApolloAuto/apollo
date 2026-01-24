#!/usr/bin/env bash

###############################################################################
# Copyright 2023 The Apollo Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
###############################################################################

declare -A dic
models_dir="/apollo/modules/perception/data/models/"

dic=([3d-r4-half_caffe.zip]="http://apollo-perception.bj.bcebos.com/core_model/3d-r4-half_caffe.zip?authorization=bce-auth-v1%2FALTAK6fleneBT08Sn61Gseah1T%2F2024-11-19T03%3A05%3A17Z%2F-1%2Fhost%2F5575ba1e6b5c33e5e989b3b379e16aa9e6546ab28e6f45a54162fadc5dce0ffa" \
     [cnnseg128_caffe.zip]="http://apollo-perception.bj.bcebos.com/core_model/cnnseg128_caffe.zip?authorization=bce-auth-v1%2FALTAK6fleneBT08Sn61Gseah1T%2F2024-11-19T03%3A06%3A36Z%2F-1%2Fhost%2Faec61e93a98ba3a70edfde89fbc5635294df057dc2f5774a59060332f182067a" \
     [cnnseg64_caffe.zip]="http://apollo-perception.bj.bcebos.com/core_model/cnnseg64_caffe.zip?authorization=bce-auth-v1%2FALTAK6fleneBT08Sn61Gseah1T%2F2024-11-19T03%3A07%3A45Z%2F-1%2Fhost%2F4c567da65ebd7e3a20bbb4b9419db6e17f5f233c5867de6d17e70d4bd9f07057" \
     [cnnseg16_caffe.zip]="http://apollo-perception.bj.bcebos.com/core_model/cnnseg16_caffe.zip?authorization=bce-auth-v1%2FALTAK6fleneBT08Sn61Gseah1T%2F2024-11-19T03%3A07%3A59Z%2F-1%2Fhost%2F05cda136284eea1ba84c552c2e5cdd4d6d054c9b23ef209fa87623b746bed185" \
     [horizontal_caffe.zip]="http://apollo-perception.bj.bcebos.com/core_model/horizontal_caffe.zip?authorization=bce-auth-v1%2FALTAK6fleneBT08Sn61Gseah1T%2F2024-11-19T03%3A08%3A14Z%2F-1%2Fhost%2Ffafa1d10174f763828b10ec251b0359da311e8bb4b37824c56e03fcdea61f9f7" \
     [quadrate_caffe.zip]="http://apollo-perception.bj.bcebos.com/core_model/quadrate_caffe.zip?authorization=bce-auth-v1%2FALTAK6fleneBT08Sn61Gseah1T%2F2024-11-19T03%3A08%3A29Z%2F-1%2Fhost%2F10bc4ce80bd0497ea92188aff6d28a9c35dbbd12ae12cd836e9b19e4ab0a79d3" \
     [vertical_caffe.zip]="http://apollo-perception.bj.bcebos.com/core_model/vertical_caffe.zip?authorization=bce-auth-v1%2FALTAK6fleneBT08Sn61Gseah1T%2F2024-11-19T03%3A08%3A41Z%2F-1%2Fhost%2F6ea0744dbc5b5e83a673e241c07ad8d3f5930763b1bc9c05084cac5c16b07d02" \
     [tl_detection_caffe.zip]="http://apollo-perception.bj.bcebos.com/core_model/tl_detection_caffe.zip?authorization=bce-auth-v1%2FALTAK6fleneBT08Sn61Gseah1T%2F2024-11-19T03%3A08%3A55Z%2F-1%2Fhost%2F524a2c191d2a332d500232600fd0dd30db21591adc83f728dc8ae5daef4ef335" \
     [smoke_torch.zip]="http://apollo-perception.bj.bcebos.com/core_model/smoke_torch.zip?authorization=bce-auth-v1%2FALTAK6fleneBT08Sn61Gseah1T%2F2024-11-19T03%3A09%3A10Z%2F-1%2Fhost%2Ffb046eae4495ef593e8cc468e6b62c9a504223d862c27596a878a8b250b756c5" \
     [yolox3d_onnx.zip]="http://apollo-perception.bj.bcebos.com/core_model/yolox3d_onnx.zip?authorization=bce-auth-v1%2FALTAK6fleneBT08Sn61Gseah1T%2F2024-11-19T03%3A09%3A24Z%2F-1%2Fhost%2F16ce6949d86c2521e4ac1d4e3f3d6d6d266b7aece8d68e18d12ee295a77de5fa" \
     [center_point_paddle.zip]="http://apollo-perception.bj.bcebos.com/core_model/center_point_paddle.zip?authorization=bce-auth-v1%2FALTAK6fleneBT08Sn61Gseah1T%2F2024-11-19T03%3A09%3A36Z%2F-1%2Fhost%2F109777339cc3fab17f5f54760711e04139dcdba62f2aa75e955d91d8d21cd963" \
     [point_pillars_radar4d_torch.zip]="http://apollo-perception.bj.bcebos.com/core_model/point_pillars_radar4d_torch.zip?authorization=bce-auth-v1%2FALTAK6fleneBT08Sn61Gseah1T%2F2024-11-19T03%3A09%3A49Z%2F-1%2Fhost%2Ff7bfb72f9cdbb866d5965a16828fa42275d97844b9f546237bfa000de252063d" \
     [point_pillars_torch.zip]="http://apollo-perception.bj.bcebos.com/core_model/point_pillars_torch.zip?authorization=bce-auth-v1%2FALTAK6fleneBT08Sn61Gseah1T%2F2024-11-19T03%3A10%3A07Z%2F-1%2Fhost%2F4d0ad2d272b36fcccb7c0a8280e3af1d665cc1dff393a877ce90b7ca0ea5a5ff" \
     [mask_pillars_torch.zip]="http://apollo-perception.bj.bcebos.com/core_model/mask_pillars_torch.zip?authorization=bce-auth-v1%2FALTAK6fleneBT08Sn61Gseah1T%2F2024-11-19T03%3A10%3A20Z%2F-1%2Fhost%2Feb4e3afe11c4a58be4421cb5271d780e4f648feb2c18b1b9d022aed04da1d838" \
    )

cd "/apollo"

for key in $(echo ${!dic[*]}); do
    download_link=${dic[$key]}
    wget -O ${key} ${download_link}
    unzip ${key}
    for i in `ls ${key::-4}`; do
        [[ -e ${models_dir}${key::-4}/${i} ]] && rm -f ${models_dir}${key::-4}/${i}
        sudo bash -c "mv -f ${key::-4}/${i} ${models_dir}${key::-4}/"
    done
    rm -rf ${key::-4} ${key}
  done
