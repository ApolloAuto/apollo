/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include <gtest/gtest.h>
#include "modules/perception/common/machine_learning/rdf_helper.h"
#include "modules/perception/common/machine_learning/rdf_model_ultra.h"

namespace apollo {
namespace perception {
namespace common {

TEST(RDFModelUltraTest, bit_operation_test) {
  uint32_t var_idx = 0x00000010;
  uint32_t dest = 0x10011000;
  uint32_t i_result = SetVarIdx(var_idx, dest);
  EXPECT_EQ(0x01011000, i_result);

  uint32_t childpos = 0x00000080;
  i_result = SetChildPos(childpos, dest);
  EXPECT_EQ(0x10000080, i_result);

  i_result = GetVarIdx(dest);
  EXPECT_EQ(0x00000100, i_result);

  i_result = GetChildPos(dest);
  EXPECT_EQ(0x00011000, i_result);

  float f_result = SetLeftChildLeaf(Uint32ToFloatByBit(dest));
  EXPECT_EQ(0x10011002, FloatToUint32ByBit(f_result));

  f_result = SetLeftChildUnleaf(Uint32ToFloatByBit(dest));
  EXPECT_EQ(0x10011000, FloatToUint32ByBit(f_result));

  f_result = SetRightChildLeaf(Uint32ToFloatByBit(dest));
  EXPECT_EQ(0x10011001, FloatToUint32ByBit(f_result));

  f_result = SetRightChildUnleaf(Uint32ToFloatByBit(dest));
  EXPECT_EQ(0x10011000, FloatToUint32ByBit(f_result));

  bool b_result = IsLeftChildLeaf(Uint32ToFloatByBit(dest));
  EXPECT_FALSE(b_result);

  b_result = IsRightChildLeaf(Uint32ToFloatByBit(dest));
  EXPECT_FALSE(b_result);

  b_result = IsBothChildLeaf(Uint32ToFloatByBit(dest));
  EXPECT_FALSE(b_result);

  b_result = IsBothChildNotLeaf(Uint32ToFloatByBit(dest));
  EXPECT_TRUE(b_result);

  f_result = SetChildClassOnSplitVal(0x00000003, Uint32ToFloatByBit(dest));
  EXPECT_EQ(0x1001100c, FloatToUint32ByBit(f_result));

  i_result = GetChildClassOnSplitVal(Uint32ToFloatByBit(dest));
  EXPECT_EQ(0, i_result);

  i_result = SetChildClassOnPos(0x00000003, dest);
  EXPECT_EQ(0x10011003, i_result);

  i_result = GetChildClassOnPos(dest);
  EXPECT_EQ(0x00000000, i_result);
}

TEST(RDFModelUltraTest, trans_old_model_to_ultra_model_test) {
  RDFModelUltra ultra_model;
  bool model_loaded = ultra_model.IsModelLoaded();
  EXPECT_FALSE(model_loaded);
  bool trans_res = ultra_model.TransOldModelToUltraModel(
      "./common_test_data/machine_learning/data/not_exist_model");
  EXPECT_FALSE(trans_res);
  trans_res = ultra_model.TransOldModelToUltraModel(
      "./common_test_data/machine_learning/data/normal_correct_model.bin");
  EXPECT_TRUE(trans_res);
  model_loaded = ultra_model.IsModelLoaded();
  EXPECT_TRUE(model_loaded);
  bool save_res = ultra_model.SaveBinModel(
      "./common_test_data/machine_learning/data/ultra_correct_model.bin");
  EXPECT_TRUE(save_res);

  const RDFModelUltra::ForestHead* head = ultra_model.GetForestHeader();
  EXPECT_EQ(2, head->tree_num);
  EXPECT_EQ(4, head->tree_node_num);
  EXPECT_EQ(0, head->tree_pos_list[0]);
  EXPECT_EQ(2, head->tree_pos_list[1]);

  const std::vector<RDFModelUltra::TreeNodeUltra>* ultra_tree_nodes =
      ultra_model.GetTreeNodeUltraList();
  EXPECT_EQ(4, ultra_tree_nodes->size());
}

TEST(RDFModelUltraTest, load_predict_test) {
  std::vector<float> test_feature1 = {0.5, 2.5, 2.6, 4.5, 4.6,
                                      6.5, 6.6, 8.5, 8.6, 10.5};
  std::vector<float> test_feature2 = {1.5, 1.5, 3.6, 3.5, 5.6,
                                      5.5, 7.6, 7.5, 9.6, 9.5};
  std::vector<float> test_features = {0.5, 2.5, 2.6,  4.5, 4.6, 6.5, 6.6,
                                      8.5, 8.6, 10.5, 1.5, 1.5, 3.6, 3.5,
                                      5.6, 5.5, 7.6,  7.5, 9.6, 9.5};
  RDFModelUltra ultra_model;
  bool load_res = ultra_model.LoadBinModel(
      "./common_test_data/machine_learning/data/ultra_correct_model.bin");
  EXPECT_TRUE(load_res);
  int ultra_tree_node_idx = ultra_model.GetTreeNodeUltraIdx(nullptr);
  EXPECT_EQ(-1, ultra_tree_node_idx);
  ultra_tree_node_idx =
      ultra_model.GetTreeNodeUltraIdx(ultra_model.tree_node_ultra_list_.data());
  EXPECT_EQ(0, ultra_tree_node_idx);
  std::vector<float> probs;
  float predict_res = ultra_model.ForestPredict(test_feature1, &probs);
  EXPECT_EQ(1, static_cast<int>(predict_res));
  predict_res = ultra_model.ForestPredict(test_feature2, &probs);
  EXPECT_EQ(0, static_cast<int>(predict_res));
  ultra_model.BatchForestPredict(test_features.data(), probs.data(), 10, 2, 2);
  EXPECT_FLOAT_EQ(0.0, probs[0]);
  EXPECT_FLOAT_EQ(1.0, probs[1]);
  EXPECT_FLOAT_EQ(0.5, probs[2]);
  EXPECT_FLOAT_EQ(0.5, probs[3]);

  ultra_model.ClearModel();
  bool model_loaded = ultra_model.IsModelLoaded();
  EXPECT_FALSE(model_loaded);
  load_res = ultra_model.LoadBinModel(
      "./common_test_data/machine_learning/data/normal_correct_model.bin");
  EXPECT_FALSE(load_res);

  load_res = ultra_model.LoadBinModel("not_exist_model");
  EXPECT_FALSE(load_res);

  bool save_res = ultra_model.SaveBinModel(
      "./common_test_data/machine_learning/data/not_exist_model");
  EXPECT_FALSE(save_res);
}

TEST(RDFModelTest, load_predict_test) {
  std::vector<float> test_feature1 = {0.5, 2.5, 2.6, 4.5, 4.6,
                                      6.5, 6.6, 8.5, 8.6, 10.5};
  std::vector<float> test_feature2 = {1.5, 1.5, 3.6, 3.5, 5.6,
                                      5.5, 7.6, 7.5, 9.6, 9.5};
  std::vector<float> test_features = {0.5, 2.5, 2.6,  4.5, 4.6, 6.5, 6.6,
                                      8.5, 8.6, 10.5, 1.5, 1.5, 3.6, 3.5,
                                      5.6, 5.5, 7.6,  7.5, 9.6, 9.5};
  RDFModel model;
  bool load_res = model.LoadBinModel(
      "./common_test_data/machine_learning/data/normal_correct_model.bin");
  EXPECT_TRUE(load_res);
  const std::vector<RDFModel::TreeNode>* node_list = model.GetTreeNodeList();
  EXPECT_EQ(10, node_list->size());
  int tree_node_idx = model.GetTreeNodeIdx(nullptr);
  EXPECT_EQ(-1, tree_node_idx);
  std::vector<float> probs;
  float predict_res = model.ForestPredict(test_feature1, &probs);
  EXPECT_EQ(1, static_cast<int>(predict_res));
  predict_res = model.ForestPredict(test_feature2, &probs);
  EXPECT_EQ(0, static_cast<int>(predict_res));
  predict_res = model.ForestPredict(test_feature2, nullptr);
  EXPECT_EQ(0, static_cast<int>(predict_res));
  model.BatchForestPredict(test_features.data(), probs.data(), 10, 2, 2);
  EXPECT_FLOAT_EQ(0.0, probs[0]);
  EXPECT_FLOAT_EQ(1.0, probs[1]);
  EXPECT_FLOAT_EQ(0.5, probs[2]);
  EXPECT_FLOAT_EQ(0.5, probs[3]);

  model.ClearModel();
  bool model_loaded = model.IsModelLoaded();
  EXPECT_FALSE(model_loaded);
  load_res = model.LoadBinModel(
      "./common_test_data/machine_learning/data/ultra_correct_model.bin");
  EXPECT_FALSE(load_res);

  load_res = model.LoadBinModel("not_exist_model");
  EXPECT_FALSE(load_res);

  bool save_res = model.SaveBinModel(
      "./common_test_data/machine_learning/data/not_exist_model");
  EXPECT_FALSE(save_res);
}

}  // namespace common
}  // namespace perception
}  // namespace apollo
