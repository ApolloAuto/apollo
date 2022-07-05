#include "modules/perception/inference/migraphx/mi_net.h"

#include <csignal>
#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"

namespace apollo {
namespace perception {
namespace inference {

struct blob {
  size_t size;
  std::shared_ptr<char> data;
};

struct network_param {
  std::string path;
  std::vector<std::string> input_names;
  std::vector<std::string> output_names;
};

class NetworkInferenceTests : public ::testing::TestWithParam<network_param> {
 protected:
  std::string path;
  std::vector<std::string> input_names;
  std::vector<std::string> output_names;

  std::vector<blob> input_data;
  std::vector<blob> output_data;

  NetworkInferenceTests()
      : path{GetParam().path},
        input_names{GetParam().input_names},
        output_names{GetParam().output_names} {}

  void SetUp() override {
    for (auto &n : input_names) {
      auto data = load_binary_data(path + n);
      input_data.push_back(data);
    }

    for (auto &n : output_names) {
      auto data = load_binary_data(path + n + "_gd");
      output_data.push_back(data);
    }
  }

 private:
  blob load_binary_data(const std::string &filename) {
    std::ifstream ifs(filename, std::ifstream::binary);

    EXPECT_TRUE(ifs);

    ifs.seekg(0, ifs.end);
    size_t size = ifs.tellg();
    ifs.seekg(0, ifs.beg);

    blob blob;
    blob.size = size;
    blob.data.reset(new char[size]);

    ifs.read(reinterpret_cast<char *>(blob.data.get()), size);
    ifs.close();

    return blob;
  }
};

TEST_P(NetworkInferenceTests, InferenceFP32) {
  auto net = std::make_unique<MINet>(path + "deploy.prototxt",
                                     path + "deploy.caffemodel", output_names,
                                     input_names);

  std::map<std::string, std::vector<int>> shape_map{};
  net->Init(shape_map);

  for (size_t i = 0; i < input_names.size(); i++) {
    auto input_blob = net->get_blob(input_names[i]);

    EXPECT_NE(input_blob.get(), nullptr);
    EXPECT_EQ(input_blob->count(), input_data[i].size / sizeof(float));

    auto input_ptr = input_blob->mutable_cpu_data();

    std::memcpy(input_ptr, input_data[i].data.get(), input_data[i].size);

    std::stringstream ss;
    ss << "Input " << input_names[i] << ": ";
    for (size_t k = 0; k < 10; k++) {
      ss << input_ptr[k] << ", ";
    }
    AINFO << ss.str();
  }

  cudaDeviceSynchronize();
  net->Infer();
  cudaDeviceSynchronize();

  for (size_t i = 0; i < output_names.size(); i++) {
    auto output_gd = output_data[i];
    auto output_gd_ptr = reinterpret_cast<float *>(output_gd.data.get());

    const auto output_name = output_names[i];
    const auto output_blob = net->get_blob(output_name);
    EXPECT_NE(output_blob.get(), nullptr);
    EXPECT_EQ(output_blob->count(), output_gd.size / sizeof(float));

    const auto output_ptr = output_blob->mutable_cpu_data();

    size_t err_cnt = 0;
    for (auto i = 0; i < output_blob->count(); i++) {
      EXPECT_NEAR(output_ptr[i], output_gd_ptr[i], 1e-2)
          << output_name << " out: " << output_ptr[i]
          << " expected: " << output_gd_ptr[i] << " idx: " << i
          << " err: " << err_cnt++;

      if (err_cnt > 10) break;
    }
  }
}

const network_param velodyne16{
    "modules/perception/inference/inference_test_data/migraphx/velodyne16/",
    {"data"},
    {"category_score", "heading_pt", "class_score", "confidence_score",
     "instance_pt", "height_pt"}};

const network_param velodyne64{
    "modules/perception/inference/inference_test_data/migraphx/velodyne64/",
    {"data"},
    {"category_score", "heading_pt", "class_score", "confidence_score",
     "instance_pt", "height_pt"}};

const network_param velodyne128{
    "modules/perception/inference/inference_test_data/migraphx/velodyne128/",
    {"data"},
    {"category_score", "heading_pt", "class_score", "confidence_score",
     "instance_pt", "height_pt"}};

const network_param denseline{
    "modules/perception/inference/inference_test_data/migraphx/denseline/",
    {"data"},
    {"conv_out"}};

const network_param darkSCNN{
    "modules/perception/inference/inference_test_data/migraphx/darkSCNN/",
    {"data"},
    {"SCNN_U", "fc_out"}};

const network_param baidu_iter_140000{
    "modules/perception/inference/inference_test_data/migraphx/"
    "traffic_light_detection/",
    {"img"},
    {"network_debug"}};

INSTANTIATE_TEST_SUITE_P(MINet_TEST, NetworkInferenceTests,
                         testing::Values(/*velodyne16, velodyne64, velodyne128,
                                            denseline, darkSCNN,*/
                                         baidu_iter_140000));

}  // namespace inference
}  // namespace perception
}  // namespace apollo