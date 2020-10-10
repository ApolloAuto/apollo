/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/perception/camera/common/data_provider.h"
#include "modules/perception/camera/common/util.h"
#include "modules/perception/camera/test/camera_common_io_util.h"
#include "modules/perception/inference/utils/cuda_util.h"
#include "modules/perception/inference/utils/util.h"

namespace apollo {
namespace perception {
namespace camera {

TEST(UtilTest, CoverTest) {
  base::RectF rect1(1, 1, 10, 10);
  base::RectF rect2(1, 1, 6, 6);
  base::RectF rect3(100, 100, 10, 10);
  ASSERT_TRUE(IsCovered(rect1, rect2, 0.3f));
  ASSERT_FALSE(IsCovered(rect1, rect2, 0.9f));
  ASSERT_FALSE(IsCoveredVertical(rect1, rect3, 0.1f));
  ASSERT_FALSE(IsCoveredHorizon(rect1, rect3, 0.1f));
  ASSERT_FALSE(IsCoveredVertical(rect1, rect3, 0.9f));
  ASSERT_FALSE(IsCoveredHorizon(rect1, rect3, 0.9f));
  ASSERT_FALSE(IsCoveredVertical(rect1, rect2, 0.9f));
  ASSERT_FALSE(IsCoveredHorizon(rect1, rect2, 0.9f));
  ASSERT_TRUE(IsCoveredVertical(rect1, rect2, 0.5f));
  ASSERT_TRUE(IsCoveredHorizon(rect1, rect2, 0.5f));
}
TEST(UtilTest, ContainTest) {
  {
    int a = 10;
    std::vector<int> array = {1, 2, 3, 4, 5, 6, 7};
    ASSERT_FALSE(Contain(array, a));
    array.push_back(10);
    ASSERT_TRUE(Contain(array, a));
  }
  {
    std::string str = "haha";
    std::vector<std::string> array = {"a", "b", "c"};
    ASSERT_FALSE(Contain(array, str));
    array.push_back("haha");
    ASSERT_TRUE(Contain(array, str));
  }
  {
    base::ObjectSubType type = base::ObjectSubType::CAR;
    std::vector<base::ObjectSubType> array = {base::ObjectSubType::VAN,
                                              base::ObjectSubType::PEDESTRIAN};
    ASSERT_FALSE(Contain(array, type));
    array.push_back(base::ObjectSubType::CAR);
    ASSERT_TRUE(Contain(array, type));
  }
}

TEST(UtilTest, BorderTest) {
  {
    int width = 400;
    int height = 300;

    int border_size = 50;

    base::RectI roi1(0, 0, 20, 20);
    ASSERT_TRUE(OutOfValidRegion(roi1, width, height, border_size));

    base::RectI roi2(0, 100, 20, 20);
    ASSERT_TRUE(OutOfValidRegion(roi2, width, height, border_size));

    base::RectI roi3(100, 0, 20, 20);
    ASSERT_TRUE(OutOfValidRegion(roi3, width, height, border_size));

    base::RectI roi4(300, 200, 100, 100);
    ASSERT_TRUE(OutOfValidRegion(roi4, width, height, border_size));

    base::RectI roi5(300, 200, 10, 100);
    ASSERT_TRUE(OutOfValidRegion(roi5, width, height, border_size));

    base::RectI roi6(300, 200, 100, 10);
    ASSERT_TRUE(OutOfValidRegion(roi6, width, height, border_size));
  }

  {
    float width = 1.f;
    float height = 2.f;

    ASSERT_FALSE(
        OutOfValidRegion(base::RectF(0.f, 0.f, 1.f, 2.f), width, height));
    ASSERT_TRUE(
        OutOfValidRegion(base::RectF(0.f, 0.f, 1.f, 2.f), width, height, 0.1f));
    ASSERT_TRUE(OutOfValidRegion(base::RectF(0.5f, 0.f, 0.5f, 2.f), width,
                                 height, 0.1f));
    ASSERT_TRUE(OutOfValidRegion(base::RectF(0.2f, 0.2f, 0.5f, 1.9f), width,
                                 height, 0.1f));
  }
}

TEST(UtilTest, RefineBoxTest) {
  {
    base::RectI rect(-1, 2, 100, 100);
    base::RectI result_rect;
    RefineBox(rect, 1920, 1080, &result_rect);
    ASSERT_FALSE(OutOfValidRegion(result_rect, 1920, 1080));

    base::BBox2DI box(rect);
    base::BBox2DI result_box;
    RefineBox(box, 1920, 1080, &result_box);
    ASSERT_FALSE(OutOfValidRegion(result_box, 1920, 1080));
  }

  {
    base::RectI rect(1, -2, 100, 100);
    base::RectI result_rect;
    RefineBox(rect, 1920, 1080, &result_rect);
    ASSERT_FALSE(OutOfValidRegion(result_rect, 1920, 1080));

    base::BBox2DI box(rect);
    base::BBox2DI result_box;
    RefineBox(box, 1920, 1080, &result_box);
    ASSERT_FALSE(OutOfValidRegion(result_box, 1920, 1080));
  }

  {
    base::RectI rect(1, 2, 10000, 10000);
    base::RectI result_rect;
    RefineBox(rect, 1920, 1080, &result_rect);
    ASSERT_FALSE(OutOfValidRegion(result_rect, 1920, 1080));

    base::BBox2DI box(rect);
    base::BBox2DI result_box;
    RefineBox(box, 1920, 1080, &result_box);
    ASSERT_FALSE(OutOfValidRegion(result_box, 1920, 1080));
  }

  {
    base::RectI rect(100000, 2000000, 100, 100);
    base::RectI result_rect;
    RefineBox(rect, 1920, 1080, &result_rect);
    ASSERT_FALSE(OutOfValidRegion(result_rect, 1920, 1080));

    base::BBox2DI box(rect);
    base::BBox2DI result_box;
    RefineBox(box, 1920, 1080, &result_box);
    ASSERT_FALSE(OutOfValidRegion(result_box, 1920, 1080));
  }

  {
    base::RectI rect(100, 100, -1, -1);
    base::RectI result_rect;
    RefineBox(rect, 1920, 1080, &result_rect);
    EXPECT_EQ(result_rect.Area(), 0);

    base::BBox2DI box(rect);
    base::BBox2DI result_box;
    RefineBox(box, 1920, 1080, &result_box);
    EXPECT_EQ(result_box.Area(), 0);
  }

  {
    base::RectI rect(100, 100, -1, -1);
    base::RectI *result_rect = nullptr;
    RefineBox(rect, 1920, 1080, result_rect);
    EXPECT_EQ(result_rect, nullptr);

    base::BBox2DI box(rect);
    base::BBox2DI *result_box = nullptr;
    RefineBox(box, 1920, 1080, result_box);
    EXPECT_EQ(result_box, nullptr);
  }

  {
    base::RectF rect(-1.f, 2.f, 100.f, 100.f);
    base::RectF result_rect;
    RefineBox(rect, 1920.f, 1080.f, &result_rect);
    ASSERT_FALSE(OutOfValidRegion(result_rect, 1920.f, 1080.f));

    base::BBox2DF box(rect);
    base::BBox2DF result_box;
    RefineBox(box, 1920.f, 1080.f, &result_box);
    ASSERT_FALSE(OutOfValidRegion(result_box, 1920.f, 1080.f));
  }

  {
    base::RectF rect(1.f, -2.f, 100.f, 100.f);
    base::RectF result_rect;
    RefineBox(rect, 1920.f, 1080.f, &result_rect);
    ASSERT_FALSE(OutOfValidRegion(result_rect, 1920.f, 1080.f));

    base::BBox2DF box(rect);
    base::BBox2DF result_box;
    RefineBox(box, 1920.f, 1080.f, &result_box);
    ASSERT_FALSE(OutOfValidRegion(result_box, 1920.f, 1080.f));
  }

  {
    base::RectF rect(1.f, 2.f, 10000.f, 10000.f);
    base::RectF result_rect;
    RefineBox(rect, 1920.f, 1080.f, &result_rect);
    ASSERT_FALSE(OutOfValidRegion(result_rect, 1920.f, 1080.f));

    base::BBox2DF box(rect);
    base::BBox2DF result_box;
    RefineBox(box, 1920.f, 1080.f, &result_box);
    ASSERT_FALSE(OutOfValidRegion(result_box, 1920.f, 1080.f));
  }

  {
    base::RectF rect(100000.f, 2000000.f, 100.f, 100.f);
    base::RectF result_rect;
    RefineBox(rect, 1920.f, 1080.f, &result_rect);
    ASSERT_FALSE(OutOfValidRegion(result_rect, 1920.f, 1080.f));

    base::BBox2DF box(rect);
    base::BBox2DF result_box;
    RefineBox(box, 1920.f, 1080.f, &result_box);
    ASSERT_FALSE(OutOfValidRegion(result_box, 1920.f, 1080.f));
  }

  {
    base::RectF rect(100.f, 100.f, -1.f, -1.f);
    base::RectF result_rect;
    RefineBox(rect, 1920.f, 1080.f, &result_rect);
    EXPECT_EQ(result_rect.Area(), 0.f);

    base::BBox2DF box(rect);
    base::BBox2DF result_box;
    RefineBox(box, 1920.f, 1080.f, &result_box);
    EXPECT_EQ(result_box.Area(), 0.f);
  }

  {
    base::RectF rect(100.f, 100.f, -1.f, -1.f);
    base::RectF *result_rect = nullptr;
    RefineBox(rect, 1920.f, 1080.f, result_rect);
    EXPECT_EQ(result_rect, nullptr);

    base::BBox2DF box(rect);
    base::BBox2DF *result_box = nullptr;
    RefineBox(box, 1920.f, 1080.f, result_box);
    EXPECT_EQ(result_box, nullptr);
  }
}

TEST(UtilTest, test_load_anchors) {
  std::string anchor_filepath = "non-exists";
  {
    std::vector<float> anchors;
    ASSERT_FALSE(LoadAnchors(anchor_filepath, &anchors));
  }
  anchor_filepath =
      "/apollo/modules/perception/testdata/camera/common/data/bad_anchors.txt";
  {
    std::vector<float> anchors;
    ASSERT_FALSE(LoadAnchors(anchor_filepath, &anchors));
  }
}

TEST(UtilTest, test_load_types) {
  std::string types_filepath = "non-exists";
  {
    std::vector<base::ObjectSubType> types;
    ASSERT_FALSE(LoadTypes(types_filepath, &types));
  }
  types_filepath =
      "/apollo/modules/perception/testdata/camera/common/data/bad_types.txt";
  {
    std::vector<base::ObjectSubType> types;
    ASSERT_FALSE(LoadTypes(types_filepath, &types));
  }
}

TEST(UtilTest, test_resize_cpu) {
  cv::Mat img = cv::imread(
      "/apollo/modules/perception/testdata/"
      "camera/common/img/test.jpg");

  DataProvider data_provider;
  DataProvider::InitOptions init_options;
  init_options.image_height = img.rows;
  init_options.image_width = img.cols;
  init_options.device_id = 0;
  data_provider.Init(init_options);

  EXPECT_TRUE(
      data_provider.FillImageData(img.rows, img.cols, img.data, "bgr8"));
  std::shared_ptr<base::Blob<uint8_t>> src_blob(new base::Blob<uint8_t>);

  DataProvider::ImageOptions image_options;
  image_options.target_color = base::Color::BGR;
  EXPECT_TRUE(data_provider.GetImageBlob(image_options, src_blob.get()));

  {
    std::vector<int> shape = {1, static_cast<int>(img.rows * 0.5),
                              static_cast<int>(img.cols * 0.5), img.channels()};
    std::shared_ptr<base::Blob<float>> dst_blob(new base::Blob<float>(shape));
    EXPECT_TRUE(ResizeCPU(*src_blob, dst_blob, data_provider.src_width(), 0));
    EXPECT_EQ(dst_blob->shape(0), src_blob->shape(0));
    EXPECT_EQ(dst_blob->shape(1), src_blob->shape(1) * 0.5);
    EXPECT_EQ(dst_blob->shape(2), src_blob->shape(2) * 0.5);
    EXPECT_EQ(dst_blob->shape(3), src_blob->shape(3));
  }

  {
    std::vector<int> shape = {1, static_cast<int>(img.rows * 0.5),
                              static_cast<int>(img.cols * 0.5), 0};
    std::shared_ptr<base::Blob<float>> dst_blob(new base::Blob<float>(shape));
    EXPECT_FALSE(ResizeCPU(*src_blob, dst_blob, data_provider.src_width(), 0));
  }
}

TEST(UtilTest, GetCyberWorkRootTest) {
  unsetenv("MODULE_PATH");
  unsetenv("CYBER_PATH");

  EXPECT_EQ(GetCyberWorkRoot(), "");

  char cyber_path[] = "CYBER_PATH=/home/caros/cyber";
  putenv(cyber_path);
  EXPECT_EQ(GetCyberWorkRoot(), "/home/caros/cyber");

  char module_path[] = "MODULE_PATH=/home/test/perception-camera";
  putenv(module_path);
  EXPECT_EQ(GetCyberWorkRoot(), "/home/test/perception-camera");

  unsetenv("MODULE_PATH");
  EXPECT_EQ(GetCyberWorkRoot(), "/home/caros/cyber");

  unsetenv("CYBER_PATH");
  EXPECT_EQ(GetCyberWorkRoot(), "");
}

TEST(UtilTest, FillObjectPolygonFromBBox3DTest) {
  const double eps = 1e-2;

  base::ObjectPtr object;
  FillObjectPolygonFromBBox3D(object.get());
  EXPECT_EQ(object, nullptr);

  object.reset(new base::Object);
  object->size(0) = 10;
  object->size(1) = 10;
  object->size(2) = 10;
  object->direction(0) = 0.707f;
  object->direction(1) = 0.707f;
  object->direction(2) = 0.0f;
  object->center(0) = 0;
  object->center(1) = 0;
  object->center(2) = 0;

  FillObjectPolygonFromBBox3D(object.get());
  EXPECT_EQ(object->polygon.size(), 4);
  EXPECT_NEAR(object->polygon[0].x, 0.0, eps);
  EXPECT_NEAR(object->polygon[0].y, 7.07, eps);
  EXPECT_NEAR(object->polygon[1].x, 7.07, eps);
  EXPECT_NEAR(object->polygon[1].y, 0.0, eps);
  EXPECT_NEAR(object->polygon[2].x, 0.0, eps);
  EXPECT_NEAR(object->polygon[2].y, -7.07, eps);
  EXPECT_NEAR(object->polygon[3].x, -7.07, eps);
  EXPECT_NEAR(object->polygon[3].y, 0.0, eps);
}

TEST(UtilTest, TestCalculateMeanAndVariance) {
  std::vector<double> data;
  double mean = 0.0;
  double var = 0.0;

  CalculateMeanAndVariance(data, static_cast<double *>(nullptr), &var);
  CalculateMeanAndVariance(data, &mean, static_cast<double *>(nullptr));
  CalculateMeanAndVariance(data, &mean, &var);

  data = std::vector<double>({-4.771477549277996, -5.993583986626174,
                              2.021726801725549, -0.13071376170261217,
                              -0.4202989526699099, 3.3351565902424305,
                              -2.6686659382771882, 3.391145123909162,
                              7.645098267749905, -3.0813026156194017});
  CalculateMeanAndVariance(data, &mean, &var);
  EXPECT_NEAR(mean, -0.06729160205462356, 1e-5);
  EXPECT_NEAR(var, 16.061274792441097, 1e-5);
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
