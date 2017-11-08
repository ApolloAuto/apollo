//
// Created by guiyilin on 17-06-19
//

#include <iostream>
#include <string>

#include <lib/base/macros.h>
#include "modules/perception/traffic_light/rectify/unity/detection.h"
#include <traffic_light/rectify/unity/unity_rectify.h>

DEFINE_string(ext,
".jpg", "ext");
DEFINE_string(hdmap_file,
"data/rect_info.txt",
"input file,contain filename and boundingbox");
DEFINE_string(img_dir,
"data/dataset/image/",
"img dir");
DEFINE_int32(detect_output_type,
0,
"detection output type; 0 for day and night, 1 for day, 2 for night");

namespace adu {
namespace perception {
namespace traffic_light {

class DebugLight {
 public:
  DebugLight(std::string filename = "result.txt") {
    fout = fopen(filename.c_str(), "w");
  }

  void show_detect_bboxes(
      const Image &image,
      const std::string ts,
      const std::vector<LightPtr> &lights) {
    cv::Mat img_bboxes = image.mat();
    for (size_t i = 0; i < lights.size(); ++i) {
      cv::rectangle(img_bboxes, lights[i]->region.projection_roi, cv::Scalar(255, 255, 0), 2);
      for (size_t j = 1; j < lights[i]->region.debug_roi.size(); ++j) {
        cv::rectangle(img_bboxes, lights[i]->region.debug_roi[j],
                      cv::Scalar(255, 128, 255), 2);
        cv::Rect rect = lights[i]->region.debug_roi[j];
        float score = lights[i]->region.debug_roi_detect_scores[j];
        cv::Point point_txt(rect.x + 2, rect.y - 5);
        cv::putText(img_bboxes, std::to_string(score), point_txt,
                    cv::FONT_HERSHEY_SIMPLEX, 0.5f, cv::Scalar(128), 1);
      }
    }
    cv::imshow("bboxes", img_bboxes);
    cv::waitKey(1);
  }

  void write(const cv::Mat &image, const std::string ts,
             const std::vector<LightPtr> &lights) {
    cv::Mat img = image.clone();
    for (size_t i = 0; i < lights.size(); ++i) {
      cv::Rect rect = lights[i]->region.rectified_roi;
      cv::rectangle(img, rect, cv::Scalar(0, 255, 255), 2);
      fprintf(fout, "%s %d %d %d %d %g\n", ts.c_str(), rect.x, rect.y, rect.width,
              rect.height,
              lights[i]->region.detect_score);

    }
    std::string fname = "result_img_bboxes/" + ts + ".jpg";
    cv::imwrite(fname, img);
  }

  ~DebugLight() {
    fclose(fout);
  }

 private:
  FILE *fout;
};

}  // namespace adu
}  // namespace perception
}  // namespace traffic_light

int main(int argc, char **argv) {
  google::ParseCommandLineFlags(&argc, &argv, true);

  std::shared_ptr<adu::perception::traffic_light::UnityRectify> _rectifier(
      new adu::perception::traffic_light::UnityRectify);

  bool init_ok = _rectifier->init();
  if (!init_ok) {
    XLOG_FATAL("Init Failed");
    return 0;
  }

  using adu::perception::traffic_light::DetectOutputBoxType;
  // Set detection output box type
  switch (FLAGS_detect_output_type) {
    default:
    case 0:_rectifier->set_output_box_type(DetectOutputBoxType::BOX_ALL);
      break;
    case 1:_rectifier->set_output_box_type(DetectOutputBoxType::BOX_VERTICAL);
      break;
    case 2:_rectifier->set_output_box_type(DetectOutputBoxType::BOX_QUADRATE);
      break;
  }

  using adu::perception::traffic_light::LightPtrs;
  using adu::perception::traffic_light::LightPtr;
  using adu::perception::traffic_light::Light;
  using adu::perception::traffic_light::Image;

  // Set test data
  // FLAGS_hdmap_file = "/home/guiyilin/Downloads/20161219T105238/rect_info.txt";
  // FLAGS_img_dir = "/home/guiyilin/Downloads/20161219T105238/camera_image/";

  std::ifstream fin_hd(FLAGS_hdmap_file.c_str());
  std::string line;
  adu::perception::traffic_light::DebugLight debug("result_all_bbox.txt");
  XLOG_INFO(FLAGS_hdmap_file.c_str());

  while (getline(fin_hd, line)) {
    std::vector<int> coords;
    std::stringstream ss;
    ss << line;

    std::string ts;
    ss >> ts;
    std::cout << "ss: " << ss.str() << std::endl;

    std::vector<LightPtr> lights;
    LightPtr light(new Light);
    light->region.projection_roi = cv::Rect(0, 0, 1920, 1080);
    lights.push_back(light);

    AINFO << "======================";
    AINFO << "Processing: " << ts << " Image";
    std::string img_fname = FLAGS_img_dir + ts + FLAGS_ext;

    cv::Mat img = cv::imread(img_fname);
    if (img.empty()) {
      AERROR << "Failed to load image: " << img_fname;
      return 0;
    }

    cv::Rect cbox;
    cbox = cv::Rect(0, 0, img.cols, img.rows);
    std::vector<LightPtr> refined_bboxes;
    _rectifier->_detect->set_crop_box(cbox);
    _rectifier->_detect->perform(img, &refined_bboxes);
    //debug.show_detect_bboxes(image, ts, lights);
    debug.write(img, ts, refined_bboxes);
  }

  return 0;
}
