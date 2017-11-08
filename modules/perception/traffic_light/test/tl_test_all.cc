//
// Created by gaohan02 on 17-3-13.
//
#include <lib/base/macros.h>
#include <traffic_light/rectify/unity/unity_rectify.h>
#include "module/perception/traffic_light/recognizer/unity/unity_recognize.h"
#include "module/perception/traffic_light/reviser/strategy/color_decision.h"
DEFINE_string(ext,
".jpg", "ext");
DEFINE_string(hdmap_file,
"data/rect_info.txt",
"input file,contain filename and boundingbox");
DEFINE_string(img_dir,
"data/dataset/image/",
"img dir");

namespace adu {
namespace perception {
namespace traffic_light {

class DebugLight {
 public:
  DebugLight(std::string filename = "result.txt") {
    fout = fopen(filename.c_str(), "w");
  }
  void write(const Image &image, const std::string ts, const std::vector<LightPtr> &lights) {
    fprintf(fout, "%s ", ts.c_str());
    for (int i = 0; i < lights.size(); ++i) {
      cv::Rect rect = lights[i]->region.rectified_roi;
      fprintf(fout, "%d %d %d %d %d %d ", rect.x, rect.y, rect.width, rect.height,
              rect.width * rect.height, lights[i]->status.color);
      cv::Mat light = image.mat()(rect);
      char name[100];
      switch (lights[i]->status.color) {
        case RED:sprintf(name, "red/%s_%d.jpg", ts.c_str(), i);
          break;
        case GREEN:sprintf(name, "green/%s_%d.jpg", ts.c_str(), i);
          break;
        case BLACK:sprintf(name, "black/%s_%d.jpg", ts.c_str(), i);
          break;
        case YELLOW:sprintf(name, "yellow/%s_%d.jpg", ts.c_str(), i);
          break;
        default:sprintf(name, "unknown/%s_%d.jpg", ts.c_str(), i);
          break;
      }
      cv::imwrite(name, light);
    }
    fprintf(fout, "\n");
  }
  ~DebugLight() {
    fclose(fout);
  }
 private:
  FILE *fout;
};
}
}
}

int main(int argc, char *argv[]) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  std::shared_ptr<adu::perception::traffic_light::BaseRectifier> _rectifier(
      new adu::perception::traffic_light::UnityRectify);
  std::shared_ptr<adu::perception::traffic_light::BaseRecognizer> _recognizer(
      new adu::perception::traffic_light::UnityRecognize);
  std::shared_ptr<adu::perception::traffic_light::BaseReviser> _reviser(
      new adu::perception::traffic_light::ColorReviser);

  bool init_ok = _rectifier->init() && _recognizer->init() && _reviser->init();
  if (!init_ok) {
    XLOG_FATAL("Init Failed");
    return 0;
  }

  using adu::perception::traffic_light::LightPtrs;
  using adu::perception::traffic_light::LightPtr;
  using adu::perception::traffic_light::Light;
  using adu::perception::traffic_light::Image;

  std::ifstream fin_hd(FLAGS_hdmap_file.c_str());
  std::string line;
  adu::perception::traffic_light::DebugLight debug;
  XLOG_INFO(FLAGS_hdmap_file.c_str());
  std::string last_ts = "";
  int count = 0;
  while (getline(fin_hd, line)) {
    std::vector<int> coords;
    std::stringstream ss;
    ss << line;
    std::string ts;
    int cord = 0;
    ss >> ts;
    while (ss >> cord) {
      coords.push_back(cord);
    }
    std::vector<LightPtr> lights;
    for (int h = 0; h < coords.size(); h += 5) {
      LightPtr light(new Light);
      light->region.projection_roi = cv::Rect(coords[h + 0],
                                              coords[h + 1],
                                              coords[h + 2],
                                              coords[h + 3]);
      lights.push_back(light);
    }
    AINFO << "======================";
    AINFO << "Processing: " << ts << " Image";
    cv::Mat img = cv::imread(FLAGS_img_dir + ts + FLAGS_ext);
    Image image;
    image.init(1, adu::perception::traffic_light::LONG_FOCUS, img);
    _rectifier->rectify(image, adu::perception::traffic_light::RectifyOption(), &lights);
    _recognizer->recognize_status(image, adu::perception::traffic_light::RecognizeOption(),
                                  &lights);
    //_reviser->revise(adu::perception::traffic_light::ReviseOption(image.ts()), &lights);
    if (ts == last_ts) {
      ++count;
    } else {
      last_ts = ts;
      count = 0;
    }
    debug.write(image, ts + "_" + std::to_string(count), lights);
  }

  return 0;
}
