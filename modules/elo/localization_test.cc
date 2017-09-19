#include <string>
#include <fstream>
#include "opencv2/opencv.hpp"
#include "camera_localization.h"

int main(int argc, char* argv[]) {
  if (argc != 4) {
    std::cout << "Input Error!" << std::endl;
    std::cout << "Usage: ./localization_test ../config/apollo_release.config ../data/testdata1/test_list.txt ../data/testdata1/image/" << std::endl;
    std::cout << "   or: ./localization_test ../config/apollo_release.config ../data/testdata2/test_list.txt ../data/testdata2/image/" << std::endl;
    return -1;
  }

  std::string config_file = argv[1];
  std::string test_file_name = argv[2];
  std::string img_dir = argv[3];
  apollo::localization::CameraLocalization loc;
  if(!loc.init(config_file)) {
    return -1;
  }
  std::ifstream test_file(test_file_name.c_str());
  std::string imname;
  double l_x = 0;
  double l_y = 0;
  double h_x = 0;
  double h_y = 0;
  while (test_file >> imname >> l_x >> l_y >> h_x >> h_y) {
    std::cout << imname << std::endl;
    std::string impath = img_dir + imname;
    cv::Mat image = cv::imread(impath);
    if (image.empty()) {
      std::cout << "[ERROR] Can not open image " << impath << std::endl;
      continue;
    }
    apollo::localization::PosInfo input_pos;
    apollo::localization::PosInfo output_pos;
    input_pos.longitude = l_x;
    input_pos.latitude = l_y;
    if (!loc.get_ego_position(image, input_pos, output_pos)) {
      continue;
    }
    std::cout << std::setiosflags(std::ios::fixed);
    std::cout << "[INFO] GROUNDTRUTH (" << std::setprecision(8)
        << h_x << ", " << std::setprecision(8) << h_y << ")\t"
        << " LOCALIZATION (" << output_pos.longitude << ", "
        << output_pos.latitude << ")" << std::endl;
    std::cout << std::endl;
  }
  return 0;
}
