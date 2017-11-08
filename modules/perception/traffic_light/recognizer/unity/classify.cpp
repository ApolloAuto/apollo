//
// Created by gaohan02 on 16-8-1.
//

#include <xlog.h>
#include <caffe/data_transformer.hpp>
#include "module/perception/traffic_light/base/utils.h"
#include "module/perception/traffic_light/recognizer/unity/classify.h"

namespace adu {
namespace perception {
namespace traffic_light {

ClassifyBySimple::ClassifyBySimple(const std::string &_class_net,
                                   const std::string &_class_model,
                                   float threshold,
                                   unsigned int resize_width,
                                   unsigned int resize_height) {
  init(_class_net, _class_model, threshold, resize_width, resize_height);
}

void ClassifyBySimple::set_crop_box(const cv::Rect &box) {
  _crop_box = box;
}
void ClassifyBySimple::init(const std::string &_class_net,
                            const std::string &_class_model,
                            float threshold, unsigned int resize_width,
                            unsigned int resize_height) {
  AINFO << "Creating testing net...";
  _classify_net_ptr = new caffe::Net<float>(_class_net, caffe::TEST);

  AINFO << "restore parameters...";
  _classify_net_ptr->CopyTrainedLayersFrom(_class_model);

  _resize_height = resize_height;
  _resize_width = resize_width;
  _unknown_threshold = threshold;

  AINFO << "Init Done";
}

void ClassifyBySimple::perform(const cv::Mat &ros_image, std::vector<LightPtr> *lights) {
  caffe::Blob<float> *input_blob_recog = _classify_net_ptr->input_blobs()[0];
  caffe::Blob<float> *output_blob_recog = _classify_net_ptr->top_vecs()[
      _classify_net_ptr->top_vecs().size() - 1][0];
  cv::Mat img = ros_image(_crop_box);
  for (LightPtr light :*lights) {
    if (!light->region.is_detected ||
        !box_is_valid(light->region.rectified_roi, ros_image.size())) {
      continue;
    }

    cv::Mat img_light = img(light->region.rectified_roi).clone();
    assert(img_light.rows > 0);
    assert(img_light.cols > 0);

    cv::resize(img_light, img_light, cv::Size(_resize_width, _resize_height));
    float *data = input_blob_recog->mutable_cpu_data();
    uchar *pdata = img_light.data;
    for (int h = 0; h < _resize_height; h++) {
      pdata = img_light.data + h * img_light.step;
      for (int w = 0; w < _resize_width; w++) {
        for (int channel = 0; channel < 3; channel++) {
          int index = (channel * _resize_height + h) * _resize_width + w;
          data[index] = static_cast<float>((*pdata));
          ++pdata;
        }
      }
    }

    _classify_net_ptr->ForwardFrom(0);
    float *out_put_data = output_blob_recog->mutable_cpu_data();
    prob_to_color(out_put_data, _unknown_threshold, light);

  }
}
ClassifyBySimple::~ClassifyBySimple() {
  delete _classify_net_ptr;
}

void ClassifyBySimple::prob_to_color(const float *out_put_data, float threshold, LightPtr light) {
  int max_color_id = 0;
  std::vector<TLColor> status_map = {BLACK, RED, YELLOW, GREEN};
  std::vector<std::string> name_map = {"Black", "Red", "Yellow", "Green"};
  std::vector<float> prob(out_put_data, out_put_data + status_map.size());
  auto max_prob = std::max_element(prob.begin(), prob.end());
  max_color_id = (*max_prob > threshold) ? static_cast<int>(std::distance(prob.begin(), max_prob))
                                         : 0;

  light->status.color = status_map[max_color_id];
  light->status.confidence = out_put_data[max_color_id];
  AINFO << "Light status recognized as " << name_map[max_color_id];
  AINFO << "Color Prob:";
  for (int j = 0; j < status_map.size(); j++) {
    AINFO << out_put_data[j];
  }
}

}
}
}
