//
// Created by gaohan02 on 16-8-1.
//

#include "module/perception/traffic_light/recognizer/color_recognize/classify.h"
#include <caffe/data_transformer.hpp>
#include <traffic_light/base/utils.h>
#include "cuda_runtime_api.h"

namespace adu {
namespace perception {
namespace traffic_light {

ClassifyByDenseBoxMulti::ClassifyByDenseBoxMulti(const std::string &_class_net,
                                                 const std::string &_class_model,
                                                 float threshold, float scale,
                                                 unsigned int resize_width,
                                                 unsigned int resize_height,
                                                 float rgb_mean[3]) {
  init(_class_net, _class_model, threshold, scale, resize_width, resize_height, rgb_mean);
}

void ClassifyByDenseBoxMulti::set_crop_box(BoundBox_t &box) {
  _crop_box = box;
}
void ClassifyByDenseBoxMulti::init(const std::string &_class_net,
                                   const std::string &_class_model,
                                   float threshold, float scale, unsigned int resize_width,
                                   unsigned int resize_height,
                                   float *rgb_mean) {
  AINFO << "Creating testing net...";
  _classify_net_ptr = new caffe::Net<float>(_class_net, caffe::TEST);

  AINFO << "restore parameters...";
  _classify_net_ptr->CopyTrainedLayersFrom(_class_model);

  _resize_height = resize_height;
  _resize_width = resize_width;
  _rgb_mean[0] = rgb_mean[0];
  _rgb_mean[1] = rgb_mean[1];
  _rgb_mean[2] = rgb_mean[2];
  _data_scale = scale;
  _unknown_threshold = threshold;

  AINFO << "Init Done";
}

void ClassifyByDenseBoxMulti::perform(cv::Mat &ros_image, std::vector<BoundBox_t> &boxes) {
  caffe::Blob<float> *input_blob_recog = _classify_net_ptr->input_blobs()[0];
  caffe::Blob<float> *output_blob_recog = _classify_net_ptr->top_vecs()[
      _classify_net_ptr->top_vecs().size() - 1][0];
  cv::Mat img = ros_image(_crop_box.rect);
  for (int i = 0; i < boxes.size(); ++i) {
    if (!boxes[i].isValid || !boxes[i].selected) {
      continue;
    }
    boxes[i].rect = refined_box(boxes[i].rect, ros_image.size());
    if (boxes[i].rect.width <= 0 || boxes[i].rect.height <= 0) {
      boxes[i].isValid = false;
      continue;
    }
    cv::Mat img_light = img(boxes[i].rect).clone();
    assert(img_light.rows > 0);
    assert(img_light.cols > 0);

    cv::resize(img_light, img_light, cv::Size(_resize_width, _resize_height));
    Dtype *data = input_blob_recog->mutable_cpu_data();
    uchar *pdata = img_light.data;
    for (int h = 0; h < _resize_height; h++) {
      pdata = img_light.data + h * img_light.step;
      for (int w = 0; w < _resize_width; w++) {
        for (int channel = 0; channel < 3; channel++) {
          int index = (channel * _resize_height + h) * _resize_width + w;
          data[index] = static_cast<Dtype>((*pdata - _rgb_mean[channel]) * _data_scale);
          ++pdata;
        }
      }
    }

    _classify_net_ptr->ForwardFrom(0);
    float *out_put_data = output_blob_recog->mutable_cpu_data();
    prob_to_color(boxes[i], out_put_data, _unknown_threshold);
    AINFO << "Color Prob:";
    for (int j = 0; j < 3; j++) {
      AINFO << out_put_data[j];
    }
  }
}
ClassifyByDenseBoxMulti::~ClassifyByDenseBoxMulti() {
  delete _classify_net_ptr;
}

void ClassifyByDenseBoxMulti::prob_to_color(BoundBox_t &box,
                                            const float *out_put_data, float threshold) {
  int max_color_id = 0;
  for (int color_id = 1; color_id < 3; color_id++) {
    if (out_put_data[color_id] > threshold &&
        out_put_data[color_id] > out_put_data[max_color_id]) {
      max_color_id = color_id;
    }
  }
  TLColor map[] = {BLACK, RED, GREEN};
  std::string name_map[] = {"Black", "Red", "Green"};
  box.light_status = map[max_color_id];
  box.probability = out_put_data[max_color_id];
  AINFO << "Light status recognized as " << name_map[max_color_id];

}

}
}
}
