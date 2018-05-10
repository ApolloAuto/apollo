#include "usb_cam_wrapper.h"
#include <time.h>   /* for clock_gettime */

#include "image_transport/camera_common.h"
#include "image_transport/publisher_plugin.h"

namespace usb_cam {

UsbCamWrapper::UsbCamWrapper(ros::NodeHandle node, ros::NodeHandle private_nh) :
    node_(node), priv_node_(private_nh), last_stamp_(0) 
{
  // grab the parameters
  priv_node_.param("topic_name", topic_name_, std::string("image_raw0"));
  priv_node_.param("video_device", video_device_name_, std::string("/dev/video0"));
  priv_node_.param("brightness", brightness_, -1); //0-255, -1 "leave alone"
  priv_node_.param("contrast", contrast_, -1); //0-255, -1 "leave alone"
  priv_node_.param("saturation", saturation_, -1); //0-255, -1 "leave alone"
  priv_node_.param("sharpness", sharpness_, -1); //0-255, -1 "leave alone"
  // possible values: mmap, read, userptr
  priv_node_.param("io_method", io_method_name_, std::string("mmap"));
  priv_node_.param("image_width", image_width_, 640);
  priv_node_.param("image_height", image_height_, 480);
  priv_node_.param("frame_rate", framerate_, 30);
  // possible values: yuyv, uyvy, mjpeg, yuvmono10, rgb24
  priv_node_.param("pixel_format", pixel_format_name_, std::string("mjpeg"));
  // enable/disable autofocus
  priv_node_.param("autofocus", autofocus_, false);
  priv_node_.param("focus", focus_, -1); //0-255, -1 "leave alone"
  // enable/disable autoexposure
  priv_node_.param("autoexposure", autoexposure_, true);
  priv_node_.param("exposure", exposure_, 100);
  priv_node_.param("gain", gain_, -1); //0-100?, -1 "leave alone"
  // enable/disable auto white balance temperature
  priv_node_.param("auto_white_balance", auto_white_balance_, true);
  priv_node_.param("white_balance", white_balance_, 4000);

  // load the camera info
  priv_node_.param("camera_frame_id", img_.header.frame_id, std::string("head_camera"));
  priv_node_.param("camera_name", camera_name_, std::string("head_camera"));
  priv_node_.param("camera_info_url", camera_info_url_, std::string(""));
  cinfo_.reset(new camera_info_manager::CameraInfoManager(node_, camera_name_, camera_info_url_));

  // default 3000 ms
  priv_node_.param("camera_timeout", cam_timeout_, 1000);
  priv_node_.param("spin_interval", spin_interval_, 0.005f);

  // Warning when diff with last > 1.5* interval
  frame_warning_interval_ = 1.5 / framerate_;
  // now max fps 30, we use a appox time 0.9 to drop image.
  frame_drop_interval_ = 0.9 / framerate_;

  // advertise the main image topic
  //image_transport::ImageTransport it(node_);
  //image_pub_ = it.advertiseCamera(topic_name_, 1);
  // Load transport publish plugin
  std::string image_topic = node_.resolveName(topic_name_);
  pub_loader_ = boost::make_shared<image_transport::PubLoader>("image_transport", "image_transport::PublisherPlugin");
  std::string lookup_name = image_transport::PublisherPlugin::getLookupName(std::string("raw"));
  image_pub_plugin_ = pub_loader_->createInstance(lookup_name);
  if (image_pub_plugin_ != nullptr) 
  {
    image_pub_plugin_->advertise(node_, image_topic, 1, image_transport::SubscriberStatusCallback(),
           image_transport::SubscriberStatusCallback(), ros::VoidPtr(), false);
  }
  else
  {
    ROS_ERROR("create image publish plugin error. lookup_name: '%s'", lookup_name.c_str());
    node_.shutdown();
    return;
  }

  // camera info publish
  std::string cam_info_topic = image_transport::getCameraInfoTopic(image_topic);
  cam_info_pub_ = node_.advertise<sensor_msgs::CameraInfo>(cam_info_topic, 1, ros::SubscriberStatusCallback(),
          ros::SubscriberStatusCallback(), ros::VoidPtr(), false);

  // create Services
  service_start_ = node_.advertiseService("start_capture", &UsbCamWrapper::service_start_cap, this);
  service_stop_ = node_.advertiseService("stop_capture", &UsbCamWrapper::service_stop_cap, this);

  // check for default camera info
  if (!cinfo_->isCalibrated())
  {
    cinfo_->setCameraName(video_device_name_);
    sensor_msgs::CameraInfo camera_info;
    camera_info.header.frame_id = img_.header.frame_id;
    camera_info.width = image_width_;
    camera_info.height = image_height_;
    cinfo_->setCameraInfo(camera_info);
  }
  //get the camera basical infomation
  cam_info_.reset(new sensor_msgs::CameraInfo(cinfo_->getCameraInfo()));

  ROS_INFO("Starting '%s' (%s) at %dx%d via %s (%s) at %i FPS", camera_name_.c_str(),
           video_device_name_.c_str(),
           image_width_, image_height_, io_method_name_.c_str(), pixel_format_name_.c_str(), framerate_);

  // set the IO method
  UsbCam::io_method io_method = UsbCam::io_method_from_string(io_method_name_);

  if (io_method == UsbCam::IO_METHOD_UNKNOWN)
  {
    ROS_FATAL("Unknown IO method '%s'", io_method_name_.c_str());
    node_.shutdown();
    return;
  }

  // set the pixel format
  UsbCam::pixel_format pixel_format = UsbCam::pixel_format_from_string(pixel_format_name_);

  if (pixel_format == UsbCam::PIXEL_FORMAT_UNKNOWN)
  {
    ROS_FATAL("Unknown pixel format '%s'", pixel_format_name_.c_str());
    node_.shutdown();
    return;
  }

  // start the camera
  cam_.start(video_device_name_.c_str(), io_method, pixel_format, image_width_, image_height_,
         framerate_);

  // set camera parameters
  if (brightness_ >= 0)
    cam_.set_v4l_parameter("brightness", brightness_);

  if (contrast_ >= 0)
    cam_.set_v4l_parameter("contrast", contrast_);

  if (saturation_ >= 0)
    cam_.set_v4l_parameter("saturation", saturation_);

  if (sharpness_ >= 0)
    cam_.set_v4l_parameter("sharpness", sharpness_);

  if (gain_ >= 0)
    cam_.set_v4l_parameter("gain", gain_);

  // check auto white balance
  if (auto_white_balance_)
  {
    cam_.set_v4l_parameter("white_balance_temperature_auto", 1);
  }
  else
  {
    cam_.set_v4l_parameter("white_balance_temperature_auto", 0);
    cam_.set_v4l_parameter("white_balance_temperature", white_balance_);
  }

  // check auto exposure
  if (!autoexposure_)
  {
    // turn down exposure control (from max of 3)
    cam_.set_v4l_parameter("exposure_auto", 1);
    // change the exposure level
    cam_.set_v4l_parameter("exposure_absolute", exposure_);
  }

  // check auto focus
  if (autofocus_) 
  {
    cam_.set_auto_focus(1);
    cam_.set_v4l_parameter("focus_auto", 1);
  }
  else
  {
    cam_.set_v4l_parameter("focus_auto", 0);

    if (focus_ >= 0)
      cam_.set_v4l_parameter("focus_absolute", focus_);
  }
}

UsbCamWrapper::~UsbCamWrapper()
{
  cam_.shutdown();
}

bool UsbCamWrapper::service_start_cap(std_srvs::Empty::Request& req,
                                      std_srvs::Empty::Response& res)
{
  cam_.start_capturing();
  return true;
}

bool UsbCamWrapper::service_stop_cap(std_srvs::Empty::Request& req,
                                     std_srvs::Empty::Response& res)
{
  cam_.stop_capturing();
  return true;
}

bool UsbCamWrapper::take_and_send_image()
{
  // grab the image
  bool get_new_image = cam_.grab_image(&img_, cam_timeout_);

  if (!get_new_image)
    return false;

  // grab the camera info
  //cam_info_ = sensor_msgs::CameraInfo(cinfo_->getCameraInfo());
  cam_info_->header.frame_id = img_.header.frame_id;
  cam_info_->header.stamp = img_.header.stamp;

  if (last_stamp_ == ros::Time(0)) 
  {
    last_stamp_ = img_.header.stamp;
  } 
  else
  {
    auto diff = (img_.header.stamp - last_stamp_).toSec();
    // drop image by frame_rate
    if (diff < frame_drop_interval_) {
      ROS_INFO_STREAM("drop image:" << img_.header.stamp);
      return true;
    }
    if (frame_warning_interval_ < diff)
      ROS_WARN_STREAM("stamp jump.last stamp:" << last_stamp_
          << " current stamp:" << img_.header.stamp);
    last_stamp_ = img_.header.stamp;
  }

  // publish the image
  image_pub_plugin_->publish(img_);
  cam_info_pub_.publish(cam_info_);

  return true;
}

bool UsbCamWrapper::spin()
{
  // spin loop rate should be in accord with the trigger frequence
  ros::Duration loop_interval(this->spin_interval_);

  while (node_.ok()) 
  {
    if (cam_.is_capturing())
      if (!take_and_send_image())
        ROS_ERROR("USB camera did not respond in time.");
    //ros::spinOnce();
    loop_interval.sleep();
  }
  return true;
}
}
