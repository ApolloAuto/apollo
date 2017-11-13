#include "usb_cam_wrapper.h"
#include <time.h>   /* for clock_gettime */

#include "image_transport/camera_common.h"
#include "image_transport/publisher_plugin.h"

namespace usb_cam {

UsbCamWrapper::UsbCamWrapper(ros::NodeHandle node, ros::NodeHandle private_nh) :
    _node(node), _priv_node(private_nh), _last_stamp(0) {
    // grab the parameters
    _priv_node.param("topic_name", _topic_name, std::string("image_raw0"));
    _priv_node.param("video_device", _video_device_name, std::string("/dev/video0"));
    _priv_node.param("brightness", _brightness, -1); //0-255, -1 "leave alone"
    _priv_node.param("contrast", _contrast, -1); //0-255, -1 "leave alone"
    _priv_node.param("saturation", _saturation, -1); //0-255, -1 "leave alone"
    _priv_node.param("sharpness", _sharpness, -1); //0-255, -1 "leave alone"
    // possible values: mmap, read, userptr
    _priv_node.param("io_method", _io_method_name, std::string("mmap"));
    _priv_node.param("image_width", _image_width, 640);
    _priv_node.param("image_height", _image_height, 480);
    _priv_node.param("frame_rate", _framerate, 30);
    // possible values: yuyv, uyvy, mjpeg, yuvmono10, rgb24
    _priv_node.param("pixel_format", _pixel_format_name, std::string("mjpeg"));
    // enable/disable autofocus
    _priv_node.param("autofocus", _autofocus, false);
    _priv_node.param("focus", _focus, -1); //0-255, -1 "leave alone"
    // enable/disable autoexposure
    _priv_node.param("autoexposure", _autoexposure, true);
    _priv_node.param("exposure", _exposure, 100);
    _priv_node.param("gain", _gain, -1); //0-100?, -1 "leave alone"
    // enable/disable auto white balance temperature
    _priv_node.param("auto_white_balance", _auto_white_balance, true);
    _priv_node.param("white_balance", _white_balance, 4000);

    // load the camera info
    _priv_node.param("camera_frame_id", _img.header.frame_id, std::string("head_camera"));
    _priv_node.param("camera_name", _camera_name, std::string("head_camera"));
    _priv_node.param("camera_info_url", _camera_info_url, std::string(""));
    _cinfo.reset(new camera_info_manager::CameraInfoManager(_node, _camera_name, _camera_info_url));

    // default 3000 ms
    _priv_node.param("camera_timeout", _cam_timeout, 1000);
    _priv_node.param("spin_interval", _spin_interval, 0.005f);
    _priv_node.param("error_code", _error_code, 11);

    // Warning when diff with last > 1.5* interval
    _frame_warning_interval = 1.5 / _framerate;
    // now max fps 30, we use a appox time 0.9 to drop image.
    _frame_drop_interval = 0.9 / _framerate;

    // advertise the main image topic
    //image_transport::ImageTransport it(_node);
    //image_pub_ = it.advertiseCamera(topic_name_, 1);

    // Load transport publish plugin
    std::string image_topic = _node.resolveName(_topic_name);
    _pub_loader = boost::make_shared<image_transport::PubLoader>("image_transport", "image_transport::PublisherPlugin");
    std::string lookup_name = image_transport::PublisherPlugin::getLookupName(std::string("raw"));
    _image_pub_plugin = _pub_loader->createInstance(lookup_name);
    _image_pub_plugin->advertise(_node, image_topic, 1, image_transport::SubscriberStatusCallback(),
                   image_transport::SubscriberStatusCallback(), ros::VoidPtr(), false);

    // camera info publish
    std::string cam_info_topic = image_transport::getCameraInfoTopic(image_topic);
    _cam_info_pub = _node.advertise<sensor_msgs::CameraInfo>(cam_info_topic, 1, ros::SubscriberStatusCallback(),
                    ros::SubscriberStatusCallback(), ros::VoidPtr(), false);

    // create Services
    _service_start = _node.advertiseService("start_capture", &UsbCamWrapper::service_start_cap, this);
    _service_stop = _node.advertiseService("stop_capture", &UsbCamWrapper::service_stop_cap, this);

    // check for default camera info
    if (!_cinfo->isCalibrated()) {
        _cinfo->setCameraName(_video_device_name);
        sensor_msgs::CameraInfo camera_info;
        camera_info.header.frame_id = _img.header.frame_id;
        camera_info.width = _image_width;
        camera_info.height = _image_height;
        _cinfo->setCameraInfo(camera_info);
    }
    //get the camera basical infomation
    _cam_info.reset(new sensor_msgs::CameraInfo(_cinfo->getCameraInfo()));

    ROS_INFO("Starting '%s' (%s) at %dx%d via %s (%s) at %i FPS", _camera_name.c_str(),
             _video_device_name.c_str(),
             _image_width, _image_height, _io_method_name.c_str(), _pixel_format_name.c_str(), _framerate);

    // set the IO method
    UsbCam::io_method io_method = UsbCam::io_method_from_string(_io_method_name);

    if (io_method == UsbCam::IO_METHOD_UNKNOWN) {
        ROS_FATAL("Unknown IO method '%s'", _io_method_name.c_str());
        _node.shutdown();
        return;
    }

    // set the pixel format
    UsbCam::pixel_format pixel_format = UsbCam::pixel_format_from_string(_pixel_format_name);

    if (pixel_format == UsbCam::PIXEL_FORMAT_UNKNOWN) {
        ROS_FATAL("Unknown pixel format '%s'", _pixel_format_name.c_str());
        _node.shutdown();
        return;
    }

    _cam.set_error_code(_error_code);
    // start the camera
    _cam.start(_video_device_name.c_str(), io_method, pixel_format, _image_width, _image_height,
               _framerate);

    // set camera parameters
    if (_brightness >= 0) {
        _cam.set_v4l_parameter("brightness", _brightness);
    }

    if (_contrast >= 0) {
        _cam.set_v4l_parameter("contrast", _contrast);
    }

    if (_saturation >= 0) {
        _cam.set_v4l_parameter("saturation", _saturation);
    }

    if (_sharpness >= 0) {
        _cam.set_v4l_parameter("sharpness", _sharpness);
    }

    if (_gain >= 0) {
        _cam.set_v4l_parameter("gain", _gain);
    }

    // check auto white balance
    if (_auto_white_balance) {
        _cam.set_v4l_parameter("white_balance_temperature_auto", 1);
    } else {
        _cam.set_v4l_parameter("white_balance_temperature_auto", 0);
        _cam.set_v4l_parameter("white_balance_temperature", _white_balance);
    }

    // check auto exposure
    if (!_autoexposure) {
        // turn down exposure control (from max of 3)
        _cam.set_v4l_parameter("exposure_auto", 1);
        // change the exposure level
        _cam.set_v4l_parameter("exposure_absolute", _exposure);
    }

    // check auto focus
    if (_autofocus) {
        _cam.set_auto_focus(1);
        _cam.set_v4l_parameter("focus_auto", 1);
    } else {
        _cam.set_v4l_parameter("focus_auto", 0);

        if (_focus >= 0) {
            _cam.set_v4l_parameter("focus_absolute", _focus);
        }
    }
}

UsbCamWrapper::~UsbCamWrapper() {
    _cam.shutdown();
}

bool UsbCamWrapper::service_start_cap(std_srvs::Empty::Request& req,
                                      std_srvs::Empty::Response& res) {
    _cam.start_capturing();
    return true;
}

bool UsbCamWrapper::service_stop_cap(std_srvs::Empty::Request& req,
                                     std_srvs::Empty::Response& res) {
    _cam.stop_capturing();
    return true;
}

bool UsbCamWrapper::take_and_send_image() {
    // grab the image
    bool get_new_image = _cam.grab_image(&_img, _cam_timeout);

    if (!get_new_image) {
        return false;
    }

    // grab the camera info
    //_cam_info = sensor_msgs::CameraInfo(cinfo_->getCameraInfo());
    _cam_info->header.frame_id = _img.header.frame_id;
    _cam_info->header.stamp = _img.header.stamp;

    if (_last_stamp == ros::Time(0)) {
        _last_stamp = _img.header.stamp;
    } else {
        auto diff = (_img.header.stamp - _last_stamp).toSec();
        // drop image by frame_rate
        if (diff < _frame_drop_interval) {
            ROS_INFO_STREAM("drop image:" << _img.header.stamp);
            return true;
        }
        if (_frame_warning_interval < diff) {
            ROS_WARN_STREAM("stamp jump.last stamp:" << _last_stamp
                    << " current stamp:" << _img.header.stamp);
        }
        _last_stamp = _img.header.stamp;
    }

    // publish the image
    _image_pub_plugin->publish(_img);
    _cam_info_pub.publish(_cam_info);

    return true;
}

bool UsbCamWrapper::spin() {
    // spin loop rate should be in accord with the trigger frequence
    ros::Duration loop_interval(this->_spin_interval);

    while (_node.ok()) {
        if (_cam.is_capturing()) {
            if (!take_and_send_image()) {
                ROS_ERROR("USB camera did not respond in time.");
            }
        }
        //ros::spinOnce();
        loop_interval.sleep();
    }
    return true;
}
}
