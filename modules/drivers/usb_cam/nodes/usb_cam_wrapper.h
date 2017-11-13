#ifndef USB_CAM_INCLUDE_USB_CAM_USBCAMWRAPPER_H_
#define USB_CAM_INCLUDE_USB_CAM_USBCAMWRAPPER_H_

#include <ros/ros.h>
#include <usb_cam/usb_cam.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <pluginlib/class_loader.h>
#include <sstream>
#include <std_srvs/Empty.h>

namespace usb_cam {

enum TriggerFrequence {
    FPS_10HZ = 10,
    FPS_15HZ = 15,
    FPS_20HZ = 20,
    DEFAULT_FPS = 30
};

class UsbCamWrapper {
  public:
    UsbCamWrapper(ros::NodeHandle node, ros::NodeHandle private_nh);
    virtual ~UsbCamWrapper();
    bool service_start_cap(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    bool service_stop_cap(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    bool take_and_send_image();
    bool spin();

  private:
    // shared image message
    sensor_msgs::Image _img;
    sensor_msgs::CameraInfoPtr _cam_info = nullptr;
    //image_transport::CameraPublisher image_pub_;

    image_transport::PubLoaderPtr _pub_loader;
    boost::shared_ptr<image_transport::PublisherPlugin> _image_pub_plugin;

    ros::Publisher _cam_info_pub;

    // parameters
    std::string _topic_name;
    std::string _video_device_name; 
    std::string _io_method_name; 
    std::string _pixel_format_name;
    std::string _camera_name;
    std::string _camera_info_url;

    //std::string start_service_name_, start_service_name_;
    //bool streaming_status_;
    int _image_width;
    int _image_height;
    int _framerate;
    int _exposure;
    int _brightness;
    int _contrast; 
    int _saturation;
    int _sharpness;
    int _focus;
    int _white_balance;
    int _gain;

    bool _autofocus;
    bool _autoexposure;
    bool _auto_white_balance;

    // usb will be reset when camera timeout
    int _cam_timeout;
    UsbCam _cam;
    boost::shared_ptr<camera_info_manager::CameraInfoManager> _cinfo;

    ros::ServiceServer _service_start; 
    ros::ServiceServer _service_stop; 

    // private ROS node handle
    ros::NodeHandle _node;
    ros::NodeHandle _priv_node;

    ros::Time _last_stamp;
    float _frame_warning_interval;
    float _frame_drop_interval;
    float _spin_interval;
    int _error_code;
};
}

#endif /* USB_CAM_INCLUDE_USB_CAM_USBCAMWRAPPER_H_ */
