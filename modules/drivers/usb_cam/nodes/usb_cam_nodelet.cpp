
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include "usb_cam_wrapper.h"

namespace usb_cam {

class UsbCamNodelet: public nodelet::Nodelet {

public:
    UsbCamNodelet() { };
    ~UsbCamNodelet() {
        ROS_INFO("shutting down driver thread");
        if (_device_thread != nullptr && _device_thread->joinable()) {
          _device_thread->join();
        }
        ROS_INFO("driver thread stopped");
};

private:
    virtual void onInit();
    boost::shared_ptr<UsbCamWrapper> _usb_cam = nullptr;
    boost::shared_ptr<boost::thread> _device_thread = nullptr;
};

void UsbCamNodelet::onInit() {
    ROS_INFO("Usb cam nodelet init");
    _usb_cam.reset(new UsbCamWrapper(getNodeHandle(), getPrivateNodeHandle()));
    // spawn device poll thread
    _device_thread = boost::shared_ptr<boost::thread>
        (new boost::thread(boost::bind(&UsbCamWrapper::spin, _usb_cam)));
}

}

// Register this plugin with pluginlib.  Names must match nodelets.xml.
//
// parameters: package, class name, class type, base class type
PLUGINLIB_DECLARE_CLASS(usb_cam, UsbCamNodelet,
                        usb_cam::UsbCamNodelet, nodelet::Nodelet);
