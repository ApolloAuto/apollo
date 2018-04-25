/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include <ros/ros.h>
#include <arpa/inet.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include "pandora/pandora.h"

using apollo::drivers::hesai::Pandora;

class PandoraHesaiClient {
 public:
  PandoraHesaiClient(ros::NodeHandle node, ros::NodeHandle nh) {
    // default configure
    std::string pandoraIP = std::string("192.168.20.51");
    int lidarRecvPort = 2368;
    int gpsRecvPort = 10110;
    int startAngle = 135;
    int pandoraCameraPort = 9870;
    std::string lidarTopic = "/apollo/sensor/pandora/hesai40/PointCloud2";
    std::string cameraTopics[5];
    cameraTopics[0] = std::string("/apollo/sensor/pandora/camera/front_color");
    cameraTopics[1] = std::string("/apollo/sensor/pandora/camera/front_gray");
    cameraTopics[2] = std::string("/apollo/sensor/pandora/camera/right_gray");
    cameraTopics[3] = std::string("/apollo/sensor/pandora/camera/back_gray");
    cameraTopics[4] = std::string("/apollo/sensor/pandora/camera/left_gray");
    bool enableCamera = true;
    int timezone = 8;
    std::string frameId = std::string("hesai40");

    // parse nodehandle param
    bool ret = parseParameter(nh, &pandoraIP, &lidarRecvPort,
            &gpsRecvPort, &startAngle, &pandoraCameraPort,
            &lidarTopic, cameraTopics, &enableCamera, &timezone, &frameId);
    if (!ret) {
        ROS_INFO("Parse parameters failed, please check parameters above.");
        return;
    }

    // advertise
    lidarPublisher = node.advertise<sensor_msgs::PointCloud2>(lidarTopic, 10);

    if (enableCamera) {
      image_transport::ImageTransport it(nh);
      for (int i = 0; i < 5; i++) {
        imgPublishers[i] = it.advertise(cameraTopics[i], 1);
      }
    }

    psdk = new Pandora(pandoraIP, lidarRecvPort, gpsRecvPort,
            boost::bind(&PandoraHesaiClient::lidarCallback, this, _1, _2),
            NULL, startAngle * 100, pandoraCameraPort,
            boost::bind(&PandoraHesaiClient::cameraCallback,
                        this, _1, _2, _3, _4),
            enableCamera, timezone, frameId);
    psdk->Start();
  }

  bool parseParameter(ros::NodeHandle nh, std::string* pandoraIP,
                      int* lidarRecvPort, int* gpsRecvPort, int* startAngle,
                      int* pandoraCameraPort, std::string* lidarTopic,
                      std::string cameraTopics[], bool* enableCamera,
                      int* timezone, std::string* frameId) {
    if (nh.hasParam("pandora_ip")) {
      nh.getParam("pandora_ip", *pandoraIP);
    }
    if (nh.hasParam("lidar_recv_port")) {
      nh.getParam("lidar_recv_port", *lidarRecvPort);
    }
    if (nh.hasParam("gps_recv_port")) {
      nh.getParam("gps_recv_port", *gpsRecvPort);
    }
    if (nh.hasParam("start_angle")) {
      nh.getParam("start_angle", *startAngle);
    }
    if (nh.hasParam("pandora_camera_port")) {
      nh.getParam("pandora_camera_port", *pandoraCameraPort);
    }
    if (nh.hasParam("lidar_topic")) {
      nh.getParam("lidar_topic", *lidarTopic);
    }
    if (nh.hasParam("camera0_topic")) {
      nh.getParam("camera0_topic", cameraTopics[0]);
    }
    if (nh.hasParam("camera1_topic")) {
      nh.getParam("camera1_topic", cameraTopics[1]);
    }
    if (nh.hasParam("camera2_topic")) {
      nh.getParam("camera2_topic", cameraTopics[2]);
    }
    if (nh.hasParam("camera3_topic")) {
      nh.getParam("camera3_topic", cameraTopics[3]);
    }
    if (nh.hasParam("camera4_topic")) {
      nh.getParam("camera4_topic", cameraTopics[4]);
    }
    if (nh.hasParam("enable_camera")) {
      nh.getParam("enable_camera", *enableCamera);
    }
    if (nh.hasParam("timezone")) {
      nh.getParam("timezone", *timezone);
    }
    if (nh.hasParam("frame_id")) {
      nh.getParam("frame_id", *frameId);
    }

    std::cout << "Configs: pandoraIP: " << *pandoraIP << ", lidarRecvPort: "
        << *lidarRecvPort << ", gpsRecvPort: " << *gpsRecvPort
        << ", startAngle: " << *startAngle << ", pandoraCameraPort: "
        << *pandoraCameraPort << ", lidarTopic: " << *lidarTopic
        << ", enableCamera: " << *enableCamera << ", frameId: "
        << *frameId << std::endl;
    for (int i = 0; i < 5; i++) {
      std::cout << "cameraTopic" << i << ": " << cameraTopics[i] << std::endl;
    }

    // check
    struct sockaddr_in sa;
    return checkPort(*lidarRecvPort) && checkPort(*gpsRecvPort)
        && checkPort(*pandoraCameraPort) && (*startAngle >= 0)
        && (*startAngle < 360)
        && (1 == inet_pton(AF_INET, pandoraIP->c_str(), &(sa.sin_addr)));
  }

  bool checkPort(int port) {
    return (port > 0) && (port < 65535);
  }

  void cameraCallback(boost::shared_ptr<cv::Mat> matp, double timestamp,
                      int pic_id, bool distortion) {
    sensor_msgs::ImagePtr imgMsg;

    if (pic_id > 4 || pic_id < 0) {
      ROS_INFO("picid wrong in getImageToPub");
      return;
    }
    imgMsg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", *matp).toImageMsg();
    imgMsg->header.stamp = ros::Time(timestamp);
    imgPublishers[pic_id].publish(imgMsg);
  }

  void lidarCallback(boost::shared_ptr<PPointCloud> cld, double timestamp) {
    pcl_conversions::toPCL(ros::Time(timestamp), cld->header.stamp);
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cld, output);
    lidarPublisher.publish(output);
  }

  ~PandoraHesaiClient() {
    if (NULL != psdk) {
      delete(psdk);
    }
  }

 private:
  ros::Publisher lidarPublisher;
  image_transport::Publisher imgPublishers[5];
  Pandora *psdk;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "pandora_ros");
  ros::NodeHandle nh("~");
  ros::NodeHandle node;
  PandoraHesaiClient pandoraClient(node, nh);

  ros::spin();
  return 0;
}
