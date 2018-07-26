 #!/usr/bin/env python  
import rospy

# Because of transformations
import tf

import tf2_ros
import geometry_msgs.msg
import time

import velodyne_msgs.msg
import sensor_msgs.msg


def handle_turtle_pose(ctime):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    if ctime:
        t.header.stamp = rospy.Time(ctime)
    else:
        t.header.stamp = rospy.Time.now()
    t.header.frame_id = "world"
    t.child_frame_id = "localization"
    t.transform.translation.x = 439917.456544
    t.transform.translation.y = 4433509.77738
    t.transform.translation.z = 36.4098397498

    t.transform.rotation.x = -0.00128203249671
    t.transform.rotation.y = -0.0211466194596
    t.transform.rotation.z = -0.984904915729
    t.transform.rotation.w = 0.171794888895 

    br.sendTransform(t)

def raw_velodyne_handler(msg):
    sec = msg.header.stamp.secs + msg.header.stamp.nsecs / 1.0e9 
    handle_turtle_pose(sec)

def point_cloud_handler(msg):
    sec = msg.header.stamp.secs + msg.header.stamp.nsecs / 1.0e9 
    print "time is %.10f" % sec
    handle_turtle_pose(sec)

if __name__ == '__main__':
    rospy.init_node('tf_pose_sender')
    raw_topic_name = "/apollo/sensor/velodyne16/VelodyneScanUnified"
    pointcloud_topic_name = "/apollo/sensor/velodyne16/PointCloud2"
    rospy.Subscriber(pointcloud_topic_name, sensor_msgs.msg.PointCloud2, point_cloud_handler);
    rospy.spin()
