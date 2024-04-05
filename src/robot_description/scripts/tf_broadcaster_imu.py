#!/usr/bin/python3

#-0.107 0.102 0.113
import rospy
import tf2_ros
import geometry_msgs.msg
from sensor_msgs.msg import Imu

def handle_imu_pose(msg):
    br = tf2_ros.TransformBroadcaster()    
    imu_transform = geometry_msgs.msg.TransformStamped()
    imu_transform.header.stamp = rospy.Time.now()
    imu_transform.header.frame_id = "base_footprint"
    imu_transform.child_frame_id = "imu_link"   
    imu_transform.transform.translation.x = -0.107
    imu_transform.transform.translation.y = 0.102
    imu_transform.transform.translation.z = 0.113
    imu_transform.transform.rotation.x = msg.orientation.x
    imu_transform.transform.rotation.y = msg.orientation.y
    imu_transform.transform.rotation.z = msg.orientation.z
    imu_transform.transform.rotation.w = msg.orientation.w
    br.sendTransform(imu_transform)

if __name__ == '__main__':
    rospy.init_node('tf_broadcaster_imu')
    rospy.Subscriber('/bno08x/raw', Imu, handle_imu_pose)
    rospy.spin()




