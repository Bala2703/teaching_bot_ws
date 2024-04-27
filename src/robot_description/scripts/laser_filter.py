#!/usr/bin/python3

import rospy
from sensor_msgs.msg import LaserScan
import array as arr

pub = rospy.Publisher("scan", LaserScan, queue_size=1)

def callback(data):
    sliced_scan = LaserScan()

    sliced_scan.header = data.header

    sliced_scan.angle_min = data.angle_min

    sliced_scan.angle_max = data.angle_max

    sliced_scan.angle_increment = data.angle_increment

    sliced_scan.time_increment = data.time_increment

    sliced_scan.range_min = data.range_min

    sliced_scan.range_max = data.range_max

    to_slice = list(data.ranges)

    cutout_angle = [0] * 486

    to_slice[0:486] = cutout_angle

    to_slice[1461:1947] = cutout_angle

    sliced_scan.ranges = to_slice

    sliced_scan.intensities = data.intensities

    pub.publish(sliced_scan)

    rospy.loginfo(len(data.ranges))

def listener():
    rospy.init_node('laser_filter')
    rospy.Subscriber("base_scan", LaserScan, callback=callback)
    rospy.spin()


if __name__ == "__main__":
    while not rospy.is_shutdown():
        listener()