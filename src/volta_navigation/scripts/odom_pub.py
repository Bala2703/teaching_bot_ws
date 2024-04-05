#!/usr/bin/python3

import rospy
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Quaternion
from math import sin, cos, pi

class OdomPublish:
        
        def __init__(self):
            rospy.init_node("omniwheel_tf")
            self.nodename = rospy.get_name()
            rospy.loginfo("-I- %s started" % self.nodename)

            #params
            self.dx = 0
            self.dy = 0
            self.dth = 0
            self.th = 0
            self.vx = 0
            self.vy = 0
            self.vth = 0
            self.current_time = rospy.Time.now()
            self.last_time = rospy.Time.now()
            self.wheel1 = 0
            self.wheel2 = 0
            self.wheel3 = 0
            self.wheel4 = 0         
            self.wheel1_enc = 0
            self.wheel2_enc = 0
            self.wheel3_enc = 0
            self.wheel4_enc = 0
            self.prev_wheel1_enc = 0
            self.prev_wheel2_enc = 0
            self.prev_wheel3_enc = 0
            self.prev_wheel4_enc = 0
            self.elapsed = 0
            
            rospy.Subscriber("value_wheels", Int32MultiArray, self.encoderCallback)
            self.odomPub = rospy.Publisher("odom", Odometry, queue_size=10)
            self.odomBroadcaster = TransformBroadcaster() 

        
        def spin(self):
            r = rospy.Rate(10)
            while not rospy.is_shutdown():
                self.update()
                r.sleep()
        
        def update(self):
            self.current_time = rospy.Time.now()
            self.elapsed = self.last_time - self.current_time
            self.elapsed = self.elapsed.to_sec()
            self.last_time = self.current_time

            self.vx = -0.0176*(self.wheel1 - self.wheel2 - self.wheel3 + self.wheel4)
            self.vy = -0.0176*(self.wheel1 + self.wheel2 - self.wheel3 - self.wheel4)

            self.dx += (self.vx * cos(self.dth) - self.vy * sin(self.dth))/self.elapsed
            self.dy += (self.vx * sin(self.dth) + self.vy * cos(self.dth))/self.elapsed
            
            quaternion = Quaternion()
            quaternion.x = 0.0
            quaternion.y = 0.0
            quaternion.z = sin(self.dth/2)
            quaternion.w = cos(self.dth/2)
            # self.odomBroadcaster.sendTransform(
            #     (self.dx, self.dy, 0),
            #     (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
            #     rospy.Time.now(),
            #     "base_footprint",
            #     "odom"
            #     )
            odom = Odometry()
            odom.header.stamp = self.current_time
            odom.header.frame_id = "odom"
            odom.pose.pose.position.x = self.dx
            odom.pose.pose.position.y = self.dy
            odom.pose.pose.position.z = 0
            odom.pose.pose.orientation = quaternion
            odom.child_frame_id = "base_footprint"
            odom.twist.twist.linear.x = self.vx
            odom.twist.twist.linear.y = self.vy
            odom.twist.twist.angular.z = self.vth

            self.odomPub.publish(odom)

        def encoderCallback(self,msg):
            
            # self.current_time = rospy.Time.now()
            # self.elapsed = self.current_time - self.last_time
            # self.elapsed = self.elapsed.to_sec()
            # self.last_time = self.current_time
            
            self.wheel1_enc = msg.data[0]
            self.wheel2_enc = msg.data[1]
            self.wheel3_enc = msg.data[2]
            self.wheel4_enc = msg.data[3]

            self.wheel1 = (self.wheel1_enc - self.prev_wheel1_enc)/378051 #329348 #362283
            self.wheel2 = (self.wheel2_enc - self.prev_wheel2_enc)/379793 #329920 #362912
            self.wheel3 = (self.wheel3_enc - self.prev_wheel3_enc)/382778 #328283 #361112
            self.wheel4 = (self.wheel4_enc - self.prev_wheel4_enc)/379420 #329270 #362198

            self.prev_wheel1_enc = self.wheel1_enc
            self.prev_wheel2_enc = self.wheel2_enc
            self.prev_wheel3_enc = self.wheel3_enc
            self.prev_wheel4_enc = self.wheel4_enc

            self.vth = (((self.wheel1 + self.wheel2 + self.wheel3 + self.wheel4)/4)/0.172)
            self.dth += self.vth

if __name__ == '__main__':
    """ main """
    try:
        omniTf = OdomPublish()
        omniTf.spin()
    except rospy.ROSInterruptException:
        pass
