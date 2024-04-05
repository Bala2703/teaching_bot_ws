#!/usr/bin/python3
import rospy
from geometry_msgs.msg import Twist
from math import atan2, sqrt, sin, cos
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class MoveToGoal:

    def __init__(self):
        rospy.init_node('omni_controller', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/odom', Odometry, self.update_pose)
        self.pose = Odometry().pose.pose.position
        self.current_yaw = 0.0
        self.rate = rospy.Rate(40)

    def update_pose(self, data):
        """Callback function to update pose."""
        self.pose = data.pose.pose.position
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)
        
        quaternion = (data.pose.pose.orientation.x, data.pose.pose.orientation.y, 
                      data.pose.pose.orientation.z, data.pose.pose.orientation.w)
        _, _, self.current_yaw = euler_from_quaternion(quaternion)

    def mover(self, x_dist, y_dist):
        """Move the robot to the specified positions."""
        rate = rospy.Rate(10)
        speed = Twist()

        # Move along the x-axis
        goal_x = self.pose.x + x_dist
        goal_y = self.pose.y

        while not rospy.is_shutdown() and abs(self.pose.x - goal_x) > 0.02:
            delta_x = goal_x - self.pose.x
            speed.linear.x = 0.2  # Adjust linear speed based on the distance to the goal
            speed.linear.y = 0.0
            speed.angular.z = 0.0
            self.velocity_publisher.publish(speed)
            rate.sleep()

        # Stop the robot
        speed.linear.x = 0.0
        self.velocity_publisher.publish(speed)
        rospy.sleep(1)  # Pause for a moment before the next movement

        # Move along the y-axis
        goal_x = self.pose.x
        goal_y = self.pose.y + y_dist

        while not rospy.is_shutdown() and abs(self.pose.y - goal_y) > 0.02:
            delta_y = goal_y - self.pose.y
            speed.linear.x = 0.0
            speed.linear.y =  0.2  # Adjust linear speed based on the distance to the goal
            speed.angular.z = 0.0
            self.velocity_publisher.publish(speed)
            rate.sleep()

        # Stop the robot
        speed.linear.y = 0.0
        self.velocity_publisher.publish(speed)

if __name__ == '__main__':
    try:
        x = MoveToGoal()
        x.mover(1.0, 0.0)  # Move 1 meter along the x-axis
        x.mover(0.0, 1.0)  # Move 1 meter along the y-axis
    except rospy.ROSInterruptException:
        pass
