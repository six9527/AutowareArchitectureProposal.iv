#!/usr/bin/python
import rospy

import numpy as np

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistWithCovarianceStamped
from std_msgs.msg import Header
import math

from ublox_msgs.msg import NavPVT

# transform between sensor and vehicle
trans_matrix = np.array([
    [0.999996, 0.002721, 0.001026, 0.287821],
    [0.002728, -0.999969, -0.007332, -0.437961],
    [0.001006, 0.007334, -0.999973, -0.322812],
    [0, 0, 0, 1]])

# rotation_matrix = np.array([
#     [0.999996, 0.002721, 0.001026],
#     [0.002728, -0.999969, -0.007332],
#     [0.001006, 0.007334, -0.999973]])

rotation_matrix = np.array([
    [1.0000000, 0.0000000, 0.0000000],
    [0.0000000, 0.9993908, -0.0348995],
   [0.0000000,  0.0348995,  0.9993908]])

def tranform_twist(rot_mat, untransed_twist):
    transed_twist = Twist()
    untransed_omega = np.array([untransed_twist.angular.x, untransed_twist.angular.y, untransed_twist.angular.z])
    transed_omega = np.matmul(rot_mat.T, untransed_omega.T)
    transed_twist.linear.x = untransed_twist.linear.x
    # transed_twist.angular.x = transed_omega[0]
    # transed_twist.angular.y = transed_omega[1]
    transed_twist.angular.z = transed_omega[2]
    return transed_twist
    


def eulerFromQuaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
     
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
     
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
     
    return roll_x, pitch_y, yaw_z

def get_quaternion_from_euler(roll, pitch, yaw):
    """
    Convert an Euler angle to a quaternion.
    
    Input
      :param roll: The roll (rotation around x-axis) angle in radians.
      :param pitch: The pitch (rotation around y-axis) angle in radians.
      :param yaw: The yaw (rotation around z-axis) angle in radians.
 
    Output
      :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
    """
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
    return [qx, qy, qz, qw]



class FixpositionConverter:
    def __init__(self):


        self.sub_current_odom = rospy.Subscriber('/fixposition/odometry_enu', Odometry, self.current_odom_callback, queue_size=10)

        self.pub_twist = rospy.Publisher('/fixposition/twist_estimator/twist', TwistStamped, queue_size=2, latch=False)
        self.pub_twist_with_covariance = rospy.Publisher('/fixposition/twist_estimator/twist_with_covariance', TwistWithCovarianceStamped, queue_size=2, latch=False)
        self.pub_navpvt = rospy.Publisher('/fixposition/navpvt', NavPVT, queue_size=2, latch=False)

    def current_odom_callback(self, msg):
        current_twist_convariances_msg = TwistWithCovarianceStamped()
        current_twist_msg = TwistStamped()
        current_navpvt_msg = NavPVT()

        header_msg = Header()
        header_msg.frame_id = "base_link"
        header_msg.stamp = msg.header.stamp

        current_twist_convariances_msg.header = header_msg
        # transed_twist = tranform_twist(rotation_matrix, msg.twist.twist)
        transed_twist = msg.twist.twist
        current_twist_convariances_msg.twist = msg.twist
        current_twist_convariances_msg.twist.twist = transed_twist

        current_twist_msg.header = header_msg
        current_twist_msg.twist = transed_twist

        temp_x = msg.pose.pose.orientation.x
        temp_y = msg.pose.pose.orientation.y
        temp_z = msg.pose.pose.orientation.z
        temp_w = msg.pose.pose.orientation.w
        euler = eulerFromQuaternion(temp_x, temp_y, temp_z, temp_w) 

        current_navpvt_msg.heading = int(math.degrees(euler[2])*100000)
        self.pub_twist.publish(current_twist_msg)
        self.pub_twist_with_covariance.publish(current_twist_convariances_msg)
        self.pub_navpvt.publish(current_navpvt_msg)
    

if __name__ == '__main__':
    rospy.init_node("fixpostion_converter", anonymous=True)
    fixposition_converter_node = FixpositionConverter()
    rospy.spin()
