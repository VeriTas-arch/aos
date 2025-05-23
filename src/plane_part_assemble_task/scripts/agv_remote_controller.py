import numpy as np
import rospy
import tf2_ros
from config import Config
from geometry_msgs.msg import Twist

config = Config()
MODE = config.MODE


class agv_controller:
    def __init__(self):
        rospy.init_node("agv_controller", anonymous=True)

        if MODE == "REAL":
            self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)  # real
        else:
            self.pub = rospy.Publisher("/agv_part/cmd_vel", Twist, queue_size=10)  # sim

        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        rospy.Rate(10)  # 10hz

        self.control_message = Twist()

    def update_message(self, v, omega):
        self.control_message.linear.x = v[0]
        self.control_message.linear.y = v[1]
        self.control_message.linear.z = v[2]
        self.control_message.angular.x = omega[0]
        self.control_message.angular.y = omega[1]
        self.control_message.angular.z = omega[2]

        return 0

    def lookup_agv2mod_trans(self, target_frame, source_frame, time, timeout):
        trans_ = self.tfBuffer.lookup_transform(target_frame, source_frame, time, timeout)
        return np.array(
            [
                trans_.transform.translation.x,
                trans_.transform.translation.y,
                trans_.transform.translation.z,
                trans_.transform.rotation.x,
                trans_.transform.rotation.y,
                trans_.transform.rotation.z,
                trans_.transform.rotation.w,
            ]
        )

    def pub_message(self, v, omega):
        self.update_message(v, omega)
        rospy.loginfo(self.control_message)
        self.pub.publish(self.control_message)

    def pub_message_ask(self):
        v = [0, 0, 0]
        omega = [0, 0, 0]
        print("type the control values, press enter to skip (default value: 0)")
        v[0] = float(input("v_x=") or 0)
        v[1] = float(input("v_y=") or 0)
        v[2] = float(input("v_z=") or 0)
        omega[0] = float(input("omega_x=") or 0)
        omega[1] = float(input("omega_y=") or 0)
        omega[2] = float(input("omega_z=") or 0)
        self.update_message(v, omega)
        rospy.loginfo(self.control_message)
        self.pub.publish(self.control_message)


if __name__ == "__main__":
    controller = agv_controller()
    while not rospy.is_shutdown():
        controller.pub_message_ask()
