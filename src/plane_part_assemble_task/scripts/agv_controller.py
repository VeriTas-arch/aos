import rospy
from geometry_msgs.msg import Twist

class agv_controller:
    def __init__(self):
        self.pub = rospy.Publisher('/agv_part/cmd_vel', Twist, queue_size=10)
        rospy.init_node('agv_controller', anonymous=True)
        rate = rospy.Rate(10) # 10hz

        self.control_message = Twist()
    
    def update_message(self,v,omega):
        self.control_message.linear.x = v[0]
        self.control_message.linear.y = v[1]
        self.control_message.linear.z = v[2]
        self.control_message.angular.x = omega[0]
        self.control_message.angular.y = omega[1]
        self.control_message.angular.z = omega[2]
        return 0
    
    def pub_message_ask(self):
        v = [0,0,0]
        omega = [0,0,0]
        v[0] = float(input("v_x="))
        v[1] = float(input("v_y="))
        v[2] = float(input("v_z="))
        omega[0] = float(input("omega_x="))
        omega[1] = float(input("omega_y="))
        omega[2] = float(input("omega_z="))
        self.update_message(v,omega)
        rospy.loginfo(self.control_message)
        self.pub.publish(self.control_message)

if __name__ == '__main__':
    controller = agv_controller()
    while not rospy.is_shutdown():
        controller.pub_message_ask()