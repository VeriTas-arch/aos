import rospy
import tf2_ros
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Bool, Float64MultiArray, Header
from tf import TransformListener, transformations


class StewartVirtualModel:
    def __init__(self):
        self.joint_names = [
            "platform_x_joint",
            "platform_y_joint",
            "platform_z_joint",
            "platform_a_joint",
            "platform_b_joint",
            "platform_c_Joint",
        ]

        self.link_names = ["world", "base_link", "bottom_Link", "platform_Link"]

        self.name_position_dict = {x: 0.0 for x in self.joint_names}

        self.js = JointState()
        self.js.header = Header()
        self.js.name = [i for i in self.name_position_dict.keys()]
        self.js.position = [self.name_position_dict[i] for i in self.js.name]
        self.js.velocity = []
        self.js.effort = []

        self.pub = rospy.Publisher("stewart_joint_states", JointState, queue_size=1)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf = tf2_ros.TransformListener(self.tf_buffer)

    def goto_zero_config(self):
        activeNames = self.joint_names
        for i in activeNames:
            self.name_position_dict[i] = 0.0

        self.publishActiveJointStates(activeNames)

    def get_relative_transform(self, sourceLink, targetLink):
        if not sourceLink in self.link_names or not targetLink in self.link_names:
            return []

        trans = self.tf_buffer.lookup_transform(sourceLink, targetLink, rospy.Time(), rospy.Duration(1.0))

        ret = [
            trans.transform.translation.x,
            trans.transform.translation.y,
            trans.transform.translation.z,
            trans.transform.rotation.x,
            trans.transform.rotation.y,
            trans.transform.rotation.z,
            trans.transform.rotation.w,
        ]

        return ret

    def publishActiveJointStates(self, activeNames):
        self.js.name = activeNames
        self.js.position = [self.name_position_dict[i] for i in activeNames]
        self.js.header.stamp = rospy.Time.now()
        self.pub.publish(self.js)
