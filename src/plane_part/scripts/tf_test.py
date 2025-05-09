import rospy
import tf
import time

def handle_link_pose(x,y,z,a,b,c, link_name_1, link_name_2):
    br = tf.TransformBroadcaster()
    br.sendTransform((x, y, z),
            tf.transformations.quaternion_from_euler(a,b,c),
            rospy.Time.now(),
            link_name_1,
            link_name_2)

if __name__ == '__main__':
    rospy.init_node('link_pose_broadcaster')
    while 1:
        x, y, z, a, b, c = 0, 2, 0, 0, 0, 0
        link_name_1 = "agv_part/agv_Link"
        link_name_2 = "modulating_part/world"
        handle_link_pose(x,y,z,a,b,c, link_name_1, link_name_2)
        print("sending...")
        time.sleep(0.1)
