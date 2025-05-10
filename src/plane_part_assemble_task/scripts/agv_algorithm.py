import numpy as np
import rospy
import tf2_ros
from agv_controller import agv_controller


def lookup_agv2mod_trans(tfBuffer, target_frame, source_frame, time, timeout):
    trans_ = tfBuffer.lookup_transform(target_frame, source_frame, time, timeout)
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


if __name__ == "__main__":
    agv_controller = agv_controller()
    tfBuffer = agv_controller.tfBuffer

    target_frame = "agv_part/image_target1"
    source_frame = "agv_part/agv_Link"
    # source_frame = "agv_part/reference_Link"
    print(
        lookup_agv2mod_trans(
            tfBuffer, target_frame, source_frame, rospy.Time(0), rospy.Duration(1.0)
        )
    )
