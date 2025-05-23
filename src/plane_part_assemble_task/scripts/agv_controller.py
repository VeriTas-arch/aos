# from plane_part_with_agv import plane_part_with_agv
import time
from collections import defaultdict

# from tf2_geometry_msgs import do_transform_pose
from math import degrees

import numpy as np
import rospy
import tf
import tf2_ros
from config import Config
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8

config = Config()
MODE = config.MODE


class agv_controller:
    def __init__(self):
        rospy.init_node("agv_dummy", anonymous=True)

        # 定义坐标系名称
        self.agv_frame = "agv_part/agv_Link"
        self.reference_frame = "agv_part/reference_Link"
        self.world_frame = "world"

        # self.platform_frame = "modulating_part/platform_Link"
        # self.modulating_frame = "modulating_part/bottom_Link"
        # self.modulatingLink_frame = "modulating_part/modulating_Link"

        self.platform_frame = config.platformLink
        self.modulating_frame = config.bottomLink
        self.modulatingLink_frame = config.modulatingLink

        # 状态跟踪
        self.current_transform = None  # 当前plat -> AGV变换矩阵
        self.adjustment_counts = defaultdict(int)  # 调整计数器

        # 创建 TF 监听器和运动控制器
        self.tfBuffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tfBuffer)

        if MODE == "REAL":
            self.velocity_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)  # real
        else:
            self.velocity_publisher = rospy.Publisher("/agv_part/cmd_vel", Twist, queue_size=10)  # sim

        self.state_subscriber = rospy.Subscriber("/agv_ctrl_cmd", Int8, self.state_callback)
        self.state_publisher = rospy.Publisher(
            "/agv_state", Int8, queue_size=10
        )  # 0: not moving; 1: moving

    def run(self):
        """主执行流程"""
        rospy.loginfo("Starting AGV adjustment sequence...")

        # 阶段1：Z轴旋转粗调
        if not self.adjust_axis(
            target_value=0,  # 理想Z轴旋转角度（度）
            get_current_pos=self.get_z_error,
            direction_sign="z_rot",
            speed=config.speed[0],
            tolerance=config.tolerance[0],
            max_attempts=600,
        ):
            return 0

        # 阶段2：XY平移调整
        if not self.adjust_axis(
            target_value=-1.56,
            get_current_pos=lambda t: t[0, 3],
            direction_sign="x",
            speed=config.speed[1],
            tolerance=config.tolerance[1],
            max_attempts=600,
        ) or not self.adjust_axis(
            target_value=0.0,
            get_current_pos=lambda t: t[1, 3],
            direction_sign="y",
            speed=config.speed[2],
            tolerance=config.tolerance[2],
            max_attempts=600,
        ):
            return 0

        # 阶段3：Z轴旋转精调
        if not self.adjust_axis(
            target_value=0,
            get_current_pos=self.get_z_error,
            direction_sign="z_rot",
            speed=config.speed[3],
            tolerance=config.tolerance[3],
            max_attempts=50,
        ):
            return 0

        # 理想Y位置（米） == 'y'

    def adjust_axis(self, target_value, get_current_pos, direction_sign, speed, tolerance, max_attempts):
        """通用轴调整函数"""
        rospy.loginfo(f"Starting {direction_sign.upper()} axis adjustment...")
        for attempt in range(1, max_attempts + 1):
            current_rel = self.get_current_rel_transform()
            if current_rel is None:
                continue

            print("current rel: ", current_rel)
            print("current pos: ", get_current_pos(current_rel))
            error = target_value - get_current_pos(current_rel)

            if abs(error) < tolerance:
                cmd = self.generate_velocity_cmd(0, direction_sign, 0)
                self.send_velocity_cmd(cmd)
                rospy.loginfo(
                    f"{direction_sign.upper()} aligned (Error: {error:.3f}{self.get_unit(direction_sign)})"
                )
                return True

            # 计算速度指令
            cmd = self.generate_velocity_cmd(error, direction_sign, speed)
            self.send_velocity_cmd(cmd, duration=1.0)

            rospy.loginfo(
                f"Attempt {attempt}/{max_attempts}: Error={error:.3f}{self.get_unit(direction_sign)}"
            )
            self.adjustment_counts[f"{direction_sign}_attempts"] += 1

        rospy.logwarn(f"Max {direction_sign} adjustments reached")

        # 这里如果一直超出误差，需要发送 0 的信息让小车停下
        cmd = self.generate_velocity_cmd(0, direction_sign, 0)
        self.send_velocity_cmd(cmd)
        return False

    def generate_velocity_cmd(self, error, direction_sign, speed):
        """生成速度指令"""
        cmd = Twist()
        if direction_sign == "x":
            cmd.linear.x = -1 * speed * np.sign(error)
        elif direction_sign == "y":
            cmd.linear.y = speed * np.sign(error)
        elif direction_sign == "z_rot":
            cmd.angular.z = -1 * speed * np.sign(error)
        return cmd

    def check_error_tolerance(self, cmd, duration=0.5):
        """实时误差检测"""
        start_time = rospy.Time.now()
        error_sum = 0.0
        count = 0

        while (rospy.Time.now() - start_time).to_sec() < duration:
            current_rel = self.get_current_rel_transform()
            if current_rel is None:
                return False

            error = self.get_instant_error(cmd, current_rel)
            error_sum += abs(error)
            count += 1

        avg_error = error_sum / count
        return avg_error < 0.01  # 1cm/1° 容差

    def send_velocity_cmd(self, cmd, duration=1.0):
        """发送速度指令并保持指定时间"""
        rate = rospy.Rate(10)  # 10Hz 更新率

        # 动态误差检测
        if self.check_error_tolerance(cmd, duration=0.5):
            rospy.logdebug("Early exit due to tolerance")

        self.velocity_publisher.publish(cmd)
        print(cmd)
        rate.sleep()

    def get_instant_error(self, cmd, current_rel):
        """根据指令类型获取瞬时误差"""
        if cmd.angular.z != 0:
            _, _, yaw = tf.transformations.euler_from_matrix(current_rel)
            return degrees(yaw)
        elif cmd.linear.x != 0:
            return current_rel[0, 3]
        elif cmd.linear.y != 0:
            return current_rel[1, 3]
        return 0.0

    def get_z_error(self, transform):
        """获取Z轴旋转误差"""
        _, _, yaw = tf.transformations.euler_from_matrix(transform)
        print("initial yaw: ", yaw)
        deg = degrees(yaw)
        # if deg < -180:
        #     deg = deg + 360
        # elif deg > 180:
        #     deg = deg - 360

        # 归一化到 [-180, 180]
        # deg = (deg + 180) % 360
        return deg

    def lookup_transform_quaternion_vector(self, target_frame, source_frame, time, timeout) -> np.ndarray:
        trans_ = self.tfBuffer.lookup_transform(target_frame, source_frame, time, timeout)
        trans_ = np.array(
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

        trans_matrix = tf.transformations.quaternion_matrix(trans_[3:7])
        # trans_matrix[0,3] = trans_[0] about -1.9
        # trans_matrix[1,3] = trans_[1]
        # trans_matrix[2,3] = trans_[2]
        trans_matrix[0:3, 3] = trans_[0:3]

        return trans_matrix

    def get_current_rel_transform(self):
        """
        获取 plat -> AGV 的7元数组 (x, y, z, qx, qy, qz, qw)
        坐标系连接顺序: plat -> modulating -> reference -> agv
        返回格式：矩阵
        """
        try:
            # 分步获取各段变换矩阵
            T_agv_trans_matrix = self.lookup_transform_quaternion_vector(
                self.reference_frame, self.modulatingLink_frame, rospy.Time(0), rospy.Duration(1.0)
            )
            # T_plat_modulating = self.lookup_transform_quaternion_vector(
            #   self.platform_frame, self.modulating_frame, rospy.Time(0), rospy.Duration(1.0)
            # )
            # T_modulating_reference = self.lookup_transform_quaternion_vector(
            #    self.modulating_frame, self.reference_frame, rospy.Time(0), rospy.Duration(1.0)
            # )
            # T_reference_agv = self.lookup_transform_quaternion_vector(
            #     self.reference_frame, self.agv_frame, rospy.Time(0), rospy.Duration(1.0)
            # )

            # 矩阵连乘（注意顺序：从右到左应用变换）
            # T_plat_reference = np.dot(T_plat_modulating, T_modulating_reference)
            # T_plat_agv = np.dot(T_plat_reference, T_reference_agv)

            return T_agv_trans_matrix

            # # 提取平移部分
            # translation = T_plat_agv[:3, 3]

            # # 提取旋转四元数
            # rotation = tf.transformations.quaternion_from_matrix(T_plat_agv)

            # # 组合成7元数组
            # return np.concatenate([translation, rotation])

        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:
            rospy.logerr(f"Failed to get plat→agv transform: {str(e)}")
            return None

    def get_unit(self, direction_sign):
        """获取单位标识"""
        return "°" if direction_sign == "z_rot" else "m"

    def state_callback(self, msg):
        print("callback...")
        print("get state", msg.data)
        state = Int8()
        if msg.data == 1 or msg.data == 3:
            state.data = 0
            for _ in range(20):
                self.state_publisher.publish(state)
                time.sleep(0.01)

        elif msg.data == 2:
            state.data = 1
            for _ in range(20):
                self.state_publisher.publish(state)
                time.sleep(0.01)
            self.run()
            state.data = 0
            for _ in range(20):
                self.state_publisher.publish(state)
                time.sleep(0.01)

        else:
            return


if __name__ == "__main__":
    try:
        controller = agv_controller()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
