import time

import rospy
from std_msgs.msg import Int8


class central_controller:
    def __init__(self):
        rospy.init_node("central_controller", anonymous=True)
        self.pub_agv = rospy.Publisher("/agv_ctrl_cmd", Int8, queue_size=10)
        self.pub_modu = rospy.Publisher("/modu_ctrl_cmd", Int8, queue_size=10)

        self.sub_agv = rospy.Subscriber(
            "/agv_state", Int8, self.agv_state_callback
        )  # 0:not moving; 1: moving
        self.sub_modu = rospy.Subscriber(
            "/modu_state", Int8, self.modu_state_callback
        )  # 0:not moving; 1: moving
        self.agv_state = 0
        self.modu_state = 0

    def agv_state_callback(self, msg):
        if msg.data == 0:
            print("agv is not working.")
            self.agv_state = 0
        elif msg.data == 1:
            print("agv is working.")
            self.agv_state = 1
        else:
            print("agv_state_error")

    def modu_state_callback(self, msg):
        if msg.data == 0:
            print("modu is not working.")
            self.modu_state = 0
        elif msg.data == 1:
            print("modu is working.")
            self.modu_state = 1
        else:
            print("modu_state_error")

    def run(self):
        state_num = Int8()

        print("state 1: modulating part is adjusting it's z")

        while self.modu_state != 1 or self.agv_state != 0:
            if self.modu_state != 1:
                state_num.data = 1
                print("waiting for modu part responses...")
                self.pub_modu.publish(state_num)
                time.sleep(0.05)
            elif self.agv_state != 0:
                state_num.data = 1
                print("waiting for agv part responses...")
                self.pub_agv.publish(state_num)
                time.sleep(0.05)

        while self.modu_state == 1 and self.agv_state == 0:
            print(self.modu_state, self.agv_state)
            print("state 1 is working")
            time.sleep(0.05)

        print("state 2: agv part is adjusting it's pos")

        while self.modu_state != 0 or self.agv_state != 1:
            if self.modu_state != 0:
                state_num.data = 2
                print("waiting for modu part responses...")
                self.pub_modu.publish(state_num)
                time.sleep(0.05)
            elif self.agv_state != 1:
                state_num.data = 2
                print("waiting for agv part responses...")
                self.pub_agv.publish(state_num)
                time.sleep(0.05)

        while self.modu_state == 0 and self.agv_state == 1:
            print(self.modu_state, self.agv_state)
            print("state 2 is working")
            time.sleep(0.05)

        input("Is agv OK?")
        print("state 3: modulating part is adjusting it's pos")

        while self.modu_state != 1 or self.agv_state != 0:
            if self.modu_state != 1:
                state_num.data = 3
                print("waiting for modu part responses...")
                self.pub_modu.publish(state_num)
                time.sleep(0.05)
            elif self.agv_state != 0:
                state_num.data = 3
                print("waiting for agv part responses...")
                self.pub_agv.publish(state_num)
                time.sleep(0.05)

        while self.modu_state == 1 and self.agv_state == 0:
            print(self.modu_state, self.agv_state)
            print("state 3 is working")
            time.sleep(0.05)

        print("ALL DONE")


if __name__ == "__main__":
    controller = central_controller()
    controller.run()
