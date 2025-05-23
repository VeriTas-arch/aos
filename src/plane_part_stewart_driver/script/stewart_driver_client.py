#!/usr/bin/python3

from __future__ import print_function

import sys

# Brings in the SimpleActionClient
import actionlib
# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
import moveit_msgs.msg
import rospy


class plane_part_stewart_client(object):
    def __init__(self):
        self.goalsub = rospy.Subscriber(
            "/execute_trajectory/goal", moveit_msgs.msg.ExecuteTrajectoryActionGoal, self.traj_cb
        )

        # Creates the SimpleActionClient, passing the type of the action
        # (FibonacciAction) to the constructor.
        self.client = actionlib.SimpleActionClient(
            "stewart_driver", moveit_msgs.msg.ExecuteTrajectoryAction
        )

        # Waits until the action server has started up and started
        # listening for goals.
        self.client.wait_for_server()

    def traj_cb(self, msg):
        traj = msg.goal.trajectory
        """
        for i in traj_points:
            #print(i.positions)
            print(i.time_from_start.secs,i.time_from_start.nsecs)
        """
        try:
            # Creates a goal to send to the action server.
            goal = moveit_msgs.msg.ExecuteTrajectoryGoal(traj)

            # Sends the goal to the action server.
            self.client.send_goal(goal)

            # Waits for the server to finish performing the action.
            self.client.wait_for_result()

            result = self.client.get_result()

            print(result)

            # Prints out the result of executing the action
            # print("Result:", ', '.join([str(n) for n in result.sequence]))

        except rospy.ROSInterruptException:
            print("program interrupted before completion", file=sys.stderr)
        return


if __name__ == "__main__":
    # Initializes a rospy node so that the SimpleActionClient can
    # publish and subscribe over ROS.
    rospy.init_node("stewart_driver_client")
    print("success node")
    client = plane_part_stewart_client()
    rospy.spin()
