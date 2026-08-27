import numpy as np
from sensor_msgs.msg import JointState
import rclpy
from rclpy.executors import ExternalShutdownException

from factr_teleop.factr_teleop import FACTRTeleop

MAX_FINGER_POSITION = 0.04


class LeaderChecker(FACTRTeleop):
    def set_up_communication(self):
        self.joint_states_pub = self.create_publisher(JointState, '/joint_states', 10)

    def update_communication(self, leader_arm_pos, leader_gripper_pos):
        gripper_pos = max(0.0, min(1.0, leader_gripper_pos / self.gripper_limit_max))
        gripper_pos = gripper_pos * MAX_FINGER_POSITION

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [f'panda_joint{i+1}' for i in range(self.num_arm_joints)] \
            + ['panda_finger_joint1', 'panda_finger_joint2']
        msg.position = list(map(float, leader_arm_pos)) + [float(gripper_pos)] * 2

        self.joint_states_pub.publish(msg)

    def get_leader_arm_external_joint_torque(self):
        return np.zeros(self.num_arm_joints)

    def get_leader_gripper_feedback(self):
        return 0.0

    def gripper_feedback(self, leader_gripper_pos, leader_gripper_vel, gripper_feedback):
        return 0.0


def main():
    rclpy.init()

    node = None

    try:
        node = LeaderChecker()

        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            msg = "Shutting down: disabling leader arm torque..."

            if rclpy.ok():
                node.get_logger().info(msg)
            else:
                print(msg)

            try:
                node.shut_down()
            except Exception as e:
                msg = f"Torque release during shutdown failed: {e}"

                if rclpy.ok():
                    node.get_logger().error(msg)
                else:
                    print(msg)
    
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()
