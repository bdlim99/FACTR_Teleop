from ament_index_python.packages import get_package_share_directory
from franka_msgs.action import Grasp, Move
from franka_msgs.msg import FrankaRobotState
from geometry_msgs.msg import PoseStamped
import numpy as np
import os
import pinocchio as pin
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions
import signal
import xacro

from factr_teleop.factr_teleop import FACTRTeleop

GRASP_SPEED = 0.1
GRASP_FORCE = 100.0
GRASP_EPSILON_INNER = 0.0
GRASP_EPSILON_OUTER = 0.08


class CartesianTeleoperator(FACTRTeleop):
    def set_up_communication(self):
        special_connection = \
            self.declare_parameter('special_connection', '').get_parameter_value().string_value
        self.des_topic_name = \
            self.declare_parameter('des_topic_name', '').get_parameter_value().string_value

        xml = xacro.process_file(
            os.path.join(
                get_package_share_directory('franka_description'), 'robots', 'panda_arm.urdf.xacro'
            ),
            mappings={'hand': 'true', 'special_connection': special_connection}
        ).toxml()

        self.panda_pin_model = pin.buildModelFromXML(xml)
        self.panda_pin_data = self.panda_pin_model.createData()

        self.panda_ee_frame_id = self.panda_pin_model.getFrameId('panda_hand_tcp')

        if self.panda_ee_frame_id >= len(self.panda_pin_model.frames):
            raise ValueError("panda_hand_tcp frame not found.")

        self.des_pose_sub = self.create_publisher(PoseStamped, self.des_topic_name, qos_profile=10)

        self.external_torque = np.zeros(self.num_arm_joints)

        self.external_torque_sub = self.create_subscription(
            FrankaRobotState,
            '/franka_robot_state_broadcaster/robot_state',
            lambda msg: setattr(self, 'external_torque', np.asarray(msg.tau_ext_hat_filtered)),
            10
        )

        self.grasp_client = ActionClient(self, Grasp, '/panda_gripper/grasp')
        self.move_client = ActionClient(self, Move, '/panda_gripper/move')

        self.last_gripper_goal = None

    def update_communication(self, leader_arm_pos, leader_gripper_pos):
        q = np.concatenate((leader_arm_pos, [0, 0]))
        q = np.asarray(q, dtype=float)

        pin.forwardKinematics(self.panda_pin_model, self.panda_pin_data, q)
        pin.updateFramePlacement(self.panda_pin_model, self.panda_pin_data, self.panda_ee_frame_id)

        pose = pin.SE3ToXYZQUAT(self.panda_pin_data.oMf[self.panda_ee_frame_id])

        pos = pose[:3]
        quat = pose[3:]

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'panda_link0'
        msg.pose.position.x = pos[0]
        msg.pose.position.y = pos[1]
        msg.pose.position.z = pos[2]
        msg.pose.orientation.x = quat[0]
        msg.pose.orientation.y = quat[1]
        msg.pose.orientation.z = quat[2]
        msg.pose.orientation.w = quat[3]

        self.des_pose_sub.publish(msg)

        if self.enable_gripper_teleop:
            if self.last_gripper_goal != 'grasp' and leader_gripper_pos < 0.45 * self.gripper_limit_max:
                goal = Grasp.Goal()
                goal.width = 0.0
                goal.force = GRASP_FORCE
                goal.speed = GRASP_SPEED
                goal.epsilon.inner = GRASP_EPSILON_INNER
                goal.epsilon.outer = GRASP_EPSILON_OUTER

                self.grasp_client.send_goal_async(goal)
                self.last_gripper_goal = 'grasp'
            elif self.last_gripper_goal != 'move' and leader_gripper_pos > 0.55 * self.gripper_limit_max:
                goal = Move.Goal()
                goal.width = 0.08
                goal.speed = GRASP_SPEED

                self.move_client.send_goal_async(goal)
                self.last_gripper_goal = 'move'

    def get_leader_arm_external_joint_torque(self):
        return self.external_torque

    def get_leader_gripper_feedback(self):
        return 0.0

    def gripper_feedback(self, leader_gripper_pos, leader_gripper_vel, gripper_feedback):
        return 0.0


def main():
    signal.signal(signal.SIGINT, signal.default_int_handler)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)

    node = None

    try:
        node = CartesianTeleoperator()

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
