#!/usr/bin/env python3
"""Simulated myCobot 280: the real robot's topic API on top of ros2_control.

Mirrors mycobot_communication/mycobot_topics.py (limo_mycobot_sk), relative to the node
namespace (e.g. /robot_0):

  sub  mycobot/angles_goal     MycobotSetAngles  [deg], speed 1-100 -> arm_controller trajectory
  sub  mycobot/coords_goal     MycobotSetCoords  [mm, deg] flange pose in g_base -> IK -> trajectory
  sub  mycobot/gripper_status  MycobotGripperStatus  True = open, False = close
  pub  mycobot/angles_real     MycobotAngles     [deg]
  pub  mycobot/coords_real     MycobotCoords     [mm, deg]
  sub  cmd_vel                 Twist -> diff_drive_base_controller/cmd_vel (TwistStamped),
                               so real-robot scripts publishing a plain Twist also drive the base.

rx, ry, rz are treated as fixed-axis XYZ angles, R = Rz(rz) Ry(ry) Rx(rx), as in pymycobot.
"""

import math

import numpy as np
import pinocchio as pin
import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Twist, TwistStamped
from mycobot_interfaces.msg import (MycobotAngles, MycobotCoords, MycobotGripperStatus,
                                    MycobotSetAngles, MycobotSetCoords)
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ARM_JOINTS = ['joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3',
              'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6']
# gripper_group_controller order (config/limo_mycobot_controllers.yaml) and former <mimic> multipliers
GRIPPER_JOINTS = ['gripper_controller', 'gripper_base_to_gripper_left2',
                  'gripper_left3_to_gripper_left1', 'gripper_base_to_gripper_right3',
                  'gripper_base_to_gripper_right2', 'gripper_right3_to_gripper_right1']
GRIPPER_MULTIPLIERS = [1.0, 1.0, -1.0, -1.0, -1.0, 1.0]
# driver range where no linkage joint is clamped by its own limit (see mycobot_description)
GRIPPER_RANGE = (-0.5, 0.15)


def rpy_to_matrix(rx, ry, rz):
    return pin.rpy.rpyToMatrix(rx, ry, rz)


def matrix_to_rpy(rot):
    return pin.rpy.matrixToRpy(rot)


class MycobotSimBridge(Node):

    def __init__(self):
        super().__init__('mycobot_sim_bridge')
        self.declare_parameter('max_joint_speed', 2.5)   # rad/s at speed = 100
        self.declare_parameter('min_duration', 0.3)      # s
        self.declare_parameter('gripper_open', 0.15)     # rad, as shown by the real robot in rviz
        # The real URDF allows -0.74, but past -0.5 the linkage joints hit their own limits
        # (left1 +-0.5, right3 < 0.7) and the fingers fold into each other. Tune live with
        # ros2 param set /<robot>/mycobot_sim_bridge gripper_closed <rad>
        self.declare_parameter('gripper_closed', -0.74)   # rad
        self.declare_parameter('gripper_time', 0.6)      # s for a full open/close
        self.declare_parameter('state_rate', 10.0)       # Hz, as mycobot_topics.py

        ns = self.get_namespace().strip('/')
        self.prefix = f'{ns}/' if ns else ''
        self.q = None            # current arm angles [rad], ARM_JOINTS order
        self.gripper = None      # current driver angle [rad]
        self.gripper_timer = None
        self.model = self.data = self.frame_id = self.q_idx = None

        self.traj_pub = self.create_publisher(JointTrajectory, 'arm_controller/joint_trajectory', 10)
        self.gripper_pub = self.create_publisher(Float64MultiArray, 'gripper_group_controller/commands', 10)
        self.angles_pub = self.create_publisher(MycobotAngles, 'mycobot/angles_real', 10)
        self.coords_pub = self.create_publisher(MycobotCoords, 'mycobot/coords_real', 10)
        self.cmd_pub = self.create_publisher(TwistStamped, 'diff_drive_base_controller/cmd_vel', 10)

        self.create_subscription(JointState, 'joint_states', self.on_joint_states, 10)
        self.create_subscription(MycobotSetAngles, 'mycobot/angles_goal', self.on_angles_goal, 10)
        self.create_subscription(MycobotSetCoords, 'mycobot/coords_goal', self.on_coords_goal, 10)
        self.create_subscription(MycobotGripperStatus, 'mycobot/gripper_status',
                                 self.on_gripper_status, 10)
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, 'robot_description', self.on_description, latched)

        self.create_timer(1.0 / self.get_parameter('state_rate').value, self.publish_state)

    # ---------------------------------------------------------------- kinematics

    def on_description(self, msg):
        try:
            full = pin.buildModelFromXML(msg.data)
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'Cannot build the arm model: {e}')
            return
        lock = [full.getJointId(j) for j in GRIPPER_JOINTS if full.existJointName(j)]
        # keep only the arm joints (the gripper does not change the flange pose)
        lock += [i for i in range(1, full.njoints)
                 if full.names[i] not in ARM_JOINTS and i not in lock]
        self.model = pin.buildReducedModel(full, lock, pin.neutral(full))
        self.data = self.model.createData()
        self.base_frame = self.model.getFrameId(f'{self.prefix}g_base')
        self.frame_id = self.model.getFrameId(f'{self.prefix}joint6_flange')
        self.q_idx = [self.model.joints[self.model.getJointId(j)].idx_q for j in ARM_JOINTS]
        self.get_logger().info('Arm kinematic model ready (coords_goal / coords_real enabled)')

    def flange_in_base(self, q_arm):
        q = pin.neutral(self.model)
        for i, v in zip(self.q_idx, q_arm):
            q[i] = v
        pin.framesForwardKinematics(self.model, self.data, q)
        return self.data.oMf[self.base_frame].actInv(self.data.oMf[self.frame_id]), q

    def solve_ik(self, target, q0, iters=300, eps=1e-4, damping=1e-6, step=0.5):
        """Damped least squares IK of the flange pose in g_base. Returns (q, converged)."""
        q_arm = np.array(q0, dtype=float)
        lo = self.model.lowerPositionLimit[self.q_idx]
        hi = self.model.upperPositionLimit[self.q_idx]
        for _ in range(iters):
            current, q = self.flange_in_base(q_arm)
            err = pin.log6(current.actInv(target)).vector
            if np.linalg.norm(err) < eps:
                return q_arm, True
            # jacobian of the flange in its local frame (g_base is fixed to the arm root)
            jac = pin.computeFrameJacobian(self.model, self.data, q, self.frame_id, pin.LOCAL)
            jac = jac[:, [self.model.joints[self.model.getJointId(j)].idx_v for j in ARM_JOINTS]]
            dq = jac.T @ np.linalg.solve(jac @ jac.T + damping * np.eye(6), err)
            q_arm = np.clip(q_arm + step * dq, lo, hi)
        return q_arm, False

    # ---------------------------------------------------------------- callbacks

    def on_joint_states(self, msg):
        pos = dict(zip(msg.name, msg.position))
        if all(j in pos for j in ARM_JOINTS):
            self.q = [pos[j] for j in ARM_JOINTS]
        if GRIPPER_JOINTS[0] in pos:
            self.gripper = pos[GRIPPER_JOINTS[0]]

    def send_trajectory(self, q_goal, speed):
        speed = min(max(int(speed), 1), 100)
        vel = self.get_parameter('max_joint_speed').value * speed / 100.0
        delta = max(abs(g - c) for g, c in zip(q_goal, self.q)) if self.q else math.pi
        t = max(delta / vel, self.get_parameter('min_duration').value)
        point = JointTrajectoryPoint(positions=[float(v) for v in q_goal],
                                     velocities=[0.0] * len(ARM_JOINTS))
        point.time_from_start = Duration(sec=int(t), nanosec=int((t % 1.0) * 1e9))
        self.traj_pub.publish(JointTrajectory(joint_names=ARM_JOINTS, points=[point]))

    def on_angles_goal(self, msg):
        deg = [msg.joint_1, msg.joint_2, msg.joint_3, msg.joint_4, msg.joint_5, msg.joint_6]
        self.send_trajectory([math.radians(a) for a in deg], msg.speed)

    def on_coords_goal(self, msg):
        if self.model is None:
            self.get_logger().warn('coords_goal ignored: no robot_description received yet')
            return
        target = pin.SE3(rpy_to_matrix(*(math.radians(a) for a in (msg.rx, msg.ry, msg.rz))),
                         np.array([msg.x, msg.y, msg.z]) / 1000.0)
        seeds = [self.q or [0.0] * 6, [0.0] * 6, [0.0, -0.5, -1.0, 0.0, 0.0, 0.0]]
        for seed in seeds:
            q_goal, ok = self.solve_ik(target, seed)
            if ok:
                self.send_trajectory(q_goal, msg.speed)
                return
        self.get_logger().warn(
            f'coords_goal unreachable: [{msg.x:.1f}, {msg.y:.1f}, {msg.z:.1f}] mm, '
            f'[{msg.rx:.1f}, {msg.ry:.1f}, {msg.rz:.1f}] deg')

    def on_gripper_status(self, msg):
        goal = self.get_parameter('gripper_open' if msg.status else 'gripper_closed').value
        goal = min(max(goal, GRIPPER_RANGE[0]), GRIPPER_RANGE[1])
        start = self.gripper if self.gripper is not None else goal
        total = self.get_parameter('gripper_time').value
        steps = max(int(total * 50), 1)
        if self.gripper_timer is not None:
            self.gripper_timer.cancel()
        self._gripper_step = 0

        # ramp the position command: a step makes the light linkage jump in Gazebo
        def tick():
            self._gripper_step += 1
            a = start + (goal - start) * min(self._gripper_step / steps, 1.0)
            self.gripper_pub.publish(Float64MultiArray(data=[m * a for m in GRIPPER_MULTIPLIERS]))
            if self._gripper_step >= steps:
                self.gripper_timer.cancel()

        self.gripper_timer = self.create_timer(1.0 / 50.0, tick)

    def on_cmd_vel(self, msg):
        out = TwistStamped(twist=msg)
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = f'{self.prefix}base_footprint'
        self.cmd_pub.publish(out)

    def publish_state(self):
        if self.q is None:
            return
        deg = [math.degrees(a) for a in self.q]
        self.angles_pub.publish(MycobotAngles(
            joint_1=deg[0], joint_2=deg[1], joint_3=deg[2],
            joint_4=deg[3], joint_5=deg[4], joint_6=deg[5]))
        if self.model is not None:
            pose, _ = self.flange_in_base(self.q)
            xyz = pose.translation * 1000.0
            rpy = np.degrees(matrix_to_rpy(pose.rotation))
            self.coords_pub.publish(MycobotCoords(
                x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2]),
                rx=float(rpy[0]), ry=float(rpy[1]), rz=float(rpy[2])))


def main():
    rclpy.init()
    node = MycobotSimBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
