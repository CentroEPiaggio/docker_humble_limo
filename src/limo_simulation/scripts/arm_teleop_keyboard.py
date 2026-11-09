#!/usr/bin/env python3
"""Keyboard teleop for the simulated myCobot 280 (run in the robot namespace, e.g. robot_0).

Jogs the joints through arm_controller/joint_trajectory, starting from the current
/<ns>/joint_states, and opens/closes the gripper through mycobot/gripper_status.
"""

import math
import select
import sys
import termios
import tty

import rclpy
from builtin_interfaces.msg import Duration
from mycobot_interfaces.msg import MycobotGripperStatus
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ARM_JOINTS = ['joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3',
              'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6']
# limits from mycobot_description (rad)
LIMITS = [(-2.9321, 2.9321), (-2.4434, 2.4434), (-2.6179, 2.6179),
          (-2.6179, 2.6179), (-2.7052, 2.7925), (-3.14159, 3.14159)]
INC = 'qwerty'   # joint i +
DEC = 'asdfgh'   # joint i -

HELP = """
myCobot 280 keyboard teleop
---------------------------
 joint:   1   2   3   4   5   6
   +  :   q   w   e   r   t   y
   -  :   a   s   d   f   g   h

 o / c : open / close gripper
 z     : home (all joints 0)
 + / - : step size up / down
 Ctrl+C: quit
"""


class ArmTeleop(Node):

    def __init__(self):
        super().__init__('arm_teleop_keyboard')
        self.declare_parameter('step_deg', 5.0)
        self.declare_parameter('move_time', 0.3)  # s per key press
        self.step = math.radians(self.get_parameter('step_deg').value)
        self.target = None
        self.traj_pub = self.create_publisher(JointTrajectory, 'arm_controller/joint_trajectory', 10)
        self.grip_pub = self.create_publisher(MycobotGripperStatus, 'mycobot/gripper_status', 10)
        self.create_subscription(JointState, 'joint_states', self.on_joint_states, 10)

    def on_joint_states(self, msg):
        if self.target is not None:
            return
        pos = dict(zip(msg.name, msg.position))
        if all(j in pos for j in ARM_JOINTS):
            self.target = [pos[j] for j in ARM_JOINTS]

    def send(self, duration):
        point = JointTrajectoryPoint(positions=list(self.target), velocities=[0.0] * 6)
        point.time_from_start = Duration(sec=int(duration),
                                         nanosec=int((duration % 1.0) * 1e9))
        self.traj_pub.publish(JointTrajectory(joint_names=ARM_JOINTS, points=[point]))

    def status(self):
        deg = ' '.join(f'{math.degrees(a):7.1f}' for a in self.target)
        sys.stdout.write(f'\rstep {math.degrees(self.step):4.1f} deg | joints [{deg}]   ')
        sys.stdout.flush()

    def on_key(self, key):
        if self.target is None:
            sys.stdout.write('\rwaiting for joint_states...   ')
            sys.stdout.flush()
            return
        move_time = self.get_parameter('move_time').value
        if key in INC or key in DEC:
            i = INC.index(key) if key in INC else DEC.index(key)
            sign = 1.0 if key in INC else -1.0
            lo, hi = LIMITS[i]
            self.target[i] = min(max(self.target[i] + sign * self.step, lo), hi)
            self.send(move_time)
        elif key == 'z':
            self.target = [0.0] * 6
            self.send(3.0)
        elif key in 'oc':
            self.grip_pub.publish(MycobotGripperStatus(status=key == 'o'))
        elif key == '+':
            self.step = min(self.step * 2.0, math.radians(30.0))
        elif key == '-':
            self.step = max(self.step / 2.0, math.radians(0.5))
        self.status()


def main():
    rclpy.init()
    node = ArmTeleop()
    settings = termios.tcgetattr(sys.stdin)
    print(HELP)
    try:
        tty.setraw(sys.stdin.fileno())
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if select.select([sys.stdin], [], [], 0.05)[0]:
                key = sys.stdin.read(1)
                if key == '\x03':
                    break
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
                node.on_key(key)
                tty.setraw(sys.stdin.fileno())
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        print()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
