#!/usr/bin/env python3
"""
Challenge 6 — State Machine: auto-sort multiple blocks by color
States: SCAN → PICK → PLACE → HOME → loop
Stops when full scan finds no remaining blocks.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from control_msgs.action import GripperCommand, FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from action_msgs.msg import GoalStatus
import ikpy.chain
import math
import time
import subprocess
import os

# ─── Config ───────────────────────────────────────────────────────────────────
URDF_PATH   = '/tmp/fr3_full.urdf'
JOINT_NAMES = [
    'fr3_joint1','fr3_joint2','fr3_joint3',
    'fr3_joint4','fr3_joint5','fr3_joint6','fr3_joint7',
]
HOME_JOINTS  = [0.0003, 0.0243, -0.0007, -0.1311, 0.0015, 0.4398, 0.0217]
IK_SEED      = [0, 0.0, -0.3, 0.0, -2.8, 0.0, 2.5, 0.0, 0, 0, 0]
ACTIVE_MASK  = [False,True,True,True,True,True,True,True,False,False,False]

BIN_POSITIONS = {
    'red':    {'x':  0.212, 'y': -0.212, 'z': 0.05},
    'blue':   {'x': -0.212, 'y':  0.212, 'z': 0.05},
    'green':  {'x': -0.212, 'y': -0.212, 'z': 0.05},
    'yellow': {'x':  0.212, 'y':  0.212, 'z': 0.05},
}

BLOCK_SIZE_Z         = 0.10
GRIPPER_OPEN_POS     = 0.040
GRIPPER_OPEN_EFFORT  = 0.0
GRIPPER_CLOSE_POS    = 0.020
GRIPPER_CLOSE_EFFORT = 15.0
PREGRASP_Z_OFFSET    = 0.20
GRASP_Z_OFFSET       = 0.03
LIFT_Z_OFFSET        = 0.28
PLACE_Z_OFFSET       = 0.18
JOINT7_GRASP         = 0.0

MAX_CYCLES     = 20
DETECT_TIMEOUT = 5.0

# 5 điểm scan: 0°, 45°, 90°, 135°, 180° — phủ ~332° tầm nhìn
_J = lambda j1: [j1, 0.0243, -0.0007, -0.1311, 0.0015, 0.4398, 0.0217]
SCAN_POSITIONS = [
    ('scan_0',    _J(0.00)),
    ('scan_90',   _J(1.571)),
    ('scan_180',  _J(2.90)),
    ('scan_n90',  _J(-1.571)),
]


# ─── IK Solver ────────────────────────────────────────────────────────────────
class IKSolver:
    def __init__(self, urdf_path):
        self.chain = ikpy.chain.Chain.from_urdf_file(
            urdf_path,
            base_elements=['fr3_link0'],
            base_element_type='link',
            active_links_mask=ACTIVE_MASK,
        )

    def solve(self, x, y, z, seed=None, override_j7=None):
        if seed is None:
            seed = IK_SEED
        joints = self.chain.inverse_kinematics(
            target_position=[x, y, z],
            target_orientation=[0, 0, -1],
            orientation_mode='Z',
            initial_position=seed,
        )
        result = joints[1:8].tolist()
        print(f'[IK raw] ({x:.3f},{y:.3f},{z:.3f}) j7={math.degrees(result[6]):.1f}°', flush=True)
        if override_j7 is not None:
            result[6] = override_j7
        return result


# ─── State Machine Node ────────────────────────────────────────────────────────
class StateMachine(Node):
    def __init__(self):
        super().__init__('state_machine')
        self.ik = IKSolver(URDF_PATH)
        self._arm_client = ActionClient(
            self, FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory')
        self._gripper_client = ActionClient(
            self, GripperCommand,
            '/gripper_controller/gripper_cmd')
        self._block_pose = None
        self.create_subscription(PoseStamped, '/block_pose', self._pose_cb, 10)
        self._cycle = 0
        self._blocks_sorted = {}

    def _pose_cb(self, msg):
        self._block_pose = msg

    # ── Helpers ───────────────────────────────────────────────────────────────
    def _wait_pose(self, timeout=DETECT_TIMEOUT, ignore_colors=None):
        """Chờ detect block, bỏ qua màu đã xong và màu không có bin."""
        self._block_pose = None
        start = time.time()
        while time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._block_pose is not None:
                color = self._block_pose.header.frame_id
                if color not in BIN_POSITIONS:
                    self._block_pose = None
                    continue
                if ignore_colors and color in ignore_colors:
                    self._block_pose = None
                    continue
                return True
        return False

    def _move(self, joints, dur=3, label=''):
        if not self._arm_client.wait_for_server(timeout_sec=5.0):
            return False
        pt = JointTrajectoryPoint()
        pt.positions = joints
        pt.time_from_start = Duration(sec=dur)
        traj = JointTrajectory()
        traj.joint_names = JOINT_NAMES
        traj.points = [pt]
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        fut = self._arm_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut)
        gh = fut.result()
        if not gh.accepted:
            print(f'[{label}] rejected', flush=True)
            return False
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf)
        ok = rf.result().status == GoalStatus.STATUS_SUCCEEDED
        print(f'[{label}] {"✓" if ok else "✗"}', flush=True)
        return ok

    def _gripper(self, pos, effort=10.0, label=''):
        if not self._gripper_client.wait_for_server(timeout_sec=5.0):
            return False
        goal = GripperCommand.Goal()
        goal.command.position = pos
        goal.command.max_effort = effort
        fut = self._gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut)
        gh = fut.result()
        if not gh.accepted:
            return False
        rclpy.spin_until_future_complete(self, gh.get_result_async())
        return True

    def _ik(self, x, y, z, prev=None, fix_j7=True, label=''):
        seed = ([0] + prev + [0, 0, 0]) if prev else None
        j = self.ik.solve(x, y, z, seed=seed,
                          override_j7=JOINT7_GRASP if fix_j7 else None)
        print(f'[IK:{label}] ({x:.3f},{y:.3f},{z:.3f})', flush=True)
        return j

    def _find_next_block(self, completed_colors):
        """Quét hết SCAN_POSITIONS, thấy block chưa xong → trả về ngay."""
        print(f'\n[SCAN] Scanning... completed={completed_colors}', flush=True)
        for label, joints in SCAN_POSITIONS:
            print(f'[SCAN] → {label}', flush=True)
            self._move(joints, dur=2, label=label)
            if self._wait_pose(timeout=DETECT_TIMEOUT, ignore_colors=completed_colors):
                color = self._block_pose.header.frame_id
                print(f'[SCAN] ✓ Found [{color}] at {label}!', flush=True)
                return self._block_pose
        print('[SCAN] Full scan — no blocks found.', flush=True)
        self._move(HOME_JOINTS, dur=3, label='home')
        return None

    # ── Single pick-place cycle ────────────────────────────────────────────────
    def pick_and_place_one(self, bx, by, bz, color):
        bin_pos = BIN_POSITIONS[color]
        px, py, pz = bin_pos['x'], bin_pos['y'], bin_pos['z']

        print(f'\n--- Cycle {self._cycle}: [{color}] ({bx:.3f},{by:.3f}) → bin ---', flush=True)

        # Pre-grasp
        j_pre = self._ik(bx, by, bz + PREGRASP_Z_OFFSET, fix_j7=False, label='pre')
        if not self._move(j_pre, dur=3, label='pre-grasp'):
            return False

        # # Re-read pose sau khi arm đến gần — chính xác hơn
        # self._block_pose = None
        # t0 = time.time()
        # while self._block_pose is None and time.time() - t0 < 5.0:
        #     rclpy.spin_once(self, timeout_sec=0.1)
        # if self._block_pose is not None:
        #     bx = self._block_pose.pose.position.x
        #     by = self._block_pose.pose.position.y
        #     bz = self._block_pose.pose.position.z
        #     print(f'[RE-READ] ({bx:.3f},{by:.3f},{bz:.3f})', flush=True)

        # Grasp
        grasp_z = bz + BLOCK_SIZE_Z / 2.0 + GRASP_Z_OFFSET
        j_grasp = self._ik(bx, by, grasp_z, prev=j_pre, fix_j7=True, label='grasp')
        if not self._move(j_grasp, dur=2, label='grasp'):
            return False

        # Close gripper
        self._gripper(GRIPPER_CLOSE_POS, GRIPPER_CLOSE_EFFORT, label='close')

        # Lift
        j_lift = self._ik(bx, by, bz + LIFT_Z_OFFSET, prev=j_grasp, fix_j7=True, label='lift')
        if not self._move(j_lift, dur=2, label='lift'):
            return False

        # Above bin
        j_above = self._ik(px, py, pz + PREGRASP_Z_OFFSET, prev=j_lift, fix_j7=True, label='above-bin')
        if not self._move(j_above, dur=3, label='above-bin'):
            return False

        # Place
        j_place = self._ik(px, py, pz + PLACE_Z_OFFSET, prev=j_above, fix_j7=True, label='place')
        if not self._move(j_place, dur=2, label='place'):
            return False

        # Release
        self._gripper(GRIPPER_OPEN_POS, GRIPPER_OPEN_EFFORT, label='release')
        self._blocks_sorted[color] = self._blocks_sorted.get(color, 0) + 1
        return True

    # ── Main loop ─────────────────────────────────────────────────────────────
    def run(self):
        print('=== State Machine starting ===', flush=True)
        completed_colors = set()

        self._gripper(GRIPPER_OPEN_POS, GRIPPER_OPEN_EFFORT, label='init-open')

        while self._cycle < MAX_CYCLES:
            pose = self._find_next_block(completed_colors)

            if pose is None:
                print('\n=== No more blocks — DONE ===', flush=True)
                break

            color = pose.header.frame_id
            bx = pose.pose.position.x
            by = pose.pose.position.y
            bz = pose.pose.position.z
            self._cycle += 1

            ok = self.pick_and_place_one(bx, by, bz, color)

            print('\n[HOME] Returning...', flush=True)
            self._move(HOME_JOINTS, dur=3, label='home')

            if ok:
                completed_colors.add(color)
                print(f'[✓] {color} done. Completed: {completed_colors}', flush=True)
            else:
                print(f'[✗] Cycle {self._cycle} failed — will retry {color}', flush=True)

        print('\n=== State Machine DONE ===', flush=True)
        print(f'Cycles: {self._cycle}', flush=True)
        print(f'Sorted: {self._blocks_sorted}', flush=True)


def main():
    if not os.path.exists(URDF_PATH):
        print('Generating URDF...', flush=True)
        subprocess.run([
            'xacro',
            os.path.expanduser(
                '~/ros2_ws/src/simple_pick_and_place_description/urdf/robot/franka_pick_and_place.urdf.xacro')
        ], stdout=open(URDF_PATH, 'w'), check=True)

    rclpy.init()
    node = StateMachine()
    try:
        node.run()
    except KeyboardInterrupt:
        print('Interrupted', flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()