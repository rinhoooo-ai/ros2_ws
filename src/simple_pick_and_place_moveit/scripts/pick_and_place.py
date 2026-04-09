#!/usr/bin/env python3
"""
Challenge 4 — Pick and Place (IK version)
Fixes:
  1. Gripper xéo ~45°  → override fr3_joint7 = 0.0 sau mỗi IK solve
  2. Kẹp quá mức       → tính gap từ block size thật (5x5x10 cm)
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
import time

# ─── Robot config ──────────────────────────────────────────────────────────────
URDF_PATH   = '/tmp/fr3_full.urdf'
JOINT_NAMES = [
    'fr3_joint1', 'fr3_joint2', 'fr3_joint3',
    'fr3_joint4', 'fr3_joint5', 'fr3_joint6', 'fr3_joint7',
]
HOME_JOINTS = [0.0003, 0.0243, -0.0007, -0.1311, 0.0015, 0.4398, 0.0217]

IK_SEED = [0, 0.0, -0.3, 0.0, -2.8, 0.0, 2.5, 0.785, 0, 0, 0]
ACTIVE_MASK = [False, True, True, True, True, True, True, True, False, False, False]

# ─── Bin positions (world frame) ───────────────────────────────────────────────
BIN_POSITIONS = {
    'red':    {'x':  0.212, 'y': -0.212, 'z': 0.05},
    'yellow': {'x':  0.212, 'y':  0.212, 'z': 0.05},
    'blue':   {'x': -0.212, 'y':  0.212, 'z': 0.05},
    'green':  {'x': -0.212, 'y': -0.212, 'z': 0.05},
}

# ─── Block dimensions (từ model.sdf: 0.05 x 0.05 x 0.10) ─────────────────────
BLOCK_SIZE_X = 0.05
BLOCK_SIZE_Y = 0.05
BLOCK_SIZE_Z = 0.10

# ─── Gripper config ────────────────────────────────────────────────────────────
GRIPPER_OPEN_POS    = 0.040
GRIPPER_OPEN_EFFORT = 0.0

# Kẹp đúng: half-width + 1mm clearance
# Block 5cm → half = 0.025 → gap = 0.026 m mỗi ngón
GRIPPER_CLOSE_POS    = 0.020
GRIPPER_CLOSE_EFFORT = 15.0

# ─── Motion parameters ─────────────────────────────────────────────────────────
PREGRASP_Z_OFFSET = 0.20
GRASP_Z_OFFSET    = -0.02
LIFT_Z_OFFSET     = 0.28
PLACE_Z_OFFSET    = 0.18

# ─── Joint7 override ──────────────────────────────────────────────────────────
# IK seed j7=0.785 rad (45°) → gripper xéo
# Override = 0.0 → gripper align với trục X world
# Thử thêm: 1.5708 (90°) nếu vẫn xéo theo hướng khác
JOINT7_GRASP = 0.0


# ─── IK Solver ─────────────────────────────────────────────────────────────────
class IKSolver:
    def __init__(self, urdf_path: str):
        self.chain = ikpy.chain.Chain.from_urdf_file(
            urdf_path,
            base_elements=['fr3_link0'],
            base_element_type='link',
            active_links_mask=ACTIVE_MASK,
        )
        print('IK chain loaded:', [l.name for l in self.chain.links], flush=True)

    def solve(self, x: float, y: float, z: float,
              seed=None, override_j7: float = None) -> list:
        """
        override_j7: nếu set, thay joint7 sau IK solve.
        Dùng để fix gripper xéo — IK không constraint rotation quanh Z.
        """
        if seed is None:
            seed = IK_SEED
        joints = self.chain.inverse_kinematics(
            target_position=[x, y, z],
            target_orientation=[0, 0, -1],
            orientation_mode='Z',
            initial_position=seed,
        )
        result = joints[1:8].tolist()
        if override_j7 is not None:
            result[6] = override_j7   # fr3_joint7 = index 6
        return result


# ─── Main Node ─────────────────────────────────────────────────────────────────
class PickAndPlace(Node):
    def __init__(self):
        super().__init__('pick_and_place')
        self.ik = IKSolver(URDF_PATH)

        self._arm_client = ActionClient(
            self, FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory')
        self._gripper_client = ActionClient(
            self, GripperCommand,
            '/gripper_controller/gripper_cmd')

        self._block_pose: PoseStamped = None
        self.create_subscription(PoseStamped, '/block_pose', self._pose_cb, 10)

    def _pose_cb(self, msg: PoseStamped):
        self._block_pose = msg

    def wait_for_pose(self, timeout=15.0) -> bool:
        print('Waiting for /block_pose ...', flush=True)
        start = time.time()
        while self._block_pose is None and time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self._block_pose is None:
            print('ERROR: /block_pose timeout', flush=True)
            return False
        color = self._block_pose.header.frame_id
        bx = self._block_pose.pose.position.x
        by = self._block_pose.pose.position.y
        bz = self._block_pose.pose.position.z
        print(f'Block detected: color={color!r}  '
              f'x={bx:.3f} y={by:.3f} z={bz:.3f}', flush=True)
        return True

    def move_arm(self, joints: list, duration_sec=3, label='') -> bool:
        print(f'[{label}] joints: {[round(j, 3) for j in joints]}', flush=True)
        if not self._arm_client.wait_for_server(timeout_sec=5.0):
            print(f'[{label}] Arm server not available!', flush=True)
            return False
        point = JointTrajectoryPoint()
        point.positions       = joints
        point.time_from_start = Duration(sec=duration_sec)
        traj             = JointTrajectory()
        traj.joint_names = JOINT_NAMES
        traj.points      = [point]
        goal             = FollowJointTrajectory.Goal()
        goal.trajectory  = traj
        future = self._arm_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        gh = future.result()
        if not gh.accepted:
            print(f'[{label}] Goal rejected!', flush=True)
            return False
        result_future = gh.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        ok = result_future.result().status == GoalStatus.STATUS_SUCCEEDED
        print(f'[{label}] {"✓ OK" if ok else "✗ FAILED"}', flush=True)
        return ok

    def gripper(self, position: float, effort=10.0, label='') -> bool:
        print(f'[gripper:{label}] pos={position*100:.1f}cm  effort={effort:.1f}',
              flush=True)
        if not self._gripper_client.wait_for_server(timeout_sec=5.0):
            print(f'[gripper:{label}] Server not available!', flush=True)
            return False
        goal = GripperCommand.Goal()
        goal.command.position   = position
        goal.command.max_effort = effort
        future = self._gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        gh = future.result()
        if not gh.accepted:
            print(f'[gripper:{label}] Goal rejected!', flush=True)
            return False
        result_future = gh.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        print(f'[gripper:{label}] ✓ done', flush=True)
        time.sleep(0.5)
        return True

    def _ik(self, x, y, z, prev_joints=None, fix_j7=True, label='') -> list:
        """
        fix_j7=True  → override joint7 = JOINT7_GRASP (fix gripper xéo)
        fix_j7=False → giữ nguyên IK result (ví dụ khi về home)
        """
        seed = ([0] + prev_joints + [0, 0, 0]) if prev_joints else None
        j7   = JOINT7_GRASP if fix_j7 else None
        j    = self.ik.solve(x, y, z, seed=seed, override_j7=j7)
        print(f'[IK:{label}] ({x:.3f},{y:.3f},{z:.3f})  '
              f'j7={j[6]:.3f}rad → {[round(v,3) for v in j]}', flush=True)
        return j

    def run(self):
        # ── Read block pose ──────────────────────────────────────────────────────
        if not self.wait_for_pose():
            return

        color = self._block_pose.header.frame_id
        bx    = self._block_pose.pose.position.x
        by    = self._block_pose.pose.position.y
        bz    = self._block_pose.pose.position.z

        if color not in BIN_POSITIONS:
            print(f'Unknown color {color!r}, fallback to red', flush=True)
            color = 'red'

        bin_pos    = BIN_POSITIONS[color]
        px, py, pz = bin_pos['x'], bin_pos['y'], bin_pos['z']
        print(f'Target bin [{color}]: ({px:.3f}, {py:.3f}, {pz:.3f})', flush=True)
        print(f'Gripper close = {GRIPPER_CLOSE_POS*100:.1f} cm/finger '
              f'(block {BLOCK_SIZE_X*100:.0f}cm wide)', flush=True)

        # ════════════════════════════════════════════════════════════════════════
        print('\n=== Step 1: Home ===', flush=True)
        if not self.move_arm(HOME_JOINTS, duration_sec=3, label='home'):
            return

        print('\n=== Step 2: Open gripper ===', flush=True)
        self.gripper(GRIPPER_OPEN_POS, effort=GRIPPER_OPEN_EFFORT, label='open')

        # ── Pre-grasp ────────────────────────────────────────────────────────────
        print('\n=== Step 3: Pre-grasp (20cm above block) ===', flush=True)
        j_pre = self._ik(bx, by, bz + PREGRASP_Z_OFFSET,
                         fix_j7=True, label='pre-grasp')
        if not self.move_arm(j_pre, duration_sec=4, label='pre-grasp'):
            return

        # ── Re-read block pose ───────────────────────────────────────────────────
        print('\n   [Re-reading block pose from closer distance...]', flush=True)
        self._block_pose = None
        t0 = time.time()
        while self._block_pose is None and time.time() - t0 < 5.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self._block_pose is not None:
            bx = self._block_pose.pose.position.x
            by = self._block_pose.pose.position.y
            bz = self._block_pose.pose.position.z
            print(f'   Updated: ({bx:.3f}, {by:.3f}, {bz:.3f})', flush=True)

        # ── Grasp ────────────────────────────────────────────────────────────────
        # Tính từ tâm block: bz là tâm, top = bz + BLOCK_SIZE_Z/2
        # Gripper tip cần ở top + GRASP_Z_OFFSET
        print('\n=== Step 4: Grasp (2cm above block top) ===', flush=True)
        grasp_z = bz + BLOCK_SIZE_Z / 2.0 + GRASP_Z_OFFSET
        j_grasp = self._ik(bx, by, grasp_z,
                           prev_joints=j_pre, fix_j7=True, label='grasp')
        if not self.move_arm(j_grasp, duration_sec=3, label='grasp'):
            return

        # ── Close gripper ────────────────────────────────────────────────────────
        print('\n=== Step 5: Close gripper ===', flush=True)
        self.gripper(GRIPPER_CLOSE_POS, effort=GRIPPER_CLOSE_EFFORT, label='close')

        # ── Lift ─────────────────────────────────────────────────────────────────
        print('\n=== Step 6: Lift ===', flush=True)
        j_lift = self._ik(bx, by, bz + LIFT_Z_OFFSET,
                          prev_joints=j_grasp, fix_j7=True, label='lift')
        if not self.move_arm(j_lift, duration_sec=3, label='lift'):
            return

        # ── Above bin ────────────────────────────────────────────────────────────
        print(f'\n=== Step 7: Above [{color}] bin ===', flush=True)
        j_above = self._ik(px, py, pz + PREGRASP_Z_OFFSET,
                           prev_joints=j_lift, fix_j7=True, label='above-bin')
        if not self.move_arm(j_above, duration_sec=4, label='above-bin'):
            return

        # ── Lower into bin ────────────────────────────────────────────────────────
        print(f'\n=== Step 8: Place into [{color}] bin ===', flush=True)
        j_place = self._ik(px, py, pz + PLACE_Z_OFFSET,
                           prev_joints=j_above, fix_j7=True, label='place')
        if not self.move_arm(j_place, duration_sec=3, label='place'):
            return

        # ── Release ───────────────────────────────────────────────────────────────
        print('\n=== Step 9: Release ===', flush=True)
        self.gripper(GRIPPER_OPEN_POS, effort=GRIPPER_OPEN_EFFORT, label='release')

        # ── Home ──────────────────────────────────────────────────────────────────
        print('\n=== Step 10: Retreat home ===', flush=True)
        self.move_arm(HOME_JOINTS, duration_sec=3, label='home')

        print('\n=== ✓ Pick and place COMPLETE ===', flush=True)


def main():
    print('=== Pick and Place (IK) starting ===', flush=True)

    import subprocess, os
    if not os.path.exists('/tmp/fr3_full.urdf'):
        print('Generating URDF...', flush=True)
        subprocess.run([
            'xacro',
            os.path.expanduser('~/ros2_ws/src/simple_pick_and_place_description/urdf/robot/franka_pick_and_place.urdf.xacro')
        ], stdout=open('/tmp/fr3_full.urdf', 'w'), check=True)
        print('URDF generated ✓', flush=True)

    rclpy.init()
    node = PickAndPlace()
    try:
        node.run()
    except KeyboardInterrupt:
        print('Interrupted', flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()