#!/usr/bin/env python3
"""
================================================================================
PyBullet Trajectory Simulation and Torque Calculator
================================================================================
使用 PyBullet 物理引擎进行轨迹仿真，按照轨迹时间戳执行，计算关节扭矩。

支持功能：
  - 时间同步仿真：按照轨迹的时间戳执行运动
  - 实时模式：实时显示仿真过程（可选）
  - 速度缩放：调整轨迹执行速度（n倍）
  - 末端负载：添加末端执行器负载
  - 力矩曲线：输出执行过程中的力矩变化曲线
  - 运动过程：输出TCP位置、关节角度变化、运动轨迹
  - 3D可视化：可选的实时仿真可视化

使用方法:
  python3 scripts/simulate_trajectory_pybullet.py --trajectory 35/3.csv --arm right --speed 1.5 --payload 2.0 --plot

参数:
  --trajectory: 轨迹 CSV 文件路径
  --arm: 手臂选择 (left 或 right)
  --speed: 速度缩放因子 (默认 1.0，例如 2.0 表示速度翻倍)
  --payload: 末端负载质量 kg (默认 0.0)
  --payload-com: 末端负载质心距离 m (默认 0.1)
  --realtime: 实时仿真模式（按真实时间执行）
  --visualize: 启用 PyBullet GUI 可视化
  --plot: 生成力矩曲线图
  --output: 输出报告路径
================================================================================
"""

import argparse
import csv
import numpy as np
from pathlib import Path
import os
import json
from datetime import datetime
import math
import tempfile
import shutil
import time


def create_pybullet_urdf(arm='right', output_dir=None):
    """
    Create a simplified URDF file for PyBullet simulation.
    PyBullet needs actual file paths, not package:// URLs.
    """
    if output_dir is None:
        output_dir = tempfile.mkdtemp()

    # Find the original URDF
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(script_dir)

    original_urdf = os.path.join(project_root, f'src/{arm}_arm_description/urdf/{arm}_arm_description.urdf')

    if not os.path.exists(original_urdf):
        # Try install directory
        original_urdf = os.path.join(project_root, f'install/{arm}_arm_description/share/{arm}_arm_description/urdf/{arm}_arm_description.urdf')

    if not os.path.exists(original_urdf):
        raise FileNotFoundError(f"Could not find URDF for {arm} arm")

    # Read original URDF
    with open(original_urdf, 'r') as f:
        urdf_content = f.read()

    # Find mesh directory
    mesh_search_paths = [
        os.path.join(project_root, f'src/{arm}_arm_description/meshes'),
        os.path.join(project_root, f'install/{arm}_arm_description/share/{arm}_arm_description/meshes'),
    ]

    mesh_dir = None
    for path in mesh_search_paths:
        if os.path.exists(path):
            mesh_dir = path
            break

    # Replace package:// URLs with actual paths
    if mesh_dir:
        urdf_content = urdf_content.replace(
            f'package://{arm}_arm_description/meshes/',
            f'{mesh_dir}/'
        )

    # Write modified URDF
    output_urdf = os.path.join(output_dir, f'{arm}_arm_pybullet.urdf')
    with open(output_urdf, 'w') as f:
        f.write(urdf_content)

    return output_urdf, output_dir


class PyBulletTrajectorySimulator:
    """
    PyBullet-based robot trajectory simulator.
    Executes trajectory according to timestamps and calculates torques.
    """

    def __init__(self, arm='right', payload_mass=0.0, payload_com_distance=0.1, visualize=False, realtime=False):
        self.arm = arm
        self.payload_mass = payload_mass
        self.payload_com_distance = payload_com_distance
        self.visualize = visualize
        self.realtime = realtime

        # Import pybullet
        try:
            import pybullet as p
            import pybullet_data
            self.p = p
            self.pybullet_data = pybullet_data
        except ImportError:
            raise ImportError("PyBullet not installed. Install with: pip install pybullet")

        # Joint names for the arm
        self.joint_names = [
            f'{arm}_shoulder_pitch_joint',
            f'{arm}_shoulder_roll_joint',
            f'{arm}_shoulder_yaw_joint',
            f'{arm}_elbow_joint',
            f'{arm}_wrist_roll_joint',
            f'{arm}_wrist_yaw_joint',
            f'{arm}_wrist_pitch_joint',
        ]

        # Effort limits from URDF
        self.effort_limits = {
            f'{arm}_shoulder_pitch_joint': 100.0,
            f'{arm}_shoulder_roll_joint': 100.0,
            f'{arm}_shoulder_yaw_joint': 65.0,
            f'{arm}_elbow_joint': 65.0,
            f'{arm}_wrist_roll_joint': 30.0,
            f'{arm}_wrist_yaw_joint': 30.0,
            f'{arm}_wrist_pitch_joint': 30.0,
        }

        # Initialize simulation
        self.physics_client = None
        self.robot_id = None
        self.payload_id = None
        self.temp_dir = None
        self.joint_indices = []
        self.num_joints = 0

        # Simulation parameters
        self.time_step = 1.0 / 1000.0  # 1ms timestep
        self.control_mode = self.p.POSITION_CONTROL if hasattr(self.p, 'POSITION_CONTROL') else 2

        # Payload tracking
        self.payload_added = False
        self.effective_payload_mass = 0.0
        self.effective_payload_com = 0.0

    def initialize(self):
        """Initialize PyBullet simulation environment."""
        # Connect to physics server
        if self.visualize:
            self.physics_client = self.p.connect(self.p.GUI)
            # Configure visualization
            self.p.configureDebugVisualizer(self.p.COV_ENABLE_GUI, 0)
            self.p.configureDebugVisualizer(self.p.COV_ENABLE_SHADOWS, 1)
        else:
            self.physics_client = self.p.connect(self.p.DIRECT)

        # Set additional search path for pybullet data
        self.p.setAdditionalSearchPath(self.pybullet_data.getDataPath())

        # Configure simulation
        self.p.setGravity(0, 0, -9.81)
        self.p.setTimeStep(self.time_step)

        # Load robot URDF
        urdf_path, self.temp_dir = create_pybullet_urdf(self.arm)

        try:
            # Load URDF with fixed base
            self.robot_id = self.p.loadURDF(
                urdf_path,
                basePosition=[0, 0, 0],
                baseOrientation=self.p.getQuaternionFromEuler([0, 0, 0]),
                useFixedBase=True
            )
            print(f"Loaded robot URDF: {urdf_path}")
        except Exception as e:
            print(f"Error loading URDF: {e}")
            raise

        # Get joint information
        self.num_joints = self.p.getNumJoints(self.robot_id)
        self.joint_indices = []

        print(f"Robot has {self.num_joints} joints")

        for i in range(self.num_joints):
            joint_info = self.p.getJointInfo(self.robot_id, i)
            joint_name = joint_info[1].decode('utf-8')
            joint_type = joint_info[2]

            # Only consider revolute and prismatic joints
            if joint_type in [self.p.JOINT_REVOLUTE, self.p.JOINT_PRISMATIC]:
                self.joint_indices.append(i)
                print(f"  Joint {i}: {joint_name} (type: {joint_type})")

        print(f"Total controllable joints: {len(self.joint_indices)}")

        # Add payload if specified
        if self.payload_mass > 0:
            self._add_payload()

        return True

    def _add_payload(self):
        """Add payload by modifying the dynamics of the last link."""
        if not self.joint_indices:
            return

        # Get the last link (end effector)
        last_joint_idx = self.joint_indices[-1]

        # Get current dynamics of the last link
        dynamics_info = self.p.getDynamicsInfo(self.robot_id, last_joint_idx)
        current_mass = dynamics_info[0]
        current_inertia = dynamics_info[2]  # local inertia diagonal

        print(f"  Original end-effector mass: {current_mass:.4f} kg")
        print(f"  Original end-effector inertia: {current_inertia}")

        # Calculate new mass and inertia with payload
        # The payload adds mass at a distance from the joint
        new_mass = current_mass + self.payload_mass

        # Calculate new inertia using parallel axis theorem
        # I_new = I_original + m_payload * d^2
        # where d is the distance from joint to payload CoM
        payload_inertia_contribution = self.payload_mass * (self.payload_com_distance ** 2)

        new_inertia = [
            current_inertia[0] + payload_inertia_contribution,
            current_inertia[1] + payload_inertia_contribution,
            current_inertia[2] + payload_inertia_contribution
        ]

        # Change the dynamics of the last link (mass and inertia)
        self.p.changeDynamics(
            self.robot_id,
            last_joint_idx,
            mass=new_mass,
            localInertiaDiagonal=new_inertia
        )

        # Verify the change
        new_dynamics = self.p.getDynamicsInfo(self.robot_id, last_joint_idx)
        print(f"  Added payload: {self.payload_mass} kg at {self.payload_com_distance} m from end-effector")
        print(f"  New end-effector mass: {new_dynamics[0]:.4f} kg")
        print(f"  New end-effector inertia: {new_dynamics[2]}")

        # Store payload info for reporting
        self.payload_added = True
        self.effective_payload_mass = self.payload_mass
        self.effective_payload_com = self.payload_com_distance

    def set_joint_positions(self, positions):
        """Set joint positions directly (for initialization)."""
        for i, joint_idx in enumerate(self.joint_indices):
            if i < len(positions):
                self.p.resetJointState(
                    self.robot_id,
                    joint_idx,
                    positions[i]
                )

    def get_joint_states(self):
        """Get current joint states."""
        states = []
        for joint_idx in self.joint_indices:
            joint_state = self.p.getJointState(self.robot_id, joint_idx)
            states.append({
                'position': joint_state[0],
                'velocity': joint_state[1],
                'reaction_forces': joint_state[2],
                'applied_torque': joint_state[3]
            })
        return states

    def get_joint_positions(self):
        """Get current joint positions."""
        positions = []
        for joint_idx in self.joint_indices:
            joint_state = self.p.getJointState(self.robot_id, joint_idx)
            positions.append(joint_state[0])
        return positions

    def get_joint_velocities(self):
        """Get current joint velocities."""
        velocities = []
        for joint_idx in self.joint_indices:
            joint_state = self.p.getJointState(self.robot_id, joint_idx)
            velocities.append(joint_state[1])
        return velocities

    def calculate_inverse_dynamics(self, positions, velocities, accelerations):
        """
        Calculate joint torques using PyBullet's inverse dynamics.

        Args:
            positions: Joint positions (rad)
            velocities: Joint velocities (rad/s)
            accelerations: Joint accelerations (rad/s^2)

        Returns:
            Joint torques (Nm)
        """
        if len(self.joint_indices) == 0:
            return np.zeros(7)

        # Use PyBullet's inverse dynamics
        torques = self.p.calculateInverseDynamics(
            self.robot_id,
            list(positions[:len(self.joint_indices)]),
            list(velocities[:len(self.joint_indices)]),
            list(accelerations[:len(self.joint_indices)])
        )

        # Ensure we return 7 values
        result = np.zeros(7)
        for i, t in enumerate(torques):
            if i < 7:
                result[i] = t
        return result

    def get_link_state(self, link_index):
        """Get state of a specific link."""
        if link_index < self.num_joints:
            return self.p.getLinkState(self.robot_id, link_index)
        return None

    def step_simulation(self, num_steps=1):
        """Step the simulation forward."""
        for _ in range(num_steps):
            self.p.stepSimulation()

    def run_trajectory(self, times, positions, velocities, accelerations, speed_scale=1.0, progress_callback=None):
        """
        Execute trajectory according to timestamps.

        Args:
            times: List of trajectory times (seconds)
            positions: List of joint positions for each point
            velocities: List of joint velocities for each point
            accelerations: List of joint accelerations for each point
            speed_scale: Speed scaling factor
            progress_callback: Optional callback function for progress updates

        Returns:
            Dictionary with simulation results
        """
        print(f"\n{'='*60}")
        print("Starting trajectory simulation")
        print(f"{'='*60}")
        print(f"  Total trajectory points: {len(times)}")
        print(f"  Trajectory duration: {times[-1]:.2f} seconds")
        print(f"  Speed scale: {speed_scale}x")
        print(f"  Realtime mode: {self.realtime}")
        print(f"  Starting position: trajectory point 0 (time=0.0s)")
        print(f"{'='*60}\n")

        # Initialize robot at first position
        # This is the starting point of the trajectory
        self.set_joint_positions(positions[0])

        # Results storage
        torque_history = {name: [] for name in self.joint_names}
        position_history = {name: [] for name in self.joint_names}
        velocity_history = {name: [] for name in self.joint_names}
        acceleration_history = {name: [] for name in self.joint_names}
        tcp_history = []

        max_torques = {name: 0.0 for name in self.joint_names}

        # Simulation timing
        sim_time = 0.0
        trajectory_idx = 0
        sim_start_time = time.time() if self.realtime else None

        # Use position control for smooth motion
        # Set joint control mode
        for joint_idx in self.joint_indices:
            self.p.setJointMotorControl2(
                self.robot_id,
                joint_idx,
                self.p.VELOCITY_CONTROL,
                force=0  # Disable default motor to use torque control
            )

        print("Executing trajectory...")

        while trajectory_idx < len(times):
            # Get current trajectory point
            target_time = times[trajectory_idx] / speed_scale
            target_positions = positions[trajectory_idx]

            # Check if we need to move to this point
            if sim_time >= target_time:
                # Calculate torques at this point
                current_positions = self.get_joint_positions()
                current_velocities = self.get_joint_velocities()

                # Get the velocity and acceleration for this trajectory point
                target_velocities = velocities[trajectory_idx]
                target_accelerations = accelerations[trajectory_idx]

                # Calculate required torques using inverse dynamics
                torques = self.calculate_inverse_dynamics(
                    target_positions,
                    target_velocities,
                    target_accelerations
                )

                # Store torque data (signed torque)
                for j, name in enumerate(self.joint_names):
                    torque_val = float(torques[j])  # Keep sign for proper physics
                    torque_history[name].append({
                        'time': sim_time,
                        'torque': torque_val
                    })
                    # Track max absolute torque for safety check
                    if abs(torque_val) > max_torques[name]:
                        max_torques[name] = abs(torque_val)

                # Store acceleration data
                for j, name in enumerate(self.joint_names):
                    acceleration_history[name].append({
                        'time': sim_time,
                        'acceleration': float(target_accelerations[j]) if j < len(target_accelerations) else 0.0
                    })

                # Store velocity data
                for j, name in enumerate(self.joint_names):
                    velocity_history[name].append({
                        'time': sim_time,
                        'velocity': float(target_velocities[j]) if j < len(target_velocities) else 0.0
                    })

                # Store position data
                for j, name in enumerate(self.joint_names):
                    position_history[name].append({
                        'time': sim_time,
                        'position': float(current_positions[j]) if j < len(current_positions) else 0.0
                    })

                # Calculate TCP position (simplified)
                tcp_pos = self._calculate_tcp_position()
                tcp_pos['time'] = sim_time
                tcp_history.append(tcp_pos)

                # Progress update
                if progress_callback:
                    progress_callback(trajectory_idx, len(times))

                if (trajectory_idx + 1) % 20 == 0 or trajectory_idx == len(times) - 1:
                    progress = (trajectory_idx + 1) / len(times) * 100
                    print(f"  Progress: {progress:.1f}% ({trajectory_idx + 1}/{len(times)} points, t={sim_time:.2f}s)")

                trajectory_idx += 1

            # Apply position control to move toward target
            if trajectory_idx < len(times):
                next_positions = positions[trajectory_idx]
                for j, joint_idx in enumerate(self.joint_indices):
                    if j < len(next_positions):
                        # Use position control
                        self.p.setJointMotorControl2(
                            self.robot_id,
                            joint_idx,
                            self.p.POSITION_CONTROL,
                            targetPosition=next_positions[j],
                            force=self.effort_limits[self.joint_names[j]],
                            maxVelocity=2.0  # Limit velocity
                        )

            # Step simulation
            self.step_simulation(1)
            sim_time += self.time_step

            # Realtime sync
            if self.realtime:
                elapsed = time.time() - sim_start_time
                if sim_time > elapsed:
                    time.sleep(sim_time - elapsed)

        print("\nTrajectory execution completed!")
        print(f"  Simulation time: {sim_time:.2f} seconds")
        print(f"  Real elapsed time: {time.time() - sim_start_time:.2f} seconds" if self.realtime else "")

        return {
            'max_torques': max_torques,
            'torque_history': torque_history,
            'position_history': position_history,
            'velocity_history': velocity_history,
            'acceleration_history': acceleration_history,
            'tcp_history': tcp_history,
            'simulation_time': sim_time
        }

    def _calculate_tcp_position(self):
        """
        Calculate TCP position using PyBullet's forward kinematics.

        Returns:
            Dictionary with x, y, z coordinates
        """
        # Get the end effector link index (last joint's child link)
        if not self.joint_indices:
            return {'x': 0.0, 'y': 0.0, 'z': 0.0}

        # Get the last link index (end effector)
        last_joint_idx = self.joint_indices[-1]
        joint_info = self.p.getJointInfo(self.robot_id, last_joint_idx)
        end_effector_link = joint_info[12]  # child link index after this joint

        # Ensure end_effector_link is an integer
        if isinstance(end_effector_link, bytes):
            end_effector_link = int.from_bytes(end_effector_link, 'little')

        # If end_effector_link is -1, use the last joint index
        if end_effector_link == -1 or end_effector_link < 0:
            end_effector_link = self.num_joints - 1

        # Get link state from PyBullet (accurate FK)
        try:
            link_state = self.p.getLinkState(self.robot_id, int(end_effector_link))
            # link_state[0] is world position of link frame
            # link_state[4] is world position of CoM
            world_pos = link_state[4]  # Use CoM position as TCP
            return {
                'x': float(world_pos[0]),
                'y': float(world_pos[1]),
                'z': float(world_pos[2])
            }
        except Exception as e:
            print(f"Warning: Could not get link state: {e}")
            return {'x': 0.0, 'y': 0.0, 'z': 0.0}

    def cleanup(self):
        """Clean up PyBullet simulation."""
        if self.physics_client is not None:
            self.p.disconnect(self.physics_client)
            self.physics_client = None

        # Clean up temporary directory
        if self.temp_dir and os.path.exists(self.temp_dir):
            try:
                shutil.rmtree(self.temp_dir)
            except (OSError, PermissionError) as e:
                print(f"Warning: Could not remove temp directory {self.temp_dir}: {e}")


def load_trajectory(csv_path, arm='right', speed_scale=1.0):
    """Load trajectory from CSV file with speed scaling."""
    times = []
    positions = []
    velocities = []
    accelerations = []

    with open(csv_path, 'r') as f:
        reader = csv.DictReader(f)
        rows = list(reader)

        for i, row in enumerate(rows):
            original_time = float(row['time'])
            times.append(original_time)
            pos = [
                float(row[f'{arm}_shoulder_pitch_joint']),
                float(row[f'{arm}_shoulder_roll_joint']),
                float(row[f'{arm}_shoulder_yaw_joint']),
                float(row[f'{arm}_elbow_joint']),
                float(row[f'{arm}_wrist_roll_joint']),
                float(row[f'{arm}_wrist_yaw_joint']),
                float(row[f'{arm}_wrist_pitch_joint']),
            ]
            positions.append(pos)

    # Normalize times to start from 0
    if times and times[0] != 0:
        time_offset = times[0]
        print(f"Note: Normalizing trajectory time offset (first point time was {time_offset:.3f}s)")
        times = [t - time_offset for t in times]

    # Apply speed scaling
    if speed_scale != 1.0:
        times = [t / speed_scale for t in times]

    # Calculate velocities and accelerations using finite differences
    # Assume robot starts from rest (velocity = 0, acceleration = 0 at the first point)
    for i in range(len(times)):
        if i == 0:
            # First point: robot is at rest, starting to move
            # Assume initial velocity = 0, acceleration = 0 (smooth start)
            vel = [0.0] * 7
            acc = [0.0] * 7
        elif i == 1:
            # Second point: use forward difference for velocity
            dt = times[1] - times[0]
            if dt > 0:
                vel = [(positions[1][j] - positions[0][j]) / dt for j in range(7)]
                # Acceleration: (vel - 0) / dt (assuming starting from rest)
                acc = [vel[j] / dt for j in range(7)]
            else:
                vel = [0.0] * 7
                acc = [0.0] * 7
        elif i == len(times) - 1:
            # Last point: use backward difference
            dt = times[i] - times[i-1]
            if dt > 0:
                vel = [(positions[i][j] - positions[i-1][j]) / dt for j in range(7)]
            else:
                vel = velocities[-1] if velocities else [0.0] * 7
            prev_vel = velocities[-1] if velocities else [0.0] * 7
            acc = [(vel[j] - prev_vel[j]) / dt for j in range(7)] if dt > 0 else [0.0] * 7
        else:
            # Middle points: use central difference for both velocity and acceleration
            # This provides better accuracy and consistency
            dt_center = times[i+1] - times[i-1]
            if dt_center > 0:
                # Central difference for velocity: (x[i+1] - x[i-1]) / (2*dt)
                vel = [(positions[i+1][j] - positions[i-1][j]) / dt_center for j in range(7)]
            else:
                vel = [0.0] * 7

            # Central difference for acceleration: (x[i+1] - 2*x[i] + x[i-1]) / dt^2
            dt = times[i] - times[i-1]
            next_dt = times[i+1] - times[i]
            avg_dt = (dt + next_dt) / 2 if (dt + next_dt) > 0 else dt
            if avg_dt > 0:
                acc = [(positions[i+1][j] - 2*positions[i][j] + positions[i-1][j]) / (avg_dt * avg_dt) for j in range(7)]
            else:
                acc = [0.0] * 7

        velocities.append(vel)
        accelerations.append(acc)

    # Check if trajectory has significant initial velocity
    # This would indicate the robot is not starting from rest
    if len(velocities) > 1:
        initial_vel = velocities[1]  # First calculated velocity (at point 1)
        max_initial_vel = max(abs(v) for v in initial_vel)
        if max_initial_vel > 0.1:  # threshold: 0.1 rad/s
            print(f"Warning: Trajectory appears to have initial velocity (max: {max_initial_vel:.3f} rad/s)")
            print("         This simulation assumes robot starts from rest at the first point.")
            print("         Torque calculations at the beginning may not be accurate.")

    return times, positions, velocities, accelerations


def print_torque_report(max_torques, effort_limits, arm='right', speed_scale=1.0, payload_mass=0.0):
    """Print torque report to console."""
    print("\n" + "=" * 80)
    print(f"PyBullet Simulation Results - {arm.upper()} Arm")
    print(f"Speed: {speed_scale}x | Payload: {payload_mass} kg")
    print("=" * 80)
    print(f"{'Joint Name':<40} {'Max Torque':>12} {'Limit':>10} {'Usage':>10} {'Status':<12}")
    print("-" * 80)

    max_usage = 0.0
    for joint_name, max_torque in max_torques.items():
        limit = effort_limits.get(joint_name, 100.0)
        usage = (max_torque / limit * 100) if limit > 0 else 0.0
        max_usage = max(max_usage, usage)

        if usage > 90:
            status = "CRITICAL"
        elif usage > 70:
            status = "WARNING"
        elif usage > 50:
            status = "MODERATE"
        else:
            status = "OK"

        print(f"{joint_name:<40} {max_torque:>10.2f} Nm {limit:>8.1f} Nm {usage:>8.1f}% {status}")

    print("-" * 80)
    print(f"Maximum usage across all joints: {max_usage:.1f}%")
    print("=" * 80)

    return max_usage


def generate_torque_plot(torque_history, velocity_history, acceleration_history, position_history,
                         joint_names, effort_limits, arm, speed_scale, payload_mass, output_dir):
    """
    Generate joint curves plot image using plotly (HTML) and PIL (PNG).
    Two images: 3 shoulder joints + 4 elbow/wrist joints.
    Layout: N columns (joints), 4 rows (torque, acceleration, velocity, position)
    """
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')

    # Split joints: first 3 (shoulder) and last 4 (elbow + wrist)
    shoulder_joints = joint_names[:3]
    elbow_wrist_joints = joint_names[3:]

    # Generate HTML with plotly (interactive) - single file with both figures
    html_paths = []
    try:
        import plotly.graph_objects as go
        from plotly.subplots import make_subplots

        for fig_idx, (joint_subset, name_suffix) in enumerate([
            (shoulder_joints, 'shoulder'),
            (elbow_wrist_joints, 'elbow_wrist')
        ]):
            if not joint_subset:
                continue

            html_path = os.path.join(output_dir, f'joint_curves_{arm}_{name_suffix}_{timestamp}.html')
            num_cols = len(joint_subset)

            row_labels = ['Torque (Nm)', 'Accel (rad/s²)', 'Vel (rad/s)', 'Pos (rad)']
            subplot_titles = []
            for joint_name in joint_subset:
                short_name = joint_name.replace(f'{arm}_', '').replace('_joint', '').replace('_', ' ').title()
                subplot_titles.append(short_name)

            fig = make_subplots(
                rows=4, cols=num_cols,
                subplot_titles=subplot_titles,
                vertical_spacing=0.08,
                horizontal_spacing=0.1
            )

            colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b', '#e377c2']

            for col, joint_name in enumerate(joint_subset):
                torque_data = torque_history.get(joint_name, [])
                vel_data = velocity_history.get(joint_name, [])
                accel_data = acceleration_history.get(joint_name, [])
                pos_data = position_history.get(joint_name, [])

                if not torque_data:
                    continue

                t_values = [d['time'] for d in torque_data]
                torque_values = [d['torque'] for d in torque_data]
                vel_values = [d['velocity'] for d in vel_data] if vel_data else []
                accel_values = [d['acceleration'] for d in accel_data] if accel_data else []
                pos_values = [d['position'] for d in pos_data] if pos_data else []

                limit = effort_limits.get(joint_name, 100.0)

                # Row 1: Torque
                fig.add_trace(
                    go.Scatter(x=t_values, y=torque_values, mode='lines',
                              line=dict(color=colors[col % len(colors)], width=2),
                              showlegend=False),
                    row=1, col=col+1
                )
                max_abs_torque = max(abs(t) for t in torque_values) if torque_values else 0
                y_range_t = max(limit * 1.2, max_abs_torque * 1.3)
                fig.add_trace(
                    go.Scatter(x=[t_values[0], t_values[-1]], y=[limit, limit],
                              mode='lines', line=dict(color='red', dash='dash', width=1),
                              showlegend=False),
                    row=1, col=col+1
                )
                fig.add_trace(
                    go.Scatter(x=[t_values[0], t_values[-1]], y=[-limit, -limit],
                              mode='lines', line=dict(color='red', dash='dash', width=1),
                              showlegend=False),
                    row=1, col=col+1
                )
                fig.update_yaxes(title_text='Nm', row=1, col=col+1, range=[-y_range_t, y_range_t])

                # Row 2: Acceleration
                if accel_values:
                    fig.add_trace(
                        go.Scatter(x=t_values, y=accel_values, mode='lines',
                                  line=dict(color='green', width=2),
                                  showlegend=False),
                        row=2, col=col+1
                    )
                    max_accel = max(abs(a) for a in accel_values) if accel_values else 1
                    fig.update_yaxes(title_text='rad/s²', row=2, col=col+1, range=[-max_accel*1.2, max_accel*1.2])

                # Row 3: Velocity
                if vel_values:
                    fig.add_trace(
                        go.Scatter(x=t_values, y=vel_values, mode='lines',
                                  line=dict(color='orange', width=2),
                                  showlegend=False),
                        row=3, col=col+1
                    )
                    max_vel = max(abs(v) for v in vel_values) if vel_values else 1
                    fig.update_yaxes(title_text='rad/s', row=3, col=col+1, range=[-max_vel*1.2, max_vel*1.2])

                # Row 4: Position
                if pos_values:
                    fig.add_trace(
                        go.Scatter(x=t_values, y=pos_values, mode='lines',
                                  line=dict(color='purple', width=2),
                                  showlegend=False),
                        row=4, col=col+1
                    )
                    min_pos = min(pos_values) if pos_values else 0
                    max_pos = max(pos_values) if pos_values else 1
                    padding = (max_pos - min_pos) * 0.1 if max_pos != min_pos else 0.5
                    fig.update_yaxes(title_text='rad', row=4, col=col+1, range=[min_pos-padding, max_pos+padding])

                fig.update_xaxes(title_text='Time (s)', row=4, col=col+1)

            fig.update_layout(
                title=dict(text=f'Joint Curves ({name_suffix.replace("_", " ").title()}) - {arm.upper()} Arm | Speed: {speed_scale}x | Payload: {payload_mass} kg',
                           font=dict(size=14)),
                height=700, width=200 + num_cols * 250,
                showlegend=False
            )

            fig.write_html(html_path)
            html_paths.append(html_path)
            print(f"Joint curves HTML saved to: {html_path}")

    except ImportError:
        print("Warning: plotly not installed. Install with: pip install plotly")

    # Generate PNG using PIL - two separate images
    png_paths = []
    try:
        from PIL import Image, ImageDraw, ImageFont

        for fig_idx, (joint_subset, name_suffix) in enumerate([
            (shoulder_joints, 'shoulder'),
            (elbow_wrist_joints, 'elbow_wrist')
        ]):
            if not joint_subset:
                continue

            num_cols = len(joint_subset)

            # Try to use a default font
            try:
                font_title = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 14)
                font_label = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 11)
                font_tick = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 9)
            except (OSError, IOError):
                font_title = ImageFont.load_default()
                font_label = ImageFont.load_default()
                font_tick = ImageFont.load_default()

            # Layout parameters - larger margins for tick labels
            margin_top = 45
            margin_left = 50  # Space for row labels on left
            margin_right = 20
            margin_bottom = 50  # Space for X-axis labels

            # Each column: Y-axis tick labels (~40px) + plot area (~200px)
            tick_label_width = 40
            plot_width = 200
            col_width = tick_label_width + plot_width
            total_plot_width = col_width * num_cols
            width = margin_left + total_plot_width + margin_right
            height = 720
            row_height = (height - margin_top - margin_bottom) // 4

            # Create image with calculated dimensions
            img = Image.new('RGB', (width, height), color=(255, 255, 255))
            draw = ImageDraw.Draw(img)

            # Title
            title = f'Joint Curves ({name_suffix.replace("_", " ").title()}) - {arm.upper()} Arm (Speed: {speed_scale}x, Payload: {payload_mass} kg)'
            draw.text((width//2, 12), title, fill='black', font=font_title, anchor='mt')

            colors_torque = [(31, 119, 180), (255, 127, 14), (44, 160, 44), (214, 39, 40),
                             (148, 103, 189), (140, 86, 75), (227, 119, 194)]
            color_accel = (34, 139, 34)
            color_vel = (255, 140, 0)
            color_pos = (128, 0, 128)

            row_types = [
                ('Torque', 'Nm', colors_torque),
                ('Accel', 'rad/s²', color_accel),
                ('Vel', 'rad/s', color_vel),
                ('Pos', 'rad', color_pos)
            ]

            # Draw column headers (joint names) - center over plot area
            for col, joint_name in enumerate(joint_subset):
                short_name = joint_name.replace(f'{arm}_', '').replace('_joint', '').replace('_', ' ').title()
                # Center over the plot area (not including tick labels)
                x_center = margin_left + col * col_width + tick_label_width + plot_width // 2
                draw.text((x_center, margin_top - 18), short_name, fill='black', font=font_label, anchor='mt')

            # Draw row labels on the left
            for row, (row_name, unit, _) in enumerate(row_types):
                y_center = margin_top + row * row_height + row_height // 2
                draw.text((15, y_center - 6), row_name, fill='black', font=font_label)
                draw.text((15, y_center + 6), f'({unit})', fill='gray', font=font_tick)

            # Draw each subplot
            for col, joint_name in enumerate(joint_subset):
                torque_data = torque_history.get(joint_name, [])
                vel_data = velocity_history.get(joint_name, [])
                accel_data = acceleration_history.get(joint_name, [])
                pos_data = position_history.get(joint_name, [])

                if not torque_data:
                    continue

                t_values = [d['time'] for d in torque_data]
                torque_values = [d['torque'] for d in torque_data]
                vel_values = [d['velocity'] for d in vel_data] if vel_data else []
                accel_values = [d['acceleration'] for d in accel_data] if accel_data else []
                pos_values = [d['position'] for d in pos_data] if pos_data else []

                limit = effort_limits.get(joint_name, 100.0)
                t_max = max(t_values) if t_values else 1

                # Data for each row
                all_data = [
                    (torque_values, 'torque', limit),
                    (accel_values, 'accel', None),
                    (vel_values, 'vel', None),
                    (pos_values, 'pos', None)
                ]

                for row, (data, data_type, torque_limit) in enumerate(all_data):
                    # Each subplot: tick_label_width (left) + plot area (right)
                    x0_tick = margin_left + col * col_width + 2
                    x0_plot = x0_tick + tick_label_width - 5
                    y0 = margin_top + row * row_height + 2
                    x1 = x0_tick + col_width - 4
                    y1 = y0 + row_height - 4

                    if not data:
                        continue

                    # Draw border around plot area only
                    draw.rectangle([x0_plot, y0, x1, y1], outline=(100, 100, 100), width=1)

                    # Calculate y range
                    if data_type == 'torque':
                        max_abs = max(abs(t) for t in data)
                        y_max = max(limit * 1.2, max_abs * 1.3) if max_abs > 0 else limit
                        y_min = -y_max
                        y_symmetric = True
                    elif data_type == 'accel':
                        max_abs = max(abs(a) for a in data) if data else 1
                        y_max = max_abs * 1.2 if max_abs > 0 else 1
                        y_min = -y_max
                        y_symmetric = True
                    elif data_type == 'vel':
                        max_abs = max(abs(v) for v in data) if data else 1
                        y_max = max_abs * 1.2 if max_abs > 0 else 1
                        y_min = -y_max
                        y_symmetric = True
                    else:  # position
                        y_min = min(data) if data else 0
                        y_max = max(data) if data else 1
                        padding = (y_max - y_min) * 0.1 if y_max != y_min else 0.5
                        y_min -= padding
                        y_max += padding
                        y_symmetric = False

                    y_range = y_max - y_min

                    # Draw grid lines and Y-axis tick labels (every subplot)
                    num_y_ticks = 5
                    for tick in range(num_y_ticks + 1):
                        y_val = y_min + (y_range * tick / num_y_ticks)
                        y_pos = y1 - 5 - int((y_val - y_min) / y_range * (y1 - y0 - 10))
                        # Grid line (in plot area)
                        draw.line([(x0_plot + 3, y_pos), (x1 - 3, y_pos)], fill=(220, 220, 220), width=1)
                        # Tick label (in tick area on left)
                        if data_type == 'torque':
                            label = f'{y_val:.0f}'
                        elif data_type in ['accel', 'vel']:
                            label = f'{y_val:.1f}'
                        else:
                            label = f'{y_val:.2f}'
                        draw.text((x0_plot - 3, y_pos - 4), label, fill='gray', font=font_tick, anchor='rm')

                    # Draw zero line for symmetric plots
                    if y_symmetric and y_min < 0 < y_max:
                        y_zero = y1 - 5 - int((0 - y_min) / y_range * (y1 - y0 - 10))
                        draw.line([(x0_plot + 3, y_zero), (x1 - 3, y_zero)], fill=(180, 180, 180), width=1)

                    # Draw limit lines for torque
                    if data_type == 'torque' and torque_limit:
                        limit_y_pos = y1 - 5 - int((limit - y_min) / y_range * (y1 - y0 - 10))
                        limit_y_neg = y1 - 5 - int((-limit - y_min) / y_range * (y1 - y0 - 10))
                        # Dashed line
                        for x in range(x0_plot + 5, x1 - 5, 6):
                            draw.line([(x, limit_y_pos), (x + 3, limit_y_pos)], fill='red', width=1)
                            draw.line([(x, limit_y_neg), (x + 3, limit_y_neg)], fill='red', width=1)

                    # Draw curve and mark max/min values
                    if len(t_values) > 1 and len(data) == len(t_values):
                        points = []
                        for t, val in zip(t_values, data):
                            px = x0_plot + 5 + int((t / t_max) * (x1 - x0_plot - 10))
                            py = y1 - 5 - int((val - y_min) / y_range * (y1 - y0 - 10))
                            py = max(y0 + 3, min(y1 - 3, py))
                            points.append((px, py))

                        color = colors_torque[col] if data_type == 'torque' else \
                                color_accel if data_type == 'accel' else \
                                color_vel if data_type == 'vel' else color_pos
                        for j in range(len(points) - 1):
                            draw.line([points[j], points[j+1]], fill=color, width=2)

                        # Mark max value
                        max_val = max(data)
                        max_idx = data.index(max_val)
                        max_px = x0_plot + 5 + int((t_values[max_idx] / t_max) * (x1 - x0_plot - 10))
                        max_py = y1 - 5 - int((max_val - y_min) / y_range * (y1 - y0 - 10))
                        max_py = max(y0 + 3, min(y1 - 3, max_py))
                        draw.ellipse([max_px-3, max_py-3, max_px+3, max_py+3], fill='blue')
                        # Max label
                        if data_type == 'torque':
                            max_label = f'{max_val:.1f}'
                        elif data_type in ['accel', 'vel']:
                            max_label = f'{max_val:.2f}'
                        else:
                            max_label = f'{max_val:.3f}'
                        draw.text((max_px + 4, max_py - 4), f'Max:{max_label}', fill='blue', font=font_tick)

                        # Mark min value
                        min_val = min(data)
                        min_idx = data.index(min_val)
                        min_px = x0_plot + 5 + int((t_values[min_idx] / t_max) * (x1 - x0_plot - 10))
                        min_py = y1 - 5 - int((min_val - y_min) / y_range * (y1 - y0 - 10))
                        min_py = max(y0 + 3, min(y1 - 3, min_py))
                        draw.ellipse([min_px-3, min_py-3, min_px+3, min_py+3], fill='purple')
                        # Min label
                        if data_type == 'torque':
                            min_label = f'{min_val:.1f}'
                        elif data_type in ['accel', 'vel']:
                            min_label = f'{min_val:.2f}'
                        else:
                            min_label = f'{min_val:.3f}'
                        draw.text((min_px + 4, min_py - 4), f'Min:{min_label}', fill='purple', font=font_tick)

                    # Draw X-axis tick labels (only on bottom row)
                    if row == 3:
                        num_x_ticks = 5
                        for tick in range(num_x_ticks + 1):
                            t_val = t_max * tick / num_x_ticks
                            x_pos = x0_plot + 5 + int((t_val / t_max) * (x1 - x0_plot - 10))
                            # Tick mark
                            draw.line([(x_pos, y1 - 3), (x_pos, y1 + 2)], fill='gray', width=1)
                            # Tick label
                            draw.text((x_pos, y1 + 5), f'{t_val:.2f}', fill='gray', font=font_tick, anchor='mt')

            # X-axis label at bottom
            draw.text((width//2, height - 12), 'Time (s)', fill='black', font=font_label, anchor='mt')

            png_path = os.path.join(output_dir, f'joint_curves_{arm}_{name_suffix}_{timestamp}.png')
            img.save(png_path)
            png_paths.append(png_path)
            print(f"Joint curves PNG saved to: {png_path}")

    except ImportError:
        print("Note: Install Pillow to generate PNG images: pip install Pillow")

    return png_paths[0] if png_paths else (html_paths[0] if html_paths else None)


def generate_output_files(torque_history, velocity_history, acceleration_history, position_history,
                          times, joint_names, effort_limits, tcp_history,
                          positions, arm, speed_scale, payload_mass, output_dir, generate_plot=True):
    """Generate output files: joint curves and motion data."""

    os.makedirs(output_dir, exist_ok=True)
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')

    # Generate joint curves plot image
    plot_path = None
    if generate_plot:
        try:
            plot_path = generate_torque_plot(torque_history, velocity_history, acceleration_history, position_history,
                                            joint_names, effort_limits, arm, speed_scale, payload_mass, output_dir)
        except Exception as e:
            print(f"Warning: Could not generate plot image: {e}")

    # Generate torque curves text file
    torque_path = os.path.join(output_dir, f'torque_curves_pybullet_{arm}_{timestamp}.txt')

    with open(torque_path, 'w') as f:
        f.write(f"{'='*100}\n")
        f.write(f"PyBullet Joint Torque Curves - {arm.upper()} Arm\n")
        f.write(f"Speed: {speed_scale}x | Payload: {payload_mass} kg\n")
        f.write(f"{'='*100}\n\n")

        for joint_name in joint_names:
            torque_data = torque_history[joint_name]
            if not torque_data:
                continue

            limit = effort_limits.get(joint_name, 100.0)

            f.write(f"\n{'-'*100}\n")
            f.write(f"{joint_name} (Limit: {limit} Nm)\n")
            f.write(f"{'-'*100}\n")

            torque_values = [d['torque'] for d in torque_data]
            max_torque = max(torque_values) if torque_values else 0

            f.write(f"Time(s)  Torque(Nm)  {'0%':<20}{'25%':<20}{'50%':<20}{'75%':<20}{'100%'}\n")
            f.write(f"{'':<11}{'':<12}|{'-'*19}|{'-'*19}|{'-'*19}|{'-'*19}|\n")

            sample_indices = list(range(0, len(torque_data), max(1, len(torque_data) // 20)))

            for idx in sample_indices:
                t = torque_data[idx]['time']
                torque = torque_data[idx]['torque']
                usage = (torque / limit * 100) if limit > 0 else 0

                bar_length = min(int(usage / 100 * 80), 80)
                bar = '#' * bar_length

                if usage > 90:
                    status = '!'
                elif usage > 70:
                    status = '*'
                else:
                    status = ' '

                f.write(f"{t:7.2f}  {torque:10.2f}  {bar:<80} {status}\n")

            f.write(f"\nMax torque: {max_torque:.2f} Nm ({max_torque/limit*100:.1f}% of limit)\n")

        f.write(f"\n{'='*100}\n")

    print(f"\nTorque curves saved to: {torque_path}")

    # Generate motion data file
    motion_path = os.path.join(output_dir, f'motion_data_pybullet_{arm}_{timestamp}.txt')

    with open(motion_path, 'w') as f:
        f.write(f"{'='*120}\n")
        f.write(f"PyBullet Robot Motion Data - {arm.upper()} Arm\n")
        f.write(f"Speed: {speed_scale}x | Payload: {payload_mass} kg\n")
        f.write(f"{'='*120}\n\n")

        # TCP trajectory
        f.write(f"{'-'*120}\n")
        f.write("TCP (Tool Center Point) Trajectory\n")
        f.write(f"{'-'*120}\n")
        f.write(f"{'Time(s)':<10} {'X(m)':<12} {'Y(m)':<12} {'Z(m)':<12}\n")
        f.write(f"{'-'*120}\n")

        for tcp in tcp_history:
            f.write(f"{tcp['time']:<10.3f} {tcp['x']:<12.4f} {tcp['y']:<12.4f} {tcp['z']:<12.4f}\n")

        # TCP motion summary
        if tcp_history:
            x_vals = [tcp['x'] for tcp in tcp_history]
            y_vals = [tcp['y'] for tcp in tcp_history]
            z_vals = [tcp['z'] for tcp in tcp_history]

            f.write(f"\n{'-'*120}\n")
            f.write("TCP Motion Summary\n")
            f.write(f"{'-'*120}\n")
            f.write(f"X range: {min(x_vals):.4f} ~ {max(x_vals):.4f} m\n")
            f.write(f"Y range: {min(y_vals):.4f} ~ {max(y_vals):.4f} m\n")
            f.write(f"Z range: {min(z_vals):.4f} ~ {max(z_vals):.4f} m\n")

        f.write(f"{'='*120}\n")

    print(f"Motion data saved to: {motion_path}")

    return torque_path, motion_path


def save_report(max_torques, torque_history, effort_limits, arm, speed_scale, payload_mass,
                payload_com, sim_time, tcp_history, output_path=None):
    """Save torque report to JSON file."""
    if output_path is None:
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        output_path = f"torque_report_pybullet_{arm}_{timestamp}.json"

    results = {
        'timestamp': datetime.now().isoformat(),
        'simulation_engine': 'PyBullet',
        'arm': arm,
        'speed_scale': speed_scale,
        'payload': {
            'mass_kg': payload_mass,
            'com_distance_m': payload_com,
        },
        'simulation_time_s': sim_time,
        'max_torques': max_torques,
        'effort_limits': effort_limits,
        'effort_usage_percent': {
            name: (max_torques[name] / limit * 100) if limit > 0 and name in max_torques else 0
            for name, limit in effort_limits.items()
        },
        'torque_history': torque_history,
        'tcp_history': tcp_history,
        'note': 'Torques calculated using PyBullet physics engine inverse dynamics with time-synchronized simulation.'
    }

    with open(output_path, 'w') as f:
        json.dump(results, f, indent=2)

    print(f"Report saved to: {output_path}")
    return output_path


def main():
    parser = argparse.ArgumentParser(
        description='PyBullet trajectory simulation with torque calculation',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Basic usage with visualization
  python3 scripts/simulate_trajectory_pybullet.py --trajectory 35/3.csv --arm right --visualize

  # With 2x speed and payload, realtime mode
  python3 scripts/simulate_trajectory_pybullet.py -t 35/3.csv -a right --speed 2.0 --payload 2.0 --realtime --visualize

  # Fast simulation without visualization
  python3 scripts/simulate_trajectory_pybullet.py -t 35/3.csv -a right --speed 1.5 --payload 2.0 --plot
        """
    )
    parser.add_argument('--trajectory', '-t', required=True, help='Path to trajectory CSV file')
    parser.add_argument('--arm', '-a', default='right', choices=['left', 'right'], help='Arm to use')
    parser.add_argument('--speed', '-s', type=float, default=1.0,
                        help='Speed scaling factor (e.g., 2.0 means 2x faster)')
    parser.add_argument('--payload', '-p', type=float, default=0.0,
                        help='End-effector payload mass in kg')
    parser.add_argument('--payload-com', type=float, default=0.1,
                        help='Distance from end-effector to payload center of mass in meters')
    parser.add_argument('--realtime', '-r', action='store_true',
                        help='Realtime simulation mode (execute at real time speed)')
    parser.add_argument('--visualize', '-v', action='store_true',
                        help='Enable PyBullet GUI visualization')
    parser.add_argument('--plot', action='store_true', help='Generate torque curve plots')
    parser.add_argument('--output-dir', default='.', help='Output directory for reports and plots')
    parser.add_argument('--output', '-o', default=None, help='Output report path')
    args = parser.parse_args()

    # Validate inputs
    if not os.path.exists(args.trajectory):
        print(f"Error: Trajectory file not found: {args.trajectory}")
        return

    if args.speed <= 0:
        print("Error: Speed scaling factor must be positive")
        return

    if args.payload < 0:
        print("Error: Payload mass cannot be negative")
        return

    print(f"\n{'='*60}")
    print(f"PyBullet Trajectory Simulation")
    print(f"{'='*60}")
    print(f"  Trajectory file: {args.trajectory}")
    print(f"  Arm: {args.arm}")
    print(f"  Speed scale: {args.speed}x")
    print(f"  Payload mass: {args.payload} kg")
    print(f"  Payload CoM distance: {args.payload_com} m")
    print(f"  Visualization: {'Enabled' if args.visualize else 'Disabled'}")
    print(f"  Realtime mode: {'Enabled' if args.realtime else 'Disabled'}")
    print(f"{'='*60}\n")

    # Load trajectory
    print("Loading trajectory...")
    times, positions, velocities, accelerations = load_trajectory(
        args.trajectory, args.arm, args.speed
    )
    print(f"Loaded {len(times)} trajectory points")
    print(f"Original duration: {times[-1] * args.speed:.2f} seconds")
    print(f"Scaled duration: {times[-1]:.2f} seconds")

    # Initialize PyBullet simulator
    print("\nInitializing PyBullet simulation...")
    simulator = PyBulletTrajectorySimulator(
        arm=args.arm,
        payload_mass=args.payload,
        payload_com_distance=args.payload_com,
        visualize=args.visualize,
        realtime=args.realtime
    )

    try:
        simulator.initialize()
        print("PyBullet simulation initialized successfully")

        # Run trajectory simulation
        results = simulator.run_trajectory(
            times, positions, velocities, accelerations,
            speed_scale=1.0  # Speed already applied in trajectory loading
        )

        # Print torque report
        max_usage = print_torque_report(
            results['max_torques'],
            simulator.effort_limits,
            args.arm,
            args.speed,
            args.payload
        )

        # Generate output files
        generate_output_files(
            results['torque_history'],
            results['velocity_history'],
            results['acceleration_history'],
            results['position_history'],
            times,
            simulator.joint_names,
            simulator.effort_limits,
            results['tcp_history'],
            positions,
            args.arm,
            args.speed,
            args.payload,
            args.output_dir,
            generate_plot=args.plot
        )

        # Save JSON report (use timestamp to avoid permission issues)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        output_path = args.output if args.output else os.path.join(
            args.output_dir, f"torque_report_pybullet_{args.arm}_{timestamp}.json"
        )
        save_report(
            results['max_torques'],
            results['torque_history'],
            simulator.effort_limits,
            args.arm,
            args.speed,
            args.payload,
            args.payload_com,
            results['simulation_time'],
            results['tcp_history'],
            output_path
        )

        # Summary
        print("\n" + "=" * 60)
        print("SIMULATION SUMMARY")
        print("=" * 60)
        print(f"  Simulation engine: PyBullet")
        print(f"  Speed scaling: {args.speed}x")
        print(f"  Payload: {args.payload} kg")
        print(f"  Simulation time: {results['simulation_time']:.2f} seconds")
        print(f"  Maximum torque usage: {max_usage:.1f}%")

        if max_usage > 90:
            print("\n  WARNING: Torque limits exceeded! Consider:")
            print("      - Reducing speed")
            print("      - Reducing payload")
            print("      - Modifying trajectory")
        elif max_usage > 70:
            print("\n  NOTICE: Torque usage above 70%")
            print("      Monitor robot during execution")
        else:
            print("\n  Trajectory is safe to execute")

        # If visualization is enabled, keep window open
        if args.visualize:
            print("\n  Simulation complete. Close the PyBullet window to exit.")
            # Keep simulation running until user closes window
            while True:
                try:
                    simulator.step_simulation(1)
                    time.sleep(0.001)
                except (KeyboardInterrupt, RuntimeError, OSError):
                    break

    finally:
        # Clean up
        print("\nCleaning up simulation...")
        simulator.cleanup()

    print("\nDone!")


if __name__ == '__main__':
    main()