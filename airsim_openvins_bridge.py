#!/usr/bin/env python3
"""
ROS2 Bridge Node: AirSim -> OpenVINS

Connects to AirSim via Python API and publishes sensor data as ROS2 topics
for consumption by OpenVINS (ov_msckf).

Published Topics:
  /imu            (sensor_msgs/Imu)        @ 200 Hz
    /camera/image   (sensor_msgs/Image)      @ 30 Hz

Usage:
  ros2 run <your_package> airsim_openvins_bridge.py
  OR
  python3 airsim_openvins_bridge.py
"""

import sys
import time
import threading
import signal

import numpy as np
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Imu, Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Header
from builtin_interfaces.msg import Time as RosTime

import airsim


def airsim_timestamp_to_ros(timestamp_ns: int) -> RosTime:
    """Convert AirSim nanosecond timestamp to ROS2 Time message."""
    t = RosTime()
    t.sec = int(timestamp_ns // 1_000_000_000)
    t.nanosec = int(timestamp_ns % 1_000_000_000)
    return t


class AirSimOpenVINSBridge(Node):
    """ROS2 node that bridges AirSim sensor data to OpenVINS-compatible topics."""

    def __init__(self):
        super().__init__('airsim_openvins_bridge')

        # Declare parameters
        self.declare_parameter('airsim_ip', '127.0.0.1')
        self.declare_parameter('vehicle_name', 'Copter')
        self.declare_parameter('camera_name', 'front_center')
        self.declare_parameter('imu_rate_hz', 200.0)
        self.declare_parameter('camera_rate_hz', 15.0)
        self.declare_parameter('image_width', 1280)
        self.declare_parameter('image_height', 960)
        self.declare_parameter('fov_degrees', 62.0)
        self.declare_parameter('gt_rate_hz', 100.0)
        self.declare_parameter('gt_filepath', '/home/shubham/openvins_ws/logs/survey-mission-2/groundtruth_asl.csv')

        # Read parameters
        self.airsim_ip = self.get_parameter('airsim_ip').value
        self.vehicle_name = self.get_parameter('vehicle_name').value
        self.camera_name = self.get_parameter('camera_name').value
        self.imu_rate = self.get_parameter('imu_rate_hz').value
        self.camera_rate = self.get_parameter('camera_rate_hz').value
        self.image_width = self.get_parameter('image_width').value
        self.image_height = self.get_parameter('image_height').value
        self.fov_degrees = self.get_parameter('fov_degrees').value
        self.gt_rate = self.get_parameter('gt_rate_hz').value
        self.gt_filepath = self.get_parameter('gt_filepath').value

        # Compute camera intrinsics from FOV
        fov_rad = self.fov_degrees * np.pi / 180.0
        self.fx = self.image_width / (2.0 * np.tan(fov_rad / 2.0))
        self.fy = self.fx  # Square pixels
        self.cx = self.image_width / 2.0
        self.cy = self.image_height / 2.0

        # QoS for IMU: SensorDataQoS (BEST_EFFORT, volatile, keep_last=10)
        imu_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # QoS for camera: reliable with small queue
        camera_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Publishers
        self.imu_pub = self.create_publisher(Imu, '/imu', imu_qos)
        self.image_pub = self.create_publisher(Image, '/camera/image', camera_qos)
        self.camera_info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', camera_qos)

        # Ground truth publishers (same topic names as OpenVINS visualizer)
        # Ground truth publishers
        # /bridge/groundtruth is subscribed by OpenVINS for live GT file recording
        # /ov_msckf/pathgt is for RViz path visualization
        self.posegt_pub = self.create_publisher(PoseStamped, '/bridge/groundtruth', 10)
        self.pathgt_pub = self.create_publisher(Path, '/ov_msckf/pathgt', 2)
        self.poses_gt = []  # accumulated poses for path

        # Subscribe to OpenVINS odometry for auto-alignment of GT frame
        self._vio_pos = None  # latest VIO position as np.array, set by callback
        self._vio_lock = threading.Lock()
        self.create_subscription(
            Odometry, '/ov_msckf/odomimu', self._vio_odom_cb, 2)

        # NOTE: msgpack-rpc / tornado IOLoop is NOT thread-safe, and AirSim's
        # MultirotorClient holds one IOLoop per instance. We must use one client
        # per polling thread (this is the same pattern used by airsim_ros_pkgs:
        # airsim_client_, airsim_client_images_, airsim_client_lidar_).
        # Each thread creates its own client inside its target function so the
        # IOLoop is bound to that thread.
        self.imu_client = None
        self.camera_client = None

        # Verify AirSim is reachable before spawning threads (uses a throwaway client)
        self._verify_airsim_reachable()

        # Statistics
        self.imu_count = 0
        self.image_count = 0
        self.last_stats_time = time.time()

        # Shutdown flag
        self._shutdown = False

        # Open groundtruth CSV file (ASL/EuRoC format for OpenVINS)
        import os
        os.makedirs(os.path.dirname(self.gt_filepath), exist_ok=True)
        self.gt_file = open(self.gt_filepath, 'w')
        self.gt_file.write('#timestamp,q_w,q_x,q_y,q_z,p_x,p_y,p_z,v_x,v_y,v_z,bg_x,bg_y,bg_z,ba_x,ba_y,ba_z\n')
        self.gt_count = 0

        # Also write ov_eval-compatible format (space-separated, seconds, qx qy qz qw order)
        self.gt_eval_filepath = self.gt_filepath.replace('.csv', '_eval.txt')
        self.gt_eval_file = open(self.gt_eval_filepath, 'w')
        self.gt_eval_file.write('# timestamp(s) px py pz qx qy qz qw\n')

        # Start sensor polling threads (each creates its own AirSim client)
        self.imu_thread = threading.Thread(target=self._imu_loop, daemon=True)
        self.camera_thread = threading.Thread(target=self._camera_loop, daemon=True)
        self.gt_thread = threading.Thread(target=self._gt_loop, daemon=True)
        self.imu_thread.start()
        self.camera_thread.start()
        self.gt_thread.start()

        # Stats timer
        self.stats_timer = self.create_timer(5.0, self._print_stats)

        self.get_logger().info(
            f'AirSim-OpenVINS bridge started: '
            f'IMU@{self.imu_rate}Hz, Camera@{self.camera_rate}Hz, '
            f'Image:{self.image_width}x{self.image_height}, FOV:{self.fov_degrees}deg'
        )

    def _verify_airsim_reachable(self):
        """Confirm AirSim is reachable using a throwaway client (main thread)."""
        max_retries = 30
        for attempt in range(max_retries):
            try:
                probe = airsim.MultirotorClient(ip=self.airsim_ip)
                probe.confirmConnection()
                self.get_logger().info(f'AirSim reachable at {self.airsim_ip}')
                # Discard the probe client; threads will create their own
                del probe
                return
            except Exception as e:
                self.get_logger().warn(
                    f'AirSim connection attempt {attempt + 1}/{max_retries} failed: {e}'
                )
                time.sleep(2.0)

        self.get_logger().error('Failed to reach AirSim after all retries')
        sys.exit(1)

    def _make_thread_client(self, label):
        """Create a fresh AirSim client owned by the calling thread."""
        for attempt in range(10):
            try:
                client = airsim.MultirotorClient(ip=self.airsim_ip)
                client.confirmConnection()
                self.get_logger().info(f'{label} thread connected to AirSim')
                return client
            except Exception as e:
                self.get_logger().warn(
                    f'{label} thread AirSim connect retry {attempt + 1}: {e}'
                )
                time.sleep(2.0)
        self.get_logger().error(f'{label} thread failed to connect to AirSim')
        return None

    def _imu_loop(self):
        """Poll IMU data from AirSim and publish at configured rate."""
        period = 1.0 / self.imu_rate
        last_timestamp = 0

        # Create this thread's own AirSim client (owns its own tornado IOLoop)
        self.imu_client = self._make_thread_client('IMU')
        if self.imu_client is None:
            return

        while rclpy.ok() and not self._shutdown:
            loop_start = time.time()

            try:
                imu_data = self.imu_client.getImuData(vehicle_name=self.vehicle_name)

                # Skip duplicate timestamps
                if imu_data.time_stamp == last_timestamp:
                    time.sleep(period * 0.1)
                    continue
                last_timestamp = imu_data.time_stamp

                # NED -> FLU body frame conversion (REP-103)
                # AirSim body: X=forward, Y=right, Z=down (NED)
                # ROS  body:   X=forward, Y=left,  Z=up   (FLU)
                # Conversion: x stays, y negates, z negates.
                # At rest in NED, accel reads [0, 0, -9.81]; in FLU it becomes [0, 0, +9.81]
                # which is what OpenVINS / RViz expect.
                gx = float(imu_data.angular_velocity.x_val)
                gy = float(imu_data.angular_velocity.y_val)
                gz = float(imu_data.angular_velocity.z_val)
                ax = float(imu_data.linear_acceleration.x_val)
                ay = float(imu_data.linear_acceleration.y_val)
                az = float(imu_data.linear_acceleration.z_val)

                # Build IMU message
                msg = Imu()
                msg.header = Header()
                msg.header.stamp = airsim_timestamp_to_ros(imu_data.time_stamp)
                msg.header.frame_id = 'imu'

                # Angular velocity (rad/s) - NED -> FLU
                msg.angular_velocity.x = gx
                msg.angular_velocity.y = -gy
                msg.angular_velocity.z = -gz

                # Linear acceleration (m/s^2) - NED -> FLU, includes gravity
                msg.linear_acceleration.x = ax
                msg.linear_acceleration.y = -ay
                msg.linear_acceleration.z = -az

                # Orientation quaternion - NED -> FLU body, NED -> ENU world
                # q_enu_flu = q_enu_ned * q_ned_flu^-1 simplifies to swapping signs
                # of y and z components for the typical NED-to-FLU body case.
                qw = float(imu_data.orientation.w_val)
                qx = float(imu_data.orientation.x_val)
                qy = float(imu_data.orientation.y_val)
                qz = float(imu_data.orientation.z_val)
                msg.orientation.w = qw
                msg.orientation.x = qx
                msg.orientation.y = -qy
                msg.orientation.z = -qz

                # Covariance: -1 indicates unknown (OpenVINS uses its own noise model)
                msg.orientation_covariance[0] = -1.0
                msg.angular_velocity_covariance[0] = -1.0
                msg.linear_acceleration_covariance[0] = -1.0

                self.imu_pub.publish(msg)
                self.imu_count += 1

            except Exception as e:
                self.get_logger().error(f'IMU read error: {e}')
                time.sleep(1.0)
                self.imu_client = self._make_thread_client('IMU')
                if self.imu_client is None:
                    return
                continue

            # Sleep to maintain rate
            elapsed = time.time() - loop_start
            sleep_time = period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def _camera_loop(self):
        """Poll camera images from AirSim and publish at configured rate."""
        period = 1.0 / self.camera_rate

        # Create this thread's own AirSim client (owns its own tornado IOLoop)
        self.camera_client = self._make_thread_client('Camera')
        if self.camera_client is None:
            return

        while rclpy.ok() and not self._shutdown:
            loop_start = time.time()

            try:
                # Request uncompressed RGB image
                responses = self.camera_client.simGetImages([
                    airsim.ImageRequest(
                        camera_name=self.camera_name,
                        image_type=airsim.ImageType.Scene,
                        pixels_as_float=False,
                        compress=False
                    )
                ], vehicle_name=self.vehicle_name)

                if not responses or len(responses) == 0:
                    self.get_logger().warn('Empty image response from AirSim')
                    time.sleep(period)
                    continue

                response = responses[0]

                if response.width == 0 or response.height == 0:
                    self.get_logger().warn('Invalid image dimensions from AirSim')
                    time.sleep(period)
                    continue

                # Convert to numpy array (AirSim returns BGRA)
                img_bgra = np.frombuffer(response.image_data_uint8, dtype=np.uint8)
                img_bgra = img_bgra.reshape(response.height, response.width, -1)

                # Convert BGRA to grayscale for VIO (mono8)
                # Grayscale = 0.114*B + 0.587*G + 0.299*R
                if img_bgra.shape[2] == 4:
                    # BGRA
                    img_gray = (
                        0.114 * img_bgra[:, :, 0].astype(np.float32) +
                        0.587 * img_bgra[:, :, 1].astype(np.float32) +
                        0.299 * img_bgra[:, :, 2].astype(np.float32)
                    ).astype(np.uint8)
                elif img_bgra.shape[2] == 3:
                    # BGR
                    img_gray = (
                        0.114 * img_bgra[:, :, 0].astype(np.float32) +
                        0.587 * img_bgra[:, :, 1].astype(np.float32) +
                        0.299 * img_bgra[:, :, 2].astype(np.float32)
                    ).astype(np.uint8)
                else:
                    img_gray = img_bgra[:, :, 0]

                # Build Image message
                img_msg = Image()
                img_msg.header = Header()
                img_msg.header.stamp = airsim_timestamp_to_ros(response.time_stamp)
                img_msg.header.frame_id = 'cam0'
                img_msg.height = response.height
                img_msg.width = response.width
                img_msg.encoding = 'mono8'
                img_msg.is_bigendian = False
                img_msg.step = response.width  # mono8: 1 byte per pixel
                img_msg.data = img_gray.tobytes()

                self.image_pub.publish(img_msg)

                # Publish CameraInfo
                info_msg = CameraInfo()
                info_msg.header = img_msg.header
                info_msg.height = response.height
                info_msg.width = response.width
                info_msg.distortion_model = 'plumb_bob'
                info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
                info_msg.k = [
                    self.fx, 0.0, self.cx,
                    0.0, self.fy, self.cy,
                    0.0, 0.0, 1.0
                ]
                info_msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
                info_msg.p = [
                    self.fx, 0.0, self.cx, 0.0,
                    0.0, self.fy, self.cy, 0.0,
                    0.0, 0.0, 1.0, 0.0
                ]

                self.camera_info_pub.publish(info_msg)
                self.image_count += 1

            except Exception as e:
                self.get_logger().error(f'Camera read error: {e}')
                time.sleep(1.0)
                self.camera_client = self._make_thread_client('Camera')
                if self.camera_client is None:
                    return
                continue

            # Sleep to maintain rate
            elapsed = time.time() - loop_start
            sleep_time = period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def _vio_odom_cb(self, msg):
        """Callback: store latest OpenVINS position for GT auto-alignment."""
        with self._vio_lock:
            self._vio_pos = np.array([
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
            ])

    def _gt_loop(self):
        """Poll ground truth from AirSim, auto-align to OpenVINS global frame,
        and publish Path + PoseStamped for real-time comparison.

        Both OpenVINS global and our converted frame have Z=up.
        The only unknown is a yaw rotation in the XY plane.
        We detect this yaw by comparing GT displacement with VIO displacement
        after ~5m of motion, then apply it to all subsequent GT.
        """
        period = 1.0 / self.gt_rate

        gt_client = self._make_thread_client('GT')
        if gt_client is None:
            return

        # Record initial NED position
        kin_init = gt_client.simGetGroundTruthKinematics(vehicle_name=self.vehicle_name)
        p0_ned = np.array([
            float(kin_init.position.x_val),
            float(kin_init.position.y_val),
            float(kin_init.position.z_val),
        ])

        # Simple NED-to-Zup conversion: (x, y, z)_NED → (x, -y, -z)
        # This gives a Z-up frame aligned with NED North.
        # The only remaining unknown vs OpenVINS global is a yaw angle.
        R_ned_to_zup = np.diag([1.0, -1.0, -1.0])

        # Yaw alignment state
        yaw_offset = None       # will be set after auto-detection
        R_yaw = np.eye(3)       # identity until detected
        align_gt_pos = None     # GT position when alignment was detected
        align_vio_pos = None    # VIO position when alignment was detected

        _gt_debug_counter = 0

        self.get_logger().info(
            f'GT loop started. Waiting for VIO + motion to auto-detect yaw alignment...')

        while rclpy.ok() and not self._shutdown:
            loop_start = time.time()
            try:
                kin = gt_client.simGetGroundTruthKinematics(vehicle_name=self.vehicle_name)
                # Use AirSim sim time so GT and IMU/camera share the same clock
                ts_ns = gt_client.getImuData(vehicle_name=self.vehicle_name).time_stamp

                # Raw NED values
                p_ned = np.array([float(kin.position.x_val),
                                  float(kin.position.y_val),
                                  float(kin.position.z_val)])
                v_ned = np.array([float(kin.linear_velocity.x_val),
                                  float(kin.linear_velocity.y_val),
                                  float(kin.linear_velocity.z_val)])

                # Convert to Z-up frame (relative to init)
                p_zup = R_ned_to_zup @ (p_ned - p0_ned)
                v_zup = R_ned_to_zup @ v_ned

                # --- Auto-detect yaw offset once we have VIO data + motion ---
                if yaw_offset is None:
                    with self._vio_lock:
                        vio_pos = self._vio_pos.copy() if self._vio_pos is not None else None
                    if vio_pos is not None:
                        gt_dist = np.linalg.norm(p_zup[:2])  # XY displacement
                        vio_dist = np.linalg.norm(vio_pos[:2])
                        if gt_dist > 5.0 and vio_dist > 5.0:
                            # Compute yaw angle between GT and VIO displacement vectors
                            gt_angle = math.atan2(p_zup[1], p_zup[0])
                            vio_angle = math.atan2(vio_pos[1], vio_pos[0])
                            yaw_offset = vio_angle - gt_angle
                            c, s = math.cos(yaw_offset), math.sin(yaw_offset)
                            R_yaw = np.array([
                                [c, -s, 0.0],
                                [s,  c, 0.0],
                                [0.0, 0.0, 1.0],
                            ])
                            self.get_logger().info(
                                f'GT yaw auto-aligned: offset={math.degrees(yaw_offset):.1f}°  '
                                f'GT_angle={math.degrees(gt_angle):.1f}° VIO_angle={math.degrees(vio_angle):.1f}°')
                            # Re-transform all previously accumulated poses
                            new_poses = []
                            for old_pose in self.poses_gt:
                                old_p = np.array([old_pose.pose.position.x,
                                                  old_pose.pose.position.y,
                                                  old_pose.pose.position.z])
                                new_p = R_yaw @ old_p
                                new_pose = PoseStamped()
                                new_pose.header = old_pose.header
                                new_pose.pose.position.x = float(new_p[0])
                                new_pose.pose.position.y = float(new_p[1])
                                new_pose.pose.position.z = float(new_p[2])
                                new_pose.pose.orientation = old_pose.pose.orientation
                                new_poses.append(new_pose)
                            self.poses_gt = new_poses

                # Apply yaw rotation
                p_G = R_yaw @ p_zup
                v_G = R_yaw @ v_zup
                px, py, pz = float(p_G[0]), float(p_G[1]), float(p_G[2])
                vx, vy, vz = float(v_G[0]), float(v_G[1]), float(v_G[2])

                # Orientation: AirSim gives NED body-to-NED-world quaternion.
                # Convert to Z-up frame: R_zup = diag(1,-1,-1) is a 180° rotation
                # about X.  Conjugating q by this rotation: (w,x,y,z) → (w,x,-y,-z).
                # Then apply yaw alignment rotation R_yaw on the left (world frame).
                q_raw = kin.orientation
                qw_n = float(q_raw.w_val)
                qx_n = float(q_raw.x_val)
                qy_n = -float(q_raw.y_val)   # negate Y for NED→Z-up
                qz_n = -float(q_raw.z_val)   # negate Z for NED→Z-up

                if yaw_offset is not None and yaw_offset != 0.0:
                    # q_yaw = (cos(yaw/2), 0, 0, sin(yaw/2)) — rotation about Z
                    cy = math.cos(yaw_offset / 2.0)
                    sy = math.sin(yaw_offset / 2.0)
                    # quaternion multiply: q_yaw * q_zup
                    qw = cy * qw_n - sy * qz_n
                    qx = cy * qx_n - sy * qy_n
                    qy = cy * qy_n + sy * qx_n
                    qz = cy * qz_n + sy * qw_n
                else:
                    qw, qx, qy, qz = qw_n, qx_n, qy_n, qz_n

                # Debug: print every ~2 seconds
                _gt_debug_counter += 1
                if _gt_debug_counter % int(self.gt_rate * 2) == 1:
                    status = f'yaw={math.degrees(yaw_offset):.1f}°' if yaw_offset is not None else 'WAITING'
                    self.get_logger().info(
                        f'GT: NED=({p_ned[0]:.1f},{p_ned[1]:.1f},{p_ned[2]:.1f}) '
                        f'Global=({px:.1f},{py:.1f},{pz:.1f}) [{status}]')

                # Write ASL CSV
                self.gt_file.write(
                    f'{ts_ns},{qw},{qx},{qy},{qz},{px},{py},{pz},'
                    f'{vx},{vy},{vz},0,0,0,0,0,0\n'
                )
                # Write ov_eval-compatible format (time tx ty tz qx qy qz qw)
                ts_sec = ts_ns / 1e9
                self.gt_eval_file.write(
                    f'{ts_sec:.9f} {px} {py} {pz} {qx} {qy} {qz} {qw}\n'
                )
                self.gt_count += 1
                if self.gt_count % 100 == 0:
                    self.gt_file.flush()
                    self.gt_eval_file.flush()

                # Publish PoseStamped on /ov_msckf/posegt
                pose_msg = PoseStamped()
                pose_msg.header.stamp = airsim_timestamp_to_ros(ts_ns)
                pose_msg.header.frame_id = 'global'
                pose_msg.pose.orientation.x = qx
                pose_msg.pose.orientation.y = qy
                pose_msg.pose.orientation.z = qz
                pose_msg.pose.orientation.w = qw
                pose_msg.pose.position.x = px
                pose_msg.pose.position.y = py
                pose_msg.pose.position.z = pz
                self.posegt_pub.publish(pose_msg)

                # Accumulate and publish Path on /ov_msckf/pathgt
                self.poses_gt.append(pose_msg)
                path_msg = Path()
                path_msg.header.stamp = airsim_timestamp_to_ros(ts_ns)
                path_msg.header.frame_id = 'global'
                step = max(1, int(len(self.poses_gt) / 16384) + 1)
                path_msg.poses = self.poses_gt[::step]
                self.pathgt_pub.publish(path_msg)

            except Exception as e:
                self.get_logger().error(f'GT read error: {e}')
                time.sleep(1.0)
                gt_client = self._make_thread_client('GT')
                if gt_client is None:
                    return
                continue

            elapsed = time.time() - loop_start
            sleep_time = period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def _print_stats(self):
        """Log throughput statistics."""
        now = time.time()
        dt = now - self.last_stats_time
        if dt > 0:
            imu_hz = self.imu_count / dt
            cam_hz = self.image_count / dt
            gt_hz = self.gt_count / dt
            self.get_logger().info(
                f'Rates - IMU: {imu_hz:.1f} Hz, Camera: {cam_hz:.1f} Hz, GT: {gt_hz:.1f} Hz'
            )
        self.imu_count = 0
        self.image_count = 0
        self.gt_count = 0
        self.last_stats_time = now

    def destroy_node(self):
        self._shutdown = True
        if hasattr(self, 'gt_file') and self.gt_file:
            self.gt_file.flush()
            self.gt_file.close()
            self.get_logger().info(f'Groundtruth CSV saved to {self.gt_filepath}')
        if hasattr(self, 'gt_eval_file') and self.gt_eval_file:
            self.gt_eval_file.flush()
            self.gt_eval_file.close()
            self.get_logger().info(f'Groundtruth eval saved to {self.gt_eval_filepath}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AirSimOpenVINSBridge()

    def signal_handler(sig, frame):
        node.get_logger().info('Shutting down...')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
