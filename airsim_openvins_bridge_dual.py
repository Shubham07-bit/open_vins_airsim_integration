#!/usr/bin/env python3
"""
ROS2 Bridge Node: AirSim -> OpenVINS (DUAL CAMERA)

Connects to AirSim and publishes IMU + TWO independent monocular cameras
for use with the airsim_vio_dual OpenVINS config.

Cameras (must be configured in AirSim settings.json):
  front_45  -> pitch -45 deg  -> /camera0/image
  front_90  -> pitch -90 deg  -> /camera1/image

Both cameras are queried in a single simGetImages RPC call so their
timestamps are identical (frame-synchronous), which is what the EKF
expects when fusing measurements from multiple cameras.

Published Topics:
  /imu              (sensor_msgs/Imu)        @ 200 Hz
  /camera0/image    (sensor_msgs/Image)      @ camera_rate Hz   (-45 deg)
  /camera0/camera_info
  /camera1/image    (sensor_msgs/Image)      @ camera_rate Hz   (-90 deg)
  /camera1/camera_info

Usage:
  python3 airsim_openvins_bridge_dual.py
"""

import sys
import time
import threading
import signal

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Imu, Image, CameraInfo
from std_msgs.msg import Header
from builtin_interfaces.msg import Time as RosTime

import airsim


def airsim_timestamp_to_ros(timestamp_ns: int) -> RosTime:
    """Convert AirSim nanosecond timestamp to ROS2 Time message."""
    t = RosTime()
    t.sec = int(timestamp_ns // 1_000_000_000)
    t.nanosec = int(timestamp_ns % 1_000_000_000)
    return t


def bgra_to_gray(img_bgra: np.ndarray) -> np.ndarray:
    """ITU-R BT.601 luma from BGRA / BGR / single-channel input."""
    if img_bgra.ndim == 3 and img_bgra.shape[2] >= 3:
        return (
            0.114 * img_bgra[:, :, 0].astype(np.float32) +
            0.587 * img_bgra[:, :, 1].astype(np.float32) +
            0.299 * img_bgra[:, :, 2].astype(np.float32)
        ).astype(np.uint8)
    return img_bgra[:, :, 0] if img_bgra.ndim == 3 else img_bgra


class AirSimDualCamBridge(Node):
    """ROS2 node bridging AirSim IMU + 2 independent cameras to OpenVINS."""

    def __init__(self):
        super().__init__('airsim_openvins_bridge_dual')

        # Declare parameters
        self.declare_parameter('airsim_ip', '127.0.0.1')
        self.declare_parameter('vehicle_name', 'Copter')
        self.declare_parameter('camera0_name', 'front_45')   # -45 deg
        self.declare_parameter('camera1_name', 'front_90')   # -90 deg
        self.declare_parameter('imu_rate_hz', 200.0)
        self.declare_parameter('camera_rate_hz', 15.0)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('fov_degrees', 62.0)

        self.airsim_ip = self.get_parameter('airsim_ip').value
        self.vehicle_name = self.get_parameter('vehicle_name').value
        self.cam0_name = self.get_parameter('camera0_name').value
        self.cam1_name = self.get_parameter('camera1_name').value
        self.imu_rate = self.get_parameter('imu_rate_hz').value
        self.camera_rate = self.get_parameter('camera_rate_hz').value
        self.image_width = self.get_parameter('image_width').value
        self.image_height = self.get_parameter('image_height').value
        self.fov_degrees = self.get_parameter('fov_degrees').value

        # Intrinsics from FOV (both cameras share these)
        fov_rad = self.fov_degrees * np.pi / 180.0
        self.fx = self.image_width / (2.0 * np.tan(fov_rad / 2.0))
        self.fy = self.fx
        self.cx = self.image_width / 2.0
        self.cy = self.image_height / 2.0

        imu_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        camera_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Publishers
        self.imu_pub = self.create_publisher(Imu, '/imu', imu_qos)
        self.image0_pub = self.create_publisher(Image, '/camera0/image', camera_qos)
        self.image1_pub = self.create_publisher(Image, '/camera1/image', camera_qos)
        self.info0_pub = self.create_publisher(CameraInfo, '/camera0/camera_info', camera_qos)
        self.info1_pub = self.create_publisher(CameraInfo, '/camera1/camera_info', camera_qos)

        # NOTE: msgpack-rpc/tornado IOLoop is NOT thread-safe. One client per thread.
        self.imu_client = None
        self.camera_client = None

        self._verify_airsim_reachable()

        # Stats
        self.imu_count = 0
        self.image0_count = 0
        self.image1_count = 0
        self.last_stats_time = time.time()

        self._shutdown = False

        self.imu_thread = threading.Thread(target=self._imu_loop, daemon=True)
        self.camera_thread = threading.Thread(target=self._camera_loop, daemon=True)
        self.imu_thread.start()
        self.camera_thread.start()

        self.stats_timer = self.create_timer(5.0, self._print_stats)

        self.get_logger().info(
            f'AirSim DUAL-cam bridge: IMU@{self.imu_rate}Hz, '
            f'Cameras[{self.cam0_name},{self.cam1_name}]@{self.camera_rate}Hz, '
            f'{self.image_width}x{self.image_height}, FOV {self.fov_degrees}deg'
        )

    def _verify_airsim_reachable(self):
        for attempt in range(30):
            try:
                probe = airsim.MultirotorClient(ip=self.airsim_ip)
                probe.confirmConnection()
                self.get_logger().info(f'AirSim reachable at {self.airsim_ip}')
                del probe
                return
            except Exception as e:
                self.get_logger().warn(
                    f'AirSim connect attempt {attempt + 1}/30 failed: {e}'
                )
                time.sleep(2.0)
        self.get_logger().error('Failed to reach AirSim')
        sys.exit(1)

    def _make_thread_client(self, label):
        for attempt in range(10):
            try:
                client = airsim.MultirotorClient(ip=self.airsim_ip)
                client.confirmConnection()
                self.get_logger().info(f'{label} thread connected to AirSim')
                return client
            except Exception as e:
                self.get_logger().warn(
                    f'{label} thread connect retry {attempt + 1}: {e}'
                )
                time.sleep(2.0)
        self.get_logger().error(f'{label} thread failed to connect')
        return None

    def _imu_loop(self):
        period = 1.0 / self.imu_rate
        last_timestamp = 0
        self.imu_client = self._make_thread_client('IMU')
        if self.imu_client is None:
            return

        while rclpy.ok() and not self._shutdown:
            loop_start = time.time()
            try:
                imu_data = self.imu_client.getImuData(vehicle_name=self.vehicle_name)
                if imu_data.time_stamp == last_timestamp:
                    time.sleep(period * 0.1)
                    continue
                last_timestamp = imu_data.time_stamp

                # NED -> FLU body conversion (REP-103)
                gx = float(imu_data.angular_velocity.x_val)
                gy = float(imu_data.angular_velocity.y_val)
                gz = float(imu_data.angular_velocity.z_val)
                ax = float(imu_data.linear_acceleration.x_val)
                ay = float(imu_data.linear_acceleration.y_val)
                az = float(imu_data.linear_acceleration.z_val)

                msg = Imu()
                msg.header = Header()
                msg.header.stamp = airsim_timestamp_to_ros(imu_data.time_stamp)
                msg.header.frame_id = 'imu'

                msg.angular_velocity.x = gx
                msg.angular_velocity.y = -gy
                msg.angular_velocity.z = -gz

                msg.linear_acceleration.x = ax
                msg.linear_acceleration.y = -ay
                msg.linear_acceleration.z = -az

                qw = float(imu_data.orientation.w_val)
                qx = float(imu_data.orientation.x_val)
                qy = float(imu_data.orientation.y_val)
                qz = float(imu_data.orientation.z_val)
                msg.orientation.w = qw
                msg.orientation.x = qx
                msg.orientation.y = -qy
                msg.orientation.z = -qz

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

            elapsed = time.time() - loop_start
            sleep_time = period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def _camera_loop(self):
        """
        Poll BOTH cameras in a single simGetImages call so their timestamps
        are identical. OpenVINS multi-camera tracking benefits significantly
        from frame-synchronous inputs.
        """
        period = 1.0 / self.camera_rate
        self.camera_client = self._make_thread_client('Camera')
        if self.camera_client is None:
            return

        # Build the multi-camera request once and reuse it.
        requests = [
            airsim.ImageRequest(
                camera_name=self.cam0_name,
                image_type=airsim.ImageType.Scene,
                pixels_as_float=False,
                compress=False,
            ),
            airsim.ImageRequest(
                camera_name=self.cam1_name,
                image_type=airsim.ImageType.Scene,
                pixels_as_float=False,
                compress=False,
            ),
        ]

        while rclpy.ok() and not self._shutdown:
            loop_start = time.time()

            try:
                responses = self.camera_client.simGetImages(
                    requests, vehicle_name=self.vehicle_name
                )

                if not responses or len(responses) < 2:
                    self.get_logger().warn(
                        f'Expected 2 image responses, got '
                        f'{0 if not responses else len(responses)}'
                    )
                    time.sleep(period)
                    continue

                resp0, resp1 = responses[0], responses[1]

                if (resp0.width == 0 or resp0.height == 0 or
                        resp1.width == 0 or resp1.height == 0):
                    self.get_logger().warn('Invalid dimensions in image response')
                    time.sleep(period)
                    continue

                # AirSim returns the responses in the SAME ORDER as the request,
                # so resp0 is always the -45 cam and resp1 is always the -90 cam.
                # Use a SINGLE timestamp for both messages: take resp0's stamp.
                # (resp0 and resp1 are produced from the same engine tick when
                # captured via one simGetImages call.)
                stamp = airsim_timestamp_to_ros(resp0.time_stamp)

                # ----- Camera 0 (-45 deg) -----
                buf0 = np.frombuffer(resp0.image_data_uint8, dtype=np.uint8)
                buf0 = buf0.reshape(resp0.height, resp0.width, -1)
                gray0 = bgra_to_gray(buf0)

                img0_msg = Image()
                img0_msg.header = Header()
                img0_msg.header.stamp = stamp
                img0_msg.header.frame_id = 'cam0'
                img0_msg.height = resp0.height
                img0_msg.width = resp0.width
                img0_msg.encoding = 'mono8'
                img0_msg.is_bigendian = False
                img0_msg.step = resp0.width
                img0_msg.data = gray0.tobytes()
                self.image0_pub.publish(img0_msg)

                info0_msg = CameraInfo()
                info0_msg.header = img0_msg.header
                info0_msg.height = resp0.height
                info0_msg.width = resp0.width
                info0_msg.distortion_model = 'plumb_bob'
                info0_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
                info0_msg.k = [self.fx, 0.0, self.cx,
                               0.0, self.fy, self.cy,
                               0.0, 0.0, 1.0]
                info0_msg.r = [1.0, 0.0, 0.0,
                               0.0, 1.0, 0.0,
                               0.0, 0.0, 1.0]
                info0_msg.p = [self.fx, 0.0, self.cx, 0.0,
                               0.0, self.fy, self.cy, 0.0,
                               0.0, 0.0, 1.0, 0.0]
                self.info0_pub.publish(info0_msg)
                self.image0_count += 1

                # ----- Camera 1 (-90 deg) -----
                buf1 = np.frombuffer(resp1.image_data_uint8, dtype=np.uint8)
                buf1 = buf1.reshape(resp1.height, resp1.width, -1)
                gray1 = bgra_to_gray(buf1)

                img1_msg = Image()
                img1_msg.header = Header()
                img1_msg.header.stamp = stamp        # SAME stamp as cam0
                img1_msg.header.frame_id = 'cam1'
                img1_msg.height = resp1.height
                img1_msg.width = resp1.width
                img1_msg.encoding = 'mono8'
                img1_msg.is_bigendian = False
                img1_msg.step = resp1.width
                img1_msg.data = gray1.tobytes()
                self.image1_pub.publish(img1_msg)

                info1_msg = CameraInfo()
                info1_msg.header = img1_msg.header
                info1_msg.height = resp1.height
                info1_msg.width = resp1.width
                info1_msg.distortion_model = 'plumb_bob'
                info1_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
                info1_msg.k = [self.fx, 0.0, self.cx,
                               0.0, self.fy, self.cy,
                               0.0, 0.0, 1.0]
                info1_msg.r = [1.0, 0.0, 0.0,
                               0.0, 1.0, 0.0,
                               0.0, 0.0, 1.0]
                info1_msg.p = [self.fx, 0.0, self.cx, 0.0,
                               0.0, self.fy, self.cy, 0.0,
                               0.0, 0.0, 1.0, 0.0]
                self.info1_pub.publish(info1_msg)
                self.image1_count += 1

            except Exception as e:
                self.get_logger().error(f'Camera read error: {e}')
                time.sleep(1.0)
                self.camera_client = self._make_thread_client('Camera')
                if self.camera_client is None:
                    return
                continue

            elapsed = time.time() - loop_start
            sleep_time = period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def _print_stats(self):
        now = time.time()
        dt = now - self.last_stats_time
        if dt > 0:
            imu_hz = self.imu_count / dt
            cam0_hz = self.image0_count / dt
            cam1_hz = self.image1_count / dt
            self.get_logger().info(
                f'Rates - IMU: {imu_hz:.1f} Hz, '
                f'cam0(-45): {cam0_hz:.1f} Hz, cam1(-90): {cam1_hz:.1f} Hz'
            )
        self.imu_count = 0
        self.image0_count = 0
        self.image1_count = 0
        self.last_stats_time = now

    def destroy_node(self):
        self._shutdown = True
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AirSimDualCamBridge()

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
