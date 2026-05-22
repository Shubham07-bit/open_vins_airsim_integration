#!/usr/bin/env python3
"""
VIO → ArduPilot External Navigation Bridge
============================================
Subscribes to OpenVINS /ov_msckf/odomimu (Odometry) and sends
VISION_POSITION_ESTIMATE + VISION_SPEED_ESTIMATE to ArduPilot SITL
via MAVLink so EKF3 can fuse VIO as its position/velocity source.

Frame conversion:
  OpenVINS global frame: Z-up, arbitrary yaw (ENU-like)
  ArduPilot expects: NED frame
  Conversion: x_ned = x_ov, y_ned = -y_ov, z_ned = -z_ov
              roll_ned = roll_ov, pitch_ned = -pitch_ov, yaw_ned = -yaw_ov

Usage:
  # Terminal 1: Start AirSim + ArduPilot SITL
  # Terminal 2: Start OpenVINS bridge + VIO
  # Terminal 3: Run this node
  ros2 run -- python3 vio_to_ardupilot.py
  # or simply:
  python3 vio_to_ardupilot.py

ArduPilot parameters (set via MAVProxy or QGC):
  VISO_TYPE       = 1      # MAVLink visual odometry
  EK3_SRC1_POSXY  = 6      # ExternalNav
  EK3_SRC1_VELXY  = 6      # ExternalNav
  EK3_SRC1_POSZ   = 1      # Baro (safer) or 6 (ExternalNav)
  EK3_SRC1_VELZ   = 6      # ExternalNav
  EK3_SRC1_YAW    = 6      # ExternalNav (VIO yaw as heading)
  EK3_IMU_MASK    = 1      # Single IMU lane — avoids "IMU0/1 not aligned"
  GPS_TYPE        = 0      # Disable GPS
  AHRS_EKF_TYPE   = 3      # Use EKF3
  EK3_ENABLE      = 1

  # CRITICAL: Disable compass when using YAW=6 to prevent "compass variance":
  COMPASS_ENABLE  = 0      # Fully disable compass
  COMPASS_USE     = 0      # (or just disable compass use if ENABLE=0 not accepted)
  COMPASS_USE2    = 0
  COMPASS_USE3    = 0

  # Tune EKF3 noise for VIO:
  EK3_POS_I_GATE  = 500    # wider gate for external pos
  EK3_VEL_I_GATE  = 500    # wider gate for external vel
  EK3_YAW_I_GATE  = 500    # wider gate for VIO yaw noise during turns
  EK3_YAW_NOISE   = 1.0    # default 0.5 — increase for VIO
"""

import math
import time
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry

from pymavlink import mavutil


def quat_to_euler(qx, qy, qz, qw):
    """Quaternion (x,y,z,w) → Euler (roll, pitch, yaw) in radians."""
    # Roll (x-axis)
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis)
    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class VIOToArduPilot(Node):
    def __init__(self):
        super().__init__('vio_to_ardupilot')

        # Parameters
        self.declare_parameter('mavlink_connection', 'udpin:0.0.0.0:14568')
        self.declare_parameter('vio_topic', '/ov_msckf/odomimu')
        self.declare_parameter('send_velocity', True)
        self.declare_parameter('send_covariance', True)
        # CMAC home/origin
        self.declare_parameter('home_lat', -35.363261)
        self.declare_parameter('home_lon', 149.165230)
        self.declare_parameter('home_alt_m', 584.0)

        conn_str = self.get_parameter('mavlink_connection').value
        vio_topic = self.get_parameter('vio_topic').value
        self.send_velocity = self.get_parameter('send_velocity').value
        self.send_covariance = self.get_parameter('send_covariance').value

        # MAVLink connection — wait for heartbeat so UDP knows return address
        self.get_logger().info(f'Connecting to ArduPilot at {conn_str}...')
        self.mav_conn = mavutil.mavlink_connection(conn_str)
        self.get_logger().info('Waiting for heartbeat from ArduPilot...')
        self.mav_conn.wait_heartbeat(timeout=30)
        self.get_logger().info(
            f'Heartbeat received (system={self.mav_conn.target_system}, '
            f'component={self.mav_conn.target_component})')

        # Auto-configure ArduPilot params for VIO with YAW=6 (ExternalNav)
        self.declare_parameter('auto_configure', True)
        if self.get_parameter('auto_configure').value:
            self._configure_ardupilot_params()

        # Set EKF origin/home immediately — send multiple times to be sure
        self.home_lat = self.get_parameter('home_lat').value
        self.home_lon = self.get_parameter('home_lon').value
        self.home_alt_m = self.get_parameter('home_alt_m').value
        for i in range(5):
            self._send_origin()
            time.sleep(0.1)

        # Subscribe to OpenVINS odometry
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.sub = self.create_subscription(Odometry, vio_topic, self._odom_cb, qos)

        # Send origin + dummy vision at 4Hz to prime EKF until real VIO arrives.
        # OpenVINS won't publish until it gets IMU excitation (movement), but
        # ArduPilot needs vision data to let EKF3 initialize and allow arming.
        self.origin_confirmed = False
        self.origin_timer = self.create_timer(0.25, self._prime_ekf)  # 4Hz
        self.prime_count = 0
        self.reset_counter = 0       # incremented when real VIO arrives

        # Stats
        self.msg_count = 0
        self.last_log_time = time.time()
        self.last_stamp = 0.0

        self.get_logger().info(
            f'VIO→ArduPilot bridge ready.\n'
            f'  VIO topic:  {vio_topic}\n'
            f'  MAVLink:    {conn_str}\n'
            f'  Velocity:   {self.send_velocity}\n'
            f'  Covariance: {self.send_covariance}')

    def _set_param(self, name, value, param_type=None):
        """Set an ArduPilot parameter via MAVLink PARAM_SET."""
        if param_type is None:
            # float by default (MAV_PARAM_TYPE_REAL32 = 9)
            param_type = mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        self.mav_conn.mav.param_set_send(
            self.mav_conn.target_system,
            self.mav_conn.target_component,
            name.encode('utf-8'),
            float(value),
            param_type,
        )
        time.sleep(0.05)  # small delay between param sets

    def _configure_ardupilot_params(self):
        """Auto-configure ArduPilot for VIO with ExternalNav yaw (no compass)."""
        self.get_logger().info('Auto-configuring ArduPilot params for VIO...')

        params = {
            # EKF3 sources — all from ExternalNav
            'EK3_SRC1_POSXY': 6,
            'EK3_SRC1_VELXY': 6,
            'EK3_SRC1_POSZ':  1,   # Baro for Z (more stable than VIO Z)
            'EK3_SRC1_VELZ':  6,
            'EK3_SRC1_YAW':   6,   # VIO yaw

            # CRITICAL: Disable compass completely to prevent "compass variance"
            # When YAW=6, compass disagrees with VIO yaw → variance → failsafe
            'COMPASS_ENABLE': 0,
            'COMPASS_USE':    0,
            'COMPASS_USE2':   0,
            'COMPASS_USE3':   0,

            # Single IMU lane to avoid "IMU0/1 not aligned"
            'EK3_IMU_MASK':   1,

            # Widen EKF gates for VIO noise tolerance
            'EK3_POS_I_GATE': 500,
            'EK3_VEL_I_GATE': 500,
            'EK3_YAW_I_GATE': 500,
            'EK3_YAW_NOISE':  1.0,   # default 0.5 — wider for VIO
            'EK3_POSNE_M_NSE': 0.5,  # external nav position noise
            'EK3_VELD_M_NSE':  0.7,  # external nav velocity noise

            # EKF variance failsafe — raise threshold so normal VIO noise
            # doesn't trigger LAND
            'FS_EKF_THRESH':  1.0,    # default 0.8 — raise to 1.0
            'FS_EKF_ACTION':  1,      # 1=LAND (keep safe, but now won't trigger easily)

            # VIO type
            'VISO_TYPE':      1,

            # GPS disabled
            'GPS_TYPE':       0,

            # EKF3 enabled
            'AHRS_EKF_TYPE':  3,
            'EK3_ENABLE':     1,
        }

        for name, value in params.items():
            self._set_param(name, value)
            self.get_logger().info(f'  Set {name} = {value}')

        self.get_logger().warn(
            '⚠ Parameters set. REBOOT ArduPilot for changes to take effect!\n'
            '  In MAVProxy: "reboot"   |   In QGC: Vehicle Setup → Reboot')

    def _send_origin(self):
        """Send SET_GPS_GLOBAL_ORIGIN + SET_HOME_POSITION so EKF can initialize."""
        lat = int(self.home_lat * 1e7)
        lon = int(self.home_lon * 1e7)
        alt = int(self.home_alt_m * 1000)  # mm

        self.mav_conn.mav.set_gps_global_origin_send(
            self.mav_conn.target_system,
            lat, lon, alt,
        )
        self.mav_conn.mav.set_home_position_send(
            self.mav_conn.target_system,
            lat, lon, alt,
            0, 0, 0,                   # x, y, z (local pos)
            [1.0, 0.0, 0.0, 0.0],     # q (w,x,y,z identity)
            0, 0, 0,                   # approach x, y, z
        )
        self.get_logger().info(
            f'Sent GPS origin & home: lat={self.home_lat}, lon={self.home_lon}, '
            f'alt={self.home_alt_m}m (CMAC) → system {self.mav_conn.target_system}')

    def _prime_ekf(self):
        """Send origin + dummy VISION_POSITION_ESTIMATE at 4Hz to prime EKF.

        ArduPilot needs external nav data before EKF3 can initialize.
        OpenVINS won't publish until it sees movement. We send stationary-
        at-origin with HIGH covariance so EKF accepts it without over-trusting.
        """
        if self.origin_confirmed:
            self.origin_timer.cancel()
            return

        self.prime_count += 1
        usec = int(time.time() * 1e6)

        # Primer covariance: position is moderate, but YAW is extremely high.
        # This tells ArduPilot "I'm roughly at origin but I have NO IDEA about heading."
        # When real VIO arrives with a known heading, EKF smoothly adopts it
        # via reset_counter — no yaw jump, no offset math needed.
        covariance = [0.0] * 21
        covariance[0]  = 1.0      # var_x  = 1 m²
        covariance[2]  = 1.0      # var_y  = 1 m²
        covariance[5]  = 1.0      # var_z  = 1 m²
        covariance[9]  = 0.1      # var_roll  = 0.1 rad²
        covariance[14] = 0.1      # var_pitch = 0.1 rad²
        covariance[20] = 99.0     # var_yaw   = 99 rad² (unknown heading)

        self.mav_conn.mav.vision_position_estimate_send(
            usec,
            0.0, 0.0, 0.0,     # x, y, z NED (stationary at origin)
            0.0, 0.0, 0.0,     # roll, pitch, yaw
            covariance,
            0,
        )

        # Also send zero velocity so EKF knows we're stationary
        self.mav_conn.mav.vision_speed_estimate_send(
            usec,
            0.0, 0.0, 0.0,
            [0.0] * 9,
            0,
        )

        # Resend origin every 3s (every 12th call at 4Hz)
        if self.prime_count % 12 == 1:
            self._send_origin()

        # Log progress every 5s
        if self.prime_count % 20 == 0:
            self.get_logger().info(
                f'Priming EKF... ({self.prime_count} msgs sent, '
                f'waiting for OpenVINS to initialize)')

        # Drain any ACKs
        while True:
            ack = self.mav_conn.recv_match(type='COMMAND_ACK', blocking=False)
            if not ack:
                break
            self.get_logger().info(f'Got ACK: cmd={ack.command} result={ack.result}')

    def _twist_cov_3x3(self, tc):
        """Extract upper-right triangle of velocity 3x3 from ROS 6x6 twist covariance."""
        # tc is 36-element row-major 6x6. We want the 3x3 upper-left (vx,vy,vz).
        # MAVLink VISION_SPEED_ESTIMATE takes 9 elements (full 3x3 row-major).
        return [tc[0], tc[1], tc[2],
                tc[6], tc[7], tc[8],
                tc[12], tc[13], tc[14]]

    def _odom_cb(self, msg: Odometry):
        """Convert OpenVINS odometry to MAVLink VISION_POSITION_ESTIMATE.

        OpenVINS initializes at its own (0,0,0) origin with an arbitrary yaw.
        We pass this directly to ArduPilot (with ENU→NED conversion).
        On the first message, reset_counter is incremented to tell the EKF
        to adopt this as its new reference frame — like setting a heading origin.
        No offset math needed because:
          - OpenVINS pos starts at ~(0,0,0) (its own init origin)
          - Primer yaw covariance was 99 rad² so EKF didn't anchor to yaw=0
          - reset_counter signals "new frame, re-initialize"
        """
        now_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # Avoid duplicate timestamps
        if now_sec <= self.last_stamp:
            return
        self.last_stamp = now_sec

        # Signal EKF to adopt VIO frame on first real message
        if not self.origin_confirmed:
            self.origin_confirmed = True
            self.reset_counter += 1
            self.get_logger().info(
                f'Real VIO data received — stopping primer, '
                f'reset_counter={self.reset_counter} (EKF will adopt VIO frame)')

        # --- Position: OpenVINS Z-up → NED Z-down ---
        x_ned = msg.pose.pose.position.x
        y_ned = -msg.pose.pose.position.y
        z_ned = -msg.pose.pose.position.z

        # --- Orientation: OpenVINS quat → NED euler ---
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        roll_ov, pitch_ov, yaw_ov = quat_to_euler(qx, qy, qz, qw)
        roll_ned = roll_ov
        pitch_ned = -pitch_ov
        yaw_ned = -yaw_ov

        # Timestamp in microseconds
        usec = int(now_sec * 1e6)

        # --- Send VISION_POSITION_ESTIMATE ---
        if self.send_covariance:
            # OpenVINS publishes a FULL 6x6 pose covariance.
            # Convert ROS 6x6 row-major → MAVLink upper-right triangle (21 elements).
            pc = msg.pose.covariance
            covariance = [0.0] * 21
            idx = 0
            for r in range(6):
                for c in range(r, 6):
                    covariance[idx] = pc[r * 6 + c]
                    idx += 1
            # Enforce minimum yaw variance for fast-turn noise
            MIN_YAW_VAR = 0.1
            if covariance[20] < MIN_YAW_VAR:
                covariance[20] = MIN_YAW_VAR
        else:
            covariance = [float('NaN')] * 21

        self.mav_conn.mav.vision_position_estimate_send(
            usec,
            x_ned, y_ned, z_ned,
            roll_ned, pitch_ned, yaw_ned,
            covariance,
            self.reset_counter,
        )

        # --- Send VISION_SPEED_ESTIMATE ---
        if self.send_velocity:
            vx_ned = msg.twist.twist.linear.x
            vy_ned = -msg.twist.twist.linear.y
            vz_ned = -msg.twist.twist.linear.z

            self.mav_conn.mav.vision_speed_estimate_send(
                usec,
                vx_ned, vy_ned, vz_ned,
                self._twist_cov_3x3(msg.twist.covariance),
                self.reset_counter,
            )

        # --- Logging ---
        self.msg_count += 1
        now = time.time()
        if now - self.last_log_time > 5.0:
            hz = self.msg_count / (now - self.last_log_time)
            self.get_logger().info(
                f'Sending VIO @ {hz:.1f} Hz | '
                f'pos=({x_ned:.2f}, {y_ned:.2f}, {z_ned:.2f}) '
                f'yaw={math.degrees(yaw_ned):.1f}°')
            self.msg_count = 0
            self.last_log_time = now


def main():
    rclpy.init()
    node = VIOToArduPilot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
