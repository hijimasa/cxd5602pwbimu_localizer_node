#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
import serial
import struct

from . import usb_reset

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose, TransformStamped

import tf2_ros

def hex_to_float(hex_str):
    """
    16進数文字列をIEEE 754形式の浮動小数点数に変換する関数
    """
    int_val = int(hex_str, 16)
    packed = struct.pack('I', int_val)
    return struct.unpack('f', packed)[0]

class SerialRosNode(Node):
    def __init__(self):
        super().__init__('serial_ros_node')

        # パラメータの宣言と取得
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('tf_parent_frame', 'world')
        self.declare_parameter('tf_child_frame', 'sensor')
        # The CP210x USB-serial bridge of the Spresense board can be left in a
        # state where nothing is received (typically after a 921600 baud raw
        # recording session). A USB-level reset before opening the port
        # recovers it; the board itself keeps running.
        self.declare_parameter('usb_reset', True)
        self.declare_parameter('usb_vid_pid', '10c4:ea60')

        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.tf_parent_frame = self.get_parameter('tf_parent_frame').get_parameter_value().string_value
        self.tf_child_frame = self.get_parameter('tf_child_frame').get_parameter_value().string_value
        do_usb_reset = self.get_parameter('usb_reset').get_parameter_value().bool_value
        usb_vid_pid = self.get_parameter('usb_vid_pid').get_parameter_value().string_value

        self.get_logger().info(f"Using serial port: {serial_port} at {baud_rate} baud")
        self.get_logger().info(f"TF frames: parent='{self.tf_parent_frame}', child='{self.tf_child_frame}'")

        if do_usb_reset:
            try:
                node_path = usb_reset.reset_by_vid_pid(usb_vid_pid)
                self.get_logger().info(f"USB-serial bridge reset ({node_path}); waiting for re-enumeration")
                time.sleep(2.0)
            except Exception as e:
                self.get_logger().warn(f"USB-serial bridge reset skipped: {e}")

        # パブリッシャの作成
        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.pose_pub = self.create_publisher(Pose, 'pose', 10)

        # tf2ブロードキャスターの作成
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # シリアルポートの初期化
        try:
            self.ser = serial.Serial(serial_port, baud_rate, timeout=0.1)
            self.get_logger().info(f"Serial port {serial_port} opened at {baud_rate} baud.")
        except Exception as e:
            self.get_logger().error(f"Error opening serial port: {e}")
            self.ser = None

        # タイマー：定期的にシリアルからデータ読み込み（例:10ms間隔）
        self.timer = self.create_timer(0.01, self.read_serial_data)

    def read_serial_data(self):
        if self.ser is None:
            return

        try:
            # 1行分読み込み（改行まで）
            line = self.ser.readline().decode('utf-8', errors='replace').strip()
            if not line:
                return
            # '#' で始まる行はファームウェアのメッセージ（較正モードなど）
            if line.startswith('#'):
                self.get_logger().info(line)
                return

            # カンマ区切りで分割（末尾の空文字は除外）
            parts = [p for p in line.split(',') if p]
            # 受信するデータは 1 (タイムスタンプ) + 1 (温度) + 3 (角速度) + 3 (加速度)
            # + 4 (クォータニオン) + 3 (速度) + 3 (位置) = 18 要素である前提
            if len(parts) != 18:
                self.get_logger().warn(f"Unexpected data length: {parts}")
                return

            # 各値の変換
            # タイムスタンプ（整数として読み取り）
            timestamp = int(parts[0], 16)

            # 角速度(x,y,z)と加速度(x,y,z)は浮動小数点数に変換
            temperature = hex_to_float(parts[1])  # noqa: F841 (not published yet)
            angular_velocity = [hex_to_float(p) for p in parts[2:5]]
            linear_acceleration = [hex_to_float(p) for p in parts[5:8]]

            # クォータニオン (w,x,y,z)
            quat_vals = [hex_to_float(p) for p in parts[8:12]]

            # 速度 (x,y,z) -- 必要に応じて別のメッセージで扱えます
            velocity = [hex_to_float(p) for p in parts[12:15]]  # noqa: F841 (not published yet)

            # 位置 (x,y,z)
            position = [hex_to_float(p) for p in parts[15:18]]

            # ROSの現在時刻をヘッダーにセット
            now = self.get_clock().now().to_msg()

            # IMUメッセージの作成
            imu_msg = Imu()
            imu_msg.header.stamp = now
            imu_msg.header.frame_id = "imu_link"
            # 角速度（rad/sとして）
            imu_msg.angular_velocity.x = angular_velocity[0]
            imu_msg.angular_velocity.y = angular_velocity[1]
            imu_msg.angular_velocity.z = angular_velocity[2]
            # 加速度（m/s^2として）
            imu_msg.linear_acceleration.x = linear_acceleration[0]
            imu_msg.linear_acceleration.y = linear_acceleration[1]
            imu_msg.linear_acceleration.z = linear_acceleration[2]
            # 方向（クォータニオン）
            imu_msg.orientation.w = quat_vals[0]
            imu_msg.orientation.x = quat_vals[1]
            imu_msg.orientation.y = quat_vals[2]
            imu_msg.orientation.z = quat_vals[3]
            # ※Covarianceは必要に応じて設定可能。ここでは不定の値（-1など）を設定しています。
            imu_msg.orientation_covariance[0] = -1
            imu_msg.angular_velocity_covariance[0] = -1
            imu_msg.linear_acceleration_covariance[0] = -1

            # Poseメッセージの作成（位置と向き）
            pose_msg = Pose()
            # 位置情報
            pose_msg.position.x = position[0]
            pose_msg.position.y = position[1]
            pose_msg.position.z = position[2]
            # 向きはIMUのクォータニオンを流用
            pose_msg.orientation.w = quat_vals[0]
            pose_msg.orientation.x = quat_vals[1]
            pose_msg.orientation.y = quat_vals[2]
            pose_msg.orientation.z = quat_vals[3]

            # パブリッシュ
            self.imu_pub.publish(imu_msg)
            self.pose_pub.publish(pose_msg)

            # TransformStampedメッセージを作成し、/tfにパブリッシュ
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = self.tf_parent_frame         # 基準フレーム
            t.child_frame_id = self.tf_child_frame           # センサーフレーム（Poseメッセージの位置姿勢）
            t.transform.translation.x = pose_msg.position.x
            t.transform.translation.y = pose_msg.position.y
            t.transform.translation.z = pose_msg.position.z
            t.transform.rotation.w = pose_msg.orientation.w
            t.transform.rotation.x = pose_msg.orientation.x
            t.transform.rotation.y = pose_msg.orientation.y
            t.transform.rotation.z = pose_msg.orientation.z

            self.tf_broadcaster.sendTransform(t)

            self.get_logger().debug(f"Published IMU and Pose (timestamp: {timestamp})")

        except Exception as e:
            self.get_logger().error(f"Error processing serial data: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SerialRosNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node...")
    finally:
        if node.ser is not None:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

