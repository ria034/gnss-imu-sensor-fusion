#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from imu_msg.msg import IMUmsg
from sensor_msgs.msg import Imu, MagneticField
import serial
import serial.tools.list_ports
import argparse
from std_msgs.msg import Header
import math 

def find_imu_port():
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        if "USB" in p.device or "ttyUSB" in p.device or "ttyACM" in p.device:
            return p.device
    raise RuntimeError("No USB serial port found. Connect the IMU or specify --port manually.")

def euler_to_quaternion(yaw_deg, pitch_deg, roll_deg):
    """Convert Euler angles (degrees, ZYX convention) to quaternion (w, x, y, z)."""
    yaw   = math.radians(yaw_deg)
    pitch = math.radians(pitch_deg)
    roll  = math.radians(roll_deg)

    cy, sy = math.cos(yaw / 2),   math.sin(yaw / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cr, sr = math.cos(roll / 2),  math.sin(roll / 2)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return w, x, y, z

class IMU_Publisher(Node):
    def __init__(self, port):
        super().__init__('IMU_publisher')
        self.publisher = self.create_publisher(IMUmsg, "/imu/data", 10)
        self.get_logger().info(f"Connecting to IMU on port: {port}")
        self.serial_port = serial.Serial(port, baudrate=115200, timeout=1)
        self.timer = self.create_timer(1, self.timer_callback)

    def timer_callback(self):
        line = self.serial_port.readline().decode("ascii", errors="ignore").strip()
        if not line.startswith("$VNYMR"):
            return
        measurements = line.split(",")
        if len(measurements) < 13:
            return
        try:
            yaw            = float(measurements[1])
            pitch          = float(measurements[2])
            roll           = float(measurements[3])
            magnetic_x     = float(measurements[4])
            magnetic_y     = float(measurements[5])
            magnetic_z     = float(measurements[6])
            acceleration_x = float(measurements[7])
            acceleration_y = float(measurements[8])
            acceleration_z = float(measurements[9])
            angular_rate_x = float(measurements[10])
            angular_rate_y = float(measurements[11])
            angular_rate_z = float(measurements[12].split("*")[0])

            stamp = self.get_clock().now().to_msg()

            #  Euler  Quaternion conversion
            qw, qx, qy, qz = euler_to_quaternion(yaw, pitch, roll)
            

            # Build sensor_msgs/Imu
            msg_imu = Imu()
            msg_imu.header.frame_id = "IMU1_Frame"
            msg_imu.header.stamp = stamp
            msg_imu.linear_acceleration.x = acceleration_x
            msg_imu.linear_acceleration.y = acceleration_y
            msg_imu.linear_acceleration.z = acceleration_z
            msg_imu.angular_velocity.x = angular_rate_x
            msg_imu.angular_velocity.y = angular_rate_y
            msg_imu.angular_velocity.z = angular_rate_z
            # ── Quaternion orientation (published inside sensor_msgs/Imu) 
            msg_imu.orientation.w = qw
            msg_imu.orientation.x = qx
            msg_imu.orientation.y = qy
            msg_imu.orientation.z = qz
            
            # Build sensor_msgs/MagneticField
            msg_mag = MagneticField()
            msg_mag.header.frame_id = "IMU1_Frame"
            msg_mag.header.stamp = stamp
            msg_mag.magnetic_field.x = magnetic_x
            msg_mag.magnetic_field.y = magnetic_y
            msg_mag.magnetic_field.z = magnetic_z

            # Build IMUmsg
            msg = IMUmsg()
            msg.header.frame_id = "IMU1_Frame"
            msg.header.stamp = stamp
            msg.yaw       = yaw       # Euler angles (degrees)
            msg.pitch     = pitch
            msg.roll      = roll
            msg.imu       = msg_imu   # contains quaternion in .orientation
            msg.mag_field = msg_mag

            self.publisher.publish(msg)
            self.get_logger().info(
                f"IMU Published | Yaw:{yaw:.2f} Pitch:{pitch:.2f} Roll:{roll:.2f} "
                f"Quat:(w:{qw:.4f} x:{qx:.4f} y:{qy:.4f} z:{qz:.4f}) "
                f"Mag:({magnetic_x:.2f},{magnetic_y:.2f},{magnetic_z:.2f}) "
                f"Acc:({acceleration_x:.2f},{acceleration_y:.2f},{acceleration_z:.2f}) "
                f"Gyro:({angular_rate_x:.2f},{angular_rate_y:.2f},{angular_rate_z:.2f})"
            )
        except ValueError as e:
            self.get_logger().warn(f"Error parsing IMU data: {e}")

def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default=None, help="Serial port for IMU device (e.g. /dev/ttyUSB0). Auto-detected if not specified.")
    parsed_args, unknown = parser.parse_known_args()
    port = parsed_args.port if parsed_args.port else find_imu_port()
    print(f"Using port: {port}")
    rclpy.init(args=args)
    node = IMU_Publisher(port)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()