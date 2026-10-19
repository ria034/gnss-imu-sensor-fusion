import rclpy
from rclpy.node import Node
from gps_msgs.msg import GpsMsg
import utm
import serial
from rclpy.time import Time


class Gps_Publisher(Node):
    def __init__(self):
        super().__init__('gps_publisher')
        self.declare_parameter('port', '/dev/ttyUSB0')
        port = self.get_parameter('port').get_parameter_value().string_value
        self.get_logger().info(f"Connecting to GPS on port: {port}")

        self.publisher = self.create_publisher(GpsMsg, "/gps", 10)

        try:
            self.serial_port = serial.Serial(port, baudrate=4800, timeout=1)
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            raise

        self.timer = self.create_timer(1, self.timer_callback)

    def destroy_node(self):
        if hasattr(self, 'serial_port') and self.serial_port.is_open:
            self.serial_port.close()
            self.get_logger().info("Serial port closed.")
        super().destroy_node()

    def timer_callback(self):
        line = self.serial_port.readline().decode("ascii", errors="ignore").strip()

        if not line.startswith("$GPGGA"):
            return

        measurements = line.split(",")

        if len(measurements) < 10:
            return

        if not measurements[2] or not measurements[4]:
            self.get_logger().warn("GPS fix not available")
            return

        try:
            lat_raw  = float(measurements[2])
            lat_dir  = measurements[3]
            lon_raw  = float(measurements[4])
            long_dir = measurements[5]
            hdop     = float(measurements[8]) if measurements[8] else 0.0
            altitude = float(measurements[9]) if measurements[9] else 0.0
            UTC      = measurements[1]

            latitude  = (lat_raw // 100) + (lat_raw % 100) / 60
            longitude = (lon_raw // 100) + (lon_raw % 100) / 60

            if lat_dir  == "S": latitude  = -latitude
            if long_dir == "W": longitude = -longitude

            utm_easting, utm_northing, zone, letter = utm.from_latlon(latitude, longitude)

            sec_data, n_sec_data = self.convert_gps_time_to_sec_nsec(UTC)

            msg = GpsMsg()
            msg.header.frame_id = "GPS1_Frame"
            msg.header.stamp    = Time(seconds=sec_data, nanoseconds=n_sec_data).to_msg()
            msg.latitude        = latitude
            msg.longitude       = longitude
            msg.altitude        = altitude
            msg.hdop            = hdop
            msg.utm_easting     = utm_easting
            msg.utm_northing    = utm_northing
            msg.zone            = int(zone)    # ensure int, match GpsMsg field type
            msg.letter          = str(letter)  # ensure str, match GpsMsg field type

            self.publisher.publish(msg)
            self.get_logger().info(
                f"Publishing GPS data: lat={latitude}, lon={longitude}, alt={altitude}, "
                f"UTC={UTC}, hdop={hdop}, easting={utm_easting}, "
                f"northing={utm_northing}, zone={zone}, letter={letter}"
            )

        except (ValueError, IndexError) as e:
            self.get_logger().warn(f"Error parsing GPS data: {e}")

    def convert_gps_time_to_sec_nsec(self, UTC):
        try:
            hour_data  = int(UTC[:2])
            min_data   = int(UTC[2:4])
            sec_data   = int(UTC[4:6])
            frac_data  = float(UTC[6:]) if len(UTC) > 6 else 0.0
            n_sec_data    = int(frac_data * 1e9)
            total_seconds = hour_data * 3600 + min_data * 60 + sec_data
            return total_seconds, n_sec_data
        except Exception as e:
            self.get_logger().warn(f"Error converting GPS time: {e}")
            return 0, 0


def main(args=None):
    rclpy.init(args=args)
    node = Gps_Publisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
