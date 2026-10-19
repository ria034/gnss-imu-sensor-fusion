import rclpy
from rclpy.node import Node
from gps_rtk_msgs.msg import Customrtk
import utm
import serial
import argparse
from std_msgs.msg import Header
from datetime import datetime
from rclpy.time import Time


class Gps_Publisher(Node):
    def __init__(self, port):
        # Initialize the Node with a name
        super().__init__('gps_publisher')

        # Create a publisher for GpsMsg
        self.publisher = self.create_publisher(Customrtk, "/gps", 10)

        # Initialize the serial port for GPS data
        self.serial_port = serial.Serial(port, baudrate=4800, timeout=1)

        # Set up a timer to call the callback function every second
        self.timer = self.create_timer(1, self.timer_callback)

    def timer_callback(self):
        # Read a line from the GPS serial stream
        line = self.serial_port.readline().decode("ascii", errors="ignore").strip()

        # Check if the line is a valid GPGGA sentence (GPS fix data)
        if not line.startswith("$GNGGA"):
            return

        # Split the NMEA sentence into fields
        measurements = line.split(",")

        # Check if we have enough fields
        if len(measurements) < 10:
            return
        
        # Skip if latitude or longitude is empty
        if not measurements[2] or not measurements[4]:
            self.get_logger().warn("GPS fix not available")
            return

        try:
            # Extract latitude, longitude, and other information
            lat = float(measurements[2])
            lat_dir = measurements[3]
            long = float(measurements[4])
            long_dir = measurements[5]
            fix_quality = int(measurements[6])
            hdop = float(measurements[8]) if measurements[8] else 0.0
            altitude = float(measurements[9]) if measurements[9] else 0.0
            UTC = measurements[1]  # GPS UTC time is in hhmmss.sss format

            # Convert latitude and longitude from DDMM.MMMM format to decimal degrees
            latitude = (lat // 100) + (lat % 100) / 60
            longitude = (long // 100) + (long % 100) / 60

            # Apply direction adjustments
            if lat_dir == "S":
                latitude = -latitude
            if long_dir == "W":
                longitude = -longitude

            # Convert latitude and longitude to UTM coordinates
            utm_easting, utm_northing, zone, letter = utm.from_latlon(latitude, longitude)

            # Convert GPS time (UTC) to seconds and nanoseconds
            sec_data, n_sec_data = self.convert_gps_time_to_sec_nsec(UTC)

            # Create a GpsMsg message
            msg = Customrtk()

            # Set the header with a frame ID and timestamp (using ROS time)
            msg.header = Header()
            msg.header.frame_id = "GPS1_Frame"
            # Create a ROS Time object with the seconds and nanoseconds from GPS time
            ros_time = Time(seconds=sec_data, nanoseconds=n_sec_data)
            msg.header.stamp = ros_time.to_msg()  # Set ROS time here

            # Set the GPS message fields
            msg.latitude = latitude
            msg.longitude = longitude
            msg.altitude = altitude
            msg.hdop = hdop
            msg.utm_easting = utm_easting
            msg.utm_northing = utm_northing
            msg.zone = zone
            msg.letter = letter
            msg.fix_quality = fix_quality 

            # Publish the message
            self.publisher.publish(msg)
            self.get_logger().info(f"Publishing GPS data: lat:{latitude}, long:{longitude}, alt: {altitude}, GPS Time: {UTC} , hdop:{hdop}, utm_easting:{utm_easting}, utm_northing:{utm_northing} zone:{zone}, letter:{letter}, fix_quality:{fix_quality}")
        
        except (ValueError, IndexError) as e:
            self.get_logger().warn(f"Error parsing GPS data: {e}")
            return

    def convert_gps_time_to_sec_nsec(self, UTC):
        """
        Converts GPS UTC time (hhmmss.sss) to seconds and nanoseconds.
        """
        try:
            # Extract hours, minutes, and seconds
            hour_data = int(UTC[:2])   # Extract hours (hh)
            min_data = int(UTC[2:4])   # Extract minutes (mm)
            sec_data = int(UTC[4:6])   # Extract seconds (ss)

            # Extract fractional seconds and convert to nanoseconds
            frac_data = float(UTC[6:]) if len(UTC) > 6 else 0.0
            n_sec_data = int(frac_data * 1e9)  # Convert to nanoseconds

            # Total seconds calculation
            total_seconds = hour_data * 3600 + min_data * 60 + sec_data

            return total_seconds, n_sec_data
        except Exception as e:
            self.get_logger().warn(f"Error converting GPS time: {e}")
            return 0, 0  # Default to 0 seconds and 0 nanoseconds if error occurs


def main(args=None):
    # Parse command-line arguments for the serial port
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/pts/4", help="Serial port for GPS device")
    parsed_args, unknown = parser.parse_known_args()

    # Initialize the ROS 2 node
    rclpy.init(args=args)

    # Create an instance of the Gps_Publisher node
    node = Gps_Publisher(parsed_args.port)

    # Keep the node running
    rclpy.spin(node)

    # Destroy the node when spinning is done
    rclpy.shutdown()
    
if __name__=='__main__':
	main()

