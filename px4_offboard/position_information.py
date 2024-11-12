import rclpy
from rclpy.node import Node
from px4_msgs.msg import SensorGps, VehicleLocalPosition, VehicleOdometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class VehicleGpsPositionListener(Node):
    def __init__(self):
        super().__init__('vehicle_global_position_listener')

        # Set the requested QoS profile
        qos_profile = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )

        # Subscription to GPS data for global lat, lon, alt
        self.gps_subscription = self.create_subscription(
            SensorGps,
            '/fmu/out/vehicle_gps_position',
            self.gps_listener_callback,
            qos_profile
        )

        # Subscription to Odometry data
        self.odometry_subscription = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.odometry_listener_callback,
            qos_profile
        )

        self.odometry_subscription = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.local_listener_callback,
            qos_profile
        )

        self.timer = self.create_timer(0.2, self.cmdloop_callback)

        # Initialize attributes for GPS data
        self.latitude_deg = None
        self.longitude_deg = None
        self.altitude_msl_m = None

        # Initialize attributes for odometry data
        self.local_x = None
        self.local_y = None
        self.local_z = None

        # Initialize attributes for odometry data
        self.odometry_x = None
        self.odometry_y = None
        self.odometry_z = None
        self.velocity_x = None
        self.velocity_y = None
        self.velocity_z = None

    def gps_listener_callback(self, msg):
        # Store only the latitude, longitude, and altitude from GPS data
        self.latitude_deg = msg.latitude_deg
        self.longitude_deg = msg.longitude_deg
        self.altitude_msl_m = msg.altitude_msl_m

    def local_listener_callback(self, msg):
        self.local_x = msg.x
        self.local_y = msg.y
        self.local_z = msg.z

    def odometry_listener_callback(self, msg):
        # Store odometry data
        self.odometry_x = msg.position[0]
        self.odometry_y = msg.position[1]
        self.odometry_z = msg.position[2]

        self.velocity_x = msg.velocity[0]
        self.velocity_y = msg.velocity[1]
        self.velocity_z = msg.velocity[2]

    def cmdloop_callback(self):
        # Log GPS data
        self.get_logger().info("\n\n\n\n")
        self.get_logger().info("===============================================")
        self.get_logger().info("RECEIVED VEHICLE GPS POSITION DATA")
        self.get_logger().info(f"lat: {self.latitude_deg}")
        self.get_logger().info(f"lon: {self.longitude_deg}")
        self.get_logger().info(f"alt: {self.altitude_msl_m}")

        # Log local position data
        self.get_logger().info("LOCAL POSITION DATA")
        self.get_logger().info(f"x: {self.local_x}")
        self.get_logger().info(f"y: {self.local_y}")
        self.get_logger().info(f"z: {self.local_z}")

        # Log odometry (velocity) data
        self.get_logger().info("ODOMETRY DATA")
        self.get_logger().info(f"odo_x: {self.odometry_x}")
        self.get_logger().info(f"odo_y: {self.odometry_y}")
        self.get_logger().info(f"odo_z: {self.odometry_z}")
        self.get_logger().info(f"velocity_x: {self.velocity_x}")
        self.get_logger().info(f"velocity_y: {self.velocity_y}")
        self.get_logger().info(f"velocity_z: {self.velocity_z}")
        self.get_logger().info("===============================================")

def main(args=None):
    rclpy.init(args=args)

    vehicle_gps_position_listener = VehicleGpsPositionListener()

    rclpy.spin(vehicle_gps_position_listener)

    vehicle_gps_position_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
