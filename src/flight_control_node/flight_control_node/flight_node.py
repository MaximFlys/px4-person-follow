import rclpy
import numpy as np
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from px4_msgs.msg import OffboardControlMode, VehicleCommand, TrajectorySetpoint, VehicleAttitude
from rclpy.qos import qos_profile_sensor_data

class FlightSubscriber(Node):

    def __init__(self):
        super().__init__('flight_node_subscriber')

        # Subscribers
        self.subscription = self.create_subscription(
            Detection2DArray,
            '/yolos_detector/detections',
            self.listener_callback,
            10)
        self.attitude_subscription = self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self.attitude_callback,
            qos_profile_sensor_data)

        # State — all initialized here to avoid uninitialized attribute errors
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0
        self.quaternion = [1.0, 0.0, 0.0, 0.0]
        self.yaw_rate = 0.0
        self.velocity_up = 0.0
        self.velocity_forward = 0.0

        # Detection watchdog — tracks time of last valid detection
        self.last_detection_time = self.get_clock().now()

        # Publishers (required for offboard control)
        self.offboard_control_mode_publisher_ = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.trajectory_setpoint_publisher_ = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.vehicle_command_publisher_ = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', 10)

        # Timer and counter
        self.offboard_setpoint_counter_ = 0
        timer_period = 0.1  # 100 milliseconds (10 Hz)
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def listener_callback(self, msg):
        msg_detections = len(msg.detections)
        self.get_logger().info('Number of detections : ' + str(msg_detections))
        self.flight_control_loop(msg)

    def attitude_callback(self, msg):
        self.quaternion = msg.q
        self.Qto3D()

    def Qto3D(self):
        # Quaternion in scalar-first (w, x, y, z) order — matches PX4 VehicleAttitude convention
        w = self.quaternion[0]
        x = self.quaternion[1]
        y = self.quaternion[2]
        z = self.quaternion[3]
        rotation_matrix = np.array([
            [1 - 2*y*y - 2*z*z, 2*x*y - 2*w*z, 2*x*z + 2*w*y],
            [2*x*y + 2*w*z,     1 - 2*x*x - 2*z*z, 2*y*z - 2*w*x],
            [2*x*z - 2*w*y,     2*y*z + 2*w*x, 1 - 2*x*x - 2*y*y]
        ])
        self.pitch = np.arcsin(-rotation_matrix[2, 0])
        self.roll  = np.arctan2(rotation_matrix[2, 1], rotation_matrix[2, 2])
        self.yaw   = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])

    def flight_control_loop(self, msg: Detection2DArray):
        if len(msg.detections) == 0:
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0
            self.yaw_rate = 0.0
        else:
            d = msg.detections[0]

            # Guard: skip if detection has no hypothesis results
            if not d.results:
                self.get_logger().warn('Detection received with no hypothesis results, skipping.')
                return

            class_id = str(d.results[0].hypothesis.class_id)

            if class_id == 'person' or class_id == '0':
                # vision_msgs/BoundingBox2D -> vision_msgs/Pose2D center
                #   -> vision_msgs/Point2D position -> float64 x, y
                center_x   = d.bbox.center.position.x
                center_z   = d.bbox.center.position.y
                bbox_height = d.bbox.size_y

                # Update watchdog on every valid person detection
                self.last_detection_time = self.get_clock().now()

                error_x = center_x - 320.0
                error_z = center_z - 240.0

                # Map horizontal error to yaw rate (rotation) instead of right/left strafing
                self.yaw_rate = error_x * 0.01 

                # NED frame: positive Z = DOWN.
                # Person in lower half of frame -> positive error_z -> positive self.z
                # -> drone descends -> person moves toward center. Sign is intentional.
                

                # Proportional forward velocity — closes distance smoothly
                if bbox_height < 120:
                    self.velocity_forward = (120.0 - bbox_height) * 0.05
                else:
                    self.velocity_forward = 0.0

                # Rotate body-frame forward velocity into NED world frame using current yaw
                self.x = self.velocity_forward * np.cos(self.yaw)
                self.y = self.velocity_forward * np.sin(self.yaw)
                self.z = self.velocity_up

    def timer_callback(self):
        if self.offboard_setpoint_counter_ == 10:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            self.arm()

        # Detection watchdog: zero all velocities if no valid detection for > 1 second
        elapsed = (self.get_clock().now() - self.last_detection_time).nanoseconds / 1e9
        if elapsed > 1.0:
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0
            self.yaw_rate = 0.0

        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint()

        if self.offboard_setpoint_counter_ < 11:
            self.offboard_setpoint_counter_ += 1

    def arm(self):
        self.get_logger().info('Arming vehicle')
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)

    def land(self):
        self.get_logger().info('Landing vehicle')
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

    def publish_trajectory_setpoint(self):
        msg = TrajectorySetpoint()
        msg.position     = [float('nan'), float('nan'), float('nan')]
        msg.velocity     = [float(self.x), float(self.y), float(self.z)]
        msg.acceleration = [float('nan'), float('nan'), float('nan')]
        msg.jerk         = [float('nan'), float('nan'), float('nan')]
        # Ignore absolute yaw position and command the yaw rate instead
        msg.yaw          = float('nan')
        msg.yawspeed     = float(self.yaw_rate)
        msg.timestamp    = self.get_clock().now().nanoseconds // 1000
        self.trajectory_setpoint_publisher_.publish(msg)

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position    = False
        msg.velocity    = True
        msg.acceleration = False
        msg.attitude    = False
        msg.body_rate   = False
        msg.timestamp   = self.get_clock().now().nanoseconds // 1000
        self.offboard_control_mode_publisher_.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1          = float(param1)
        msg.param2          = float(param2)
        msg.command         = command
        msg.target_system   = 1
        msg.target_component = 1
        msg.source_system   = 1
        msg.source_component = 191  # 191 = Companion Computer ID
        msg.from_external   = True
        msg.timestamp       = self.get_clock().now().nanoseconds // 1000
        self.vehicle_command_publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    flight_node_subscriber = FlightSubscriber()

    try:
        rclpy.spin(flight_node_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        flight_node_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()