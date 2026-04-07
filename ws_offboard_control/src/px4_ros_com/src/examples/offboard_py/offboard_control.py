#!/usr/bin/env python3
import threading
import time
from enum import Enum
try:
    from pymavlink.dialects.v20 import common as mavlink2
    from pymavlink import mavutil
except ImportError:
    mavlink2 = None
    mavutil = None
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus


class OffboardState(Enum):
    TAKEOFF = 1
    WAYPOINTS = 2
    RETURN = 3
    LAND = 4

class OffboardControl(Node):
    def _start_gcs_heartbeat(self):
        if mavutil is not None:
            t = threading.Thread(target=self._send_gcs_heartbeat, daemon=True)
            t.start()
        else:
            self.get_logger().warning('pymavlink not found: GCS heartbeat will not be sent. Install with "pip install pymavlink" if needed or activate venv.')

    def _send_gcs_heartbeat(self):
        try:
            mav = mavutil.mavlink_connection('udpout:127.0.0.1:14580', source_system=255, source_component=0)
            while True:
                mav.mav.heartbeat_send(
                    mavlink2.MAV_TYPE_GCS,
                    mavlink2.MAV_AUTOPILOT_INVALID,
                    0, 0, 0
                )
                self.get_logger().info('GCS heartbeat sent')
                time.sleep(1)
        except Exception as e:
            self.get_logger().error(f'GCS heartbeat thread error: {e}')
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self) -> None:
        super().__init__('offboard_control_takeoff_and_land')

        # Start GCS heartbeat thread
        self._start_gcs_heartbeat()

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Create subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v1', self.vehicle_status_callback, qos_profile)

        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.takeoff_height = -10.0

        # Waypoint navigation variables
        self.waypoints = [
            (0.0, 0.0, self.takeoff_height),
            (20.0, 0.0, self.takeoff_height),
            (40.0, 0.0, self.takeoff_height),
            (60.0, 0.0, self.takeoff_height),
            (60.0, 20.0, self.takeoff_height),
            (40.0, 20.0, self.takeoff_height),
            (20.0, 20.0, self.takeoff_height),
            (0.0, 20.0, self.takeoff_height)
        ]
        self.current_wp_idx = 0
        self.reached_last_wp = False
        self.waypoint_threshold = 0.7  # meters, increased for robustness
        self.waypoint_dwell_counter = 0
        self.waypoint_dwell_required = 10  # number of timer cycles to dwell at waypoint
        self.offboard_armed = False
        self.setpoint_counter = 0
        # Takeoff/return logic
        self.state = OffboardState.TAKEOFF  # Use enum for state
        self.start_position_initialized = False
        self.start_x = None
        self.start_y = None
        self.start_z = None
        # Create a timer to publish control commands
        self.timer = self.create_timer(0.1, self.timer_callback)

    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position
        # Save start position on first callback
        if not self.start_position_initialized:
            self.start_x = vehicle_local_position.x
            self.start_y = vehicle_local_position.y
            self.start_z = vehicle_local_position.z
            self.start_position_initialized = True

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status

    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def land(self):
        """Switch to land mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self, x: float, y: float, z: float):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        #msg.yaw = 1.57079  # (90 degree)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info(f"Publishing position setpoints {[x, y, z]}")

    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def timer_callback(self) -> None:
        """Callback function for the timer."""
        self.publish_offboard_control_heartbeat_signal()

        # Wait for start position to be initialized
        if not self.start_position_initialized:
            self.get_logger().info("Waiting for start position initialization...")
            return

        # State machine: TAKEOFF -> WAYPOINTS -> RETURN -> LAND
        if self.state == OffboardState.TAKEOFF:
            # Go to (start_x, start_y, takeoff_height)
            x = self.start_x
            y = self.start_y
            z = self.takeoff_height
            self.publish_position_setpoint(x, y, z)
            if not self.offboard_armed:
                self.setpoint_counter += 1
                if self.setpoint_counter > 50:
                    self.engage_offboard_mode()
                    self.arm()
                    self.offboard_armed = True
            # Check if reached takeoff height
            dz = self.vehicle_local_position.z - z
            dx = self.vehicle_local_position.x - x
            dy = self.vehicle_local_position.y - y
            distance = (dx**2 + dy**2 + dz**2)**0.5
            self.get_logger().info(f"[TAKEOFF] Distance to takeoff point: {distance:.2f}")
            if distance < self.waypoint_threshold:
                self.waypoint_dwell_counter += 1
                if self.waypoint_dwell_counter >= self.waypoint_dwell_required:
                    self.state = OffboardState.WAYPOINTS
                    self.current_wp_idx = 0
                    self.waypoint_dwell_counter = 0
                    self.get_logger().info("Takeoff complete. Starting waypoint navigation.")
            else:
                self.waypoint_dwell_counter = 0
            return

        elif self.state == OffboardState.WAYPOINTS:
            # Navigate through waypoints
            x, y, z = self.waypoints[self.current_wp_idx]
            self.publish_position_setpoint(x, y, z)
            if not self.offboard_armed:
                self.setpoint_counter += 1
                if self.setpoint_counter > 50:
                    self.engage_offboard_mode()
                    self.arm()
                    self.offboard_armed = True
            self.get_logger().info(f"[WAYPOINTS] Current WP: {self.current_wp_idx}, Position: {[self.vehicle_local_position.x, self.vehicle_local_position.y, self.vehicle_local_position.z]}")
            dx = self.vehicle_local_position.x - x
            dy = self.vehicle_local_position.y - y
            dz = self.vehicle_local_position.z - z
            distance = (dx**2 + dy**2 + dz**2)**0.5
            self.get_logger().info(f"[WAYPOINTS] Distance to waypoint: {distance:.2f}")
            if distance < self.waypoint_threshold:
                self.waypoint_dwell_counter += 1
                if self.waypoint_dwell_counter >= self.waypoint_dwell_required:
                    if self.current_wp_idx < len(self.waypoints) - 1:
                        self.current_wp_idx += 1
                        self.waypoint_dwell_counter = 0
                        self.get_logger().info(f"Moving to waypoint {self.current_wp_idx}: {self.waypoints[self.current_wp_idx]}")
                        return
                    else:
                        self.state = OffboardState.RETURN
                        self.waypoint_dwell_counter = 0
                        self.get_logger().info("All waypoints complete. Returning to start position.")
            else:
                self.waypoint_dwell_counter = 0
            return

        elif self.state == OffboardState.RETURN:
            # Go back to (start_x, start_y, takeoff_height)
            x = self.start_x
            y = self.start_y
            z = self.takeoff_height
            self.publish_position_setpoint(x, y, z)
            self.get_logger().info(f"[RETURN] Returning to start: {[x, y, z]}")
            dx = self.vehicle_local_position.x - x
            dy = self.vehicle_local_position.y - y
            dz = self.vehicle_local_position.z - z
            distance = (dx**2 + dy**2 + dz**2)**0.5
            self.get_logger().info(f"[RETURN] Distance to start: {distance:.2f}")
            if distance < self.waypoint_threshold:
                self.waypoint_dwell_counter += 1
                if self.waypoint_dwell_counter >= self.waypoint_dwell_required:
                    self.state = OffboardState.LAND
                    self.get_logger().info("Returned to start. Initiating landing.")
            else:
                self.waypoint_dwell_counter = 0
            return

        elif self.state == OffboardState.LAND:
            self.land()
            exit(0)


def main(args=None) -> None:
    print('Starting offboard control node...')
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
