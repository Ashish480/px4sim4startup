#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

# Import PX4 message types
from px4_msgs.msg import VehicleCommand, OffboardControlMode, TrajectorySetpoint, VehicleStatus

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty


msg = """
PX4 Drone Teleop Control
----------------------------
This node takes keypresses from the keyboard and controls a PX4 drone.
Using the arrow keys and WASD you have Mode 2 RC controls.

MOVEMENT CONTROLS:
W: Up (+Z in NED frame)
S: Down (-Z in NED frame)
A: Yaw Left
D: Yaw Right
Up Arrow: Forward (+X in NED frame)
Down Arrow: Backward (-X in NED frame)
Left Arrow: Left (+Y in NED frame)
Right Arrow: Right (-Y in NED frame)

COMMAND CONTROLS:
SPACE: Arm/Disarm
O: Enable Offboard Control Mode
T: Takeoff
L: Land
Esc: Exit

Keyboard control steps:
1. Press SPACE to arm the drone
2. Press O to enable offboard control
3. Press T to takeoff or use W to ascend
4. Use movement keys to control the drone
5. Press L to land
6. Press SPACE to disarm
"""

moveBindings = {
    'w': (0, 0, -0.5, 0),     # Up (+Z in PX4 NED frame is negative)
    's': (0, 0, 0.5, 0),      # Down (-Z in PX4 NED frame is positive)
    'a': (0, 0, 0, -0.1),     # Yaw Left
    'd': (0, 0, 0, 0.1),      # Yaw Right
    '\x1b[A': (0.5, 0, 0, 0),  # Up Arrow - Forward (+X)
    '\x1b[B': (-0.5, 0, 0, 0), # Down Arrow - Backward (-X)
    '\x1b[C': (0, -0.5, 0, 0), # Right Arrow - Right (-Y in NED)
    '\x1b[D': (0, 0.5, 0, 0),  # Left Arrow - Left (+Y in NED)
}


def getKey(settings):
    if sys.platform == 'win32':
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        if key == '\x1b':  # if the first character is \x1b, we might be dealing with an arrow key
            additional_chars = sys.stdin.read(2)  # read the next two characters
            key += additional_chars  # append these characters to the key
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


class PX4Teleop(Node):
    def __init__(self):
        super().__init__('px4_teleop')
        
        # QoS profile for the publishers
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        
        # Subscribers
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        
        # Initialize variables
        self.armed = False
        self.nav_state = 0
        self.position = [0.0, 0.0, -2.0]  # x, y, z in NED frame (negative z is up)
        self.yaw = 0.0
        self.speed = 0.5
        
        # Timer for publishing control mode
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info('PX4 Teleop Control started')

    def vehicle_status_callback(self, msg):
        """Callback function for vehicle_status topic subscriber."""
        self.armed = msg.arming_state == 2  # ARMING_STATE_ARMED
        self.nav_state = msg.nav_state
        
    def timer_callback(self):
        """Timer callback for publishing control mode."""
        self.publish_offboard_control_mode()
        
    
    def publish_offboard_control_mode(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.offboard_control_mode_publisher.publish(msg)
    
    def publish_trajectory_setpoint(self):
        """Publish trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position[0] = float(self.position[0])  # x
        msg.position[1] = float(self.position[1])  # y
        msg.position[2] = float(self.position[2])  # z
        msg.yaw = float(self.yaw)
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info(f"Position: x={self.position[0]:.2f}, y={self.position[1]:.2f}, z={self.position[2]:.2f}, yaw={self.yaw:.2f}")
    
    def publish_vehicle_command(self, command, param1=0.0, param2=0.0, param3=0.0, param4=0.0, param5=0.0, param6=0.0, param7=0.0):
        """Publish vehicle command."""
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.param3 = float(param3)
        msg.param4 = float(param4)
        msg.param5 = float(param5)
        msg.param6 = float(param6)
        msg.param7 = float(param7)
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_command_publisher.publish(msg)
    
    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info('ARM command sent')
    
    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self.get_logger().info('DISARM command sent')
    
    def engage_offboard_mode(self):
        """Switch to offboard control mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        self.get_logger().info('OFFBOARD mode command sent')
    
    def takeoff(self):
        """Send takeoff command."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.5)
        self.get_logger().info('TAKEOFF command sent')
    
    def land(self):
        """Send land command."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info('LAND command sent')
    
    def process_key(self, key):
        """Process keyboard input."""
        if key in moveBindings.keys():
            # Update position based on key
            self.position[0] += moveBindings[key][0]
            self.position[1] += moveBindings[key][1]
            self.position[2] += moveBindings[key][2]
            self.yaw += moveBindings[key][3]
            self.publish_trajectory_setpoint()
        
        elif key == ' ':  # Space - arm/disarm
            if self.armed:
                self.disarm()
            else:
                self.arm()
        
        elif key == 'o':  # 'o' - offboard mode
            self.engage_offboard_mode()
        
        elif key == 't':  # 't' - takeoff
            self.takeoff()
        
        elif key == 'l':  # 'l' - land
            self.land()
        
        elif key == '\x1b':  # Esc - exit
            return False
        
        return True


def main():
    rclpy.init()
    teleop = PX4Teleop()
    
    settings = saveTerminalSettings()
    
    try:
        print(msg)
        
        running = True
        while running:
            key = getKey(settings)
            
            if key == '\x03':  # Ctrl-C
                break
            
            running = teleop.process_key(key)
    
    except Exception as e:
        print(f"Error: {e}")
    
    finally:
        # Reset position when exiting
        teleop.position = [0.0, 0.0, 0.0]
        teleop.yaw = 0.0
        teleop.publish_trajectory_setpoint()
        
        restoreTerminalSettings(settings)
        teleop.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
