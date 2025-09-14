#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, Bool
import signal
import sys
import os

sys.path.append(os.path.join(os.path.dirname(__file__), 'drivers'))
from motor_driver import MotorDriver

class MotorControllerNode(Node):
    def __init__(self):
        super().__init__('motor_controller')
        
        # L298N pin configuration - adjust these for your actual wiring
        self.motor_pins = {
            'left': {
                'pin1': 15,  # IN1
                'pin2': 16,  # IN2  
                'enable': 32  # ENA (PWM)
            },
            'right': {
                'pin1': 11,  # IN3
                'pin2': 13,  # IN4
                'enable': 33  # ENB (PWM)
            }
        }
        
        # Heat management parameters - conservative for 12V motors
        self.declare_parameters(
            namespace='',
            parameters=[
                ('max_speed', 10.0),      # Reduced for heat management
                ('wheel_base', 0.25),
                ('timeout_duration', 1.5), # Shorter timeout for safety
                ('publish_rate', 10.0)
            ]
        )
        
        self.max_speed = self.get_parameter('max_speed').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.timeout_duration = self.get_parameter('timeout_duration').value
        self.publish_rate = self.get_parameter('publish_rate').value
        
        # Initialize motor driver with 1kHz PWM for heat management
        try:
            self.motor_driver = MotorDriver(self.motor_pins, pwm_frequency=1000)
            self.get_logger().info(f"Motor driver initialized - Max speed: {self.max_speed}%")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize motor driver: {e}")
            sys.exit(1)
        
        # Subscribers and publishers
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.motor_speeds_pub = self.create_publisher(Float32MultiArray, 'motor_speeds', 10)
        self.motor_status_pub = self.create_publisher(Bool, 'motors_enabled', 10)
        
        # Timers
        self.timeout_timer = None
        self.status_timer = self.create_timer(1.0 / self.publish_rate, self.publish_status)
        
        # State variables
        self.last_cmd_time = self.get_clock().now()
        self.current_linear_vel = 0.0
        self.current_angular_vel = 0.0
        self.motors_enabled = True
        
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
        self.get_logger().info("Motor controller ready for 12V motors with heat management")
        self.get_logger().info("Subscribe to /cmd_vel to control motors")
    
    def cmd_vel_callback(self, msg: Twist):
        try:
            self.last_cmd_time = self.get_clock().now()
            self.current_linear_vel = msg.linear.x
            self.current_angular_vel = msg.angular.z
            
            # Apply heat management - limit velocities
            max_linear = 1.0  # m/s
            max_angular = 2.0  # rad/s
            
            linear_vel = max(-max_linear, min(max_linear, self.current_linear_vel))
            angular_vel = max(-max_angular, min(max_angular, self.current_angular_vel))
            
            self.motor_driver.set_differential_drive(
                linear_vel, angular_vel, self.max_speed, self.wheel_base
            )
            
            # Reset timeout timer
            if self.timeout_timer:
                self.timeout_timer.cancel()
            self.timeout_timer = self.create_timer(self.timeout_duration, self.timeout_callback)
            
            self.get_logger().debug(f"Motors: linear={linear_vel:.2f}, angular={angular_vel:.2f}")
            
        except Exception as e:
            self.get_logger().error(f"Error processing cmd_vel: {e}")
    
    def timeout_callback(self):
        self.get_logger().warn("Motor timeout - stopping for heat protection")
        self.motor_driver.stop_all_motors()
        self.current_linear_vel = 0.0
        self.current_angular_vel = 0.0
        if self.timeout_timer:
            self.timeout_timer.cancel()
            self.timeout_timer = None
    
    def publish_status(self):
        try:
            speeds_msg = Float32MultiArray()
            speeds_msg.data = [self.current_linear_vel, self.current_angular_vel]
            self.motor_speeds_pub.publish(speeds_msg)
            
            status_msg = Bool()
            status_msg.data = self.motors_enabled
            self.motor_status_pub.publish(status_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error publishing status: {e}")
    
    def signal_handler(self, signum, frame):
        self.get_logger().info("Shutdown signal - stopping motors safely")
        self.cleanup()
        sys.exit(0)
    
    def cleanup(self):
        try:
            if hasattr(self, 'motor_driver'):
                self.motor_driver.cleanup()
            self.get_logger().info("Motor controller cleaned up")
        except Exception as e:
            self.get_logger().error(f"Error during cleanup: {e}")

def main(args=None):
    rclpy.init(args=args)
    try:
        node = MotorControllerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'node' in locals():
            node.cleanup()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
