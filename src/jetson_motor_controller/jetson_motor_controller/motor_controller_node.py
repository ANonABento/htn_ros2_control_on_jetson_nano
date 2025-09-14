import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import Jetson.GPIO as GPIO
import time
import signal
import sys

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        
        # First, clean up any existing GPIO state
        self.reset_gpio()
        
        # Initialize GPIO
        self.setup_gpio()
        
        # Create subscriber
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
        
        self.get_logger().info('Motor Controller Node Started')
        
        # Set up signal handler for graceful shutdown
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
    def reset_gpio(self):
        """Reset GPIO state before starting"""
        try:
            GPIO.cleanup()
            self.get_logger().info('Initial GPIO cleanup completed')
        except Exception as e:
            self.get_logger().warn(f'Initial GPIO cleanup warning: {e}')
        
    def setup_gpio(self):
        """Setup GPIO pins for two L298N motor drivers"""
        try:
            # Set GPIO mode
            GPIO.setmode(GPIO.BOARD)
            GPIO.setwarnings(False)
            
            # Motor Driver 1 (Left Motor) - Using different pins to avoid conflicts
            self.MOTOR1_IN1 = 18  # GPIO18
            self.MOTOR1_IN2 = 16  # GPIO19
            self.MOTOR1_ENA = 32  # GPIO12 (PWM)
            
            # Motor Driver 2 (Right Motor)  
            self.MOTOR2_IN3 = 11  # GPIO20
            self.MOTOR2_IN4 = 13  # GPIO21
            self.MOTOR2_ENB = 33  # Changed from 13 to 16 to avoid conflicts
            
            # List all pins we'll use
            self.all_pins = [self.MOTOR1_IN1, self.MOTOR1_IN2, self.MOTOR1_ENA,
                            self.MOTOR2_IN3, self.MOTOR2_IN4, self.MOTOR2_ENB]
            
            self.get_logger().info(f'Using GPIO pins: {self.all_pins}')
            
            # Setup GPIO pins
            for pin in self.all_pins:
                GPIO.setup(pin, GPIO.OUT)
                GPIO.output(pin, GPIO.LOW)
                self.get_logger().info(f'Setup GPIO {pin}')
            
            # Setup PWM for speed control
            self.motor1_pwm = GPIO.PWM(self.MOTOR1_ENA, 1000)  # 1kHz frequency
            self.motor2_pwm = GPIO.PWM(self.MOTOR2_ENB, 1000)  # 1kHz frequency
            
            self.motor1_pwm.start(0)
            self.motor2_pwm.start(0)
            
            # Initialize PWM objects list for cleanup
            self.pwm_objects = [self.motor1_pwm, self.motor2_pwm]
            
            self.get_logger().info('GPIO setup complete')
            
        except Exception as e:
            self.get_logger().error(f'GPIO setup failed: {e}')
            raise
        
    def cmd_vel_callback(self, msg):
        """Process velocity commands"""
        try:
            linear_x = msg.linear.x
            angular_z = msg.angular.z
            
            # Calculate motor speeds (differential drive)
            wheel_base = 0.2  # Distance between wheels in meters
            max_speed = 100   # Maximum PWM value (0-100)
            
            # Calculate left and right wheel speeds
            left_speed = linear_x - (angular_z * wheel_base / 2)
            right_speed = linear_x + (angular_z * wheel_base / 2)
            
            # Scale to PWM range and clamp
            left_pwm = max(-max_speed, min(max_speed, left_speed * max_speed))
            right_pwm = max(-max_speed, min(max_speed, right_speed * max_speed))
            
            # Set motor directions and speeds
            self.set_motor_speed(1, left_pwm)   # Left motor (Motor 1)
            self.set_motor_speed(2, right_pwm)  # Right motor (Motor 2)
            
            self.get_logger().info(f'Motor speeds - Left: {left_pwm:.1f}, Right: {right_pwm:.1f}')
            
        except Exception as e:
            self.get_logger().error(f'Error in cmd_vel_callback: {e}')
        
    def set_motor_speed(self, motor, speed):
        """Set speed and direction for a specific motor"""
        try:
            if motor == 1:  # Left motor (Driver 1)
                if speed > 0:  # Forward
                    GPIO.output(self.MOTOR1_IN1, GPIO.HIGH)
                    GPIO.output(self.MOTOR1_IN2, GPIO.LOW)
                elif speed < 0:  # Backward
                    GPIO.output(self.MOTOR1_IN1, GPIO.LOW)
                    GPIO.output(self.MOTOR1_IN2, GPIO.HIGH)
                else:  # Stop
                    GPIO.output(self.MOTOR1_IN1, GPIO.LOW)
                    GPIO.output(self.MOTOR1_IN2, GPIO.LOW)
                
                self.motor1_pwm.ChangeDutyCycle(abs(speed))
                
            elif motor == 2:  # Right motor (Driver 2)
                if speed > 0:  # Forward
                    GPIO.output(self.MOTOR2_IN3, GPIO.HIGH)
                    GPIO.output(self.MOTOR2_IN4, GPIO.LOW)
                elif speed < 0:  # Backward
                    GPIO.output(self.MOTOR2_IN3, GPIO.LOW)
                    GPIO.output(self.MOTOR2_IN4, GPIO.HIGH)
                else:  # Stop
                    GPIO.output(self.MOTOR2_IN3, GPIO.LOW)
                    GPIO.output(self.MOTOR2_IN4, GPIO.LOW)
                    
                self.motor2_pwm.ChangeDutyCycle(abs(speed))
                
        except Exception as e:
            self.get_logger().error(f'Error setting motor {motor} speed: {e}')
    
    def stop_motors(self):
        """Stop both motors"""
        try:
            self.set_motor_speed(1, 0)
            self.set_motor_speed(2, 0)
            self.get_logger().info('Motors stopped')
        except Exception as e:
            self.get_logger().error(f'Error stopping motors: {e}')
    
    def cleanup_gpio(self):
        """Clean up GPIO resources"""
        try:
            self.get_logger().info('Starting GPIO cleanup...')
            
            # Stop motors first
            self.stop_motors()
            
            # Stop PWM objects
            if hasattr(self, 'pwm_objects'):
                for pwm in self.pwm_objects:
                    try:
                        pwm.stop()
                        self.get_logger().info('PWM stopped')
                    except Exception as e:
                        self.get_logger().warn(f'Error stopping PWM: {e}')
            
            # Set all pins to LOW before cleanup
            if hasattr(self, 'all_pins'):
                for pin in self.all_pins:
                    try:
                        GPIO.output(pin, GPIO.LOW)
                    except Exception as e:
                        self.get_logger().warn(f'Error setting pin {pin} LOW: {e}')
            
            # Clean up GPIO
            GPIO.cleanup()
            self.get_logger().info('GPIO cleanup completed successfully')
            
        except Exception as e:
            self.get_logger().error(f'Error during GPIO cleanup: {e}')
    
    def signal_handler(self, signum, frame):
        """Handle shutdown signals gracefully"""
        self.get_logger().info(f'Received signal {signum}, shutting down...')
        self.cleanup_gpio()
        rclpy.shutdown()
        sys.exit(0)
    
    def __del__(self):
        """Destructor to ensure cleanup"""
        try:
            self.cleanup_gpio()
        except:
            pass  # Ignore errors in destructor

def main(args=None):
    rclpy.init(args=args)
    
    motor_controller = None
    try:
        motor_controller = MotorController()
        rclpy.spin(motor_controller)
        
    except KeyboardInterrupt:
        print("\nKeyboard interrupt received, shutting down...")
        
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        
    finally:
        # Ensure cleanup happens
        if motor_controller:
            motor_controller.cleanup_gpio()
        
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
