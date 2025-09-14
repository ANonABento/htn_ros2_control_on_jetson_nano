#!/usr/bin/env python3
import Jetson.GPIO as GPIO
import time
from typing import Tuple, Optional

class MotorDriver:
    def __init__(self, motor_pins: dict, pwm_frequency: int = 1000):
        self.motor_pins = motor_pins
        self.pwm_frequency = pwm_frequency
        self.pwm_instances = {}
        self.current_speeds = {}  # Track current speeds for gradual changes
        
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)
        
        for motor_name, pins in self.motor_pins.items():
            GPIO.setup(pins['pin1'], GPIO.OUT)
            GPIO.setup(pins['pin2'], GPIO.OUT)
            GPIO.setup(pins['enable'], GPIO.OUT)
            
            pwm = GPIO.PWM(pins['enable'], self.pwm_frequency)
            pwm.start(0)
            self.pwm_instances[motor_name] = pwm
            self.current_speeds[motor_name] = 0.0
            
            GPIO.output(pins['pin1'], GPIO.LOW)
            GPIO.output(pins['pin2'], GPIO.LOW)
    
    def set_motor_speed(self, motor_name: str, speed: float, gradual: bool = True):
        if motor_name not in self.motor_pins:
            raise ValueError(f"Unknown motor: {motor_name}")
        
        pins = self.motor_pins[motor_name]
        pwm = self.pwm_instances[motor_name]
        
        # Clamp speed for heat management (12V motors)
        speed = max(-100.0, min(100.0, speed))
        
        # Gradual acceleration for heat management
        if gradual:
            current_speed = self.current_speeds[motor_name]
            max_change = 5.0  # Max 20% change per call
            
            if abs(speed - current_speed) > max_change:
                if speed > current_speed:
                    speed = current_speed + max_change
                else:
                    speed = current_speed - max_change
        
        # Update current speed tracking
        self.current_speeds[motor_name] = speed
        
        # Set direction and PWM
        if speed > 0:
            GPIO.output(pins['pin1'], GPIO.HIGH)
            GPIO.output(pins['pin2'], GPIO.LOW)
            pwm.ChangeDutyCycle(abs(speed))
        elif speed < 0:
            GPIO.output(pins['pin1'], GPIO.LOW)
            GPIO.output(pins['pin2'], GPIO.HIGH)
            pwm.ChangeDutyCycle(abs(speed))
        else:
            GPIO.output(pins['pin1'], GPIO.LOW)
            GPIO.output(pins['pin2'], GPIO.LOW)
            pwm.ChangeDutyCycle(0)
    
    def set_differential_drive(self, linear_velocity: float, angular_velocity: float, 
                              max_speed: float = 60.0, wheel_base: float = 0.2):
        # Differential drive kinematics
        left_vel = linear_velocity - (angular_velocity * wheel_base / 2.0)
        right_vel = linear_velocity + (angular_velocity * wheel_base / 2.0)
        
        # Scale to motor speed range (conservative for 12V motors)
        max_vel = max(abs(left_vel), abs(right_vel))
        if max_vel > 0:
            # Scale factor to keep within max_speed limit
            scale = min(max_speed / max_vel, max_speed / 0.8)  # 0.8 m/s max reasonable speed
            left_speed = left_vel * scale * 8  # Reduced scaling for heat management
            right_speed = right_vel * scale * 8
        else:
            left_speed = 0
            right_speed = 0
        
        # Apply speeds with gradual acceleration
        self.set_motor_speed('left', left_speed, gradual=True)
        self.set_motor_speed('right', right_speed, gradual=True)
    
    def stop_all_motors(self):
        """Emergency stop - immediate stop without gradual deceleration"""
        for motor_name in self.motor_pins.keys():
            self.set_motor_speed(motor_name, 0, gradual=False)
            self.current_speeds[motor_name] = 0.0
    
    def get_motor_speeds(self):
        """Return current motor speeds for monitoring"""
        return self.current_speeds.copy()
    
    def cleanup(self):
        """Clean up GPIO resources"""
        self.stop_all_motors()
        time.sleep(0.1)  # Give time for motors to stop
        for pwm in self.pwm_instances.values():
            pwm.stop()
        GPIO.cleanup()
    
    def __del__(self):
        try:
            self.cleanup()
        except:
            pass
