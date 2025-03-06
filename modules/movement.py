#!/usr/bin/env python3
# Sheikah AI Car Control - Movement Controller Module

import logging
import time
import threading
import math
try:
    import RPi.GPIO as GPIO
    from adafruit_pca9685 import PCA9685
    import board
    import busio
    HARDWARE_AVAILABLE = True
except ImportError:
    HARDWARE_AVAILABLE = False
    logging.warning("Hardware libraries not available, running in simulation mode")

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class MovementController:
    """
    Controls the movement of the four-wheel drive car using PCA9685 and L298N
    """
    
    def __init__(self):
        """Initialize the movement controller"""
        logger.info("Initializing Movement Controller")
        
        # Motor pins configuration
        self.MOTOR_PINS = {
            'front_left': {'pwm': 0, 'in1': 17, 'in2': 18},
            'front_right': {'pwm': 1, 'in1': 22, 'in2': 23},
            'rear_left': {'pwm': 2, 'in1': 24, 'in2': 25},
            'rear_right': {'pwm': 3, 'in1': 26, 'in2': 27}
        }
        
        # Movement state
        self.current_direction = 'stop'
        self.current_speed = 0
        self.max_speed = 4095  # Max PWM value for PCA9685
        
        # Initialize hardware if available
        if HARDWARE_AVAILABLE:
            try:
                # Initialize I2C bus and PCA9685
                i2c = busio.I2C(board.SCL, board.SDA)
                self.pca = PCA9685(i2c)
                self.pca.frequency = 50  # Set PWM frequency to 50Hz
                
                # Initialize GPIO
                GPIO.setmode(GPIO.BCM)
                for motor in self.MOTOR_PINS.values():
                    GPIO.setup(motor['in1'], GPIO.OUT)
                    GPIO.setup(motor['in2'], GPIO.OUT)
                
                logger.info("Hardware initialized successfully")
            except Exception as e:
                logger.error(f"Failed to initialize hardware: {e}")
                HARDWARE_AVAILABLE = False
        
        # Start watchdog timer to automatically stop motors if no commands received
        self.last_command_time = time.time()
        self.watchdog_thread = threading.Thread(target=self._watchdog, daemon=True)
        self.watchdog_thread.start()
    
    def move(self, direction, speed_percent):
        """
        Move the car in the specified direction at the specified speed
        
        Args:
            direction (str): 'forward', 'backward', 'left', 'right', or 'stop'
            speed_percent (int): Speed percentage (0-100)
        
        Returns:
            bool: Success status
        """
        logger.info(f"Moving {direction} at {speed_percent}% speed")
        
        # Update last command time for watchdog
        self.last_command_time = time.time()
        
        # Update current state
        self.current_direction = direction
        self.current_speed = speed_percent
        
        # Convert speed percentage to PWM value
        pwm_value = int((speed_percent / 100) * self.max_speed)
        
        # Set motor directions and speeds based on movement direction
        if direction == 'forward':
            self._set_motors_forward(pwm_value)
        elif direction == 'backward':
            self._set_motors_backward(pwm_value)
        elif direction == 'left':
            self._set_motors_left(pwm_value)
        elif direction == 'right':
            self._set_motors_right(pwm_value)
        elif direction == 'stop':
            self._set_motors_stop()
        else:
            logger.error(f"Invalid direction: {direction}")
            return False
        
        return True
    
    def _set_motors_forward(self, pwm_value):
        """Set all motors to move forward"""
        if HARDWARE_AVAILABLE:
            for motor_name, motor in self.MOTOR_PINS.items():
                GPIO.output(motor['in1'], GPIO.HIGH)
                GPIO.output(motor['in2'], GPIO.LOW)
                self.pca.channels[motor['pwm']].duty_cycle = pwm_value
        else:
            logger.info("Simulation: All motors moving forward")
    
    def _set_motors_backward(self, pwm_value):
        """Set all motors to move backward"""
        if HARDWARE_AVAILABLE:
            for motor_name, motor in self.MOTOR_PINS.items():
                GPIO.output(motor['in1'], GPIO.LOW)
                GPIO.output(motor['in2'], GPIO.HIGH)
                self.pca.channels[motor['pwm']].duty_cycle = pwm_value
        else:
            logger.info("Simulation: All motors moving backward")
    
    def _set_motors_left(self, pwm_value):
        """Set motors to turn left"""
        if HARDWARE_AVAILABLE:
            # Left side motors backward, right side motors forward
            for motor_name, motor in self.MOTOR_PINS.items():
                if 'left' in motor_name:
                    GPIO.output(motor['in1'], GPIO.LOW)
                    GPIO.output(motor['in2'], GPIO.HIGH)
                else:
                    GPIO.output(motor['in1'], GPIO.HIGH)
                    GPIO.output(motor['in2'], GPIO.LOW)
                self.pca.channels[motor['pwm']].duty_cycle = pwm_value
        else:
            logger.info("Simulation: Motors turning left")
    
    def _set_motors_right(self, pwm_value):
        """Set motors to turn right"""
        if HARDWARE_AVAILABLE:
            # Left side motors forward, right side motors backward
            for motor_name, motor in self.MOTOR_PINS.items():
                if 'left' in motor_name:
                    GPIO.output(motor['in1'], GPIO.HIGH)
                    GPIO.output(motor['in2'], GPIO.LOW)
                else:
                    GPIO.output(motor['in1'], GPIO.LOW)
                    GPIO.output(motor['in2'], GPIO.HIGH)
                self.pca.channels[motor['pwm']].duty_cycle = pwm_value
        else:
            logger.info("Simulation: Motors turning right")
    
    def _set_motors_stop(self):
        """Stop all motors"""
        if HARDWARE_AVAILABLE:
            for motor_name, motor in self.MOTOR_PINS.items():
                GPIO.output(motor['in1'], GPIO.LOW)
                GPIO.output(motor['in2'], GPIO.LOW)
                self.pca.channels[motor['pwm']].duty_cycle = 0
        else:
            logger.info("Simulation: All motors stopped")
    
    def _watchdog(self):
        """Watchdog timer to stop motors if no commands received for 5 seconds"""
        while True:
            if time.time() - self.last_command_time > 5:
                logger.warning("Watchdog triggered: No movement commands for 5 seconds")
                self._set_motors_stop()
            time.sleep(1)
    
    def cleanup(self):
        """Clean up GPIO and PCA9685 resources"""
        logger.info("Cleaning up movement controller resources")
        self._set_motors_stop()
        if HARDWARE_AVAILABLE:
            try:
                GPIO.cleanup()
            except Exception as e:
                logger.error(f"Error during GPIO cleanup: {e}") 