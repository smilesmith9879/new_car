#!/usr/bin/env python3
# Sheikah AI Car Control - Camera Controller Module

import logging
import time
import threading
import cv2
import numpy as np
from datetime import datetime
import os
import base64
import json

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

class CameraController:
    """
    Controls the camera gimbal and video streaming
    """
    
    def __init__(self):
        """Initialize the camera controller"""
        logger.info("Initializing Camera Controller")
        
        # Camera settings
        self.resolution = (640, 480)
        self.framerate = 30
        self.camera = None
        self.is_streaming = False
        self.stream_thread = None
        self.frame_buffer = None
        
        # Gimbal settings
        self.SERVO_CHANNELS = {
            'pan': 4,  # PCA9685 channel for pan servo
            'tilt': 5  # PCA9685 channel for tilt servo
        }
        self.pan_angle = 0  # Current pan angle (-90 to 90 degrees)
        self.tilt_angle = 0  # Current tilt angle (-45 to 45 degrees)
        
        # Servo pulse ranges (in microseconds)
        self.SERVO_MIN_PULSE = 1000  # 1ms pulse (0 degrees)
        self.SERVO_MAX_PULSE = 2000  # 2ms pulse (180 degrees)
        
        # Initialize hardware if available
        if HARDWARE_AVAILABLE:
            try:
                # Initialize I2C bus and PCA9685 for servos
                i2c = busio.I2C(board.SCL, board.SDA)
                self.pca = PCA9685(i2c)
                self.pca.frequency = 50  # Set PWM frequency to 50Hz
                
                # Initialize camera
                self.camera = cv2.VideoCapture(0)
                self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.resolution[0])
                self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolution[1])
                self.camera.set(cv2.CAP_PROP_FPS, self.framerate)
                
                # Center the gimbal
                self.set_gimbal_angle('pan', 0)
                self.set_gimbal_angle('tilt', 0)
                
                logger.info("Camera hardware initialized successfully")
            except Exception as e:
                logger.error(f"Failed to initialize camera hardware: {e}")
                HARDWARE_AVAILABLE = False
        
        # Initialize simulation camera if hardware not available
        if not HARDWARE_AVAILABLE:
            self._init_simulation_camera()
    
    def _init_simulation_camera(self):
        """Initialize a simulated camera for testing"""
        logger.info("Initializing simulation camera")
        
        # Create a blank frame for simulation
        self.frame_buffer = np.zeros((self.resolution[1], self.resolution[0], 3), dtype=np.uint8)
        
        # Start a thread to update the simulated camera frame
        self.sim_thread = threading.Thread(target=self._update_simulation, daemon=True)
        self.sim_thread.start()
    
    def _update_simulation(self):
        """Update the simulated camera frame"""
        while True:
            # Create a simulated frame with a grid pattern
            frame = np.zeros((self.resolution[1], self.resolution[0], 3), dtype=np.uint8)
            
            # Add grid lines
            grid_size = 20
            color = (0, 100, 200)  # Sheikah blue color
            
            # Horizontal grid lines
            for y in range(0, self.resolution[1], grid_size):
                cv2.line(frame, (0, y), (self.resolution[0], y), color, 1)
            
            # Vertical grid lines
            for x in range(0, self.resolution[0], grid_size):
                cv2.line(frame, (x, 0), (x, self.resolution[1]), color, 1)
            
            # Add timestamp
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            cv2.putText(frame, timestamp, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # Add pan/tilt angles
            angle_text = f"Pan: {self.pan_angle}°, Tilt: {self.tilt_angle}°"
            cv2.putText(frame, angle_text, (10, self.resolution[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # Add Sheikah eye in the center
            center_x, center_y = self.resolution[0] // 2, self.resolution[1] // 2
            radius = 40
            cv2.circle(frame, (center_x, center_y), radius, color, 2)
            cv2.circle(frame, (center_x, center_y), radius // 3, color, 2)
            cv2.line(frame, (center_x, center_y - radius // 3), (center_x, center_y - radius), color, 2)
            
            # Update the frame buffer
            self.frame_buffer = frame
            
            # Sleep to simulate framerate
            time.sleep(1 / self.framerate)
    
    def set_gimbal_angle(self, control, angle):
        """
        Set the gimbal angle for pan or tilt
        
        Args:
            control (str): 'pan' or 'tilt'
            angle (int): Angle in degrees
                - Pan: -90 to 90 degrees
                - Tilt: -45 to 45 degrees
        
        Returns:
            bool: Success status
        """
        logger.info(f"Setting {control} angle to {angle} degrees")
        
        # Validate control type
        if control not in ['pan', 'tilt']:
            logger.error(f"Invalid control: {control}")
            return False
        
        # Validate angle range
        if control == 'pan' and not -90 <= angle <= 90:
            logger.error(f"Pan angle out of range: {angle}")
            return False
        elif control == 'tilt' and not -45 <= angle <= 45:
            logger.error(f"Tilt angle out of range: {angle}")
            return False
        
        # Update current angle
        if control == 'pan':
            self.pan_angle = angle
        else:
            self.tilt_angle = angle
        
        # Set servo position if hardware is available
        if HARDWARE_AVAILABLE:
            try:
                # Convert angle to pulse width
                # Map angle range to pulse width range:
                # Pan: -90 to 90 degrees -> 1000 to 2000 microseconds
                # Tilt: -45 to 45 degrees -> 1000 to 2000 microseconds
                if control == 'pan':
                    pulse_width = self._map_value(angle, -90, 90, self.SERVO_MIN_PULSE, self.SERVO_MAX_PULSE)
                else:
                    pulse_width = self._map_value(angle, -45, 45, self.SERVO_MIN_PULSE, self.SERVO_MAX_PULSE)
                
                # Convert microseconds to duty cycle (0-65535)
                # For 50Hz PWM, period is 20ms (20000us)
                duty_cycle = int((pulse_width / 20000) * 65535)
                
                # Set PWM duty cycle
                self.pca.channels[self.SERVO_CHANNELS[control]].duty_cycle = duty_cycle
                
                logger.debug(f"Set {control} servo to {pulse_width}us (duty cycle: {duty_cycle})")
                return True
            except Exception as e:
                logger.error(f"Failed to set {control} angle: {e}")
                return False
        else:
            # In simulation mode, just update the angle
            logger.info(f"Simulation: Set {control} angle to {angle} degrees")
            return True
    
    def _map_value(self, value, in_min, in_max, out_min, out_max):
        """Map a value from one range to another"""
        return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    
    def start_streaming(self):
        """Start video streaming"""
        if self.is_streaming:
            logger.warning("Video streaming is already active")
            return False
        
        logger.info("Starting video streaming")
        self.is_streaming = True
        
        # Start streaming thread
        self.stream_thread = threading.Thread(target=self._stream_video, daemon=True)
        self.stream_thread.start()
        
        return True
    
    def stop_streaming(self):
        """Stop video streaming"""
        if not self.is_streaming:
            logger.warning("Video streaming is not active")
            return False
        
        logger.info("Stopping video streaming")
        self.is_streaming = False
        
        # Wait for streaming thread to end
        if self.stream_thread:
            self.stream_thread.join(timeout=1.0)
            self.stream_thread = None
        
        return True
    
    def _stream_video(self):
        """Video streaming thread function"""
        logger.info("Video streaming thread started")
        
        while self.is_streaming:
            try:
                if HARDWARE_AVAILABLE and self.camera:
                    # Read frame from camera
                    ret, frame = self.camera.read()
                    if not ret:
                        logger.error("Failed to read frame from camera")
                        time.sleep(0.1)
                        continue
                else:
                    # Use simulated frame
                    frame = self.frame_buffer.copy()
                
                # Process frame here if needed (e.g., add overlays, apply filters)
                
                # Convert frame to JPEG
                _, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
                
                # Store the frame for retrieval by WebRTC or other methods
                self.current_frame = jpeg.tobytes()
                
                # Sleep to maintain framerate
                time.sleep(1 / self.framerate)
            
            except Exception as e:
                logger.error(f"Error in video streaming: {e}")
                time.sleep(0.1)
    
    def get_current_frame(self):
        """Get the current camera frame as JPEG bytes"""
        if not self.is_streaming:
            logger.warning("Video streaming is not active")
            return None
        
        return self.current_frame if hasattr(self, 'current_frame') else None
    
    def get_frame_base64(self):
        """Get the current camera frame as base64 encoded JPEG"""
        frame = self.get_current_frame()
        if frame:
            return base64.b64encode(frame).decode('utf-8')
        return None
    
    def cleanup(self):
        """Clean up resources"""
        logger.info("Cleaning up camera controller resources")
        
        # Stop streaming
        if self.is_streaming:
            self.stop_streaming()
        
        # Release camera
        if HARDWARE_AVAILABLE and self.camera:
            self.camera.release()
        
        # Center gimbal before shutdown
        self.set_gimbal_angle('pan', 0)
        self.set_gimbal_angle('tilt', 0) 