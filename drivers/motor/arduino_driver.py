"""
Arduino UART Driver for Raspberry Pi
Handles serial communication with Arduino Uno
Supports 5 IR sensors as camera fallback
"""

import serial
import json
import threading
import time
import logging
from typing import Optional, Dict, Callable

logger = logging.getLogger(__name__)


class ArduinoDriver:
    """
    Arduino UART Communication Driver
    Sends commands and receives sensor data via Serial
    Supports camera fallback to IR sensors
    """
    
    def __init__(self, port: str = '/dev/ttyACM0', baudrate: int = 115200):
        """
        Initialize Arduino driver
        
        Args:
            port: Serial port path (e.g., /dev/ttyACM0, /dev/ttyUSB0)
            baudrate: Communication baudrate (default: 115200)
        """
        self.port = port
        self.baudrate = baudrate
        self.serial: Optional[serial.Serial] = None
        
        # Current state
        self.connected = False
        self.left_speed = 0
        self.right_speed = 0
        
        # Sensor data
        self.sensor_data = {
            'line': [0] * 5,  # Changed from 8 to 5
            'line_pos': 0,
            'distance': 0.0,
            'left_speed': 0,
            'right_speed': 0,
            'uptime': 0,
            'ir_enabled': False  # NEW: IR sensor status
        }
        
        # Camera status tracking
        self.camera_working = True
        self.ir_fallback_active = False
        
        # Callbacks
        self.sensor_callback: Optional[Callable] = None
        
        # Thread control
        self.running = False
        self.read_thread: Optional[threading.Thread] = None
        
        # Try to connect
        self.connect()
    
    def connect(self) -> bool:
        """
        Connect to Arduino via UART
        
        Returns:
            True if connected successfully
        """
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1,
                write_timeout=1
            )
            
            # Wait for Arduino to boot
            time.sleep(2)
            
            # Flush buffers
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()
            
            # Send ping to check connection
            response = self.send_command({'cmd': 'PING'})
            if response and response.get('status') == 'ok':
                self.connected = True
                logger.info(f"Connected to Arduino on {self.port}")
                
                # Start reading thread
                self.running = True
                self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
                self.read_thread.start()
                
                return True
            else:
                logger.warning("Arduino did not respond to PING")
                return False
                
        except serial.SerialException as e:
            logger.error(f"Failed to connect to Arduino: {e}")
            self.connected = False
            return False
        except Exception as e:
            logger.error(f"Unexpected error connecting to Arduino: {e}")
            self.connected = False
            return False
    
    def disconnect(self):
        """Disconnect from Arduino"""
        self.running = False
        if self.read_thread:
            self.read_thread.join(timeout=2.0)
        
        if self.serial and self.serial.is_open:
            self.serial.close()
        
        self.connected = False
        logger.info("Disconnected from Arduino")
    
    def send_command(self, command: Dict, wait_response: bool = True) -> Optional[Dict]:
        """
        Send JSON command to Arduino
        
        Args:
            command: Command dictionary
            wait_response: Wait for acknowledgment
            
        Returns:
            Response dictionary or None
        """
        if not self.connected or not self.serial:
            logger.warning("Cannot send command: Not connected to Arduino")
            return None
        
        try:
            # Serialize to JSON
            json_str = json.dumps(command) + '\n'
            
            # Send via serial
            self.serial.write(json_str.encode('utf-8'))
            self.serial.flush()
            
            logger.debug(f"Sent: {command}")
            
            # Wait for response if requested
            if wait_response:
                timeout = time.time() + 1.0
                while time.time() < timeout:
                    if self.serial.in_waiting > 0:
                        line = self.serial.readline().decode('utf-8').strip()
                        if line:
                            try:
                                response = json.loads(line)
                                logger.debug(f"Received: {response}")
                                return response
                            except json.JSONDecodeError:
                                logger.warning(f"Invalid JSON received: {line}")
                    time.sleep(0.01)
                
                logger.warning("Command response timeout")
                return None
            
            return {'status': 'sent'}
            
        except Exception as e:
            logger.error(f"Error sending command: {e}")
            return None
    
    def enable_ir_sensors(self):
        """Enable IR line sensors (fallback mode)"""
        logger.info("Enabling IR sensors (camera fallback mode)")
        response = self.send_command({'cmd': 'ENABLE_IR'})
        if response and response.get('status') == 'ok':
            self.ir_fallback_active = True
            logger.info("IR sensors enabled successfully")
            return True
        return False
    
    def disable_ir_sensors(self):
        """Disable IR line sensors (camera mode)"""
        logger.info("Disabling IR sensors (camera primary mode)")
        response = self.send_command({'cmd': 'DISABLE_IR'})
        if response and response.get('status') == 'ok':
            self.ir_fallback_active = False
            logger.info("IR sensors disabled successfully")
            return True
        return False
    
    def set_camera_status(self, working: bool):
        """
        Update camera status and switch to IR fallback if needed
        
        Args:
            working: True if camera is working, False if failed
        """
        if working != self.camera_working:
            self.camera_working = working
            
            if not working:
                logger.warning("Camera failure detected! Switching to IR sensors...")
                self.enable_ir_sensors()
            else:
                logger.info("Camera restored. Switching back from IR sensors...")
                self.disable_ir_sensors()
    
    def set_motors(self, left_speed: int, right_speed: int):
        """
        Set motor speeds
        
        Args:
            left_speed: -255 to 255 (negative = backward)
            right_speed: -255 to 255 (negative = backward)
        """
        left_speed = max(-255, min(255, left_speed))
        right_speed = max(-255, min(255, right_speed))
        
        self.left_speed = left_speed
        self.right_speed = right_speed
        
        command = {
            'cmd': 'MOVE',
            'left': left_speed,
            'right': right_speed
        }
        
        self.send_command(command, wait_response=False)
    
    def forward(self, speed: int = 200):
        """Move forward"""
        self.set_motors(speed, speed)
    
    def backward(self, speed: int = 200):
        """Move backward"""
        self.set_motors(-speed, -speed)
    
    def turn_left(self, speed: int = 150):
        """Turn left (rotate in place)"""
        self.set_motors(-speed, speed)
    
    def turn_right(self, speed: int = 150):
        """Turn right (rotate in place)"""
        self.set_motors(speed, -speed)
    
    def stop(self):
        """Stop motors"""
        self.set_motors(0, 0)
        command = {'cmd': 'STOP'}
        self.send_command(command, wait_response=False)
    
    def get_speeds(self):
        """Get current motor speeds"""
        return (self.left_speed, self.right_speed)
    
    def get_sensor_data(self) -> Dict:
        """Get latest sensor data"""
        return self.sensor_data.copy()
    
    def is_ir_fallback_active(self) -> bool:
        """Check if IR fallback mode is active"""
        return self.ir_fallback_active
    
    def set_sensor_callback(self, callback: Callable):
        """
        Set callback function for sensor data updates
        
        Args:
            callback: Function to call when new sensor data arrives
        """
        self.sensor_callback = callback
    
    def _read_loop(self):
        """Background thread to read sensor data from Arduino"""
        logger.info("Arduino read loop started")
        
        while self.running:
            try:
                if self.serial and self.serial.in_waiting > 0:
                    line = self.serial.readline().decode('utf-8').strip()
                    
                    if line:
                        try:
                            data = json.loads(line)
                            
                            # Check if it's sensor data
                            if 'line' in data and 'distance' in data:
                                self.sensor_data = data
                                
                                # Update IR fallback status
                                if 'ir_enabled' in data:
                                    self.ir_fallback_active = data['ir_enabled']
                                
                                # Call callback if set
                                if self.sensor_callback:
                                    self.sensor_callback(data)
                            
                            # Log other messages
                            elif 'status' in data:
                                if data['status'] == 'ready':
                                    logger.info(f"Arduino ready: {data}")
                                elif data['status'] == 'error':
                                    logger.error(f"Arduino error: {data.get('message', 'Unknown')}")
                        
                        except json.JSONDecodeError:
                            logger.debug(f"Non-JSON line: {line}")
                
                time.sleep(0.01)
                
            except Exception as e:
                logger.error(f"Error in read loop: {e}")
                time.sleep(0.1)
        
        logger.info("Arduino read loop stopped")
    
    def cleanup(self):
        """Cleanup resources"""
        self.stop()
        self.disconnect()
        logger.info("Arduino driver cleaned up")
    
    def __del__(self):
        """Destructor"""
        try:
            self.cleanup()
        except:
            pass
    
    def connect(self) -> bool:
        """
        Connect to Arduino via UART
        
        Returns:
            True if connected successfully
        """
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1,
                write_timeout=1
            )
            
            # Wait for Arduino to boot
            time.sleep(2)
            
            # Flush buffers
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()
            
            # Send ping to check connection
            response = self.send_command({'cmd': 'PING'})
            if response and response.get('status') == 'ok':
                self.connected = True
                logger.info(f"Connected to Arduino on {self.port}")
                
                # Start reading thread
                self.running = True
                self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
                self.read_thread.start()
                
                return True
            else:
                logger.warning("Arduino did not respond to PING")
                return False
                
        except serial.SerialException as e:
            logger.error(f"Failed to connect to Arduino: {e}")
            self.connected = False
            return False
        except Exception as e:
            logger.error(f"Unexpected error connecting to Arduino: {e}")
            self.connected = False
            return False
    
    def disconnect(self):
        """Disconnect from Arduino"""
        self.running = False
        if self.read_thread:
            self.read_thread.join(timeout=2.0)
        
        if self.serial and self.serial.is_open:
            self.serial.close()
        
        self.connected = False
        logger.info("Disconnected from Arduino")
    
    def send_command(self, command: Dict, wait_response: bool = True) -> Optional[Dict]:
        """
        Send JSON command to Arduino
        
        Args:
            command: Command dictionary (e.g., {'cmd': 'MOVE', 'left': 200, 'right': 200})
            wait_response: Wait for acknowledgment
            
        Returns:
            Response dictionary or None
        """
        if not self.connected or not self.serial:
            logger.warning("Cannot send command: Not connected to Arduino")
            return None
        
        try:
            # Serialize to JSON
            json_str = json.dumps(command) + '\n'
            
            # Send via serial
            self.serial.write(json_str.encode('utf-8'))
            self.serial.flush()
            
            logger.debug(f"Sent: {command}")
            
            # Wait for response if requested
            if wait_response:
                timeout = time.time() + 1.0  # 1 second timeout
                while time.time() < timeout:
                    if self.serial.in_waiting > 0:
                        line = self.serial.readline().decode('utf-8').strip()
                        if line:
                            try:
                                response = json.loads(line)
                                logger.debug(f"Received: {response}")
                                return response
                            except json.JSONDecodeError:
                                logger.warning(f"Invalid JSON received: {line}")
                    time.sleep(0.01)
                
                logger.warning("Command response timeout")
                return None
            
            return {'status': 'sent'}
            
        except Exception as e:
            logger.error(f"Error sending command: {e}")
            return None
    
    def set_motors(self, left_speed: int, right_speed: int):
        """
        Set motor speeds
        
        Args:
            left_speed: -255 to 255 (negative = backward)
            right_speed: -255 to 255 (negative = backward)
        """
        left_speed = max(-255, min(255, left_speed))
        right_speed = max(-255, min(255, right_speed))
        
        self.left_speed = left_speed
        self.right_speed = right_speed
        
        command = {
            'cmd': 'MOVE',
            'left': left_speed,
            'right': right_speed
        }
        
        self.send_command(command, wait_response=False)
    
    def forward(self, speed: int = 200):
        """Move forward"""
        self.set_motors(speed, speed)
    
    def backward(self, speed: int = 200):
        """Move backward"""
        self.set_motors(-speed, -speed)
    
    def turn_left(self, speed: int = 150):
        """Turn left (rotate in place)"""
        self.set_motors(-speed, speed)
    
    def turn_right(self, speed: int = 150):
        """Turn right (rotate in place)"""
        self.set_motors(speed, -speed)
    
    def stop(self):
        """Stop motors"""
        self.set_motors(0, 0)
        command = {'cmd': 'STOP'}
        self.send_command(command, wait_response=False)
    
    def get_speeds(self):
        """Get current motor speeds"""
        return (self.left_speed, self.right_speed)
    
    def get_sensor_data(self) -> Dict:
        """Get latest sensor data"""
        return self.sensor_data.copy()
    
    def set_sensor_callback(self, callback: Callable):
        """
        Set callback function for sensor data updates
        
        Args:
            callback: Function to call when new sensor data arrives
                     Signature: callback(sensor_data: Dict)
        """
        self.sensor_callback = callback
    
    def _read_loop(self):
        """Background thread to read sensor data from Arduino"""
        logger.info("Arduino read loop started")
        
        while self.running:
            try:
                if self.serial and self.serial.in_waiting > 0:
                    line = self.serial.readline().decode('utf-8').strip()
                    
                    if line:
                        try:
                            data = json.loads(line)
                            
                            # Check if it's sensor data
                            if 'line' in data and 'distance' in data:
                                self.sensor_data = data
                                
                                # Call callback if set
                                if self.sensor_callback:
                                    self.sensor_callback(data)
                            
                            # Log other messages
                            elif 'status' in data:
                                if data['status'] == 'ready':
                                    logger.info("Arduino ready")
                                elif data['status'] == 'error':
                                    logger.error(f"Arduino error: {data.get('message', 'Unknown')}")
                        
                        except json.JSONDecodeError:
                            logger.debug(f"Non-JSON line: {line}")
                
                time.sleep(0.01)  # Small delay to prevent CPU hogging
                
            except Exception as e:
                logger.error(f"Error in read loop: {e}")
                time.sleep(0.1)
        
        logger.info("Arduino read loop stopped")
    
    def cleanup(self):
        """Cleanup resources"""
        self.stop()
        self.disconnect()
        logger.info("Arduino driver cleaned up")
    
    def __del__(self):
        """Destructor"""
        try:
            self.cleanup()
        except:
            pass