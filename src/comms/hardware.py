#!/usr/bin/env python3

import rospy
import RPi.GPIO as GPIO
from threading import Lock

class GPIOManager:
    """Manages GPIO pins on Raspberry Pi"""
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        self.pin_modes = {}  # Tracks pin modes
        self.pin_states = {}  # Tracks pin states
        self.lock = Lock()
        
    def setup_pin(self, pin, mode):
        """Setup a GPIO pin"""
        with self.lock:
            if mode.upper() == "OUT":
                GPIO.setup(pin, GPIO.OUT)
                self.pin_modes[pin] = "OUT"
            elif mode.upper() == "IN":
                GPIO.setup(pin, GPIO.IN)
                self.pin_modes[pin] = "IN"
            elif mode.upper() == "PWM":
                GPIO.setup(pin, GPIO.OUT)
                self.pin_modes[pin] = "PWM"
                return GPIO.PWM(pin, 100)  # 100Hz default
                
    def write_pin(self, pin, value):
        """Write to a GPIO pin"""
        with self.lock:
            if pin in self.pin_modes:
                if self.pin_modes[pin] == "OUT":
                    GPIO.output(pin, value)
                    self.pin_states[pin] = value
                    
    def read_pin(self, pin):
        """Read from a GPIO pin"""
        with self.lock:
            if pin in self.pin_modes:
                if self.pin_modes[pin] == "IN":
                    return GPIO.input(pin)
        return None
        
    def cleanup(self):
        """Cleanup GPIO pins"""
        GPIO.cleanup()

class I2CManager:
    """Manages I2C communication"""
    def __init__(self, bus_number=1):
        try:
            import smbus
            self.bus = smbus.SMBus(bus_number)
            self.lock = Lock()
        except ImportError:
            rospy.logerr("smbus module not found. I2C functionality will be limited.")
            self.bus = None
            
    def write_byte(self, address, value):
        """Write a byte to an I2C device"""
        with self.lock:
            if self.bus:
                try:
                    self.bus.write_byte(address, value)
                    return True
                except Exception as e:
                    rospy.logerr(f"I2C write error: {e}")
        return False
        
    def read_byte(self, address):
        """Read a byte from an I2C device"""
        with self.lock:
            if self.bus:
                try:
                    return self.bus.read_byte(address)
                except Exception as e:
                    rospy.logerr(f"I2C read error: {e}")
        return None

class SerialManager:
    """Manages serial communication"""
    def __init__(self, port="/dev/ttyUSB0", baudrate=115200):
        try:
            import serial
            self.serial = serial.Serial(port, baudrate)
            self.lock = Lock()
        except ImportError:
            rospy.logerr("pyserial module not found. Serial functionality will be limited.")
            self.serial = None
            
    def write_data(self, data):
        """Write data to serial port"""
        with self.lock:
            if self.serial and self.serial.is_open:
                try:
                    self.serial.write(data)
                    return True
                except Exception as e:
                    rospy.logerr(f"Serial write error: {e}")
        return False
        
    def read_data(self, size=1):
        """Read data from serial port"""
        with self.lock:
            if self.serial and self.serial.is_open:
                try:
                    return self.serial.read(size)
                except Exception as e:
                    rospy.logerr(f"Serial read error: {e}")
        return None
        
    def close(self):
        """Close serial port"""
        if self.serial and self.serial.is_open:
            self.serial.close() 