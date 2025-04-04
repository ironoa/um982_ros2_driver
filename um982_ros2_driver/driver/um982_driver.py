"""
Core driver implementation for the UM982 GNSS device.

This module contains the UM982Driver class which handles communication
with the UM982 GNSS device and processes the received messages.
"""

import threading
import time
import math
import numpy as np
import serial
from pyproj import CRS, Transformer

from .utils import msg_seperate, check_crc, check_checksum, determine_utm_zone_and_hemisphere


class UM982Driver(threading.Thread):
    """
    Driver for the UM982 GNSS device.
    
    This class handles communication with the UM982 GNSS device and
    processes the received messages. It inherits from threading.Thread
    to run the communication in a separate thread.
    """
    
    def __init__(self, port, baud_rate):
        """
        Initialize the UM982Driver.
        
        Args:
            port (str): Serial port for the UM982 device
            baud_rate (int): Baud rate for the serial connection
        """
        super().__init__()
        
        # Serial connection
        self.port = port
        self.baud_rate = baud_rate
        self.ser = None
        
        # Thread control
        self.is_running = False
        
        # Data storage
        # Position data (from PVTSLN message)
        self.bestpos_hgt = 0.0
        self.bestpos_lat = 0.0
        self.bestpos_lon = 0.0
        self.bestpos_hgtstd = 0.0
        self.bestpos_latstd = 0.0
        self.bestpos_lonstd = 0.0
        
        # Velocity data (from BESTNAV message)
        self.vel_east = 0.0
        self.vel_north = 0.0
        self.vel_up = 0.0
        self.vel_horstd = 0.0
        self.vel_verstd = 0.0
        
        # Orientation data (from GNHPR message)
        self.heading = 0.0
        self.pitch = 0.0
        self.roll = 0.0
        
        # UTM coordinates
        self.utm_x = 0.0
        self.utm_y = 0.0
        
        # Velocity standard deviations
        self.vel_east_std = 0.0
        self.vel_north_std = 0.0
        self.vel_hor_cov = 0.0
        self.vel_up_std = 0.0
        
        # Coordinate transformer
        self.transformer = None
        
        # Data locks
        self.data_lock = threading.Lock()
        
        # Initialization flag
        self.is_initialized = False

    def connect(self):
        """
        Connect to the UM982 device.
        
        Returns:
            bool: True if connection successful, False otherwise
        """
        try:
            self.ser = serial.Serial(self.port, self.baud_rate, timeout=1.0)
            return True
        except serial.SerialException as e:
            print(f"Error connecting to UM982: {e}")
            return False

    def disconnect(self):
        """
        Disconnect from the UM982 device.
        """
        if self.ser and self.ser.is_open:
            self.ser.close()

    def start(self):
        """
        Start the driver thread.
        """
        if not self.ser:
            if not self.connect():
                return False
        
        self.is_running = True
        super().start()
        return True

    def stop(self):
        """
        Stop the driver thread.
        """
        self.is_running = False
        self.join()
        self.disconnect()

    def run(self):
        """
        Main thread function.
        
        This function runs in a separate thread and continuously reads
        and processes messages from the UM982 device.
        """
        # Read initial data to initialize the transformer
        for _ in range(10):
            self.read_frame()
            if self.bestpos_lat != 0.0 and self.bestpos_lon != 0.0:
                break
            time.sleep(0.1)
        
        # Initialize the transformer
        self._init_transformer()
        
        # Set initialization flag
        self.is_initialized = True
        
        # Main loop
        while self.is_running:
            self.read_frame()
            
            # Update UTM coordinates
            with self.data_lock:
                self._update_utm_coordinates()
                self._update_velocity_std()

    def read_frame(self):
        """
        Read and process a single frame from the UM982 device.
        """
        if not self.ser or not self.ser.is_open:
            return
        
        try:
            line = self.ser.readline().decode('utf-8').strip()
            self._parse_message(line)
        except (serial.SerialException, UnicodeDecodeError) as e:
            print(f"Error reading from UM982: {e}")

    def _parse_message(self, message):
        """
        Parse a message from the UM982 device.
        
        Args:
            message (str): Message from the UM982 device
        """
        if not message:
            return
        
        with self.data_lock:
            try:
                if message.startswith("#PVTSLNA") and check_crc(message):
                    self._parse_pvtsln(message)
                elif message.startswith("$GNHPR") and check_checksum(message):
                    self._parse_gnhpr(message)
                elif message.startswith("#BESTNAVA") and check_crc(message):
                    self._parse_bestnav(message)
                elif message.startswith("$KSXT") and check_checksum(message):
                    self._parse_ksxt(message)
            except Exception as e:
                print(f"Error parsing message: {e}")

    def _parse_pvtsln(self, message):
        """
        Parse a PVTSLN message.
        
        Args:
            message (str): PVTSLN message
        """
        parts = msg_seperate(message)
        self.bestpos_hgt = float(parts[3+7])
        self.bestpos_lat = float(parts[4+7])
        self.bestpos_lon = float(parts[5+7])
        self.bestpos_hgtstd = float(parts[6+7])
        self.bestpos_latstd = float(parts[7+7])
        self.bestpos_lonstd = float(parts[8+7])

    def _parse_gnhpr(self, message):
        """
        Parse a GNHPR message.
        
        Args:
            message (str): GNHPR message
        """
        parts = msg_seperate(message)
        self.heading = float(parts[3-1])
        self.pitch = float(parts[4-1])
        self.roll = float(parts[5-1])

    def _parse_bestnav(self, message):
        """
        Parse a BESTNAV message.
        
        Args:
            message (str): BESTNAV message
        """
        parts = msg_seperate(message)
        self.vel_horstd = float(parts[-1])
        self.vel_verstd = float(parts[-2])
        
        # Extract velocity data if not using KSXT
        vel_ver = float(parts[-3])
        vel_heading = float(parts[-4])
        vel_hor = float(parts[-5])
        
        # Calculate east and north velocities
        self.vel_east = vel_hor * math.sin(math.radians(vel_heading))
        self.vel_north = vel_hor * math.cos(math.radians(vel_heading))
        self.vel_up = vel_ver

    def _parse_ksxt(self, message):
        """
        Parse a KSXT message.
        
        Args:
            message (str): KSXT message
        """
        parts = msg_seperate(message)
        self.vel_east = float(parts[18-1])
        self.vel_north = float(parts[19-1])
        self.vel_up = float(parts[20-1])

    def _init_transformer(self):
        """
        Initialize the coordinate transformer.
        """
        if self.bestpos_lat == 0.0 and self.bestpos_lon == 0.0:
            return
        
        wgs84_crs = CRS("epsg:4326")
        zone_number, is_north = determine_utm_zone_and_hemisphere(
            self.bestpos_lat, self.bestpos_lon
        )
        utm_crs_str = f"epsg:326{zone_number}" if is_north else f"epsg:327{zone_number}"
        utm_crs = CRS(utm_crs_str)
        self.transformer = Transformer.from_crs(wgs84_crs, utm_crs, always_xy=True)

    def _update_utm_coordinates(self):
        """
        Update UTM coordinates from latitude and longitude.
        """
        if not self.transformer:
            self._init_transformer()
            return
        
        self.utm_x, self.utm_y = self.transformer.transform(
            self.bestpos_lon, self.bestpos_lat
        )

    def _update_velocity_std(self):
        """
        Update velocity standard deviations.
        """
        heading_rad = np.deg2rad(self.heading)
        cos_h = np.cos(heading_rad)
        sin_h = np.sin(heading_rad)
        
        vel_cov_xy = self.vel_horstd ** 2
        self.vel_east_std = np.sqrt(vel_cov_xy * cos_h ** 2)
        self.vel_north_std = np.sqrt(vel_cov_xy * sin_h ** 2)
        self.vel_hor_cov = np.sqrt(vel_cov_xy * cos_h * sin_h)
        self.vel_up_std = self.vel_verstd

    def get_position(self):
        """
        Get the current position.
        
        Returns:
            tuple: (latitude, longitude, height, lat_std, lon_std, height_std)
        """
        with self.data_lock:
            return (
                self.bestpos_lat,
                self.bestpos_lon,
                self.bestpos_hgt,
                self.bestpos_latstd,
                self.bestpos_lonstd,
                self.bestpos_hgtstd
            )

    def get_velocity(self):
        """
        Get the current velocity.
        
        Returns:
            tuple: (east, north, up, east_std, north_std, up_std)
        """
        with self.data_lock:
            return (
                self.vel_east,
                self.vel_north,
                self.vel_up,
                self.vel_east_std,
                self.vel_north_std,
                self.vel_up_std
            )

    def get_orientation(self):
        """
        Get the current orientation.
        
        Returns:
            tuple: (heading, pitch, roll)
        """
        with self.data_lock:
            return (
                self.heading,
                self.pitch,
                self.roll
            )

    def get_utm_position(self):
        """
        Get the current UTM position.
        
        Returns:
            tuple: (x, y, z)
        """
        with self.data_lock:
            return (
                self.utm_x,
                self.utm_y,
                self.bestpos_hgt
            )

    def is_ready(self):
        """
        Check if the driver is initialized and ready.
        
        Returns:
            bool: True if ready, False otherwise
        """
        return self.is_initialized
