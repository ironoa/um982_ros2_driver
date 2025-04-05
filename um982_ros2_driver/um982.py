from pyproj import CRS, Transformer
import threading
import serial
import time
import math


def crc_table():
    table = []
    for i in range(256):
        crc = i
        for j in range(8, 0, -1):
            if crc & 1:
                crc = (crc >> 1) ^ 0xEDB88320
            else:
                crc >>= 1
        table.append(crc)
    return table

NMEA_EXPEND_CRC_TABLE = crc_table()


def nmea_expend_crc(nmea_expend_sentence):
    def calculate_crc32(data):
        crc = 0
        for byte in data:
            crc = NMEA_EXPEND_CRC_TABLE[(crc ^ byte) & 0xFF] ^ (crc >> 8)
        return crc & 0xFFFFFFFF

    try:
        sentence, crc = nmea_expend_sentence[1:].split("*")
        crc = crc[:8]
    except:
        return False
    calculated_crc = calculate_crc32(sentence.encode())
    return crc.lower() == format(calculated_crc, '08x')

def nmea_crc(nmea_sentence):
    # Remove the starting '$' and '*' and the checksum part
    try:
        sentence, crc = nmea_sentence[1:].split("*")
        crc = crc[:2]
    except:
        return False
    calculated_checksum = 0
    # XOR operation for each character in the string
    for char in sentence:
        calculated_checksum ^= ord(char)
    # Convert the calculated checksum to hexadecimal format and uppercase
    calculated_checksum_hex = format(calculated_checksum, 'X')
    # Checksum comparison
    return calculated_checksum_hex.zfill(2) == crc.upper()

def msg_seperate(msg:str):
    return msg[1:msg.find('*')].split(',')

def PVTSLN_solver(msg:str):
    parts = msg_seperate(msg)
    bestpos_hgt    = float(parts[3+7])          # Altitude in meters
    bestpos_lat    = float(parts[4+7])          # Latitude in degrees (output to 11 decimal places)
    bestpos_lon    = float(parts[5+7])          # Longitude in degrees (output to 11 decimal places)
    bestpos_hgtstd = float(parts[6+7])          # Altitude standard deviation in meters
    bestpos_latstd = float(parts[7+7])          # Latitude standard deviation in meters
    bestpos_lonstd = float(parts[8+7])          # Longitude standard deviation in meters
    fix = (bestpos_hgt, bestpos_lat, bestpos_lon, bestpos_hgtstd, bestpos_latstd, bestpos_lonstd)
    return fix


def GNHPR_solver(msg:str):
    parts = msg_seperate(msg)
    heading = float(parts[3-1])
    pitch   = float(parts[4-1])
    roll    = float(parts[5-1])
    orientation = (heading, pitch, roll)
    return orientation


def BESTNAV_solver(msg:str):
    parts = msg_seperate(msg)
    vel_hor_std = float(parts[-1])  # Horizontal velocity standard deviation, unit m/s
    vel_ver_std = float(parts[-2])  # Vertical velocity standard deviation, unit m/s
    vel_ver     = float(parts[-3])  # Vertical velocity, m/s, positive value indicates increasing height (upward), negative value indicates decreasing height (downward)
    vel_heading = float(parts[-4])  # Actual ground motion direction relative to true north (relative to ground track), deg
    vel_hor     = float(parts[-5])  # Horizontal ground velocity, m/s
    vel_north   = vel_hor * math.cos(math.radians(vel_heading))     # Decompose to get north direction velocity
    vel_east    = vel_hor * math.sin(math.radians(vel_heading))     # Decompose to get east direction velocity
    return (vel_east, vel_north, vel_ver, vel_hor_std, vel_hor_std, vel_ver_std)


def create_utm_trans(lat, lon):
    """Build a converter to convert points from WGS84 geographic coordinate system to UTM coordinate system.

    Args:
        lon (float): Longitude of the point.
        lat (float): Latitude of the point.

    Returns:
        transformer: Converter
    """
    # UTM zone number is determined by longitude, with each zone spanning 6 degrees starting from -180 degrees.
    zone_number            = int((lon + 180) / 6) + 1
    # Northern hemisphere is the region above the equator (latitude 0 degrees).
    isnorth                = lat >= 0
    # Define WGS84 coordinate system
    wgs84_crs              = CRS("epsg:4326")
    # Choose the appropriate UTM EPSG code based on whether it's in the northern hemisphere
    utm_crs_str            = f"epsg:326{zone_number}" if isnorth else f"epsg:327{zone_number}"
    utm_crs                = CRS(utm_crs_str)
    # Create a coordinate converter from WGS84 to UTM
    transformer            = Transformer.from_crs(wgs84_crs, utm_crs, always_xy=True)
    return transformer


def utm_trans(transformer, lon, lat):
    """Convert a point from WGS84 geographic coordinate system to UTM coordinate system.

    Args:
        transformer: Converter
        lon (float): Longitude of the point.
        lat (float): Latitude of the point.

    Returns:
        tuple: A tuple containing the converted x (easting) and y (northing) in the UTM coordinate system.
    """
    # Perform coordinate conversion
    utm_x, utm_y           = transformer.transform(lon, lat)
    return (utm_x, utm_y)


class UM982Serial(threading.Thread):
    def __init__(self, port, band):
        super().__init__()
        # Open serial port
        self.ser            = serial.Serial(port, band)
        # Set running flag
        self.isRUN          = True
        # Data
        self.fix            = None   # Data needed for sensor_msgs/NavSatFix
        self.orientation    = None   # Heading angle
        self.vel            = None   # Velocity
        self.utmpos         = None
        # Read initial data
        for i in range(10):
            self.read_frame()
        # wgs84 to utm
        bestpos_hgt, bestpos_lat, bestpos_lon, bestpos_hgtstd, bestpos_latstd, bestpos_lonstd = self.fix
        self.transformer = create_utm_trans(bestpos_lat, bestpos_lon)
        self.utmpos      = utm_trans(self.transformer, bestpos_lon, bestpos_lat)


    def stop(self):
        """ End running """
        self.isRUN = False
        time.sleep(0.1)
        self.ser.close()


    def read_frame(self):
        frame = self.ser.readline().decode('utf-8')
        if frame.startswith("#PVTSLNA") and nmea_expend_crc(frame):
            self.fix = PVTSLN_solver(frame)
        elif frame.startswith("$GNHPR") and nmea_crc(frame):
            self.orientation = GNHPR_solver(frame)
        elif frame.startswith("#BESTNAVA") and nmea_expend_crc(frame):
            self.vel = BESTNAV_solver(frame)


    def run(self):
        while self.isRUN:
            self.read_frame()
            bestpos_hgt, bestpos_lat, bestpos_lon, bestpos_hgtstd, bestpos_latstd, bestpos_lonstd = self.fix
            self.utmpos = utm_trans(self.transformer, bestpos_lon, bestpos_lat)


if __name__ == "__main__":
    um982 = UM982Serial("/dev/rtk1", 921600)
    um982.start()
