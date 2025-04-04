"""
Utility functions for the UM982 driver.

This module contains utility functions for parsing and processing
messages from the UM982 GNSS device.
"""

import math


def crc_table():
    """
    Generate CRC32 table for NMEA extended messages.
    
    Returns:
        list: CRC32 table
    """
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


# Pre-computed CRC table for efficiency
NMEA_EXPEND_CRC_TABLE = crc_table()


def check_crc(nmea_expend_sentence):
    """
    Check CRC for NMEA extended messages.
    
    Args:
        nmea_expend_sentence (str): NMEA extended sentence
        
    Returns:
        bool: True if CRC is valid, False otherwise
    """
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


def check_checksum(nmea_sentence):
    """
    Check checksum for standard NMEA messages.
    
    Args:
        nmea_sentence (str): NMEA sentence
        
    Returns:
        bool: True if checksum is valid, False otherwise
    """
    try:
        sentence, crc = nmea_sentence[1:].split("*")
        crc = crc[:2]
    except:
        return False
    calculated_checksum = 0
    for char in sentence:
        calculated_checksum ^= ord(char)
    calculated_checksum_hex = format(calculated_checksum, 'X')
    return calculated_checksum_hex.zfill(2) == crc.upper()


def msg_seperate(msg):
    """
    Separate NMEA message into fields.
    
    Args:
        msg (str): NMEA message
        
    Returns:
        list: List of fields
    """
    return msg[1:msg.find('*')].split(',')


def determine_utm_zone_and_hemisphere(lat, lon):
    """
    Determine UTM zone and hemisphere from latitude and longitude.
    
    Args:
        lat (float): Latitude in degrees
        lon (float): Longitude in degrees
        
    Returns:
        tuple: (zone_number, is_northern_hemisphere)
    """
    # UTM zone is determined by longitude, from -180 to 180 degrees
    zone_number = int((lon + 180) / 6) + 1
    
    # Northern hemisphere if latitude is >= 0
    is_northern_hemisphere = lat >= 0
    
    return zone_number, is_northern_hemisphere
