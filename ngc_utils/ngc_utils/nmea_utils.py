from datetime import datetime
from ngc_utils.geo_utils import decimal_degrees_to_degrees_minutes
import math

def calculate_nmea_checksum(nmea_sentence_without_dollar):
    """
    Calculates the checksum for an NMEA sentence, excluding the initial '$'.
    The checksum is a simple XOR of all the ASCII values of the characters in the sentence.

    :param nmea_sentence_without_dollar: The NMEA sentence without the '$'.
    :return: The two-character hexadecimal checksum, as a string, uppercase.
    """
    checksum = 0
    for char in nmea_sentence_without_dollar:
        checksum ^= ord(char)
    return f"{checksum:02X}"

def append_checksum(nmea_sentence_without_dollar):
    """
    Appends the checksum to an NMEA sentence.

    :param nmea_sentence_without_dollar: The NMEA sentence without the '$' and without the checksum.
    :return: The complete NMEA sentence with the checksum appended.
    """
    checksum = calculate_nmea_checksum(nmea_sentence_without_dollar)
    return f"${nmea_sentence_without_dollar}*{checksum}"

def format_lat_lon_for_nmea(decimal_degrees, is_latitude):
    """
    Formats latitude or longitude in decimal degrees for NMEA 0183 as degrees and minutes.

    :param decimal_degrees: Latitude or longitude in decimal degrees.
    :param is_latitude: Boolean indicating if the input is latitude (True) or longitude (False).
    :return: Formatted string in degrees and minutes for NMEA 0183.
    """
    degrees, minutes = decimal_degrees_to_degrees_minutes(decimal_degrees)
    direction = 'N' if is_latitude and degrees >= 0 else 'S' if is_latitude else 'E' if degrees >= 0 else 'W'
    degrees = abs(degrees)
    degrees_format = "{:02d}" if is_latitude else "{:03d}"
    formatted_string = f"{degrees_format.format(degrees)}{minutes:06.3f},{direction}"
    return formatted_string

def create_gga_message(latitude, longitude, fix_quality=1, num_satellites=10, horizontal_dilution=1.0, altitude=0.0, height_geoid=0.0):
    """
    Creates a GGA NMEA message using the system's current time for the timestamp and includes a checksum.

    :param latitude: Latitude in decimal degrees.
    :param longitude: Longitude in decimal degrees.
    :param fix_quality: GPS Fix quality.
    :param num_satellites: Number of satellites being tracked.
    :param horizontal_dilution: Horizontal dilution of position.
    :param altitude: Altitude, meters above mean sea level.
    :param height_geoid: Height of geoid (mean sea level) above WGS84 ellipsoid.
    :return: A string representing the GGA message with checksum.
    """
    time_utc = datetime.utcnow().strftime("%H%M%S.%f")[:-5]  # Keeping up to milliseconds
    formatted_lat = format_lat_lon_for_nmea(latitude, True)
    formatted_lon = format_lat_lon_for_nmea(longitude, False)
    
    nmea_sentence_without_dollar = f"GPGGA,{time_utc},{formatted_lat},{formatted_lon},{fix_quality},{num_satellites},{horizontal_dilution},{altitude},M,{height_geoid},M,,"
    final_sentence = append_checksum(nmea_sentence_without_dollar)
    
    return final_sentence

def create_hdt_message(true_heading):
    """
    Creates an HDT NMEA message indicating the true heading.

    :param true_heading: True heading in degrees.
    :return: A string representing the HDT message with checksum.
    """
    nmea_sentence_without_dollar = f"GPHDT,{true_heading:.1f},T"
    final_sentence = append_checksum(nmea_sentence_without_dollar)
    
    return final_sentence

def create_vtg_message(u, v):
    """
    Creates a VTG NMEA message for the track made good and ground speed from surge and sway velocities.
    
    :param u: Surge velocity in meters per second (forward movement).
    :param v: Sway velocity in meters per second (sideways movement to the starboard).
    :return: A string representing the VTG message with checksum.
    """
    # Convert surge and sway velocities to ground speed in knots and km/h
    ground_speed_ms = math.sqrt(u**2 + v**2)  # Ground speed in meters per second
    ground_speed_knots = ground_speed_ms * 1.94384  # Convert m/s to knots
    ground_speed_kmh = ground_speed_ms * 3.6  # Convert m/s to km/h
    
    # Calculate the true course made good
    true_course_radians = math.atan2(v, u)  # Angle in radians from North
    true_course_degrees = math.degrees(true_course_radians)
    if true_course_degrees < 0:
        true_course_degrees += 360  # Normalize angle to 0-360 degrees
    
    # Construct the VTG sentence without the initial '$' and without the checksum
    nmea_sentence_without_dollar = f"GPVTG,{true_course_degrees:.2f},T,,M,{ground_speed_knots:.2f},N,{ground_speed_kmh:.2f},K"
    
    # Append the checksum to the sentence
    final_sentence = append_checksum(nmea_sentence_without_dollar)
    
    return final_sentence

def create_rot_message(rate_of_turn):
    """
    Creates a ROT NMEA message indicating the rate of turn.
    
    :param rate_of_turn: Rate of turn in degrees per minute (positive for starboard).
    :return: A string representing the ROT message with checksum.
    """
    # Convert rate of turn to degrees per second
    rate_of_turn_deg_per_sec = rate_of_turn / 60.0
    
    # Convert rate of turn to NMEA format (positive for starboard, negative for port)
    rate_of_turn_nmea = rate_of_turn if rate_of_turn >= 0 else rate_of_turn + 360
    
    # Construct the ROT sentence without the initial '$' and without the checksum
    nmea_sentence_without_dollar = f"ROTHR,{rate_of_turn_nmea:.2f}"
    
    # Append the checksum to the sentence
    final_sentence = append_checksum(nmea_sentence_without_dollar)
    
    return final_sentence

def create_mwv_message(wind_angle, wind_speed_mps):
    """
    Creates a MWV NMEA message for wind speed and angle relative to the vessel's heading.

    :param wind_angle: Wind angle in degrees relative to the vessel's heading.
    :param wind_speed_mps: Wind speed in meters per second.
    :return: A string representing the MWV message with checksum.
    """
    # Conversion factor from meters per second to knots
    mps_to_knots = 1.94384

    # Convert wind speed to knots
    wind_speed_knots = wind_speed_mps * mps_to_knots

    # Reference is always 'R' for relative to the ship's heading
    reference = 'R'
    # Status 'A' means data is valid
    status = 'A'

    # Construct the MWV sentence without the initial '$' and without the checksum
    nmea_sentence_without_dollar = f"GPWMV,{wind_angle:.2f},{reference},{wind_speed_knots:.2f},N,{status}"
    
    # Append the checksum to the sentence
    final_sentence = append_checksum(nmea_sentence_without_dollar)
    
    return final_sentence