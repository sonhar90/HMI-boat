from geopy.distance import geodesic, distance
from geopy.point import Point
import math

# Explicitly specifying the ellipsoid
ELLIPSOID = 'WGS-84'

def add_distance_to_lat_lon(lat, lon, distance_meters_north, distance_meters_east):
    """
    Adds distance (in meters) to a latitude and longitude point using the WGS84 datum.
    
    :param lat: Latitude of the starting point
    :param lon: Longitude of the starting point
    :param distance_meters_north: Distance to move north
    :param distance_meters_east: Distance to move east
    :return: A tuple of (new_latitude, new_longitude)
    """
    original_point = Point(lat, lon)
    # Specify the ellipsoid in the geodesic method
    destination_north = geodesic(meters=distance_meters_north, ellipsoid=ELLIPSOID).destination(original_point, 0)  # North
    final_destination = geodesic(meters=distance_meters_east, ellipsoid=ELLIPSOID).destination(destination_north, 90)  # East
    return final_destination.latitude, final_destination.longitude

def calculate_distance_north_east(lat1, lon1, lat2, lon2):
    """
    Calculates the distance in meters north and east between two lat/lon coordinates using the WGS84 datum.
    
    :param lat1: Latitude of the first point
    :param lon1: Longitude of the first point
    :param lat2: Latitude of the second point
    :param lon2: Longitude of the second point
    :return: A tuple of (distance_north, distance_east)
    """
    initial_point = Point(lat1, lon1)
    target_point = Point(lat2, lon2)
    
    # Calculate the distance north by comparing the difference in latitude while keeping longitude constant.
    # Specify the ellipsoid in the distance method
    dist_north = distance(initial_point, Point(lat2, lon1), ellipsoid=ELLIPSOID).meters
    dist_east = distance(Point(lat2, lon1), target_point, ellipsoid=ELLIPSOID).meters
    
    # Adjust signs based on direction
    dist_north = dist_north if lat2 > lat1 else -dist_north
    dist_east = dist_east if lon2 > lon1 else -dist_east
    
    return dist_north, dist_east

def decimal_degrees_to_degrees_minutes(decimal_degrees):
    """
    Converts decimal degrees to degrees and decimal minutes.
    
    :param decimal_degrees: Latitude or longitude in decimal degrees.
    :return: A tuple of (degrees, minutes) where degrees is an integer
             and minutes is a float.
    """
    degrees = int(decimal_degrees)
    minutes = (abs(decimal_degrees) - abs(degrees)) * 60
    return degrees, minutes

def add_body_frame_pos_to_lat_lon(lat, lon, x, y, z, roll, pitch, heading):
    """
    Adjusts a given latitude and longitude based on body frame position offsets and the ship's orientation.
    Accounts for the fact that the body frame's Z-axis is positive downward, X-axis is positive forward,
    and Y-axis is positive to the starboard side of the ship.

    :param lat: Initial latitude in decimal degrees.
    :param lon: Initial longitude in decimal degrees.
    :param x: Displacement along the ship's longitudinal axis in meters.
    :param y: Displacement along the ship's lateral axis in meters.
    :param z: Displacement along the ship's vertical axis in meters (positive downwards).
    :param roll: Ship's roll in degrees.
    :param pitch: Ship's pitch in degrees.
    :param heading: Ship's heading in degrees (from North).
    :return: Tuple of (new_latitude, new_longitude) in decimal degrees.
    """
    # Convert angles from degrees to radians
    heading_rad = math.radians(heading)
    roll_rad = math.radians(roll)
    pitch_rad = math.radians(pitch)

    # Correct x and y for the roll and pitch first, considering small angles
    corrected_x = x + z * math.sin(pitch_rad)
    corrected_y = y + z * math.sin(roll_rad)

    # Calculate northward and eastward displacement considering the heading
    northward_displacement = corrected_x * math.cos(heading_rad) + corrected_y * math.sin(heading_rad)
    eastward_displacement = corrected_y * math.cos(heading_rad) - corrected_x * math.sin(heading_rad)

    # Calculate the new global position based on the displacements
    original_point = geodesic(meters=northward_displacement, ellipsoid=ELLIPSOID).destination((lat, lon), 0)  # North
    final_point = geodesic(meters=eastward_displacement, ellipsoid=ELLIPSOID).destination(original_point, 90)  # East

    return final_point.latitude, final_point.longitude
