import math
import rospy
from sensor_msgs.msg import NavSatFix
from util.tf_utils import EARTH_RADIUS


def calculate_new_coordinates(lat, lon, distance, magnetic_heading):
    # Convert headings to radians
    true_heading_rad = math.radians(magnetic_heading)
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)

    # Radius of the Earth (mean radius in kilometers)
    radius = EARTH_RADIUS

    # Calculate new latitude and longitude
    new_lat = math.asin(math.sin(lat_rad) * math.cos(distance / radius) +
                        math.cos(lat_rad) * math.sin(distance / radius) * math.cos(true_heading_rad))
    
    new_lon = lon_rad + math.atan2(math.sin(true_heading_rad) * math.sin(distance / radius) * math.cos(lat_rad),
                                    math.cos(distance / radius) - math.sin(lat_rad) * math.sin(new_lat))

    # Convert new latitude and longitude to degrees
    new_lat_deg = math.degrees(new_lat)
    new_lon_deg = math.degrees(new_lon)

    print(f"New latitude: {new_lat_deg}")
    print(f"New longitude: {new_lon_deg}")

def main():
    
    curr_waypoint = rospy.wait_for_message('/gps/fix', NavSatFix, timeout=5)
    curr_lon = curr_waypoint.longitude
    curr_lat = curr_waypoint.latitude

    heading_in = input("Compass Heading (Degrees): ")
    dist_in = input("Distance (m): ")

    calculate_new_coordinates(lat = curr_lat,lon= curr_lon, distance = dist_in, magnetic_heading= heading_in)


if __name__ == "__main__":
    main()