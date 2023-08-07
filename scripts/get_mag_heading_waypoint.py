import rospy
from sensor_msgs.msg import NavSatFix
from util.tf_utils import EARTH_RADIUS

from math import asin, atan2, sin, cos

def main():
    deg = input("Compass Heading Degrees: ")
    min = input("Compass Heading Minutes (\"): ")
    sec = input("Compass Heading Seconds (\'): ")
    dist = input("Distance (m): ")

    theta = deg + min / 60 + sec / 3600

    waypoint = rospy.wait_for_message('/gps/fix', NavSatFix, timeout=5)
    long_i = waypoint.longitude
    lat_i = waypoint.latitude
    delta = dist / EARTH_RADIUS

    # source: https://www.movable-type.co.uk/scripts/latlong.html
    lat_f = asin(sin(lat_i) * cos(delta) + cos(lat_i) * sin(delta) * cos(theta))
    long_f = long_i + atan2(sin(theta) * sin(delta) * cos(lat_i), cos(delta) - sin(lat_i) * sin(lat_f))
    print("longitude: " + long_f + " N")
    print("latitude: " + lat_f + " E")