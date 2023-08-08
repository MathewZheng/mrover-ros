import rospy
from sensor_msgs.msg import NavSatFix
from util.tf_utils import EARTH_RADIUS

from math import asin, atan2, sin, cos

def calc_final_waypoint():
    deg = float(input("Compass Heading Degrees: "))
    min = float(input("Compass Heading Minutes (\"): "))
    sec = float(input("Compass Heading Seconds (\'): "))
    dist = float(input("Distance (m): "))

    theta = deg + min / 60 + sec / 3600

    waypoint = rospy.wait_for_message("gps/fix", NavSatFix, timeout=5)

    long_i = waypoint.longitude
    lat_i = waypoint.latitude
    delta = dist / EARTH_RADIUS

    # source: https://www.movable-type.co.uk/scripts/latlong.html
    lat_f = asin(sin(lat_i) * cos(delta) + cos(lat_i) * sin(delta) * cos(theta))
    long_f = long_i + atan2(sin(theta) * sin(delta) * cos(lat_i), cos(delta) - sin(lat_i) * sin(lat_f))
    print("longitude: " + str(long_f) + " N")
    print("latitude: " + str(lat_f) + " E")

def main():
    rospy.init_node("heading_waypoint")
    calc_final_waypoint()

if __name__ == "__main__":
    main()