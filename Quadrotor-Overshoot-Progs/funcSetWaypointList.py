from mavros_msgs.msg import Waypoint

def set_Waypoint(lat, lon, alt):

    waypoint = Waypoint()

    waypoint.autocontinue = True
    waypoint.frame = 0

    waypoint.x_lat = lat
    waypoint.y_long = lon
    waypoint.z_alt = alt

    return waypoint

