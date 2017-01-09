import rospy
from mavros_msgs.msg import PositionTarget


def set_Local_Waypoint(x_Pos, y_Pos, travel_Height, x_Vel, y_Vel, z_Vel, yaw_Angle):
    """
    This function sets the local waypoint using ENU notation.

    x_Pos:  East - positive position in meters from home position
    y_Pos:  North - positive position in meters from home position
    z_Pos:  Up - positive position in meters from the home position
    x_Vel, y_Vel, z_Vel:  Corresponding velocities.  Not currently working properly.
    yaw_Angle:  Desired yaw, in radians, as measured counterclockwise from North

    """
    takeoff_Waypoint = PositionTarget()
    #Timestamp message
    takeoff_Waypoint.header.stamp = rospy.get_rostime()

    #Set reference frame as global
    takeoff_Waypoint.header.frame_id = "1"
    #Local ENU coordinate system
    takeoff_Waypoint.coordinate_frame = 1


    #Taken from ardupilot documentation.
    #Sets acceleration and force control bits to 1, or inactive
    #Sets position, velocity, and yaw bits to 0, or active, allowing for
    #  control of these elements.  Decimal equivalent is 3008.
    takeoff_Waypoint.type_mask = 0b0000101111000000

    #set desired positions and travel velocities
    takeoff_Waypoint.position.x = x_Pos
    takeoff_Waypoint.position.y = y_Pos
    takeoff_Waypoint.position.z = travel_Height

    takeoff_Waypoint.velocity.x = x_Vel
    takeoff_Waypoint.velocity.y = y_Vel
    takeoff_Waypoint.velocity.z = z_Vel

    takeoff_Waypoint.yaw = yaw_Angle

    return takeoff_Waypoint
