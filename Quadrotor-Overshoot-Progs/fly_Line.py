import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.srv import CommandBool, SetMode, StreamRate
from mavros_msgs.msg import PositionTarget
from sensor_msgs.msg import NavSatFix
import time
import sys

from fnxnsForRaw import set_Local_Waypoint

rospy.init_node("send_Waypoints")

rospy.wait_for_service("mavros/set_stream_rate")
setRate = rospy.ServiceProxy("mavros/set_stream_rate", StreamRate)

#Set rate at 50 Hz
setRate(0, 50, 1)
rospy.Rate(50.0)

global read_Position, local_Pose, local_Vel

def quad_Command(mode, armVar = False):
    rospy.wait_for_service("mavros/cmd/arming")
    armQuad = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
    rospy.wait_for_service("mavros/set_mode")
    modeSet = rospy.ServiceProxy("mavros/set_mode", SetMode) 

    #Set the desired mode
    for i in range(10):
        modeSet(mode[0],mode[1])
        time.sleep(0.1)
    print "Mode set to: ", mode
    #Arm the quad
    armQuad(armVar)
    print "System Arm Status: ", armVar


def position_callback(GPS_Position_From_Quad):
    #function to get the global position of the quad
    global read_Position
    read_Position = GPS_Position_From_Quad

def local_Pos_Callback(data):
    #Local position data
    global local_Pose
    local_Pose = data

def local_Vel_Callback(data):
    global local_Vel
    local_Vel = data

def main():
    global read_Position, local_Pose, local_Vel

    rospy.Subscriber("/mavros/global_position/global", NavSatFix, position_callback)
    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, local_Pos_Callback)
    rospy.Subscriber("/mavros/local_position/velocity", TwistStamped, local_Vel_Callback)

    pub_Position = rospy.Publisher("/mavros/setpoint_raw/local", PositionTarget, queue_size = 0)
    
    time.sleep(.1)
    #Set home position
    home_Position = read_Position

    travel_Height = 10

    #Assign tuple with maximum allowable altitude
    max_Height = (read_Position.altitude + 50,)
    ground_Level = (read_Position.altitude,)
    print "The home position is: ", home_Position

    #Display takeoff waypoint to user for 1 second
    takeoff_Waypoint = set_Local_Waypoint(0,0,10,0.01,0.01,2, 0)
    print takeoff_Waypoint
    time.sleep(1)

    #Waypoint has to be sent to FCU before mode can be changed to OFFBOARD.
    i = 0
    while i < 100:
        takeoff_Waypoint = set_Local_Waypoint(0,0,10,0.01,0.01,2, 0)
        pub_Position.publish(takeoff_Waypoint)
        time.sleep(0.01)
        i = i + 1
        print i

    #Set mode variable to OFFBOARD
    mode_List = [0, "OFFBOARD"]
    #Call function to set mode and arm motor
    quad_Command(mode_List, True)

    #take off to requested height
    while local_Pose.pose.position.z < .95 * int(travel_Height):
        takeoff_Waypoint = set_Local_Waypoint(0,0,10,0.01,0.01,2, 0)
        pub_Position.publish(takeoff_Waypoint)
        time.sleep(0.1)
        height = read_Position.altitude - ground_Level[0]
        print "Taking off.  The height is: ", height       
    
    print "The desired height has been reached: ", read_Position.altitude - ground_Level[0]
    time.sleep(0.4)

    #Set first waypoint and send to quadrotor at 10 Hz
    while local_Vel.twist.linear.y < 7.5:
        first_Waypoint = set_Local_Waypoint(0,250,10, 0, 10, 0, 0)
        pub_Position.publish(first_Waypoint)
        time.sleep(0.1)
        print "Distance North of home.", local_Pose.pose.position.y

    print "Recording data."

    #Need to add calls to programs for collecting data here.





    time1 = time.time()

    while time.time() - time1 < 12.5:
        desired_Y = local_Pose.pose.position.y + 10

        first_Waypoint = set_Local_Waypoint(0,desired_Y,10, 0, 5, 0, 0)
        pub_Position.publish(first_Waypoint)
        time.sleep(0.1)
        print "Distance North of home.", local_Pose.pose.position.y

    pos_Y = local_Pose.pose.position.y
   

    #Change mode to LOITER so that the quadrotor maintains its final position.
    rospy.wait_for_service("mavros/set_mode")
    modeSet = rospy.ServiceProxy("mavros/set_mode", SetMode)

    modeSet(0, "AUTO.LOITER")
    time.sleep(5)
    #RTH can be enabled by using the GCS or the RC controller.

#Run the program
if __name__ == "__main__":
    main()
