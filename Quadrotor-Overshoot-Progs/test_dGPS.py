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

global local_Pose, local_Vel#, read_Position

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


#def position_callback(GPS_Position_From_Quad):
    #function to get the global position of the quad
    #global read_Position
    #read_Position = GPS_Position_From_Quad

def local_Pos_Callback(data):
    #Local position data
    global local_Pose
    local_Pose = data

def local_Vel_Callback(data):
    global local_Vel
    local_Vel = data

def main():
    global local_Pose, local_Vel#, read_Position

    #rospy.Subscriber("/mavros/global_position/global", NavSatFix, position_callback)
    rospy.Subscriber("/dGPS/Position", PoseStamped, local_Pos_Callback)
    rospy.Subscriber("/dGPS/Velocity", TwistStamped, local_Vel_Callback)

    pub_Position = rospy.Publisher("/mavros/setpoint_raw/local", PositionTarget, queue_size = 0)
    
    time.sleep(.1)
    #Set home position
    home_Position = local_Pose
    #home_North = (local_Pose.pose.position.x)
    #home_East = (local_Pose.pose.position.y)

    travel_Height = 1.5

    #Assign tuple with maximum allowable altitude
    max_Height = (local_Pose.pose.position.z + 50,)
    ground_Level = (local_Pose.pose.position.z,)
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
    while (local_Pose.pose.position.z-ground_Level[0]) < .95 * int(travel_Height):
        takeoff_Waypoint = set_Local_Waypoint(0,0,travel_Height,0.01,0.01,2, 0)
        pub_Position.publish(takeoff_Waypoint)
        time.sleep(0.1)
        height = local_Pose.pose.position.z - ground_Level[0]
        print "Taking off.  The height is: ", height       
    
    print "The desired height has been reached: ", local_Pose.pose.position.z - ground_Level[0]
    time.sleep(0.4)

    #Set first waypoint and send to quadrotor at 10 Hz
    while local_Vel.twist.linear.y < 0.5:
        first_Waypoint = set_Local_Waypoint(0,250,10, 0, 10, 0, 0)
        pub_Position.publish(first_Waypoint)
        time.sleep(0.1)
        print "Distance North of home.", local_Pose.pose.position.x - home_Position.pose.position.x

    #print "Record data."

    #Need to add calls to programs for collecting data here.





    time1 = time.time()

    while time.time() - time1 < 12.5:
        desired_North = local_Pose.pose.position.x - home_Position.pose.position.x + 10

        first_Waypoint = set_Local_Waypoint(0,desired_North,10, 0, 5, 0, 0)
        pub_Position.publish(first_Waypoint)
        time.sleep(0.1)
        print "Distance North of home.", local_Pose.pose.position.x - home_Position.pose.position.x

    pos_N = local_Pose.pose.position.x - home_Position.pose.position.x
   

    #Change mode to LOITER so that the quadrotor maintains its final position.
    rospy.wait_for_service("mavros/set_mode")
    modeSet = rospy.ServiceProxy("mavros/set_mode", SetMode)

    modeSet(0, "AUTO.LOITER")
    time.sleep(5)
    #RTH can be enabled by using the GCS or the RC controller.

#Run the program
if __name__ == "__main__":
    main()
