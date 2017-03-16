import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.srv import CommandBool, SetMode, StreamRate
from mavros_msgs.msg import PositionTarget
from sensor_msgs.msg import NavSatFix
import time
import sys
import numpy
import threading

from fnxnsForRaw import set_Local_Waypoint

rospy.init_node("send_Waypoints")

rospy.wait_for_service("mavros/set_stream_rate")
setRate = rospy.ServiceProxy("mavros/set_stream_rate", StreamRate)

#Set rate at 50 Hz
setRate(0, 50, 1)
rospy.Rate(50.0)

global local_Pose, local_Vel, fcu_Pose, fcu_Vel, exitFlag#, read_Position

exitFlag = 0

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


def fcu_Pos_Callback(data):
    global fcu_Pose
    fcu_Pose = data

def fcu_Vel_Callback(data):
    global fcu_Vel
    fcu_Vel = data


def emergency_Loiter():
    global exitFlag
    rospy.wait_for_service("mavros/set_mode")
    modeSet = rospy.ServiceProxy("mavros/set_mode", SetMode)

    raw_input("Press enter twice to Loiter")
    raw_input("Press enter once to Loiter")

    exitFlag = 1

    modeSet(0, "AUTO.LOITER")

    time.sleep(5)


def main():
    global local_Pose, local_Vel, fcu_Pose, fcu_Vel, exitFlag#, read_Position
    #Use ENU coords

    #allow loiter mode to be enabled by pressing the enter key twice
    loiterThread = threading.Thread(name='emergency_Loiter', target=emergency_Loiter)
    loiterThread.start()


    fcu_Pose = PoseStamped()

    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, fcu_Pos_Callback)
    rospy.Subscriber("/mavros/local_position/velocity", TwistStamped, fcu_Vel_Callback)

    #allow mode change to LOITER so that the quadrotor maintains its final position.
    rospy.wait_for_service("mavros/set_mode")
    modeSet = rospy.ServiceProxy("mavros/set_mode", SetMode)

    pub_Position = rospy.Publisher("/mavros/setpoint_raw/local", PositionTarget, queue_size = 0)
    
    time.sleep(.1)



    travel_Height = 20
    max_North = 70



    #wait for initial fcu update.
    while fcu_Pose.pose.position.z == 0.0:
        time.sleep(0.1)
        print "Waiting for FCU update"

    fcu_ground_Level = (fcu_Pose.pose.position.z,)


    #Display takeoff waypoint to user for 1 second
    takeoff_Waypoint = set_Local_Waypoint(0,0,travel_Height,0,0,2, 0)
    print takeoff_Waypoint
    time.sleep(1)

    #Waypoint has to be sent to FCU before mode can be changed to OFFBOARD.
    i = 0
    while i < 100:
        takeoff_Waypoint = set_Local_Waypoint(0,0,travel_Height,0,0,2, 0)
        pub_Position.publish(takeoff_Waypoint)
        time.sleep(0.01)
        i = i + 1
        print i

    #Set mode variable to OFFBOARD
    mode_List = [0, "OFFBOARD"]
    #Call function to set mode and arm motor
    quad_Command(mode_List, True)

    #take off to requested height




    #change to use FCU position and velocity.  Will add delay, but is safer

    #while (local_Pose.pose.position.z-ground_Level[0]) < .95 * travel_Height or (local_Pose.pose.position.z - ground_Level[0]) > 1.05*travel_Height:

    while (fcu_Pose.pose.position.z-fcu_ground_Level[0]) < .9 * travel_Height or (fcu_Pose.pose.position.z - fcu_ground_Level[0]) > 1.1*travel_Height or abs(fcu_Vel.twist.linear.z) > 0.3:

        if exitFlag != 0:
            time.sleep(3)
            sys.exit()

        takeoff_Waypoint = set_Local_Waypoint(0,0,travel_Height,0,0,2, 0)
        pub_Position.publish(takeoff_Waypoint)
        time.sleep(0.2)
        height = fcu_Pose.pose.position.z - fcu_ground_Level[0]
        print "Taking off.  The height is: ", height

    print "The desired height has been reached: ", fcu_Pose.pose.position.z - fcu_ground_Level[0]
    time.sleep(0.4)






    #Set first waypoint and send to quadrotor at 10 Hz
    time0 = time.time()

    #fly for 10 seconds to build up speed
    #while abs(local_Vel.twist.linear.x) < 6.5 and time.time() - time0 < 10:
    while time.time() - time0 < 15:
        first_Waypoint = set_Local_Waypoint(0,max_North,travel_Height, 0, 8, 0, 0)
        pub_Position.publish(first_Waypoint)
        time.sleep(0.1)
        print "Distance North of home.", fcu_Pose.pose.position.y



    modeSet(0, "AUTO.LOITER")
    time.sleep(5)


#Run the program
if __name__ == "__main__":
    main()
