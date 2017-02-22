import rospy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped, TwistStamped
import time
import os
import datetime

global pos_File, glob_File, vel_File

def globalCallback(data):
    global glob_File

    timestamp = float(data.header.stamp.secs) + float(data.header.stamp.nsecs)/1000000000
    timestamp = repr(timestamp)

    glob_File.write(timestamp + ',')
    glob_File.write(str(data.latitude) + ',')
    glob_File.write(str(data.longitude) + ',')
    glob_File.write(str(data.altitude) + ',')
    glob_File.write(str(data.status.status) + '\n')
    glob_File.flush()


def posCallback(data):
    global pos_File

    timestamp = float(data.header.stamp.secs) + float(data.header.stamp.nsecs)/1000000000
    timestamp = repr(timestamp)

    pos_File.write(timestamp + ',')
    pos_File.write(str(data.pose.position.x) + ',')
    pos_File.write(str(data.pose.position.y) + ',')
    pos_File.write(str(data.pose.position.z) + '\n')
    pos_File.flush()


def velCallback(data):
    global vel_File

    timestamp = float(data.header.stamp.secs) + float(data.header.stamp.nsecs)/1000000000
    timestamp = repr(timestamp)

    vel_File.write(timestamp + ',')
    vel_File.write(str(data.twist.linear.x) + ',')
    vel_File.write(str(data.twist.linear.y) + ',')
    vel_File.write(str(data.twist.linear.z) + '\n')
    vel_File.flush()



def collect():
    global pos_File, vel_File, glob_File

    filenameLocal = "/home/lab/Data_From_Tests/localPose" + str(datetime.datetime.utcnow()) + ".txt"

    filenameGPS = "/home/lab/Data_From_Tests/localGPS" + str(datetime.datetime.utcnow()) + ".txt"

    filenameVelocity = "/home/lab/Data_From_Tests/Velocity" + str(datetime.datetime.utcnow()) + ".txt"

    
    if os.path.isfile(filenameLocal):
        print("Error: Filename already exists")
        SystemExit


    pos_File = open(filenameLocal, 'w')
    pos_File.write('NED coordinates' + '\n')

    vel_File = open(filenameVelocity, 'w')
    vel_File.write('VelocitiesN, E, D' + '\n')

    glob_File = open(filenameGPS, 'w')
    glob_File.write('GPS_time Lat,Lon,Alt' + '\n')

    #rospy.init_node("collect_Data")


    rospy.Subscriber("/dGPS/Global", NavSatFix, globalCallback)
    rospy.Subscriber("/dGPS/Position", PoseStamped, posCallback)
    rospy.Subscriber("/dGPS/Velocity", TwistStamped, velCallback)
    rospy.spin()
