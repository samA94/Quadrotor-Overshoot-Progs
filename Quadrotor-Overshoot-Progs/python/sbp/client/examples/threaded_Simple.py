# Copyright (C) 2015 Swift Navigation Inc.
# Contact: Fergus Noble <fergus@swiftnav.com>
#
# This source is subject to the license found in the file 'LICENSE' which must
# be be distributed together with this source. All other rights reserved.
#
# THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
# EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.


"""
the :mod:`sbp.client.examples.simple` module contains a basic example of
reading SBP messages from a serial port, decoding BASELINE_NED messages and
printing them out.
"""
import matplotlib.pyplot as plt

from data_Collection import collect

import time
import sys
from datetime import datetime, timedelta
import threading

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

from sbp.client.drivers.pyserial_driver import PySerialDriver
from sbp.client import Handler, Framer
from sbp.client.loggers.json_logger import JSONLogger
from sbp.navigation import SBP_MSG_BASELINE_NED, MsgBaselineNED, SBP_MSG_VEL_NED,MsgVelNED, SBP_MSG_POS_LLH, MsgPosLLH, SBP_MSG_DOPS, MsgDops, SBP_MSG_DOPS, MsgDops
import argparse

global source, exitFlag
exitFlag = False


def convert_Timestamp(timestring):

        time_Date = datetime.strptime(timestring[:-7], '%Y-%m-%dT%H:%M:%S') - datetime(1970,1,1)

        micro_sec = int(timestring[20:])
        epoch = datetime(1970,1,1)

        unix_Time = time_Date.seconds + time_Date.days * 86400

        secs = unix_Time
        nsecs = micro_sec * 1000 #convert to nanosec for ROS

        return secs, nsecs


def velocity_NED():
    global source, exitFlag
    vel = TwistStamped()
    pub_Velocity = rospy.Publisher("/dGPS/Velocity", TwistStamped, queue_size=2)

    for msg, metadata in source.filter(SBP_MSG_VEL_NED):
        #print "Velocities: %.3f,%.3f,%.3f" % (msg.n*1e-3, msg.e*1e-3, msg.d*1e-3)
        #print msg
        timestring = metadata['time']

        unix_Time, nano_Secs = convert_Timestamp(timestring)

        vel.header.stamp.secs = unix_Time
        vel.header.stamp.nsecs = nano_Secs #convert to nanosec for ROS

        vel.twist.linear.x = msg.n/1000.0
        vel.twist.linear.y = msg.e/1000.0
        vel.twist.linear.z = msg.d/1000.0

        if exitFlag==False:
            pub_Velocity.publish(vel)
        else:
            sys.exit()
        

        
def baseline_NED():
    global source, exitFlag
    pos = PoseStamped()
    pub_Position = rospy.Publisher("/dGPS/Position", PoseStamped, queue_size=2)

    fig = plt.figure(1)
    plt.show(block=False)

    x_Fixed = []
    y_Fixed = []

    x_Other = []
    y_Other = []


    for msg, metadata in source.filter(SBP_MSG_BASELINE_NED):
        #print msg
        timestring = metadata['time']
        unix_Time, nano_Secs = convert_Timestamp(timestring)

        pos.header.stamp.secs = unix_Time
        pos.header.stamp.nsecs = nano_Secs
        
        pos.pose.position.x = -msg.n/1000.0
        pos.pose.position.y = -msg.e/1000.0
        pos.pose.position.z = msg.d/1000.0

        if exitFlag==False:
            pub_Position.publish(pos)
        else:
            sys.exit()

        #if flags=1, fixed.  otherwise, not fixed
        status = msg.flags
        
        if status == 1:
            x_Fixed.append(-msg.n/1000.0)
            y_Fixed.append(-msg.e/1000.0)
            #plt.show
            if len(x_Fixed) % 10 == 0:
                plt.plot(y_Fixed, x_Fixed, '-y')
                #plt.scatter(msg.e, msg.n)
                plt.draw()
                print status
                print "Fixed"

        else:
            x_Other.append(-msg.n/1000.0)
            y_Other.append(-msg.e/1000.0)
            #plt.show()
            if len(x_Other) % 10 == 0:
                plt.plot(y_Other, x_Other, '-b')
                #plt.scatter(msg.e, msg.n)
                plt.draw()
                print status
                print "Non-fixed RTK solution"

def gps_Pos():
    global source, exitFlag
    pos_Glob = NavSatFix()
    pub_Global = rospy.Publisher("/dGPS/Global", NavSatFix, queue_size=2)

    for msg, metadata in source.filter(SBP_MSG_POS_LLH):
        timestring = metadata['time']
        #print "%.10f, %.10f" % (msg.lat, msg.lon)
        #print msg
        unix_Time, nano_Secs = convert_Timestamp(timestring)

        pos_Glob.status.status = msg.flags
        pos_Glob.status.service = msg.n_sats

        pos_Glob.header.stamp.secs = unix_Time
        pos_Glob.header.stamp.nsecs = nano_Secs
        pos_Glob.latitude = msg.lat
        pos_Glob.longitude = msg.lon
        pos_Glob.altitude = msg.height
        
        if exitFlag==False:
            pub_Global.publish(pos_Glob)
        else:
            sys.exit()

def dop_Info():
    global source, exitFlag
    DOP_String = String()
    pub_DOP = rospy.Publisher("/dGPS/DOP", String, queue_size = 2)

    for msg, metadata in source.filter(SBP_MSG_DOPS):
        dop = ''
        timestring = metadata['time']

        unixTime, nano_Secs = convert_Timestamp(timestring)

        dop = str(unixTime) + '.' + str(nano_Secs) + ',' + str(msg.pdop) + ',' + str(msg.gdop) + ','
        dop = dop + str(msg.tdop) + ',' + str(msg.hdop) + ',' + str(msg.vdop)

        DOP_String.data = dop
        if exitFlag==False:
            pub_DOP.publish(DOP_String.data)
        else:
            sys.exit()


def dataCollect():

    if exitFlag==False:
        collect()
    else:
        sys.exit()
        

def main():
    global source, exitFlag

    rospy.init_node("dGPS_Data", anonymous=True)

    parser = argparse.ArgumentParser(description="Swift Navigation SBP Example.")
    parser.add_argument("-p", "--port",
                      default=['/dev/ttyUSB0'], nargs=1,
                      help="specify the serial port to use.")
    args = parser.parse_args()

    # Open a connection to Piksi using the default baud rate (1Mbaud)
    with PySerialDriver(args.port[0], baud=1000000) as driver:
        with Handler(Framer(driver.read, None, verbose=True)) as source:
            try:
                
                t1 = threading.Thread(name='baseline_NED', target=baseline_NED)
                t2 = threading.Thread(name='Vel_NED', target=velocity_NED)
                t3 = threading.Thread(name='dop_Info', target=dop_Info)
                t4 = threading.Thread(name='gps_Pos', target=gps_Pos)

                dataCollectThread = threading.Thread(name='dataCollect', target = dataCollect)

                t1.start()
                t2.start()
                t3.start()
                t4.start()

                dataCollectThread.start()

                rospy.spin()

            except KeyboardInterrupt:
                exitFlag = True
                raise SystemExit


if __name__ == "__main__":
    main()


