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

import time
from datetime import datetime, timedelta
import threading

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import NavSatFix

from sbp.client.drivers.pyserial_driver import PySerialDriver
from sbp.client import Handler, Framer
from sbp.client.loggers.json_logger import JSONLogger
from sbp.navigation import SBP_MSG_BASELINE_NED, MsgBaselineNED, SBP_MSG_VEL_NED,MsgVelNED, SBP_MSG_POS_LLH, MsgPosLLH, SBP_MSG_DOPS, MsgDops
import argparse

global source


def convert_Timestamp(timestring):

        time_Date = datetime.strptime(timestring[:-7], '%Y-%m-%dT%H:%M:%S') - datetime(1970,1,1)

        micro_sec = int(timestring[20:])
        epoch = datetime(1970,1,1)

        unix_Time = time_Date.seconds + time_Date.days * 86400

        secs = unix_Time
        nsecs = micro_sec * 1000 #convert to nanosec for ROS

        return secs, nsecs


def velocity_NED():
    global source
    vel = TwistStamped()
    pub_Velocity = rospy.Publisher("dGPS_Velocity", TwistStamped, queue_size=2)

    for msg, metadata in source.filter(SBP_MSG_VEL_NED):
        #print "Velocities: %.3f,%.3f,%.3f" % (msg.n*1e-3, msg.e*1e-3, msg.d*1e-3)
        timestring = metadata['time']

        unix_Time, nano_Secs = convert_Timestamp(timestring)

        vel.header.stamp.secs = unix_Time
        vel.header.stamp.nsecs = nano_Secs #convert to nanosec for ROS

        vel.twist.linear.x = msg.n/1000.0
        vel.twist.linear.y = msg.e/1000.0
        vel.twist.linear.z = msg.d/1000.0

        pub_Velocity.publish(vel)
        

        
def baseline_NED():
    global source
    pos = PoseStamped()
    pub_Position = rospy.Publisher("dGPS_Position", PoseStamped, queue_size=2)

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
        
        pos.pose.position.x = msg.n/1000.0
        pos.pose.position.y = msg.e/1000.0
        pos.pose.position.z = msg.d/1000.0

        pub_Position.publish(pos)

        #if flags=1, fixed.  otherwise, not fixed
        status = msg.flags
        
        if status == 1:
            x_Fixed.append(msg.n)
            y_Fixed.append(msg.e)
            plt.plot(y_Fixed, x_Fixed, '-y')
            #plt.scatter(msg.e, msg.n)
            plt.draw()
            if len(x_Fixed) % 20 == 0:
                print "Fixed"

        else:
            x_Other.append(msg.n)
            y_Other.append(msg.e)
            plt.plot(y_Other, x_Other, '-b')
            #plt.scatter(msg.e, msg.n)
            plt.draw()
            if len(x_Other) % 20 == 0:
                print "Non-fixed RTK solution"


def gps_Pos():
    global source
    pos_Global = NavSatFix()
    pub_Global = rospy.Publisher("dGPS_Global", NavSatFix, queue_size=2)

    for msg, metadata in source.filter(SBP_MSG_POS_LLH):
        timestring = metadata['time']
        #print "%.10f, %.10f" % (msg.lat, msg.lon)
        unix_Time, nano_Secs = convert_Timestamp(timestring)

        pos_Global.header.stamp.secs = unix_Time
        pos_Global.header.stamp.nsecs = nano_Secs

        pos_Global.latitude = msg.lat
        pos_Global.longitude = msg.lon
        pos_Global.altitude = msg.height

        pub_Global.publish(pos_Global)
        

def main():
    global source
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
                
                t4 = threading.Thread(name='gps_Pos', target=gps_Pos)


                t1.start()
                t2.start()
                
                t4.start()
                rospy.spin()
                while True:
                    time.sleep(1)


            except KeyboardInterrupt: 
                pass




if __name__ == "__main__":
    main()


