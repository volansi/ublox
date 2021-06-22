#!/usr/bin/env python3
import rospy
import subprocess

if __name__ == '__main__':
    rospy.init_node("rtcm_sender")
    p = subprocess.call('socat /dev/ttyUSB0,b230400,raw,echo=0 TCP:10.223.219.10:7000', shell=True)
    rospy.signal_shutdown('connection closed')
