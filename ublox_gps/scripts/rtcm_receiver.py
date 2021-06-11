#!/usr/bin/env python3
import rospy
import subprocess

if __name__ == '__main__':
    rospy.init_node("rtcm_receiver")
    p = subprocess.call('socat  TCP-LISTEN:7000,fork /dev/ttyUSB_gps_uart2,b115200,raw,echo=0', shell=True)
    rospy.signal_shutdown('connection closed')
