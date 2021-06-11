#!/usr/bin/env python3
import rospy
import subprocess

if __name__ == '__main__':
    rospy.init_node("mavlink_router")
    p = subprocess.call('mavlink-routerd /dev/ttyUSB_telem2:921600', shell=True)
    rospy.signal_shutdown('connection closed')
