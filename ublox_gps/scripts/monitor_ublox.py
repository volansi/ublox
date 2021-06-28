#!/usr/bin/env python3
import rospy
import subprocess

from sensor_msgs.msg import NavSatFix

def monitor():
  topic = rospy.get_param("~topic", "/ublox_movingbase/fix")
  nodename = rospy.get_param("~nodename", "ublox_movingbase")
  rospy.loginfo("monitoring topic: " + topic + "from node name" + nodename)
  while not rospy.core.is_shutdown():
    # check if node is publishing anything
    try:
       rospy.sleep(1)
       #rospy.wait_for_message('/ublox_movingbase/fix',NavSatFix,timeout=1)
       rospy.wait_for_message(topic,NavSatFix,timeout=1)
       rospy.loginfo('ublox active')
    except rospy.ROSException:
       # timeout --> kill it for relaunch
       rospy.loginfo('ublox timeout: killing node')
       #p = subprocess.call('rosnode kill /ublox_movingbase', shell=True)
       p = subprocess.call('rosnode kill /'+nodename, shell=True)
       rospy.sleep(10)

if __name__ == '__main__':
    rospy.init_node('monitor_ublox', anonymous=True)
    rospy.sleep(20) # wait for nodes to start

    try:
        monitor()
    except rospy.ROSInterruptException:
        pass

    rospy.signal_shutdown('connection closed')
