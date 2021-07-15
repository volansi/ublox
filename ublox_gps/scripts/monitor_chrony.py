#!/usr/bin/env python3
import rospy
import subprocess

def check_chrony():
   result = subprocess.run("chronyc tracking",
                           shell=True,
                           stdout=subprocess.PIPE,
                           stderr=subprocess.PIPE,  # get all output
                           universal_newlines=True)  # return string not bytes)
   if result.returncode == 0:
     for line in result.stdout.splitlines():
       if "Leap status" in line:
         #rospy.loginfo(line)
         print(line)
         if "Normal" in line:
           return True
         else:
           return False
   else:     
     return False

def monitor():
  while not rospy.core.is_shutdown():
    # check if system is synchronized
    try:
       rospy.sleep(1)
       if check_chrony():
         #rospy.loginfo("Syncronized --> exiting!")
         print("Syncronized --> exiting!")
         exit(0)
       #rospy.loginfo("Not Syncronized :( Sleeping")
       print("Not Syncronized :( Sleeping")
    except rospy.ROSException:
       pass

if __name__ == '__main__':
    #rospy.init_node('monitor_chrony', anonymous=True)
    #rospy.sleep(20) # wait for nodes to start

    try:
        monitor()
    except rospy.ROSInterruptException:
        pass

    rospy.signal_shutdown('connection closed')
