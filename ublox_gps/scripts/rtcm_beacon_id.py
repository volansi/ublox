#!/usr/bin/env python3
import rospy
import math
import serial
import io
import base64
import sys
import socket
from ublox_msgs.msg import MavStatus

# global socket instance
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
have_status = False
ID=0

def status_cb(status):
  global status_ID, status_time, have_status
  status_time = rospy.get_rostime()
  have_status = True
  status_ID=status.beacon_ID
  rospy.loginfo("RTCM ID is %d Target %d" % (status_ID,ID))

def read_rtcm():
  serport = rospy.get_param("~serport", "/dev/ttyACM_mb")
  baud = rospy.get_param("~baud", 230400)
  rospy.loginfo("opening rtcm port: " + serport + " at baud %d" % (baud))

  # open serial port
  ser = serial.Serial(serport, baud, timeout=0.001, inter_byte_timeout=0.0001) 
  ser.reset_input_buffer()

  last = rospy.get_rostime()
  rate = rospy.Rate(500)
  # count msgs sent
  mav_msg_count=0 # max value 32

  while not rospy.core.is_shutdown():

    # readline (is RTCMV3 canonical IO base64 encoded? I think so)
    str = ser.readline()
    now = rospy.get_rostime()

    # print stats
    dt = (now-last).to_sec()
    last=now
    numbytes = len(str)

    if numbytes > 0:
      #rospy.loginfo("read length: %d dt: %f seq: %d" % (len(str), dt, mav_msg_count))
      # check if beacon is active and send
      if have_status:
        dts = (now-status_time).to_sec()
        if (status_ID==ID) and (dts<2.0):
          #rospy.loginfo("sending: %d to %d dt: %f" % (len(str), ID, dts))
          try:
            sock.sendall(str)
          except socket.error as e:
            rospy.logwarn("rtcm send failed (socket error): %s" % e)
            sys.exit(0)
    else:
      rate.sleep()
      
if __name__ == '__main__':
    rospy.init_node('read_rtcm', anonymous=True)

    transport_protocol = rospy.get_param('~transport_protocol', 'tcp')
    address = rospy.get_param('~address', '127.0.0.1')
    port = rospy.get_param('~port', 6666)
    ID = rospy.get_param('~id', 10)
    status_topic = rospy.get_param('~status_topic', '/mav_status')

    connection_str = transport_protocol + ":" + address + ": %d" % (port)
    rospy.loginfo(f"Connecting to: {connection_str} ...")

    # Connect the socket to the port where the server is listening
    server_address = (address, port)
    try:
      sock.connect(server_address)
    except socket.error as e:
      rospy.logwarn("Connect failed (socket error): %s" % e)
      sys.exit(0)
      
    sock.settimeout(1) # set timeout as only low latency is useful

    # register callback for mav monitor
    rospy.Subscriber(status_topic, MavStatus, status_cb)
    
    try:
        read_rtcm()
    except rospy.ROSInterruptException:
        pass

    sock.close()
    rospy.signal_shutdown('connection closed')
