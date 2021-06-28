#!/usr/bin/env python3
import rospy
import math
import serial
import io
import base64
from pymavlink import mavutil
from pymavlink.dialects.v20.common import MAVLink_gps_rtcm_data_message

def read_rtcm():
  port = rospy.get_param("~port", "/dev/ttyS0")
  baud = rospy.get_param("~baud", 230400)
  rospy.loginfo("opening rtcm port: " + port + " at baud %d" % (baud))

  # open serial port
  ser = serial.Serial(port, baud, timeout=0.001, inter_byte_timeout=0.0001) 
  ser.reset_input_buffer()
  #sio = io.TextIOWrapper(io.BufferedRWPair(ser, ser)) # does readline work wihout this?
  last = rospy.get_rostime()

  # count msgs sent
  mav_msg_count=0 # max value 32

  while not rospy.core.is_shutdown():

    # readline (is RTCMV3 canonical IO base64 encoded? I think so)
    #str = sio.readline()
    str = ser.readline()
    now = rospy.get_rostime()

    # print stats
    dt = (now-last).to_sec()
    last=now
    numbytes = len(str)

    if numbytes > 0:
      rospy.loginfo("length: %d dt: %f seq: %d" % (len(str), dt, mav_msg_count))
      #print("length: ", len(str), "dt: ", dt, "seq: ", mav_msg_count)
      # send heartbeat
      connection.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GENERIC, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
      # check if string is too large for mavlink 4x180
      if numbytes > 720:
        rospy.logerror("Mah Too Mah Bytes: " + numbytes)
        continue

      # pack in mavlink https://mavlink.io/en/messages/common.html#GPS_RTCM_DATA
      chunks = math.ceil(numbytes/180)
      for fragment in range(chunks):

	#Fields are:
	# flags uint8
	#    bit 0 --> fragmented
	#    bit 2:1 --> frag count
	#    bit 7:3 --> sequence count
	# len   uint8 
	# data  unit8[180]
        # send to mavlink router
        mav_msg_count %= 32
        flgs = (chunks>1) | (fragment<<1) | (mav_msg_count<<3)
        if chunks == 1:
          length = numbytes
        elif fragment == chunks-1:
          length = numbytes % 180
        else:
          length = 180
	# is there a nicer way to do this in python? msg constructor copies 180 bytes so it needs padding and extra copies?
        bytes=str[(fragment*180):((fragment*180)+length-1)].zfill(180)
        #rospy.loginfo("chunks: %d fragment: %d flags: %s length: %d start: %d end: %d" % (chunks, fragment, hex(flgs), length, fragment*180, (fragment*180)+length-1)) 
        rtcm_msg = MAVLink_gps_rtcm_data_message(
           flags=flgs,
           len=length,
           data=bytes)
        connection.mav.send(rtcm_msg)
        mav_msg_count += 1

if __name__ == '__main__':
    rospy.init_node('read_rtcm', anonymous=True)

    transport_protocol = rospy.get_param('~transport_protocol', 'tcp')
    address = rospy.get_param('~address', '10.223.219.9')
    port = rospy.get_param('~port', '5760')
    id = rospy.get_param('~id', 10)

    connection_str = transport_protocol + ":" + address + ":" + port
    rospy.loginfo(f"Connecting to: {connection_str} ...")

    try:
        connection = mavutil.mavlink_connection(
            connection_str, 
            source_system=id, 
            autoreconnect=True, 
            input=False)
    except ConnectionRefusedError:
        rospy.logwarn("Connection refused")
        sys.exit(0)
    except OSError:
        rospy.logwarn("OS error during connection")
        sys.exit(0)

    try:
        read_rtcm()
    except rospy.ROSInterruptException:
        pass

    rospy.signal_shutdown('connection closed')
