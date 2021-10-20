#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import NavSatFix
from ublox_msgs.msg import MavStatus
import math
from pymavlink import mavutil, mavparm
from pymavlink.dialects.v20.common import MAVLink_global_position_int_message 
import struct
import subprocess
from pymap3d import geodetic2ned
import sys

# monitor distance, ping time, and input ID of UAS
# allows other nodes to send data based on this info
# currently will exit on mavlink connection loss and rely on roslaunch respawn

have_id=False
input_id=0
pingms=10000
base_mode=0
custom_mode=0
system_status=0
time_boot_ms=0

def ping(addr):
   result = subprocess.run("ping -c 1 -w 1 " +addr,
                           shell=True,
                           stdout=subprocess.PIPE,
                           stderr=subprocess.PIPE,  # get all output
                           universal_newlines=True)  # return string not bytes)
   if result.returncode == 0:
     for line in result.stdout.splitlines():
       if "icmp_seq" in line:
         return float(line.split('time=')[-1].split(' ms')[0])
   else:     
     return -1

def base_gps_callback(msg):
    global base_lat, base_lon, base_alt, base_time, base_updated
    base_lat = msg.latitude
    base_lon = msg.longitude
    base_alt = msg.altitude
    base_time = msg.header.stamp
    base_updated = True

if __name__ == '__main__':
    rospy.init_node('monitor_mav', anonymous=True)

    transport_protocol = rospy.get_param('~transport_protocol', 'tcp')
    address = rospy.get_param('~address', '127.0.0.1')
    port = rospy.get_param('~port', '5760')
    id = rospy.get_param('~id', 10)
    fix_topic = rospy.get_param('~fix_topic', '/ublox_movingbase/fix')

    rospy.Subscriber(fix_topic, NavSatFix, base_gps_callback)
    pub = rospy.Publisher("/mav_status", MavStatus, queue_size=10)
    status=MavStatus()
    base_updated = False
    start_time = rospy.get_rostime()
    
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

    last = rospy.get_rostime()
    while not rospy.core.is_shutdown():
      msg = connection.recv_match()
      if not msg:
        continue
      if msg.get_type() == 'GLOBAL_POSITION_INT':
        rospy.loginfo("Got message: %s GPS_ID: %d ping: %f ms base_active: %r" % (msg.get_type(),input_id, pingms, base_updated))
        #print("Message: %s" % msg)
        time_boot_ms=msg.time_boot_ms
        
        if base_updated:
          N, E, D = geodetic2ned(msg.lat*1.0e-7, msg.lon*1.0e-7, msg.alt*1.0e-3,
                                 base_lat, base_lon, base_alt, ell=None, deg=True)
        
      # lower freq check on GPS_INPUT_ID and ping time  
      now = rospy.get_rostime()
      if (now-last).to_sec() > 1.0:
        # request MAV_GPS_INPUT_ID
        connection.mav.param_request_read_send(
          connection.target_system, connection.target_component,
          b'MAV_GPS_INPUT_ID',
          -1
        )

        message = connection.recv_match(type='PARAM_VALUE',
                                        blocking=True,
                                        timeout=1)
        # check if returned NoneType?  
        #print(message)
        bytes=struct.pack(">f", message.param_value)
        # mavlink decodes everything to float32 packing?
        input_id = struct.unpack(">I", bytes)[0]
        #print('name: %s\tvalue: %d' %
        #      (message.param_id, input_id))
        last=now

        pingms = ping(address)
        #print(res)

        # check modes in heartbeat
        msg = connection.recv_match(type='HEARTBEAT',
                                    blocking=True,
                                    timeout=3)
        # why does this sometimes return wrong field values?
        # check if returned NoneType?
        #print("Message: %s" % msg)
        base_mode=msg.base_mode
        custom_mode=msg.custom_mode
        system_status=msg.system_status
          
        # publish status
        if base_updated:
          status.base_stamp = base_time
          status.time_boot_ms = time_boot_ms
          status.N = N
          status.E = E
          status.D = D
          status.distance = math.sqrt(N*N+E*E)
          status.ping_ms = pingms
          status.beacon_ID = input_id
          status.base_mode = base_mode
          status.custom_mode = custom_mode
          status.system_status = system_status
          pub.publish(status)
          base_updated = False
        '''
        else:
          status.base_stamp = start_time
          status.time_boot_ms = time_boot_ms
          status.N = float("NaN")
          status.E = float("NaN")
          status.D = float("NaN")
          status.distance = float("NaN")
          status.ping_ms = pingms
          status.beacon_ID = input_id
          status.base_mode = base_mode
          status.custom_mode = custom_mode
          status.system_status = system_status
          pub.publish(status)
        '''
          
    #shutdown    
    rospy.signal_shutdown('connection closed')
