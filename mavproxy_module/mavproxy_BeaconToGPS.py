import os
import os.path
import sys
from pymavlink import mavutil
import errno
import time
import math

import ConfigParser #config file

#import pozyx control lib
from pypozyx import POZYX_POS_ALG_TRACKING, POZYX_3D, Coordinates, POZYX_SUCCESS, POZYX_RANGE_PROTOCOL_PRECISION,DeviceCoordinates, PozyxSerial, get_first_pozyx_serial_port, SingleRegister, DeviceList,DeviceRange,PositionError,UWBSettings

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import mp_settings

class BeaconToGPS(mp_module.MPModule):

	IGNORE_FLAG_ALL =  (mavutil.mavlink.GPS_INPUT_IGNORE_FLAG_ALT |
                        mavutil.mavlink.GPS_INPUT_IGNORE_FLAG_VDOP |
                        mavutil.mavlink.GPS_INPUT_IGNORE_FLAG_VEL_VERT |
                        mavutil.mavlink.GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY)


	def __init__(self, mpstate):
        	"""Initialise module"""
        	super(BeaconToGPS, self).__init__(mpstate, "BeaconToGPS", "")
		self.anchor_config=None #'[[0x6e08,0,0,950];[0x6e45,5000,0,2330];[0x6e48,0,6010,2360];[0x6e65,5000,6010,1370]]'

                self.config_file_parser=ConfigParser.ConfigParser()
                self.config_file_parser.read( os.getcwd() + '/config/uwb_config.conf')

                self.anchor_config=self.config_file_parser.get("Anchor", "anchor_coordinates")
                if self.anchor_config is None:
                        print("Need set the anchor coordinate!")
                        return

                self.yaw_deg=self.config_file_parser.getfloat("NED", "yaw_form_ned_to_uwb")
                if self.yaw_deg is None:
                        print("Need set the yaw from ned to uwb!")
                        return
                else:
                        print("NED to UWB yaw:" + str(self.yaw_deg)+" deg")

                self.debug = 0
                self.debug=self.config_file_parser.getint("SYS","debug"):
                if self.debug is None:
                    self.debug=0    

        	serial_port_dev = get_first_pozyx_serial_port()
		if serial_port_dev is None:
			print("No Pozyx connected. Check your USB cable or your driver!")
			return

		self.pozyx = PozyxSerial(serial_port_dev)
		self.anchors = self.anchor_config[1:len( self.anchor_config)-1].split(";")

		self.anchor_list=[]
		self.position = Coordinates()
		self.velocity = Coordinates()
		self.pos_last = Coordinates()
		self.pos_last_time=0
		self.setup_pozyx()

 		self.CONSTANTS_RADIUS_OF_EARTH=6378100.0
		self.DEG_TO_RAD=0.01745329251994329576
		self.RAD_TO_DEG=57.29577951308232087679
	       	self.reference_lat=36.26586666666667
        	self.reference_lon=120.27563333333333
		self.reference_lat_rad=self.reference_lat*self.DEG_TO_RAD
		self.reference_lon_rad=self.reference_lon*self.DEG_TO_RAD
		self.cos_lat=math.cos(self.reference_lat_rad)
		self.target_lon_param= self.CONSTANTS_RADIUS_OF_EARTH *self.cos_lat
        	self.current_lat=0
        	self.current_lon=0
        	self.tag_pos_ned=Coordinates()
        	self.tag_velocity_ned=Coordinates()
                self.yaw=math.radians(self.yaw_deg)  
		self.cos_yaw=math.cos(self.yaw)
		self.sin_yaw=math.sin(self.yaw)
		self.location_update=False
		self.location_update_time=0
        	self.data = {
            		'time_usec' : 0,                        # (uint64_t) Timestamp (micros since boot or Unix epoch)
            		'gps_id' : 0,                           # (uint8_t) ID of the GPS for multiple GPS inputs
            		'ignore_flags' : self.IGNORE_FLAG_ALL,  # (uint16_t) Flags indicating which fields to ignore (see GPS_INPUT_IGNORE_FLAGS enum). All other fields must be provided.
            		'time_week_ms' : 0,                     # (uint32_t) GPS time (milliseconds from start of GPS week)
            		'time_week' : 0,                        # (uint16_t) GPS week number
            		'fix_type' : 0,                         # (uint8_t) 0-1: no fix, 2: 2D fix, 3: 3D fix. 4: 3D with DGPS. 5: 3D with RTK
            		'lat' : 0,                              # (int32_t) Latitude (WGS84), in degrees * 1E7
            		'lon' : 0,                              # (int32_t) Longitude (WGS84), in degrees * 1E7
            		'alt' : 0,                              # (float) Altitude (AMSL, not WGS84), in m (positive for up)
            		'hdop' : 0,                             # (float) GPS HDOP horizontal dilution of position in m
            		'vdop' : 0,                             # (float) GPS VDOP vertical dilution of position in m
            		'vn' : 0,                               # (float) GPS velocity in m/s in NORTH direction in earth-fixed NED frame
            		've' : 0,                               # (float) GPS velocity in m/s in EAST direction in earth-fixed NED frame
            		'vd' : 0,                               # (float) GPS velocity in m/s in DOWN direction in earth-fixed NED frame
           		'speed_accuracy' : 0,                   # (float) GPS speed accuracy in m/s
            		'horiz_accuracy' : 0,                   # (float) GPS horizontal accuracy in m
            		'vert_accuracy' : 0,                    # (float) GPS vertical accuracy in m
           		'satellites_visible' : 0                # (uint8_t) Number of satellites visible.
        		}

	def setAnchorsManual(self):
		''' config anchor'''
		status = self.pozyx.clearDevices()
		for temp_anchor in self.anchors:
			anchor=temp_anchor[1:len(temp_anchor)-1].split(",")
			pozyx_anchor=DeviceCoordinates(int(anchor[0],16), 1, Coordinates(int(anchor[1]), int(anchor[2]), int(anchor[3])))
			status &= self.pozyx.addDevice(pozyx_anchor)
			self.anchor_list.append(pozyx_anchor)

		if len(self.anchors) > 4:
			status &= self.pozyx.setSelectionOfAnchors(POZYX_ANCHOR_SEL_AUTO, len(self.anchors))

	def print_anchor_config(self):
		print("Anchor coordinate config:")
		anchor_coordinate = Coordinates()
		uwb_setting=UWBSettings()
		for i in range(len(self.anchor_list)):
			status=self.pozyx.getDeviceCoordinates(self.anchor_list[i].network_id,anchor_coordinate)
			if status == POZYX_SUCCESS:
				print("anchor "+hex(self.anchor_list[i].network_id) +" coordinate is X: "+ str(anchor_coordinate.x) +" mm; Y: "+str(anchor_coordinate.y)+" mm; Z: "+str(anchor_coordinate.z)+" mm;")
			else:
				print("get anchor"+hex(self.anchor_list[i].network_id)+" coordinate config err")

			status=self.pozyx.getUWBSettings(uwb_setting,self.anchor_list[i].network_id)
			if status == POZYX_SUCCESS:
				print("UWB Setting, channel: "+str(uwb_setting.channel)+" Bitrate: "+str(uwb_setting.bitrate)+" Prf: "+str(uwb_setting.prf)+ " Plen: "+ str(uwb_setting.plen) + " Gain: "+str(uwb_setting.gain_db)+" DB")
			else:
				print("get UWB Setting err")

	def setup_pozyx(self):
		self.setAnchorsManual()
		if self.debug > 0:
			self.print_anchor_config()

		self.pozyx.doPositioning(self.pos_last, POZYX_3D, 1000, POZYX_POS_ALG_TRACKING)
		self.pos_last_time=time.time()

	def get_location(self):
		"""Performs positioning and displays/exports the results."""	
		pos_mm= Coordinates()
		status = self.pozyx.doPositioning(pos_mm, POZYX_3D, 1000, POZYX_POS_ALG_TRACKING)
		now=time.time()
		if status == POZYX_SUCCESS:
			pos_err=PositionError()
			self.pozyx.getPositionError(pos_err)
			self.position.x = pos_mm.x*0.001 #mm-->m
			self.position.y = pos_mm.y*0.001
			self.position.z = pos_mm.z*0.001

			self.get_tag_velocity(pos_mm,now)
			self.location_update=True
			if self.debug ==2 :
				print(" Postion is X: "+str(self.position.x)+" m; Y: "+str(self.position.y)+" m; Z: "+str(self.position.z)+" m;"+" err: "+str(pos_err.xy))
		else:
			if self.debug ==2 :
				print("Do not get tag position")

	def get_tag_velocity(self, position_now,time_now):
		delt_pos=Coordinates()
		#unit: mm
		delt_pos.x = position_now.x-self.pos_last.x
		delt_pos.y = position_now.y-self.pos_last.y
		delt_pos.z = position_now.z-self.pos_last.z
		delt_time  = (time_now-self.pos_last_time)*1000.0 #s-->ms

		self.pos_last = position_now
		self.pos_last_time = time_now

		self.velocity.x=delt_pos.x/delt_time;  #m/s
		self.velocity.y=delt_pos.y/delt_time;
		self.velocity.z=delt_pos.z/delt_time;

		if self.debug ==2 :
			print("Tag velocity is X=" +str(self.velocity.x)+"m/s; Y="+str(self.velocity.y)+"m/s Z="+str(self.velocity.z)+"m/s")

	def mavlink_packet(self, m):
		'''handle mavlink packets'''
		pass

	def send_gps_message(self):
		'''send gps message to fc '''
		self.data['lat']=self.current_lat*1e7
		self.data['lon']=self.current_lon*1e7
		self.data['alt']=self.tag_pos_ned.z
		self.data['vn']=self.tag_velocity_ned.x
		self.data['ve']=self.tag_velocity_ned.y
		self.data['vd']=self.tag_velocity_ned.z
		self.data['speed_accuracy']=0.05
		self.data['horiz_accuracy']=0.1
		self.data['vert_accuracy']=0.1
		self.data['satellites_visible']=20
		self.data['time_week_ms']=0
		self.data['time_usec']=time.time()*1e6
		self.data['gps_id']=0
		self.data['time_week']=0
		self.data['fix_type']=5

		self.master.mav.gps_input_send(
			self.data['time_usec'],
			self.data['gps_id'],
			self.data['ignore_flags'],
			self.data['time_week_ms'],
			self.data['time_week'],
			self.data['fix_type'],
			self.data['lat'],
			self.data['lon'],
			self.data['alt'],
			self.data['hdop'],
			self.data['vdop'],
			self.data['vn'],
			self.data['ve'],
			self.data['vd'],
			self.data['speed_accuracy'],
			self.data['horiz_accuracy'],
                	self.data['vert_accuracy'],
                	self.data['satellites_visible'])

	def global_point_from_vector(self):
		#lat_reference_rad=self.reference_lat*self.DEG_TO_RAD
		#lon_reference_rad=self.reference_lon*self.DEG_TO_RAD
		self.current_lat=( self.reference_lat_rad + self.tag_pos_ned.x/self.CONSTANTS_RADIUS_OF_EARTH )*self.RAD_TO_DEG
        	self.current_lon=( self.reference_lon_rad + self.tag_pos_ned.y/self.target_lon_param )*self.RAD_TO_DEG
		if self.debug == 2:
			print("Current lat:"+str(self.current_lat)+"; lon:"+str(self.current_lon))

	def convert_to_ned(self,vector):
		ned_vector=Coordinates()
		#ned_vector.x=vector.x*math.cos(self.yaw)-vector.y*math.sin(self.yaw)
		ned_vector.x=vector.x*self.cos_yaw - vector.y*self.sin_yaw
		ned_vector.y=vector.x*self.sin_yaw + vector.y*self.cos_yaw
		ned_vector.z=vector.z
		return  ned_vector

	def idle_task(self):
		'''get location by uwb'''
		if self.pozyx== None:
			print("pozyx dev is none")
			return

		self.get_location()
		if self.location_update:
                        self.tag_pos_ned=self.convert_to_ned(self.position) #m
                        self.tag_velocity_ned=self.convert_to_ned(self.velocity) #m/s
                        self.tag_velocity_ned.z=-self.tag_velocity_ned.z  #ned
                        self.global_point_from_vector()
                        self.send_gps_message()
                        self.location_update=False
                        if self.debug == 1:
                                now=time.time()
                                print("update hz:"+str(1/(now-self.location_update_time)))
                                self.location_update_time=now

def init(mpstate):
    '''initialise module'''
    return BeaconToGPS(mpstate)
