#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Author: HOANG VAN QUANG - BEE
# DATE: 03/08/2022

# from message_pkg.msg import *
from message_pkg.msg import CAN_send, CAN_status, CAN_received
from sti_msgs.msg import *
from geometry_msgs.msg import Twist
import time
import rospy

# - Pape 53
class statusWord_more:
	def __init__(self, id_driver):
		self.idDriver = id_driver
		self.ReadyToSwitchOn = 0
		self.SwitchOn = 0
		self.OperationEnable = 0
		self.Fault = 0
		self.VoltageOutput = 0
		self.QuickStop = 0
		self.SwitchOnDisabled = 0
		self.ModeSpecific_bit8 = 0
		self.Remote = 0
		self.TargetReached = 0
		self.PositionLimitActive = 0
		self.ModeSpecific_bit12 = 0
		self.ModeSpecific_bit13 = 0
		self.Speed = 0 # - RPM
		self.Alarm = 0
		self.ErrorCode = 0
		self.Mean_ErrorCode = ""

# - Pape 71
class controlWord_more:
	def __init__(self, id_driver):
		self.resetNode = 0
		self.idDriver = id_driver
		self.remote = 0
		self.resetCommunication = 0
		self.prePperational = 0

		self.operationEnable = 0
		self.resetAlarm = 0

		self.modeOfOperation = 0
		self.controlWord = 0
		self.profileAcceleration = 0
		self.profileDeceleration = 0

		self.targetVelocity = 0
		self.rotationDirection = 0
	

class CAN_CONVERT_ELD2():
	def __init__(self):
		print("ROS Initial!")
		rospy.init_node('CAN_CONVERT_ELD2', anonymous=False)
		# -------------
		self.frequence_sendCAN = 20 # self.frequence_control*self.numberFrame_communicate # - Hz
		self.saveTime_sendCAN = time.time()
		self.numberStep = 2. # 
		self.frequence_loop = self.frequence_sendCAN*self.numberStep # hz
		# -------------
		self.rate = rospy.Rate(self.frequence_loop)
		# ------------- ROS ------------- #

		# ------------- CAN ------------- #
		# rospy.Subscriber("/CAN_status", CAN_status, self.statusCAN_callback)
		# self.CAN_status = CAN_status()

		rospy.Subscriber("/CAN_received", CAN_received, self.CAN_callback)
		self.data_receivedCAN = CAN_received()

		self.pub_sendCAN = rospy.Publisher("/CAN_send", CAN_send, queue_size= 10)
		self.data_sendCan = CAN_send()

		# -- 
		self.radiator = 20
		self.pulsesPerRotation = 10000 # - 6092h - Pape 29
		# ------------- Control ------------- #
		self.isRevert_driver1 = 0
		self.isRevert_driver2 = 0
		self.isRevert_driver3 = 0
		self.isRevert_driver4 = 0

		# ------------- Status ------------- #
		self.status_driver1 = statusWord_more(1)
		self.status_driver2 = statusWord_more(2)
		self.status_driver3 = statusWord_more(3)
		self.status_driver4 = statusWord_more(4)

		self.control_driver1 = controlWord_more(1)
		self.control_driver2 = controlWord_more(2)
		self.control_driver3 = controlWord_more(3)
		self.control_driver4 = controlWord_more(4)

		

		self.sort_send = 1
		self.sort_loop = 1
		self.switch_id = 0
		# -------------
		self.configRun_resetNode = 0
		self.configRun_resetCommunication = 1
		self.configRun_enterPreOperational = 2
		self.configRun_startRemote = 3
		self.configRun_stopRemote = 4
		self.configRun_controlWord_06 = 5
		self.configRun_controlWord_07 = 6
		self.configRun_enableServo = 7
		self.configRun_disableServo = 8
		self.configRun_operationMode = 9
		self.configRun_acceleration = 10
		self.configRun_deceleration = 11
		# self.configRun_ = 0

	def CAN_callback(self, dat):
		self.data_receivedCAN = dat
		# -- RECEIVED CAN
		self.receivedCAN_run()

	def statusCAN_callback(self, dat):
		self.CAN_status = dat

	def int_to_binary8bit(self, int_value):
		value_now = int_value
		arr_bit = [0, 0, 0, 0, 0, 0, 0, 0]

		if (int_value > 255):
			print ("int_to_binary8bit Error Max: ", int_value)
			arr_bit = [1, 1, 1, 1, 1, 1, 1, 1]
		elif (int_value < 0):
			print ("int_to_binary8bit Error Min: ", int_value)
			arr_bit = [0, 0, 0, 0, 0, 0, 0, 0]
		else:
		    if value_now > 2**7:
		        arr_bit[0] = 1
		        value_now = value_now - (2**7)
				
		    if value_now > 2**6:
		        arr_bit[1] = 1
		        value_now = value_now - (2**6)

		    if value_now > 2**5:
		        arr_bit[2] = 1
		        value_now = value_now - (2**5)

		    if value_now > 2**4:
		        arr_bit[3] = 1
		        value_now = value_now - (2**4)

		    if value_now > 2**3:
		        arr_bit[4] = 1
		        value_now = value_now - (2**3)

		    if value_now > 2**2:
		        arr_bit[5] = 1
		        value_now = value_now - (2**2)

		    if value_now > 2**1:
		        arr_bit[6] = 1
		        value_now = value_now - (2**1)

		    if value_now >= 2**0:
		        arr_bit[7] = 1
		        value_now = value_now - (2**0)
		return arr_bit	

	def int_to_binary16bit(self, int_value):
		value_now = int_value
		arr_bit = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

		if (int_value >= 2**16):
			print ("int_to_binary16bit Error Max: ", int_value)
			arr_bit = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
		elif (int_value < 0):
			print ("int_to_binary16bit Error Min: ", int_value)
			arr_bit = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
		else:
		    if value_now > 2**15:
		        arr_bit[0] = 1
		        value_now = value_now - (2**15)
				
		    if value_now > 2**14:
		        arr_bit[1] = 1
		        value_now = value_now - (2**14)

		    if value_now > 2**13:
		        arr_bit[2] = 1
		        value_now = value_now - (2**13)

		    if value_now > 2**12:
		        arr_bit[3] = 1
		        value_now = value_now - (2**12)

		    if value_now > 2**11:
		        arr_bit[4] = 1
		        value_now = value_now - (2**11)

		    if value_now > 2**10:
		        arr_bit[5] = 1
		        value_now = value_now - (2**10)

		    if value_now > 2**9:
		        arr_bit[6] = 1
		        value_now = value_now - (2**9)

		    if value_now >= 2**8:
		        arr_bit[7] = 1
		        value_now = value_now - (2**8)

		    if value_now > 2**7:
		        arr_bit[8] = 1
		        value_now = value_now - (2**7)
				
		    if value_now > 2**6:
		        arr_bit[9] = 1
		        value_now = value_now - (2**6)

		    if value_now > 2**5:
		        arr_bit[10] = 1
		        value_now = value_now - (2**5)

		    if value_now > 2**4:
		        arr_bit[11] = 1
		        value_now = value_now - (2**4)

		    if value_now > 2**3:
		        arr_bit[12] = 1
		        value_now = value_now - (2**3)

		    if value_now > 2**2:
		        arr_bit[13] = 1
		        value_now = value_now - (2**2)

		    if value_now > 2**1:
		        arr_bit[14] = 1
		        value_now = value_now - (2**1)

		    if value_now >= 2**0:
		        arr_bit[15] = 1
		        value_now = value_now - (2**0)
		return arr_bit	

	def int_to_4bytes(self, value): # => STT: [byte No.0, byte No.1, byte No.2, byte No.3]
		value_now = 0
		if (value >= 0):
			value_now = value
		else:
			value_now = (256**4) + value

		byteArray = [0, 0, 0, 0]
		byteArray[3] =  value_now/(256**3)
		byteArray[2] = (value_now - byteArray[3]*(256**3))/256**2
		byteArray[1] = (value_now - byteArray[3]*(256**3) - byteArray[2]*(256**2) )/256
		byteArray[0] =  value_now - byteArray[3]*(256**3) - byteArray[2]*(256**2) - byteArray[1]*256
		return byteArray

	def bitArray_toInt(self, arrBit):
		value_out = 0
		leng = len(arrBit)
		for i in range(leng):
			if (arrBit[leng - i - 1] == 1):
				value_out += 2**i
			print ("b: ", leng - i - 1)
		return value_out

	# - Pape 71.
	def controlWord(self, Halt, Fault_reset, Mode_specific, Enable_operation, Quick_stop, Enable_voltage, Switch_on):
		bit_arr = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
		# - bit No.8
		bit_arr[7] = Halt
		# - bit No.7
		bit_arr[8] = Fault_reset
		# - bit No.6~4
		bits = int_to_binary8bit(Mode_specific)
		bit_arr[9] = bits[2]
		bit_arr[10] = bits[1]
		bit_arr[11] = bits[0]
		# - bit No.3
		bit_arr[12] = Enable_operation
		# - bit No.2
		bit_arr[13] = Quick_stop
		# - bit No.1
		bit_arr[14] = Enable_voltage
		# - bit No.0
		bit_arr[15] = Switch_on

		return bitArray_toInt(bit_arr)

	def convert_4byte_int(self, byte0, byte1, byte2, byte3):
		int_out = 0
		int_out = byte0 + byte1*256 + byte2*256*256 + byte3*256*256*256
		if int_out > pow(2, 32)/2.:
			int_out = int_out - pow(2, 32)
		return int_out

	def getBit_fromInt8(self, value_in, pos):
		bit_out = 0
		value_now = value_in
		for i in range(8):
			bit_out = value_now%2
			value_now = value_now/2
			if (i == pos):
				return bit_out

			if (value_now < 1):
				return 0		
		return 0

	def bit16_to_int(self, bit_arr): # - arr[bit15, ..., bit0]
		# Ex: bit_arr = '0000000000001111'
		return int(bit_arr, 2)

	def read_statusWord(self, data_received):
		statusWord_data = statusWord()
		if (data_received.idSend == 132 and data_received.byte1 == 65 and data_received.byte2 == 96):
			value_statusWord = self.convert_4byte_int(data_received.byte4, data_received.byte5, data_received.byte6, data_received.byte7)
			bit16_arr = self.int_to_binary16bit(value_statusWord)
			statusWord_data.ReadyToSwitchOn = bit16_arr[0]
			statusWord_data.SwitchOn = bit16_arr[1]
			statusWord_data.OperationEnable = bit16_arr[2]
			statusWord_data.Fault = bit16_arr[3]
			statusWord_data.VoltageOutput = bit16_arr[4]
			statusWord_data.QuickStop = bit16_arr[5]
			statusWord_data.SwitchOnDisabled = bit16_arr[6]
			statusWord_data.ModeSpecific_bit8 = bit16_arr[8]
			statusWord_data.Remote = bit16_arr[9]
			statusWord_data.TargetReached = bit16_arr[10]
			statusWord_data.PositionLimitActive = bit16_arr[11]
			statusWord_data.ModeSpecific_bit12 = bit16_arr[12]
			statusWord_data.ModeSpecific_bit13 = bit16_arr[13]
		return statusWord_data

	def analysis_dataReceived(self, data_received):
		speed_rpm = 0
		# - 606Ch - Actual feedback speed value - Pape 28.
		if (data_received.idSend == 132 and data_received.byte1 == 108 and data_received.byte2 == 96):
			by0 = data_received.byte4
			by1 = data_received.byte5
			by2 = data_received.byte6
			by3 = data_received.byte7
			speed_pps = self.convert_4byte_int(by0, by1, by2, by3)
			speed_rpm = (speed_pps*60)/(self.pulsesPerRotation*self.radiator)
			print ("Read speed_rpm: ", speed_rpm)
		return speed_rpm 

	def errorCode_list(self, value): # - 603Fh - Pape 106
		switcher={
			8721: 'Over-current!',
			8722: 'Over-current of intelligent power module (IPM)',
			12624: 'Current detection circuit error',
			12625: 'Current detection circuit error',
			12627: 'Power line(U、V、W)break',
			12801: 'DC bus circuit error',
			12817: 'DC bus over-voltage',
			12833: 'DC bus under-voltage',
			16912: 'Drive over-heat',
			21808: 'CRC verification error when EEPROM parameter saved',
			21809: 'I2C Communication status error',
			21810: 'Read/write history alarm error',
			21811: 'Read/write diagnostic data error',
			21812: 'Read/write bus communication parameters error',
			21813: 'Read/write 402 parameters error',
			25377: 'Input interface allocation error',
			25378: 'Input interface function set error',
			25379: 'Output interface function set error',
			25385: 'FPGA communication error',
			28962: 'Motor code error',
			29473: 'Encoder wiring error',
			29474: 'Encoder data error',
			29475: 'Encoder initial position error',
			29476: 'Encoder data error',
			29481: 'Positive/negative limit input active',
			30465: 'Brake resistor discharged circuit overload',
			30466: 'Brake resistor error',
			33040: 'CAN bus over-run',
			33056: 'CAN in error passive mode',
			33072: 'Lifeguard error',
			33088: 'Recovered from CAN bus off.',
			33089: 'CAN Bus off occurred.',
			33104: 'ID error',
			33552: 'Motor over-load',
			33553: 'Drive over-load',
			33541: 'Torque saturation alarm',
			33793: 'Vibration is too large',
			33794: 'Over-speed 1',
			33795: 'Motor speed out of control',
			34051: 'Electronic gear ratio error',
			34321: 'Too large position pulse deviation',
			34320: 'Too large velocity deviation',
			34322: 'Position pulse input frequency error'
		}
		return switcher.get(value, 'Not found!')

	def check_frame(self):
		self.data_sendCan.id = 1536 + 4
		self.data_sendCan.byte0 = 64 	# 40
		self.data_sendCan.byte1 = 108 	# 6Ch
		self.data_sendCan.byte2 = 96  	# 60h
		self.data_sendCan.byte3 = 0   	# 
		self.data_sendCan.byte4 = 0 	# 
		self.data_sendCan.byte5 = 0 	# 
		self.data_sendCan.byte6 = 0
		self.data_sendCan.byte7 = 0
		self.sort_send = 2

		self.pub_sendCAN.publish(self.data_sendCan)

	def checkFrame_readSpeed(self, ID):
		self.data_sendCan.id = 1536 + ID
		self.data_sendCan.byte0 = 64 	# 40
		self.data_sendCan.byte1 = 108 	# 6Ch
		self.data_sendCan.byte2 = 96  	# 60h
		self.data_sendCan.byte3 = 0   	# 
		self.data_sendCan.byte4 = 0 	# 
		self.data_sendCan.byte5 = 0 	# 
		self.data_sendCan.byte6 = 0
		self.data_sendCan.byte7 = 0
		self.pub_sendCAN.publish(self.data_sendCan)

	def receivedCAN_run(self):
		self.readSpeed()

	def string_array(self):
		arr_str = "0123456"
		print ("OUT0: ", arr_str[2:4])
		print ("OUT1: ", arr_str[0:2])
		print ("OUT2: ", arr_str[:2])
		print ("OUT3: ", arr_str[:-3])
		print ("OUT4: ", arr_str[-2:0])
		print ("OUT5: ", arr_str[2:0])

	def config_begin(self):
		# - 
		pass

	def config_run(self, type_config, ID):
		data_sendCan = CAN_send()
		# Pape 61
		# - Reset Node | 
		if (type_config == 0)
			data_sendCan.id = 0
			data_sendCan.byte0 = 129 # 81h
			data_sendCan.byte1 = ID  # h
			data_sendCan.byte2 = 0   # h
			data_sendCan.byte3 = 0
			data_sendCan.byte4 = 0
			data_sendCan.byte5 = 0
			data_sendCan.byte6 = 0
			data_sendCan.byte7 = 0

		# - Reset communication | 
		elif (type_config == 1)
			data_sendCan.id = 0
			data_sendCan.byte0 = 130 # 82h
			data_sendCan.byte1 = ID  # h
			data_sendCan.byte2 = 0   # h
			data_sendCan.byte3 = 0
			data_sendCan.byte4 = 0
			data_sendCan.byte5 = 0
			data_sendCan.byte6 = 0
			data_sendCan.byte7 = 0

		# - Enter pre-operational | 
		elif (type_config == 2)
			data_sendCan.id = 0
			data_sendCan.byte0 = 128 # 80h
			data_sendCan.byte1 = ID  # h
			data_sendCan.byte2 = 0   # h
			data_sendCan.byte3 = 0
			data_sendCan.byte4 = 0
			data_sendCan.byte5 = 0
			data_sendCan.byte6 = 0
			data_sendCan.byte7 = 0

		# - Start remote control | 
		elif (type_config == 3)
			data_sendCan.id = 0
			data_sendCan.byte0 = 1  # h
			data_sendCan.byte1 = ID # h
			data_sendCan.byte2 = 0  # h
			data_sendCan.byte3 = 0
			data_sendCan.byte4 = 0
			data_sendCan.byte5 = 0
			data_sendCan.byte6 = 0
			data_sendCan.byte7 = 0

		# - Stop remote control | 
		elif (type_config == 4)
			data_sendCan.id = 0
			data_sendCan.byte0 = 2  # h
			data_sendCan.byte1 = ID # h
			data_sendCan.byte2 = 0  # h
			data_sendCan.byte3 = 0
			data_sendCan.byte4 = 0
			data_sendCan.byte5 = 0
			data_sendCan.byte6 = 0
			data_sendCan.byte7 = 0

		# - Control word | 06h
		elif (type_config == 5)
			data_sendCan.id = 1538 + ID
			data_sendCan.byte0 = 43 # 2Bh
			data_sendCan.byte1 = 64 # 40h
			data_sendCan.byte2 = 96 # 60h
			data_sendCan.byte3 = 0
			data_sendCan.byte4 = 6
			data_sendCan.byte5 = 0
			data_sendCan.byte6 = 0
			data_sendCan.byte7 = 0

		# - Control word | 07h
		elif (type_config == 6)
			data_sendCan.id = 1538 + ID
			data_sendCan.byte0 = 43 # 2Bh
			data_sendCan.byte1 = 64 # 40h
			data_sendCan.byte2 = 96 # 60h
			data_sendCan.byte3 = 0
			data_sendCan.byte4 = 7
			data_sendCan.byte5 = 0
			data_sendCan.byte6 = 0
			data_sendCan.byte7 = 0

		# - Control word | 0Fh | Servo-Enabled
		elif (type_config == 7)
			data_sendCan.id = 1538 + ID
			data_sendCan.byte0 = 43 # 2Bh
			data_sendCan.byte1 = 64 # 40h
			data_sendCan.byte2 = 96 # 60h
			data_sendCan.byte3 = 0
			data_sendCan.byte4 = 15 # 0Fh
			data_sendCan.byte5 = 0
			data_sendCan.byte6 = 0
			data_sendCan.byte7 = 0

		# - Control word | 0Fh | Servo-Disable
		elif (type_config == 8)
			data_sendCan.id = 1538 + ID
			data_sendCan.byte0 = 43 # 2Bh
			data_sendCan.byte1 = 64 # 40h
			data_sendCan.byte2 = 96 # 60h
			data_sendCan.byte3 = 0
			data_sendCan.byte4 = 0  # 0h
			data_sendCan.byte5 = 0
			data_sendCan.byte6 = 0
			data_sendCan.byte7 = 0

		# - Mode of operation | 03h | operation mode
		elif (type_config == 9)
			data_sendCan.id = 1538 + ID
			data_sendCan.byte0 = 47 # 2Fh
			data_sendCan.byte1 = 96 # 60h
			data_sendCan.byte2 = 96 # 60h
			data_sendCan.byte3 = 0
			data_sendCan.byte4 = 3 # 03h
			data_sendCan.byte5 = 0
			data_sendCan.byte6 = 0
			data_sendCan.byte7 = 0

		# - Profile acceleration | 0h | Pape 29
		elif (type_config == 10)
			data_sendCan.id = 1538 + ID
			data_sendCan.byte0 = 35  # 23h
			data_sendCan.byte1 = 131 # 83h
			data_sendCan.byte2 = 96  # 60h
			data_sendCan.byte3 = 0
			data_sendCan.byte4 = 142 # h
			data_sendCan.byte5 = 208 # h
			data_sendCan.byte6 = 0
			data_sendCan.byte7 = 0

		# - Profile deceleration | 0h | 
		elif (type_config == 11)
			data_sendCan.id = 1538 + ID
			data_sendCan.byte0 = 35  # 23h
			data_sendCan.byte1 = 132 # 83h
			data_sendCan.byte2 = 96  # 60h
			data_sendCan.byte3 = 0
			data_sendCan.byte4 = 142 # h
			data_sendCan.byte5 = 208 # h
			data_sendCan.byte6 = 0
			data_sendCan.byte7 = 0

	def setVelocity(self, ID, velocity_rpm): # - (velocity_rpm - Tốc độ trục đã qua hộp số)
		data_sendCan = CAN_send()
		# - Target velocity | 0h | 
		data_sendCan.id = 1538 + ID
		data_sendCan.byte0 = 35  # 23h
		data_sendCan.byte1 = 255 # FFh
		data_sendCan.byte2 = 96  # 60h
		data_sendCan.byte3 = 0

		PPS = (velocity_rpm/60)*self.pulsesPerRotation*self.radiator
		arr = self.int_to_4bytes(PPS)
		data_sendCan.byte4 = arr[0] # h
		data_sendCan.byte5 = arr[1] # h
		data_sendCan.byte6 = arr[2]
		data_sendCan.byte7 = arr[3]

		return data_sendCan

	def control_drivers(self):
		# --------------- resetNode --------------- #
		if (self.sort_loop == 1):
			if (self.control_driver1.resetNode == 1):
				self.data_sendCan = self.config_run(self.configRun_resetNode, self.control_driver1.idDriver)
			self.sort_loop = 2

		elif (self.sort_loop == 2):
			if (self.control_driver2.resetNode == 1):
				self.data_sendCan = self.config_run(self.configRun_resetNode, self.control_driver2.idDriver)
			self.sort_loop = 3

		elif (self.sort_loop == 3):
			if (self.control_driver3.resetNode == 1):
				self.data_sendCan = self.config_run(self.configRun_resetNode, self.control_driver3.idDriver)
			self.sort_loop = 4

		elif (self.sort_loop == 4):
			if (self.control_driver4.resetNode == 1):
				self.data_sendCan = self.config_run(self.configRun_resetNode, self.control_driver4.idDriver)
			self.sort_loop = 11

		# --------------- Reset Communication --------------- #
		if (self.sort_loop == 11):
			if (self.control_driver1.resetCommunication == 1):
				self.data_sendCan = self.config_run(self.configRun_resetCommunication, self.control_driver1.idDriver)
			self.sort_loop = 12

		elif (self.sort_loop == 12):
			if (self.control_driver2.resetCommunication == 1):
				self.data_sendCan = self.config_run(self.configRun_resetCommunication, self.control_driver2.idDriver)
			self.sort_loop = 13

		elif (self.sort_loop == 13):
			if (self.control_driver3.resetCommunication == 1):
				self.data_sendCan = self.config_run(self.configRun_resetCommunication, self.control_driver3.idDriver)
			self.sort_loop = 14

		elif (self.sort_loop == 14):
			if (self.control_driver4.resetCommunication == 1):
				self.data_sendCan = self.config_run(self.configRun_resetCommunication, self.control_driver4.idDriver)
			self.sort_loop = 21

		# --------------- Enter Pre-operational --------------- #
		if (self.sort_loop == 21):
			if (self.control_driver1.prePperational == 1):
				self.data_sendCan = self.config_run(self.configRun_enterPreOperational, self.control_driver1.idDriver)
			self.sort_loop = 22

		elif (self.sort_loop == 22):
			if (self.control_driver2.prePperational == 1):
				self.data_sendCan = self.config_run(self.configRun_enterPreOperational, self.control_driver2.idDriver)
			self.sort_loop = 23

		elif (self.sort_loop == 23):
			if (self.control_driver3.prePperational == 1):
				self.data_sendCan = self.config_run(self.configRun_enterPreOperational, self.control_driver3.idDriver)
			self.sort_loop = 24

		elif (self.sort_loop == 24):
			if (self.control_driver4.prePperational == 1):
				self.data_sendCan = self.config_run(self.configRun_enterPreOperational, self.control_driver4.idDriver)
			self.sort_loop = 31

		# --------------- Remote --------------- #
		if (self.sort_loop == 31):
			if (self.control_driver1.remote == 1):
				self.data_sendCan = self.config_run(self.configRun_startRemote, self.control_driver1.idDriver)
			else:
				self.data_sendCan = self.config_run(self.configRun_stopRemote, self.control_driver1.idDriver)
			self.sort_loop = 32

		elif (self.sort_loop == 32):
			if (self.control_driver2.remote == 1):
				self.data_sendCan = self.config_run(self.configRun_startRemote, self.control_driver2.idDriver)
			else:
				self.data_sendCan = self.config_run(self.configRun_stopRemote, self.control_driver2.idDriver)
			self.sort_loop = 33

		elif (self.sort_loop == 33):
			if (self.control_driver3.remote == 1):
				self.data_sendCan = self.config_run(self.configRun_startRemote, self.control_driver3.idDriver)
			else:
				self.data_sendCan = self.config_run(self.configRun_stopRemote, self.control_driver3.idDriver)
			self.sort_loop = 34

		elif (self.sort_loop == 44):
			if (self.control_driver4.remote == 1):
				self.data_sendCan = self.config_run(self.configRun_startRemote, self.control_driver4.idDriver)
			else:
				self.data_sendCan = self.config_run(self.configRun_stopRemote, self.control_driver4.idDriver)
			self.sort_loop = 41

		# --------------- Operation Enable --------------- #
		if (self.sort_loop == 51):
			if (self.control_driver1.resetNode == 1):
				self.data_sendCan = self.config_run(self.configRun_stopRemote, self.control_driver1.idDriver)
			self.sort_loop = 52

		elif (self.sort_loop == 52):
			if (self.control_driver2.resetNode == 1):
				self.data_sendCan = self.config_run(self.configRun_stopRemote, self.control_driver2.idDriver)
			self.sort_loop = 53

		elif (self.sort_loop == 53):
			if (self.control_driver3.resetNode == 1):
				self.data_sendCan = self.config_run(self.configRun_stopRemote, self.control_driver3.idDriver)
			self.sort_loop = 54

		elif (self.sort_loop == 54):
			if (self.control_driver4.resetNode == 1):
				self.data_sendCan = self.config_run(self.configRun_stopRemote, self.control_driver4.idDriver)
			self.sort_loop = 61


		self.pub_sendCAN.publish(self.data_sendCan)

	def run(self):
	    while not rospy.is_shutdown():
			# -- SEND CAN
			delta_time = (time.time() - self.saveTime_sendCAN)%60 
			if (delta_time > 1/self.frequence_sendCAN):
				self.saveTime_sendCAN = time.time()


            self.rate.sleep()

def main():
	print('Program starting')
	program = CAN_ROS()
	program.run()
	# program.try_run()
	# program.string_arry()
	print('Programer stopped')

if __name__ == '__main__':
    main()

"""
Cài đặt:
- Chế độ hoạt động.
- Thời gian tăng tốc.
- Thời gian giảm tốc.

Bước điều khiển:
1, Kết nối - Remote.
2, Bật Driver.
3, Cài chế độ điều khiển.
4, 
""