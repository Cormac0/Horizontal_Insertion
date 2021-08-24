import numpy as np
import cv2
import cv2.aruco as aruco
import glob
import argparse
import pdb
import time
import csv
import serial
import datetime
import math
from scipy.spatial.transform import Rotation as R
import apriltag

from OdriveClass import *



#pip install apriltag
class FullSystem():
#https://medium.com/@aliyasineser/aruco-marker-tracking-with-opencv-8cb844c26628
	def __init__(self):
			
			self.testname=input("Enter Test name: ")
			self.docalibrate=int(input("Calibrate Odrives? 3 for yes: "))
			print(self.testname)
			self.camnum1=input('video camera number1: ')
			# self.camnum2=input('video camera number2: ')
			self.vidname='/home/rachel/odrive/Data_and_Vids/'+ self.testname
			self.filename=self.vidname+".csv"
			self.vidfile1=self.vidname+".avi"
			self.vidfile2=self.vidname+"2"+".avi"

			self.ser_add='/dev/ttyACM2'

			self.cap1 = cv2.VideoCapture(int(self.camnum1))	
			self.frame_width1 = int(self.cap1.get(3))
			self.frame_height1 = int(self.cap1.get(4))
			self.out1 = cv2.VideoWriter(self.vidfile1,cv2.VideoWriter_fourcc('M','J','P','G'), 10, (self.frame_width1,self.frame_height1))
			self.out2 = cv2.VideoWriter(self.vidfile2,cv2.VideoWriter_fourcc('M','J','P','G'), 10, (self.frame_width1,self.frame_height1))
			self.window = 'Camera'


			#Load Camera Params
			options = apriltag.DetectorOptions(families="tag36h11")
			self.detector = apriltag.Detector(options)

			self.cv_file = cv2.FileStorage("camera.yml", cv2.FILE_STORAGE_READ)
			#self.camera_params=[571.9581849983156, 571.4978038888228, 331.32548130391393, 303.2562440785643]
			self.camera_params=[645.6106501798513, 641.8595843993536, 314.3376844316258, 253.36556605212172]

			self.tag_size=.079
			# note we also have to specify the type to retrieve other wise we only get a
			# FileNode object back instead of a matrix
			self.matrix_coefficients = self.cv_file.getNode("K").mat()
			self.distortion_coefficients = self.cv_file.getNode("D").mat()
			self.cv_file.release()




			# self.cap2 = cv2.VideoCapture(int(self.camnum2))	
			# self.frame_width2 = int(self.cap2.get(3))
			# self.frame_height2 = int(self.cap2.get(4))
			# self.out2 = cv2.VideoWriter(self.vidfile2,cv2.VideoWriter_fourcc('M','J','P','G'), 10, (self.frame_width2,self.frame_height2))






			self.D=(2+.002)*.0254
			self.d=2*.0254
			self.emin=(math.sqrt(self.D**2-self.d**2))*math.cos(math.asin(math.sqrt(1-(self.d/self.D)**2)))
			self.list_of_floats=[]




			self.xval1=0
			self.yval1=0
			self.buttjpin=0
			self.butt1pin=0
			self.butt2pin=0
			self.butt3pin=0
			self.str1=0
			self.str2=0
			self.str3=0
			self.ytilt=0
			self.ztilt=0
			self.qw=0
			self.qx=0
			self.qy=0
			self.qz=0
			self.accx=0
			self.accy=0
			self.accz=0
			self.sys_cal=0
			self.gyro_cal=0
			self.acc_cal=0
			self.mag_cal=0
			
			self.accz_thresh_wedgeBreaking=0.5
			self.ytilt_zero=-.81
			self.ztilt_zero=2.87
			self.accx_zero=--.133
			self.accy_zero=-.5
			self.accz_zero=10.08
			self.angle_Zthresh=.75
			self.angle_Ythresh=.75
			self.exitholeflag=0

			self.psiwedge=0

			self.pitch=0
			self.roll=0


			self.psi=0


			self.v4=0.0
			self.v4_prev=0.0
			self.mot1spd=0.0
			self.mot1spd_prev=0.0
			self.mot2spd=0.0
			self.mot2spd_prev=0.0
			self.mot3spd=0.0
			self.mot3spd_prev=0.0

			self.ytiltw=0
			self.ztiltw=0

			self.sv=0

			if self.sv==1:
				self.str1thresh=20
				self.str2thresh=20
				self.str3thresh=20
			else:
				self.str1thresh=5
				self.str2thresh=5
				self.str3thresh=5

			#print([self.str1thresh,self.str2thresh,self.str3thresh])
			#time.sleep(.5)
			# get calibration parameters

			self.list_of_floats=[]
			self.list_of_floats_temp=[]
			self.TotalList=[]


			# #ARUCO STUFF
			# self.markersize=0.0996
			# self.rveclist=[]
			# self.tveclist=[]
			# self.firstMarkerID = 0	#peg
			# self.secondMarkerID = 1	#hole
			# self.thirdMarkerID= 2	 #sys
			# self.des_num_ids=1
			# self.pointCircle = (0, 0)
			# self.markerTvecList = []
			# self.markerRvecList = []
			# self.markerXvecList=[]
			# self.markerYvecList=[]
			# self.markerZvecList=[]
			# self.composedRvec, self.composedTvec = None, None
			self.pegrvec=[0,0,0]
			self.pegtvec=[0,0,0]
			self.holervec=[0,0,0]
			self.holetvec=[0,0,0]
			self.difftvec=[0,0,0]
			self.diffrvec=[0,0,0]

			## DEPTH VALS
			self.depth_zero1=-12.7
			self.depth_max1=517

			self.depth_zero_meas=-12.7
			self.depth_max_meas=395
			self.depth=0
			self.depth_1=0
			self.depth_2=0

			self.depth_1_curavg=0
			self.depth_2_curavg=0



	#OLD ARUCO FUNCS
	# def read_node_real( self,reader, name ):
	# 	node = reader.getNode( name )
	# 	return node.real()

	# def read_node_string(self, reader, name ):
	# 	node = reader.getNode( name )
	# 	return node.string()

	# def read_node_matrix( self,reader, name ):
	# 	node = reader.getNode( name )
	# 	return node.mat()

	# def inversePerspective(self,rvec, tvec):
	# 		#""" Applies perspective transform for given rvec and tvec. """
	# 		R, _ = cv2.Rodrigues(rvec)
	# 		R = np.matrix(R).T
	# 		invTvec = np.dot(R, np.matrix(-tvec))
	# 		invRvec, _ = cv2.Rodrigues(R)
	# 		return invRvec, invTvec


	# def relativePosition(self,rvec1, tvec1, rvec2, tvec2):
	# 		""" Get relative position for rvec2 & tvec2. Compose the returned rvec & tvec to use composeRT with rvec2 & tvec2 """

	# 		rvec1, tvec1 = rvec1.reshape((3, 1)), tvec1.reshape((3, 1))
	# 		rvec2, tvec2 = rvec2.reshape((3, 1)), tvec2.reshape((3, 1))

	# 		# Inverse the second marker, the right one in the image
	# 		invRvec, invTvec = self.inversePerspective(rvec2, tvec2)

	# 		info = cv2.composeRT(rvec1, tvec1, invRvec, invTvec)
	# 		composedRvec, composedTvec = info[0], info[1]

	# 		composedRvec = composedRvec.reshape((3, 1))
	# 		composedTvec = composedTvec.reshape((3, 1))
	# 		return composedRvec, composedTvec





	##### MOVEMENT FUNCS

	def ConnectToOdrive(self):
		self.od0 = Odrive('208637853548')
		self.od1 = Odrive('2061377C3548')

		if self.docalibrate==3:
			#Calibrate Odrive
			print('ODrive 0 Calibrating')
			self.od0.full_init()
			time.sleep(2)
			print('ODrive 1 Calibrating')
			self.od1.full_init()
			print('Calibration Complete')


	def getJoystickMotorSpeed(self,spd):

		self.getJoystickSpeed(spd)

		if self.butt1pin==0 :
			self.mot1spd=0
		else:
			self.mot1spd=self.v4
		
		if self.butt2pin==0 :
			self.mot2spd=0

		else:
			self.mot2spd=self.v4

		if self.butt3pin==0 :
			self.mot3spd=0
		else:
			self.mot3spd=self.v4

	def getJoystickSpeed(self,spd=0):

		if spd==0:
			tocmd=10000

		if spd==1:
			tocmd=25000

		if self.yval1 < 300:
			#v4= -100000
			self.v4=-tocmd
			#print('v4')
		elif self.yval1 > 600:
			#v4=100000
			self.v4=tocmd
		elif (self.yval1> 300) and (self.yval1 < 600):
			self.v4=0
		
	

	def CmdMotors(self):
		if self.mot1spd != self.mot1spd_prev:
			self.od0.VelMove(self.mot1spd,0)
			self.mot1spd_prev=self.mot1spd

		if self.mot2spd != self.mot2spd_prev:
			self.od0.VelMove(self.mot2spd,1)
			self.mot2spd_prev=self.mot2spd

		if self.mot3spd != self.mot3spd_prev:
			self.od1.VelMove(self.mot3spd,1)
			self.mot3spd_prev=self.mot3spd


	def RaisePegCmdGen(self,spdv):
		if spdv==1:
			spd=10000
		if spdv==0:
			spd=25000
		self.mot1spd=-spd
		self.mot2spd=-spd
		self.mot3spd=-spd

	def LowerPegCmdGen(self,spdv):
		if spdv==1:
			spd=10000
		if spdv==0:
			spd=25000
		self.mot1spd=spd
		self.mot2spd=spd
		self.mot3spd=spd

	def StopMotCmdGen(self):
		spd=0
		self.mot1spd=spd
		self.mot2spd=spd
		self.mot3spd=spd

	def RaisePeg(self,spdv):
		self.RaisePegCmdGen(spdv)
		self.CmdMotors()

	def LowerPeg(self,spdv):
		self.LowerPegCmdGen(spdv)
		self.CmdMotors()

	def StopPeg(self):
		self.StopMotCmdGen()
		self.CmdMotors()

	def tension_cont_generate_mot_commands(self):
		strlist=[self.str1,self.str2,self.str3]
		print(strlist)

		maxpos = strlist.index(max(strlist))
		#print(maxpos)

		err1=strlist[maxpos]-strlist[0]
		err2=strlist[maxpos]-strlist[1]
		err3=strlist[maxpos]-strlist[2]

		#print(err1,err2,err3)

		Kp=50
		self.mot1spd=10000-Kp*err1
		self.mot2spd=10000-Kp*err2
		self.mot3spd=10000-Kp*err3
		# self.mot1spd=10000-int(round(self.map(err1,0,300,0,10000)))
		# self.mot2spd=10000-int(round(self.map(err2,0,300,0,10000)))
		# self.mot3spd=10000-int(round(self.map(err3,0,300,0,10000)))

		#print(self.mot1spd,self.mot2spd,self.mot3spd)

		if self.mot1spd > 24000:
			self.mot1spd=24000
		if self.mot2spd > 24000:
			self.mot2spd=24000
		if self.mot3spd > 24000:
			self.mot3spd=24000



	##### ARDUINO SERIAL FUNCS

	def ArduinoSetup(self):
		userinput=input('Setting up the arduino. If you restarted the arduino, unload everything and then enter 1 so it can calibrate')
		#print(type(int(userinput)))
		self.ser = serial.Serial(self.ser_add, 115200,timeout=5)
		
		self.ser.flushInput()
		
		self.ser.write(int(userinput))
		self.ser.flushInput()
		
		print("connected")


		# Calibrate Arduino if needed
		line=[]
		ctr=0
		while a.buttjpin==0:

				try:
						
						line = self.ser.readline()
						line.decode('ascii').strip()
						#print(line.decode('ascii').strip())
						list_of_floats_temp=[]
						list_of_floats_temp_2=[]
						list_of_floats_temp = [float(item) for item in line.decode('ascii').strip().split(';')]
						print(list_of_floats_temp)
						# if len(self.list_of_floats_temp)==13:
						# 	list_of_floats_temp2=list_of_floats_temp
						# 	#print(self.list_of_floats)
						# 	list_of_floats_temp2[8]=180-(360-list_of_floats_temp[8])
						# 	list_of_floats_temp2[9]=90-list_of_floats_temp[9]
						# print(list_of_floats_temp2)

						ctr=ctr+1
				except:
						pass

				#	 print("Keyboard Interrupt")
				finally:
						#if len(line)>0:
						#if line.decode('ascii').strip()=="good":
						if ctr>5:
								break

		



		
		# input('Everything calibrated. LOAD UP. Then serial data will display. Enter 1 to start ')
		# line=[]
		timeout_start=time.time()
		timeout=2;


		#display serialdata for 5 seconds to make sure it looks good
		while time.time() < timeout_start + timeout:
				self.get_data(0)
				print([	self.str1,self.str2,self.str3])

		input('If you are happy with the serial, press 1 to continue. otherwise, restart the python ')

	def ReadSerial(self,tosaveflag):

		


		self.ser.flushInput()

		while (self.ser.inWaiting()<30):
				pass

		try:		
				line = self.ser.readline()
				line2=line
				#print(len(line))
				# print(line)
				self.list_of_floats_temp=[]
				self.list_of_floats_temp = [float(item) for item in line.decode('ascii').strip().split(';')]


		except:
				pass

		# print(line.decode('ascii').strip())
		if len(self.list_of_floats_temp)==22:
				self.list_of_floats=[]
				self.list_of_floats=self.list_of_floats_temp
				self.xval1=self.list_of_floats[0]
				self.yval1=self.list_of_floats[1]
				self.buttjpin=self.list_of_floats[2]
				self.butt1pin=self.list_of_floats[3]
				self.butt2pin=self.list_of_floats[4]
				self.butt3pin=self.list_of_floats[5]
				self.str1=round(self.list_of_floats[6],1)
				self.str2=round(self.list_of_floats[7],1)
				self.str3=round(self.list_of_floats[8],1)
				self.ytilt=self.list_of_floats[9]
				self.ztilt=self.list_of_floats[10]
				self.qw=self.list_of_floats[11]
				self.qx=self.list_of_floats[12]
				self.qy=self.list_of_floats[13]
				self.qz=self.list_of_floats[14]
				self.accx=self.list_of_floats[15]
				self.accy=self.list_of_floats[16]
				self.accz=self.list_of_floats[17]
				self.sys_cal=self.list_of_floats[18]
				self.gyro_cal=self.list_of_floats[19]
				self.acc_cal=self.list_of_floats[20]
				self.mag_cal=self.list_of_floats[21]
				self.calculatepsi()
				self.list_of_floats.append(self.psi)
				self.list_of_floats.append(self.ytilt_zero)
				self.list_of_floats.append(self.ztilt_zero)
				self.list_of_floats.append(self.accx_zero)
				self.list_of_floats.append(self.accx_zero)
				self.list_of_floats.append(self.accz_zero)

				

				self.pitch = 180 * np.arctan2(self.accx ,np.sqrt(self.accy*self.accy+ self.accz*self.accz))/3.14;
				self.roll = 180 * np.arctan2(self.accy, np.sqrt(self.accx*self.accx + self.accz*self.accz))/3.14;

				self.list_of_floats.append(self.pitch)
				self.list_of_floats.append(self.roll)


				#append aruco stuff
				self.list_of_floats.extend(self.pegrvec)
				self.list_of_floats.extend(self.pegtvec)
				self.list_of_floats.extend(self.holervec)
				self.list_of_floats.extend(self.holetvec)

				self.getPegDepth()
				#print(self.depth_1,self.depth_2)
				
				self.list_of_floats.append(self.depth_1)

				
				self.list_of_floats.insert(0,time.time())

				if tosaveflag==1:
					self.DataToSave()
		

	def CalibrateIMU(self):
		self.buttjpin=0
		input("Calibrate the IMU. Press 1 to start, hit the joystick button 4 to stop")
		while self.buttjpin==0:
			self.get_data(0)
			print(self.buttjpin,self.sys_cal,self.gyro_cal,self.acc_cal,self.mag_cal)

		self.get_data(0)
		time.sleep(2)
		input("get IMU Data. Hit joystick button to stop")
		self.get_data(0)
		self.buttjpin=0
		while self.buttjpin==0:
			self.get_data(0)
			pitch = 180 * np.arctan2(self.accx ,np.sqrt(self.accy*self.accy+ self.accz*self.accz))/3.14;
			roll = 180 * np.arctan2(self.accy, np.sqrt(self.accx*self.accx + self.accz*self.accz))/3.14;
	
			#print([self.ytilt,self.ytilt-self.ytilt_zero, self.ztilt,self.ztilt-self.ztilt_zero,self.psi ,self.accz, self.sys_cal, self.gyro_cal, self.acc_cal, self.mag_cal])
			#print([self.psi,self.accx-self.accx_zero,self.accy-self.accy_zero,self.accz-self.accz_zero])
			print([round(self.ytilt,2),round(self.ztilt,2)])


	def GetIMUOffsets(self):

		print(self.roll,self.roll-self.ytilt_zero, self.pitch,self.pitch-self.ztilt_zero)
		getimuoffsets_var=input("Enter 1 to get IMU Offsets. Enter 0 to use stored offsets: ")
		if int(getimuoffsets_var)==1:
			input("Adjust peg so it is in the hole")
			time.sleep(1)
			self.get_data(0)
			self.buttjpin=0
			print(self.buttjpin)
			while self.buttjpin==0:
				self.get_data(0)
				self.getJoystickMotorSpeed(1)
				print(self.mot1spd,self.mot2spd,self.mot3spd)
				self.CmdMotors()

			
			input("Let the peg rest so vals can be obtained: ")
			tc=time.time()
			accxlist=[]
			accylist=[]
			acczlist=[]
			ytiltlist=[]
			ztiltlist=[]
			rolllist=[]
			pitchlist=[]
			while time.time()-tc<5:
				self.ReadSerial(0)
				accxlist.append(self.accx)
				accylist.append(self.accy)
				acczlist.append(self.accz)
				ytiltlist.append(self.ytilt)
				ztiltlist.append(self.ztilt)
				rolllist.append(self.roll)
				pitchlist.append(self.pitch)

			self.accx_zero=self.avg(accxlist)
			self.accy_zero=self.avg(accylist)
			self.accz_zero=self.avg(acczlist)
			# self.ytilt_zero=self.avg(rolllist)
			# self.ztilt_zero=self.avg(pitchlist)
			self.ytilt_zero=self.avg(ytiltlist)
			self.ztilt_zero=self.avg(ztiltlist)
			print("done!")
			print(self.ytilt_zero,self.ztilt_zero,self.accx_zero,self.accy_zero,self.accz_zero)
			input("Write down ytilt_zero,ztilt_zero and accx_zero,accy_zero,accz_zero in the code for future use!!")



		# self.get_data(0)
		# time.sleep(2)
		# input("get IMU Data. Hit joystick button to stop")
		# self.get_data(0)
		# a.buttjpin=0
		# while a.buttjpin==0:
		# 	self.get_data(0)
		# 	pitch = 180 * np.arctan2(self.accx ,np.sqrt(self.accy*self.accy+ self.accz*self.accz))/3.14;
		# 	roll = 180 * np.arctan2(self.accy, np.sqrt(self.accx*self.accx + self.accz*self.accz))/3.14;
	
		# 	#print([self.ytilt,self.ytilt-self.ytilt_zero, self.ztilt,self.ztilt-self.ztilt_zero,self.psi ,self.accz, self.sys_cal, self.gyro_cal, self.acc_cal, self.mag_cal])
		# 	#print([self.psi,self.accx-self.accx_zero,self.accy-self.accy_zero,self.accz-self.accz_zero])
		# 	print([self.ytilt-self.ytilt_zero,self.roll-self.ytilt_zero, self.ztilt-self.ztilt_zero,self.pitch-self.ztilt_zero])

	def IMUData(self):
		self.get_data(0)
		time.sleep(2)
		input("get IMU Data. Hit joystick button to stop")
		self.get_data(0)
		a.buttjpin=0
		while a.buttjpin==0:
			self.get_data(0)
			pitch = 180 * np.arctan2(self.accx ,np.sqrt(self.accy*self.accy+ self.accz*self.accz))/3.14;
			roll = 180 * np.arctan2(self.accy, np.sqrt(self.accx*self.accx + self.accz*self.accz))/3.14;
	
			#print([self.ytilt,self.ytilt-self.ytilt_zero, self.ztilt,self.ztilt-self.ztilt_zero,self.psi ,self.accz, self.sys_cal, self.gyro_cal, self.acc_cal, self.mag_cal])
			#print([self.psi,self.accx-self.accx_zero,self.accy-self.accy_zero,self.accz-self.accz_zero])
			print([self.ytilt-self.ytilt_zero,self.roll-self.ytilt_zero, self.ztilt-self.ztilt_zero,self.pitch-self.ztilt_zero])

	def IMUData2(self):
		self.get_data(0)
		time.sleep(2)
		input("get IMU Data. Hit joystick button to stop")
		self.get_data(0)
		a.buttjpin=0
		while a.buttjpin==0:
			self.get_data(0)
			pitch = 180 * np.arctan2(self.accx ,np.sqrt(self.accy*self.accy+ self.accz*self.accz))/3.14;
			roll = 180 * np.arctan2(self.accy, np.sqrt(self.accx*self.accx + self.accz*self.accz))/3.14;
	
			#print([self.ytilt,self.ytilt-self.ytilt_zero, self.ztilt,self.ztilt-self.ztilt_zero,self.psi ,self.accz, self.sys_cal, self.gyro_cal, self.acc_cal, self.mag_cal])
			#print([self.psi,self.accx-self.accx_zero,self.accy-self.accy_zero,self.accz-self.accz_zero])
			print([round(self.ytilt,2),round(self.ztilt,2)])


	##### CAMERA AND TAGS
	def DisplayCameraTest_noTAGS(self):
		# pdb.set_trace()
		#test capture to make sure both tags are detected
		input('camera test 1 will display. This is a test')
		while True:
						
			self.ret1, self.frame1 = self.cap1.read()
			# operations on the frame come here
			gray = cv2.cvtColor(self.frame1, cv2.COLOR_BGR2GRAY)	# Change grayscale
			

			cv2.imshow('frame', self.frame1)
			
			# Wait 3 milisecoonds for an interaction. Check the key and do the corresponding job.
			key = cv2.waitKey(3) & 0xFF
			if key == ord('q'):	# Quit
					print('quit!')
					break

		cv2.destroyAllWindows()
		userinput=input('If you are happy with the capture, enter 1 to continue otherwise restart')

	#def DisplayCameraTest_singleARUCO(self):
	# 	# pdb.set_trace()
	# 	#test capture to make sure both tags are detected
	# 	input('camera test 1 will display. This is a test')
		

						
							
	# 	self.ret1, self.frame1 = self.cap1.read()
		

	# 	# operations on the frame come here
	# 	gray = cv2.cvtColor(self.frame1, cv2.COLOR_BGR2GRAY)	# Change grayscale
		
	# 	cv2.imshow('frame', self.frame1)
	# 	self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)	# Use 5x5 dictionary to find markers
	# 	self.parameters = aruco.DetectorParameters_create()	# Marker detection parameters
	# 	# lists of ids and the corners beloning to each id
	# 	self.corners, self.ids, self.rejected_img_points = aruco.detectMarkers(gray, self.aruco_dict,parameters=self.parameters,cameraMatrix=self.matrix_coefficients,distCoeff=self.distortion_coefficients)
	# 	if np.all(self.ids is not None):	# If there are markers found by detector


	# 			self.zipped = zip(self.ids, self.corners)
	# 			self.ids, self.corners = zip(*(sorted(self.zipped)))

	# 			for i in range(0, len(self.ids)):	# Iterate in markers
	# 					# Estimate pose of each marker and return the values rvec and tvec---different from camera coefficients
	# 					self.rvec, self.tvec, self.markerPoints = aruco.estimatePoseSingleMarkers(self.corners[i], self.markersize, self.matrix_coefficients, self.distortion_coefficients)
	# 					if self.ids[i] == self.firstMarkerID:
	# 							self.firstRvec = self.rvec
	# 							self.firstTvec = self.tvec
	# 							self.isFirstMarkerCalibrated = True
	# 							self.firstMarkerCorners = self.corners[i]

	# 					elif self.ids[i] == self.secondMarkerID:
								
	# 							self.secondRvec = self.rvec
	# 							self.secondTvec = self.tvec
	# 							self.isSecondMarkerCalibrated = True
	# 							self.secondMarkerCorners = self.corners[i]

		

	# 					(self.rvec - self.tvec).any()	# get rid of that nasty numpy value array error
	# 					aruco.drawDetectedMarkers(self.frame1, self.corners)	# Draw A square around the markers
	# 					aruco.drawAxis(self.frame1, self.matrix_coefficients, self.distortion_coefficients, self.rvec, self.tvec, 0.01)	# Draw Axis
	# 					cv2.imshow('frame', self.frame1)
						
	# 			try:
					
	# 				composedRvec, composedTvec= self.relativePosition(self.firstRvec, self.firstTvec, self.secondRvec, self.secondTvec)
					
	# 				composedTvecList=composedTvec.reshape(1,3)[0].tolist()
	# 				tveclist = [round(element * 1000,1) for element in composedTvecList]
	# 				#print(tveclist)

	# 				composedRvecList=composedRvec.reshape(1,3)[0].tolist()
	# 				Rveclist = [round(element * 1000,1) for element in composedRvecList]
	# 				print(tveclist)


	# 			except:
	# 				pass
						
	# 				# Wait 3 milisecoonds for an interaction. Check the key and do the corresponding job.

	# 		#cv2.imshow('frame', self.frame)

	
						
					# Wait 3 milisecoonds for an interaction. Check the key and do the corresponding job.

			#cv2.imshow('frame', self.frame)
	#def DisplayCameraTestARUCO(self):
		# pdb.set_trace()
		#test capture to make sure both tags are detected
		# input('camera test 1 will display. This is a test')
		# while True:

						
								
		# 	self.ret1, self.frame1 = self.cap1.read()
			

		# 	# operations on the frame come here
		# 	gray = cv2.cvtColor(self.frame1, cv2.COLOR_BGR2GRAY)	# Change grayscale
			
		# 	cv2.imshow('frame', self.frame1)
		# 	self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)	# Use 5x5 dictionary to find markers
		# 	self.parameters = aruco.DetectorParameters_create()	# Marker detection parameters
		# 	# lists of ids and the corners beloning to each id
		# 	self.corners, self.ids, self.rejected_img_points = aruco.detectMarkers(gray, self.aruco_dict,parameters=self.parameters,cameraMatrix=self.matrix_coefficients,distCoeff=self.distortion_coefficients)
		# 	if np.all(self.ids is not None):	# If there are markers found by detector


		# 			self.zipped = zip(self.ids, self.corners)
		# 			self.ids, self.corners = zip(*(sorted(self.zipped)))

		# 			for i in range(0, len(self.ids)):	# Iterate in markers
		# 					# Estimate pose of each marker and return the values rvec and tvec---different from camera coefficients
		# 					self.rvec, self.tvec, self.markerPoints = aruco.estimatePoseSingleMarkers(self.corners[i], self.markersize, self.matrix_coefficients, self.distortion_coefficients)
		# 					if self.ids[i] == self.firstMarkerID:
		# 							self.firstRvec = self.rvec
		# 							self.firstTvec = self.tvec
		# 							self.isFirstMarkerCalibrated = True
		# 							self.firstMarkerCorners = self.corners[i]

		# 					elif self.ids[i] == self.secondMarkerID:
									
		# 							self.secondRvec = self.rvec
		# 							self.secondTvec = self.tvec
		# 							self.isSecondMarkerCalibrated = True
		# 							self.secondMarkerCorners = self.corners[i]

			

		# 					(self.rvec - self.tvec).any()	# get rid of that nasty numpy value array error
		# 					aruco.drawDetectedMarkers(self.frame1, self.corners)	# Draw A square around the markers
		# 					aruco.drawAxis(self.frame1, self.matrix_coefficients, self.distortion_coefficients, self.rvec, self.tvec, 0.01)	# Draw Axis
		# 					cv2.imshow('frame', self.frame1)
							
		# 			try:
						
		# 				composedRvec, composedTvec= self.relativePosition(self.firstRvec, self.firstTvec, self.secondRvec, self.secondTvec)
						
		# 				composedTvecList=composedTvec.reshape(1,3)[0].tolist()
		# 				tveclist = [round(element * 1000,1) for element in composedTvecList]
		# 				#print(tveclist)

		# 				composedRvecList=composedRvec.reshape(1,3)[0].tolist()
		# 				Rveclist = [round(element * 1000,1) for element in composedRvecList]
		# 				print(tveclist)


		# 			except:
		# 				pass
							
		# 			# Wait 3 milisecoonds for an interaction. Check the key and do the corresponding job.
		# 	key = cv2.waitKey(3) & 0xFF
		# 	if key == ord('q'):	# Quit
		# 			print('quit!')
		# 			break
		# 	#cv2.imshow('frame', self.frame)
									

		# cv2.destroyAllWindows()
		# userinput=input('If you are happy with the capture, enter 1 to continue otherwise restart')

# 	def DisplayCameraTestApril(self):
# #		 pdb.set_trace()
# 		#test capture to make sure both tags are detected
# 		input('camera test will display. This is a test')
# 		while True:

						
# 				self.ret1, self.frame1 = self.cap1.read()
# 				# operations on the frame come here
# 				gray = cv2.cvtColor(self.frame1, cv2.COLOR_BGR2GRAY)	# Change grayscale
# 				detections,dimg = self.detector.detect(gray,return_image=True)
# 				overlay = self.frame1 // 2 + dimg[:, :, None] // 2

# 				for detection in detections:
# 					#pdb.set_trace()
# 					#print(detection.tag_id)
# 					if detection.tag_id==0:
# 					    print("peg")
# 					if detection.tag_id==2:
# 					    print("hole")

# 					center1 = detection.center
# 					#print("center: ")
# 					#print(center1)
# 					corners1 = detection.corners
# 					#print("corners: ")
# 					#print(corners1)
# 					self.draw_angled_rec2(corners1,overlay)
# 					angle = self.cornerAngle(corners1,center1,overlay)
# 					#print(180*angle/(2*np.pi))
# 					groundpt = np.add(np.matrix([center1]).T, np.dot(self.rotz2D(angle), np.matrix([[-120],[0]])))
# 					#print(groundpt)
# 					cv2.line(overlay,tuple([int(a) for a in center1]),tuple([int(a) for a in groundpt]),(0,0,200),2)
# 						#cv2.imshow("frame",self.frame1)

# 				#except:
# 				#	pass
# 				cv2.imshow(self.window,overlay)
# 				key = cv2.waitKey(1) & 0xFF
# 				if key == ord('q'):	# Quit
# 						print('quit!')
# 						break


								

# 		cv2.destroyAllWindows()





	def DisplayCameraTestApril(self):
#		 pdb.set_trace()
		#test capture to make sure both tags are detected
		input('camera test will display. This is a test')
		while True:

						
				self.ret1, self.frame1 = self.cap1.read()
				# operations on the frame come here
				gray = cv2.cvtColor(self.frame1, cv2.COLOR_BGR2GRAY)	# Change grayscale
				detections,dimg = self.detector.detect(gray,return_image=True)
				overlay = self.frame1 // 2 + dimg[:, :, None] // 2

				for detection in detections:
					#pdb.set_trace()
					#print(detection.tag_id)
					if detection.tag_id==0:
					    print("peg")
					if detection.tag_id==2:
					    print("hole")

					center1 = detection.center
					#print("center: ")
					#print(center1)
					corners1 = detection.corners
					#print("corners: ")
					#print(corners1)
					self.draw_angled_rec2(corners1,self.frame1)
					angle = self.cornerAngle(corners1,center1,self.frame1)
					#print(180*angle/(2*np.pi))
					groundpt = np.add(np.matrix([center1]).T, np.dot(self.rotz2D(angle), np.matrix([[-120],[0]])))
					#print(groundpt)
					cv2.line(self.frame1,tuple([int(a) for a in center1]),tuple([int(a) for a in groundpt]),(0,0,200),2)
						#cv2.imshow("frame",self.frame1)

				#except:
				#	pass
				cv2.imshow("frame",self.frame1)
				key = cv2.waitKey(1) & 0xFF
				if key == ord('q'):	# Quit
						print('quit!')
						break
		cv2.destroyAllWindows()
		userinput=input('If you are happy with the capture, enter 1 to continue otherwise restart')




	def ReadDisplayCVApril(self, tosaveflag):
		x,y,z,rx,ry,rz=(0,0,0,0,0,0) 			
		
		self.ret1, self.frame1 = self.cap1.read()
		if tosaveflag==1:
			if self.ret1 == True: 
					self.out1.write(self.frame1)
		#self.out1.write(self.frame1)
		# operations on the frame come here
		gray = cv2.cvtColor(self.frame1, cv2.COLOR_BGR2GRAY)	# Change grayscale		
		# cv2.imshow(self.window, self.frame1)
		detections,dimg = self.detector.detect(gray,return_image=True)	
		overlay = self.frame1 // 2 + dimg[:, :, None] // 2

				
		for detection in detections:
			pose_mtx, init_error, final_error =self.detector.detection_pose(detection, camera_params=self.camera_params, tag_size=self.tag_size, z_sign=1)
			x = pose_mtx[0][3]
			y = pose_mtx[1][3]
			z = pose_mtx[2][3]
			rx=pose_mtx[0][0]*180/3.14
			ry=pose_mtx[1][1]*180/3.14
			rz=pose_mtx[2][2]*180/3.14
			if detection.tag_id==0:
				self.pegtvec=[x,y,z]
				self.pegrvec=[rx,ry,rz]
			if detection.tag_id==2:
				self.holetvec=[x,y,z]
				self.holervec=[rx,ry,rz]

			center1 = detection.center
			#print("center: ")
			#print(center1)
			corners1 = detection.corners
			#print("corners: ")
			#print(corners1)
			self.draw_angled_rec2(corners1,self.frame1)
			angle = self.cornerAngle(corners1,center1,self.frame1)
			#print(180*angle/(2*np.pi))
			groundpt = np.add(np.matrix([center1]).T, np.dot(self.rotz2D(angle), np.matrix([[-120],[0]])))
			#print(groundpt)
			cv2.line(self.frame1,tuple([int(a) for a in center1]),tuple([int(a) for a in groundpt]),(0,0,200),2)
			#cv2.line(overlay,tuple([int(a) for a in center1]),tuple([int(a) for a in groundpt]),(0,0,200),2)
				#cv2.imshow("frame",self.frame1)

		cv2.imshow("frame",self.frame1)
		#cv2.imshow(self.window,overlay)
		if tosaveflag==1:
			self.out2.write(self.frame1)
		key = cv2.waitKey(3) & 0xFF
		if key == ord('q'):	# Quit
				print('quit!')
	
	#def ReadDisplayCVAruco(self, tosaveflag):
						
		# self.pegrvec=[0,0,0]
		# self.pegtvec=[0,0,0]
		# self.holervec=[0,0,0]
		# self.holetvec=[0,0,0]
		# self.diffrvec=[0,0,0]
		# self.difftvec=[0,0,0]
		# self.depth=0

		# ctr=0
		# while ctr<2:
		# 	ctr=ctr+1
		# self.ret1, self.frame1 = self.cap1.read()
		# if tosaveflag==1:
		# 	if self.ret1 == True: 
		# 			self.out1.write(self.frame1)
		# #self.out1.write(self.frame1)
		# # operations on the frame come here
		# gray = cv2.cvtColor(self.frame1, cv2.COLOR_BGR2GRAY)	# Change grayscale
		
		# cv2.imshow('frame', self.frame1)
		# self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)	# Use 5x5 dictionary to find markers
		# self.parameters = aruco.DetectorParameters_create()	# Marker detection parameters
		# # lists of ids and the corners beloning to each id
		# self.corners, self.ids, self.rejected_img_points = aruco.detectMarkers(gray, self.aruco_dict,parameters=self.parameters,cameraMatrix=self.matrix_coefficients,distCoeff=self.distortion_coefficients)
		# if np.all(self.ids is not None):	# If there are markers found by detector

		# 		self.zipped = zip(self.ids, self.corners)
		# 		self.ids, self.corners = zip(*(sorted(self.zipped)))

		# 		for i in range(0, len(self.ids)):	# Iterate in markers
		# 				# Estimate pose of each marker and return the values rvec and tvec---different from camera coefficients
		# 				self.rvec, self.tvec, self.markerPoints = aruco.estimatePoseSingleMarkers(self.corners[i], self.markersize, self.matrix_coefficients, self.distortion_coefficients)
		# 				if self.ids[i] == self.firstMarkerID:
		# 						self.firstRvec = self.rvec
		# 						self.firstTvec = self.tvec
		# 						self.isFirstMarkerCalibrated = True
		# 						self.firstMarkerCorners = self.corners[i]
		# 						self.pegrvec=self.rvec[0][0].tolist()
		# 						self.pegtvec=self.tvec[0][0].tolist()

		# 				elif self.ids[i] == self.secondMarkerID:
								
		# 						self.secondRvec = self.rvec
		# 						self.secondTvec = self.tvec
		# 						self.isSecondMarkerCalibrated = True
		# 						self.secondMarkerCorners = self.corners[i]
		# 						self.holervec=self.rvec[0][0].tolist()
		# 						self.holetvec=self.tvec[0][0].tolist()

		

		# 				(self.rvec - self.tvec).any()	# get rid of that nasty numpy value array error
		# 				aruco.drawDetectedMarkers(self.frame1, self.corners)	# Draw A square around the markers
		# 				aruco.drawAxis(self.frame1, self.matrix_coefficients, self.distortion_coefficients, self.rvec, self.tvec, 0.01)	# Draw Axis
				
		# 				cv2.imshow('frame', self.frame1)


		# 		try:		
		# 			composedRvec, composedTvec= self.relativePosition(self.firstRvec, self.firstTvec, self.secondRvec, self.secondTvec)
					
		# 			composedTvecList=composedTvec.reshape(1,3)[0].tolist()
		# 			self.difftvec = [round(element * 1000,1) for element in composedTvecList]
		# 			self.depth=round(self.difftvec[2],1)
		# 			#print(tveclist)

		# 			composedRvecList=composedRvec.reshape(1,3)[0].tolist()
		# 			self.diffrvec = [round(element * 1000,1) for element in composedRvecList]
		# 		except:
		# 			pass
					


		# cv2.imshow('frame', self.frame1)
		# if tosaveflag==1:
		# 	self.out2.write(self.frame1)
		# key = cv2.waitKey(3) & 0xFF
		# if key == ord('q'):	# Quit
		# 		print('quit!')
				
				
	
									



	def getPegDepth_zeros(self):
		print('Get Depth Zeros')
		getdepth_var=input("Enter 1 to get Peg Depth. Enter 0 to use stored depth: ")
		if int(getdepth_var)==1:
			input("Adjust peg so it is resting on TOP of hole")
			time.sleep(1)
			self.get_data(0)
			self.buttjpin=0
			print(self.buttjpin)
			while self.buttjpin==0:
				self.get_data(0)
				self.getJoystickMotorSpeed(1)
				print(self.depth)
				self.CmdMotors()

			
			input("Let the peg rest so vals can be obtained: ")
			tc=time.time()
			depthlist1=[]
			depthlist2=[]

			while time.time()-tc<5:
				self.get_data(0)

				
				depthlist1.append((self.pegtvec[1]-self.holetvec[1])*1000)

			self.depth_zero1=round(self.avg(depthlist1),1)
			
			print("done!")
			print(self.depth_zero1)
			input("Write down depth_zero for future use!")




			if int(getdepth_var)==1:
				input("Adjust peg so it is resting in the BOTTOM of the hole")
				time.sleep(1)
				self.get_data(0)
				self.buttjpin=0
				print(self.buttjpin)
				while self.buttjpin==0:
					self.get_data(0)
					self.getJoystickMotorSpeed(1)
					print(self.depth)
					self.CmdMotors()

				
				input("Let the peg rest so vals can be obtained: ")
				tc=time.time()
				depthlist1=[]
				depthlist2=[]
				accxlist=[]
				accylist=[]
				acczlist=[]
				ytiltlist=[]
				ztiltlist=[]
				rolllist=[]
				pitchlist=[]

				while time.time()-tc<5:
					self.get_data(0)
					
					depthlist1.append((self.pegtvec[1]-self.holetvec[1])*1000)
					accxlist.append(self.accx)
					accylist.append(self.accy)
					acczlist.append(self.accz)
					ytiltlist.append(self.ytilt)
					ztiltlist.append(self.ztilt)
					rolllist.append(self.roll)
					pitchlist.append(self.pitch)

				self.depth_max1=round(self.avg(depthlist1),1)			
				self.accx_zero=self.avg(accxlist)
				self.accy_zero=self.avg(accylist)
				self.accz_zero=self.avg(acczlist)
				self.ytilt_zero=self.avg(ytiltlist)
				self.ztilt_zero=self.avg(ztiltlist)
				
				print("done!")

				print(self.depth_max1 ,self.ytilt_zero,self.ztilt_zero,self.accx_zero,self.accy_zero,self.accz_zero)
				input("Write down max depth, ytilt_zero,ztilt_zero and accx_zero,accy_zero,accz_zero in the code for future use!!")






	def getPegDepth(self):
		#map(self,x,in_min,in_max,out_min,out_max)
		#rint(self.pegtvec[2],self.holetvec[2])
		d1=(self.pegtvec[1]-self.holetvec[1])*1000
		#print(d1,self.depth_zero1,self.depth_max1,self.depth_zero_meas,self.depth_max_meas)
		self.depth_1=self.map(d1,self.depth_zero1,self.depth_max1,self.depth_zero_meas,self.depth_max_meas)
		





	def getCurrentAvgPegDepth(self):
		tc=time.time()
		depthlist1=[]
		depthlist2=[]
		psilist=[]

		while time.time()-tc<5:
			self.get_data(1)

			depthlist2.append(self.difftvec[1])
			depthlist1.append((self.holetvec[1]-self.pegtvec[1])*1000)
			psilist.append(self.psi)

		self.depth_1_curavg=round(self.avg(depthlist1),1)
		self.depth_2_curavg=round(self.avg(depthlist2),1)
		self.psiwedge=round(self.avg(psilist),1)

		print(self.depth_1_curavg,self.depth_2_curavg,self.psiwedge)


	def draw_angled_rec(self,x0, y0, width, height, angle, img):
		#pdb.set_trace()
		_angle = angle * math.pi / 180.0
		b = math.cos(_angle) * 0.5
		a = math.sin(_angle) * 0.5
		pt0 = (int(x0 - a * height - b * width),
		       int(y0 + b * height - a * width))
		pt1 = (int(x0 + a * height - b * width),
		       int(y0 - b * height - a * width))
		pt2 = (int(2 * x0 - pt0[0]), int(2 * y0 - pt0[1]))
		pt3 = (int(2 * x0 - pt1[0]), int(2 * y0 - pt1[1]))

		cv2.line(img, pt0, pt1, (255, 255, 255), 3)
		cv2.line(img, pt1, pt2, (255, 255, 255), 3)
		cv2.line(img, pt2, pt3, (255, 255, 255), 3)
		cv2.line(img, pt3, pt0, (255, 255, 255), 3)

	def draw_angled_rec2(self,pts, img):
		#pdb.set_trace()
		for i in range(0,len(pts)):
			if i+1 == len(pts):
				cv2.line(img, tuple([int(a) for a in pts[i]]), tuple([int(a) for a in pts[0]]), (0, 200, 0), 2)
			else:
				cv2.line(img, tuple([int(a) for a in pts[i]]), tuple([int(a) for a in pts[i+1]]), (0, 200, 0), 2)


	def cornerAngle(self,pts,centert, img):
		try:

			ang = 0
			angtemp2=[]
			for i in range(0,len(pts)):
				#pdb.set_trace()
				if i+1 == len(pts):
					angTemp = +i*np.pi/2 + np.arctan2(pts[i][1]-pts[0][1],pts[i][0]-pts[0][0])
					ang+=angTemp
					angtemp2.append(angTemp)
					groundpt = np.add(np.matrix([centert]).T, np.dot(self.rotz2D(angTemp), np.matrix([[0],[50]])))
					#cv2.line(img,tuple([int(a) for a in centert]),tuple([int(a) for a in groundpt]),(200,0,0),2)
				else:
					angTemp = +i*np.pi/2 + np.arctan2(pts[i][1]-pts[i+1][1],pts[i][0]-pts[i+1][0])
					ang+=angTemp
					angtemp2.append(angTemp)
					groundpt = np.add(np.matrix([centert]).T, np.dot(self.rotz2D(angTemp), np.matrix([[0],[50]])))
					cv2.line(img,tuple([int(a) for a in centert]),tuple([int(a) for a in groundpt]),(200,0,0),2)
			#print(angtemp2)
			return ang/4.0
		except:
			pass

	def rotz(self,theta):
	    return np.matrix([[np.cos(theta), -np.sin(theta), 0], [np.sin(theta),  np.cos(theta), 0], [0, 0, 1]])

	def rotz2D(self,theta):
		return np.matrix([[np.cos(theta), -np.sin(theta)], [np.sin(theta),  np.cos(theta)]])



	### SAVING AND GET DATA

	def DataToSave(self):
		#self.TotalList.append([self.ytilt,self.ztilt,self.str1,self.str2,self.str3])
		self.TotalList.append(self.list_of_floats)


	def writevideo(self):
		self.ret1, self.frame1 = self.cap1.read()
		self.out1.write(self.frame1)

		# self.ret2, self.frame2 = self.cap2.read()
		# self.out2.write(self.frame2)


	def delaywithvideo(self,timedelay):
		tc=time.time()
		while time.time()-tc<timedelay:
			#self.writevideo()
			self.get_data(1)


	def writefile(self):
		with open(self.filename, "w") as f:
				writer = csv.writer(f)
				writer.writerows(self.TotalList)
				print("saved")

	def finishtestrecording(self):
		self.StopPeg()
		#a.writevideo()
		self.writefile()
		self.cap1.release()
		self.out1.release()
		self.out2.release()

		

	def get_data(self,tosaveflag):
		self.ReadDisplayCVApril(tosaveflag)
		self.ReadSerial(tosaveflag)




	### OTHER FUNCS

	def calculatepsi(self):
		# self.yaw=0;

		# w=np.array(((0),(0),(1)))
		# v=np.array(((0),(1),(0)))
		# u=np.array(((1),(0),(0)))
		# cy, sy = np.cos(self.yaw), np.sin(self.yaw)
		# self.roll=(self.ytilt-self.ytilt_zero)*3.14/180
		# self.pitch=(self.ztilt-self.ztilt_zero)*3.14/180
		# cp, sp = np.cos(self.pitch), np.sin(self.pitch)
		# cr, sr = np.cos(self.roll), np.sin(self.roll)
		# Ry = np.array(((cy, -sy ,0), (sy, cy,0),(0,0,1)))
		# Rp = np.array(((cp, 0 ,sp), (0,1,0),(-sp,0,cp)))
		# Rr = np.array(((1,0,0), (0 ,cr,-sr),(0,sr,cr)))
		# Ryp=np.matmul(Ry,Rp)
		# Rypr=np.matmul(Ryp,Rr)

		# wprime=np.matmul(Rypr,w)

		# wprog=(u*np.dot(u,wprime)) + (v*np.dot(v,wprime))
		# top=np.dot(wprime,u)
		# bot=np.linalg.norm(wprog)*np.linalg.norm(u)
		# #self.psi=round(np.arccos(top/bot)*180/3.14,2)
		#self.psi=round(np.arctan2((self.accy-self.accy_zero),(self.accx-self.accx_zero))*180/3.14,2)
		self.psi=180-abs(round(np.arctan2((self.accx-self.accx_zero),(self.accy-self.accy_zero))*180/3.14,2))


	def avg(self,lst): 
	    return sum(lst) / len(lst) 

	def map(self,x,in_min,in_max,out_min,out_max):

		v1=x-in_min
		v2=out_max - out_min
		v3=in_max - in_min

		v4=(v1*v2)/v3 + out_min
		return v4

	

	### WEDGING FUNCS


	def checkWedging(self):
		print("Checking Wedging")
		str1list=[]
		str2list=[]
		str3list=[]
		str1avg=0
		str2avg=0
		str3avg=0
		psilist=[]
		for i in range(0,10):
			self.get_data(1)
			#self.writevideo()
			str1list.append(self.str1)
			str2list.append(self.str2)
			str3list.append(self.str3)
			psilist.append(self.psi)
		str1avg=self.avg(str1list)
		str2avg=self.avg(str2list)
		str3avg=self.avg(str3list)
		self.psiwedge=self.avg(psilist)
		if str1avg<self.str1thresh and str2avg<self.str2thresh and str3avg<self.str3thresh:
			wedgectrflag=1
			print('wedged!')
		else:
			wedgectrflag=0
			print("not wedged")
			time.sleep(2)
			

		return wedgectrflag

	
	def breakWedge(self):
		print("break wedge")
		#pdb.set_trace()
		self.getCurrentAvgPegDepth()
		print(self.depth_1_curavg,self.psiwedge)
		#self.writevideo()
		self.buttjpin=0
		print(self.accz,self.accz_zero,self.accz_thresh_wedgeBreaking)
		depthctr=0
		#pdb.set_trace()
		try:
			while abs(self.accz-self.accz_zero)<self.accz_thresh_wedgeBreaking and self.buttjpin==0 and depthctr<5:

				self.get_data(1)
				


				# if (self.depth_1 -self.depth_1_curavg) <=-25:
				# 	depthctr=depthctr+1

				print(abs(self.accz-self.accz_zero),self.depth_1 ,self.depth_1_curavg,self.depth_2 ,self.depth_2_curavg)

				self.RaisePeg(0)  #SET WEDGE BREAKING TENSION HERE


			self.StopPeg()
			print('peg raised!')
		except:
			self.StopPeg()
			print("break wedge error!")

	def rotateYtilt(self):
		if self.ytilt-self.ytilt_zero<0:
			self.mot1spd=-2500
			self.mot2spd=0
			self.mot3spd=0
			self.CmdMotors()
		if self.ytilt-self.ytilt_zero>0:
			self.mot1spd=0
			self.mot2spd=-2500
			self.mot3spd=0
			self.CmdMotors()

	def rotateZtilt(self):
		if self.ztilt-self.ztilt_zero<0:
			self.mot1spd=-2500
			self.mot2spd=-2500
			self.mot3spd=0
			self.CmdMotors()

		if self.ztilt-self.ztilt_zero>0:
			self.mot1spd=0
			self.mot2spd=0
			self.mot3spd=-2500
			self.CmdMotors()

	def correctYtilt(self):
		try:
			# pitch = 180 * atan2(accelX, sqrt(accelY*accelY + accelZ*accelZ))/PI;
			# roll = 180 * atan2(accelY, sqrt(accelX*accelX + accelZ*accelZ))/PI;
	
			#print(self.roll,self.roll-self.ytilt_zero)
			print(self.ytilt,self.ytilt-self.ytilt_zero)
			#self.writevideo()
			
			ctr=0
			self.exitholeflag=0
			#vprev=self.roll-self.ytilt_zero
			vprev=self.ytilt-self.ytilt_zero
			self.get_data(1)
			print('read serial')
			
			while (ctr<5) and self.buttjpin==0 and self.exitholeflag==0:
				#self.writevideo()
				self.get_data(1)
				self.rotateYtilt()
				#print(round(self.roll-self.ytilt_zero,3),ctr,self.mot1spd,self.mot2spd,self.mot3spd)
				print(round(self.ytilt-self.ytilt_zero,3),ctr,self.mot1spd,self.mot2spd,self.mot3spd)
				#v=self.roll-self.ytilt_zero
				v=self.ytilt-self.ytilt_zero
				# if abs(v)>abs(vprev):
				# 	self.exitholeflag=1
				# 	print('exitholeflag')
				# else:
				# 	vprev=v

				if abs(self.ytilt-self.ytilt_zero)<self.angle_Ythresh:
					ctr=ctr+1

			self.StopPeg()
		
		except:
			print("Correct Ytilt Error")
			self.StopPeg()

	def correctZtilt(self):
		try:
			print(self.ztilt,self.ztilt-self.ztilt_zero)
			#print(self.pitch,self.pitch-self.ztilt_zero)
			#self.writevideo()
			print('vid')
			ctr=0
			self.get_data(1)
			#vprev=self.pitch-self.ztilt_zero
			vprev=self.ztilt-self.ztilt_zero
			print('read serial')
			while (ctr<5) and self.buttjpin==0 and self.exitholeflag==0:
				#self.writevideo()
				self.get_data(1)
				self.rotateZtilt()
				#print(round(self.pitch-self.ztilt_zero,3), ctr,self.mot1spd,self.mot2spd,self.mot3spd)
				print(round(self.ztilt-self.ztilt_zero,3), ctr,self.mot1spd,self.mot2spd,self.mot3spd)
				#v=self.pitch-self.ztilt_zero
				v=self.ztilt-self.ztilt_zero
				# if abs(v)>abs(vprev) :
				# 	self.exitholeflag=1
				# 	print('exitholeflag')
				# else:
				# 	vprev=v
				if abs(self.ztilt-self.ztilt_zero)<self.angle_Zthresh:
					ctr=ctr+1

			self.StopPeg()
		
		except:
			print("Correct Ztilt Error")
			self.StopPeg()


	def correctWedge(self):
		try:
			print(self.ytilt)
			#self.writevideo()
			self.get_data(1)
			print('correct z tilt')
			self.correctYZilt()
			self.StopPeg()
			print('ztilt corrected')
			self.delaywithvideo(2)
			print('correct y tilt')
			self.correctYtilt()
			self.StopPeg()
			print('ytilt corrected')
			self.delaywithvideo(2)
			
		
		except:
			print("Correct Wedge Error")
			self.StopPeg()
	

	def getWedgedAngle(self):
		print("Checking Wedging")
		ytiltlist=[]
		ztiltlist=[]
		ytiltavg=0
		ztiltavg=0
		for i in range(0,25):
			self.get_data(1)
			ytiltlist.append(self.ytilt)
			ztiltlist.append(self.ztilt)
			
		self.ytiltw=self.avg(ytiltlist)
		self.ztiltw=self.avg(ztiltlist)


	def correctWedge_Tens(self):
		self.get_data(1)
		strlist=[self.str1,self.str2,self.str3]
		print(strlist)

		maxpos = strlist.index(max(strlist))
		#print(maxpos)

		err1=strlist[maxpos]-strlist[0]
		err2=strlist[maxpos]-strlist[1]
		err3=strlist[maxpos]-strlist[2]
		print(err1, err2, err3)

		while err1>5 or err2>5 or err3>5:
			self.get_data(1)
			strlist=[self.str1,self.str2,self.str3]
			

			maxpos = strlist.index(max(strlist))
			#print(maxpos)

			err1=strlist[maxpos]-strlist[0]
			err2=strlist[maxpos]-strlist[1]
			err3=strlist[maxpos]-strlist[2]


		#print(err1,err2,err3)
			# if err1 >0:
		
			# 	self.mot1spd=-10000
			# if err2>0:

			# 	self.mot2spd=-10000
			# if err3>0:
			# 	self.mot3spd=-10000

			if err1==0:
				self.mot1spd=0
			else:
				self.mot1spd=-int(round(self.map(err1,0,50,0,10000)))

			if err2==0:
				self.mot2spd=0
			else:
				self.mot2spd=-int(round(self.map(err2,0,50,0,10000)))

			if err3==0:
				self.mot3spd=0
			else:
				self.mot3spd=-int(round(self.map(err3,0,50,0,10000)))

		#print(self.mot1spd,self.mot2spd,self.mot3spd)

			print(self.str1,self.str2,self.str3,self.mot1spd,self.mot2spd,self.mot3spd)
			self.CmdMotors()





	

# CONTROLLERS-----------------------------------------------------------------
	def SimpleAdjust_noRecord(self):
		input("Simple Adjust, No Record: ")
		self.buttjpin=0
		self.get_data(0)
		print(self.buttjpin)
		print('loop')
		while self.buttjpin==0:
			self.get_data(0)
			
			self.getJoystickMotorSpeed(1)
			
			self.CmdMotors()
			
			#print(a.ytilt,a.ztilt,a.yval1,a.buttjpin,a.mot1spd, a.mot2spd, a.mot3spd)
			#print(self.str1,self.str2,self.str3,self.buttjpin,round(self.depth_1,1))#,round(self.depth_2,1))
			#print(self.depth_1)
			# self.accx
			# self.accy
			# self.accz
			#print(self.psi,self.psi2,self.accx,self.accy,self.accz)
		self.StopPeg()

	def SimpleAdjust_noVideo(self):
		input("Simple Adjust, No Video: ")
		self.buttjpin=0
		self.ReadSerial(0)
		print(self.buttjpin)
		print('loop')
		while self.buttjpin==0:
			self.ReadSerial(0)
			self.getJoystickMotorSpeed(1)
			self.CmdMotors()
			#print(a.ytilt,a.ztilt,a.yval1,a.buttjpin,a.mot1spd, a.mot2spd, a.mot3spd)
			print(self.str1,self.str2,self.str3,self.buttjpin)

		self.StopPeg()
	
	def SimpleAdjust_Record(self):
		##NO WEDGE CORRECTION
		wedgectrflag=0
		wedgectr=0
		self.buttjpin=0
		self.get_data(1)
		time.sleep(1)
		input("begin test, simpleAdjust_Record: ")
		self.get_data(1)
		self.buttjpin=0
		#a.writevideo()
		while self.buttjpin==0 :
			
			self.get_data(1)

			print(self.depth,self.str1,self.str2,self.str3)
			self.getJoystickMotorSpeed(0)
			self.CmdMotors()


	def insertion_wedgecorrection_IMU_joystick(self):
		wedgectrflag=0
		wedgectr=0
		self.buttjpin=0
		self.get_data(1)
		time.sleep(1)
		input("begin test: ")
		self.get_data(1)
		self.buttjpin=0
		#a.writevideo()
		while self.buttjpin==0 :
			self.get_data(1)
			#a.writevideo()
			self.getJoystickMotorSpeed(0)
			self.CmdMotors()
			#print(round(a.roll,3),round(a.pitch,3))
			print(round(self.ytilt,3),round(self.ztilt,3))
			if (self.str1<self.str1thresh and self.str2<self.str2thresh and self.str3<self.str3thresh):
				wedgectr=wedgectr+1

			if wedgectr==25:
				self.StopPeg()
				wedgectrflag=a.checkWedging()
				if wedgectrflag==0:
					wedgectr=0
					
			if wedgectrflag==1:
				print("Peg is wedged")
				self.StopPeg()
				self.delaywithvideo(2)
				print('break')
				self.breakWedge()
				print('wedge is broken')
				self.StopPeg()
				self.delaywithvideo(2)
				self.correctWedge()
				self.StopPeg()
				print('wedge is corrected. contunue inserting')
				self.delaywithvideo(2)
				self.get_data(1)
				wedgectr=0
				wedgectrflag=0


	def insertion_wedgecorrection_IMU_auto(self):
		wedgectrflag=0
		wedgectr=0
		self.buttjpin=0
		self.get_data(1)
		time.sleep(1)
		input("begin test: ")
		self.get_data(1)
		self.buttjpin=0
		#a.writevideo()
		while self.buttjpin==0 :
			self.get_data(1)
			LowerPeg(self,1)
			print(round(self.ytilt,3),round(self.ztilt,3))
			if (self.str1<self.str1thresh and self.str2<self.str2thresh and self.str3<self.str3thresh):
				wedgectr=wedgectr+1

			if wedgectr==25:
				self.StopPeg()
				wedgectrflag=a.checkWedging()
				if wedgectrflag==0:
					wedgectr=0
					
			if wedgectrflag==1:
				print("Peg is wedged")
				self.StopPeg()
				self.delaywithvideo(2)
				print('break')
				self.breakWedge()
				print('wedge is broken')
				self.StopPeg()
				self.delaywithvideo(2)
				self.correctWedge()
				self.StopPeg()
				print('wedge is corrected. contunue inserting')
				self.delaywithvideo(2)
				self.get_data(1)
				wedgectr=0
				wedgectrflag=0



	def insertion_tensioncontroller_entire(self):
		self.buttjpin=0
		input("Insertion Tension Controller Entire: ")
		time.sleep(1)
		self.get_data(1)
		self.buttjpin=0
		#pdb.set_trace()
		ctr=0
		while self.buttjpin==0:

			try:
				self.get_data(1)
				#print('data')

				if ctr<10:
					self.buttjpin=0
				ctr=ctr+1
				#print('ctr')
				
				self.tension_cont_generate_mot_commands()
				
				print(self.mot1spd,self.mot2spd,self.mot3spd)

				self.CmdMotors()
			except:
				print('controller error')
				pdb.set_trace()










		# if spdv==1:
		# 	spd=10000
		# if spdv==0:
		# 	spd=25000
		# self.mot1spd=spd
		# self.mot2spd=spd
		# self.mot3spd=spd
		# #pdb.set_trace()
		# self.ReadSerial(1)
		# err=abs(self.str1-self.str2)
		# spd2=5
		# if self.str1>self.str2:
		# 	try:
		# 		spd=int(round(self.map(err,0,self.str1_zero,0,10),0))
		# 	except:
		# 		spd=int(self.map(err,0,2000,0,5))
		# 	self.MotCmd1=64+spd2
		# 	self.MotCmd2=64-spd+spd2

		# else:
		# 	try:
		# 		spd=int(round(self.map(err,0,self.str2_zero,0,10),0))
		# 	except:
		# 		spd=int(self.map(err,0,2000,0,5))
		# 	self.MotCmd1=64-spd+spd2
		# 	self.MotCmd2=64+spd2
		#print('tension controller')
		#print(self.MotCmd1,self.MotCmd2)





	def insertion_tensioncontroller_entire_wedgecorrection_IMU(self):

		wedgectrflag=0
		wedgectr=0
		self.buttjpin=0
		self.get_data(1)
		time.sleep(1)
		input("begin test, insertion_tensioncontroller_entire_wedgecorrection_IMU: ")
		self.get_data(1)
		self.buttjpin=0
		#a.writevideo()
		while self.buttjpin==0 :
			self.get_data(1)
			#a.writevideo()
			self.tension_cont_generate_mot_commands()
			self.CmdMotors()
			#print(round(a.roll,3),round(a.pitch,3))
			print(round(self.ytilt,3),round(self.ztilt,3))
			if (self.str1<self.str1thresh and self.str2<self.str2thresh and self.str3<self.str3thresh):
				wedgectr=wedgectr+1

			if wedgectr==25:
				self.StopPeg()
				wedgectrflag=a.checkWedging()
				if wedgectrflag==0:
					wedgectr=0
					
			if wedgectrflag==1:
				print("Peg is wedged")
				self.StopPeg()
				self.delaywithvideo(2)
				print('break')
				self.breakWedge()
				print('wedge is broken')
				self.StopPeg()
				self.delaywithvideo(2)
				self.correctWedge()
				self.StopPeg()
				print('wedge is corrected. contunue inserting')
				self.delaywithvideo(2)
				self.get_data(1)
				wedgectr=0
				wedgectrflag=0




	def insertion_tensioncontroller_entire_wedgecorrection_tens(self):

		wedgectrflag=0
		wedgectr=0
		self.buttjpin=0
		self.get_data(1)
		time.sleep(1)
		input("begin test, insertion_tensioncontroller_entire_wedgecorrection_tens : ")
		self.get_data(1)
		self.buttjpin=0
		#a.writevideo()
		while self.buttjpin==0 :
			self.get_data(1)
			#a.writevideo()
			self.tension_cont_generate_mot_commands()
			self.CmdMotors()
			#print(round(a.roll,3),round(a.pitch,3))
			#print(round(self.ytilt,3),round(self.ztilt,3))
			if (self.str1<self.str1thresh and self.str2<self.str2thresh and self.str3<self.str3thresh):
				wedgectr=wedgectr+1

			if wedgectr==25:
				self.StopPeg()
				wedgectrflag=a.checkWedging()
				if wedgectrflag==0:
					wedgectr=0
					
			if wedgectrflag==1:
				print("Peg is wedged")
				self.StopPeg()
				self.delaywithvideo(2)
				print('break')
				self.breakWedge()
				print('wedge is broken')
				self.StopPeg()
				self.delaywithvideo(5)
				self.correctWedge_Tens()
				self.StopPeg()
				print('wedge is corrected. contunue inserting')
				self.delaywithvideo(2)
				self.get_data(1)
				wedgectr=0
				wedgectrflag=0

if __name__ == '__main__':

		#Start up arduino and calibrate imu
		a=FullSystem()

		a.DisplayCameraTestApril()
		#cv2.namedWindow(a.window)


		a.ArduinoSetup()

		a.ConnectToOdrive()


		a.getPegDepth_zeros()

		#a.GetIMUOffsets()

	###### TEST READ SERIAL DATA SAVE AND VIDEO WRITE
		a.SimpleAdjust_noRecord()
		#a.SimpleAdjust_Record()
		try:
			#a.CalibrateIMU()
			#a.GetIMUOffsets()

			#a.SimpleAdjust_Record()
			#a.SimpleAdjust_noRecord()
			
			
			#a.insertion_tensioncontroller_entire_wedgecorrection_tens()
			#a.insertion_tensioncontroller_entire_wedgecorrection_IMU()

			#a.insertion_tensioncontroller_entire()


			




			# a.getPegDepth()


		
			# a.SimpleAdjust_noRecord()

			# #a.SimpleAdjust_Record()
			# a.insertion_tensioncontroller_entire()

		
			




		#WEDGE CORRECTION
		# wedgectrflag=0
		# wedgectr=0
		# a.buttjpin=0
		# a.ReadSerial(1)
		# time.sleep(1)
		# input("begin test: ")
		# a.ReadSerial(1)
		# a.buttjpin=0
		# #a.writevideo()
		# while a.buttjpin==0 :
		# 	a.ReadSerial(1)
		# 	#a.writevideo()
		# 	a.getJoystickMotorSpeed(0)
		# 	a.CmdMotors()
		# 	#print(round(a.roll,3),round(a.pitch,3))
		# 	print(round(a.ytilt,3),round(a.ztilt,3))
		# 	if (a.str1<a.str1thresh and a.str2<a.str2thresh and a.str3<a.str3thresh):
		# 		wedgectr=wedgectr+1

		# 	if wedgectr==25:
		# 		a.StopPeg()
		# 		wedgectrflag=a.checkWedging()
		# 		if wedgectrflag==0:
		# 			wedgectr=0
					
		# 	if wedgectrflag==1:
		# 		print("Peg is wedged")
		# 		a.StopPeg()
		# 		a.delaywithvideo(2)
		# 		print('break')
		# 		a.breakWedge()
		# 		print('wedge is broken')
		# 		a.StopPeg()
		# 		a.delaywithvideo(2)
		# 		a.correctWedge()
		# 		a.StopPeg()
		# 		print('wedge is corrected. contunue inserting')
		# 		a.delaywithvideo(2)
		# 		#a.writevideo()
		# 		a.ReadSerial(1)
		# 		wedgectr=0
		# 		wedgectrflag=0

			a.finishtestrecording()
			print("Peg inserted! Put peg in unloaded positi0 on")
			a.SimpleAdjust_noVideo()

			print('all done')
			time.sleep(2)


		except:
			print("FullSys error!")
			a.StopPeg()
			#pdb.set_trace()
			print("Adjust peg to unloaded position")
			a.ReadSerial(0)
			while a.buttjpin==0 :
				a.ReadSerial(0)
				a.getJoystickMotorSpeed(1)
				a.CmdMotors()
			a.cap1.release()
			a.out1.release()
			a.out2.release()

		
		

	






















