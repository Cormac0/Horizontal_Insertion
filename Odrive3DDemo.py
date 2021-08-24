import numpy as np
import cv2
import glob
import argparse
import cv2.aruco as aruco
import pdb
import time
import csv
import serial
import threading
import multiprocessing as mp
import datetime
import math


from OdriveClass import *


def ReadSerial(ser):
			#try:
			ser.flushInput()

			while (ser.inWaiting()<30):
					pass

			try:		
					line = ser.readline()
					#print(len(line))
					# print(line)
					list_of_floats_temp=[]
					list_of_floats_temp = [float(item) for item in line.decode('ascii').strip().split(';')]
					# print("available")
					# print(line.decode('ascii').strip())

					# self.speed1=self.list_of_floats[10]
					# self.speed2=self.list_of_floats[11]
					# self.MotCmd1s=self.list_of_floats[12]
					# self.MotCmd2s=self.list_of_floats[13]

			except:
					pass
					# line = self.ser.readline()
					# print("not available")
					# print(len(line))
					# print(line.decode('ascii').strip())
			
			# # print(line.decode('ascii').strip())
			if len(list_of_floats_temp)==5:
				return list_of_floats_temp
			else:
				return []
					# xval1=list_of_floats[0]
					# yval1=list_of_floats[1]
					# butt1pin=list_of_floats[2]
					# butt2pin=list_of_floats[3]
					# butt3pin=list_of_floats[4]
					# buttjpin=list_of_floats[5]

def map(x,in_min,in_max,out_min,out_max):

			v1=x-in_min
			v2=out_max - out_min
			v3=in_max - in_min

			v4=(v1*v2)/v3 + out_min
			return v4



#208637853548
#2061377C3548
doCalibrate =1

od0 = Odrive('208637853548')
od1 = Odrive('2061377C3548')



ser_add='/dev/ttyACM2'
ser = serial.Serial(ser_add, 115200,timeout=5)
ser.flushInput()

print("connected")

# tc=time.time()
# while (time.time()-tc < 5):
# 	vals=[]

# 	vals=ReadSerial(ser)
# 	#print(vals)
# 	if len(vals)>0:
# 		print(vals)
# 		ul=670
# 		if abs(vals[0]-333)<5:
# 			vals[0]=670/2

# 		#v4=int(map(vals[0],0,ul,-100000,100000))
# 		if vals[0] < 320:
# 			v4=-100000
# 		if vals[0] > 340:
# 			v4=-100000
# 		print(vals[0],v4,vals[2],vals[3],vals[4])
# print('test done')
# time.sleep(5)


if (doCalibrate):
	print('ODrive 0 Calibrating')
	od0.full_init()
	time.sleep(2)
	print('ODrive 1 Calibrating')
	od1.full_init()
	print('Calibration Complete')

ods = [od0, od1]



mot1spd=0
mot2spd=0
mot3spd=0
mot1spd_prev=0
mot2spd_prev=0
mot3spd_prev=0
v4prev=0
v4=0
butt1jprev=0
butt2jprev=0
butt3jprev=0
xval=0
xvalprev=0
tc=time.time()
while (time.time()-tc < 60):
	vals=ReadSerial(ser)


	
	if len(vals)>0:
		#print(vals)
		xval=vals[0]
		butt1=vals[2]
		butt2=vals[3]
		butt3=vals[4]
		ul=670
		#if abs(vals[0]-333)<5:
		#	vals[0]=670/2
		#v4=int(map(vals[0],0,ul,-100000,100000))
		if xval < 300:
			#v4= -100000
			v4=-25000
			#print('v4')
		elif xval > 500:
			#v4=100000
			v4=25000
		elif (xval > 300) and (xval < 500):
			v4=0
			#print('else')

		#print(xval<300)	
		#print(xval,vals[0],v4)
		#print(vals[0],v4,vals[2],vals[3],vals[4])
		v4prev=v4
		xvalprev=xval
		butt1jprev=vals[2]
		butt2jprev=vals[3]
		butt3jprev=vals[4]
	else:
		v4=v4prev
		butt1=butt1jprev
		butt2=butt2jprev
		butt3=butt3jprev



	if butt1==0 and butt2==0 and butt3==0:
		mot1spd=0
		mot2spd=0
		mot3spd=0


	else:
		if butt1==1:
			mot1spd=v4

		if butt2==1:
			mot2spd=v4

		if butt3==1:
			mot3spd=v4



	print(xvalprev,v4,butt1,butt2,butt3,mot1spd,mot2spd,mot3spd)
	if mot1spd != mot1spd_prev:
		od0.VelMove(mot1spd,0)
		mot1spd_prev=mot1spd

	if mot2spd != mot2spd_prev:
		od0.VelMove(mot2spd,1)
		mot2spd_prev=mot2spd

	if mot3spd != mot3spd_prev:
		od1.VelMove(mot3spd,1)
		mot3spd_prev=mot3spd

	# od0.VelMove(mot1spd,1)
	# od1.VelMove(mot3spd,1)


od0.VelMove(0,0)
od0.VelMove(0,1)
od1.VelMove(0,1)
print('all done')
time.sleep(2)






# print('moving 1')
# od0.PosMove(400000,0)
# od0.PosMove(400000,1)
# od1.PosMove(400000,1)
# time.sleep(5)
# print('moving 2')
# od0.PosMove(0,0)
# od0.PosMove(0,1)
# od1.PosMove(0,1)
# time.sleep(5)

# print('vel moving 1')
# od0.VelMove(100000,0)
# od0.VelMove(100000,1)
# od1.VelMove(100000,1)
# time.sleep(5)
# od0.VelMove(0,0)
# od0.VelMove(0,1)
# od1.VelMove(0,1)
# time.sleep(1)
# print('vel moving 2')
# od0.VelMove(-100000,0)
# od0.VelMove(-100000,1)
# od1.VelMove(-100000,1)
# time.sleep(5)
# print('stop 1')
# od0.VelMove(0,0)
# od0.VelMove(0,1)
# od1.VelMove(0,1)

#VelMove(self,vel_setpt, num):


def ReadSerial(ser):
			#try:
			ser.flushInput()

			while (ser.inWaiting()<30):
					pass

			try:		
					line = ser.readline()
					#print(len(line))
					# print(line)
					list_of_floats_temp=[]
					list_of_floats_temp = [float(item) for item in line.decode('ascii').strip().split(';')]
					# print("available")
					# print(line.decode('ascii').strip())

					# self.speed1=self.list_of_floats[10]
					# self.speed2=self.list_of_floats[11]
					# self.MotCmd1s=self.list_of_floats[12]
					# self.MotCmd2s=self.list_of_floats[13]

			except:
					pass
					# line = self.ser.readline()
					# print("not available")
					# print(len(line))
					# print(line.decode('ascii').strip())
			return list
			# # print(line.decode('ascii').strip())
			# if len(list_of_floats_temp)==13:
			# 		xval1=list_of_floats[0]
			# 		yval1=list_of_floats[1]
			# 		butt1pin=list_of_floats[2]
			# 		butt2pin=list_of_floats[3]
			# 		butt3pin=list_of_floats[4]
			# 		buttjpin=list_of_floats[5]