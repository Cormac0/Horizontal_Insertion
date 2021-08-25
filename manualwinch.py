import tkinter as tk
import pygubu
import csv
import serial
import rospy
import numpy as np
import PID

import ctypes
from can_msgs import msg
import fcntl
import termios
import sys
import select
import subprocess
import os
from threading import Timer
import signal
from termios import tcflush, TCIOFLUSH
from rospy.core import NullHandler
import tty

from pyquaternion import Quaternion
from OdriveClass import *

# Note that positive velocity values lower winch 1
MAX_VEL = 100000 # Max. speed for winch in encoder counts per second

LIVEPLOTTER = 1
doCalibrate = 0

class numhex64(ctypes.Union):
    _fields_ = [("num", ctypes.c_double),
                ("sint", ctypes.c_int64),
                ("uint", ctypes.c_uint64),
                ("hex", ctypes.c_ubyte * 8)]

class numhex32(ctypes.Union):
    _fields_ = [("num", ctypes.c_float),
                ("sint", ctypes.c_int32),
                ("uint", ctypes.c_uint32),
                ("hex", ctypes.c_ubyte * 4)]


class ManualWinchApp:
    
    def __init__(self):

        # Initalize the gantry:
        os.system("stty -echo")
        rospy.init_node('can_send', anonymous=True)
        pub = rospy.Publisher('sent_messages', msg.Frame, queue_size=100)
        rate = rospy.Rate(50)
        xSpeed = numhex64()
        ySpeed = numhex64()
        msgData = ""
        frame = msg.Frame()
        frame.is_rtr = False
        frame.is_extended = False
        frame.dlc = 8

        self.gantry_x_pid = PID.PID(P=0.2, I=0.0, D=0.0)
        self.gantry_z_pid = PID.PID(P=0.2, I=0.0, D=0.0)

        #1: Create a builder
        self.builder = builder = pygubu.Builder()

        #2: Load an ui file
        builder.add_from_file('WinchesManualGUI.ui')

        #3: Create the mainwindow
        self.mainwindow = builder.get_object('MainWindow')

        #4: Connect callbacks
        builder.connect_callbacks(self)

        self.ser_add='/dev/ttyACM0'   #For Strain Gauges and IMU
        self.testCounter=1

        self.list_of_floats=[]

        # Data from gantry crane:
        self.x = 0
        self.y = 0
        self.z = 0

        #self.xval1=0
        #self.yval1=0
        #self.buttjpin=0
        #self.butt1pin=0
        #self.butt2pin=0
        #self.butt3pin=0
        #self.str1=0
        #self.str2=0
        #self.str3=0
        self.ytilt=0
        self.ztilt=0
        self.xtilt=0        

        self.qw=0
        self.qx=0
        self.qy=0
        self.qz=0
        self.q = Quaternion(self.qw,self.qx,self.qy,self.qz)
        self.rot_ax = self.q.axis
        self.rot_ang = self.q.degrees

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

        #print([self.str1thresh,self.str2thresh,self.str3thresh])
        #time.sleep(.5)
        # get calibration parameters

        self.list_of_floats=[]
        self.list_of_floats_temp=[]
        self.TotalList=[]
        
    def run(self):
        self.mainwindow.mainloop()

    def winch1scale_move(self, vel):
        des_vel = MAX_VEL*float(vel)/100
        odrv0.VelMove(des_vel,0)

    def winch2scale_move(self, vel):
        des_vel = MAX_VEL*float(vel)/100
        odrv1.VelMove(des_vel,0)

    def winch3scale_move(self, vel):
        des_vel = MAX_VEL*float(vel)/100
        odrv1.VelMove(des_vel,1)

    def move_all(self,vel):
        des_vel = MAX_VEL*float(vel)/100
        odrv0.VelMove(des_vel,0)
        odrv1.VelMove(des_vel,0)
        odrv1.VelMove(des_vel,1)

    def stopall_butt(self):
        odrv0.VelMove(0,0)
        odrv1.VelMove(0,0)
        odrv1.VelMove(0,1)

    def get_gantry_coords(self):
        self.x = 0
        self.y = 0
        self.z = 0

    def move_gantry(self,x,y,z):
        xSpeed.num = 0.0
        zSpeed.num = 0.0

        frame.id = 0x01
        msgData = ""
        for idx in range(8):
            msgData += chr(xSpeed.hex[idx])
        frame.data = msgData
        frame.header.stamp = rospy.Time.now()
        pub.publish(frame)

        frame.id = 0x02
        msgData = ""
        for idx in range(8):
            msgData += chr(zSpeed.hex[idx])
        frame.data = msgData
        frame.header.stamp = rospy.Time.now()
        pub.publish(frame)

        rospy.loginfo("x: %f rps, y: %f rps", xSpeed.num, zSpeed.num) 
        
        rate.sleep()
    
        return 0

##### ARDUINO SERIAL FUNCS

    def ArduinoSetup(self):

        userinput=input('Setting up the arduino. If you restarted the arduino, unload everything and then enter 1 so it can calibrate')
        #print(type(int(userinput)))
        print(self.ser_add)
        print(self.ser_add2)
        self.ser = serial.Serial(self.ser_add, 115200,timeout=1)
        
        # self.ser.flushInput()
        
        # self.ser.write(int(userinput))
        # self.ser.flushInput()


        self.ser2 = serial.Serial(self.ser_add2, 115200,timeout=1)
        
        # self.ser2.flushInput()
        
        # self.ser2.write(int(userinput))
        # self.ser2.flushInput()


        print(self.ser_add)
        print(self.ser_add2)
        print("connected")


        # Calibrate Arduino if needed
        line=[]
        ctr=0
        while a.buttjpin==0:

                try:
                        
                        line = self.ser.readline()
                        line.decode('ascii').strip()
                        print(line.decode('ascii').strip())
                        list_of_floats_temp=[]
                        list_of_floats_temp_2=[]
                        list_of_floats_temp_1= [float(item) for item in line.decode('ascii').strip().split(';')]

                        line2 = self.ser2.readline()
                        line2.decode('ascii').strip()
                        print(line2.decode('ascii').strip())
                        
                        
                        list_of_floats_temp_2= [float(item) for item in line2.decode('ascii').strip().split(';')]

                        list_of_floats_temp_1.extend(list_of_floats_temp_2)
                        print(list_of_floats_temp_1)



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
                #print([	self.str1,self.str2,self.str3])
                print([	self.str1,self.str2,self.str3, self.phi1enc,self.phi2enc,self.phi3enc])
                #print([	self.phi1enc,self.phi2enc,self.phi3enc, self.phi1deg,self.phi2deg,self.phi3deg])
                

        input('If you are happy with the serial, press 1 to continue. otherwise, restart the python ')

    def ReadSerial(self,tosaveflag):

        self.ser.flushInput()
        self.ser2.flushInput()
        # while (self.ser.inWaiting()<30 and self.ser2.inWaiting()<15):
        # 		pass

        try:
            line2 = self.ser2.readline()
            self.list_of_floats_temp2=[]
            self.list_of_floats_temp2 = [float(item) for item in line2.decode('ascii').strip().split(';')]


            line = self.ser.readline()
            self.list_of_floats_temp=[]
            self.list_of_floats_temp = [float(item) for item in line.decode('ascii').strip().split(';')]
            
            self.list_of_floats_temp.extend(self.list_of_floats_temp2)
            #print(self.list_of_floats_temp)
            #print(len(self.list_of_floats_temp))

        # while (self.ser.inWaiting()<30):
        # 		pass
        # 		#print('less')

        # try:		
        # 		line = self.ser.readline()
        # 		line2=line
        # 		#print(len(line))
        # 		# print(line)
        # 		self.list_of_floats_temp=[]
        # 		self.list_of_floats_temp = [float(item) for item in line.decode('ascii').strip().split(';')]


        # except:
        # 		pass


        # while (self.ser2.inWaiting()<30):
        # 		pass

        # try:		
        # 		line2 = self.ser2.readline()
        # 		self.list_of_floats_temp2=[]
        # 		self.list_of_floats_temp2 = [float(item) for item in line2.decode('ascii').strip().split(';')]


        # except:
        # 		pass


        
        #self.list_of_floats_temp.extend(self.list_of_floats_temp2)

        #print(self.list_of_floats_temp)
        # print(line.decode('ascii').strip())
        #print(len(self.list_of_floats_temp))
            if len(self.list_of_floats_temp)==28:
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
                self.q = Quaternion(self.qw,self.qx,self.qy,self.qz)
                self.rot_ax = self.q.axis
                self.rot_ang = self.q.degrees

                self.accx=self.list_of_floats[15]
                self.accy=self.list_of_floats[16]
                self.accz=self.list_of_floats[17]
                self.sys_cal=self.list_of_floats[18]
                self.gyro_cal=self.list_of_floats[19]
                self.acc_cal=self.list_of_floats[20]
                self.mag_cal=self.list_of_floats[21]

                self.phi1enc=self.list_of_floats[22]
                self.phi2enc=self.list_of_floats[23]
                self.phi3enc=self.list_of_floats[24]
                self.beta1enc=self.list_of_floats[25]
                self.beta2enc=self.list_of_floats[26]
                self.beta3enc=self.list_of_floats[27]


                self.phi1deg=360-self.phi1enc/16384*360
                self.phi2deg=360-self.phi2enc/16384*360
                self.phi3deg=360-self.phi3enc/16384*360
                self.beta1deg=self.beta1enc/16384*360
                self.beta2deg=self.beta2enc/16384*360
                self.beta3deg=self.beta3enc/16384*360



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
                #self.list_of_floats.extend(self.pegrvec)
                #self.list_of_floats.extend(self.pegtvec)
                #self.list_of_floats.extend(self.holervec)
                #self.list_of_floats.extend(self.holetvec)

                #self.getPegDepth()
                #print(self.depth_1,self.depth_2)
                #self.list_of_floats.append(self.phi1deg)
                #self.list_of_floats.append(self.phi2deg)
                #self.list_of_floats.append(self.phi3deg)
                #self.list_of_floats.append(self.beta1deg)
                #self.list_of_floats.append(self.beta2deg)
                #self.list_of_floats.append(self.beta3deg)

                #self.list_of_floats.append(self.depth_1)

                self.winchenc1=0
                self.winchenc2=0
                self.winchenc3=0

                if self.connectflag==1:
                    self.winchenc1=self.odrv0.get_encoder_count(0)
                    self.winchenc2=self.odrv1.get_encoder_count(0)
                    self.winchenc3=self.odrv1.get_encoder_count(1)
                self.list_of_floats.append(self.winchenc1)
                self.list_of_floats.append(self.winchenc2)
                self.list_of_floats.append(self.winchenc3)

                self.list_of_floats.append(self.mot1spd)
                self.list_of_floats.append(self.mot2spd)
                self.list_of_floats.append(self.mot3spd)
                




                #self.phi1rad=self.phi1deg*3.14/180
                #self.phi2rad=self.phi2deg*3.14/180
                #self.phi3rad=self.phi3deg*3.14/180

                #self.str1P=self.str1*np.cos(self.phi1rad)
                #self.str2P=self.str2*np.cos(self.phi2rad)
                #self.str3P=self.str3*np.cos(self.phi3rad)

                
                self.list_of_floats.insert(0,time.time())



                #yrdgs.append((self.ytilt-self.ytilt_zero))
                #zrdgs.append((self.ztilt-self.ztilt_zero))

                #self.ytilta=self.avg(yrdgs)
                #self.ztilta=self.avg(zrdgs)

                #self.list_of_floats.append(self.ytilta)
                #self.list_of_floats.append(self.ztilta)
                if len(yrdgs)==20:
                    yrdgs.pop(0)
                if len(zrdgs)==20:
                    zrdgs.pop(0)




                if tosaveflag==1:
                    self.DataToSave()
        except:
            pass
    

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


""" 	def Sensorcheck(self):
        self.buttjpin=0
        sensorcheckflag=0
        sensorcheckflag=int(input("Do you want to check sensors? 1 for yes: "))
        if sensorcheckflag==1:
            while self.buttjpin==0:
                    self.get_data(0)
                    #print([	self.str1,self.str2,self.str3])
                    print([	round(self.str1,2), round(self.str2,2),round(self.str3,2), round(self.phi1deg,1),round(self.phi2deg,1),round(self.phi3deg,1), round(self.beta1deg,1),round(self.beta2deg,1),round(self.beta3deg,1)])
                    #print([	self.phi1enc,self.phi2enc,self.phi3enc, self.phi1deg,self.phi2deg,self.phi3deg])
            time.sleep(1)
            self.buttjpin=0
            while self.buttjpin==0:
                    self.get_data(0)
                    #print([	self.str1,self.str2,self.str3])
                    print([	round(self.ytilt,3), round(self.ztilt,3)])
                    #print([	self.phi1enc,self.phi2enc,self.phi3enc, self.phi1deg,self.phi2deg,self.phi3deg]) """

        ### SAVING AND GET DATA

    def SetupNewFile(self):
            if self.testname=='1':
                self.tn=input("Enter Test name for series, without number: ")
            self.TotalList=[]
            self.testname=self.tn+'_'+str(self.testCounter) 
            self.testCounter=self.testCounter+1
            print(self.testname)
            self.vidname='/home/rachel/odrive/Data_and_Vids/'+ self.testname
            self.filename=self.vidname+".csv"
            self.vidfile1=self.vidname+".avi"
            self.vidfile2=self.vidname+"_2"+".avi"
            # self.cap1 = cv2.VideoCapture(int(self.camnum1))	
            # self.frame_width1 = int(self.cap1.get(3))
            # self.frame_height1 = int(self.cap1.get(4))
            
            self.window = 'Camera'
            self.out1 = cv2.VideoWriter(self.vidfile1,cv2.VideoWriter_fourcc('M','J','P','G'), 10, (self.frame_width1,self.frame_height1))
            self.out2 = cv2.VideoWriter(self.vidfile2,cv2.VideoWriter_fourcc('M','J','P','G'), 10, (self.frame_width1,self.frame_height1))
            
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
        #self.cap1.release()
        self.out1.release()
        self.out2.release()
        cv2.destroyAllWindows()

        

    def get_data(self,tosaveflag):
        self.ReadDisplayCVApril(tosaveflag)
        self.ReadSerial(tosaveflag)

    def ring_alignment(self):
        """ Perform the alignment of the ring and the peg using data from the IMU. """
        xtilt_thresh = 3
        ytilt_thresh = 3
        while True:
            dz = 0.03
            self.move_gantry(self.x, self.y, self.z+dz)
            perp_vec = np.cross(self.rot_ax, [0,0,1])
            if self.rot_ang > 3:
                self.move_gantry(self.x-perp_vec(0),self.y-perp_vec(2),self.z-perp_vec(3))
            elif self.rot_ang < 3:
                break
        self.move_gantry(self.x,self.y,self.z+0.1)


    def exit_system(self):
            os.system("stty echo")
            sys.exit()



        



if __name__ == '__main__':
    odrv0 = Odrive('20673881304E') # Only has 1 winch
    odrv1 = Odrive('2087377E3548') # Has 2 winches

    if (doCalibrate):
        print('ODrive 0 Calibrating')
        od0.full_init()
        time.sleep(2)
        print('ODrive 1 Calibrating')
        od1.full_init()
        print('Calibration Complete')

    app = ManualWinchApp()
    app.run()
