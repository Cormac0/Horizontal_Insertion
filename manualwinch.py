import tkinter as tk
import pygubu
import csv
import serial
import rospy
import numpy as np

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
GANTRY_MAX = 2

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
        self.pub = rospy.Publisher('/sent_messages', msg.Frame, queue_size=100)
        self.rate = rospy.Rate(50)

        #1: Create a builder
        self.builder = builder = pygubu.Builder()

        #2: Load an ui file
        builder.add_from_file('WinchesManualGUI.ui')

        #3: Create the mainwindow
        self.mainwindow = builder.get_object('MainWindow')

        #4: Connect callbacks
        builder.connect_callbacks(self)

        self.ser_add='/dev/ttyACM1'   #For IMU/Arduino
        self.testCounter=1

        self.list_of_floats=[]

        # Data from gantry crane:
        self.vx = 0
        self.vy = 0
        self.vz = 0

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
        self.ytilt_zero=1.6875
        self.ztilt_zero=-0.4375
        self.xtilt_zero=-3.2
        self.accx_zero=0.34
        self.accy_zero=0.0280
        self.accz_zero=9.43
        self.angle_Zthresh=.75
        self.angle_Ythresh=.75
        self.exitholeflag=0

        self.psiwedge=0

        self.pitch=0
        self.roll=0


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

    def move_gantry(self,vx,vy,vz):
        # INPUTS ARE IN THE FORM OF THE IMU ROT_AX COORDINATE SYSTEM
        ySpeed = numhex64()
        xSpeed = numhex64()

        msgData1 = ""
        frame1 = msg.Frame()
        frame1.is_rtr = False
        frame1.is_extended = False
        frame1.dlc = 8

        msgData2 = ""
        frame2 = msg.Frame()
        frame2.is_rtr = False
        frame2.is_extended = False
        frame2.dlc = 8
        # flip signs to convert from IMU coordinate system to that used by the gantry.
        xSpeed.num = vx
        ySpeed.num = vy

        # Odrive speed
        zSpeed = -vz
    
        frame1.id = 0x01
        msgData1 = ""
        for idx in range(8):
            msgData1 += chr(xSpeed.hex[idx])
        frame1.data = msgData1
        frame1.header.stamp = rospy.Time.now()
        self.pub.publish(frame1)

        frame2.id = 0x02
        msgData2 = ""
        for idx in range(8):
            msgData2 += chr(ySpeed.hex[idx])
        frame2.data = msgData2
        frame2.header.stamp = rospy.Time.now()
        self.pub.publish(frame2)

        des_vel = MAX_VEL*float(zSpeed)/100
        odrv0.VelMove(des_vel,0)

        self.vx = xSpeed.num
        self.vy = ySpeed.num
        self.vz = -zSpeed

        rospy.loginfo("x: %f rps, y: %f rps, z: %f perc", xSpeed.num, ySpeed.num, -zSpeed) 
        
        self.rate.sleep()
    
        return 0

    def vx_scl_cb(self, vel):
        vdes = (vel/100)*GANTRY_MAX
        self.move_gantry(vdes,self.vy,self.vz)
        return 0
    
    def vy_scl_cb(self, vel):
        vdes = (vel/100)*GANTRY_MAX
        self.move_gantry(self.vx,vdes,self.vz)
        return 0

    def vz_scl_cb(self, vel):
        self.move_gantry(self.vx,self.vy,vdes)
        return 0

##### ARDUINO SERIAL FUNCS

    def ArduinoSetup(self):
        userinput=input('Setting up the arduino. If you restarted the arduino, unload everything and then enter 1 so it can calibrate')
        #print(type(int(userinput)))
        print(self.ser_add)
        self.ser = serial.Serial(self.ser_add, 115200,timeout=1)
        
        # self.ser.flushInput()
        
        # self.ser.write(int(userinput))
        # self.ser.flushInput()

        print(self.ser_add)
        print("connected")


        # Calibrate Arduino if needed
        line=[]
        ctr=0
        t_init = time.time()
        while time.time()<t_init+10:

                try:
                        
                        line = self.ser.readline()
                        line.decode('ascii').strip()
                        print(line.decode('ascii').strip())
                        #print(line.decode('ascii').strip().split(';'))
                        list_of_floats_temp=[]
                        #list_of_floats_temp_2=[]
                        list_of_floats_temp_1= [float(item) for item in line.decode('ascii').strip().split(';')]
                        

                        #list_of_floats_temp_1.extend(list_of_floats_temp_2)
                        print(list_of_floats_temp_1)
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
                print([	self.xtilt,self.ytilt,self.ztilt])
                #print([	self.phi1enc,self.phi2enc,self.phi3enc, self.phi1deg,self.phi2deg,self.phi3deg])
                

        input('If you are happy with the serial, press 1 to continue. otherwise, restart the python ')
    
    def ReadSerial_cb():
        self.ReadSerial()
        self.get_data(0)
        print(self.xtilt, self.ytilt, self.ztilt)
    
    def ReadSerial(self,tosaveflag):
        self.ser.flushInput()
        # while (self.ser.inWaiting()<30 and self.ser2.inWaiting()<15):
        # 		pass
        while True:
            try:
                line = self.ser.readline()
                line.decode('ascii').strip()
                self.list_of_floats_temp=[]
                if line=='':
                    print('empty line')
                    return 0
                if line==' ':
                    print('space line')
                    return 0
                for item in line.decode('ascii').strip().split(';'):
                    print(item)
                    #print('next')
                    if item == '':
                        print('empty item')
                        return 0
                    if item == ' ':
                        print('space item')
                        return 0
                self.list_of_floats_temp = [float(item) for item in line.decode('ascii').strip().split(';')]
                print(self.list_of_floats_temp)
                
                if len(self.list_of_floats_temp)==14:
                    #print("List of floats accessed.")
                    self.list_of_floats=[]
                    self.list_of_floats=self.list_of_floats_temp

                    self.ytilt=self.list_of_floats[0]
                    self.ztilt=self.list_of_floats[1]
                    self.xtilt=self.list_of_floats[2]
                    # Ensure range is from (-180,180]
                    if self.ytilt>180:
                        self.ytilt = self.ytilt-360
                    if self.xtilt>180:
                        self.xtilt = self.xtilt-360
                    if self.ztilt>180:
                        self.ztilt = self.ztilt-360

                    self.qw=self.list_of_floats[3]
                    self.qx=self.list_of_floats[4]
                    self.qy=self.list_of_floats[5]
                    self.qz=self.list_of_floats[6]
                    self.q = Quaternion(self.qw,self.qx,self.qy,self.qz)
                    self.rot_ax = self.q.axis
                    self.rot_ang = self.q.degrees

                    self.accx=self.list_of_floats[7]
                    self.accy=self.list_of_floats[8]
                    self.accz=self.list_of_floats[9]
                    self.sys_cal=self.list_of_floats[10]
                    self.gyro_cal=self.list_of_floats[11]
                    self.acc_cal=self.list_of_floats[12]
                    self.mag_cal=self.list_of_floats[13]

                    self.list_of_floats.append(self.ytilt_zero)
                    self.list_of_floats.append(self.ztilt_zero)
                    self.list_of_floats.append(self.xtilt_zero)
                    self.list_of_floats.append(self.accx_zero)
                    self.list_of_floats.append(self.accx_zero)
                    self.list_of_floats.append(self.accz_zero)






                    self.pitch = 180 * np.arctan2(self.accx ,np.sqrt(self.accy*self.accy+ self.accz*self.accz))/3.14;
                    self.roll = 180 * np.arctan2(self.accy, np.sqrt(self.accx*self.accx + self.accz*self.accz))/3.14;

                    self.list_of_floats.append(self.pitch)
                    self.list_of_floats.append(self.roll)

                    self.winchenc1=0
                    self.winchenc2=0
                    self.winchenc3=0

                    self.list_of_floats.append(self.winchenc1)
                    self.list_of_floats.append(self.winchenc2)
                    self.list_of_floats.append(self.winchenc3)
                    
                    self.list_of_floats.insert(0,time.time())

                    if tosaveflag==1:
                        self.DataToSave()
                else:
                    print("List of floats not correct length.")
                    print(len(self.list_of_floats_temp))
                    pass
                break       
            except ValueError:
                print("Value error. Trying again...")
    

    def CalibrateIMU(self):
        value=input("Calibrate the IMU. Press 1 to start.")
        t_init = time.time()
        while time.time()<t_init+3:
            self.get_data(0)
            print(self.sys_cal,self.gyro_cal,self.acc_cal,self.mag_cal)

        self.get_data(0)
        time.sleep(2)
        input("get IMU Data. Press 1 to start.")
        self.get_data(0)
        t_init = time.time()
        while time.time()<t_init+3:
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
            t_init = time.time()
            while time.time()<t_init+1:
                self.get_data(0)
            
            input("Let the peg rest so vals can be obtained: ")
            t_init=time.time()
            accxlist=[]
            accylist=[]
            acczlist=[]
            ytiltlist=[]
            ztiltlist=[]
            xtiltlist=[]
            rolllist=[]
            pitchlist=[]
            while time.time()-t_init<5:
                self.ReadSerial(0)
                accxlist.append(self.accx)
                accylist.append(self.accy)
                acczlist.append(self.accz)
                ytiltlist.append(self.ytilt)
                ztiltlist.append(self.ztilt)
                rolllist.append(self.roll)
                pitchlist.append(self.pitch)
                xtiltlist.append(self.xtilt)

            self.accx_zero=self.avg(accxlist)
            self.accy_zero=self.avg(accylist)
            self.accz_zero=self.avg(acczlist)
            # self.ytilt_zero=self.avg(rolllist)
            # self.ztilt_zero=self.avg(pitchlist)
            self.ytilt_zero=self.avg(ytiltlist)
            self.ztilt_zero=self.avg(ztiltlist)
            self.xtilt_zero=self.avg(xtiltlist)
            print("done!")
            print(self.ytilt_zero,self.ztilt_zero,self.xtilt_zero,self.accx_zero,self.accy_zero,self.accz_zero)
            input("Write down ytilt_zero,ztilt_zero,xtilt_zero and accx_zero,accy_zero,accz_zero in the code for future use!!")

    def avg(self,lst):
        return sum(lst) / len(lst) 
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
        #self.ReadDisplayCVApril(tosaveflag)
        self.ReadSerial(tosaveflag)

    def pause(self,time_arr,curr_data,xtilt_data,ytilt_data, duration):
        t_init = time.time()
        while time.time() < t_init+duration:
            t_init_2 = time.time()
            temp_data = []
            temp_data1 = []
            temp_data2 = []
            while time.time() < t_init_2 + 0.4:
                self.get_data(1)
                temp_data.append(self.rot_ang)
                temp_data1.append(self.xtilt)
                temp_data2.append(self.ytilt)
                #temp_data1.append(self.rot_ax[1])
                #temp_data2.append(self.rot_ax[2])
            curr_data.append(np.mean(temp_data))
            time_arr.append(t_init+(duration/2))
            xtilt_data.append(np.mean(temp_data1))
            ytilt_data.append(np.mean(temp_data2))
        return curr_data

    def nopause(self,time_arr,curr_data,xtilt_data,ytilt_data, duration):
        t_init = time.time()
        while time.time() < t_init+duration:
            t_init_2 = time.time()
            temp_data = []
            temp_data1 = []
            temp_data2 = []
            while time.time() < t_init_2 + 0.2:
                self.get_data(1)
                temp_data.append(self.rot_ang)
                temp_data1.append(self.xtilt)
                temp_data2.append(self.ytilt)
                #temp_data1.append(self.rot_ax[1])
                #temp_data2.append(self.rot_ax[2])
            curr_data.append(np.mean(temp_data))
            time_arr.append(t_init+(duration/2))
            xtilt_data.append(np.mean(temp_data1))
            ytilt_data.append(np.mean(temp_data2))
        return curr_data

    def ring_alignment(self):
        """ Perform the alignment of the ring and the peg using data from the IMU. """
        graph_data_arr = []
        graph_time_arr = []
        graph_xtilt_arr = []
        graph_ytilt_arr = []

        rot_thresh = 6
        self.get_data(0)
        t_init0 = time.time()
        while abs(self.ytilt)<8:
            self.get_data(0)
            self.move_gantry(1,0,0)
            self.nopause(graph_time_arr,graph_data_arr,graph_xtilt_arr,graph_ytilt_arr,0.5)
            if time.time()-t_init0 > 10:
                break
        self.move_gantry(1,0,0)
        rot_ang = self.rot_ang
        t_last_correction = time.time()
        while True:
            #break
            if time.time()-t_init0>180:
                print("Abort")
                break
            self.get_data(0)
            rot_ang = self.rot_ang
            dx = 1
            self.move_gantry(dx, 0, 0)
            perp_vec = np.cross(self.rot_ax, [1,0,0])
            print(self.rot_ax)
            print(perp_vec)
            print(rot_ang)
            #time.sleep(1.5)
            self.pause(graph_time_arr,graph_data_arr,graph_xtilt_arr,graph_ytilt_arr,1.5)
            self.move_gantry(0,0,0)
            #time.sleep(2)
            self.pause(graph_time_arr,graph_data_arr,graph_xtilt_arr,graph_ytilt_arr,2)
            #if abs(self.rot_ang) > rot_thresh and (abs(self.ytilt)>3 or abs(self.xtilt)>3):
            if (abs(self.ytilt)>6 or abs(self.xtilt)>6):
                if self.rot_ang > 0:
                    vy = 0
                    vz = 0
                    if self.ytilt<-2:
                        vz = -abs(perp_vec[2])
                    elif self.ytilt>2:
                        vz = abs(perp_vec[2])
                    if self.xtilt<-2:
                        vy = abs(perp_vec[1])
                    elif self.xtilt>2:
                        vy = -abs(perp_vec[1])
                    self.move_gantry(-dx,vy*3,vz*3)
                    t_last_correction = time.time()
                    #time.sleep(1.5)
                    self.pause(graph_time_arr,graph_data_arr,graph_xtilt_arr,graph_ytilt_arr,1.5)
                    #self.move_gantry(0,0,0)
                    #print(rot_ang)
                elif self.rot_ang < 0:
                    pass
            else:
                print(rot_ang)
                self.move_gantry(dx,0,0)
                #time.sleep(1)
                self.pause(graph_time_arr,graph_data_arr,graph_xtilt_arr,graph_ytilt_arr,1)
                if time.time()-t_last_correction>30:
                    break
        self.move_gantry(0,0,0)
        print('Insertion complete.')
         # Parameters
        y_range = [-3,1]

        # Create figure for plotting
        fig = plt.figure()
        #ax.set_ylim(y_range)
        graph_time_arr = [t - t_init0 for t in graph_time_arr]

        filename = "angle_data.csv"
        with open(filename, "w") as f:
            writer = csv.writer(f)
            writer.writerow(graph_time_arr)
            writer.writerow(graph_data_arr)
            writer.writerow(graph_xtilt_arr)
            writer.writerow(graph_ytilt_arr)
            print("saved")

        plt.plot(graph_time_arr,graph_data_arr)

        plt.title('IMU Data')
        plt.xlabel('Time')
        plt.ylabel('Angle')
        plt.show()


    def exit_system(self):
            os.system("stty echo")
            sys.exit()

    def animate(i, ys, odrv1):
        time_end = time.time()+0.2
        graph_data_arr = []
        graph_data_arr.append(odrv1.get_current(0))
        while time.time() < time_end:
            graph_data_arr.append(odrv1.get_current(0))
        graph_data = np.mean(graph_data_arr)

        # Add x and y to lists
        #xs.append(dt.datetime.now().strftime('%H:%M:%S.%f'))
        #xs.append(i)
        ys.append(graph_data)

        # Limit x and y lists to 20 items
        #xs = xs[-20:]
        #ys = ys[-20:]
        ys = ys[-x_len:]
        
        # Format plot
        #ax.clear()
        #ax.plot(xs,ys)

        # Update line with new Y values
        line.set_ydata(ys)

        return line,

        # Format plot
        #plt.xticks(rotation=45, ha='right')
        #plt.subplots_adjust(bottom=0.30)
        #plt.title('Current over Time')
        #plt.ylabel('Current (A)')



        



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

    os.system("stty -echo")
    rospy.init_node('can_send', anonymous=True)

    app = ManualWinchApp()
    #app.move_gantry(0,0,0)
    #t_init = time.time()
    #while time.time() < t_init+2:
    #    app.move_gantry(0.5,20,0)
    #app.move_gantry(0,0,0)
    #rospy.spin()
    app.ArduinoSetup()
    app.CalibrateIMU()
    app.GetIMUOffsets()
    app.ReadSerial(0)
    val = input("Press 1 to start ring alignment test.")
    app.ring_alignment()
    
    app.run()
