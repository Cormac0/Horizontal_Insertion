from OdriveClass import *


#208637853548
#2061377C3548
doCalibrate = 0

od0 = Odrive('208637853548')
od1 = Odrive('2061377C3548')



if (doCalibrate):
	print('ODrive 0 Calibrating')
	od0.full_init()
	time.sleep(2)
	print('ODrive 1 Calibrating')
	od1.full_init()
	print('Calibration Complete')

ods = [od0, od1]
time.sleep(2)

print('moving 1')
od0.PosMove(400000,0)
od0.PosMove(400000,1)
od1.PosMove(400000,1)
time.sleep(5)
print('moving 2')
od0.PosMove(0,0)
od0.PosMove(0,1)
od1.PosMove(0,1)
time.sleep(5)

print('vel moving 1')
od0.VelMove(100000,0)
od0.VelMove(100000,1)
od1.VelMove(100000,1)
time.sleep(5)
od0.VelMove(0,0)
od0.VelMove(0,1)
od1.VelMove(0,1)
time.sleep(1)
print('vel moving 2')
od0.VelMove(-100000,0)
od0.VelMove(-100000,1)
od1.VelMove(-100000,1)
time.sleep(5)
print('stop 1')
od0.VelMove(0,0)
od0.VelMove(0,1)
od1.VelMove(0,1)

#VelMove(self,vel_setpt, num):