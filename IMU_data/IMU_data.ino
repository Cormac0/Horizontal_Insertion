#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

uint16_t BNO055_SAMPLERATE_DELAY_MS = 20; 
Adafruit_BNO055 bno = Adafruit_BNO055(55);

float ytilt=0.0;
float ztilt=0.0;
float xtilt=0.0;
float qw=0.0;
float qx=0.0;
float qy=0.0;
float qz=0.0;
float accx=0.0;
float accy=0.0;
float accz=0.0;

int sys_cal=0;
int acc_cal=0;
int mag_cal=0;
int gyro_cal=0;

void setup(void) 
{
  Serial.begin(115200);
  Serial.println("Orientation Sensor Test"); Serial.println("");
  
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  delay(1000);
    
  bno.setExtCrystalUse(true);
}

void loop(void) 
{
  /* Get a new sensor event */ 
  sensors_event_t event; 
  bno.getEvent(&event);

  imu::Quaternion quat = bno.getQuat();
  qw=quat.w();
  qx=quat.y();
  qy=quat.x();
  qz=quat.z();
  ytilt=event.orientation.y;
  ztilt=event.orientation.z;
  xtilt=event.orientation.x;

  imu::Vector<3> accvec=bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  accx=accvec.x();
  accy=accvec.y();
  accz=accvec.z();


  uint8_t systemc, gyro, accel, mag;
  systemc = gyro = accel = mag = 0;
  bno.getCalibration(&systemc, &gyro, &accel, &mag);

  sys_cal=systemc;
  gyro_cal=gyro;
  acc_cal=accel;
  mag_cal=mag;
  
  /* Display the floating point data */

  PrintAllData();
}


void PrintAllData()
{ 
//
  Serial.print(ytilt, 4);
  Serial.flush();
  Serial.print(';');
  Serial.flush();
  Serial.print(ztilt, 4);
  Serial.flush();
  Serial.print(';');
  Serial.print(xtilt, 4);
  Serial.flush();
  Serial.print(';');
  
  Serial.print(qw, 4);
  Serial.flush();
  Serial.print(';');
  Serial.print(qx, 4);
  Serial.flush();
  Serial.print(';');
  Serial.print(qy, 4);
  Serial.flush();
  Serial.print(';');
  Serial.flush();
  Serial.print(qz, 4);
  

  Serial.flush();
  Serial.print(';');
  Serial.print(accx);
  Serial.flush();
  Serial.print(';');
  Serial.print(accy);
  Serial.flush();
  Serial.print(';');
  Serial.flush();
  Serial.print(accz);
  
  Serial.flush();
  Serial.print(';');
  Serial.flush();
  Serial.print(sys_cal, DEC);
  Serial.flush();
  Serial.print(';');
  Serial.print(gyro_cal, DEC);
  Serial.flush();
  Serial.print(';');
  Serial.print(acc_cal, DEC);
  Serial.flush();
  Serial.print(';');
  Serial.flush();
  Serial.print(mag_cal, DEC);
  Serial.print('\n');
  //Serial.print(';');
  Serial.flush();

  
  //Serial.println(" ");

  //delay(30);
  delay(1000);
  
  }




 
