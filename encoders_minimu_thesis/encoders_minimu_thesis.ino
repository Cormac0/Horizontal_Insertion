

//Includes required to use Roboclaw library
#include <SoftwareSerial.h>
//#include "RoboClaw.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
  

uint16_t BNO055_SAMPLERATE_DELAY_MS = 20;  
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

/* Include the SPI library for the arduino boards */
#include <SPI.h>

//See limitations of Arduino SoftwareSerial
//SoftwareSerial serial(10,11);  
//RoboClaw roboclaw(&serial,10000);




/* SPI commands */
#define AMT22_NOP       0x00
#define AMT22_RESET     0x60
#define AMT22_ZERO      0x70

/* Define special ascii characters */
#define NEWLINE         0x0A
#define TAB             0x09

/* We will use these define macros so we can write code once compatible with 12 or 14 bit encoders */
#define RES12           12
#define RES14           14

/* SPI pins */
#define ENC_1            40
#define ENC_2            41
#define ENC_3            42
#define ENC_4            43
#define ENC_5            44
#define ENC_6            45


// #define ENC_1            48
#define SPI_MOSI        51
#define SPI_MISO        50
#define SPI_SCLK        52



#include "HX711.h"
#define CLK1  22
#define DOUT1  23
#define CLK2  24
#define DOUT2  25
#define CLK3  26
#define DOUT3  27
int movval=0;
int opcval=0;
int tpcval=0;
float str1=0;
float str2=0;
float str3=0;
int myspd=0;
int buttval=0;

//Roboclaw speed between 0 and 127 
const int buttjpin=4;
const int butt1pin=7;
const int butt2pin=6;
const int butt3pin=5;
int xval1;
int yval1;

int butt1State1=0;
int butt2State1=0;
int butt3State1=0;
int buttjState1=0;



HX711 scale1;
HX711 scale2;
HX711 scale3;
float calibration_factor1 = -21900;
float calibration_factor2 = -21560;
float calibration_factor3 = -22090;


float ytilt=0.0;
float ztilt=0.0;
float w=0.0;
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


uint16_t encoderPosition1;
uint16_t encoderPosition2;
uint16_t encoderPosition3;
uint16_t encoderPosition4;
uint16_t encoderPosition5;
uint16_t encoderPosition6;



float tickdeg=(360/16384);
float encoderPhi1=0;
float encoderPhi2=0;
float encoderPhi3=0;
float encoderPhi4=0;
float encoderPhi5=0;
float encoderPhi6=0;



char val = 's';
char rx_byte = 0;
char rx_byte2=0;
String readString;



const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];        // temporary array for use when parsing

      // variables to hold the parsed data
char messageFromPC[numChars] = {0};
//int MotCmd1 = 64;//. The range of values of still 0-127, however now 0 indicates full backwards speed, 
//int MotCmd2 = 64;//64 in the middle indicates stop, and 127 indicates full forwards speed.

boolean newData = false;



void setup() {
  
  Serial.begin(115200);
//  Serial.println("serb");
//  //while(!Serial.available() ){}
//
  pinMode(SPI_SCLK, OUTPUT);
  pinMode(SPI_MOSI, OUTPUT);
  pinMode(SPI_MISO, INPUT);
  pinMode(ENC_1, OUTPUT);
  pinMode(ENC_2, OUTPUT);
  pinMode(ENC_3, OUTPUT);
  pinMode(ENC_4, OUTPUT);
  pinMode(ENC_5, OUTPUT);
  pinMode(ENC_6, OUTPUT);

//
//
  digitalWrite(ENC_1, HIGH);
  digitalWrite(ENC_2, HIGH); 
  digitalWrite(ENC_3, HIGH); 
  digitalWrite(ENC_4, HIGH);  
  digitalWrite(ENC_5, HIGH); 
  digitalWrite(ENC_6, HIGH);


//  //set the clockrate. Uno clock rate is 16Mhz, divider of 32 gives 500 kHz.
  SPI.setClockDivider(SPI_CLOCK_DIV32);    // 500 kHz
//
//  //start SPI bus
  SPI.begin();

   //set the zero position for encoders
  setZeroSPI(ENC_1);
  setZeroSPI(ENC_2);
  setZeroSPI(ENC_3);
  setZeroSPI(ENC_4);
  setZeroSPI(ENC_5);
  setZeroSPI(ENC_6);
  delay(500);
// /Serial.println("encoders");
Serial.flush();

Serial.println(" strain");

pinMode(butt1pin,INPUT);
  pinMode(butt2pin,INPUT);
  pinMode(butt3pin,INPUT);
  pinMode(buttjpin,INPUT);
  pinMode(A1,INPUT); //joystick
  pinMode(A0,INPUT);
  

  scale1.begin(DOUT1, CLK1);
  scale1.set_scale(calibration_factor1); //This value is obtained by using the SparkFun_HX711_Calibration sketch
  scale1.tare(); //Assuming there is no weight on the scale at start up, reset the scale to 0
  
  scale2.begin(DOUT2, CLK2);
  scale2.set_scale(calibration_factor2); //This value is obtained by using the SparkFun_HX711_Calibration sketch
  scale2.tare(); //Assuming there is no weight on the scale at start up, reset the scale to 0
  
  scale3.begin(DOUT3, CLK3);
  scale3.set_scale(calibration_factor3); //This value is obtained by using the SparkFun_HX711_Calibration sketch
  scale3.tare(); //Assuming there is no weight on the scale at start up, reset the scale to 0

//



  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  delay(1000);
    
  bno.setExtCrystalUse(true);
//Serial.println(" imu");



}




void loop() {

  
  butt1State1=digitalRead(butt1pin);
  butt2State1=digitalRead(butt2pin);
  butt3State1=digitalRead(butt3pin);
  buttjState1=digitalRead(buttjpin);
  xval1=analogRead(A1);
  yval1=analogRead(A0);
  
  sensors_event_t event; 
  bno.getEvent(&event);
  imu::Quaternion quat = bno.getQuat();
  w=quat.w();
  qx=quat.y();
  qy=quat.x();
  qz=quat.z();
ytilt=event.orientation.y;
ztilt=event.orientation.z;

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

  str1=scale1.get_units(); //scale.get_units() returns a float
  str2=scale2.get_units();//You can change this to kg but you'll need to refactor the calibration_factor  
  str3=scale3.get_units();//You can change this to kg but you'll need to refactor the calibration_factor  



  
  encoderPosition1 = getPositionSPI(ENC_1, RES14);
  encoderPhi1= ((encoderPosition1)/16384.0)*360.0;

    encoderPosition2 = getPositionSPI(ENC_2, RES14);
  encoderPhi2= ((encoderPosition1)/16384.0)*360.0;
//  
    encoderPosition3 = getPositionSPI(ENC_3, RES14);
  encoderPhi3= ((encoderPosition1)/16384.0)*360.0;

  encoderPosition4 = getPositionSPI(ENC_4, RES14);
  encoderPhi4= ((encoderPosition1)/16384.0)*360.0;

    encoderPosition5 = getPositionSPI(ENC_5, RES14);
  encoderPhi5= ((encoderPosition1)/16384.0)*360.0;

    encoderPosition6 = getPositionSPI(ENC_6, RES14);
  encoderPhi6= ((encoderPosition1)/16384.0)*360.0;
  
//    

PrintAllData();  

  
}



void PrintAllData()
{ 
//
    Serial.print(xval1);
  Serial.flush();
  Serial.print(';');
  Serial.flush();
  Serial.print(yval1);
  Serial.flush();
  Serial.print(';');
  Serial.flush();
  Serial.print(buttjState1);
  Serial.flush();
  Serial.print(';');
  Serial.flush();
  Serial.print(butt1State1);
  Serial.flush();
  Serial.print(';');
  Serial.flush();
  Serial.print(butt2State1);
  Serial.flush();
  Serial.print(';');
  Serial.flush();
  Serial.print(butt3State1);
  Serial.flush();
  Serial.print(';');
  Serial.print(str1,3);
  Serial.flush();
  Serial.print(';');
  Serial.flush();
  Serial.print(str2,3);
  Serial.flush();
  Serial.print(';');
  Serial.flush();
  Serial.print(str3,3);
  Serial.flush();
  Serial.print(';');
  Serial.flush();
  Serial.print(ytilt, 4);
  Serial.flush();
  Serial.print(';');
  Serial.flush();
  Serial.print(ztilt, 4);
  Serial.flush();
  Serial.print(';');
  
  Serial.print(w, 4);
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
  Serial.print(';');
  Serial.flush();
  Serial.print(encoderPosition1);
  Serial.flush();
  Serial.print(';');
  Serial.flush();
  Serial.print(encoderPosition2);
  Serial.flush();
  Serial.print (';');
  Serial.flush();
  Serial.print(encoderPosition3);
  Serial.flush();
  Serial.print(';');
  Serial.flush();
  Serial.print(encoderPosition4);
  Serial.flush();
  Serial.print(';');
  Serial.flush();
  Serial.print(encoderPosition5);
  Serial.flush();
  Serial.print(';');
  Serial.flush();
  Serial.println(encoderPosition6);
  Serial.flush();  

  
//  Serial.println(" ");

  delay(30);
  
  }




/*
 * This function gets the absolute position from the AMT22 encoder using the SPI bus. The AMT22 position includes 2 checkbits to use
 * for position verification. Both 12-bit and 14-bit encoders transfer position via two bytes, giving 16-bits regardless of resolution.
 * For 12-bit encoders the position is left-shifted two bits, leaving the right two bits as zeros. This gives the impression that the encoder
 * is actually sending 14-bits, when it is actually sending 12-bit values, where every number is multiplied by 4. 
 * This function takes the pin number of the desired device as an input
 * This funciton expects res12 or res14 to properly format position responses.
 * Error values are returned as 0xFFFF
 */
uint16_t getPositionSPI(uint8_t encoder, uint8_t resolution)
{
  uint16_t currentPosition;       //16-bit response from encoder
  bool binaryArray[16];           //after receiving the position we will populate this array and use it for calculating the checksum

  //get first byte which is the high byte, shift it 8 bits. don't release line for the first byte
  currentPosition = spiWriteRead(AMT22_NOP, encoder, false) << 8;   

  //this is the time required between bytes as specified in the datasheet.
  //We will implement that time delay here, however the arduino is not the fastest device so the delay
  //is likely inherantly there already
  delayMicroseconds(3);

  //OR the low byte with the currentPosition variable. release line after second byte
  currentPosition |= spiWriteRead(AMT22_NOP, encoder, true);        

  //run through the 16 bits of position and put each bit into a slot in the array so we can do the checksum calculation
  for(int i = 0; i < 16; i++) binaryArray[i] = (0x01) & (currentPosition >> (i));

  //using the equation on the datasheet we can calculate the checksums and then make sure they match what the encoder sent
  if ((binaryArray[15] == !(binaryArray[13] ^ binaryArray[11] ^ binaryArray[9] ^ binaryArray[7] ^ binaryArray[5] ^ binaryArray[3] ^ binaryArray[1]))
          && (binaryArray[14] == !(binaryArray[12] ^ binaryArray[10] ^ binaryArray[8] ^ binaryArray[6] ^ binaryArray[4] ^ binaryArray[2] ^ binaryArray[0])))
    {
      //we got back a good position, so just mask away the checkbits
      currentPosition &= 0x3FFF;
    }
  else
  {
    currentPosition = 0xFFFF; //bad position
  }

  //If the resolution is 12-bits, and wasn't 0xFFFF, then shift position, otherwise do nothing
  if ((resolution == RES12) && (currentPosition != 0xFFFF)) currentPosition = currentPosition >> 2;

  return currentPosition;
}

/*
 * This function does the SPI transfer. sendByte is the byte to transmit. 
 * Use releaseLine to let the spiWriteRead function know if it should release
 * the chip select line after transfer.  
 * This function takes the pin number of the desired device as an input
 * The received data is returned.
 */
uint8_t spiWriteRead(uint8_t sendByte, uint8_t encoder, uint8_t releaseLine)
{
  //holder for the received over SPI
  uint8_t data;

  //set cs low, cs may already be low but there's no issue calling it again except for extra time
  setCSLine(encoder ,LOW);

  //There is a minimum time requirement after CS goes low before data can be clocked out of the encoder.
  //We will implement that time delay here, however the arduino is not the fastest device so the delay
  //is likely inherantly there already
  delayMicroseconds(3);

  //send the command  
  data = SPI.transfer(sendByte);
  delayMicroseconds(3); //There is also a minimum time after clocking that CS should remain asserted before we release it
  setCSLine(encoder, releaseLine); //if releaseLine is high set it high else it stays low
  
  return data;
}

/*
 * This function sets the state of the SPI line. It isn't necessary but makes the code more readable than having digitalWrite everywhere 
 * This function takes the pin number of the desired device as an input
 */
void setCSLine (uint8_t encoder, uint8_t csLine)
{
  digitalWrite(encoder, csLine);
}

/*
 * The AMT22 bus allows for extended commands. The first byte is 0x00 like a normal position transfer, but the 
 * second byte is the command.  
 * This function takes the pin number of the desired device as an input
 */
void setZeroSPI(uint8_t encoder)
{
  spiWriteRead(AMT22_NOP, encoder, false);

  //this is the time required between bytes as specified in the datasheet.
  //We will implement that time delay here, however the arduino is not the fastest device so the delay
  //is likely inherantly there already
  delayMicroseconds(3); 
  
  spiWriteRead(AMT22_ZERO, encoder, true);
  delay(250); //250 second delay to allow the encoder to reset
}

/*
 * The AMT22 bus allows for extended commands. The first byte is 0x00 like a normal position transfer, but the 
 * second byte is the command.  
 * This function takes the pin number of the desired device as an input
 */
void resetAMT22(uint8_t encoder)
{
  spiWriteRead(AMT22_NOP, encoder, false);

  //this is the time required between bytes as specified in the datasheet.
  //We will implement that time delay here, however the arduino is not the fastest device so the delay
  //is likely inherantly there already
  delayMicroseconds(3); 
  
  spiWriteRead(AMT22_RESET, encoder, true);
  
  delay(250); //250 second delay to allow the encoder to start back up
}





  void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;

    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

//============

//void parseData() {      // split the data into its parts
//
//    char * strtokIndx; // this is used by strtok() as an index
//
//    strtokIndx = strtok(tempChars,",");      // get the first part - the string
//    //strcpy(messageFromPC, strtokIndx); // copy it to messageFromPC
//    MotCmd1 = atoi(strtokIndx);
//    
//    //strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
//    //integerFromPC = atoi(strtokIndx);     // convert this part to an integer
//
//    strtokIndx = strtok(NULL, ",");
//    MotCmd2  = atoi(strtokIndx);     // convert this part to a float
//
//}

//============

//void showParsedData() {
////    Serial.print("Message ");
////    Serial.println(messageFromPC);
//    Serial.print("Integer ");
//    Serial.println(MotCmd1);
//    Serial.print("INterger2 ");
//    Serial.println(MotCmd2 );
//}




 
