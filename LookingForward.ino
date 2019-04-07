//UBC - IGEN 230 - Group 10 - LookingForward
//Spring 2019 - Presented April 4th/2019
//A proof-of-concept collision avoidance system implemented on a skateboard.
//Inputs:   Distance from LiDAR Lite V3, Angle from Tower Pro SG90 Servo, Yaw/Pitch/Roll from MPU6050's Gyro.
//Process:  Time stamped vectors pointing to nearby obstacles, with angle offsets (due to dynamic nature of a moving 
//          skateboard) from gyro stored; derivative of vectors * distance = seconds to impact (SOI) to nearest object
//          in current trajectory. SOI = 2.2s is set as the danger threshhold (based on research & experimention).
//Outputs:  Piezo Buzzer alarming the user, with frequency mapped to increase as SOI decreases.
//
//Link to final report: https://drive.google.com/open?id=18kKv8XsAvu9eewQbNph3zWeGGHRPOC1O
//---------------------------------------------------------------------------------------------------------------------

#include <SPI.h>
#include <Servo.h>
#include <Wire.h>
//https://github.com/garmin/LIDARLite_Arduino_Library/tree/master/src
#include <LIDARLite.h>
//https://www.i2cdevlib.com/
#include "I2Cdev.h"
//https://maker.pro/files/MPU6050.zip
#include "MPU6050_6Axis_MotionApps20.h"

//--MPU 6050 Definitions--
MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL
bool dmpReady = false;  // set true if DMP init_0 was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {mpuInterrupt = true;}
//---

//--ToDo: use pointers to operate data.
//        reduce functions to object "Obstacle"
//        unused code: 
//          @lines ~45-51, 65, 88-101, 105. (part of future plans) 
//          @lines ~114-150 (IMU resistor broke during presentation, commented to avoid faulty data.)

//class Obstacle{
//  public:
//    Obstacle(double crntT_1, byte vec_1);
//    void danger();
//  private:
//    int _pin;
//};

bool buzzAltBool;
unsigned long prevT_0, prevT_1;
Servo myServ;
byte servTheta = 60;
byte phi_0, phi_1, theta_0, theta_1;
bool servCCW = true;
bool init_0 = false;
bool newObj = true;
bool objLost = false;
LIDARLite myLid;
int loopCnt = 0;

//int p*;//unused. To be implement in later revisions to increase data processing efficiency.

int objWidth;
const unsigned int pi = 31416;

unsigned int distArr[1][120];//--------------
unsigned long thetaArr[1][120];
short crntAvgDist, prevAvgDist;//-----
short distAgg;

void setup(){
  buzzAltBool = true;
  distAgg = 0;
  pinMode(11, OUTPUT); //buzzer
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
  mpu.dmpInitialize();
  devStatus = mpu.dmpInitialize();
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
/*
  if (devStatus == 0) {
    //turn on the DMP, now that it's ready
    mpu.setDMPEnabled(true);
  
    //enable Arduino interrupt detection
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
  
    //set our DMP Ready flag so the main loop() function knows it's okay to use it
    dmpReady = true;
    
    //get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } 
*/
  for(int i = 0; i < 1; i++){for(int j = 0; j < 120;j++){distArr[i][j] = 0 ;}} //!! Dummy initing @ cap. MemoryTest.
  for(int i = 0; i < 1; i++){for(int j = 0; j < 120;j++){thetaArr[i][j] = 0 ;}}//!! Dummy initing @ cap. MemoryTest.
//memset(distArr,null,aa); //unused. To be implement in later revisions to increase data processing efficiency.
  myServ.attach(3);
  myServ.write(servTheta);
  myLid.begin(0,true); 

  prevT_0 = prevT_1 = 0;
}

void loop(){
/*  
  if (!dmpReady) return;
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
     // reset so we can continue cleanly
     mpu.resetFIFO();
    //Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  }

  else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
  
    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    
    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

// -- Print IMU Values --
//            Serial.print("ypr\t");
//            Serial.print(ypr[0] * 180/M_PI);
//            Serial.print("\t");
//            Serial.print(ypr[1] * 180/M_PI);
//            Serial.print("\t");
//            Serial.println(ypr[2] * 180/M_PI);
  }//end elseif
*/  

  unsigned long crntT = micros();
  
  

  distAgg =+ myLid.distance();
  loopCnt++;

//------------------Initializing--------------
  if((crntT - prevT_1) > 5){
    prevT_1 = crntT; 
    crntAvgDist = distAgg / loopCnt;
    distAgg = loopCnt = 0;
    init_0 = true;//initialized once, to get the first average distance
  }//- 

//----------------Servo & Angle Arguments-------
  if((servTheta <= 0) || (servTheta >= 120)){servCCW = !servCCW;} //switch rotation

  //perform every 2000 microseconds
  if((crntT - prevT_0) > 2000){
    prevT_0 = crntT;
    if(servCCW){servTheta++;}//-servo CCW rot fen
    else if(!servCCW){servTheta--;}//-servo CW rot fen
    if(servTheta <= 0){servTheta = 0;} //redundancy 
    if(servTheta >= 120){servTheta = 120;} //redundancy
    myServ.write(servTheta); //m0000ve
  }//-

//-----Distance & Danger Decision Arguments-----
  
  if(init_0){
    if((crntAvgDist < 1000) && newObj){
      prevAvgDist = crntAvgDist;
      newObj = false;
      phi_0 = phi();
      theta_0 = servTheta;
    }  
    else if((crntAvgDist < 1000) && !newObj){
      short distDiff = (prevAvgDist - crntAvgDist);
      if(abs(distDiff) > 50){
        phi_1 = phi();
        theta_1 = servTheta;
        newObj = true;
        int objCent = avg(phi_0,phi_1);
        int objAngCent = avg(theta_0,theta_1);
        for(int i = theta_0; i < theta_1; i++){
          //perform if old distance is longer than new distance (aka object is closer)
          if((distArr[0][i] - crntAvgDist) > 0){
            if((objAngCent > 45)&&(objAngCent < 75)){
              //thetaOffset(crntAvgDist,i);//call function to offset theta based on orientation
              isDanger(crntAvgDist,crntT,i);
            }
          }//obj has been here
          else{
            distArr[0][objAngCent] = crntAvgDist;
            thetaArr[0][objAngCent] = crntT;
          }
        } 
      }//end abs(distDiff) if
    }//end elseif
  }//end (init_0) if 
}//- End main loop

//offsets angle of IDed obstacle's vector, w.r.t. lean/orienation of skateboard.
void thetaOffset(int i, short avgDist){
  double rho = 50/(2*(sin(atan(tan(ypr[1])*sin(ypr[2])))));
  if(servCCW){thetaArr[0][i] =+ asin(avgDist/(2*rho));}
  else if(!servCCW){thetaArr[0][i] =- asin(avgDist/(2*rho));}
}

//to assign quadrants to servo location as defined by centerline @ 60degrees.
bool inFirstQuad(){
  if(servTheta >= 60){return false;}
  else if(servTheta < 60){return true;}    
}//-

//phi ToBe used later to makes it easier to use sin(phi) to get object width
//based on a 0 center line and +60deg to both left (second quadrant) and right (first quadrant) 
byte phi (){
  if(inFirstQuad){return(60-servTheta);}
  else{return(servTheta-60);}
}//-

void buzz(long secToImpact){analogWrite(11, map(secToImpact, 2200000, 0, 20, 150));}//to alarm user with piezo buzzer, based on seconds to impact & PWM limits of analogWrite.
void noBuzz(){analogWrite(11, 0);}//to piezo buzzer alarm off when not required.
byte avg(byte a, byte b){return ((a+b)/2);}

bool isDanger(short avgDist, long crntT, int i){
  // -- 1/v = 1/(dx/dt) [microseconds]/[meter]  --> 1/v * distance = [microseconds] to impact
  long secToImpact = ((((thetaArr[0][i]) - crntT) / (distArr[0][i] - avgDist)) * avgDist);
  if(secToImpact < 1){secToImpact = 0;}
  if(secToImpact < 2200000){
    if((millis() - crntT) > 100){buzzAltBool = !buzzAltBool;}
    if(buzzAltBool){buzz(secToImpact);} 
    return 1;//No current use for bool return. To be incorporated in later revision.
  }
  else {
    if((millis() - crntT) > 100){buzzAltBool = !buzzAltBool;}
    if(buzzAltBool){noBuzz();}   
    return 0; //No current use for bool return. To be incorporated in later revision.
  }
}
