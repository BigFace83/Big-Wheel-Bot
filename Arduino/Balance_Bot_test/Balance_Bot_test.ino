#include <PID_v1.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

#define enA 9
#define enB 10
#define in1R 6
#define in2R 5
#define in1L 8
#define in2L 7

int HeartBeat = 0;

bool LeftForward = true;
bool RightForward = true;

#define BUFFERSIZE 64
char databuffer[BUFFERSIZE];
int serialcounter = 0;

const int DBand = 2;

unsigned long previousMillis = 0;

volatile int XCommand = 0; //Speed reference for left and right motors
volatile int YCommand = 0;

//Define Variables for PID controller
double PitchSetpoint, PitchInput, SpeedOutput;

MPU6050 mpu;

#define SonarSensorLeft A1
#define SonarSensorRight A0

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

int Rollangle;
int Pitchangle;
int Yawangle;
int Temperature;

int Rightsonar;
int Leftsonar;

int ax, ay, az;
int gx, gy, gz;

char data_packet[20];


//Define Variables we'll be connecting to
double LeftSetpoint, LeftInput, LeftOutput;
double RightSetpoint, RightInput, RightOutput;

/*
//Specify the links and initial tuning parameters
PID LeftPID(&LeftInput, &LeftOutput, &LeftSetpoint,0.5,2.5,0.03, DIRECT);
PID RightPID(&RightInput, &RightOutput, &RightSetpoint,0.5,2.5,0.03, DIRECT);
*/
PID BalancePID(&PitchInput, &SpeedOutput, &PitchSetpoint,8,0.5,0.05, DIRECT);


void setup() {
  Serial.begin(115200);
  TCCR1B &= ~7;
  TCCR1B |= 2; 
  pinMode(enA, OUTPUT);
  pinMode(in1R, OUTPUT);
  pinMode(in2R, OUTPUT);

  pinMode(enB, OUTPUT);
  pinMode(in1L, OUTPUT);
  pinMode(in2L, OUTPUT);

  // Set initial rotation direction
  digitalWrite(in1R, HIGH);
  digitalWrite(in2R, HIGH);
  digitalWrite(enA, HIGH);
  
  digitalWrite(in1L, HIGH);
  digitalWrite(in2L, HIGH);
  digitalWrite(enB, HIGH);


  /*
  LeftInput = Lencoder;
  LeftSetpoint = 0;
  //turn the PID on
  LeftPID.SetSampleTime(200);
  LeftPID.SetMode(AUTOMATIC);

  RightInput = Rencoder;
  RightSetpoint = 0;
  //turn the PID on
  RightPID.SetSampleTime(200);
  RightPID.SetMode(AUTOMATIC);
  */

  PitchInput = Pitchangle;
  PitchSetpoint = 0;
  //turn the PID on
  BalancePID.SetSampleTime(100);
  BalancePID.SetMode(AUTOMATIC);
  BalancePID.SetOutputLimits(-150, 150);

  

   Wire.begin();
    TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    
    mpu.initialize();
    devStatus = mpu.dmpInitialize();
    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(-135);
    mpu.setYGyroOffset(77);
    mpu.setZGyroOffset(-147);
    mpu.setZAccelOffset(1466); // 1688 factory default for my test chip
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        mpu.setDMPEnabled(true);

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    }
}

void loop() {

  GetMPUdata();
    
    //keep an eye out for serial data
    while(Serial.available() > 0)  // if something is available from serial port
    { 
        char c=Serial.read();      // get it
        databuffer[serialcounter] = c; //add it to the data buffer
        serialcounter++;
        if(c=='\n'){  //newline character denotes end of message            
            serialcounter = 0; //reset serialcounter ready for the next message
            sscanf (databuffer,"%d,%d",&XCommand, &YCommand);
            memset(databuffer, 0, sizeof(databuffer));//clear data buffer when command has been received
            HeartBeat = 0;
            break;
        }           
    }


  /*
  if (LeftWheel >= DBand) {
    digitalWrite(in1L, HIGH);
    digitalWrite(in2L, LOW);
    LeftForward = true;
  }
  else if (LeftWheel <= -DBand) {
    digitalWrite(in1L, LOW);
    digitalWrite(in2L, HIGH);
    LeftForward = false;
  }
  else{ //Brake
      digitalWrite(in1L, HIGH);
      digitalWrite(in2L, HIGH);
      digitalWrite(enB, HIGH);
  }

  if (RightWheel >= DBand) {
    digitalWrite(in1R, HIGH);
    digitalWrite(in2R, LOW);
    RightForward = true;
  }
  else if (RightWheel <= -DBand) {
    digitalWrite(in1R, LOW);
    digitalWrite(in2R, HIGH);
    RightForward = false;
  }
  else{ //Brake
      digitalWrite(in1R, HIGH);
      digitalWrite(in2R, HIGH);
      digitalWrite(enA, HIGH);
  }

  if(LeftWheel >= DBand || LeftWheel < -DBand){
  LeftInput = Lencoder;
  if(LeftPID.Compute()){
    Lencoder = 0;
    analogWrite(enB,LeftOutput);

  }
  }

  if(RightWheel >= DBand || RightWheel < -DBand){
  RightInput = Rencoder;
  if(RightPID.Compute()){
    Rencoder = 0;
    analogWrite(enA,RightOutput); 
 
  }
  }
  */

  PitchInput = Pitchangle;
  if(BalancePID.Compute()){

    
    Serial.print(PitchInput);
    Serial.print(",");
    Serial.println(SpeedOutput);
    
  //set wheel directions
  if (SpeedOutput >= 5){ //Motors forward
    digitalWrite(in1L, HIGH);
    digitalWrite(in2L, LOW);
    digitalWrite(in1R, HIGH);
    digitalWrite(in2R, LOW);
    analogWrite(enA,abs(SpeedOutput)+105); 
    analogWrite(enB,abs(SpeedOutput)+105);

  }
  else if (SpeedOutput <= -5){ //Motors reverse
    digitalWrite(in1L, LOW);
    digitalWrite(in2L, HIGH);
    digitalWrite(in1R, LOW);
    digitalWrite(in2R, HIGH);
    analogWrite(enA,abs(SpeedOutput)+105); 
    analogWrite(enB,abs(SpeedOutput)+105);

  }
  else {//Brake
    digitalWrite(in1L, HIGH);
    digitalWrite(in2L, HIGH);
    digitalWrite(enB, HIGH);
    digitalWrite(in1R, HIGH);
    digitalWrite(in2R, HIGH);
    digitalWrite(enA, HIGH);

    analogWrite(enA,0); 
    analogWrite(enB,0);

  }
  

     

    
  }


  

  int Leftsonar = ReadSonar(SonarSensorLeft);
  int Rightsonar = ReadSonar(SonarSensorRight);

  unsigned long currentMillis = millis();
        
  if(currentMillis - previousMillis > 300) //use this to determine frequency of sending data
  {
      previousMillis = currentMillis;
  
      //sprintf(data_packet, "%d,%d",Pitchangle, SpeedOutput);
      //Serial.println(data_packet);
     
  }      
}


int ReadSonar(int Pin) //Read sonar sensor
{
  
  //read head sonar sensor 
  pinMode(Pin, OUTPUT);
  digitalWrite(Pin, LOW);             // Make sure pin is low before sending a short high to trigger ranging
  delayMicroseconds(2);
  digitalWrite(Pin, HIGH);            // Send a short 10 microsecond high burst on pin to start ranging
  delayMicroseconds(10);
  digitalWrite(Pin, LOW);             // Send pin low again before waiting for pulse back in
  pinMode(Pin, INPUT);
  int duration = pulseIn(Pin, HIGH);  // Reads echo pulse in from SRF05 in micro seconds
  int sonardist = duration/58;      // Dividing this by 58 gives us a distance in cm
  
  return sonardist;
  
}


void GetMPUdata(){
   

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();
   
    // check for overflow (this should never happen unless our code is too inefficient)
    if (fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else{// if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
      
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        //Temperature = (mpu.getTemperature()/340.00+36.53);
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        Yawangle = (ypr[0] * 180/M_PI);
        Pitchangle = (ypr[1] * 180/M_PI);
        Rollangle = (ypr[2] * 180/M_PI);
        
        
    }
}



