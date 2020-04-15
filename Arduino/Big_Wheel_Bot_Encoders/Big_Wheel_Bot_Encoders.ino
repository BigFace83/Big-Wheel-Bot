#include <PID_v1.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include "Servo.h"

#define enA 5
#define enB 6
#define in1R 7
#define in2R 4
#define in1L 9
#define in2L 8

int Lencoder = 0;
int Lencodertotal = 0;
int Rencoder = 0;
int Rencodertotal = 0;

int Lencdebug = 0;
int Rencdebug = 0;

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

volatile int XArm = 0; //Arm command from transmitter
volatile int YArm = 0;

MPU6050 mpu;

#define SonarFrontPin 10
#define SonarRearPin 11

#define LeftIRPin A2
#define RightIRPin A3

#define Serv1Pin A6
#define Serv2Pin A7




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

int ax, ay, az;
int gx, gy, gz;

char data_packet[50];

int Arm1Pos = 178;
int Arm2Pos = 178;

/**************************************
Interrupt handlers for encoders
**************************************/
void incrementL()
{
  Lencoder ++;
  //Lencdebug ++;
  if (LeftForward) Lencodertotal ++;
  else Lencodertotal --;
}

void incrementR()
{
  Rencoder ++;
  //Rencdebug ++;
  if (RightForward) Rencodertotal ++;
  else Rencodertotal --;
}

//Define Variables we'll be connecting to
double LeftSetpoint, LeftInput, LeftOutput;
double RightSetpoint, RightInput, RightOutput;

//Specify the links and initial tuning parameters
PID LeftPID(&LeftInput, &LeftOutput, &LeftSetpoint,4.0,3.0,0.75, DIRECT);
PID RightPID(&RightInput, &RightOutput, &RightSetpoint,4.0,3.0,0.75, DIRECT);

Servo Arm1;
Servo Arm2;

void setup() {
  Serial.begin(115200);
  //TCCR1B &= ~7;
  //TCCR1B |= 2; 
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

  attachInterrupt(0, incrementR, CHANGE);
  attachInterrupt(1, incrementL, CHANGE);

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

  Serial.println("Setup complete");

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

    Arm1.attach(12);
    Arm2.attach(13);

    Arm1.write(Arm1Pos);
    Arm2.write(Arm2Pos); 
}

void loop() {

  HeartBeat = 0;
  GetMPUdata();
    
    //keep an eye out for serial data
    while(Serial.available() > 0)  // if something is available from serial port
    { 
        char c=Serial.read();      // get it
        databuffer[serialcounter] = c; //add it to the data buffer
        serialcounter++;
        if(c=='\n'){  //newline character denotes end of message            
            serialcounter = 0; //reset serialcounter ready for the next message
            sscanf (databuffer,"%d,%d,%d,%d",&XCommand, &YCommand,&XArm, &YArm);
            memset(databuffer, 0, sizeof(databuffer));//clear data buffer when command has been received
            HeartBeat = 0;
            break;
        }           
    }


  //Update arm position based on serial input. 
  if (XArm > 100){ //Deadband
    Arm1Pos = Arm1Pos + 2;
  }
  else if (XArm < -100){
    Arm1Pos = Arm1Pos - 2;
  }
  Arm1Pos = constrain(Arm1Pos, 0 ,180);
  Arm1.write(Arm1Pos);
  
  if (YArm > 100){
    Arm2Pos = Arm2Pos - 2;
  }
  else if (YArm < -100){
    Arm2Pos = Arm2Pos + 2;
  }
  Arm2Pos = constrain(Arm2Pos, 0 ,180);
  Arm2.write(Arm2Pos); 


  

  

  //Calculate left and right wheel speeds from joystick commands
  int INVXCommand = -XCommand; //Invert x command and scale steering input

  int V = ((255-abs(INVXCommand))*(float(YCommand)/255)) + YCommand;
  int W = ((255-abs(YCommand))*(float(INVXCommand)/255)) + INVXCommand;
  
  int RightWheel = (V+W)/2;
  int LeftWheel = (V-W)/2;

  //Map -255 to 255 to wheel pulse counts
  LeftSetpoint = map(abs(LeftWheel), 0, 255, 0, 30);
  RightSetpoint = map(abs(RightWheel), 0, 255, 0, 30);
  //set wheel directions based on wheel speed references calculated above

  if(HeartBeat > 20){
    LeftSetpoint = 0;
    RightSetpoint = 0;
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



  unsigned long currentMillis = millis();
        
  if(currentMillis - previousMillis > 300) //use this to determine frequency of sending data
  {
      previousMillis = currentMillis;

      int Frontsonar = ReadSonar(SonarFrontPin);
      int Rearsonar = ReadSonar(SonarRearPin);
      int LeftIRcm;
      int RightIRcm;

      int LeftIR = analogRead(LeftIRPin);
      if (LeftIR > 3){
          LeftIRcm = (6787.0 / (LeftIR-3)) - 4.0; //Calculate distance in cm
          LeftIRcm = constrain(LeftIRcm, 0, 60);
      }
      else{ 
          LeftIRcm = 60;
      }
      
      int RightIR = analogRead(RightIRPin);
      if (RightIR > 3){
          RightIRcm = (6787.0 / (RightIR-3)) - 4.0; //Calculate distance in cm
          RightIRcm = constrain(RightIRcm, 0, 60);
      }
      else{
          RightIRcm = 60;
      }

      int Serv1= analogRead(Serv1Pin);
      int Serv2 = analogRead(Serv2Pin);

      
      sprintf(data_packet, "%d,%d,%d,%d,%d,%d,%d,%d,%d",Lencodertotal,Rencodertotal,Yawangle,Frontsonar,Rearsonar,LeftIRcm,RightIRcm,Serv1,Serv2);
      Lencodertotal = 0; //Reset encoder counts as soon as possible
      Rencodertotal = 0;
      Serial.println(data_packet);

      //Serial.println(LeftIR);
      //Serial.println(RightIR);
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



