#define GYRO_SAMPLE_RATE 0x01 //gyro output rate (200Hz / (1 + newRate))
#define PID_PITCH_INFLUENCE 40
#define PID_ROLL_INFLUENCE 40
#define PID_SAMPLE_TIME 10

#include <Servo.h>
#include <Wire.h>
#include "PinChangeInt.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "PID_v1.h"
#include <EEPROM.h>
#include <avr/wdt.h>
#include "Adafruit_GPS.h"
#include <SPI.h>
#include <SD.h>

//cs for sd-card
const int chipSelect = 4;

float PITCH_P_VAL = 11; //Pitch Pid Values
float PITCH_I_VAL = 9;
float PITCH_D_VAL = 9;

float ROLL_P_VAL = 18; //Roll Pid Values
float ROLL_I_VAL = 11;
float ROLL_D_VAL = 9;

bool lightOn = false;
unsigned long lastLightTimeStamp = 0;

volatile long rcThrottleDuration;
volatile unsigned long rcThrottleStart = 0;
volatile long rcStartDuration;
volatile unsigned long rcStartStart = 0;
volatile unsigned long rcPitchStart = 0;
volatile long rcPitchDuration;
volatile unsigned long rcRollStart = 0;
volatile long rcRollDuration;

Servo myMotor1;
Servo myMotor2;
Servo myMotor3;
Servo myMotor4;
double yawRegulator = 0.0f;
double baseSpeed = 0;
double pitchBalancer = 0.0f;
double rollBalancer = 0.0f;
double pitchBalancerSetpoint = 0.0;
double rollBalancerSetpoint = 0.0;
double yawSetpoint = 0.0;
int pitchValue;
float dmpPOffset, dmpROffset; //calibration offset

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
double ypr[3] = {0.0f, 0.0f, 0.0f};         // [yaw, pitch, roll]   yaw/pitch/roll container

MPU6050 gyro; //gyro object
uint8_t mpuIntStatus;   // Interrupt status byte from MPU
uint8_t devStatus;      // Device state (0 = success, !0 = error)
uint16_t packetSize;    // estimated DMP packet size, default = 42byte
uint16_t fifoCount;     // Aktuell verfügbare Bytes im FIFO Storage
uint8_t fifoBuffer[64]; // FIFO storage

//pid controllers
PID pitchReg(&ypr[1], &pitchBalancer, &pitchBalancerSetpoint, PITCH_P_VAL, PITCH_I_VAL, PITCH_D_VAL, REVERSE);
PID rollReg(&ypr[2], &rollBalancer, &rollBalancerSetpoint, ROLL_P_VAL, ROLL_I_VAL, ROLL_D_VAL, REVERSE);
PID yawReg(&ypr[0], &yawRegulator, &yawSetpoint, 15.0f, 13.0f, 10.0f, REVERSE);

Adafruit_GPS GPS(&Serial3);

bool systemReady = false;

File logFile;
bool sdReady = false;
unsigned long lastLogWrite = 0;

unsigned long lastOutput = 0;

void setup(){
  Serial.begin(9600);
  pinMode(47, OUTPUT); //Set debug LED to output
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if 8MHz CPU)
  
  //readPIDValues(); //read saved PID values
  initializeLight(); //init lights
  initializePID(); //init PID
  initializeGyro(); //init gyro MPU6050
  initializeMotors(); //init motors
  initializeRCInterrupts(); //register rc interrupts
  initializeGPS(); //inits the GPS module
  //initializeSD(); //inits the sd slot
  wdt_enable(WDTO_2S); //set watchdog timer to 2 secs
}

//inits the sd slot
void initializeSD(){
  sdReady = SD.begin(chipSelect);
}

//disables the GPS interrupt
void disableGPS(){
  TIMSK0 &= ~_BV(OCIE0A);
}

//initializes the GPS module
void initializeGPS(){
  GPS.begin(9600);
  // turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  
  //set update rate to 5Hz
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PMTK_API_SET_FIX_CTL_1HZ);
  
  //request antenna status updates
  GPS.sendCommand(PGCMD_ANTENNA);
  
  //enable interrupt on Timer0
  OCR0A = 0xAF;
  TIMSK0 |= _BV(OCIE0A);
  
  //waiting a second for initializing of the GPS module
  delay(1000);
}

//ISR for incoming GPS data
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  if (c) UDR0 = c; //writing the byte this way is much faster then serial.print
}

//read saved PID values from the EEPROM
void readPIDValues(){
  PITCH_P_VAL = EEPROM.read(0);
  PITCH_I_VAL = EEPROM.read(1);
  PITCH_D_VAL = EEPROM.read(2);
  ROLL_P_VAL = EEPROM.read(3);
  ROLL_I_VAL = EEPROM.read(4);
  ROLL_D_VAL = EEPROM.read(5);
  pitchReg.SetTunings(PITCH_P_VAL, PITCH_I_VAL, PITCH_D_VAL);
  rollReg.SetTunings(ROLL_P_VAL, ROLL_I_VAL, ROLL_D_VAL);
}

//ISR for throttle
void throttleISR(){
  if (!systemReady){
    return;
  }
  if (digitalRead(A8) == HIGH){
    rcThrottleStart = micros();
  }else{
    if (rcThrottleStart > 0){
      rcThrottleDuration = micros() - rcThrottleStart;
      //calculate new throttle value
      if (rcThrottleDuration <= 1040.0){
        rcThrottleDuration = 0.0;
      }else{
        rcThrottleDuration -= 1040.0;
      }
      if (rcThrottleDuration > 0.0){
        baseSpeed = (int)(120.0/(980.0/rcThrottleDuration));
      }else{
        baseSpeed = 0;
        //switch off
        systemReady = false;
        digitalWrite(47, LOW);
      }
    }
  }
}

//ISR for toggle on/off
void startISR(){
  if (digitalRead(A9) == HIGH){
    rcStartStart = micros();
  }else{
    if (rcStartStart > 0){
      rcStartDuration = micros() - rcStartStart;
      if (rcStartDuration - 2020 > -50 && rcStartDuration - 2020 < 50){
        digitalWrite(47, HIGH);
        systemReady = true;
      }else if(rcStartDuration - 1520 > -50 && rcStartDuration - 1520 < 50){
        //calibration
        //dmpPOffset += ypr[1];
        //dmpROffset += ypr[2];
        systemReady = false;
        
      }else{
        //switch off
        digitalWrite(47, LOW);
        systemReady = false;
      }
    }
  }
}

//ISR for rolling
void rollISR(){
  if (digitalRead(A11) == HIGH){
    rcRollStart = micros();
  }else{
    if (rcRollStart > 0){
      rcRollDuration = micros() - rcRollStart;
      if (rcRollDuration -1520.0 > -50 && rcRollDuration -1520.0 < 50){
        //quadrocopter doesn't have to roll right or left
        rollBalancerSetpoint = 0.0;
      }else {
        //rolling left or right
        double newSetpoint = 0.25 /(500.0 / (rcRollDuration - 1520.0));
        rollBalancerSetpoint = newSetpoint;
      }
     }
  }
}

//ISR for pitching
void pitchISR(){
  if (digitalRead(A10) == HIGH){
    rcPitchStart = micros();
  }else{
    if (rcPitchStart > 0){
      rcPitchDuration = micros() - rcPitchStart;
      if (rcPitchDuration -1520.0 > -50 && rcPitchDuration -1520.0 < 50){
        //quadrocopter doesn't have to pitch up or down
        pitchBalancerSetpoint = 0.0;
      }else {
        //pitching down or up
        double newSetpoint = -0.25 /(500.0 / (rcPitchDuration - 1520.0));
        pitchBalancerSetpoint = newSetpoint;
      }
     }
  }
}

//ISRs für RC bekannt machen
void initializeRCInterrupts(){
  pinMode(A8, INPUT);
  pinMode(A9, INPUT);
  pinMode(A10, INPUT);
  pinMode(A11, INPUT);
  attachPinChangeInterrupt(A8,throttleISR,CHANGE); //Throttle ISR
  attachPinChangeInterrupt(A9,startISR,CHANGE); //Throttle ISR
  attachPinChangeInterrupt(A10,pitchISR,CHANGE); //Pitch ISR
  attachPinChangeInterrupt(A11,rollISR,CHANGE); //Roll ISR
}

//init motors and stop them
void initializeMotors () {
  myMotor1.attach(8); //Motor1 on Pin 8
  myMotor1.write(0);

  myMotor2.attach(10); //Motor2 on Pin 10
  myMotor2.write(0);

  myMotor3.attach(5); //Motor3 on Pin 5
  myMotor3.write(0);

  myMotor4.attach(7); //Motor4 on Pin 7
  myMotor4.write(0);
}

//init Gyro MPU6050
void initializeGyro() {
  gyro.initialize();
  //init DMP
  devStatus = gyro.dmpInitialize();

  //reset calibration
  dmpPOffset = 0.0f;
  dmpROffset = 0.0f;


  //just go on if initialization succeeded
  if (devStatus == 0) {
    //turn on DMP
    gyro.setDMPEnabled(true);
  
    mpuIntStatus = gyro.getIntStatus();

    // read estimated packet size
    packetSize = gyro.dmpGetFIFOPacketSize();

  }
  else {
    // ERROR-Handling ;)
    
  }
}

//init light on PIN 9
void initializeLight(){
  pinMode(9, OUTPUT);
  digitalWrite(9, HIGH); //Licht einschalten
  lightOn = true;
  lastLightTimeStamp = millis(); //set current Timestamp
}

//init PID
void initializePID() {
  pitchReg.SetSampleTime(PID_SAMPLE_TIME);
  pitchReg.SetMode(MANUAL); //turn off controller
  pitchReg.SetOutputLimits(-PID_PITCH_INFLUENCE, PID_PITCH_INFLUENCE);

  rollReg.SetSampleTime(PID_SAMPLE_TIME);
  rollReg.SetMode(MANUAL);
  rollReg.SetOutputLimits(-PID_ROLL_INFLUENCE, PID_ROLL_INFLUENCE);
  
  yawReg.SetSampleTime(PID_SAMPLE_TIME);
  yawReg.SetMode(MANUAL);
  yawReg.SetOutputLimits(-10, 10);
}

void loop(){
  //check light state
  checkLight();
  
  //check states
  checkStates();
  
  //do the balancing (flying)
  holdBalance();
  
  //read values from the gyro
  readDMP();
  
  //read serial commands
  readCommand();
  
  //write log if necessary
  //writeLog();
  
  //reset wdt
  wdt_reset();
}

//write log if necessary
void writeLog(){
  if (!sdReady){
    return;
  }
  if (millis() - lastLogWrite >= 1000){
    File dataFile = SD.open("log.txt", FILE_WRITE);
    if(!dataFile){
      return;
    }
    byte byteBuffer[12];
    union doubleMemoryDivider {
      double value;
      struct {
        byte b1;
        byte b2;
        byte b3;
        byte b4;
      }bytes;
    };
    union doubleMemoryDivider memUnion;
    memUnion.value = ypr[0];
    byteBuffer[0] = memUnion.bytes.b1;
    byteBuffer[1] = memUnion.bytes.b2;
    byteBuffer[2] = memUnion.bytes.b3;
    byteBuffer[3] = memUnion.bytes.b4;
    memUnion.value = ypr[1];
    byteBuffer[4] = memUnion.bytes.b1;
    byteBuffer[5] = memUnion.bytes.b2;
    byteBuffer[6] = memUnion.bytes.b3;
    byteBuffer[7] = memUnion.bytes.b4;
    memUnion.value = ypr[2];
    byteBuffer[8] = memUnion.bytes.b1;
    byteBuffer[9] = memUnion.bytes.b2;
    byteBuffer[10] = memUnion.bytes.b3;
    byteBuffer[11] = memUnion.bytes.b4;
    dataFile.write(byteBuffer, 12);
    dataFile.close();
    lastLogWrite = millis();
  }
}

//read and interpret a serial command, if available
void readCommand(){
  if (Serial.available()){
    byte bytesCount = Serial.read();
    byte data[bytesCount];
    //read all remaining bytes
    for (int i = 0; i < bytesCount; i++){
      byte currentByte = -1;
      while(!Serial.available()){
      }
      currentByte = Serial.read();
      data[i] = currentByte;
    }
    //interpret the command now (data[0] is the command type)
    if(data[0] == 0){ //set pid values
      PITCH_P_VAL = data[1];
      PITCH_I_VAL = data[2];
      PITCH_D_VAL = data[3];
      ROLL_P_VAL = data[4];
      ROLL_I_VAL = data[5];
      ROLL_D_VAL = data[6];
      pitchReg.SetTunings(PITCH_P_VAL, PITCH_I_VAL, PITCH_D_VAL);
      rollReg.SetTunings(ROLL_P_VAL, ROLL_I_VAL, ROLL_D_VAL);
      //write data to eeprom
      for (int i= 0; i < bytesCount - 1; i++){
        EEPROM.write(i, data[i+1]);
      }
    }
  }
}

void readDMP() { //read DMP when possible
  //read FIFO size
  fifoCount = gyro.getFIFOCount();
  if (fifoCount < packetSize){
    return;
  }
  

  mpuIntStatus = gyro.getIntStatus();
  //check for overflow
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    //reset, if overflow happened
    gyro.resetFIFO();
    return;
  }
  else if (mpuIntStatus & 0x02) {
    //reset MPU6050 if the data size is not valid
    if (fifoCount > 0 && fifoCount % packetSize != 0)
    {
      gyro.resetFIFO();
      return;
    }
    //read one packet from the FIFO
    gyro.getFIFOBytes(fifoBuffer, packetSize);

    fifoCount -= packetSize;

    //read raw accel values
    gyro.dmpGetAccel(&aa, fifoBuffer);

    //eulerangle ypr
    gyro.dmpGetQuaternion(&q, fifoBuffer);
    gyro.dmpGetGravity(&gravity, &q);
    gyro.dmpGetYawPitchRoll((float*)ypr, &q, &gravity);
    //change pitch and roll
    float pitchTemp = ypr[1];
    ypr[1] = ypr[2];
    ypr[2] = pitchTemp;
    
    //clean the values with the calibration values
    ypr[1] -= dmpPOffset;
    ypr[2] -= dmpROffset;
    
  }

}


void holdBalance() {
  //pitch < 0 front goes down
  //roll < 0 right side goes down
  
  //Yaw to right => yawRegulator gets bigger
  //Yaw to left => yawRegulator gets smaller
  
  //check if system is ready to fly
  if (!systemReady || baseSpeed == 0) {
    return;
  }

  //compute new PID values
  pitchReg.Compute();
  rollReg.Compute();
  yawReg.Compute();
  
  //set motors to a new calculated value
  myMotor1.write((baseSpeed - yawRegulator) - pitchBalancer - rollBalancer);
  myMotor2.write((baseSpeed + yawRegulator) - pitchBalancer + rollBalancer);
  myMotor3.write((baseSpeed - yawRegulator) + pitchBalancer + rollBalancer);
  myMotor4.write((baseSpeed + yawRegulator) + pitchBalancer - rollBalancer);
}

void checkStates(){
  //if system is not ready, turn off all components
  if (!systemReady){
    pitchReg.SetMode(MANUAL);
    rollReg.SetMode(MANUAL);
    yawReg.SetMode(MANUAL);
    myMotor1.write(0);
    myMotor2.write(0);
    myMotor3.write(0);
    myMotor4.write(0);
    baseSpeed = 0;
  }else{
    pitchReg.SetMode(AUTOMATIC);
    rollReg.SetMode(AUTOMATIC);
    yawReg.SetMode(AUTOMATIC);
  }
}

//check light state and, if necessary, turn it on or off
void checkLight(){
  if (lightOn){ //light is turned on
    if (millis() - lastLightTimeStamp >= 70){ //turn off light after 100ms
      digitalWrite(9, LOW);
      lastLightTimeStamp = millis();
      lightOn = false;
    }
  }else{ //Licht ist ausgeschaltet
    if (millis() - lastLightTimeStamp >= 800){ //turn on light every second
      digitalWrite(9, HIGH);
      lastLightTimeStamp = millis();
      lightOn = true;
    }
  }
}
