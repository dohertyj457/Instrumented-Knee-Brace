#include <Arduino.h>

#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_MPU6050.h>

File myFile;
uint32_t myTime;
int SDPin = 10;
const int MPU_ADDR1 = 0b1101000; // IMU I2C - differentiated using the AD0 pin set HIGH or LOW
const int MPU_ADDR2 = 0b1101001; // (AD0 = 0, 1101000. AD0 = 1, 1101001)

//IMU1 = AD0 LOW, IMU2 = AD0 HIGH

//Establishing variables for values that will be recorded and stored
long allData [17];
//THE NUMBER ON THE VARIABLE CORRESPONDS TO THE SENSOR:
// t1= TIME IMU1 DATA WERE TAKEN
// t2 = TIME IMU 2 DATA WERE TAKEN
//t3 = TIME BOTH EMG SIGNALS WERE TAKEN
//allData[0]-allData[6]: t1, aX1, aY1, aZ1, gyroX1, gyroY1, gyroZ1
//allData[7]-allData[13]: t2, aX2, aY2, aZ2, gyroX2, gyroY2, gyroZ2
//allData[14]-allData[16]: t3, EMG1, EMG2

int gyroX; int gyroY; int gyroZ; int aX; int aY; int aZ;

//FUNCTIONS: I was getting errors without establishing functions here...
void setupMPU();
void setupSD();
void recordAccel1(int sensor);
void recordGyro1(int sensor);
void recordAccel2(int sensor);
void recordGyro2(int sensor);
void emgSig1(int vPin);
void emgSig2(int vPin);
void writeIMU();
void writeEMG();

//-----SETUP-----
void setup() {
  // put your setup code here, to run once:
  Serial.begin(500000);
  Wire.begin();
  setupMPU();
  setupSD();
}

//-----LOOP-----
void loop() {
  //Serial.println("Here we go");
  myTime = millis();
  allData[0] = myTime;
  //Serial.println("Recording from IMU1");
  recordAccel1(MPU_ADDR1);
  //Serial.println("Onto gyro");
  recordGyro1(MPU_ADDR1);
  allData[7] = myTime;
  //Serial.println("Recording from IMU2");
  recordAccel2(MPU_ADDR2);
  recordGyro2(MPU_ADDR2);
  allData[14] = myTime;
  //Serial.println("on to EMG");
  emgSig1(A1);
  emgSig2(A2);
  //Serial.println("EMG DONE");
  //Serial.println("We made it!");
  for(int i = 0; i < 17; i++){
      Serial.print(allData[i]);
      Serial.print(" ");
  }
  Serial.println();
  //void writeIMU();
  //writeEMG();
  delay(3);
}


//Functions called above_______________________________________________________________________

//SETUP FUNCTIONS__
void setRegister (int sens, int reg, int regset){
  // Begin coms with sensor, access required register, set desired values
  Wire.beginTransmission(sens); //I2C address
  Wire.write(reg); //Accesses register 
  Wire.write(regset); //Sets register
  Wire.endTransmission();
}

void setupMPU(){
  // Use setRegister function to wake up sensors and set data collection frequencies
  setRegister(MPU_ADDR1,0x6B,0b00000000); //Register 6B - Power Management - Wakes up IMU
  setRegister(MPU_ADDR2,0x6B,0b00000000);
  setRegister(MPU_ADDR1,0x1B,0x00000000); //Register 1B - Gyroscope - sets gyro outputs to +- 250 degrees per second
  setRegister(MPU_ADDR2,0x1B,0x00000000);
  setRegister(MPU_ADDR1,0x1C,0x00000000); //Register 1C - Accelerometer - sets accelerometer outputs to +-2g
  setRegister(MPU_ADDR2,0x1C,0x00000000);
}

void setupSD(){
  //Establishes connection with SD reader
  Serial.println("Attempting to set up SD card");
  while (!Serial) { // wait for serial port to connect. Needed for native USB port only
    }
  Serial.print("Initializing SD card...");
  if (!SD.begin(SDPin)) {
  Serial.println("initialization failed!");
  while (1);
  }
  Serial.println("initialization done."); 
}

//DATA COLLECTION FUNCTIONS___

void recordAccel1(int sensor){
  //Serial.println("Now I'm in here");
  //records accel values in x,y,z directions from one sensor
  
  Wire.beginTransmission(sensor); //I2C Address
  Wire.write(0x3B); //Register 3B - Accelerometer readings
  Wire.endTransmission();
  //Serial.println("Still working?");
  Wire.requestFrom(sensor,6); //Reads from registers 3B-40
  while(Wire.available() < 6);
  int aX = Wire.read()<<8|Wire.read(); //first 2 bytes = ax
  int aY = Wire.read()<<8|Wire.read(); //second 2 bytes = ay
  int aZ = Wire.read()<<8|Wire.read(); //last 2 bytes = az
  //Serial.println("STILL working?");
  // Must be processed: based on Accelerometer register settings
  allData[1] = aX;
  allData[2] = aY;
  allData[3] = aZ;
}

void recordGyro1(int sensor){
  // 
  Wire.beginTransmission(sensor); //I2C Address
  Wire.write(0x43); //Register 43
  Wire.endTransmission();
  Wire.requestFrom(sensor,6); //Reads from registers 3B-40
  while(Wire.available() < 6);
  int gyroX = Wire.read()<<8|Wire.read(); //first 2 bytes = gx
  int gyroY = Wire.read()<<8|Wire.read(); //second 2 bytes =gy
  int gyroZ = Wire.read()<<8|Wire.read(); //last 2 bytes = gz
  allData[4] = gyroX;
  allData[5] = gyroY;
  allData[6] = gyroZ;

}
void recordAccel2(int sensor){
  //records accel values in x,y,z directions from one sensor
  
  Wire.beginTransmission(sensor); //I2C Address
  Wire.write(0x3B); //Register 3B - Accelerometer readings
  Wire.endTransmission();
  Wire.requestFrom(sensor,6); //Reads from registers 3B-40
  while(Wire.available() < 6);
  int aX = Wire.read()<<8|Wire.read(); //first 2 bytes = ax
  int aY = Wire.read()<<8|Wire.read(); //second 2 bytes = ay
  int aZ = Wire.read()<<8|Wire.read(); //last 2 bytes = az
  // Must be processed: based on Accelerometer register settings
  allData[8] = aX;
  allData[9] = aY;
  allData[10] = aZ;
}


void recordGyro2(int sensor){
  // 
  Wire.beginTransmission(sensor); //I2C Address
  Wire.write(0x43); //Register 43
  Wire.endTransmission();
  Wire.requestFrom(sensor,6); //Reads from registers 3B-40
  while(Wire.available() < 6);
  int gyroX = Wire.read()<<8|Wire.read(); //first 2 bytes = gx
  int gyroY = Wire.read()<<8|Wire.read(); //second 2 bytes =gy
  int gyroZ = Wire.read()<<8|Wire.read(); //last 2 bytes = gz

  allData[11] = gyroX;
  allData[12] = gyroY;
  allData[13] = gyroZ;

}

void emgSig1(int vPin){
  //emg
  int sensorValue = analogRead(vPin);
  float voltage = sensorValue;//*(5.0/1024.0); 
  allData[15] = voltage;
}

void emgSig2(int vPin){
  //emg
  int sensorValue = analogRead(vPin);
  float voltage = sensorValue;//*(5.0/1024.0);
  allData[16] = voltage;
}

void writeIMU(){
  //print data onto SD Module
  myFile = SD.open("IMUtest.txt", FILE_WRITE);
  // if the file opened okay, write to it:
  if (myFile) {
    for(int i = 0; i < 13; i++){
      myFile.println(allData[i]);
    }
  // //close the file:
  myFile.close();
  }
  else {
  //if the file didn't open, print an error:
  Serial.println("error opening test.txt");
  }
}

void writeEMG(){
  //print data onto SD Module
  myFile = SD.open("EMGtest.txt", FILE_WRITE);
  // if the file opened okay, write to it:
  if (myFile) {
    for(int i = 14; i < 17; i++)
    {
      myFile.print(allData[i]);
      myFile.print(" ");
    }
    myFile.println();
  //close the file:
    myFile.close();
  }
  else {
  //if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }
}