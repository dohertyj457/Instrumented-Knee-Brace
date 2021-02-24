//Sensor_data_collection.ino
//UVM SEED Team 15: Instrumented Knee Brace
//Created 2/23/2021 from previous drafts in Fall 2020
//Master code to operate the sensors on the knee brace and patient: chunky version without as many functions yet

//Libraries
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <MPU6050.h>


//Defining anything that will be used

File myFile;
unsigned long myTime;
int times[4];

//Arduino communication
int SDPin = 10; //Pin SD Reader is connected to
const int MPU_ADDR1 = 0b1101000; // IMU I2C - differentiated using the AD0 pin set HIGH or LOW
const int MPU_ADDR2 = 0b1101001; // (AD0 = 0, 1101000. AD0 = 1, 1101001)

//Data Variables

//EMG
float voltage1, voltage2;
//Accelerometer
long aX1, aY1, aZ1, aX2, aY2, aZ2;
float gForceX1,gForceY1,gForceZ1, gForceX2,gForceY2,gForceZ2;

// Gyroscope
long gyroX1, gyroY1, gyroZ1, gyroX2, gyroY2, gyroZ2;
float rotX1, rotY1, rotZ1, rotX2, rotY2, rotZ2;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin();
  setupMPU();
  setupSD();
}

void loop() {
  myTime = millis();
  recordAccelRegisters1();
  recordGyroRegisters1();
  times[1] = myTime;
  recordAccelRegisters2();
  recordGyroRegisters2();
  times[2] = myTime;
  voltage1 = emgSig1();
  times[3] = myTime;
  voltage2 = emgSig2();
  times[4] = myTime;
  writeIMU(times[1],times[2]);
  writeEMG(times[3],times[4]);
  printData();
  //delay(100);

}


//Functions called above_______________________________________________________________________

void setupMPU(){
  //Sets up recording parameters
  
  //Gets MPU6050 1 out of sleep mode
  Wire.beginTransmission(MPU_ADDR1); //I2C address of MPU (2nd IMU would have address 0b1101001)
  Wire.write(0x6B); //Accesses register 6B - Power Management
  Wire.write(0b00000000); //Sets SLEEP register to 0
  Wire.endTransmission();

  //Gets MPU6050 2 out of sleep mode
  Wire.beginTransmission(MPU_ADDR2); //I2C address of MPU (2nd IMU would have address 0b1101001)
  Wire.write(0x6B); //Accesses register 6B - Power Management
  Wire.write(0b00000000); //Sets SLEEP register to 0
  Wire.endTransmission();
   
   
  //Configures gyroscope recording parameters (MPU1)
  Wire.beginTransmission(MPU_ADDR1); //I2C Address
  Wire.write(0x1B); //Register 1B - Gyroscope Configuration
  Wire.write(0x00000000); // Sets full scale range of gyroscope outputs to +- 250 degrees per second
  Wire.endTransmission();

 //Configures gyroscope recording parameters (MPU2)
  Wire.beginTransmission(MPU_ADDR2); //I2C Address
  Wire.write(0x1B); //Register 1B - Gyroscope Configuration
  Wire.write(0x00000000); // Sets full scale range of gyroscope outputs to +- 250 degrees per second
  Wire.endTransmission();

  
  //Configures accelerometer recording parameters (MPU1)
  Wire.beginTransmission(MPU_ADDR1); //I2C Address
  Wire.write(0x1C); //Register 1B - Gyroscope Configuration
  Wire.write(0x00000000); // Sets full scale range of accelerometer outputs to +-2g
  Wire.endTransmission();

   //Configures accelerometer recording parameters (MPU2)
  Wire.beginTransmission(MPU_ADDR2); //I2C Address
  Wire.write(0x1C); //Register 1B - Gyroscope Configuration
  Wire.write(0x00000000); // Sets full scale range of accelerometer outputs to +-2g
  Wire.endTransmission();
}


void setupSD(){
  //SD setup
  while (!Serial) {; // wait for serial port to connect. Needed for native USB port only
    }
  Serial.print("Initializing SD card...");
  if (!SD.begin(SDPin)) {
  Serial.println("initialization failed!");
  while (1);
  }
  Serial.println("initialization done."); 
}


//Records ax, ay, and az data and converts it into a g-force value
void recordAccelRegisters1(){
  Wire.beginTransmission(MPU_ADDR1); //I2C Address
  Wire.write(0x3B); //Register 3B - Accelerometer readings
  Wire.endTransmission();
  Wire.requestFrom(MPU_ADDR1,6); //Reads from registers 3B-40
  while(Wire.available() < 6);
  aX1 = Wire.read()<<8|Wire.read(); //first 2 bytes = ax
  aY1 = Wire.read()<<8|Wire.read(); //second 2 bytes = ay
  aZ1 = Wire.read()<<8|Wire.read(); //last 2 bytes = az
  processAccelData();

}

void recordAccelRegisters2(){
  Wire.beginTransmission(MPU_ADDR2); //I2C Address
  Wire.write(0x3B); //Register 3B - Accelerometer readings
  Wire.endTransmission();
  Wire.requestFrom(MPU_ADDR2,6); //Reads from registers 3B-40
  while(Wire.available() < 6);
  aX2 = Wire.read()<<8|Wire.read(); //first 2 bytes = ax
  aY2 = Wire.read()<<8|Wire.read(); //second 2 bytes = ay
  aZ2 = Wire.read()<<8|Wire.read(); //last 2 bytes = az
  processAccelData();

}

void processAccelData(){
  gForceX1 = aX1 / 16384.0;
  gForceY1 = aY1 / 16384.0;
  gForceZ1 = aZ1 / 16384.0;
  gForceX2 = aX2 / 16384.0;
  gForceY2 = aY2 / 16384.0;
  gForceZ2 = aZ2 / 16384.0;
  
}

//Records gx, gy, and gz data and converts it into a  value
void recordGyroRegisters1(){
  Wire.beginTransmission(MPU_ADDR1); //I2C Address
  Wire.write(0x43); //Register 3B - Accelerometer readings
  Wire.endTransmission();
  Wire.requestFrom(MPU_ADDR1,6); //Reads from registers 3B-40
  while(Wire.available() < 6);
  gyroX1 = Wire.read()<<8|Wire.read(); //first 2 bytes = gx
  gyroY1 = Wire.read()<<8|Wire.read(); //second 2 bytes =gy
  gyroZ1 = Wire.read()<<8|Wire.read(); //last 2 bytes = gz
  processGyroData();

}


void recordGyroRegisters2(){
  Wire.beginTransmission(MPU_ADDR2); //I2C Address
  Wire.write(0x43); //Register 3B - Accelerometer readings
  Wire.endTransmission();
  Wire.requestFrom(MPU_ADDR2,6); //Reads from registers 3B-40
  while(Wire.available() < 6);
  gyroX2 = Wire.read()<<8|Wire.read(); //first 2 bytes = gx
  gyroY2 = Wire.read()<<8|Wire.read(); //second 2 bytes =gy
  gyroZ2 = Wire.read()<<8|Wire.read(); //last 2 bytes = gz
  processGyroData();

}

void processGyroData(){
  rotX1 = gyroX1 / 131;
  rotY1 = gyroY1 / 131;
  rotZ1 = gyroZ1 / 131;
  rotX2 = gyroX2 / 131;
  rotY2 = gyroY2 / 131;
  rotZ2 = gyroZ2 / 131;
}

float emgSig1(){
  //emg
  int sensorValue = analogRead(A0);
  float voltage1 = sensorValue*(5.0/1023.0);
  // print out the value you read:
  //-Serial.println(voltage);
  // delay(10);    
  return voltage1;
}

float emgSig2(){
  //emg
  int sensorValue = analogRead(A1);
  float voltage2 = sensorValue*(5.0/1023.0);
  // print out the value you read:
  //Serial.println(voltage);
  // delay(10);    
  return voltage2;
}

void printData(){
  Serial.println("Data loaded to SD");
//  Serial.print("Gyro1(deg):");
//  Serial.print("X=");
//  Serial.print(rotX1);
//  Serial.print(",");
//  Serial.print("Y=");
//  Serial.print(rotY1);
//  Serial.print(",");
//  Serial.print("Z=");
//  Serial.print(rotZ1);
//  Serial.print("Accel1(g):");
//  Serial.print("X=");
//  Serial.print(gForceX1);
//  Serial.print(",");
//  Serial.print("Y=");
//  Serial.print(gForceY1);
//  Serial.print(",");
//  Serial.print("Z=");
//  Serial.println(gForceZ1);

  
//  Serial.print("Gyro2(deg):");
//  Serial.print("X=");
//  Serial.print(rotX2);
//  Serial.print(",");
//  Serial.print("Y=");
//  Serial.print(rotY2);
//  Serial.print(",");
//  Serial.print("Z=");
//  Serial.print(rotZ2);
//  Serial.print("Accel2(g):");
//  Serial.print("X=");
//  Serial.print(gForceX2);
//  Serial.print(",");
//  Serial.print("Y=");
//  Serial.print(gForceY2);
//  Serial.print(",");
//  Serial.print("Z=");
//  Serial.println(gForceZ2);
}

void writeIMU(long time1, long time2){
  //print data onto SD Module
  myFile = SD.open("IMUtest.txt", FILE_WRITE);
  // if the file opened okay, write to it:
  if (myFile) {
  //Serial.print("IMUtest.txt...");
  //myFile.println(sigtitle1);
  myFile.print(time1);myFile.print(":");myFile.print(rotX1); myFile.print(","); myFile.print(rotY1); myFile.print(","); myFile.print(rotZ1); myFile.print(","); 
  myFile.print(gForceX1); myFile.print(","); myFile.print(gForceY1); myFile.print(","); myFile.print(gForceZ1); myFile.print(","); 
  myFile.print(time2);myFile.print(":");myFile.print(rotX2); myFile.print(","); myFile.print(rotY2); myFile.print(","); myFile.print(rotZ2); myFile.print(",");
  myFile.print(gForceX2); myFile.print(","); myFile.print(gForceY2); myFile.print(","); myFile.println(gForceZ2);// this may need to be adjusted when printing more than one signal 
  //close the file:
  myFile.close();
  }
  else {
  //if the file didn't open, print an error:
  Serial.println("error opening test.txt");
  }
}

void writeEMG(long time3, long time4){
  //print data onto SD Module
  myFile = SD.open("EMGtest.txt", FILE_WRITE);
  // if the file opened okay, write to it:
  if (myFile) {
  //Serial.print("EMGtest.txt...");
  //myFile.println(sigtitle1);
  myFile.print(time3); myFile.print(":"); myFile.print(voltage1); myFile.print(","); myFile.print(time4); myFile.print(":"); myFile.println(voltage2); // this may need to be adjusted when printing more than one signal 
  //close the file:
  myFile.close();
  }
  else {
  //if the file didn't open, print an error:
  Serial.println("error opening test.txt");
  }
}
