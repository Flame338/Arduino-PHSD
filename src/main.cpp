#include <Arduino.h>
#include <SimpleKalmanFilter.h>
#include <Wire.h>

#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define CTRL_REG5 0x24

SimpleKalmanFilter dirKalmanFilter(1, 1, 0.01);

void getGyroValues();
//void integrate(float lower_limit, float upper_limit, float velocity);
void setupL3G4200D(int scale);
void writeRegister(int deviceAddress, byte address, byte val);
int readRegister(int deviceAddress, byte address);

int L3G4200D_Address = 105; //I2C  address of GY-50 L3G4200D

int x, y, z;

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(115200);
 
  Serial.println("starting up L3G4200D");
  setupL3G4200D(500); // Configure L3G4200 - 250, 500 or 2000 deg/sec
 
  delay(1000); //wait for the sensor to be ready
}

void loop() {
  // put your main code here, to run repeatedly:
  int angleX=0, angleY=0, angleZ=0;
  int accelX=0, accelY=0, accelZ=0;
  for(int i=0;i<1000;i++){
    getGyroValues();
    angleX += x;
    angleY += y;
    angleZ += z;
  }
    accelX = (angleX/50); 
    accelY = (angleY/50) - 620;
    accelX = accelX/2;
    accelY = accelY/2;

    angleX = (angleX/1000);
    angleY = (angleY/1000);
    angleZ = (angleZ/1000);

  //Serial.printf("%3d, %3d, %3d\n",sumX, sumY, sumZ); //Angular velocity
  Serial.printf("Amplitude:%2f, %2f\n", accelX/sin(angleX), accelY/sin(angleY) + 260); // Amplitude
  Serial.printf("w:  %2d, %2d, %2d\n", angleX, angleY, angleZ);
  Serial.printf("S:  %2d, %2d\n", accelX/2, accelY/2);
}

// put function definitions here:
void getGyroValues(){
 
byte xMSB = readRegister(L3G4200D_Address, 0x29);
 byte xLSB = readRegister(L3G4200D_Address, 0x28);
 x = ((xMSB << 8) | xLSB);
 x = x * 0.000571;
 
byte yMSB = readRegister(L3G4200D_Address, 0x2B);
 byte yLSB = readRegister(L3G4200D_Address, 0x2A);
 y = ((yMSB << 8) | yLSB);
 y = y * 0.000571;
 
byte zMSB = readRegister(L3G4200D_Address, 0x2D);
 byte zLSB = readRegister(L3G4200D_Address, 0x2C);
 z = ((zMSB << 8) | zLSB);
 z = z * 0.000571;
}

void setupL3G4200D(int scale) {
  writeRegister(L3G4200D_Address, CTRL_REG1, 0b00001111);
  writeRegister(L3G4200D_Address, CTRL_REG2, 0b00000000);
  writeRegister(L3G4200D_Address, CTRL_REG3, 0b00001000);

  if(scale == 250){
    writeRegister(L3G4200D_Address, CTRL_REG4, 0b00000000);
  }else if(scale == 500){
    writeRegister(L3G4200D_Address, CTRL_REG4, 0b00010000);
  }else{
    writeRegister(L3G4200D_Address, CTRL_REG4, 0b00110000);
  }
  writeRegister(L3G4200D_Address, CTRL_REG5, 0b00000000);
}

void writeRegister(int deviceAddress, byte address, byte val) {
  Wire.beginTransmission(deviceAddress); // start transmission to device 
  Wire.write(address); // send register address
  Wire.write(val); // send value to write
  Wire.endTransmission(); // end transmission
}

int readRegister(int deviceAddress, byte address){
  int v;
  Wire.beginTransmission(deviceAddress);
  Wire.write(address); // register to read
  Wire.endTransmission();
 
  Wire.requestFrom(deviceAddress, 1); // read a byte
 
  while(!Wire.available()) {
  // waiting
  }
 
  v = Wire.read();
 return v;
}
