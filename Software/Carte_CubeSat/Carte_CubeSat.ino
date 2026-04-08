#define RM3100_POLL_ADDRESS (0x00)
#define RM3100_CMM_ADDRESS  (0x01)
#define RM3100_READ  (0x01)
#define RM3100_WRITE (0x00)

//#define RM3100_DEBUG

//#define

//SPI
#include <SPI.h>
const int slaveSelectPin = A0;

//Bluetooth
#include <SoftwareSerial.h>
SoftwareSerial bluetooth(8, 4); // (RX, TX)

//Magnetorquers
#include <SoftPWM.h>
const int EN = 2;
const int pin_Coil1_L = 3;
const int pin_Coil1_R = 5;
const int pin_Coil2_L = 6;
const int pin_Coil2_R = 9;
const int pin_Coil3_L = 10;
const int pin_Coil3_R = 7;

//ITG3200
#include <Wire.h>
#define ITG3200_ADDRESS         (0x69)
#define ITG3200_WHO_AM_I        (0x00)
#define ITG3200_SMPLRT_DIV      (0x15)
#define ITG3200_DLPF_FS         (0x16)
#define ITG3200_GYRO_XOUT_H     (0x1D)
#define ITG3200_GYRO_XOUT_L     (0x1E)
#define ITG3200_GYRO_YOUT_H     (0x1F)
#define ITG3200_GYRO_YOUT_L     (0x20)
#define ITG3200_GYRO_ZOUT_H     (0x21)
#define ITG3200_GYRO_ZOUT_L     (0x22)
//This is a list of settings that can be loaded into the registers.
//DLPF, Full Scale Register Bits
//FS_SEL must be set to 3 for proper operation
//Set DLPF_CFG to 3 for 1kHz Fint and 42 Hz Low Pass Filter
char DLPF_CFG_0 = 1<<0;
char DLPF_CFG_1 = 1<<1;
char DLPF_CFG_2 = 1<<2;
char DLPF_FS_SEL_0 = 1<<3;
char DLPF_FS_SEL_1 = 1<<4;


//Variables
double magnetoX,magnetoY, magnetoZ;
unsigned long initTime;

void setup() {
  Serial.begin(9600);

  //Bluetooth setup
  bluetooth.begin(19200);

  //SPI setup for RM3200 sensor
  pinMode(slaveSelectPin, OUTPUT);
  digitalWrite(slaveSelectPin, HIGH);
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV32);
  SPI.setDataMode(SPI_MODE0);
  RM3100_startContinuousMeasurement();

  //Magnetorquers
  SoftPWMBegin();
  SoftPWMSet(pin_Coil3_R, 0);
  SoftPWMSetFadeTime(pin_Coil3_R, 0, 0);
  magnetorquersSetup();

  //ITG3200
  Wire.begin();
}

void loop() {
  RM3100_refresh();
  Serial.print(magnetoX/75.0f,2);
  Serial.print(" ");
    Serial.print(magnetoY/75.0f,2);
  Serial.print(" ");
    Serial.print(magnetoZ/75.0f,2);
  Serial.print("\r\n");
  delay(250);
}


/*
  ___ __  __   ____  _  __   __  
 | _ \  \/  | |__ / / |/  \ /  \ 
 |   / |\/| |  |_ \ | | () | () |
 |_|_\_|  |_| |___/ |_|\__/ \__/ 
                                 
*/
byte rm3100(byte readWrite, byte address,byte data) {
  byte value,stat;
  digitalWrite(slaveSelectPin, LOW);
  //  send in the address and value via SPI:
  if(readWrite) address += 0x80;
  
  stat = SPI.transfer(address);
  value = SPI.transfer(data);

  #ifdef RM3100_DEBUG
  Serial.print("Status: ");
  Serial.println(stat);
  Serial.print("Value: ");
  Serial.println(value);
  #endif

  // take the SS pin high to de-select the chip:
  digitalWrite(slaveSelectPin, HIGH);
  return value;
}

byte RM3100_refresh() {
  byte statusRM3100;
  
  digitalWrite(slaveSelectPin, LOW);

  statusRM3100 = SPI.transfer(0xA4);
  magnetoX = (double)SPI.transfer(0)*256*256 + (double)SPI.transfer(0)*256 + (double)SPI.transfer(0);
  if(magnetoX >= 8388608) magnetoX -= 16777216;

  magnetoY = (double)SPI.transfer(0)*256*256 + (double)SPI.transfer(0)*256 + (double)SPI.transfer(0);
  if(magnetoY >= 8388608) magnetoY -= 16777216;

  magnetoZ = (double)SPI.transfer(0)*256*256 + (double)SPI.transfer(0)*256 + (double)SPI.transfer(0);
  if(magnetoZ >= 8388608) magnetoZ -= 16777216;

  // take the SS pin high to de-select the chip:
  digitalWrite(slaveSelectPin, HIGH);

  return statusRM3100;
}
void RM3100_startContinuousMeasurement() {
  rm3100(RM3100_WRITE,RM3100_CMM_ADDRESS,0b01110001);
}
void RM3100_stopContinuousMeasurement() {
  rm3100(RM3100_WRITE,RM3100_CMM_ADDRESS,0b01110000);
}

/*
  __  __                    _                                
 |  \/  |__ _ __ _ _ _  ___| |_ ___ _ _ __ _ _  _ ___ _ _ ___
 | |\/| / _` / _` | ' \/ -_)  _/ _ \ '_/ _` | || / -_) '_(_-<
 |_|  |_\__,_\__, |_||_\___|\__\___/_| \__, |\_,_\___|_| /__/
             |___/                        |_|                
*/
void magnetorquers(byte duty1, byte dir1, byte duty2, byte dir2, byte duty3, byte dir3){
  magnetorquerX(duty1,dir1);
  magnetorquerY(duty2,dir2);
  magnetorquerZ(duty3,dir3);
}

void magnetorquersSetup(){
    pinMode(EN,OUTPUT);
    pinMode(pin_Coil1_L,OUTPUT);
    pinMode(pin_Coil1_R,OUTPUT);
    pinMode(pin_Coil2_L,OUTPUT);
    pinMode(pin_Coil2_R,OUTPUT);
    pinMode(pin_Coil3_L,OUTPUT);
    pinMode(pin_Coil3_R,OUTPUT);
    magnetorquers(0,0,0,0,0,0);
    digitalWrite(EN,HIGH);
}
void magnetorquerX(byte duty, byte dir){
  if(dir){
    digitalWrite(pin_Coil1_L,LOW);
    analogWrite(pin_Coil1_R,duty);
  }
  else{
    digitalWrite(pin_Coil1_R,LOW);
    analogWrite(pin_Coil1_L,duty);
  }
}

void magnetorquerY(byte duty, byte dir){
  if(dir){
    digitalWrite(pin_Coil2_L,LOW);
    analogWrite(pin_Coil2_R,duty);
  }
  else{
    digitalWrite(pin_Coil2_R,LOW);
    analogWrite(pin_Coil2_L,duty);
  }
}

void magnetorquerZ(byte duty, byte dir){
  if(dir){
    digitalWrite(pin_Coil3_L,LOW);
    SoftPWMSet(pin_Coil3_R,duty);
  }
  else{
    SoftPWMSet(pin_Coil3_R,0);
    analogWrite(pin_Coil3_L,duty);
  }
}

/*
  ___ _____ ___   _______ __   __  
 |_ _|_   _/ __| |__ /_  )  \ /  \ 
  | |  | || (_ |  |_ \/ / () | () |
 |___| |_| \___| |___/___\__/ \__/ 
                                   
*/
