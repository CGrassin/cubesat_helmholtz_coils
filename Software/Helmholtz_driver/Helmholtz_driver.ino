/*
* This Arduino code drives a 3D Helmholtz coils system through
* the serial port.
* See article at https://charleslabs.fr/en/project-ECE3SAT+-+Helmholtz+Coils
*
* To control the coils, send a serial string with format "X_value Y_value Z_value\n"
* where values are floating point number in gauss.
* Eg: "1.1 -1.1 0\n" sets coil X to generate 1.1 gauss, Y -1.1 gauss and Z 0.
*
* This code automatically computes the physical parameters of the coil and outputs it
* to the serial monitor.
*
* Note: this code assumes that the 6 coils (3 pairs) are identical.
*
* Licence: MIT
* Charles Grassin, 2021
*/

// Pin positions
#define FET_PA_1 (3)
#define FET_NB_1 (2)
#define FET_NA_1 (4)
#define FET_PB_1 (5)
#define FET_PA_2 (6)
#define FET_NB_2 (7)
#define FET_NA_2 (8)
#define FET_PB_2 (9)
#define FET_PA_3 (10)
#define FET_NB_3 (A1)
#define FET_NA_3 (A0)
#define FET_PB_3 (11)

// Input voltage in volts.
#define PARAMETER_INPUT_VOLTAGE   30.0f // V
// DC resistance of the coils.
#define PARAMETER_COIL_RESISTANCE 23.0f // ohm
// Physical calcultation parameters:
#define PARAMETER_MU_0            0.0126f
// Number of turns of the coils.
#define PARAMETER_N               80.0f //turns
// Coils radius.
#define PARAMETER_R               0.4f  // meters
// Numeric factor in Helmholtz equation = (4/5)^(3/2)
#define PARAMETER_CONSTANT        0.716f // no unit
// Minimum and maximum current.
#define PARAMETER_MIN_I 0.0f // A
#define PARAMETER_MAX_I (PARAMETER_INPUT_VOLTAGE / PARAMETER_COIL_RESISTANCE) // A
// Max field strength
#define PARAMETER_MAX_GAUSS ( (PARAMETER_CONSTANT * PARAMETER_N * PARAMETER_MU_0 * PARAMETER_MAX_I ) / PARAMETER_R ) // gauss
// Maximum autorized delta between previous
// value and current (in gauss).
// This parameter prevents massive swings.
// Set to 999 if unused.
#define PARAMETER_MAX_GAUSS_DELTA       1.0f // gauss

// Serial
#define SERIAL_BAUDRATE 115200
#define SERIAL_BUFFER_LENGTH 100
char serialBuffer[SERIAL_BUFFER_LENGTH];
uint8_t currentIndex=0;

//Variables
float previousXField=0, previousYField=0, previousZField=0;

// MAIN
void setup() {
  Serial.begin(SERIAL_BAUDRATE);
  coilSetup();

  Serial.println("# Coil settings:");
  Serial.print("# * PARAMETER_INPUT_VOLTAGE=");
  Serial.println(PARAMETER_INPUT_VOLTAGE);
  Serial.print("# * PARAMETER_COIL_RESISTANCE=");
  Serial.println(PARAMETER_COIL_RESISTANCE);
  Serial.print("# * PARAMETER_N=");
  Serial.println(PARAMETER_N);
  Serial.print("# * PARAMETER_R=");
  Serial.println(PARAMETER_R);
  Serial.print("# * PARAMETER_MIN_I=");
  Serial.println(PARAMETER_MIN_I);
  Serial.print("# * PARAMETER_MAX_I=");
  Serial.println(PARAMETER_MAX_I);
  Serial.print("# * PARAMETER_MAX_GAUSS=");
  Serial.println(PARAMETER_MAX_GAUSS);
  Serial.print("# * PARAMETER_MAX_GAUSS_DELTA=");
  Serial.println(PARAMETER_MAX_GAUSS_DELTA);
}
void loop() {
  serialReadCommand();
  delay(10);
}

// SERIAL READ COMMAND
uint8_t serialReadCommand(){
  while (Serial.available() > 0) {
    char recieved = Serial.read();

    serialBuffer[currentIndex] = recieved;
    currentIndex ++;
    if (currentIndex >= SERIAL_BUFFER_LENGTH) {
      // Invalid command: command is too long
      Serial.println("NOK");
      currentIndex = 0;
    }

    if (recieved == '\n' || recieved == '\r') {
    // Close string
      serialBuffer[currentIndex-1] = '\0';
    // Clear buffer for next serial transaction
      currentIndex = 0; 

    // Split the string
      char* commaIndex = strchr(serialBuffer, ' ');
      if (commaIndex==NULL) {
        // Invalid command: command is malformed
        Serial.println("NOK");
        return 0;
      }
      commaIndex[0] = '\0';
      char* secondCommaIndex = strchr(commaIndex+1, ' ');
      if (secondCommaIndex==NULL) {
        // Invalid command: command is malformed
        Serial.println("NOK");
        return 0;
      }
      secondCommaIndex[0] = '\0';

      // Convert & use values
      float xvalue = atof(serialBuffer);
      float yvalue = atof(commaIndex+1);
      float zvalue = atof(secondCommaIndex+1);
      byte returnValue = 1;
      returnValue &= coilXField(xvalue);
      returnValue &= coilYField(yvalue);
      returnValue &= coilZField(zvalue);

      Serial.println("OK");
      return returnValue;
    }
  }
  return 0;
}

// HIGH LEVEL COIL CONTROL
byte coilXField(float gauss){
  if (fabsf(gauss-previousXField) > PARAMETER_MAX_GAUSS_DELTA) return 0;
  previousXField=gauss;
  coilX(fieldToPWM(gauss),gauss>0);
  return 1;
}
byte coilYField(float gauss){
  if (fabsf(gauss-previousYField) > PARAMETER_MAX_GAUSS_DELTA) return 0;
  previousYField=gauss;
  coilY(fieldToPWM(gauss),gauss>0);
  return 1;
}
byte coilZField(float gauss){
  if (fabsf(gauss-previousZField) > PARAMETER_MAX_GAUSS_DELTA) return 0;
  previousZField=gauss;
  coilZ(fieldToPWM(gauss),gauss>0);
  return 1;
}
byte fieldToPWM(float gauss){
  // clip to max field (100% PWM)
  if(gauss>PARAMETER_MAX_GAUSS) gauss = PARAMETER_MAX_GAUSS;
  else if (gauss<-PARAMETER_MAX_GAUSS) gauss = -PARAMETER_MAX_GAUSS;
  return (uint8_t)mapfloat(PARAMETER_R * fabsf(gauss) /( PARAMETER_CONSTANT * PARAMETER_N * PARAMETER_MU_0 ),PARAMETER_MIN_I,PARAMETER_MAX_I,0.0f,255.0f);
}

// LOW LEVEL COIL CONTROL
void coilSetup(){
  pinMode(FET_PA_1,OUTPUT);
  pinMode(FET_NB_1,OUTPUT);
  pinMode(FET_NA_1,OUTPUT);
  pinMode(FET_PB_1,OUTPUT);
  pinMode(FET_PA_2,OUTPUT);
  pinMode(FET_NB_2,OUTPUT);
  pinMode(FET_NA_2,OUTPUT);
  pinMode(FET_PB_2,OUTPUT);
  pinMode(FET_PA_3,OUTPUT);
  pinMode(FET_NB_3,OUTPUT);
  pinMode(FET_NA_3,OUTPUT);
  pinMode(FET_PB_3,OUTPUT);

  digitalWrite(FET_PA_1,LOW);
  digitalWrite(FET_NB_1,LOW);
  digitalWrite(FET_NA_1,LOW);
  digitalWrite(FET_PB_1,LOW);
  digitalWrite(FET_PA_2,LOW);
  digitalWrite(FET_NB_2,LOW);
  digitalWrite(FET_NA_2,LOW);
  digitalWrite(FET_PB_2,LOW);
  digitalWrite(FET_PA_3,LOW);
  digitalWrite(FET_NB_3,LOW);
  digitalWrite(FET_NA_3,LOW);
  digitalWrite(FET_PB_3,LOW);
}
void coilX(byte duty, byte dir){
  if(dir){
    digitalWrite(FET_NA_1,HIGH);
    digitalWrite(FET_NB_1,LOW);
    analogWrite(FET_PA_1,duty);
    digitalWrite(FET_PB_1,LOW);
  } else {
    digitalWrite(FET_NB_1,HIGH);
    digitalWrite(FET_NA_1,LOW);
    analogWrite(FET_PB_1,duty);
    digitalWrite(FET_PA_1,LOW);
  }
}
void coilY(byte duty, byte dir){
  if(dir){
    digitalWrite(FET_NA_2,HIGH);
    digitalWrite(FET_NB_2,LOW);
    analogWrite(FET_PA_2,duty);
    digitalWrite(FET_PB_2,LOW);
  } else {
    digitalWrite(FET_NB_2,HIGH);
    digitalWrite(FET_NA_2,LOW);
    analogWrite(FET_PB_2,duty);
    digitalWrite(FET_PA_2,LOW);
  }
}
void coilZ(byte duty, byte dir){
  if(dir){
    digitalWrite(FET_NA_3,HIGH);
    digitalWrite(FET_NB_3,LOW);
    analogWrite(FET_PA_3,duty);
    digitalWrite(FET_PB_3,LOW);
  } else {
    digitalWrite(FET_NB_3,HIGH);
    digitalWrite(FET_NA_3,LOW);
    analogWrite(FET_PB_3,duty);
    digitalWrite(FET_PA_3,LOW);
  }
}

// Maths fuctions
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max) {
  if(x<=in_min) return out_min;
  else if (x>=in_max) return out_max;
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
