#include <DynamixelShield.h>
#include <stdlib.h>
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
#include <SoftwareSerial.h>
SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
#define DEBUG_SERIAL soft_serial
#elif defined(ARDUINO_SAM_DUE) || defined(ARDUINO_SAM_ZERO)
#define DEBUG_SERIAL SerialUSB
#else
#define DEBUG_SERIAL Serial
#endif
const float DXL_PROTOCOL_VERSION = 2.0;
DynamixelShield dxl;

uint8_t IDs[] = {0, 3}; // Defined as [leg 1, leg 2]

void setup() {
  DEBUG_SERIAL.begin(115200);
  dxl.begin(1000000);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
}

void loop() {
  float alpha1 = 0, alpha2 = 0, angle = 0;
  int task = 1; // 2, or 3.

  if(task == 1) {

  for(angle = 180; angle >= 0; angle -= 0.01){ // Upper semi circle.
    alpha1 = 1.1 * cos(angle) * 180.0 / PI; // Converting radians to degrees.
    alpha2 = 1.1 * sin(angle) * 180.0 / PI; // Converting radians to degrees.
    dxl.setGoalPosition(0, -alpha1 + 60, UNIT_DEGREE); // Adding the motor offset.
    dxl.setGoalPosition(3, alpha2 + 60, UNIT_DEGREE); // Adding the motor offset.
  }
  
  for(angle = 0; angle >= -180; angle -= 0.01){ // Lower semi circle.
    alpha1 = 1.1 * cos(angle) * 180.0 / PI;
    alpha2 = 1.1 * sin(angle) * 180.0 / PI;
    dxl.setGoalPosition(0, -alpha1 + 60, UNIT_DEGREE);
    dxl.setGoalPosition(3, alpha2 + 60, UNIT_DEGREE);
  }
  }

  if(task == 2) {

  for(angle = 180; angle >= 0; angle -= 0.01){
    alpha1 = 1.1 * (cos(angle) * 180.0 / PI) + 0.9;
    alpha2 = 1.1 * (sin(angle) * 180.0 / PI) + 0.9;
    dxl.setGoalPosition(0, -alpha1 + 60, UNIT_DEGREE);
    dxl.setGoalPosition(3, alpha2 + 60, UNIT_DEGREE);
  }
  
  for(angle = 0; angle >= -180; angle -= 0.01){
    alpha1 = 1.1 * (cos(angle) * 180.0 / PI) + 0.9;
    alpha2 = 1.1 * (sin(angle) * 180.0 / PI) + 0.9;
    dxl.setGoalPosition(0, -alpha1 + 60, UNIT_DEGREE);
    dxl.setGoalPosition(3, alpha2 + 60, UNIT_DEGREE);
  }
  }

  if(task == 3) {

    for(angle = 180; angle >= 0; angle -= 0.01){
    alpha1 = 1.1 * (cos(angle) * 180.0 / PI) - 0.9;
    alpha2 = 1.1 * (sin(angle) * 180.0 / PI) - 0.9;
    dxl.setGoalPosition(0, -alpha1 + 60, UNIT_DEGREE);
    dxl.setGoalPosition(3, alpha2 + 60, UNIT_DEGREE);
  }
  
  for(angle = 0; angle >= -180; angle -= 0.01){
    alpha1 = 1.1 * (cos(angle) * 180.0 / PI) - 0.9;
    alpha2 = 1.1 * (sin(angle) * 180.0 / PI) - 0.9;
    dxl.setGoalPosition(0, -alpha1 + 60, UNIT_DEGREE);
    dxl.setGoalPosition(3, alpha2 + 60, UNIT_DEGREE);
  }
  }
