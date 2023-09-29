#include <DynamixelShield.h>
#include <stdlib.h>
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8);
  #define DEBUG_SERIAL soft_serial
#elif defined(ARDUINO_SAM_DUE) || defined(ARDUINO_SAM_ZERO)
  #define DEBUG_SERIAL SerialUSB    
#else
  #define DEBUG_SERIAL Serial
#endif
const float DXL_PROTOCOL_VERSION = 2.0;
DynamixelShield dxl;

int robot_legs = 2;
uint8_t IDs[] = {0, 3};
// Declaring the global variables:
float clock_period = 10; // Define the desired clock period for the motor. 
bool backward = false; // Variable used to define the direction of the leg's swing. false- forward kinematics; true- reverse kinematics.
long start;
float cycle = clock_period / 2, angle = 0.0;
float speed = 100.0 / cycle; // 100 is the maximum angle at which the leg swings. 

float semicircle(int leg, long time_elapsed){
  float timer = fmod(time_elapsed / 1000.0, clock_period);
  
  if(timer <= cycle){ // Forward trajectory.
    angle = speed * timer;
  }
  else { // Reverse trajectory.
    angle = 100 - speed * (timer - cycle);
  }

  if(backward && leg == 1){
    return 100 - angle;
  }
  return angle; // Adding the motor's offset.
}

void setup(){
  DEBUG_SERIAL.begin(115200);
  dxl.begin(1000000);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  for (int i = 0; i <= 0; i++){
    dxl.torqueOff(IDs[i]);
    dxl.setOperatingMode(IDs[i], OP_POSITION);
    dxl.torqueOn(IDs[i]);
    delay(100);
  }

  start = millis();
}

void loop(){
  long time_elapsed = millis() - start;

  for(int i = 0; i < robot_legs; i++){
    float print_angle = semicircle(i, time_elapsed);
    DEBUG_SERIAL.print(IDs[i]);
    DEBUG_SERIAL.print(",");
    DEBUG_SERIAL.print(print_angle);
    DEBUG_SERIAL.print(",");
    DEBUG_SERIAL.print(fmod(time_elapsed, clock_period * 1000) / 1000.0);
    DEBUG_SERIAL.println();
    dxl.setGoalPosition(IDs[i], print_angle, UNIT_DEGREE); // Setting the initial position of the leg.
  }
  delay(10);
}
