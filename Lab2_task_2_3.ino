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

char mode[] = "SEMICIRCLE";
uint8_t IDs[] = {0, 3};
float clock_period = 12, cycle = clock_period / 2, theta_1, theta_2, angle = 0.0, speed = 100.0 / cycle;
bool backward = true;
long time_start;
int leg_1, leg_2, total_legs = 2;;

void semicircle(long time_elapsed, float theta_2){
  float timer = fmod(time_elapsed/1000.0, clock_period), x = 0.0, y = 0.0, distance = 8, speed = distance / clock_period;
  theta_1 = 0.0;
  theta_2 = 0.0;
  leg_1 = 7;
  leg_2 = 7;
  
  if(timer <= 4 / speed){
    x = 4 - speed * timer;
    y = 10;
  }
  else{
    x = speed * (timer - 4 / speed);
    y = 10 + sqrt(4-(x - 2) * (x - 2));
  }
  
  theta_2 = acos((x * x + y * y - leg_1 * leg_1 - leg_2 * leg_2) / (2 * leg_1 * leg_2)) * (180 / PI);
  theta_1 = atan2(y, x) - atan2(leg_2 * sin(theta_2), (leg_1 + leg_2 * cos(theta_2))) * (180 / PI);
}

void rectangle(long time_elapsed){
  float timer = fmod(time_elapsed / 1000.0, clock_period), x = 0.0, y = 0.0; 
  theta_1 = 0.0;
  theta_2 = 0.0;
  \\ Length of the legs:
  leg_1 = 7;
  leg_2 = 7;
  
  if(timer <= 2){
    x = 0;
    y = timer + 8;
  }
  else if(timer>2 && timer<=6){
    x = timer-2;
    y = 10;
  }
  else if(timer > 6 && timer <= 8){
    x = 4;
    y = 10 - (timer - 6);
  }
  else{
    x = 4 - (timer - 8);
    y = 8;
  }
  
  theta_2 = acos((x * x + y * y - leg_1 * leg_1 - leg_2 * leg_2) / (2 * leg_1 * leg_2)) * (180 / PI);
  theta_1 = atan2(y, x) - atan2(leg_2 * sin(theta_2), (leg_1 + leg_2 * cos(theta_2))) * (180 / PI);
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

  time_start = millis();
}

void loop(){
  long time_elapsed = millis() - time_start;

  if(strcmp(mode, "TASK3") == 0){
    for (int i = 0; i < total_legs; i++){
      float print_angle = leg_position(i, time_elapsed);
      theta_2 = 0.5 * theta_1
      semicircle(time_elapsed, theta_2);
      dxl.setGoalPosition(IDs[i], print_angle, UNIT_DEGREE);
    }
    delay(10);
  }
  else if(strcmp(mode, "SEMICIRCLE") == 0){
    theta_2 = 0
    semicircle(time_elapsed, theta_2);
    dxl.setGoalPosition(IDs[0], theta_1, UNIT_DEGREE);
    dxl.setGoalPosition(IDs[1], theta_2, UNIT_DEGREE);
    delay(10);
  }
  else if(strcmp(mode, "RECTANGLE") == 0){
    rectangle(time_elapsed);
    dxl.setGoalPosition(IDs[0], theta_1, UNIT_DEGREE);
    dxl.setGoalPosition(IDs[1], theta_2, UNIT_DEGREE);
    delay(10);
  }
}
