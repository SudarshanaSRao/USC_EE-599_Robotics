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

// Initializing the number of legs:
int total_legs = 4;
float angle, t;

uint8_t IDs[] = {4, 1, 100, 6}; // Leg ID corresponding to [RF, LF, LR, RR] legs.   
uint8_t Directions[] = {1, 0, 0, 1}; // Leg rotating direction corresponding to [RF, LF, LR, RR]- value = 0 then rotate CCW, value = 1 then rotate CW.  
float gait[] = {0, 0.5, 0.5}; // (\phi_1, \phi_2, \phi_3) = [LF-RF, LR-RF, RR-RF]. 
float gait_deg[] = {0, 0, 180, 180}; // Gait in degrees: [RF, LF, LR, RR] the value for RF will always be 0, the value for LF = 360 * \phi_1, LR = 360 * \phi_2, RR = 360 * \phi_3- calculated in translate_gait_deg().

int Leg_zeroing_offset[]={110, -40, -60, 90}; // Zeroing calibration offset in degrees, leg angular position, ϕ, is defined as the angle measured clockwise about the axle from the upward vertical to the leg position, in radians. A zeroing calibration offset is needed because the servo’s zero position is not vertically downward and there is the offset between the servo-leg connector and the servo.  
                                         
// Clock parameters below needs to be calculated in clock_init():
float time_slow_start = 1; // In seconds, time after the start of the period that the the leg enters the slow phase.
float time_slow_end = 3; // In seconds, time after the start of the period that the the leg exits the slow phase.
float degree_slow_start = 150; // In deg, the slow phase starting position
float degree_slow_end = 210; // In deg, the slow phase starting position, if degree_slow_start = 150 and degree_slow_end = 210, this means whenever the leg's position is between 150 and 210, it will be in the slow phase.  

// You can change the desired Buehler clock parameters here:
float phi_s = 1.04; // In rad, ϕ_s is the angular extent of the slow phase.
float phi_0 = 3.66; // In rad, ϕ_0 is the center of the slow phase. 
float d_c = 0.4; // d_c is the duty factor of the slow phase (i.e, fraction of the period spent in the slow phase). 
float clock_period = 2; // In seconds, time to complete 1 rotation.

const float pi = 22 / 7;
float phi_s_degree = (phi_s * 180) / pi;
float phi_0_degree = (phi_0 * 180) / pi;

float omega_slow(){ 
    
  return phi_s_degree / (clock_period * d_c); // Returns desired leg speed in slow phase in deg/s.
}

float omega_fast(){  
    
  return (360 - phi_s_degree) / (clock_period * (1 - d_c));1// Returns desired leg speed in fast phase in deg/s.
}

// Configure the timing parameters:
void clock_init(){   
  // At the beginning of each stride period, the desired angle, \phi, should be 0 degree (leg should point vertically upward if you have a leg installed). 
  // Please note that the deadzone (300deg to 360deg) is fully within the fast phase (i.e., 0 < degree_slow_start < degree_slow_end < 300). Also, make sure 0 < time_slow_start < time_slow_end < clock_period. 

  degree_slow_start = phi_0_degree - (phi_s_degree / 2); // Returns the position that the the leg enters the slow phase in deg.
  degree_slow_end = phi_0_degree + (phi_s_degree / 2); // Returns the position that the the leg exits the slow phase in deg.
  time_slow_start = (phi_0_degree - (phi_s_degree / 2)) / omega_fast(); // Returns the time after the start of the period that the the leg enters the slow phase.
  time_slow_end = time_slow_start + (phi_s_degree / omega_slow()); // Returns the time after the start of the period that the the leg exits the slow phase.
 
  return;
}


float get_adjusted_time(int leg, float phase) {

  float delay_time = 0;

  if (phase < degree_slow_start) { 
        // Phase difference in the fast phase
        delay_time = phase / omega_fast();
    } else if (phase >= degree_slow_start && phase <= degree_slow_end) { 
        // Phase difference in the slow phase
        float slow_phase_time = phase - degree_slow_start;
        delay_time = time_slow_start + slow_phase_time / omega_slow();
    } else {
        // Phase difference is in the fast phase, but it comes after the slow phase
        float time_fast = phase - degree_slow_end;
        delay_time = time_slow_end + time_fast / omega_fast();
    }

    return delay_time;
}

// Computes desired motor angle at any time instance:
float get_desired_angle(int leg,                    // Robot's leg: in 0, 1, 2, 3.
                        long elapsed               // Time elapsed since start, in ms.
                   ) { // Return the desired postion of the robot's leg given the leg number and the time elapsed in deg. It's an absolute position from 0 to 360. Don't use cumulative positions. It's ok to return positions in the deadzone (>300).    


    float elapsed_time_second = elapsed / 1000.0;  // Converting the time in ms to second.
    float delay_elapsed_seconds = get_adjusted_time(leg, gait_deg[leg]);
    float time_second_final = elapsed_time_second + delay_elapsed_seconds;
    float tim = fmod(time_second_final, clock_period);

    // When motor is in the fast phase but before entering slow phase:
    if(tim < time_slow_start){ 
      angle = omega_fast() * tim;
      }

    // If motor is in slow phase:
    else if(time_slow_start <= tim && tim <= time_slow_end){
      angle = degree_slow_start + (omega_slow() * (tim - time_slow_start));
      }

    // If motor is in fast phase but after slow phase:
    else{
      angle = degree_slow_end + (omega_fast() * (tim - time_slow_end));
      }

    // Returning the angle with the zeroing offset:
    angle = angle + Leg_zeroing_offset[leg];
    
    if (angle > 360){
      return angle - 360;
    }
    
    else if(angle < 0){
      return angle + 360;
    }
    
    else{
      return angle;
    }

}

// Prints desired motor position and associated elapsed time to debug serial:
void print_position(long t, int leg, float desired_pos){
    
  DEBUG_SERIAL.print("Leg:");
  DEBUG_SERIAL.print(leg);
  DEBUG_SERIAL.print(";");
  DEBUG_SERIAL.print("Desired pos:");
  DEBUG_SERIAL.print(desired_pos);
  DEBUG_SERIAL.print(";");
  DEBUG_SERIAL.print("Time elapsed:"); 
  DEBUG_SERIAL.println((t % 4000) / 1000.0);
  
  return 0;
}

void translate_gait_deg(){ // Calculates the value of the array 'gait_deg[]': 
 
  gait_deg[0] = 0;  //RF
  gait_deg[1] = gait[0] * 360; //LF
  gait_deg[2] = gait[1] * 360; //LB
  gait_deg[3] = gait[2] * 360; //RB
  
   return;
}

int dead_zone_speed_tuning =- 10; // Adjustment to tune deadzone speed, MAGIC variable as we are using position control outside the deadzone and speed control inside deadzone so the speed might be different; this variable is manually selected from observation, and it's ok that it's not working well.  
//int different_direction_offset =- 110; // Adjustment to compensate position offset between 2 legs with different rotating direction. Another MAGIC variable that requires manual observation. 

long start;

void setup() {

  DEBUG_SERIAL.begin(115200);

  // Set Port baudrate to 1000000bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(1000000);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // Get DYNAMIXEL information
  
  // Turn off torque when configuring items in EEPROM area:
  for (int i = 0; i <= 0; i++){
    dxl.torqueOff(IDs[i]);
    dxl.setOperatingMode(IDs[i], OP_POSITION);
    dxl.torqueOn(IDs[i]);
    delay(100);
  }

  start = millis();
  clock_init();
  translate_gait_deg();
  delay(3000); 
}


long last_time = 0;
int time_step = 100;
bool in_dead_zone[] = {0, 0, 0, 0}; // 0 = not, 1 = in.

void loop() {
  
  // Please refer to e-Manual(http://emanual.robotis.com/docs/en/parts/interface/dynamixel_shield/) for available range of value. 
  // Set Goal Position in RAW value:
  long elapsed = millis() - start;
  
  if (elapsed - last_time > time_step){
    last_time = elapsed;
    for (int i = 0; i < total_legs; i++){
      float desired_pos = get_desired_angle(i, elapsed);
      if (Directions[i] == 1) desired_pos = 360.0 - desired_pos;
        
      print_position(elapsed, i,desired_pos);
      
      if (in_dead_zone[i] == 0){

        
        if (desired_pos < 300)
          dxl.setGoalPosition(IDs[i], desired_pos, UNIT_DEGREE);
        else{
          in_dead_zone[i] = 1;

          float present_speed = dxl.getPresentVelocity(IDs[i]);
          dxl.torqueOff(IDs[i]);
          dxl.setOperatingMode(IDs[i], OP_VELOCITY);
          dxl.torqueOn(IDs[i]);
          //1 rpm=6 deg/s=9 unit
          //1 unit= 2/3 deg/s
          delay(10);
          if(Directions[i] == 0)
            dxl.setGoalVelocity(IDs[i], 1.5 * omega_fast() + dead_zone_speed_tuning);
          else
            dxl.setGoalVelocity(IDs[i], 1024 + 1.5 * omega_fast() + dead_zone_speed_tuning);
          
        }
        delay(10);   
      }
      else{
        int current_pos = dxl.getPresentPosition(IDs[i], UNIT_DEGREE);
        bool flag_temp = 0;
        
        
        if(Directions[i] == 0)
            flag_temp = current_pos > 20;
         else
            flag_temp = current_pos < 280;
          
        
        if (flag_temp && desired_pos > 0 && desired_pos < 300){
          in_dead_zone[i] = 0;
          dxl.torqueOff(IDs[i]);
          dxl.setOperatingMode(IDs[i], OP_POSITION);
          dxl.torqueOn(IDs[i]);
          //1 rpm=6 deg/s=9 unit
          //1 unit= 2/3 deg/s
          delay(10);
          dxl.setGoalPosition(IDs[i], desired_pos, UNIT_DEGREE);
        }
        
      }
    }
  }
  
}
