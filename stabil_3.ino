
 /* 
  *  A Simple RC airplane autopilot using a Teensy LC with Prop Shield
  *  
   The LED drivers on the prop shield are used to drive servos for aileron and elevator output.
   Incoming signals to pins 20 22 23 need to be level changed to 3 volts with FET's or
      a 74LVTxxxx.
   An AUX input only channel is used to change flight modes.
      Mode 0 - normal unrestricted flight.
      Mode 1 - allowed bank angle and pitch restricted. Slight trim corrections at center sticks.
      Mode 2 - Hands off auto pilot. 
     ( there is no yaw control although that could be added and driven from pin 17 buffered on Teensy LC ).
 */      
/*
 * to do
 *   
 */

/****   Adjust values **********/
#define AIL_DEAD 30.0            //  for desired mode 1 flight characteristics
#define AIL_REVERSE 0
#define ELE_DEAD 20.0            // wing attack is added. so 20 and 5 becomes -15 to 25 degress.
#define ELE_REVERSE 0

#define TRIMFACTOR   0.1         // adjust for min trim change mode 0 to mode 1. always positive.
#define WING_ATTACK_ANGLE 5.0    // adjust +- for desired level trim speed


//  Pid routine factors
float P_ail = 5.0;
float I_ail = 0.1;
float D_ail = 5.0;
float ail_setpoint = 0.0;   // not an adjustment
float ail_dead = 30.0;      // not an adjustment. Change the define above.

float P_ele = 2.0;
float I_ele = 0.1;
float D_ele = 1.0;
float ele_setpoint = 0.0;   // not an adjustment. Wing attack added elsewhere.
float ele_dead = 20.0;      // not an adjustment. Change the define above.

const int Idead = 30;   // sticks in trim region.  Value is in usec.   1500 +- Idead.

// not normally changed after wiring is setup.  These are the input signals. ( need to be at 3 volts logic ).
#define AIL_PIN 23
#define ELE_PIN 22
#define AUX_PIN 20

/*****************************************/

//#include <PWMServo.h>
#include <Servo.h>

#include <NXPMotionSense.h>
#include <MadgwickAHRS.h>
#include <Wire.h>
#include <EEPROM.h>

NXPMotionSense imu;
Madgwick filter;

elapsedMicros ail_time;  
elapsedMicros ele_time;
elapsedMicros aux_time;

elapsedMillis sanity_counter;    // check that we are receiving user inputs
elapsedMillis fail_safe_timer;   // check that imu is working

//elapsedMillis temp_timer;
//PWMServo Aileron2;
Servo Aileron;
Servo Elevator;


volatile int user_ail, user_ele, user_aux;   // user commands via transmitter sticks
int          base_ail, base_ele, base_aux;   // copy of the volatile values
int          auto_ail, auto_ele;

float roll, pitch, heading;
int mode;
int insane = 0;


 /***********************************************************************/
void setup() {
  // put your setup code here, to run once:
  pinMode(AIL_PIN,INPUT);
  pinMode(ELE_PIN,INPUT);
  pinMode(AUX_PIN,INPUT);
  
  pinMode(7,OUTPUT);
  digitalWrite(7,HIGH);     // enable the LED 5 volt drivers on prop board as servo signals
  
  user_ail = user_ele = user_aux = 1500;   // defaults if signals are missing
  base_ail = base_ele = 1500;
  
  attachInterrupt( AIL_PIN, ail_service, CHANGE );
  attachInterrupt( ELE_PIN, ele_service, CHANGE );
  attachInterrupt( AUX_PIN, aux_service, CHANGE );

//Aileron2.attach(3,1000,2000);  // pins 3 and 4 use timer FTM2
//Aileron2.write(90);
  Aileron.attach(13,1000,2000); 
  Elevator.attach(11,1000,2000);

//  delay(1000);   // keep sticks centered
//  noInterrupts();
//   ail_zero = user_ail;
//   ele_zero = user_ele;
//   aux_zero = user_aux;     // not really needed
//  interrupts();

  Serial.begin(9600);       // debug only - remove
  imu.begin();
  filter.begin(100);
}

void loop() {
int m;
static int servo_flag;

   // servo_flag = 0;     // is it time to write out new servo info
   sanity_check();     // are the input servo signals arriving as expected
   
   m = calc_mode();
   if( m != mode ){    // anything needed for mode change, like change PID parameters
    mode = m;
    if( mode > 0 ){
       ail_dead = (mode == 2) ? 3.0 : AIL_DEAD;   // trim deadband or fly free +- some bank angel
    }
   }

   if( imu_process() ){              // imu lib is a once every 10 ms process
      ++servo_flag;                  // que a write servos at 50 percent duty, every 20 ms
   }
   else servo_flag += fail_safe();   // what if the imu hangs up

   if( servo_flag > 1 ){            // write servos every other time for IMU data
       write_servos();
       servo_flag = 0;
       fail_safe_timer = 0;
   }
}

int imu_process(){
  float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;

   if (imu.available()) {   // data avail about every 10ms by timers in library. 100 times a second
                            // twice as fast as the servo frame rate.

    // Read the motion sensors
    imu.readMotionSensor(ax, ay, az, gx, gy, gz, mx, my, mz);

    // Update the Madgwick filter
    filter.updateIMU(gx, gy, gz, ax, ay, az);

    roll = filter.getRoll();
    pitch = filter.getPitch();
    heading = filter.getYaw();  // unused for now
    return 1;
  }

  return 0;
}

void write_servos(){

  get_base_values();
  if( mode > 0 ){
    trim_adjust();
    auto_ail_pid( base_ail,roll );
    auto_ele_pid( base_ele,pitch );
    base_ail += auto_ail;
    base_ele += auto_ele;
  }
  base_ail = constrain(base_ail,1000,2000);
  base_ele = constrain(base_ele,1000,2000);
Serial.println(base_ail); 
  Aileron.write(base_ail);
  Elevator.write(base_ele);
}

void trim_adjust(){     // change the setpoints for trim changes
int t;

   // aileron setpoint
    t = base_ail - 1500;
    t = constrain(t,-Idead,Idead);
    ail_setpoint = TRIMFACTOR * t;       // !!! trim factor adjust to match when changing modes
    base_ail -= t;                // remove trim from stick inputs,
   // ? !!! do we need to reverse the setpoint for aileron servo reversing?
   
  // elevator setpoint with offset from zero somehow.  +5 degrees wing angle?     
    t = base_ele - 1500;
    t = constrain(t,-Idead,Idead);
    ele_setpoint = (TRIMFACTOR * t) + WING_ATTACK_ANGLE; 
    base_ele -= t;                // remove trim from stick inputs,
    
}

void get_base_values(){       // base values are the user input via tx sticks

  if( insane ){
     base_ele = base_ail = 1500;    // user input lost, use center sticks value
  }
  else{
     noInterrupts();
     base_ele = user_ele;
     base_ail = user_ail;
     interrupts();
  }

   // more sanity checking when base values are known
  if( base_ele > 2500 || base_ele < 500 ) base_ele = 1500;      
  if( base_ail > 2500 || base_ail < 500 ) base_ail = 1500;
  
  base_ele = constrain(base_ele,1200,1800);   // allow 200 us for PID changes
  base_ail = constrain(base_ail,1200,1800);
}

void sanity_check(){    // validate the user signals

  if( sanity_counter > 110 ) insane = 1;   // more than 5 missed frames 
  else insane = 0;
}

int calc_mode(){    // 3 modes from a 3 position switch channel
int m;
static unsigned long previous;   // only check few times a second to avoid many many interrupt lockouts
unsigned long current;

  m = mode;
  if( insane ) m = 2;     // user inputs lost, fly on auto pilot only
  else{
    current = millis();
    if(( current - previous ) > 400 ){
      previous = current;
      noInterrupts();
      base_aux = user_aux;
      interrupts();
      if( base_aux < 1300 ) m = 0;
      else if( base_aux > 1700 ) m = 2;
      else m = 1;
    }
  }
  return m;
}

int fail_safe(){   // imu should be in a 10ms update schedule. Detect if it is not updating on schedule.

  if( fail_safe_timer > 50 ){
    mode = 0;      // use only the user inputs.  This mode change is ephemeral.
    return 2;
  }
  return 0;
}

/***************    Pin Change Interrupt Service Functions *******************/
void ail_service(){
   if( digitalReadFast(AIL_PIN) == HIGH ) ail_time = 0;
   else{
    user_ail = ail_time;
    sanity_counter = 0;             // this should reset every 20 ms if all is working
   }
}

void ele_service(){
   if( digitalReadFast(ELE_PIN) == HIGH ) ele_time = 0;
   else user_ele = ele_time;
}

void aux_service(){
   if( digitalReadFast(AUX_PIN) == HIGH ) aux_time = 0;
   else user_aux = aux_time;
}

/*****************   Special PID routines with deadbands and trim system *************************/

void auto_ail_pid( int base, float val ){
float error;
float dval;
static float last_val;
float result;
static float result_sum;
float dterm;
static float old_dterm;
// float pterm;
// static int old_auto_ail;

      error = ail_setpoint - val;
      dval = val - last_val;
      last_val = val;

      result = 0;         // calc I term only if user is not using the sticks
      if( base > (1500 - Idead)  && base < (1500 + Idead) ) result_sum += I_ail * error;
      result_sum = constrain(result_sum,-Idead,Idead);

      // soft proportional inside the deadband
      //pterm = (P_ail / 8.0) * error;

      // recalculate the error for the P deadband
      if( error > ail_dead ) error -= ail_dead;
      else if( error < -ail_dead ) error += ail_dead;
      else error = 0;

      result = result_sum + P_ail * error;   // + pterm;

      dterm = D_ail * dval;
      if( dterm > old_dterm + 5 ) old_dterm = dterm - 2.5;      // remove noise
      else if( dterm < old_dterm - 5 ) old_dterm = dterm + 2.5;
      else if( old_dterm > 1.0 ) old_dterm -= 0.5;
      else if( old_dterm < -1.0 ) old_dterm += 0.5;
      else old_dterm = 0;
      
      // result -=  D_ail * dval;
      result -= old_dterm;

     // error = (result - (float)auto_ail) / 4.0;   // slow down the servo
     // auto_ail += error;
      
      auto_ail = constrain(result,-300,300);
      if( AIL_REVERSE ) auto_ail = -auto_ail;
      
      // there seems to be a 1 us boggle when changing to integer values 
      // but maybe when flying it will not matter. This code makes output jump in steps of two  
     // if( abs(auto_ail - old_auto_ail) == 1 ) auto_ail = old_auto_ail;
     // old_auto_ail = auto_ail;
      
}

void auto_ele_pid( int base, float val ){
  // !!! may want to constrain the values differently plus or minus
  // engine off in mode 2 will stall if it tries to maintain +5 degress positive wing incidence
  
}

