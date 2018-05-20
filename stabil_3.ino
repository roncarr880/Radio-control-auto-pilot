
 /* 
  *  A Simple RC airplane autopilot using a Teensy LC with Prop Shield
  *  
   The LED drivers on the prop shield are used to drive servos for aileron and elevator output.
   Incoming signals to pins 20 22 23 need to be level changed to 3 volts with FET's or
      a CD4050 or 74LVC245.

   Removed trim system.  Retained yaw system
      
     Test model is a slow stick with ailerons.  ( Hobby King version )

   Current flying characteristics:  Tx trims seem ineffective.  Model wants to turn either left or right.
   With weight back, does not climb on full power which is desired for this model.  Too much down thrust?

   Current test is mode 0 - limit bank angles 
                   mode 1 - full fly by wire
                   mode 2 - allow the yaw correction on airerons
     
 */      
/*
 * to do
 * 
 * How should channel gain be handled.  The slow stick has 66% on all 3 axis.
 * 
 *   when working, consider giving up mode 0( complete manual control ) and add mode 3 
 *   using the barometer to maintain the current altitude ( barometer adjusts elevator set point ) 
 *   and using yaw readings( adjust ailerons set point ) to fly straight. 
 *     ( Would the air movement cause the barometer to read incorrectly? Baffle needed? )
 *   
 *   Test 3 added an I term and changed the elevator gain 4 to 6 i think
 *   
 */

int debug = false;                // controls serial printing

/****   Adjust values **********/
#define AIL_DEAD1 30.0            //  for desired mode 1 flight characteristics. +-30 degrees bank allowed.
#define AIL_DEAD2 0.2            //  for desired mode 2 flight.  Stronger auto flight control.
#define AIL_REVERSE 0             //  reverse just the calculated adjustment. 
#define AIL_ZERO_TRIM 1497        // servo pulse time at zero trim setting
#define AIL_SERVO_GAIN 66         // use 100% gain in tx and adjust here
#define ELE_DEAD1 20.0            // wing attack is added. so 20 and 5 becomes -15 to 25 degress.
#define ELE_DEAD2 0.2            // allow 10 deg descent angle power off glide
#define ELE_REVERSE 1
#define ELE_ZERO_TRIM 1496        // zero trim tx signal length as received via the CD4050
#define ELE_SERVO_GAIN 60

// trim.  50 usec servo travel for 3 deg bank gives TRIMFACTOR of 0.06 and I gain of 16.6 ( 1/16.6 is 0.06 )
// bank angle of 3 deg * 16.6 = 50.   Trim change of 50 * .06 is bank change of 3 deg.
// for a +-5 degree trim, TRIMFACTOR would be 0.1 and I gain of 10.  Trying that.
// settings T_ail,T_ele; TRIMFACTTOR, and Tdead all interact.  The above numbers are for a Tdead of 50 which
// is the change in usec of the signals to the servos.
// for Tdead of 100, 5 * 20 = 100.  1/20 is 0.05

//#define TRIMFACTOR   0.05        // was .1
#define WING_ATTACK_ANGLE 1.8    // adjust +- for desired level trim speed 1st test was 2, tried 1.8
                                 // the slow stick has some positive incidence built in
const int Tdead = 118;   // sticks in trim region.  Value is in usec.   1500 +- Tdead.

//  Pid routine factors.
// !!! not adjustments now.  If like param_setup then change these to defines and use the defines in param_setup
float P_ail = 7.0;
//const float T_ail = 20.0;         // 5 deg angle for 50 usec( Tdead ) change in servo timing
float I_ail = 0.0;         // an actual I term mode 2 only for now
float D_ail = 5.0;
float YI_ail;
float ail_setpoint = 0.0;   // not an adjustment
float ail_dead = 30.0;      // not an adjustment. Change the define above.

float P_ele = 3.0;
// const float T_ele = 20.0;
float D_ele = 1.0;
float ele_setpoint = WING_ATTACK_ANGLE;   // not an adjustment. Wing attack added elsewhere.
float ele_dead = 20.0;      // not an adjustment. Change the define above.



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
int imu_fail;


 /***********************************************************************/
void setup() {
  // put your setup code here, to run once:
  pinMode(AIL_PIN,INPUT_PULLUP);   // noise when just input when testing disconnected
  pinMode(ELE_PIN,INPUT_PULLUP);
  pinMode(AUX_PIN,INPUT_PULLUP);
  
  pinMode(7,OUTPUT);
  digitalWrite(7,HIGH);     // enable the LED 5 volt drivers on prop board as servo signals
  
  base_ail = user_ail = AIL_ZERO_TRIM;
  base_ele = user_ele = ELE_ZERO_TRIM;
  user_aux = 1500;   // defaults if signals are missing
  
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

  if( debug )  Serial.begin(9600);       // debug on
  imu.begin();
  filter.begin(100);
  param_setup();
}

void loop() {
int m;
static int servo_flag;

   // servo_flag = 0;     // is it time to write out new servo info
   sanity_check();     // are the input servo signals arriving as expected
   
   m = calc_mode();
   if( m != mode ){    // anything needed for mode change, like change PID parameters
     mode = m;
     param_setup();
 //    if( mode > 0 ){
 //      ail_dead = (mode == 2) ? AIL_DEAD2 : AIL_DEAD1;
 //      ele_dead = (mode == 2) ? ELE_DEAD2 : ELE_DEAD1;
 //    }
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
    imu_fail = 0;
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
  if( imu_fail == 0 ){
    fly_by_wire();
    auto_ail_pid( base_ail,roll );
    auto_ele_pid( base_ele,pitch );
    base_ail += auto_ail;
    base_ele += auto_ele;
  }
  base_ail = constrain(base_ail,1000,2000);
  base_ele = constrain(base_ele,1200,2000);   //!!! correct?  elevator hits rudder on up ele
  //if( debug ) Serial.println(base_ail);    // or base_ele for debug plotting
  base_ail = servo_gain( (int32_t)base_ail, AIL_ZERO_TRIM, AIL_SERVO_GAIN );
  Aileron.write(base_ail);
  base_ele = servo_gain( (int32_t)base_ele, ELE_ZERO_TRIM, ELE_SERVO_GAIN );  
  Elevator.write(base_ele);
}

void fly_by_wire(){     // change the setpoints instead of direct control of the servo
float t;
float dir;   // !!! remove if not needed

    if( mode == 0 ) return;
    
   // change aileron setpoint
    dir = ( AIL_REVERSE ) ? -1.0 : 1.0;
   // dir = 1.0;  // !!! not needed here ?
    t = base_ail - AIL_ZERO_TRIM;
    // t = constrain(t,-Tdead,Tdead);
    ail_setpoint = 0.15 * t * dir;       // .09 from 45 deg bank allowed / 500 us stick movement
    base_ail = AIL_ZERO_TRIM;             
   
    dir = ( ELE_REVERSE ) ? -1.0 : 1.0;
   // dir = 1.0;
    t = base_ele - ELE_ZERO_TRIM;
    // t = constrain(t,-Tdead,Tdead);
    ele_setpoint = (0.15 * t * dir) + WING_ATTACK_ANGLE;   // 22.5 deg allowed 0.045
    base_ele = ELE_ZERO_TRIM;                
    
}

void get_base_values(){       // base values are the user input via tx sticks
static int debug_counter;

  if( insane ){
     base_ele = ELE_ZERO_TRIM;
     base_ail = AIL_ZERO_TRIM;    // user input lost, use center sticks value
  }
  else{
     noInterrupts();
     base_ele = user_ele;
     base_ail = user_ail;
     interrupts();
  }
  
  if( debug ){
    if( ++debug_counter > 25 ){
      Serial.print("Elevator ");  
      Serial.print(base_ele); Serial.write(' '); Serial.println(pitch);
      Serial.print("Aileron ");
      Serial.print(base_ail); Serial.write(' '); Serial.println(roll);
      Serial.println();
      debug_counter = 0;
    }
  }

   // more sanity checking when base values are known
   // the new receivers have servo hold when loosing lock, so this code may never come into play
  if( base_ele > 2500 || base_ele < 500 ) base_ele = ELE_ZERO_TRIM;      
  if( base_ail > 2500 || base_ail < 500 ) base_ail = AIL_ZERO_TRIM;
  
 // base_ele = constrain(base_ele,1200,1800);   // allow 200 us for PID changes
 // base_ail = constrain(base_ail,1200,1800);   // comment out to allow full movement as set by servo gain
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
      base_aux = user_aux;     // 3 position switch channel
      interrupts();
      if( base_aux < 1300 ) m = 2;      // may want to reverse these for desired switch position for each mode
      else if( base_aux > 1700 ) m = 0;
      else m = 1;
    }
  }
  return m;
}

int fail_safe(){   // imu should be in a 10ms update schedule. Detect if it is not updating on schedule.

  if( fail_safe_timer > 50 ){
    imu_fail = 1;      // use only the user inputs.
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

      // Trim system
      result = 0;
     // result = T_ail * error;
     // result = constrain(result,-Tdead,Tdead);

      //I term when tx sticks centered
      if( base > (1500 - Tdead)  && base < (1500 + Tdead) ) result_sum += I_ail * error;
      else result_sum = 0;  // zero I when moving the sticks ?
      result_sum = constrain(result_sum,-Tdead/2,Tdead/2);
     
      // recalculate the error for the P deadband
      if( error > ail_dead ) error -= ail_dead;
      else if( error < -ail_dead ) error += ail_dead;
      else error = 0;

      result = result + P_ail * error + result_sum;   // + pterm;

      dterm = D_ail * dval;
      if( dterm > old_dterm + 5 ) old_dterm = dterm - 2.5;      // remove noise
      else if( dterm < old_dterm - 5 ) old_dterm = dterm + 2.5;
      else if( old_dterm > 1.0 ) old_dterm -= 0.5;
      else if( old_dterm < -1.0 ) old_dterm += 0.5;
      else old_dterm = 0;
      
      // result -=  D_ail * dval;
      result -= old_dterm;

      result = yaw_correction( result, base );
      
      auto_ail = constrain(result,-400,400);
      if( AIL_REVERSE ) auto_ail = -auto_ail;
      
      auto_ail = average_ail(auto_ail);
}

void auto_ele_pid( int base, float val ){
float error;
float dval;
static float last_val;
float result;
//static float result_sum;
float dterm;
static float old_dterm;
  

      error = ele_setpoint - val;
      dval = val - last_val;
      last_val = val;

      // Trim system
      result = 0;
     // result = T_ele * error;
     // result = constrain(result,-Tdead,Tdead);

      // old I term code
     // result = 0;         // calc I term only if user is not using the sticks
     // if( base > (1500 - Tdead)  && base < (1500 + Tdead) ) result_sum += I_ele * error;
     // result_sum = constrain(result_sum,-Tdead,Tdead);  

      // recalculate the error for the P deadband
      if( error > ele_dead ) error -= ele_dead;
      else if( error < -ele_dead ) error += ele_dead;
      else error = 0;

      result = result + P_ele * error;   // + pterm;

      dterm = D_ele * dval;
      if( dterm > old_dterm + 5 ) old_dterm = dterm - 2.5;      // remove noise
      else if( dterm < old_dterm - 5 ) old_dterm = dterm + 2.5;
      else if( old_dterm > 1.0 ) old_dterm -= 0.5;
      else if( old_dterm < -1.0 ) old_dterm += 0.5;
      else old_dterm = 0;
      
      // result -=  D_ele * dval;
      result -= old_dterm;
       
      auto_ele = constrain(result,-Tdead,400);  // !!! limits down elevator to trim region
      if( ELE_REVERSE ) auto_ele = -auto_ele;
      auto_ele = average_ele( auto_ele );

}


int average_ail( int val ){   // slow down the servo attack rate if needed.  Average 4 values
static int ave[4];            // will this bump the control surface on fast transitions past the trim boundary?
static int in;                // perhaps can push up the derivative term for stability ?
int i;

   ave[in++] = val;
   in &= 3;
   val = 0;
   for( i = 0; i < 4; ++i ) val += ave[i];
   return ( val >> 2 );
}

int average_ele( int val ){
static int ave[4];            // will this bump the control surface on fast transitions past the trim boundary?
static int in;                // it probably will when changing modes
int i;

   ave[in++] = val;
   in &= 3;
   val = 0;
   for( i = 0; i < 4; ++i ) val += ave[i];
   return ( val >> 2 );  
}

int servo_gain( int32_t val, int zero, int gain_){

   val -= zero;
   val *= gain_;
   val /= 100;
   val += zero;
   return val;
}

void param_setup(){
// int m;

  if( mode == 0 ){
     ail_dead = AIL_DEAD1;
     ele_dead = ELE_DEAD1;
      // gain of 10 gives full servo travel at 50 degrees bank
     P_ail = 10.0;  I_ail = 0.0;  D_ail = 20.0;  YI_ail = 0.0;
     P_ele = 10.0;  D_ele = 10.0;
     ail_setpoint = 0;
     ele_setpoint = WING_ATTACK_ANGLE;
  }
  
  if( mode == 1 || mode == 2 ){         // fly by wire
    ail_dead = AIL_DEAD2;
    ele_dead = ELE_DEAD2;
    P_ail = 5.0;  I_ail = 0.5;  D_ail = 10.0;  YI_ail = 0.0;
    P_ele = 5.0;  D_ele = 10.0;
    
  }
  if( mode == 2 ) YI_ail = 5.0;  // loose limits with yaw factor.  How does this fly?
}

float yaw_correction( float val, int base ){   // add yaw correction to ailerons with leaky integral
static float result_sum;                       // otherwise coordinated turn not detected?
static float last_heading;                     // no auto control on rudder, so added to ailerons
float error;
static int debug_counter;

      error = last_heading - heading;    
      // heading seems to precess when standing still.  Why? And rate changes when not level in roll,pitch.
         error -= 0.3/26.0;
      last_heading = heading;
      if( error < -10.0 || error > 10.0 ) return (val + result_sum);  // ignores passing 360 degrees, 0 degrees
      
      // only calc when sticks are neutral
      if( base > (AIL_ZERO_TRIM - Tdead)  && base < (AIL_ZERO_TRIM + Tdead) ) result_sum += YI_ail * error;
      result_sum = constrain(result_sum,-Tdead,Tdead);
      result_sum = result_sum  - (result_sum / 40.0);   //!!! what should the leak value be?
      if(debug && (++debug_counter > 25)){
       //  Serial.print("Yaw "); Serial.print(heading);  Serial.write(' ');
       //  Serial.print(result_sum); Serial.write(' ');
       //  Serial.print("Trim "); Serial.print(val);
       //  Serial.write(' '); Serial.print(base);   // should counter base, not add
         debug_counter = 0;                       // ie bank right, turn right, get left correction
       //  Serial.print(" Roll ");  Serial.println(roll);
      }
      return (val + result_sum);
}

