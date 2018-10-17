
 /* 
  *  A Simple RC airplane autopilot using a Teensy LC with Prop Shield
  *  

   Added altimeter with version 4.  Altitude hold term is 0.0 for version 5 for glider.
      !!! also commented out the call to altitude_hold function
   Test plane is now a ASW28 glider. Want launch mode as I threw it into the ground on 1st attempts.
   
   The LED drivers on the prop shield are used to drive servos for aileron and elevator output.
   Incoming signals to pins 20 22 23 need to be level changed to 3 volts with FET's or
      a CD4050 or 74LVC245.

   Current test is mode 0 - direct control pass through mode
                   mode 1 - limit bank and pitch angles. Otherwise direct control.
                   mode 2 - Full fly by wire with submodes of climb, or launch mode.

   This version drives two servos.  A possible improvement could use the LED driver via pin 17 of 
   the Teensy LC to also drive the Rudder servo.   Currently yaw corrections are applied to ailerons.

 */      

const int debug = false;                // controls serial printing
const int four_channels_only = false;    // true if the tx does not have an alt channel.
                                        // if using a 3 or 4 channel radio, the mode will be as hardcoded
                                        // in the calc_mode function

/****   Adjust values **********/
#define AIL_DEAD1 45.0            //  for desired mode 1 flight characteristics. +-n degrees bank allowed.
#define AIL_DEAD2 0.2             //  fly by wire and large deadband does not work
#define AIL_REVERSE 0             //  reverse just the calculated adjustment. 
#define AIL_ZERO_TRIM 1497        // servo pulse time at zero trim setting.  Print out to find the value.
#define AIL_SERVO_GAIN 100        // use 100% gain in tx and adjust here
#define ELE_DEAD1 30.0            // wing attack is added. so 20 and 5 would become -15 to 25 degress.
#define ELE_DEAD2 0.2 
#define ELE_REVERSE 1
#define ELE_ZERO_TRIM 1496        // zero trim tx signal length as received via the CD4050
#define ELE_SERVO_GAIN 100        // auto up ele also limited elsewhere as the elevator hits the rudder on this plane
#define YAW_REVERSE 1
#define ALT_REVERSE 0             // altitude hold direction.  Normally false unless mounted backwards.

#define WING_ATTACK_ANGLE 1.5    // adjust +- for desired level trim speed.

                                 
const int Tdead = 118;   // sticks in trim region.  Value is in usec.   1500 +- Tdead.

//  Pid routine factors.
// not adjustments, see param_setup() now.
float P_ail;
float I_ail;
float D_ail;
float YP_ail;               // Yaw applied to aileron
float ail_setpoint;
float ail_dead;

float P_ele;
// float I_ele;       // removed to allow the nose to drop a bit on power off.
float D_ele;
float ele_setpoint = WING_ATTACK_ANGLE;
float ele_dead;
float ele_alt_hold;
float climb_angle;    // for takeoff and climbing to altitude for gliding

float ave_gz;   // yaw gyro applied to ailerons



// not normally changed after wiring is setup.  These are the input signals. ( need to be at 3 volts logic ).
#define AIL_PIN 23
#define ELE_PIN 22
#define AUX_PIN 20

/*****************************************/

//#include <PWMServo.h>  // only has about 5 us steps.  1000 us / 180 degrees.  12 bits 50 hz -> 20000 us / 4096
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
int          base_ail, base_ele, base_aux;   // working copy of the volatile values
int          auto_ail, auto_ele;             // calculated servo commands
int          stick_ail, stick_ele;           // another copy as base is changed for fly by wire

float roll, pitch, heading;
int mode;
int insane = 0;
int imu_fail;
float altimeter;
float altitude_fixed;
int full_up;             // for takeoff mode


 /***********************************************************************/
void param_setup(){
static unsigned long last_mode2_time;   // last time we changed into mode 2
static int mode2_state;  // 0-normal fly by wire, 1-climb 20 deg, 2-launch, 3-launch

  ele_alt_hold = 0.0; full_up = 0; climb_angle = 0.0;  // reset mode 2 params

  if( mode == 0 ){                     // normal R/C control.  Passthrough mode
     ail_dead = 190.0;
     ele_dead = 190.0;
      // gain of 10 gives full servo travel at 50 degrees bank past dead zone
     P_ail = 0.0;  I_ail = 0.0;  D_ail = 0.0;  YP_ail = 0.0;
     P_ele = 0.0;  D_ele = 0.0;
     ail_setpoint = 0;
     ele_setpoint = WING_ATTACK_ANGLE;
  }

  if( mode == 1 ){                     // normal R/C control with bank angle limits. Safe mode.
     ail_dead = AIL_DEAD1;
     ele_dead = ELE_DEAD1;
      // gain of 10 gives full servo travel at 50 degrees bank past dead zone
     P_ail = 10.0;  I_ail = 0.0;  D_ail = 20.0;  YP_ail = 0.0;
     P_ele = 5.0;  D_ele = 10.0;
     ail_setpoint = 0;
     ele_setpoint = WING_ATTACK_ANGLE;
  }
  
  if( mode == 2 ){         // fly by wire.  Less twitchy with zero D terms?
    ail_dead = AIL_DEAD2;
    ele_dead = ELE_DEAD2;
    P_ail = 5.0;  I_ail = 0.25;  D_ail = 20.0;  YP_ail = 0.4;
    P_ele = 5.0;  D_ele = 10.0;
  }
  
  // switching to mode 2 less than 5 seconds from last change enables mode 2 state change.  otherwise altitide hold mode.
  // does the mode switch need debouncing now or does the transmitter handle that ?
  // have noticed one time an uncommanded change to climbout mode from altitude hold. Why?  Bounce?  A hold from RX?
  if(mode == 2 ){  

    if( (millis() - last_mode2_time ) < 5000 ) ++mode2_state;
    else mode2_state = 0;
    
    mode2_state &= 3;
    last_mode2_time = millis();

    switch( mode2_state ){
      case 0:   ele_alt_hold = 0.0; full_up = 0; climb_angle = 0.0;  break;    // altitude hold was 0.8. now just normal
      case 1:   ele_alt_hold = 0.0; full_up = 0; climb_angle = 20.0;  break;   // climb
      case 2:                                                                  // launch, no break
      case 3:   ele_alt_hold = 0.0; full_up = 1; climb_angle = 20.0;           // launch
                YP_ail = 0.0;  // turn off the yaw to aileron correction so wings don't dig into ground on launch
                P_ail = 10.0;  I_ail = 0.0;  P_ele = 10.0;  // stronger corrections close to ground
                                                            // this will effect rates, test if ok.
      break;
    }
  }
  
}

 
void setup() {
  // put your setup code here, to run once:
  pinMode(AIL_PIN,INPUT_PULLUP);
  pinMode(ELE_PIN,INPUT_PULLUP);
  pinMode(AUX_PIN,INPUT_PULLUP);
  
  pinMode(7,OUTPUT);
  digitalWrite(7,HIGH);     // enable the LED 5 volt drivers on prop board as servo signals
  
  base_ail = user_ail = AIL_ZERO_TRIM;
  base_ele = user_ele = ELE_ZERO_TRIM;
  user_aux = 1500;   // defaults if signals are missing
  
  attachInterrupt( AIL_PIN, ail_service, CHANGE );
  attachInterrupt( ELE_PIN, ele_service, CHANGE );
  if( four_channels_only == false ) attachInterrupt( AUX_PIN, aux_service, CHANGE );

  Aileron.attach(13,1000,2000); 
  Elevator.attach(11,1000,2000);

  if( debug )  Serial.begin(9600);       // debug on
  imu.begin();
  filter.begin(100);
  param_setup();
}

void loop() {
int m;
static int servo_flag;
static int alt_time;

   sanity_check();     // are the input servo signals arriving as expected
   
   m = calc_mode();
   if( m != mode ){    // anything needed for mode change, like change PID parameters
     mode = m;
     param_setup();
     if( mode == 2 ) altitude_fixed = altimeter;   // save current altitude
   }
   
   if( imu_process() ){              // imu lib is a once every 10 ms process
      ++servo_flag;                  // que a write servos at 50 percent duty, every 20 ms
   }
   else servo_flag += fail_safe();   // what if the imu hangs up

   if( servo_flag > 1 ){            // write servos every other time for IMU data
       write_servos();
       servo_flag = 0;
       fail_safe_timer = 0;
       if(++alt_time >= 25 ){       // every 500ms read the new altitude
          altimeter = imu.getAltitude();
          alt_time = 0;
          if( debug ){
             //Serial.print("Altitude: ");  Serial.println(altimeter);
          }
       }
   }

}

int imu_process(){
  float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;
static int debug_counter;

static float ave_gx, ave_gy;

   if (imu.available()) {   // data avail about every 10ms by timers in library. 100 times a second
                            // twice as fast as the servo frame rate.
    imu_fail = 0;
    // Read the motion sensors
    imu.readMotionSensor(ax, ay, az, gx, gy, gz, mx, my, mz);

    // Update the Madgwick filter.  The gyro calibration fudge factors should be moved to defined constants.
      filter.updateIMU(gx-0.43, gy+5.35, gz+0.54, ax, ay, az);
      roll = filter.getRoll();
      pitch = filter.getPitch();
      heading = filter.getYaw(); 

      gz += 0.54;
      ave_gz = (99.0*ave_gz + gz)/100.0;      // gyro on yaw produces final trim for ailerons.
       
      if( debug){
        ave_gx = (99.0*ave_gx + gx)/100.0;
        ave_gy = (99.0*ave_gy + gy)/100.0;
      }
              
      if( debug  && ++debug_counter > 25){
        //Serial.print("G X axis "); Serial.println(ave_gx); 
        //Serial.print("G Y axis "); Serial.println(ave_gy);
        //Serial.print("G Z axis "); Serial.println(ave_gz); Serial.println();
        Serial.print("Roll "); Serial.println(roll);
        Serial.print("Pitch "); Serial.println(pitch);
        Serial.print("Heading "); Serial.println(heading);  Serial.println();
        debug_counter = 0;
      }
    
    return 1;
  }

  return 0;
}

void write_servos(){

  get_base_values();                // get user inputs 1000 to 2000 us
  if( imu_fail == 0 ){
    fly_by_wire();                  // convert user inputs to setpoint angle, else setpoint needs to be zero
    auto_ail_pid( stick_ail,roll );
    auto_ele_pid( stick_ele,pitch );
    base_ail += auto_ail;
    base_ele += auto_ele;
  }
  base_ail = constrain(base_ail,1000,2000);
  base_ele = constrain(base_ele,1000,2000);
  //if( debug ) Serial.println(base_ail);    // or base_ele for debug plotting
  base_ail = servo_gain( (int32_t)base_ail, AIL_ZERO_TRIM, AIL_SERVO_GAIN );
  Aileron.write(base_ail);
  base_ele = servo_gain( (int32_t)base_ele, ELE_ZERO_TRIM, ELE_SERVO_GAIN );  
  Elevator.write(base_ele);
}

void fly_by_wire(){     // change the setpoints instead of direct control of the servo
float t;
float dir;

    if( mode != 2 ) return;
    
   // change aileron setpoint
    dir = ( AIL_REVERSE ) ? -1.0 : 1.0;
    t = base_ail - AIL_ZERO_TRIM;
    ail_setpoint = 0.2 * t * dir;       // 0.2 * 500ns == can set a 100 degree bank angle but no more.
    ail_setpoint += yaw_correction( stick_ail );
    base_ail = AIL_ZERO_TRIM;            // base to neutral when changing setpoints
   
    dir = ( ELE_REVERSE ) ? -1.0 : 1.0;
    t = base_ele - ELE_ZERO_TRIM;
    ele_setpoint = (0.2 * t * dir) + WING_ATTACK_ANGLE + climb_angle;   // was 0.15
    // if( mode == 2 ) ele_setpoint += altitude_hold( stick_ele ); !!! unused on the asw28 glider
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
  //    Serial.print("Elevator ");  
  //    Serial.print(base_ele); Serial.write(' '); Serial.println(pitch);
  //    Serial.print("Aileron ");
  //    Serial.print(base_ail); Serial.write(' '); Serial.println(roll);
  //    Serial.println();
      debug_counter = 0;
    }
  }

   // more sanity checking when base values are known
   // the new receivers have servo hold when loosing lock, so this code may never come into play
  if( base_ele > 2500 || base_ele < 500 ) base_ele = ELE_ZERO_TRIM;      
  if( base_ail > 2500 || base_ail < 500 ) base_ail = AIL_ZERO_TRIM;

  // a copy to save the stick positions before they are modified by the fly_by_wire function.
  stick_ail = base_ail;
  stick_ele = base_ele;
  
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
  else if( four_channels_only ) m = 1;
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

/*****************   Special PID routines with deadbands  *************************/
void auto_ail_pid( int stick, float val ){
float error;
float dval;
static float last_val;
float result;
static float result_sum;
float dterm;
static float old_dterm;

      error = ail_setpoint - val;
      dval = val - last_val;
      last_val = val;

      result = 0;

      //I term when tx sticks centered.  Does this cause issues when in a coordinated turn?  In a turn
      // the accelerometers may say the wing is level when it isn't.
      if( stick > (1500 - Tdead)  && stick < (1500 + Tdead) ) result_sum += I_ail * error;
      if( I_ail == 0.0 ) result_sum = 0.0;  // Needed for passthrough mode or when I_ail is zero
      result_sum = constrain(result_sum,-Tdead/2,Tdead/2);   // clip to half the trim region
     
      // recalculate the error for the P deadband
      if( error > ail_dead ) error -= ail_dead;
      else if( error < -ail_dead ) error += ail_dead;
      else error = 0;

      result = result + P_ail * error + result_sum;   // + pterm;

      dterm = D_ail * dval;
//      if( dterm > old_dterm + 5 ) old_dterm = dterm - 2.5;      // remove noise by ignoring +-5 changes.
//      else if( dterm < old_dterm - 5 ) old_dterm = dterm + 2.5;
//      else if( old_dterm > 1.0 ) old_dterm -= 0.5;              // leak to zero
//      else if( old_dterm < -1.0 ) old_dterm += 0.5;
//      else old_dterm = 0;

      if( dterm > old_dterm + 8 ) old_dterm += 3;      // try slower attack and decay
      else if( dterm < old_dterm - 8 ) old_dterm -= 3;
      else if( old_dterm > 1.0 ) old_dterm -= 0.3;              // leak to zero
      else if( old_dterm < -1.0 ) old_dterm += 0.3;
      else old_dterm = 0;

      
      // result -=  D_ail * dval;
      result -= old_dterm;                 // this does damping only with D on the controlled value instead of error
                                           // value
      
      auto_ail = constrain(result,-500,500);   // servo correction to be summed with what the pilot is doing.
      if( AIL_REVERSE ) auto_ail = -auto_ail;
      
}

void auto_ele_pid( int stick, float val ){
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

      result = 0;

      // old I term code
     // result = 0;         // calc I term only if user is not using the sticks
     // if( stick > (1500 - Tdead)  && stick < (1500 + Tdead) ) result_sum += I_ele * error;
     // result_sum = constrain(result_sum,-Tdead,Tdead);  

      // recalculate the error for the P deadband
      if( error > ele_dead ) error -= ele_dead;
      else if( error < -ele_dead ) error += ele_dead;
      else error = 0;

      result = result + P_ele * error;   // + result_sum

      dterm = D_ele * dval;
  //    if( dterm > old_dterm + 5 ) old_dterm = dterm - 2.5;      // remove noise
  //    else if( dterm < old_dterm - 5 ) old_dterm = dterm + 2.5;
  //    else if( old_dterm > 1.0 ) old_dterm -= 0.5;
  //    else if( old_dterm < -1.0 ) old_dterm += 0.5;
  //    else old_dterm = 0;

      if( dterm > old_dterm + 8 ) old_dterm += 3;      // try slower attack and decay
      else if( dterm < old_dterm - 8 ) old_dterm -= 3;
      else if( old_dterm > 1.0 ) old_dterm -= 0.3;              // leak to zero
      else if( old_dterm < -1.0 ) old_dterm += 0.3;
      else old_dterm = 0;

      
      // result -=  D_ele * dval;
      result -= old_dterm;
       
      auto_ele = constrain(result,-500,500);

      // launch mode full up elevator until get hands on the controls.
      // Not tested when Ele is not reversed in the TX.
      if( (stick > (1500+Tdead)) || (stick < (1500-Tdead)) ) full_up = 0;
      if( auto_ele < -Tdead ) full_up = 0;        // wants to go down, nose too high  
      if( full_up ) auto_ele = 500;               // careful if enabled, launch only
      
      if( ELE_REVERSE ) auto_ele = -auto_ele;

}


int servo_gain( int32_t val, int zero, int gain_){

   val -= zero;
   val *= gain_;
   val /= 100;
   val += zero;
   return val;
}



 // no auto control on rudder, so add yaw correction to ailerons by moving the setpoint
 // this routine hunts left and right and sometimes loses course
/*
 * 
float yaw_correction( int stick ){
static float course_;                    
float error;
static int last_stick;
static int debug_counter;
static float yaw;

     if( insane ) return 0.0;                         // prevent flyaway on loss of signal
      
   // establish if want a new heading
     if( stick > (last_stick + 30) || stick < (last_stick - 30) ){   //
        last_stick = stick;
        course_ = heading;
     }

    // course_ -= 0.290/25.0;         // heading drift correction from stationary serial print
    // if( course_ < 0 ) course_ += 360.0;
   
     error = course_ - heading;

    // passing 360 to 0 degrees ?
      if( error > 180 ) error -= 360;
      if( error < -180 ) error += 360;
      if( error < -90.0 || error > 90.0 ){   // lost our lock on course, give up and establish a new heading
        course_ = heading;
        return 0.0;
      }
                                                         
      yaw = YP_ail * error;
      yaw = constrain(yaw,-10.0,10.0);     // max 20 degrees bank to correct
      if( YAW_REVERSE ) yaw = -yaw;
      
      if(debug && (++debug_counter > 25)){
    //     Serial.print("Course ");   Serial.print(course_);
    //     Serial.print(" Heading "); Serial.print(heading);
    //     Serial.print(" Correction ");  Serial.println(yaw);
         debug_counter = 0; 
      }
      return yaw; 
}
*/

// simpler yaw correction based only upon gyro.  Returns a bank angle.
float yaw_correction( int stick ){
float yaw;

    // assume pilot knows what he is doing when stick is outside the trim region.
    if( stick < ( AIL_ZERO_TRIM - Tdead ) || stick > (AIL_ZERO_TRIM + Tdead ) ){
      if( ave_gz < -1 ) ave_gz += 1.0;   // ? gentle release ?
      if( ave_gz >  1 ) ave_gz -= 1.0;
    }
    
    yaw = YP_ail * ave_gz;              // gyro running average, too slow?  Too slow will apply correction out of phase.
    yaw = constrain(yaw,-10.0,10.0);    // max bank angle allowed for this correction
    if( YAW_REVERSE ) yaw = -yaw;

    return yaw;
  
}

float altitude_hold( int stick ){       // returns a pitch angle
float alt;

   // if pilot is moving the stick, capture a new value for the altitude to fly at
   if( stick < ( ELE_ZERO_TRIM - Tdead ) || stick > (ELE_ZERO_TRIM + Tdead ) ){
     // altitude_fixed = altimeter;    // maybe not a good idea
   }

   alt = ele_alt_hold * ( altitude_fixed - altimeter );
   alt = constrain( alt, -5.0, 5.0 );     // allow 5 degrees correction
   if( ALT_REVERSE ) alt = -alt;          // think will need to be false 

   return alt;
}

