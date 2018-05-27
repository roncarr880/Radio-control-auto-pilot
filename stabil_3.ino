
 /* 
  *  A Simple RC airplane autopilot using a Teensy LC with Prop Shield
  *  
   The LED drivers on the prop shield are used to drive servos for aileron and elevator output.
   Incoming signals to pins 20 22 23 need to be level changed to 3 volts with FET's or
      a CD4050 or 74LVC245.
      
     Test model is a slow stick with ailerons.  ( Hobby King version )

   Current test is mode 0 - limit bank angles, otherwise direct control
                   mode 1 - full fly by wire
                   mode 2 - Add YP term for final trim
                        
 */      

int debug = false;                // controls serial printing

/****   Adjust values **********/
#define AIL_DEAD1 30.0            //  for desired mode 1 flight characteristics. +-30 degrees bank allowed.
#define AIL_DEAD2 0.75            //  5 usec signal input deadband with 0.15 factor in fly_by_wire
#define AIL_REVERSE 0             //  reverse just the calculated adjustment. 
#define AIL_ZERO_TRIM 1497        // servo pulse time at zero trim setting
#define AIL_SERVO_GAIN 70         // use 100% gain in tx and adjust here
#define ELE_DEAD1 20.0            // wing attack is added. so 20 and 5 would become -15 to 25 degress.
#define ELE_DEAD2 5.0            
#define ELE_REVERSE 1
#define ELE_ZERO_TRIM 1496        // zero trim tx signal length as received via the CD4050
#define ELE_SERVO_GAIN 70
#define YAW_REVERSE 0

#define WING_ATTACK_ANGLE 1.5    // adjust +- for desired level trim speed 1st test was 2, tried 1.8
                                 // the slow stick has some positive incidence built in
const int Tdead = 118;   // sticks in trim region.  Value is in usec.   1500 +- Tdead.

//  Pid routine factors.
// not adjustments, see param_setup() now.
float P_ail;
float I_ail;
float D_ail;
float YP_ail;               // Yaw
float ail_setpoint;
float ail_dead;

float P_ele;
float D_ele;
float ele_setpoint = WING_ATTACK_ANGLE;
float ele_dead;



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
int          base_ail, base_ele, base_aux;   // working copy of the volatile values
int          auto_ail, auto_ele;             // calculated servo commands
int          stick_ail, stick_ele;           // another copy as base is changed for fly by wire

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

  if( debug )  Serial.begin(9600);       // debug on
  imu.begin();
  filter.begin(100);
  param_setup();
}

void loop() {
int m;
static int servo_flag;

   sanity_check();     // are the input servo signals arriving as expected
   
   m = calc_mode();
   if( m != mode ){    // anything needed for mode change, like change PID parameters
     mode = m;
     param_setup();
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
static int debug_counter;

static float ave_gx, ave_gy, ave_gz;

   if (imu.available()) {   // data avail about every 10ms by timers in library. 100 times a second
                            // twice as fast as the servo frame rate.
    imu_fail = 0;
    // Read the motion sensors
    imu.readMotionSensor(ax, ay, az, gx, gy, gz, mx, my, mz);

    // Update the Madgwick filter
      filter.updateIMU(gx-0.43, gy+5.35, gz+0.54, ax, ay, az);
      roll = filter.getRoll();
      pitch = filter.getPitch();
      heading = filter.getYaw(); 
      
      if( debug){
        ave_gx = (99.0*ave_gx + gx)/100.0;
        ave_gy = (99.0*ave_gy + gy)/100.0;
        ave_gz = (99.0*ave_gz + gz)/100.0;
      }
              
      if( debug  && ++debug_counter > 25){
        Serial.print("Heading "); Serial.println(heading); 
        Serial.print("G X axis "); Serial.println(ave_gx); 
        Serial.print("G Y axis "); Serial.println(ave_gy);
        Serial.print("G Z axis "); Serial.println(ave_gz); Serial.println();
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
  base_ele = constrain(base_ele,1200,2000);   //limit up elevator. hits rudder on up ele
  //if( debug ) Serial.println(base_ail);    // or base_ele for debug plotting
  base_ail = servo_gain( (int32_t)base_ail, AIL_ZERO_TRIM, AIL_SERVO_GAIN );
  Aileron.write(base_ail);
  base_ele = servo_gain( (int32_t)base_ele, ELE_ZERO_TRIM, ELE_SERVO_GAIN );  
  Elevator.write(base_ele);
}

void fly_by_wire(){     // change the setpoints instead of direct control of the servo
float t;
float dir;

    if( mode == 0 ) return;
    
   // change aileron setpoint
    dir = ( AIL_REVERSE ) ? -1.0 : 1.0;
    t = base_ail - AIL_ZERO_TRIM;
    ail_setpoint = 0.15 * t * dir;       // .09 from 45 deg bank allowed / 500 us stick movement
    ail_setpoint += yaw_correction( stick_ail );
    base_ail = AIL_ZERO_TRIM;            // base to neutral when changing setpoints
   
    dir = ( ELE_REVERSE ) ? -1.0 : 1.0;
    t = base_ele - ELE_ZERO_TRIM;
    ele_setpoint = (0.15 * t * dir) + WING_ATTACK_ANGLE;
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

  // a copy to save the stick positions
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
// the trim system has been removed and replaced with full fly by wire
void auto_ail_pid( int stick, float val ){
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
      if( stick > (1500 - Tdead)  && stick < (1500 + Tdead) ) result_sum += I_ail * error;
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
      result -= old_dterm;                 // damping only with D on the controlled value
      
      auto_ail = constrain(result,-400,400);
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

      // Trim system
      result = 0;
     // result = T_ele * error;
     // result = constrain(result,-Tdead,Tdead);

      // old I term code
     // result = 0;         // calc I term only if user is not using the sticks
     // if( stick > (1500 - Tdead)  && stick < (1500 + Tdead) ) result_sum += I_ele * error;
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
       
      auto_ele = constrain(result,-2*Tdead,400);  // limits down elevator to 2 * trim region
      if( ELE_REVERSE ) auto_ele = -auto_ele;

}


int servo_gain( int32_t val, int zero, int gain_){

   val -= zero;
   val *= gain_;
   val /= 100;
   val += zero;
   return val;
}

void param_setup(){

  if( mode == 0 ){
     ail_dead = AIL_DEAD1;
     ele_dead = ELE_DEAD1;
      // gain of 10 gives full servo travel at 50 degrees bank past dead zone
     P_ail = 10.0;  I_ail = 0.0;  D_ail = 20.0;  YP_ail = 0.0;
     P_ele = 5.0;  D_ele = 10.0;
     ail_setpoint = 0;
     ele_setpoint = WING_ATTACK_ANGLE;
  }
  
  if( mode == 1 || mode == 2 ){         // fly by wire
    ail_dead = AIL_DEAD2;
    ele_dead = ELE_DEAD2;
    P_ail = 5.0;  I_ail = 0.0;  D_ail = 20.0;  YP_ail = 0.0;
    P_ele = 5.0;  D_ele = 10.0;
    
  }
  if(mode == 2 ) YP_ail = 20.0/45.0;    // degrees bank for 45 degrees off course
}


 // no auto control on rudder, so add yaw correction to ailerons by moving the setpoint
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

