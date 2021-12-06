#include <Servo.h> 

/////////////////////////////
// Configurable parameters //
/////////////////////////////

// Arduino pin assignment
#define PIN_LED 9
#define PIN_SERVO 10 
#define PIN_IR A0     

// Framework setting
#define _DIST_TARGET 290
#define _DIST_MIN 85 
#define _DIST_MAX 420 

// Distance sensor
#define _DIST_ALPHA 0.35

// Servo range
#define _DUTY_MIN 550 
#define _DUTY_NEU 1475
#define _DUTY_MAX 2400 

// Servo speed control
#define _SERVO_ANGLE 25
#define _SERVO_SPEED 100

// Event periods
#define _INTERVAL_DIST 20 
#define _INTERVAL_SERVO 20 
#define _INTERVAL_SERIAL 100  

// PID parameters
#define _KP 1.5
#define a 70
#define b 417

//////////////////////
// global variables //
//////////////////////

// Servo instance
Servo myservo; 

// Distance sensor
float dist_target; // location to send the ball
float dist_raw, dist_ema, alpha;   

// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial;
bool event_dist, event_servo, event_serial; 

// Servo speed control
int duty_chg_per_interval; 
int duty_target, duty_curr; 

// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm;


void setup() {
// initialize GPIO pins for LED and attach servo 
  pinMode(PIN_LED, OUTPUT); 
                          
  myservo.attach(PIN_SERVO); 

// initialize global variables
  alpha = _DIST_ALPHA;   
  dist_ema = 0;         
  duty_curr = _DUTY_NEU; 

// move servo to neutral position
  myservo.writeMicroseconds(_DUTY_NEU);
// initialize serial port
  Serial.begin(57600);

// convert angle speed into duty change per interval.
  duty_chg_per_interval = (_DUTY_MAX - _DUTY_MIN) * (_SERVO_SPEED / 180.0) * (_INTERVAL_SERVO / 1000.0);                
  last_sampling_time_dist = 0; 
  last_sampling_time_servo = 0; 
  last_sampling_time_serial = 0;
  event_dist = event_servo = event_serial = false; 
}

void loop() {
/////////////////////
// Event generator //
/////////////////////
  unsigned long time_curr = millis();  
  if(time_curr >= last_sampling_time_dist + _INTERVAL_DIST) {
    last_sampling_time_dist += _INTERVAL_DIST;
    event_dist = true; 
  }
  if(time_curr >= last_sampling_time_servo + _INTERVAL_SERVO) {
    last_sampling_time_servo += _INTERVAL_SERVO;
    event_servo = true;
  }
  if(time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL) {
    last_sampling_time_serial += _INTERVAL_SERIAL;
    event_serial = true;
  }

////////////////////
// Event handlers //
////////////////////

  // get a distance reading from the distance sensor
  if(event_dist) { 
     event_dist = false;
      dist_raw = ir_distance_filtered();  
      if (dist_ema == 0){                  
        dist_ema = dist_raw;               
      }                                    
      else{
        dist_ema = alpha * dist_raw + (1-alpha) * dist_ema;   
      }  
    
  // PID control logic
    error_curr = _DIST_TARGET - dist_raw; 
    pterm = error_curr;                   
    control = _KP * pterm;               
                                         

  // duty_target = f(duty_neutral, control)
    duty_target = _DUTY_NEU + control; 

  //(pass) keep duty_target value within the range of [_DUTY_MIN, _DUTY_MAX]
  
    if(duty_target < _DUTY_MIN) { duty_target = _DUTY_MIN; } 
    else if(duty_target > _DUTY_MAX) { duty_target = _DUTY_MAX; }

  }
  
  if(event_servo) {
    event_servo = false; 
    // adjust duty_curr toward duty_target by duty_chg_per_interval
    if(duty_target > duty_curr) { 
      duty_curr += duty_chg_per_interval; 
      if(duty_curr > duty_target) {duty_curr = duty_target;} 
    }
    else {
      duty_curr -= duty_chg_per_interval; 
      if (duty_curr < duty_target) {duty_curr = duty_target;} 
    }
    // update servo position
    myservo.writeMicroseconds(duty_curr); 

  }
  
  if(event_serial) {
    event_serial = false; 
    Serial.print("Min:0,Low:200,dist:");
    Serial.print(dist_raw); 
    Serial.print(",pterm:");
    Serial.print(pterm);   
    Serial.print(",duty_target:"); 
    Serial.print(duty_target); 
    Serial.print(",duty_curr:"); 
    Serial.print(duty_curr);
    Serial.println(",High:310,Max:2000");
  }
}
float ir_distance(void){ 
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;            
 
}                                             

float ir_distance_filtered(void){
             
  float val = ir_distance();        
  return 100 + 300.0 / (b - a) * (val - a);
}
