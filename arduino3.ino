#include <Servo.h> 

/////////////////////////////
// Configurable parameters //
/////////////////////////////

// Arduino pin assignment
#define PIN_LED 9
#define PIN_SERVO 10
#define PIN_IR A0

// Framework setting
#define _DIST_TARGET 280
#define _DIST_MIN 100
#define _DIST_MAX 410

// Distance sensor
#define _DIST_ALPHA 0.2

// Servo range
#define _DUTY_MIN 900
#define _DUTY_NEU 1300
#define _DUTY_MAX 1600

// Servo speed control
#define _SERVO_ANGLE 25
#define _SERVO_SPEED 900

// Event periods
#define _INTERVAL_DIST 30
#define _INTERVAL_SERVO 20
#define _INTERVAL_SERIAL 100 

// PID parameters
#define _KP 1.1
#define _KD 9.0 
#define _KI 0.003
#define a 90
#define b 310
#define DELAY 1500
//////////////////////
// global variables //
//////////////////////

// Servo instance
Servo myservo;

// Distance sensor
float dist_ema=0;
float num_list=3;
float dist_raw;  

// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial; 
bool event_dist, event_servo, event_serial; 

// Servo speed control
int duty_chg_per_interval;
int duty_target, duty_curr;

// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm;

void setup() {
myservo.attach(PIN_SERVO);
pinMode(PIN_LED,OUTPUT);
duty_curr = _DUTY_NEU;

// move servo to neutral position
myservo.writeMicroseconds(_DUTY_NEU);

// initialize serial port
Serial.begin(57600);

// convert angle speed into duty change per interval.
  duty_chg_per_interval = (_DUTY_MAX - _DUTY_MIN) * (_SERVO_SPEED / _SERVO_ANGLE ) * (_INTERVAL_SERVO / 1000.0);
}
  

void loop() {
unsigned long time_curr = millis();
if(time_curr >= last_sampling_time_dist + _INTERVAL_DIST){
    last_sampling_time_dist += _INTERVAL_DIST;
    event_dist = true;
}
  
if(time_curr >= last_sampling_time_servo + _INTERVAL_SERVO ){
    last_sampling_time_servo += _INTERVAL_SERVO;
    event_servo = true;
}

if(time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL ){
    last_sampling_time_serial += _INTERVAL_SERIAL;
    event_serial = true;
}

  // get a distance reading from the distance sensor
  if(event_dist) {
     event_dist = false;
     dist_raw = ir_distance_filtered();

  // PID control logic
    error_curr = _DIST_TARGET - dist_raw;
    pterm = _KP * error_curr;
    dterm = _KD * (error_curr - error_prev);
    iterm += _KI * error_curr; 
    control = pterm + dterm + iterm;
    
  // duty_target = f(duty_neutral, control)
    duty_target = _DUTY_NEU + control;

  // keep duty_target value within the range of [_DUTY_MIN, _DUTY_MAX]
    if(duty_target > _DUTY_MAX){
      duty_target = _DUTY_MAX;
    }
    if(duty_target < _DUTY_MIN){
      duty_target = _DUTY_MIN;
    }

  // update error_prev
    error_prev = error_curr;
  }
  
  if(event_servo) {
    event_servo=false;
    // adjust duty_curr toward duty_target by duty_chg_per_interval
    if(duty_target>duty_curr) {
      duty_curr += duty_chg_per_interval;
      if(duty_curr > duty_target) duty_curr = duty_target;
    }
    else {
      duty_curr -= duty_chg_per_interval;
      if(duty_curr < duty_target) duty_curr = duty_target;
    }
    // update servo position
     myservo.writeMicroseconds(duty_curr);
  }   

  if(event_serial) {
    float dist_target=255;
    event_serial = false;
    Serial.print("IR:");
    Serial.print(dist_raw);
    Serial.print(",T:");
    Serial.print(dist_target);
    Serial.print(",P:");
    Serial.print(map(pterm,-1000,1000,510,610));
    Serial.print(",D:");
    Serial.print(map(dterm,-1000,1000,510,610));
    Serial.print(",I:");
    Serial.print(map(iterm,-1000,1000,510,610));
    Serial.print(",DTT:");
    Serial.print(map(duty_target,1000,2000,410,510));
    Serial.print(",DTC:");
    Serial.print(map(duty_curr,1000,2000,410,510));
    Serial.println(",-G:245,+G:265,m:0,M:800");
    
  }
}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return 300.0 / (b - a) * (val - a) + 100;
}

float filtered(void){
  int curread;
  int largeread = 0;
  for (int i = 0; i < num_list; i++) {
    curread = ir_distance();
    if (curread > largeread) { largeread = curread; }
    delayMicroseconds(DELAY);
  }
  return largeread;
}

float ir_distance_filtered(void){
  int curread;
  int lowread = 1024;
  dist_raw = ir_distance(); 
  for (int i = 0; i < num_list; i++) {
    curread = filtered();
    if (curread < lowread) { lowread = curread; }
  }
  dist_ema = _DIST_ALPHA *lowread + (1-_DIST_ALPHA )*dist_ema;
  return dist_ema;
}
