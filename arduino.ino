#include <Servo.h>

/////////////////////////////
// Configurable parameters //
/////////////////////////////

// Arduino pin assignment
#define PIN_LED 9 // [1234] LED를 아두이노 GPIO 9번 핀에 연결
#define PIN_SERVO 10 // [1352] 서보모터를 아두이노의 10번 핀에 연결
#define PIN_IR A0 // [1352] IR센서를 아두이노 A0 핀에 연결

// Framework setting
#define _DIST_TARGET 215 // [1352]목표값을 탁구공 중심 위치까지 거리 255mm로 Fix
#define _DIST_MIN 85 // [1352] 최소 측정 거리를 100mm로 설정
#define _DIST_MAX 345 // [1352] 측정 거리의 최댓값을 410mm로 설정

// Distance sensor
#define _DIST_ALPHA 0.3 // [2979] 알파값 설정

// Servo range
#define _DUTY_MIN 1000 //[2998] Servo Minimum microseconds
#define _DUTY_NEU 1450 //[2983] Servo 중간값 설정
#define _DUTY_MAX 2000 // [2979] Servo Max값 설정

// Servo speed control
#define _SERVO_ANGLE 25 // [2992] 서보 각 설정
#define _SERVO_SPEED 100 // [2976] 서보 스피드 설정

// Event periods
#define _INTERVAL_DIST 20 // [2987] 거리측정 INTERVAL값 설정
#define _INTERVAL_SERVO 20 // [2980] 서보 INTERVAL값 설정
#define _INTERVAL_SERIAL 100 //[2989] 시리얼 출력 INTERVAL 설정

// PID parameters
#define _KP 1.0 // [3000] K🇵값 초기화
#define _Ki 0.0 // [3000] K¡값 초기화
#define _Kd 45.0 // [3000] Kd값 초기화

//calibrate sensor base value
#define DIST_10C 130 //[2998] sample value for sensor 10cm dist value for calibrate
#define DIST_40C 340 //[2998] sample value for sensor 40cm dist value for calibrate
#define interval 2


//////////////////////
// global variables //
//////////////////////

// Servo instance
Servo myservo; //[2991] create servo object to control a servo

// Distance sensor
float dist_target; // location to send the ball [2976] 공 위치
float dist_min, dist_max, dist_raw, dist_ema, dist_prev, alpha;// [2981] 거리 변수 설정(현재, ema 필터 적용, 직전값) 

// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial; 
bool event_dist, event_servo, event_serial; //[2989] 논리형 변수 설정

// Servo speed control
int duty_chg_per_interval; //[1352] 주기동안 duty 변화량 변수 
int duty_target, duty_curr;//[1352] 서보모터의 목표위치, 서보에 실제 입력할 위치

// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm;

//[2998] initialize variables for Low Pass Filter
unsigned long oldmil; //[2998] old milliseconds var
static long apt = 0;  //[2998] filter sampling time saver
int fc = 15; //[2998] cut off frequency (5~ 15 hz recommended)
float dt = interval/1000.0; //[2998] interval setup
float lambda = 2*PI*fc*dt; //[2998] lambda setup
float calidist = 0.0, filter = 0.0, prev = 0.0; //[2998] setup for filter and prev var’s

void setup() {
  pinMode(PIN_LED, OUTPUT); 
  myservo.attach(PIN_SERVO); //[2998] Servo Attaching 
  myservo.writeMicroseconds(_DUTY_NEU); //[2998] Servo Set to neutral position

   // initialize global variables
//   dist_min = _DIST_MIN; //[2999] dist_min 값 적용
//   dist_max = _DIST_MAX; //[2999] dist_max 값 적용
   dist_ema = 0.0; //[2999] dist_ema 값 초기화
   alpha = _DIST_ALPHA; //[2999] alpha 선언 및 값 적용
   duty_curr = _DUTY_NEU;

   // convert angle speed into duty change per interval.
   duty_chg_per_interval = (_DUTY_MAX - _DUTY_MIN) * ((float)_SERVO_SPEED / 180) * ((float)_INTERVAL_DIST / 1000); //[2985] duty_chg_per_interval 설정
 
   // initialize serial port
   Serial.begin(57600); //[2999] serial port 57600
   duty_chg_per_interval = (_DUTY_MAX - _DUTY_MIN) * (_SERVO_SPEED / 180.0) * (_INTERVAL_SERVO / 1000.0);
   last_sampling_time_dist = 0; 
   last_sampling_time_servo = 0; 
   last_sampling_time_serial = 0;
   event_dist = event_servo = event_serial = false;
}

  

void loop() {
  
  unsigned long time_curr = millis();
  
  if (time_curr >= last_sampling_time_dist + _INTERVAL_DIST){
    last_sampling_time_dist += _INTERVAL_DIST;
    event_dist = true;
  }
  if (time_curr >= last_sampling_time_dist + _INTERVAL_SERVO){
    last_sampling_time_servo += _INTERVAL_SERVO;
    event_servo = true;
  }
  if (time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL){
    last_sampling_time_serial += _INTERVAL_SERIAL;
    event_serial = true;
  }
  
  if(event_dist) {
    event_dist = false;
    dist_raw = ir_distance_filtered();
    if (dist_ema == 0){                  
        dist_ema = dist_raw;               
    } 
    else{
      dist_ema = alpha * dist_raw + (1-alpha) * dist_ema;   
    }
  }
  // PID control logic
  error_curr = _DIST_TARGET - dist_raw;
  pterm = _KP * error_curr;
  dterm = _Kd * (error_curr - error_prev);
  iterm += _Ki * error_curr;
  control = pterm + dterm + iterm;

  // duty_target = f(duty_neutral, control)
  duty_target = _DUTY_NEU + control;
  
  if(duty_target < _DUTY_MIN) duty_target = _DUTY_MIN;  
  
  else if(duty_target > _DUTY_MAX) duty_target = _DUTY_MAX; 
  
  // keep duty_target value within the range of [_DUTY_MIN, _DUTY_MAX]
  if(event_servo) {
    event_servo = false; 
    
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

  if (event_serial) {
    event_serial = false;
    dist_target = 255;
    Serial.print("IR:");
    Serial.print(dist_raw);
    Serial.print(",T:");
    Serial.print(dist_target);
    Serial.print(",P:");
    Serial.print(map(pterm, -1000, 1000, 510, 610));
    Serial.print(",D:");
    Serial.print(map(dterm, -1000, 1000, 510, 610));
    Serial.print(",I:");
    Serial.print(map(iterm, -1000, 1000, 510, 610));
    Serial.print(",DTT:");
    Serial.print(map(duty_target, 1000, 2000, 410, 510));
    Serial.print(",DTC:");
    Serial.print(map(duty_curr, 1000, 2000, 410, 510));
    Serial.println(",-G:245,+G:265,m:0,M:800");
  }
}

float ir_distance(void) {
  float val; //[2998] Announce val 
  float volt = float(analogRead(PIN_IR)); //[2998] Read analog data from PIN_IR
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  float calidist = 100.0 + 300.0 / (DIST_40C - DIST_10C) * (val - DIST_10C);
  return calidist; //[2998] Return val 
    //[2998] Fixed some indentation
}
 
float ir_distance_filtered(void) {
  unsigned long dmil = 0;
  float calidist =ir_distance();
  unsigned long mil = millis();
  if (mil != oldmil) { 
    dmil = mil-oldmil; 
    oldmil = mil;   
  } 
  apt -= dmil;
   
  if (apt <= 0) {  
    apt += interval;
    calidist = analogRead(PIN_IR); 
    filter = lambda / (1+lambda) * calidist + 1 / (1+lambda) * prev; 
    prev = filter;
  }
  return filter;
}
