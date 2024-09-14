/**
 * YALY FASHION - 3D SCANNER
 * Author: HUA VAN KHOA
 *
 * Reviewer: 
 * 27.1.2024 Timer coding
 */

#include <AccelStepper.h>
#include <EEPROM.h>
/* DEBUG MODE
 * Define DEBUG as 1 to enable Serial output for statements
 */
#define DEBUG 0

#if DEBUG == 0
#define outputDebug(x) Serial.print(x)
#define outputDebugLine(x) Serial.println(x)
#else
#define outputDebug(x)
#define outputDebugLine(x)
#endif

/**
 * Phần 1: Mã Quan trọng
 * 
 * This section contains critical code that should not be changed without review.
 */

/**
 * Set Input 
 * Notice: Interrupt pins for NANO are 2 & 3
 */
#define PROX_SENSOR_01 A1 
#define PROX_SENSOR_02 A2 
/**
 * Set ISR Pin 
 */
#define DRV_ALR        2

/**
 * Set Output
 */
#define S1 6
#define D1 5
#define EN 4
#define LASER 11
#define EDGE_LASER 10
#define BRAKE 9

/**
 * Khai báo thông số chuyển động
 */
#define     SPEED_UP                   192         // (mm/s) Vận tốc đi lên 192
#define     SPEED_DOWN                 192         // (mm/s) Vận tốc đi xuống
#define     SPEED_PITCH                150         // (mm/s) Vận tốc chạy thủ công 
#define     ACCELERATION_UP            800         // (mm/s^2) Gia tốc đi lên  
#define     ACCELERATION_DOWN          120         // (mm/s^2) Gia tốc đi xuống
#define     ACCELERATION_PITCH         150         // (mm/s^2) Gia tốc chạy thủ công 
#define     TEETHCOUNT_INGEAR          20          // (tooth) Số răng trên pulley đầu vào
#define     TEETHCOUNT_TRANSGEAR       40          // (tooth) Số răng trên pulley trung gian
#define     TEETHCOUNT_OUTGEAR         30          // (tooth) Số răng trên pulley đầu ra
#define     PITCH_TOOTH                5           // (mm) Khoảng cách mỗi bước răng
#define     MAX_DISTANCE               2100        // (mm) Khoảng hành trình tối đa
#define     FORWARD_DISTANCE           2020        // (mm) Khoảng cách chạy lên
#define     BACKWARD_DISTANCE          1950        // (mm) Khoảng cách chạy xuống trước tìm điểm home
#define     HOME_OFFSET                5           // (mm) Khoảng cách từ sensor đến vị trí home 
#define     HOME_UP                    20          // (mm) Khoảng cách cần quét điểm home
#define     MOTOR_RUNTIME              15          // (giây) Thời gian không tác động đến cảm biến. Nếu vượt, sẽ báo lỗi.
/**QUY TRÌNH VẬN HÀNH

* b_unknownPosition > b_findHome_T (nhận yêu cầu 'sh') > b_sky_idlePosition > b_runback > b_homeback > b_idlePosition (đợi lệnh 'sm') > b_run > b_top_T > b_finished

* b_unknownPosition:
- Hệ thống không xác định được vị trí gốc. 

* b_findHome: 
- Hệ thống đang tìm kiếm vị trí gốc hoặc điểm tham chiếu. 
- Vị trí gốc là điểm bắt đầu từ đó các hành động hoặc di chuyển khác có thể được tính toán.

* b_sky_idlePosition: 
- Hệ thống đang ở trạng thái nghỉ yên trên không.

* b_idlePosition: 
- Hệ thống đã thành công trong việc tìm thấy vị trí gốc.
- Hiện đang ở trạng thái chờ lệnh tiếp theo.

* b_run: 
- Hệ thống đang thực hiện một nhiệm vụ SCAN.

* b_top: 
- Hệ thống đã đạt đến vị trí cao nhất.

* b_runback: 
- Hệ thống đang di chuyển quay lại.

* b_finished: 
- Hệ thống đã hoàn thành nhiệm vụ.

* b_backhome: 
- Hệ thống đã quay trở lại vị trí gốc.
 */
volatile bool b_unknownPosition = 1;
volatile bool b_timeOut         = 0;
volatile bool b_sky_idlePosition= 0;
volatile bool b_findHome        = 0;
volatile bool b_idlePosition    = 0;
volatile bool b_run             = 0;
volatile bool b_top             = 0;
volatile bool b_runback         = 0;
volatile bool b_finished        = 0;
volatile bool b_backhome        = 0;
volatile bool b_laser           = 0;
void systemState() {
  outputDebug("b_unknownPosition ");
  outputDebugLine(b_unknownPosition);
  outputDebug("b_timeOut ");
  outputDebugLine(b_timeOut);
  outputDebug("b_sky_idlePosition ");
  outputDebugLine(b_sky_idlePosition);
  outputDebug("b_findHome ");
  outputDebugLine(b_findHome);
  outputDebug("b_idlePosition ");
  outputDebugLine(b_idlePosition);
  outputDebug("b_run ");
  outputDebugLine(b_run);
  outputDebug("b_top ");
  outputDebugLine(b_top);    
  outputDebug("b_runback ");
  outputDebugLine(b_runback);    
  outputDebug("b_finished ");
  outputDebugLine(b_finished);    
  outputDebug("b_backhome ");
  outputDebugLine(b_backhome);   
}
void resetState(){
  //outputDebug("Reset all states ");
  b_findHome        = 0;
  b_idlePosition    = 0;
  b_timeOut         = 0;
  b_sky_idlePosition= 0;
  b_run             = 0;
  b_top             = 0;
  b_runback         = 0;
  b_finished        = 0;
  b_backhome        = 0;
  b_laser           = 0;
}

void setState(const char* bitName) {
// Reset all the bits to 0
  outputDebug("SET FLAG ");
  
    if (strcmp(bitName, "b_unknownPosition") == 0) {
    resetState(); 
    b_unknownPosition = 1;
    outputDebugLine(bitName);
  } 
    else if (strcmp(bitName, "b_findHome") == 0 ) {
    resetState(); 
    b_findHome = 1;
    outputDebugLine(bitName);
  }  else if (strcmp(bitName, "b_timeOut") == 0 ) {
    //resetState(); 
    b_timeOut = 1;
    outputDebugLine(bitName);
  } else if (strcmp(bitName, "b_sky_idlePosition") == 0 ) {
    resetState(); 
    b_sky_idlePosition = 1;
  } else if (strcmp(bitName, "b_idlePosition") == 0 ) {
    resetState(); 
    b_idlePosition = 1;
    outputDebugLine(bitName);
  } else if (strcmp(bitName, "b_run") == 0 && b_idlePosition == 1) {
    resetState(); 
    b_run = 1;
    outputDebugLine(bitName);
  } else if (strcmp(bitName, "b_top") == 0 && b_run == 1) {
    resetState(); 
    b_top = 1;
    outputDebugLine(bitName);
  } else if (strcmp(bitName, "b_finished") == 0 && b_top == 1) {
    resetState(); 
    b_finished = 1;
    outputDebugLine(bitName);
  } else if (strcmp(bitName, "b_runback") == 0 && b_sky_idlePosition == 1) {
    resetState(); 
    b_runback = 1;
    outputDebugLine(bitName);
  } else if (strcmp(bitName, "b_backhome") == 0 && b_runback == 1 ) {
    resetState(); 
    b_backhome = 1;
    outputDebugLine(bitName);
  } else if (strcmp(bitName, "b_laser") == 0) {
    resetState(); 
    b_laser = 1;
    outputDebugLine(bitName);
  }
}

/** 
 * Phần 2: Mã Tính Toán
 * 
 * This section contains critical code that should not be changed without review.
 */
/**
 * Set Parameters for MICROSTEPS
 */

/**STEPDRIVER_CONFIG
 * DEFAULT  STEPS_PER_REVOLUTION = 200;            // STEP MOTOR 1.8deg 200steps
 * 1/2    400   MICROSTEPS = 2;   
 * 1/4    800   MICROSTEPS = 4;
 * 1/8    1600  MICROSTEPS = 8;
 * 1/16   3200  MICROSTEPS = 16;                        
 */

const float STEPS_PER_REVOLUTION = 200;
const float MICROSTEPS = 4;
float DISTANCE_PER_REVOLUTION = gearbox2mm(PITCH_TOOTH, TEETHCOUNT_INGEAR, TEETHCOUNT_TRANSGEAR, TEETHCOUNT_OUTGEAR); //mm milimeter
AccelStepper stepper(1, S1, D1);

/**
 * 1. Convert distance to pulse steps
 */
long mm2step(int distance) {
  // Calculate the total number of steps required for the given distance
  float stepsPerMillimeter = (STEPS_PER_REVOLUTION * MICROSTEPS) / DISTANCE_PER_REVOLUTION;
  long  steps              = distance * stepsPerMillimeter;
  return steps;
}
int step2mm(long steps) {
  // Calculate the distance per step
  float distancePerStep = DISTANCE_PER_REVOLUTION / (STEPS_PER_REVOLUTION * MICROSTEPS);

  // Calculate the distance in millimeters
  float distance = steps * distancePerStep;
  return distance;
}
/**
 * 2. Convert teethcount in gearBox to distances
 */
float gearbox2mm(int pitch,float teethcnt_inGear, float teethcnt_transGear, float teethcnt_outGear){
  float gearRatio = (teethcnt_inGear/teethcnt_transGear)*teethcnt_outGear;
  float gearbox2distances = gearRatio * pitch;
  return gearbox2distances;
}

/** 
 * Phần 3: Chương trình điều khiển 
 * 
 * This section contains critical code that should not be changed without review.
 */


/**
 * Chương trình chuyển đổi thuận-đảo đầu vào
 */
bool inputState(int inputPin){
  pinMode(inputPin, INPUT_PULLUP);
  bool currentState = digitalRead(inputPin);
//bool input_currentState = !digitalRead(inputPin); // invert signal
  return currentState;
}

/**
 * Chương trình điều khiển phanh
 */
bool brakeControl(bool state, int time){
  pinMode(BRAKE, OUTPUT);
//bool digitalWrite(EN, stateMicrostep);
  bool Brake_currentState = state;
  switch (Brake_currentState) {
    case 0:
      outputDebugLine("Brake Applying");
      digitalWrite(BRAKE, Brake_currentState); // invert signal
      delay(time);
      outputDebugLine("Brake Applied");
      break;
    case 1:
      outputDebugLine("Brake Releasing");
      digitalWrite(BRAKE, Brake_currentState); // invert signal
      delay(time);
      outputDebugLine("Brake Released");
      break;
    default:
      // Handle any other cases here
      break;
  }
}
/**
 * Chương trình dừng khẩn cấp - cấp I
 */
long debouncing_time = 15; //Debouncing Time in Milliseconds
volatile unsigned long last_micros;
void stopEmergency(){
    if((long)(micros() - last_micros) >= debouncing_time * 1000) {
    while(1){
    outputDebugLine("EMERGENCY STOP");
    Serial.println("pmtf");
    digitalWrite(BRAKE, LOW); // BRAKE APPLY
    digitalWrite(EN,HIGH);
    digitalWrite(S1,HIGH);
    digitalWrite(D1,HIGH);
    digitalWrite(13,HIGH);
    }
    last_micros = micros();
  }
}
/**
 * Chương trình dừng khẩn cấp - cấp II
 */
void mechanicalDamaged(){

    while(1){
    outputDebugLine("MECHANICAL FAILED");
    Serial.println("phf");
    digitalWrite(BRAKE, LOW); // BRAKE APPLY
    digitalWrite(EN,HIGH);
    digitalWrite(S1,HIGH);
    digitalWrite(D1,HIGH);
    digitalWrite(13,HIGH);
    }  
}
/**
 * Chương trình báo lỗi khi mất kết nối thiết bị - cấp III
 */
/*
 * Chương trình ngắt Timer
 */
volatile bool timer=0;
volatile int count;
volatile int countSecond;
ISR (TIMER1_OVF_vect) 
{
  TCNT1 = 40536;
  if(timer)
  {
  count++;
  if(count>=9){
        countSecond++;
        outputDebug(F("Time:"));
        outputDebugLine(countSecond);
        count=0;
    }
  }
  // TRƯỜNG HỢP ĐỘNG CƠ HOẠT ĐỘNG ỔN ĐỊNH TRONG MỖI 15 GIÂY
  if(countSecond>=MOTOR_RUNTIME){

    if(b_findHome==1 ){
      Serial.println("b_findHome_F");
      TIMSK1 = (0 << TOIE1);
      countSecond=0; 
    }

    else if(b_run==1){
      Serial.println("b_top_F");
      TIMSK1 = (0 << TOIE1);
      countSecond=0; 
    }
    else if(b_backhome==1){
      Serial.println("b_backhome_F");
      TIMSK1 = (0 << TOIE1);
      countSecond=0; 
    }
    
    //mechanicalDamaged();
  }
}
bool setTimer(int ctlTimer){
  switch (ctlTimer){
    case 0: // disable timer
    outputDebugLine("TIMER RESET");
    if(b_idlePosition==1){
      Serial.println("b_findHome_T");
    }
    else if(b_top==1){
      Serial.println("b_top_T");
    }
    else if(b_backhome==1){
      Serial.println("b_backhome_T");
    }
    TIMSK1 = (0 << TOIE1);
    countSecond=0; 
    break;
    case 1: // enable timer
    outputDebugLine("SET TIMER");
    timer=1;
    TIMSK1 = (1 << TOIE1); 
    break;
    case 2: // stop timer
    TIMSK1 = (0 << TOIE1);
    countSecond=0; 
    timer=0;
    break;
   }

}

/**
 * Convert logic signal Enable MICROSTEP
 */
bool driverStepEN(bool stateMicrostep){
  bool DriverStep_currentState = stateMicrostep;
  switch (stateMicrostep) {
    case 0:
      brakeControl(DriverStep_currentState, 2000);
      digitalWrite(EN, DriverStep_currentState);
      outputDebugLine("DriveMotor Disable");
      break;
    case 1:
      outputDebugLine("DriveMotor Enabled");
      digitalWrite(EN, DriverStep_currentState);
      brakeControl(DriverStep_currentState, 1000);
      break;
    default:
      // Handle any other cases here
      break;
  }
  return DriverStep_currentState;
}
void resetSpdAccDis(){
    stepper.setSpeed(0);
    stepper.setMaxSpeed(0);
    stepper.setAcceleration(0);
    stepper.move(0);
}
void setSpdAccDis(long setSpeed, long setAcceleration, int setDistanceMove){
    resetSpdAccDis();
    stepper.setSpeed(setSpeed);
    stepper.setMaxSpeed(setSpeed);
    stepper.setAcceleration(setAcceleration);
    stepper.move(setDistanceMove);
}
int laserPWM = EEPROM.read(0);
bool laserControl(bool state){
 
//bool digitalWrite(EN, stateMicrostep);
  bool Laser_currentState = state;
  if (Laser_currentState == 0) {
    outputDebugLine("Laser is OFF");
    analogWrite(LASER, 0); 
  }
  else if(Laser_currentState == 1)
  {
    outputDebugLine("Laser is ON");
    analogWrite(LASER, laserPWM); 
  }
}
bool edge_laserControl(bool state){
  bool edge_laser_currentState = state;
  if (edge_laser_currentState == 0) {
    outputDebugLine("Edge laser is OFF");
    digitalWrite(EDGE_LASER,HIGH); 
  }
  else if(edge_laser_currentState == 1)
  {
    outputDebugLine("Edge laser is ON");
    digitalWrite(EDGE_LASER,LOW); ; 
  }
}


/**TEST_MODE
 * b_idlePosition >|[0,pitchStep,200] > b_finished 
 * int_pitchStep   [1 10 50 100 500]
 */
int int_pitchStep = 1;
int currentPosition = 0; 

void setup() {
 // Interface Setting
  Serial.begin(9600);
  edge_laserControl(1);
  brakeControl(0,500);
  pinMode(DRV_ALR, INPUT_PULLUP);
  attachInterrupt(1, stopEmergency, LOW);
  pinMode(PROX_SENSOR_01, INPUT_PULLUP);
  pinMode(PROX_SENSOR_02, INPUT_PULLUP);
  pinMode(LASER, OUTPUT);
  pinMode(EDGE_LASER,OUTPUT);
  pinMode(EN, OUTPUT);
 // State GPIO Setting
  digitalWrite(EDGE_LASER,LOW);
  digitalWrite(LASER,LOW);
  digitalWrite(EN,LOW);
  setState("b_unknownPosition");
  if(inputState(PROX_SENSOR_01)==1 )
  {
    b_unknownPosition = 0;
    resetState();
    currentPosition = 0;
  }
  // 
     cli();                                
    /* Reset Timer/Counter1 */
    TCCR1A = 0;
    TCCR1B = 0;
    TIMSK1 = 0;
    /* Setup Timer/Counter1 */
    TCCR1B |= (1 << CS11) | (1 << CS10);    // prescale = 64
    TCNT1 = 40536;
    sei();                                  
}

void moveUp() {
  //setTimer(1);
  laserControl(1);
  setSpdAccDis(mm2step(SPEED_UP),mm2step(ACCELERATION_UP),mm2step(FORWARD_DISTANCE));
  bool previousState = false;
  while (stepper.distanceToGo() != 0) {
    stepper.run();
    bool currentState = inputState(PROX_SENSOR_02);
    if (previousState != currentState && currentState == true) {
        setState("b_top");
        //setTimer(0);
        currentPosition = FORWARD_DISTANCE; 
        b_unknownPosition = 0 ;
    }
    previousState = currentState;
    }
  setState("b_sky_idlePosition");
  laserControl(0);
  delay(3000);
  edge_laserControl(1);
}

void runScan() {
//PRECONDITION
  outputDebugLine("Run scan");
  setState("b_run");
  setTimer(1);

//ACTION
  laserControl(1);
  setSpdAccDis(mm2step(SPEED_UP),mm2step(ACCELERATION_UP),mm2step(FORWARD_DISTANCE));
  bool previousState = false;
  while (stepper.distanceToGo() != 0) {
    stepper.run();
    bool currentState = inputState(PROX_SENSOR_02);
    if (previousState != currentState && currentState == true) {
        setState("b_top");
        setTimer(0);
        currentPosition = FORWARD_DISTANCE; 
        b_unknownPosition = 0 ;
    }
    previousState = currentState;
  }
  setState("b_sky_idlePosition");
  laserControl(0);
  /*
  delay(1000); 
  // Chạy ngược trở lại vị trí home
  setState("b_runback");
  setTimer(1);
  setSpdAccDis(mm2step(SPEED_DOWN),mm2step(ACCELERATION_DOWN)/2,-mm2step(BACKWARD_DISTANCE));
  while (stepper.distanceToGo() != 0) {
    stepper.run();
  }
  setTimer(0);
  setState("b_finished");
  // Tìm vị trí home, đến khi có ngắt của PROX_SENSOR_01 >> proxEvent() động cơ sẽ dừng lại
  setSpdAccDis(mm2step(SPEED_DOWN)/8,mm2step(ACCELERATION_DOWN)/6,-mm2step(500));
  setTimer(1);
  previousState = inputState(PROX_SENSOR_01);
  while(  inputState(PROX_SENSOR_01)==0 && countSecond <= 5){
    stepper.run();

  }
  if (previousState == !inputState(PROX_SENSOR_01)) {
      setState("b_backhome");
  }
  */
  delay(3000);
  edge_laserControl(1);
}

void findHome() {
//PRECONDITION
  
  
  setState("b_findHome");
  bool previousState_SENSOR_01;
  bool previousState_SENSOR_02;
  setTimer(1);
//ACTION
  setSpdAccDis(mm2step(SPEED_UP),mm2step(ACCELERATION_UP)/6,mm2step(HOME_UP));
  while (stepper.distanceToGo() != 0) {
    stepper.run();
  }
  previousState_SENSOR_01 = inputState(PROX_SENSOR_01);
  previousState_SENSOR_02 = inputState(PROX_SENSOR_02);
  delay(500); 
  setSpdAccDis(mm2step(SPEED_DOWN)/8,mm2step(ACCELERATION_DOWN)/6,-mm2step(500));
  while(inputState(PROX_SENSOR_01)==0 && inputState(PROX_SENSOR_02)==0 && countSecond != 0 ){
    
    stepper.run();
  }
  if( previousState_SENSOR_01 == !inputState(PROX_SENSOR_01)){
    setState("b_idlePosition");
    setTimer(0);
    currentPosition = HOME_OFFSET;
  }
  else if(previousState_SENSOR_02 == !inputState(PROX_SENSOR_02)) {
    setState("b_sky_idlePosition");
    setTimer(0);
    currentPosition = FORWARD_DISTANCE; 
     setTimer(1);
     setState("b_runback");
  setSpdAccDis(mm2step(SPEED_DOWN),mm2step(ACCELERATION_DOWN)/2,-mm2step(BACKWARD_DISTANCE));
  while (stepper.distanceToGo() != 0) {
    stepper.run();
  }
  setTimer(0);
  // Tìm vị trí home, đến khi có ngắt của PROX_SENSOR_01 >> proxEvent() động cơ sẽ dừng lại
  setSpdAccDis(mm2step(SPEED_DOWN)/8,mm2step(ACCELERATION_DOWN)/6,-mm2step(500));
  setTimer(1);
  setState("b_backhome");
  previousState_SENSOR_01 = inputState(PROX_SENSOR_01);
  while(  inputState(PROX_SENSOR_01)==0 && countSecond <= 5){
    stepper.run();

  }
  if (previousState_SENSOR_01 == !inputState(PROX_SENSOR_01)) {
      setState("b_idlePosition");
      setTimer(0);
      currentPosition = HOME_OFFSET;
  }
  }
  stepper.stop(); 
  edge_laserControl(0);
  delay(250);
  if(b_idlePosition==1)
  {
  setSpdAccDis(mm2step(SPEED_UP),mm2step(ACCELERATION_UP)/2,mm2step(HOME_OFFSET));
  while (stepper.distanceToGo() != 0) 
    {
    stepper.run();
    }
  }
}

void loop() {

const int BUFFER_SIZE = 3; // Adjust buffer size as needed

if (Serial.available() > 0) {
            


  char buffer[BUFFER_SIZE];
  int bytesRead = Serial.readBytesUntil('\n', buffer, BUFFER_SIZE - 1);
  char subCommand = 0;
  char laserLevel = 0;
  buffer[bytesRead] = '\0';
        if (buffer[0] == 's') {
        subCommand = buffer[1];
          memset(buffer, 0, sizeof(buffer));
        }
        if (buffer[0] == 'l') {
        laserLevel = buffer[1];
          memset(buffer, 0, sizeof(buffer));
        }
        else{
          memset(buffer, 0, sizeof(buffer));
          } 
          
        switch (subCommand) {
          case 'A':
          brakeControl(1,500);
          break;

          case 'a':
          brakeControl(0,500);
          break;

          case 'L':
          laserControl(1);
          break;

          case 'l':
          laserControl(0);
          break;

          case 'E':
          driverStepEN(1);
          break;

          case 'e':
          driverStepEN(0);
          break;

          case 'F':
          if( digitalRead(EN)==0)
          {
          driverStepEN(1);
          }
          if(b_unknownPosition==0)
          {
              if (currentPosition + int_pitchStep < FORWARD_DISTANCE) {
              currentPosition = currentPosition + int_pitchStep;
              setSpdAccDis(mm2step(SPEED_PITCH),mm2step(ACCELERATION_PITCH),mm2step(int_pitchStep));
              outputDebug("FORWARD & CURRENT POSITION ");
              outputDebugLine(currentPosition);
            } else if (currentPosition + int_pitchStep >= FORWARD_DISTANCE) {
              outputDebug("maxPosition ");
              outputDebugLine(currentPosition);
            }
          }
          else if (b_unknownPosition==1) 
          {
            setSpdAccDis(mm2step(SPEED_PITCH),mm2step(ACCELERATION_PITCH),mm2step(int_pitchStep));
            outputDebug("unknown Position, FORWARD ");
            outputDebugLine(int_pitchStep);
          }
          while (stepper.distanceToGo() != 0 ) 
            {
            stepper.run();
            }
            break;
            
          case 'B':
          if( digitalRead(EN)==0)
          {
          driverStepEN(1);
          }
          else
          {
          }
          switch (b_unknownPosition) {
          case 0:
            if (currentPosition - int_pitchStep >= 0) {
              currentPosition = currentPosition - int_pitchStep;
              setSpdAccDis(mm2step(SPEED_PITCH), mm2step(ACCELERATION_PITCH),-mm2step(int_pitchStep));
              outputDebug("BACKWARD & CURRENT POSITION ");
              outputDebugLine(currentPosition);
            } else if (currentPosition - int_pitchStep < 0) {
              outputDebug("minPosition ");
              outputDebugLine(currentPosition);
            }
            break;

          case 1:
            setSpdAccDis(mm2step(SPEED_PITCH),mm2step(ACCELERATION_PITCH),-mm2step(int_pitchStep));
            outputDebug("unknownPosition, BACKWARD ");
            if(inputState(PROX_SENSOR_01)==1)
            {
            outputDebugLine("LIMIT REACHED");
            b_unknownPosition=0;
            stepper.move(0);
            }
            else{
            outputDebugLine(int_pitchStep);
            
            }
            break;
          }
          while (stepper.distanceToGo() != 0) {
            stepper.run();
          }
            break;
          case 'S':
            systemState();
            break;
          case 's':
            resetState();
            break;
          case 'm':
            Serial.println("psmo");
            if((currentPosition  + FORWARD_DISTANCE < MAX_DISTANCE)&&(b_idlePosition==1) )
            {
            setTimer(2);
            runScan();
            systemState();
            resetState();
            driverStepEN(0);
            }
            else if (currentPosition  + FORWARD_DISTANCE >= MAX_DISTANCE)
            {
              outputDebugLine("Please turn it down");
            }
            else if(b_idlePosition==0)
            {
               outputDebugLine("Please send the sh command to move to the b_idlePosition.");
            }
            break;
          case 'h':
            Serial.println("psho");
            if(currentPosition + HOME_UP < MAX_DISTANCE )
            {
            if(digitalRead(EN) == 0)
              {
                driverStepEN(1);
              }
            findHome();
            setTimer(1);

              //Serial.println(countSecond);
           
            }
            else if (currentPosition +  HOME_UP >= MAX_DISTANCE)
            {
              outputDebugLine("Please turn it down");
            }

            break;
          case '1':
            int_pitchStep = 1;
            outputDebug("pitch distances:");
            outputDebugLine(int_pitchStep);
            break;
          case '2':
            int_pitchStep = 10;
            outputDebug("pitch distances:");
            outputDebugLine(int_pitchStep);
            break;
          case '3':
            int_pitchStep = 50;
            outputDebug("pitch distances:");
            outputDebugLine(int_pitchStep);
            break;
          case '4':
            int_pitchStep = 100;
            outputDebug("pitch distances:");
            outputDebugLine(int_pitchStep);
            break;
          case '5':
            int_pitchStep = 500;
            outputDebug("pitch distances:");
            outputDebugLine(int_pitchStep);
            break;
        
        
        }
        switch (laserLevel) {
          case '1':
            EEPROM.write(0, 80);
            delay(5);
            laserPWM = EEPROM.read(0);
            outputDebug("Laser Level: ");
            outputDebugLine(laserPWM);
            laserControl(1);
            break;
          case '2':
            EEPROM.write(0, 100);
            delay(5);
            laserPWM = EEPROM.read(0);
            outputDebug("Laser Level: ");
            outputDebugLine(laserPWM);
            laserControl(1);
            break;
          case '3':
             EEPROM.write(0, 150);
             delay(5);
             laserPWM = EEPROM.read(0);
            outputDebug("Laser Level: ");
            outputDebugLine(laserPWM);
            laserControl(1);
            break;
          case '4':
            EEPROM.write(0, 200);
            delay(5);
            laserPWM = EEPROM.read(0);
            outputDebug("Laser Level: ");
            outputDebugLine(laserPWM);
            laserControl(1);
            break;
          case '5':
            EEPROM.write(0, 254);
            delay(5);
            laserPWM = EEPROM.read(0);
            outputDebug("Laser Level: ");
            outputDebugLine(laserPWM);
            laserControl(1);
            break;
        }
    }
    else if(b_idlePosition==1 && countSecond >= 10){
            {
              setState("b_timeOut");
              setTimer(2);
              moveUp();
              systemState();
              resetState();
              driverStepEN(0);
            }
    }
}