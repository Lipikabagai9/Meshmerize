#define AIN1 19
#define BIN1 18
#define AIN2 21
#define BIN2 5
#define PWMA 13
#define PWMB 12
#define STBY 4
#define LED 2 
#define INT0 39  //Interrupt pins
#define INT1 36
#define SW1 22
#define SW2 23

float Kp = 0.08;       // 0.04 and 2 good for 100 speed
float Kd = 4.0;
const int numChannels = 6;

const int sensorArray[6] = {32, 25, 14, 33, 26, 27}; // Example pins for ESP32   // 2 and 3 Online , 1 and 4 for PID , 0 and 5 for turns
unsigned int sensorValues[numChannels];

int max_speed=100;

int thresholdl  = 500; //low
int thresholdh  = 2000; //high
int threshold_avg = 1500; // average
int end = 0 ;
int P, D, I, previousError, PIDvalue;
double error;
int lsp = max_speed;
int rsp = max_speed;
char path[100];

int path_length=0;
int flag_final_run = 0;
int flag_turn = 0;
int flag_fake_turn = 0;
int prev_time = millis();

void TurnAvailable();

void setup() {
  
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(STBY,OUTPUT);
  pinMode(SW2, INPUT_PULLDOWN);
  digitalWrite(STBY, HIGH);

  Serial.begin(115200); // Initialize serial communication at 115200 baud

  
  //Attaching the Extreme sensor readings to interupt for Turns
  attachInterrupt(digitalPinToInterrupt(INT0), TurnAvailable, FALLING);
  attachInterrupt(digitalPinToInterrupt(INT1), TurnAvailable, FALLING);
}


void Turn();
void GetSensorData();
void linefollow();
void PID();
void motor1run(int motorSpeed);
void motor2run(int motorSpeed);
void Right();
void Left();
void simplify_path();

void loop()
{
  if(end)
    {
      digitalWrite(LED, HIGH);
      motor1run(0);
      motor2run(0);
      for(int i=0;i<path_length;i++)
      {
        Serial.print(path[i]);
        Serial.print(" ");
      }
      Serial.println("");
      flag_turn = 0;
      if(digitalRead(SW2))
      {
        flag_final_run = 1;
      }
      int counter=0;
      while(flag_final_run){
        digitalWrite(LED, LOW);
        
        // if(counter>path_length)return;
        GetSensorData();
        linefollow();
        if(flag_turn)
        {
          motor1run(0);
          motor2run(0);
          delay(100);
          motor1run(100);
          motor2run(100);
          delay(195);
          if(path[counter]=='L')
          {
            Left();
            counter++;
          }
          else if(path[counter]=='R')
          {
            Right();
            counter++;
          }
          else if(path[counter] == 'E'){
            motor1run(0);
            motor2run(0);
            break;
          }
          else
          {
            counter++;
            GetSensorData();
            linefollow();
          }
          flag_turn = 0;
        }
      }
      flag_final_run=0;
    }
  if(!end){
    GetSensorData();
    if(flag_turn )
    {
      //Serial.println("turn");
      Turn();
      simplify_path();
    }

    linefollow();
    GetSensorData();

    if(sensorValues[0] > 2500 && sensorValues[1] > 2500 && sensorValues[2] > 2500 && sensorValues[3] > 2500 && sensorValues[4] > 2500 && sensorValues[5] > 2500){
      delay(25);
      GetSensorData();
      if(sensorValues[0] > 2500 && sensorValues[1] > 2500 && sensorValues[2] > 2500 && sensorValues[3] > 2500 && sensorValues[4] > 2500 && sensorValues[5] > 2500)
      {
      //Serial.println("back");
      Backward();
      path[path_length]='B';
      path_length++;
      }
    } 
  }
}

void Turn(){
  motor1run(0);
  motor2run(0);
  int left_available = 0;
  int forward_available = 0;
  int right_available = 0; 
  //Check if Left turn exists
  if(sensorValues[0] < thresholdl && (sensorValues[2]< thresholdl ||  sensorValues[3]< thresholdl)){
    left_available = 1;

  }
  //Check if Right turn exists
  if(sensorValues[5] < thresholdl && (sensorValues[2]< thresholdl ||  sensorValues[3]< thresholdl)){
    right_available = 1;
    //Serial.println("RGT");
  }
  motor1run(0);
  motor2run(0);
  delay(100);
  motor1run(100);
  motor2run(100);
  delay(195);

   if(!left_available)
  {
    motor1run(100);
    motor2run(100);
    delay(30);
    if(sensorValues[0] < thresholdl && (sensorValues[2]< thresholdl ||  sensorValues[3]< thresholdl))left_available = 1;
  }
  // Move forward enough to take turn and check forward
  
  // Get sensor data to check for forward
  GetSensorData();
  if((sensorValues[2]< thresholdl ||  sensorValues[3]< thresholdl)){
    forward_available = 1;
  }

  if(sensorValues[0]<thresholdl && sensorValues[1]<thresholdl && sensorValues[2]<thresholdl && sensorValues[3]<thresholdl && sensorValues[4]<thresholdl && sensorValues[5]<thresholdl) //End
  {

    path[path_length]='E';
    path_length++;
    end=1;
    return;
  }

  // LEFT PRIORITIZED
  if(left_available){
   // Serial.println("Left");
    Left();
    path[path_length]='L';
    path_length++;
  }
  else if(forward_available){
    //Serial.println("Forward");
    path[path_length]='F';
    path_length++;
    
  }
  else if(right_available){
    //Serial.println("Right");
    Right();
    path[path_length]= 'R';
    path_length++;
  }
  flag_turn = 0;
  //digitalWrite(LED,LOW);
}

void Left(){
  // 1 sec stop for testing
  // motor1run(0);
  // motor2run(0);
  // delay(1000);

  motor1run(40);
  motor2run(-40);
  GetSensorData();
  while(sensorValues[3]<=threshold_avg){
    GetSensorData();
  }

  GetSensorData();
  while(sensorValues[3]>threshold_avg){
    GetSensorData();
  }
}

void Right(){
  // 1 sec stop for testing
  // motor1run(0);
  // motor2run(0);
  // delay(1000);

  GetSensorData();
  motor1run(-40);
  motor2run(40);
  while(sensorValues[2]<=threshold_avg){
    GetSensorData();
  }

  GetSensorData();
  while(sensorValues[2]>threshold_avg){
    GetSensorData();
  }
}

void Backward(){
  // 1 sec stop for testing
 
  motor1run(100);
  motor2run(100);
  delay(150);
  motor1run(0);
  motor2run(0);

  GetSensorData();
  motor1run(-35);
  motor2run(35);

  while(sensorValues[2]>threshold_avg && sensorValues[3]>threshold_avg){
    GetSensorData();
  }
  motor1run(0);
  motor2run(0);

  flag_turn=0;
}

void GetSensorData(){
  for(int i = 0; i < numChannels; i++){
    sensorValues[i] = analogRead(sensorArray[i]);
    // Serial.print(sensorValues[i]);
    // Serial.print(" ");
  }
  // Serial.println("");
}
void linefollow()
{
    GetSensorData();
    PID();
}
void PID()
{
  int error = 0.0* sensorValues[0] + 0.3 * sensorValues[1] + sensorValues[2] - sensorValues[3] - 0.3 * sensorValues[4] - 0.0* sensorValues[5];

  P = error;
  D = error - previousError;

  PIDvalue = (Kp * P) + (Kd * D);
  previousError = error;
  
  lsp = lsp - PIDvalue;
  rsp = rsp + PIDvalue;
  
  if (lsp > max_speed) {
    lsp = max_speed;
  }
  if (lsp < 0) {
    lsp = 0;
  }
  if (rsp > max_speed) {
    rsp = max_speed;
  }
  if (rsp < 0) {
    rsp = 0;
  }

  motor1run(lsp);
  motor2run(rsp);
}


void motor1run(int motorSpeed) {
  motorSpeed = constrain(motorSpeed, -255, 255);
  if (motorSpeed > 0) {
    digitalWrite(AIN1, 1);
    digitalWrite(AIN2, 0);
    analogWrite(PWMA, motorSpeed);
  } else if (motorSpeed < 0) {
    digitalWrite(AIN1, 0);
    digitalWrite(AIN2, 1);
    analogWrite(PWMA, abs(motorSpeed));
  } else {
    digitalWrite(AIN1, 1);
    digitalWrite(AIN2, 1);
    analogWrite(PWMA, 0);
  }
}

void motor2run(int motorSpeed) {
  motorSpeed = constrain(motorSpeed, -255, 255);
  if (motorSpeed > 0) {
    digitalWrite(BIN1, 1);
    digitalWrite(BIN2, 0);
    analogWrite(PWMB, motorSpeed);
  } else if (motorSpeed < 0) {
    digitalWrite(BIN1, 0);
    digitalWrite(BIN2, 1);
    analogWrite(PWMB, abs(motorSpeed));
  } else {
    digitalWrite(BIN1, 1);
    digitalWrite(BIN2, 1);
    analogWrite(PWMB, 0);
  }
}


void TurnAvailable(){
  flag_turn = 1;
  //digitalWrite(LED, HIGH);
}

void simplify_path()
{
  // Path simplification. The strategy is that whenever we encounter a
  // sequence xBx, we can simplify it by cutting out the dead end.

  if (path_length < 3 || path[path_length - 2] != 'B')      // simplify the path only if the second-to-last turn was a 'B'
    return;
  int total_angle = 0;
  int m;
  // Get the angle as a number between 0 and 360 degrees.
  for (m = 1; m <= 3; m++)
  {
    switch (path[path_length - m])
    {
      case 'R':
        total_angle += 90;
        break;
      case 'L':
        total_angle += 270;
        break;
      case 'B':
        total_angle += 180;
        break;
    }
  }
  // Replace all of those turns with a single one.
  total_angle = total_angle % 360;
  switch (total_angle)
  {
    case 0:
      path[path_length - 3] = 'F';
      break;
    case 90:
      path[path_length - 3] = 'R';
      break;
    case 180:
      path[path_length - 3] = 'B';
      break;
    case 270:
      path[path_length - 3] = 'L';
      break;
  }
  path_length -= 2;
}