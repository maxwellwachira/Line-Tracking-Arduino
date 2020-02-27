int enable1 = 9;
int enable2 = 6;
int In1 = 10;
int In2 = 11;
int In3 = 12;
int In4 = 13;

int pot = A4;
float potVal;

int sensor[4] = {0, 0, 0, 0};
int error, previousError;

float motorSpeed = 120.0;

float PID;
float pid_p = 0.0;
float pid_i = 0.0;
float pid_d = 0.0;

double kp = 12.12;
double ki;
double kd;

void setup()
{
  Serial.begin(9600);

  pinMode(enable1, OUTPUT);
  pinMode(In1, OUTPUT);
  pinMode(In2, OUTPUT);
  pinMode(enable2, OUTPUT);
  pinMode(In3, OUTPUT);
  pinMode(In4, OUTPUT);
}

void loop()
{
  sensor[0] = digitalRead(A0);
  sensor[1] = digitalRead(A1);
  sensor[2] = digitalRead(A2);
  sensor[3] = digitalRead(A3);
  //sensor[4] = digitalRead(A4);

  if((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 1))
  error = 3;
  else if((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 1))
  error = 2;
  else if((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 0))
  error = 1;
  else if((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 0))
  error = 0;
  else if((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 0) && (sensor[3] == 0))
  error = -1;
  else if((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 0) && (sensor[3] == 0))
  error = -2;
  else if((sensor[0] == 1) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0))
  error = -3;
  else if((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0))
  {
    if(error == -3)
    error = -4;
    else if(error == 3)
    error = 4;
  }

  pid_magic();
  motorOutput();
  
}

void pid_magic()
{
  potVal = analogRead(pot);
  kd = ((potVal / 100.0) + 0.0);
  
  pid_p = kp * error;

  if((error > -4) && (error < 4))
  {
    pid_i = pid_i + (ki * error);
  }

  pid_d = kd * (error - previousError);

  PID = pid_p + pid_i + pid_d;

  Serial.print(PID);
  Serial.print("\t");
  Serial.print(kd);
}

void motorOutput()
{
  
  digitalWrite(In1, HIGH);
  digitalWrite(In2, LOW);
  
  digitalWrite(In3, LOW);
  digitalWrite(In4, HIGH);
  
  float LMotorSpeed = motorSpeed + PID;
  float RMotorSpeed = motorSpeed - PID;

  LMotorSpeed = constrain(LMotorSpeed, 0, 255);
  RMotorSpeed = constrain(RMotorSpeed, 0, 255);

  analogWrite(enable2, LMotorSpeed);
  analogWrite(enable1, RMotorSpeed);

  Serial.print(LMotorSpeed);
  Serial.print(" ");
  Serial.print("L");
  Serial.print("\t");
  Serial.print("\t");
  Serial.print(RMotorSpeed);
  Serial.print(" ");
  Serial.println("R");
}
