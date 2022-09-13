#include <Servo.h>
Servo myservo;
int inputPin = A0; // ultrasonic module   ECHO to A0
int outputPin = A1; // ultrasonic module  TRIG to A1
//#define Lpwm_pin  5     //pin of controlling speed---- ENA of motor driver board
//#define Rpwm_pin  6    //pin of controlling speed---- ENB of motor driver board
int pinLB = 4;           //pin of controlling turning---- IN1 of motor driver board
int pinLF = 5;           //pin of controlling turning---- IN2 of motor driver board
int pinRB = 6;          //pin of controlling turning---- IN3 of motor driver board
int pinRF = 7;          //pin of controlling turning---- IN4 of motor driver board
//unsigned char Lpwm_val = 250; //initialized left wheel speed at 250
//unsigned char Rpwm_val = 250; //initialized right wheel speed at 250
int Car_state = 0;           //the working state of car
int myangle;                //defining variable of angle
int pulsewidth;              //defining variable of pulse width
unsigned char DuoJiao = 90;  //initialized angle of motor at 90°
void M_Control_IO_config(void)
{
  pinMode(pinLB, OUTPUT); // /pin 4
  pinMode(pinLF, OUTPUT); // pin 5
  pinMode(pinRB, OUTPUT); // pin 6
  pinMode(pinRF, OUTPUT); // pin 7
  // pinMode(Lpwm_pin,OUTPUT);  // pin 5 (PWM)
  // pinMode(Rpwm_pin,OUTPUT);  // pin 6(PWM)
}
//void Set_Speed(unsigned char Left,unsigned char Right) //function of setting speed
//{
// analogWrite(Lpwm_pin,Left);
//  analogWrite(Rpwm_pin,Right);
//}
void advance()    //  going forward
{
  digitalWrite(pinRB, LOW); //making motor move towards right rear
  digitalWrite(pinRF, LOW);
  digitalWrite(pinLB, LOW);
  digitalWrite(pinLF, HIGH); //making motor move towards left front
  Car_state = 1;
}
void turnR()        //turning right(dual wheel)
{
  digitalWrite(pinRB, LOW); //making motor move towards right rear
  digitalWrite(pinRF, HIGH);
  digitalWrite(pinLB, HIGH);
  digitalWrite(pinLF, LOW); //making motor move towards left front
  Car_state = 4;
}
void turnL()         //turning left(dual wheel)
{
  digitalWrite(pinRB, HIGH); //making motor move towards right rear
  digitalWrite(pinRF, LOW);
  digitalWrite(pinLB, LOW);
  digitalWrite(pinLF, HIGH); //making motor move towards left front
  Car_state = 3;
}
void stopp()        //stop
{
  digitalWrite(pinRB, HIGH);
  digitalWrite(pinRF, HIGH);
  digitalWrite(pinLB, HIGH);
  digitalWrite(pinLF, HIGH);
  Car_state = 5;
}
void back()         //back up
{
  digitalWrite(pinRB, HIGH); //making motor move towards right rear
  digitalWrite(pinRF, HIGH);
  digitalWrite(pinLB, HIGH);
  digitalWrite(pinLF, LOW); //making motor move towards left front
  Car_state = 2;
}



void Self_Control(void)//self-going, ultrasonic obstacle avoidance
{
  int H;
  myservo.write(DuoJiao);
  H = Ultrasonic_Ranging(1);
  delay(300);
  if (Ultrasonic_Ranging(1) < 15)
  {
    stopp();
    delay(100);
    back();
    delay(50);
  }

  if (Ultrasonic_Ranging(1) < 30)
  {
    stopp();
    delay(100);
    myservo.write(0);
    int L = ask_pin_L(2);
    delay(300);
    myservo.write(100);;
    int R = ask_pin_R(3);
    delay(300);

    if (ask_pin_L(2) > ask_pin_R(3))
    {
      back();
      delay(100);
      turnL();
      delay(400);
      stopp();
      delay(50);
      myservo.write(DuoJiao);
      H = Ultrasonic_Ranging(1);
      delay(500);
    }

    if (ask_pin_L(2)  <= ask_pin_R(3))
    {
      back();
      delay(250);
      turnL();
      delay(1600);
      stopp();
      delay(50);
      myservo.write(DuoJiao);
      H = Ultrasonic_Ranging(1);
      delay(300);
    }
    if (ask_pin_L(2)  < 35 && ask_pin_R(3) < 35)
    {
      stopp();
      delay(50);
      back();
      delay(50);
    }
  }
  else
  {
    advance();
  }
}
int Ultrasonic_Ranging(unsigned char Mode)//function of ultrasonic distance detecting ，MODE=1，displaying，no displaying under other situation

{
  int old_distance;
  digitalWrite(outputPin, LOW);
  delayMicroseconds(2);
  digitalWrite(outputPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(outputPin, LOW);
  int distance = pulseIn(inputPin, HIGH);  // reading the duration of high level
  distance = distance / 58; // Transform pulse time to distance
  if (Mode == 1) {
    Serial.print("\n H = ");
    Serial.print(distance, DEC);
    return distance;
  }
  else  return distance;
}
int ask_pin_L(unsigned char Mode)
{
  int old_Ldistance;
  digitalWrite(outputPin, LOW);
  delayMicroseconds(2);
  digitalWrite(outputPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(outputPin, LOW);
  int Ldistance = pulseIn(inputPin, HIGH);
  Ldistance = Ldistance / 58; // Transform pulse time to distance
  if (Mode == 2) {
    Serial.print("\n L = ");
    Serial.print(Ldistance, DEC);
    return Ldistance;
  }
  else  return Ldistance;
}
int ask_pin_R(unsigned char Mode)
{
  int old_Rdistance;
  digitalWrite(outputPin, LOW);
  delayMicroseconds(2);
  digitalWrite(outputPin, HIGH); //
  delayMicroseconds(10);
  digitalWrite(outputPin, LOW);
  int Rdistance = pulseIn(inputPin, HIGH);
  Rdistance = Rdistance / 58; // Transform pulse time to distance
  if (Mode == 3) {
    Serial.print("\n R = ");
    Serial.print(Rdistance, DEC);
    return Rdistance;
  }
  else  return Rdistance;
}

void setup()
{
  myservo.attach(A2);
  M_Control_IO_config();     //motor controlling the initialization of IO
  //Set_Speed(Lpwm_val,Rpwm_val);  //setting initialized speed
  myservo.write(DuoJiao);       //setting initialized motor angle
  pinMode(inputPin, INPUT);      //starting receiving IR remote control signal
  pinMode(outputPin, OUTPUT);    //IO of ultrasonic module
  Serial.begin(9600);            //initialized serial port , using Bluetooth as serial port, setting baud
  stopp();                       //stop
  delay(1000);
}
void loop()
{

  Self_Control();
}
