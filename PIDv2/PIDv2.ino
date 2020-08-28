#include "DualVNH5019MotorShield.h"
#include "PinChangeInt.h"


DualVNH5019MotorShield md;
int incomingByte = 0; // for incoming serial data
bool valid = true;
int inPinL = 11; // 3-> right   11-> left
int inPinR = 3;
volatile unsigned long time0_R=0,time1_R=0,time0_L=0,time1_L=0;
volatile unsigned int enCountL = 0,enCountR = 0;

const float constant = 106714;
const float k1_L = 3, k2_L = -1,k3_L=0;
const float k1_R = 3.5, k2_R = -1.2,k3_R=0;
float error_L[] = {0,0,0}, error_R[] = {0,0,0};
volatile unsigned int index= 0;
float u_L = 100, rpm_L = 0;
float u_R = 100, rpm_R = 0;
int turn_time = 65, startSpeed = 200;
int speed_R = 0, speed_L = 0;
//int counter_L = 0, counter_R = 0;

void Interrupt_L(void)
{
//    if (counter_L > 10){
      enCountL++;
      time1_L = time0_L;
      time0_L = micros();
      //Serial.println(enCountL);
   
      //counter_L = 0;
//    }
//    counter_L++;
    
}
void Interrupt_R(void)
{
//    if (counter_R > 10){
    enCountR++;
    time1_R = time0_R;
    time0_R = micros();
    //counter_R=0;
//    }
//    counter_R++;

}

void setup()
{
  Serial.begin(9600);
  //Serial.println("Dual VNH5019 Motor Shield");
  Serial.setTimeout(50);
  md.init();
  //pinMode(inPin, INPUT);    // sets the digital pin as input
  PCintPort::attachInterrupt(inPinL,Interrupt_L,RISING);
  PCintPort::attachInterrupt(inPinR,Interrupt_R,RISING);
    md.setSpeeds(u_R,u_L);
    
}


void loop()
{
    forward(70, 562);
    delay(200);
  //  left_turn(50,turn_time);
    delay(200);
//    left_turn(50,turn_time);
//    delay(100);
//    right_turn(70, turn_time);
    delay(200);
//    delay(500);
//    left_turn(70, turn_time);
//    delay(500);
//    left_turn(70, turn_time);
//    delay(500);
//    right_turn(70, turn_time);
//    delay(500);
//    right_turn(70, turn_time);
//    delay(500);
//    right_turn(70, turn_time);
//    delay(500);
//    right_turn(70, turn_time);
    while(1);
}



void forward(int setRPM, int num){
    noInterrupts();
    enCountL = 0;
    enCountR = 0;
    interrupts();
    startMotor(0, setRPM);

    while (enCountL<num){
      set_rpm(setRPM);
      speed_L = (u_L + 20.7)*2.778;
      speed_R = (u_R + 22.3)*2.703;
      md.setSpeeds(speed_R, speed_L);
     // count++;
    }
    md.setBrakes(400,400);
    delay(100);
    Serial.print(enCountR);
    Serial.print(" ");
    Serial.println(enCountL);
    //stopMotor(0);
}

void right_turn(int setRPM, int num){
    startMotor(1, setRPM);
    int count = 0;
    while (count < num){
    set_rpm(setRPM);
    speed_L = (u_L + 20.7)*2.778;
    speed_R = (u_R + 22.5)*2.703;
    md.setSpeeds(-speed_R, speed_L);
    count++;
    }
    stopMotor(1);
}
void left_turn(int setRPM, int num){
    startMotor(2, setRPM);
    int count = 0;
    while (count < num){
    set_rpm(setRPM);
    speed_L = (u_L + 18.3)*2.857;
    speed_R = (u_R + 22.3)*2.703;
    md.setSpeeds(speed_R, -speed_L);
    count++;
    }
    stopMotor(2);
}

void startMotor(int dir, int u){
  
  switch(dir){
    case 0:
            speed_L = (u + 20.7)*0.347;//2.778
            speed_R = (u + 22.3)*0.337;//2.703
            md.setSpeeds(speed_R, speed_L);   // forward
            delay(15);
            md.setSpeeds(speed_R*2, speed_L*2);   // forward
            delay(15);
            md.setSpeeds(speed_R*4, speed_L*4);   // forward
            delay(15);
            md.setSpeeds(speed_R*8, speed_L*8);   // forward
            delay(15);
            break;
    case 1: speed_L = (u + 20.7)*2.778;
            speed_R = (u_R + 22.3)*2.703;
            md.setSpeeds(-startSpeed, startSpeed);    // right
            break;
    case 2: speed_R = (u + 22.3)*2.703;
            speed_L = (u_L + 18.3)*2.857;
            md.setSpeeds(startSpeed, -startSpeed);    // left
            break;
    default: break;
  }
    delay(50);
}

void stopMotor(int dir){
    switch(dir){
    case 0: md.setSpeeds(150, 150);   // forward
            break;
    case 1: md.setSpeeds(-150, 150);    // right
            break;
    case 2: md.setSpeeds(150, -150);    // left
            break;
    default: break;
  }
  delay(50);
  md.setSpeeds(0, 0);
}

void set_rpm(float setRPM){
    delay(20);
    noInterrupts();

    rpm_L = constant/float(time0_L-time1_L);
    rpm_R = constant/float(time0_R-time1_R);
    index = (index+1)%3;
    error_L[index] = setRPM - rpm_L;
    error_R[index] = setRPM - rpm_R;
    
    u_R = u_R + k1_R*error_R[index]+k2_R*error_R[(index-1)%3]+k3_R*error_R[(index-2)%3];
    u_L = u_L + k1_L*error_L[index]+k2_L*error_L[(index-1)%3]+k3_L*error_L[(index-2)%3]; 

    
//    Serial.print(" ");
//    Serial.print(u_L);
//    Serial.print(" ");
//    Serial.println(u_R);
    
    interrupts();
}
