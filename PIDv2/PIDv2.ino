#include "DualVNH5019MotorShield.h"
#include "PinChangeInt.h"


DualVNH5019MotorShield md;
int incomingByte = 0; // for incoming serial data
bool valid = true;
int inPinL = 11; // 3-> right   11-> left
int inPinR = 3;
volatile unsigned long time0_R=0,time1_R=0,time0_L=0,time1_L=0;
volatile unsigned int enCountL = 0,enCountR = 0;
int acc_rate = 20;
const float constant = 106714;
const float k1_L = 0.8, k2_L = -0.15,k3_L=0;
const float k1_R = 1, k2_R = -0.15,k3_R=0;

//const float k1_L = 0.36, k2_L = -0.15,k3_L=0;
//const float k1_R = 0.4, k2_R = -0.16,k3_R=0;


float error_L[] = {0,0,0}, error_R[] = {0,0,0};
volatile unsigned int index= 0;
float u_L = 0, rpm_L = 0;
float u_R = 0, rpm_R = 0;
int turn_time = 65, startSpeed = 200;
signed int speed_R = 0, speed_L = 0;
//int counter_L = 0, counter_R = 0;

void Interrupt_L(void)
{
      enCountL++;
      time1_L = time0_L;
      time0_L = micros();
   
    
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
  Serial.setTimeout(70);
  md.init();
  //pinMode(inPin, INPUT);    // sets the digital pin as input
  PCintPort::attachInterrupt(inPinL,Interrupt_L,RISING);
  PCintPort::attachInterrupt(inPinR,Interrupt_R,RISING);
    
    
}

int deg_to_tick_R(int degree){
  return int((float)degree*335.0/72.0)-60.0;
}
int deg_to_tick_L(int degree){
  return int((float)degree*169.0/36.0)-60;
}

int distance_to_ticks(int dist){
  return int(29.429*(float)dist-33);
}

void loop()
{
//    right_turn(50, degree_to_ticks(90));
//    delay(1000);
//    left_turn(50, degree_to_ticks(90));
//    delay(1000);

    //384->90 degree
  //1633->360 degree
  // stopping ticks ~ 33 ticks


  //1033 ticks -> 35cm
    forward(60,distance_to_ticks(100));
    delay(3000);
//    forward(60,distance_to_ticks(20));
//    delay(3000);
//    forward(60,distance_to_ticks(30));
//    delay(3000);
    right_turn(60, deg_to_tick_R(360));
    delay(3000);
    right_turn(60, deg_to_tick_R(180));
    delay(3000);
    

    
}



void forward(int setRPM, int num){
    enCountL = 0;
    enCountR = 0;
    
    startMotor(0, setRPM);
    u_L = setRPM;
    u_R = setRPM;
    while (enCountL<num){
      set_rpm(setRPM);
      speed_L = (u_L + 20.7)*2.778;
      speed_R = (u_R + 22.3)*2.703;
      md.setSpeeds(speed_R, speed_L);
    }

    stopMotor(0, setRPM);

}

void right_turn(int setRPM, int num){
    enCountL=0;
    enCountR=0;
    
    startMotor(1, setRPM);
    u_L = setRPM;
    u_R = setRPM;
    while (enCountL < num){
    set_rpm(setRPM);
    speed_L = (u_L + 20.7)*2.778;
    speed_R = (u_R + 22.5)*2.703;
    md.setSpeeds(-speed_R, speed_L);
    }
        Serial.print(enCountR);
    Serial.print(" ");
    Serial.println(enCountL);
    stopMotor(1, setRPM);
        Serial.print(enCountR);
    Serial.print(" ");
    Serial.println(enCountL);
    delay(100);
    Serial.print(enCountR);
    Serial.print(" ");
    Serial.println(enCountL);
}
void left_turn(int setRPM, int num){
    enCountL=0;
    enCountR=0;
    
    startMotor(2, setRPM);
    u_L = setRPM;
    u_R = setRPM;
    while (enCountL < num){
    set_rpm(setRPM);
    speed_L = (u_L + 18.3)*2.857;
    speed_R = (u_R + 22.3)*2.703;
    md.setSpeeds(speed_R, -speed_L);
    }
    stopMotor(2, setRPM);
}

void startMotor(int dir, int u){
  
  switch(dir){
    case 0:
            speed_L = (u + 20.7)*0.340;//2.778
            speed_R = (u + 22.3)*0.337;//2.703
            md.setSpeeds(speed_R, speed_L);   // forward
            delay(acc_rate);
            md.setSpeeds(speed_R*2, speed_L*2);   // forward
            delay(acc_rate);
            md.setSpeeds(speed_R*4, speed_L*4);   // forward
            delay(acc_rate);
            md.setSpeeds(speed_R*6, speed_L*6);   // forward
            delay(acc_rate);
            md.setSpeeds(speed_R*8, speed_L*8);   // forward
            break;
    case 1: speed_L = (u + 20.7)*0.347; //2.778     //right
            speed_R = (u + 25.3)*0.337; //2.703
            md.setSpeeds(-speed_R, speed_L);   
            delay(acc_rate);
            md.setSpeeds(-speed_R*2, speed_L*2);   
            delay(acc_rate);
            md.setSpeeds(-speed_R*4, speed_L*4);   
            delay(acc_rate);
            md.setSpeeds(-speed_R*6, speed_L*6);   
            delay(acc_rate);
            md.setSpeeds(-speed_R*8, speed_L*8);   
            break;
    case 2: speed_R = (u + 22.3)*0.337;//2.703
            speed_L = (u + 18.3)*0.375; //2.875  left
            md.setSpeeds(speed_R, -speed_L);   
            delay(acc_rate);
            md.setSpeeds(speed_R*2, -speed_L*2);   
            delay(acc_rate);
            md.setSpeeds(speed_R*4, -speed_L*4);   
            delay(acc_rate);
            md.setSpeeds(speed_R*6, -speed_L*6);   
            delay(acc_rate);
            md.setSpeeds(speed_R*8, -speed_L*8);  
            break;
  }
    delay(acc_rate);
}

void stopMotor(int dir, int u){
    switch(dir){
    case 0: speed_L = (u + 20.7)*0.347;//2.778
            speed_R = (u + 22.3)*0.337;//2.703
            md.setSpeeds(speed_R*4, speed_L*4);   // forward
            delay(acc_rate);
            md.setSpeeds(speed_R*2, speed_L*2);   // forward
            delay(acc_rate);
            md.setSpeeds(speed_R, speed_L);   // forward
//            delay(acc_rate);
//            md.setSpeeds(speed_R, speed_L);   // forward
            break;
    case 1: speed_L = (u + 20.7)*0.347; //2.778
            speed_R = (u + 25.3)*0.337; //2.703
            md.setSpeeds(-speed_R*7, speed_L*7);   // right
            delay(acc_rate);
            md.setSpeeds(-speed_R*6, speed_L*6);   // right
            delay(acc_rate);
            md.setSpeeds(-speed_R*5, speed_L*5);   // right
            delay(acc_rate);
            md.setSpeeds(-speed_R*4, speed_L*4);   // right
            delay(acc_rate);
            md.setSpeeds(-speed_R*3, speed_L*3);   // right
            delay(acc_rate);
            md.setSpeeds(-speed_R*2, speed_L*2);   // right
            delay(acc_rate);
            md.setSpeeds(-speed_R, speed_L);   // right
            break;
    case 2: speed_R = (u + 22.3)*0.337;//2.703
            speed_L = (u + 18.3)*0.375; //2.875
            md.setSpeeds(speed_R*8, -speed_L*8);   // left
            delay(acc_rate);
            md.setSpeeds(speed_R*6, -speed_L*6);   // left
            delay(acc_rate);
            md.setSpeeds(speed_R*4, -speed_L*4);   // left
            delay(acc_rate);
            md.setSpeeds(speed_R*2, -speed_L*2);   // left
            delay(acc_rate);
            md.setSpeeds(speed_R, -speed_L);   // left
            break;
    default: break;
  }
  delay(acc_rate);
  md.setBrakes(400, 400);
}

void set_rpm(float setRPM){
    delay(20);
    noInterrupts();
    rpm_L = constant/float(time0_L-time1_L);
    rpm_R = constant/float(time0_R-time1_R);
    interrupts();
    
    index = (index+1)%3;
    error_L[index] = setRPM - rpm_L;
    error_R[index] = setRPM - rpm_R;
    
    u_R = u_R + k1_R*error_R[index]+k2_R*error_R[(index-1)%3];  //+k3_R*error_R[(index-2)%3];
    u_L = u_L + k1_L*error_L[index]+k2_L*error_L[(index-1)%3];  //+k3_L*error_L[(index-2)%3]; 

//    Serial.print(" ");
//    if (rpm_L > 200)
//      Serial.print("0");
//    else Serial.print(rpm_L);
//    Serial.print(" ");
//    if (rpm_L > 200)
//      Serial.print("0");
//    else Serial.println(rpm_R);
    
}
