#include "DualVNH5019MotorShield.h"
#include "PinChangeInt.h"
//#include <avr/interrupt.h>

DualVNH5019MotorShield md;
int incomingByte = 0; // for incoming serial data
bool valid = true;
int inPinL = 11; // 3-> right   11-> left
int inPinR = 3;
volatile unsigned long time0_R,time1_R,time0_L,time1_L=0;
const long constant = 106762;

void stopIfFault()
{
  if (md.getM1Fault())
  {
    Serial.println("M1 fault");
    while(1);
  }
  if (md.getM2Fault())
  {
    Serial.println("M2 fault");
    while(1);
  }

}


void Interrupt_L(void)
{

    time1_L = time0_L;
    time0_L = micros();
    
    //Serial.println(time0_L-time1_L);
}
void Interrupt_R(void)
{

    time1_R = time0_R;
    time0_R = micros();


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
    md.setSpeeds(300,300);
  
}


void loop()
{
//    send data only when you receive data:
//  if (Serial.available() > 0) {
//    // read the incoming byte:
//    incomingByte = Serial.parseInt();
//    if(valid){
//      Serial.println(int(incomingByte));
//      md.setSpeeds(incomingByte,incomingByte);
//      valid = false;
//    }else valid = true;
//  }
  stopIfFault();

if (millis() > 3000)
  md.setSpeeds(250,250);

//long l = 533570.476/(time0_L-time1_L);
//Serial.println(l);
    delay(10);
    noInterrupts();    
    Serial.print(time0_L-time1_L);
    Serial.print("   ");
    Serial.println(time0_R-time1_R);
    interrupts();
    
    
}
