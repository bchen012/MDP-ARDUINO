//things to do:
// add range to side (done)
// calibrate what is 0, 1, 2 (done)
// calibrate turn and move (need to do turn)

#include "DualVNH5019MotorShield.h"
#include "PinChangeInt.h"
#define sideF A5
#define sideB A4
#define frontL A0
#define frontR A2
//505 A4 493 A5 right sensors ideal
// 497 A0 505 A2 front sensors ideal
DualVNH5019MotorShield md;
int incomingByte = 0; // for incoming serial data
bool valid = true;
int inPinL = 11, inPinR = 3; // 3-> right   11-> left 
double frontL_cm = 0, frontR_cm = 0, sideF_cm =0.0, sideB_cm=0.0;
double diff_fl = 0, diff_fr = 0, diff_sf = 0, diff_sb = 0, diff_F = 0, diff_S = 0;
double  frontL_raw = 0, frontR_raw= 0, sideF_raw = 0, sideB_raw = 0;
int mult = 70,sample = 30, breakOut = 60, cb = 0;
void front_fix(bool isSideFix = false);
String instruction = "F1F1F1F1F1F1F1F1F1F1F1F1RF1F1R", sensorData = "000000";

volatile unsigned long time0_R=0,time1_R=0,time0_L=0,time1_L=0;
volatile unsigned int enCountL = 0,enCountR = 0;
int acc_rate = 20;
const double constant = 106714;
const double k1_L = 0.8, k2_L = -0.15,k3_L=0;
const double k1_R = 1, k2_R = -0.15,k3_R=0;
double error_L[] = {0,0,0}, error_R[] = {0,0,0};
volatile unsigned int index= 0;
double u_L = 0, rpm_L = 0;
double u_R = 0, rpm_R = 0;
int turn_time = 65, startSpeed = 200;
signed int speed_R = 0, speed_L = 0;

void Interrupt_L(void)
{
      enCountL++;
      time1_L = time0_L;
      time0_L = micros();
   
    
}
void Interrupt_R(void)
{
    enCountR++;
    time1_R = time0_R;
    time0_R = micros();
}

int deg_to_tick_R(int degree){
  return int((double)degree*335.0/72.0)-60.0;
}
int deg_to_tick_L(int degree){
  return int((double)degree*162.0/36.0)-60.0;
}

int distance_to_ticks(int dist){
  return int(29.429*(double)dist-33);
}


void setup()
{
  Serial.begin(9600);
  md.init();
  PCintPort::attachInterrupt(inPinL,Interrupt_L,RISING);
  PCintPort::attachInterrupt(inPinR,Interrupt_R,RISING);
  //initial_setup();
}

void initial_setup(){
  front_dist_fix('0','0');
  delay(1000);
  side_dist_fix('0','0');
  delay(1000);
  right_turn(50, deg_to_tick_R(90));
  delay(1000);
  side_angle_fix('0','0');
}

void loop()
{
//  int a_4 = sensorRead(sample, A3);
//  int a_1 = sensorRead(sample, A4);
//  int a_5 = sensorRead(sample, A5);
//  
//
//  if(a_5 <1000 && a_4<1000 ){
//    Serial.print(a_4);
//    Serial.print(" ");
//    Serial.print(a_1);
//    Serial.print(" ");
//    Serial.println(a_5);
//    delay(500);
//  }

//detect_surrounding();   // need calibrate
//Serial.println(sensorData);
//delay(500);
//send_data();   
//side_dist_fix(sensorData[2],sensorData[0]);
//front_angle_fix(sensorData[2],sensorData[0]);
//front_dist_fix(sensorData[2],sensorData[0]);
//recieve_instruction();
//execute_instructions();
//side_angle_fix('1','0');
//md.setSpeeds(400,0);
//front_dist_fix('0','0');
//front_angle_fix('0','0');
//side_dist_fix('0','0');
}

void detect_surrounding(){
  int a_0 = sensorRead(sample, A0); // range 2
  int a_1 = sensorRead(sample, A1); // range 2
  int a_2 = sensorRead(sample, A2); // range 2
  int a_3 = sensorRead(sample, A3); // range 4
  int a_4 = sensorRead(sample, A4); // range 2
  int a_5 = sensorRead(sample, A5); // range 2
  sensorData[0] = find_closest_2(a_0, 529, 275, 185);
  sensorData[1] = find_closest_2(a_1, 625, 296, 192);
  sensorData[2] = find_closest_2(a_2, 531, 270, 187);
  sensorData[3] = find_closest_4(a_3, 533, 423, 323, 256, 211);
  sensorData[4] = find_closest_2(a_4, 517, 266, 179);
  sensorData[5] = find_closest_2(a_5, 522, 257, 176);
}

char find_closest_2(int reading, int range_0, int range_1, int range_2){
  int diff_0 = abs(reading - range_0);
  int diff_1 = abs(reading - range_1);
  int diff_2 = abs(reading - range_2);
  if (diff_0 < diff_1 && diff_0 < diff_2)
      return '0';
  else if (diff_1 < diff_2)
      return '1';
  else
      return '2';
}

char find_closest_4(int reading, int range_0, int range_1, int range_2, int range_3, int range_4){
  int diff_0 = abs(reading - range_0);
  int diff_1 = abs(reading - range_1);
  int diff_2 = abs(reading - range_2);
  int diff_3 = abs(reading - range_3);
  int diff_4 = abs(reading - range_4);
  if (diff_0 < diff_1 && diff_0 < diff_2 && diff_0 < diff_3 && diff_0 < diff_4)
      return '0';
  else if (diff_1 < diff_2 && diff_1 < diff_3 && diff_1 < diff_4)
      return '1';
  else if (diff_2 < diff_3 && diff_2 < diff_4)
      return '2';
  else if (diff_3 < diff_4)
      return '3';
  else
      return '4';
}

void send_data(){
  Serial.println(sensorData);
}

void recieve_instruction(){
  while (!Serial.available());
  // wait for string
  while (Serial.available()) {
    // read the incoming byte:
      instruction = Serial.readString();
      if (instruction == "End\n"){
        Serial.println("Finished"); // turn off send data
        while(1);
      }
      Serial.println(instruction);
      Serial.println("Received");
  }
}

void execute_instructions(){
  for(int i = 0; i<instruction.length(); i++){
    switch(instruction[i]){
      case 'F': 
        i++;
        forward(60, distance_to_ticks(10*(instruction[i]-'0')));
        break;
      case 'L':
        left_turn(60, deg_to_tick_L(90));
        break;
      case 'R':
        right_turn(60, deg_to_tick_R(90));
        break;
      default:
        break;
    }
    delay(100);
    Serial.println("[b]Finished");
    detect_surrounding();
    side_dist_fix(sensorData[5],sensorData[4]);
    front_angle_fix(sensorData[2],sensorData[0]);
    front_dist_fix(sensorData[2],sensorData[0]);
  }
  
}

  
void side_angle_fix(char front, char back){
   if (front == '2' || back == '2')
      return;
   double front_offset = 0.0, back_offset = 0.0;
   if(front == '1')
      front_offset = 10;
   if (back == '1')
      back_offset = 11;
    cb = 0;
    sideF_raw = sensorRead(sample, sideF);
    sideB_raw = sensorRead(sample, sideB);
    sideF_cm = convertToCM_sf(sideF_raw) - front_offset;
    sideB_cm = convertToCM_sb(sideB_raw) - back_offset;
//    Serial.print(sideF_cm);
//    Serial.print(" ");
//    Serial.println(sideB_cm);
    diff_S = sideF_cm-sideB_cm;
    
    while (abs(diff_S)>0.2 && cb<80){
      if(diff_S<0)
        diff_S=-1;
      else
        diff_S=1;
      md.setSpeeds(diff_S*mult, -diff_S*mult);
      delay(90);
      md.setSpeeds(0,0);
      delay(10);
      sideF_cm = convertToCM_sf(sensorRead(sample, sideF)) - front_offset;
      sideB_cm = convertToCM_sb(sensorRead(sample, sideB)) - back_offset;
      diff_S = sideF_cm-sideB_cm;
      cb++;
    }
  
  md.setBrakes(400,400);
}

void side_dist_fix(char front, char back){
  if (front == '2' || back == '2')
      return;
  sideF_raw = sensorRead(sample, sideF);
  sideB_raw = sensorRead(sample, sideB);
  
  diff_sf = sideF_raw - 562;
  diff_sb = sideB_raw - 520;
  if(front == '1')
    diff_sf = sideF_raw - 268;
  if(back == '1')
    diff_sb = sideB_raw - 263;
  
  if(abs(diff_sf)>15 && abs(diff_sb)>15){
      //turn left
      side_angle_fix(front, back);
      delay(100);
      left_turn(60, deg_to_tick_L(90));
      delay(100);
      front_angle_fix(front, back);
      delay(100);
      front_dist_fix(front, back);
      delay(100);
      front_angle_fix(front , back);
      delay(100);
      //turn right
      right_turn(60, deg_to_tick_R(90));
      delay(100);
      side_angle_fix(front, back);
  }
  
}

void front_angle_fix(char right, char left){
  if (right == '2' || left == '2')
      return;
   double right_offset = 0.5, left_offset = 0;
   if(right == '1')
      right_offset = 10.5;
   if (left == '1')
      left_offset = 10;
  frontL_raw = sensorRead(sample, frontL);
  frontR_raw = sensorRead(sample, frontR);
  frontL_cm = convertToCM_fl(frontL_raw) - left_offset;
  frontR_cm = convertToCM_fr(frontR_raw) - right_offset;
  diff_F = frontL_cm-frontR_cm;  
  cb = 0;
  while (abs(diff_F)>0.2 && cb<breakOut){
    if(diff_F<0)
        diff_F=-1;
      else
        diff_F=1;
      md.setSpeeds(-diff_F*mult, diff_F*mult);
      delay(80);
      md.setSpeeds(0,0);
      delay(10);
    frontL_cm = convertToCM_fl(sensorRead(sample, frontL)) - left_offset;
    frontR_cm= convertToCM_fr(sensorRead(sample, frontR)) - right_offset;
    diff_F = frontL_cm-frontR_cm;
    cb++;
  }
  md.setBrakes(400,400);

}


void front_dist_fix(char right, char left){
  if (right == '2' || left == '2')
      return;
  int lRaw = 0, lLimit = 0, rRaw = 0, rLimit = 0;
  double lMult = 0, rMult = 0;
   //Serial.println("front_dist_fix start");
  if(left == '0'){
    lRaw = 532;
    lLimit = 3;
    lMult = 1;
  }else if(left == '1'){
    lRaw = 264;  
    lLimit = 3; 
    lMult = 1;
  }

  if(right == '0'){
    rRaw = 532;
    rLimit = 3;
    rMult = 1;
  }else if(right == '1'){
    rRaw = 268; 
    rLimit = 3; 
    rMult = 1;
  }

  frontL_raw = sensorRead(sample, frontL);
  frontR_raw = sensorRead(sample, frontR);
  diff_fl = frontL_raw - lRaw;
  diff_fr = frontR_raw - rRaw;
  cb = 0;
  while ((abs(diff_fl)>lLimit || abs(diff_fr)>rLimit) && cb<breakOut){
//    if (diff_fl <  0 )
//        
//    diff_fl = max(45,min(diff_fl,150))*((diff_fl<0)? -1:1);
//    diff_fr = max(45,min(diff_fr,150))*((diff_fr<0)? -1:1);
   // Serial.println(diff_fr);
    if (diff_fl <  0 && diff_fl > -70)
        diff_fl = -70;
    if (diff_fl >  0 && diff_fl < 70)
        diff_fl = 70;
        
    if (diff_fr <  0 && diff_fr > -70)
        diff_fr = -70;
    if (diff_fr >  0 && diff_fr < 70)
        diff_fr = 70;
    
    md.setSpeeds(-diff_fr, -diff_fl);
    delay(90);
    md.setSpeeds(0,0);
    delay(10);
    frontL_raw = sensorRead(sample, frontL);
    frontR_raw = sensorRead(sample, frontR);
//    Serial.print(frontL_raw);
//    Serial.print(" ");
//    Serial.println(frontR_raw);
    diff_fl = frontL_raw - lRaw;
    diff_fr = frontR_raw - rRaw;
    cb++;
  }
    md.setBrakes(400,400);
 //Serial.println("done");
}

//A2
double convertToCM_fr(double val){
  double dist;
  if(val>628)
    dist=1;
  else if(val<=628&&val>618)  //1cm to 2cm
    dist=(val-638.0)/-10.0;
  else if(val<=618&&val>573)  //2cm to 3cm
    dist=(val-708.0)/-45.0;
  else if(val<=573&&val>518)  //3cm to 4cm
    dist=(val-738.0)/-55.0;
  else if(val<=518&&val>482)  //4cm to 5cm
    dist=(val-662.0)/-36.0;
  else if(val<=482&&val>439)  //5cm to 6cm
    dist=(val-697.0)/-43.0;
  else if(val<=439&&val>402)  //6cm to 7cm
    dist=(val-661.0)/-37.0;
  else if(val<=402&&val>377)  //7cm to 8cm
    dist=(val-577.0)/-25.0;
  else if(val<=377&&val>351)  //8cm to 9cm
    dist=(val-585.0)/-26.0;
  else if(val<=351&&val>329)  //9cm to 10cm
    dist=(val-549.0)/-22.0;
  else if(val<=329&&val>309)  //10cm to 11cm
    dist=(val-529.0)/-20.0;
  else if(val<=309&&val>295)  //11cm to 12cm
    dist=(val-463.0)/-14.0;
  else if(val<=295&&val>279)  //12cm to 13cm
    dist=(val-487.0)/-16.0;
  else if(val<=279&&val>265)  //13cm to 14cm
    dist=(val-461.0)/-14.0;
  else if(val<=265&&val>255)  //14cm to 15cm
    dist=(val-405.0)/-10.0;
  else if(val<=255&&val>243)  //15cm to 16cm
    dist=(val-435.0)/-12.0;
  else if(val<=243&&val>232)  //16cm to 17cm
    dist=(val-419.0)/-11.0;
  else if(val<=232&&val>224)  //17cm to 18cm
    dist=(val-368.0)/-8.0;
  else if(val<=224&&val>213)  //18cm to 19cm
    dist=(val-422.0)/-11.0;
  else if(val<=213&&val>205)  //19cm to 20cm
    dist=(val-365.0)/-8.0;
  else
    dist=20.0;
  return dist;
}


//A0
double convertToCM_fl(double val){
  double dist;
  if(val>625)
    dist=1;
  else if(val<=625&&val>617)  //1cm to 2cm
    dist=(val-633.0)/-8.0;
  else if(val<=617&&val>563)  //2cm to 3cm
    dist=(val-725.0)/-54.0;
  else if(val<=563&&val>514)  //3cm to 4cm
    dist=(val-710.0)/-49.0;
  else if(val<=514&&val>467)  //4cm to 5cm
    dist=(val-702.0)/-47.0;
  else if(val<=467&&val>435)  //5cm to 6cm
    dist=(val-627.0)/-32.0;
  else if(val<=435&&val>401)  //6cm to 7cm
    dist=(val-639.0)/-34.0;
  else if(val<=401&&val>376)  //7cm to 8cm
    dist=(val-576.0)/-25.0;
  else if(val<=376&&val>350)  //8cm to 9cm
    dist=(val-584.0)/-26.0;
  else if(val<=350&&val>328)  //9cm to 10cm
    dist=(val-548.0)/-22.0;
  else if(val<=328&&val>312)  //10cm to 11cm
    dist=(val-488.0)/-16.0;
  else if(val<=312&&val>297)  //11cm to 12cm
    dist=(val-477.0)/-15.0;
  else if(val<=297&&val>282)  //12cm to 13cm
    dist=(val-477.0)/-15.0;
  else if(val<=282&&val>267)  //13cm to 14cm
    dist=(val-477.0)/-15.0;
  else if(val<=267&&val>256)  //14cm to 15cm
    dist=(val-421.0)/-11.0;
  else if(val<=256&&val>246)  //15cm to 16cm
    dist=(val-406.0)/-10.0;
  else if(val<=246&&val>238)  //16cm to 17cm
    dist=(val-374.0)/-8.0;
  else if(val<=238&&val>227)  //17cm to 18cm
    dist=(val-425.0)/-11.0;
  else if(val<=227&&val>219)  //18cm to 19cm
    dist=(val-371.0)/-8.0;
  else if(val<=219&&val>211)  //19cm to 20cm
    dist=(val-371.0)/-8.0;
  else
    dist=20.0;
  return dist;
}

//A4
double convertToCM_sb(double val){
 int dist;
  if(val>630)
    dist=3;
  else if(val<=630&&val>601)  //3cm to 4cm
    dist=(val-717.0)/-29.0;
  else if(val<=601&&val>544)  //4cm to 5cm
    dist=(val-829.0)/-57.0;
  else if(val<=544&&val>494)  //5cm to 6cm
    dist=(val-794.0)/-50.0;
  else if(val<=494&&val>447)  //6cm to 7cm
    dist=(val-776.0)/-47.0;
  else if(val<=447&&val>415)  //7cm to 8cm
    dist=(val-671.0)/-32.0;
  else if(val<=415&&val>387)  //8cm to 9cm
    dist=(val-639.0)/-28.0;
  else if(val<=387&&val>358)  //9cm to 10cm
    dist=(val-648.0)/-29.0;
  else if(val<=358&&val>335)  //10cm to 11cm
    dist=(val-588.0)/-23.0;
  else if(val<=335&&val>317)  //11cm to 12cm
    dist=(val-533.0)/-18.0;
  else if(val<=317&&val>302)  //12cm to 13cm
    dist=(val-497.0)/-15.0;
  else if(val<=302&&val>283)  //13cm to 14cm
    dist=(val-549.0)/-19.0;
  else if(val<=283&&val>274)  //14cm to 15cm
    dist=(val-409.0)/-9.0;
  else if(val<=274&&val>259)  //15cm to 16cm
    dist=(val-499.0)/-15.0;
  else if(val<=259&&val>251)  //16cm to 17cm
    dist=(val-387.0)/-8.0;
  else if(val<=251&&val>240)  //17cm to 18cm
    dist=(val-438.0)/-11.0;
  else if(val<=240&&val>228)  //18cm to 19cm
    dist=(val-456.0)/-12.0;
  else if(val<=228&&val>220)  //19cm to 20cm
    dist=(val-380.0)/-8.0;
  else
    dist=20.0;
  return dist;
}
//A5
double convertToCM_sf(double val){
 int dist;
  if(val>627)
    dist=3;
  else if(val<=627&&val>600)  //3cm to 4cm
    dist=(val-708.0)/-27.0;
  else if(val<=600&&val>544)  //4cm to 5cm
    dist=(val-824.0)/-56.0;
  else if(val<=544&&val>491)  //5cm to 6cm
    dist=(val-809.0)/-53.0;
  else if(val<=491&&val>451)  //6cm to 7cm
    dist=(val-731.0)/-40.0;
  else if(val<=451&&val>414)  //7cm to 8cm
    dist=(val-710.0)/-37.0;
  else if(val<=414&&val>381)  //8cm to 9cm
    dist=(val-678.0)/-33.0;
  else if(val<=381&&val>355)  //9cm to 10cm
    dist=(val-615.0)/-26.0;
  else if(val<=355&&val>333)  //10cm to 11cm
    dist=(val-575.0)/-22.0;
  else if(val<=333&&val>311)  //11cm to 12cm
    dist=(val-575.0)/-22.0;
  else if(val<=311&&val>294)  //12cm to 13cm
    dist=(val-515.0)/-17.0;
  else if(val<=294&&val>279)  //13cm to 14cm
    dist=(val-489.0)/-15.0;
  else if(val<=279&&val>264)  //14cm to 15cm
    dist=(val-489.0)/-15.0;
  else if(val<=264&&val>253)  //15cm to 16cm
    dist=(val-589.0)/-21.0;
  else if(val<=253&&val>244)  //16cm to 17cm
    dist=(val-397.0)/-9.0;
  else if(val<=244&&val>233)  //17cm to 18cm
    dist=(val-431.0)/-11.0;
  else if(val<=233&&val>225)  //18cm to 19cm
    dist=(val-377.0)/-8.0;
  else if(val<=225&&val>214)  //19cm to 20cm
    dist=(val-434.0)/-11.0;
  else
    dist=20.0;
  return dist;
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
    delay(200);
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
    //stopMotor(2, setRPM);
    Serial.print(enCountR);
    Serial.print(" ");
    Serial.println(enCountL);
    stopMotor(2, setRPM);
    delay(200);
    Serial.print(enCountR);
    Serial.print(" ");
    Serial.println(enCountL);
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
            md.setSpeeds(speed_R*7, -speed_L*7);   // left
            delay(acc_rate);
            md.setSpeeds(speed_R*6, -speed_L*6);   // left
            delay(acc_rate);
            md.setSpeeds(speed_R*5, -speed_L*5);   // left
            delay(acc_rate);
            md.setSpeeds(speed_R*4, -speed_L*4);   // left
            delay(acc_rate);
            md.setSpeeds(speed_R*3, -speed_L*3);   // left
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

void set_rpm(double setRPM){
    delay(20);
    noInterrupts();
    rpm_L = constant/double(time0_L-time1_L);
    rpm_R = constant/double(time0_R-time1_R);
    interrupts();
    
    index = (index+1)%3;
    error_L[index] = setRPM - rpm_L;
    error_R[index] = setRPM - rpm_R;
    
    u_R = u_R + k1_R*error_R[index]+k2_R*error_R[(index-1)%3];  //+k3_R*error_R[(index-2)%3];
    u_L = u_L + k1_L*error_L[index]+k2_L*error_L[(index-1)%3];  //+k3_L*error_L[(index-2)%3]; 
}

double sensorRead(int n, int sensor){
  double x[n];
  int i;
//  int sum = 0;
  for(i=0;i<n;i++){
    delay(1);
    x[i] = analogRead(sensor);
  }
  mergeSort(x, n);
  return x[n/2];          //Return Median
}

void mergeSort(double ar[], int size){
    int i;
    if(size<2)
        return;
    else
    {
        int mid = size/2;
        double left[mid];
        double right[size-mid];
        for(i=0; i<mid; i++)
        {
            left[i] = ar[i];
        }
        for(i=mid; i<size; i++)
        {
            right[i-mid] = ar[i];
        }
        mergeSort(left, mid);
        mergeSort(right, size-mid);
        merge(ar, left, right, size);
    }
}

void merge(double ar[], double left[], double right[], int size){
    int i=0, j=0, k=0;
    int mid=size/2;
    while(i<mid && j<size-mid)
    {
        if(left[i]>right[j])
        {
            ar[k] = right[j];
            j++;
        }
        else
        {
            ar[k] = left[i];
            i++;
        }
        k++;
    }
    while(i<mid)
    {
        ar[k]=left[i];
        k++; i++;
    }
    while(j<size-mid)
    {
        ar[k]=right[j];
        j++; k++;
    }
}
