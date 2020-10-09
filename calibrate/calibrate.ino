#include "DualVNH5019MotorShield.h"
#include "PinChangeInt.h"

#define frontL A0
#define frontM A1
#define frontR A2
#define longR A3
#define sideB A4
#define sideF A5
#define BUFFER_LENGTH 128

//505 A4 493 A5 right sensors ideal
// 497 A0 505 A2 front sensors ideal
DualVNH5019MotorShield md;
enum sense { FRONT_LEFT = 0, FRONT_MID = 1, FRONT_RIGHT = 2,LONG_RANGE = 3,SIDE_BACK = 4, SIDE_FRONT = 5};
int incomingByte = 0; // for incoming serial data
bool valid = true;
int inPinL = 11, inPinR = 3; // 3-> right   11-> left 
double frontL_cm = 0, frontR_cm = 0, sideF_cm =0, sideB_cm=0;
double diff_fl = 0, diff_fr = 0, diff_sf = 0, diff_sb = 0, diff_F = 0, diff_S = 0;
double  frontL_raw = 0,frontM_raw = 0, frontR_raw= 0, sideF_raw = 0, sideB_raw = 0, longR_raw = 0;
int mult = 70,sample = 32, breakOut = 22, cb = 0;
String sensorData = "000000";
char instruction[BUFFER_LENGTH]="";
char prevS = '0';

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
int turn_time = 65, startSpeed = 200, forwardSpeed = 62;
signed int speed_R = 0, speed_L = 0;
double tick360L = 1668, tick360R = 1669;  
bool enable_send_data = true;
int send_ack=0;

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
//TO DO: CALIBRATE THESE 2
int deg_to_tick_R(double degree){
  return int(((tick360R)/360)*degree -36);
}
int deg_to_tick_L(double degree){
  return int(((tick360L)/360)*degree - 36);
}
int distance_to_ticks(int dist){
  return int(29.429*(double)dist-36);
}

void setup()
{
  Serial.begin(9600);
  md.init();
  PCintPort::attachInterrupt(inPinL,Interrupt_L,RISING);
  PCintPort::attachInterrupt(inPinR,Interrupt_R,RISING);

//TURNING CALIBRATION

// double left_error=1, right_error=1;
//  while(abs(left_error) > 0.3 || abs(right_error) > 0.3){
//      side_angle_fix('0','0');
//      delay(200);
//      left_turn(60, deg_to_tick_L(360));
//      delay(100);
//      sideF_raw = sensorRead(sample, sideF);
//      sideB_raw = sensorRead(sample, sideB);
//      sideF_cm = convertToCM_sf(sideF_raw);
//      sideB_cm = convertToCM_sb(sideB_raw);
//      left_error = sideF_cm-sideB_cm;
//      tick360L += left_error*4;
//      Serial.print(left_error);
//      Serial.print("  left:  ");
//      Serial.println(tick360L);
//      
//      
//      side_angle_fix('0','0');
//      delay(200);
//      right_turn(60, deg_to_tick_R(360));
//      delay(200);
//      sideF_raw = sensorRead(sample, sideF);
//      sideB_raw = sensorRead(sample, sideB);
//      sideF_cm = convertToCM_sf(sideF_raw);
//      sideB_cm = convertToCM_sb(sideB_raw);
//      right_error = sideB_cm-sideF_cm;
//      tick360R += right_error*4;
//      Serial.print(right_error);
//      Serial.print("  right:  ");
//      Serial.println(tick360R);
//    }
//while(1);

//  md.setSpeeds(400,0);   //when right wheel jams
//  while(1);

//  initial_setup();
//  delay(100);
//  right_turn(60, deg_to_tick_R(90));
//  delay(100);
//  side_angle_fix('0','0');
}


void loop()
{
  recieve_instruction();
  execute_instructions();  // calls detect surrounding in execute_instructions()
  send_data();
}


void initial_setup(){
  front_dist_fix('0','0');
  delay(500);
  side_dist_fix('0','0');
  delay(500);
  front_dist_fix('0','0');
  delay(500);
}

void detect_surrounding(){
  frontL_raw = sensorRead(sample, frontL); // range 2
  frontM_raw = sensorRead(sample, frontM); // range 2
  frontR_raw = sensorRead(sample, frontR); // range 2
  longR_raw  = sensorRead(sample, longR); // range 4
  sideB_raw = sensorRead(sample, sideB); // range 2
  sideF_raw = sensorRead(sample, sideF); // range 2
  sensorData[FRONT_LEFT] = find_closest_2(frontL_raw, 529, 275, 185);
  sensorData[FRONT_MID] = find_closest_2(frontM_raw, 625, 296, 192);
  sensorData[FRONT_RIGHT] = find_closest_2(frontR_raw, 531, 270, 187);
  sensorData[LONG_RANGE] = find_closest_4(longR_raw, 533, 423, 323, 256, 211);
  sensorData[SIDE_BACK] = find_closest_2(sideB_raw, 517, 266, 179);
  sensorData[SIDE_FRONT] = find_closest_2(sideF_raw, 522, 257, 176);
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
  if (enable_send_data == false) return;
  
  String s = "[b]"+sensorData;
  Serial.println(s);
  s = "[c]Explore:"+sensorData;
  Serial.println(s);
}

void recieve_instruction(){
//  Serial.println("[b]Recieve");
  int i = 1;
  while (Serial.available()==0);
  // wait for string;
  while(true){
    instruction[0] = Serial.read();
    delay(20);
    if (instruction[0] == 'F'|| instruction[0] ==  'L' || instruction[0]=='R'||instruction[0]=='E' || instruction[0]=='S' || instruction[0]=='C'){
      break;
      }
    }


    
  while (true) {
    // read the incoming byte:
    bool valid;
      instruction[i] = Serial.read();
      valid = instruction[i]=='F' || instruction[i]=='C' || instruction[i]=='L' || instruction[i]=='R'||instruction[i]=='E'|| instruction[i]=='S' || (instruction[i]>= '0' && instruction[i]<='9');
      if(!valid)break;
      i++;
      delay(20);
  }
  instruction[i]='\n';
}

void execute_instructions(){
  int dist = 0;
  
  for(int i = 0; i<BUFFER_LENGTH && instruction[i] != '\0'&& instruction[i] != '\n'&& instruction[i] != '\r'; i++){
      
      if( instruction[i]=='F'){
        dist = 0;
        i++;
        dist = instruction[i]-'0';
        if(instruction[i+1]>= '0' && instruction[i+1]<='9'){
          i++;
          dist =dist*10+(instruction[i]-'0');
        }
        Serial.print("[b]F");
        Serial.println(dist);
//        if (!enable_send_data)
//          Serial.println("[c]M");
        forward(forwardSpeed, distance_to_ticks(dist*10));
      }
      else if(instruction[i] == 'L'){
        Serial.println("[b]L");
//        if (!enable_send_data)
//          Serial.println("[c]M");
        left_turn(60, deg_to_tick_L(90));
      }
      else if(instruction[i] == 'R'){
        Serial.println("[b]R");
//        if (!enable_send_data)
//          Serial.println("[c]M");
        right_turn(60, deg_to_tick_R(90));
      }
      else if( instruction[i]=='E'){
        enable_send_data = false;
        forwardSpeed = 70;
      }
      else if(instruction[i]=='C'){
//        initial_setup();
//        i=0;
//        while(i<BUFFER_LENGTH) {
//          instruction[i]='\n';
//          i+=1;
//        }
        Serial.println("[b]R");
        right_turn(60, deg_to_tick_R(90));
        side_angle_fix('0','0');
        Serial.println("[b]E");       //let android know exploration ended
      }
      
    
    
    detect_surrounding();
    front_dist_fix(sensorData[FRONT_RIGHT],sensorData[FRONT_LEFT]);
    if (!side_angle_fix(sensorData[SIDE_FRONT],sensorData[SIDE_BACK]))
        front_angle_fix(sensorData[FRONT_RIGHT],sensorData[FRONT_LEFT]);

//    delay(100);
  }
  if (!enable_send_data){
      if(send_ack==0) send_ack+=1;
      else if(send_ack==1) {
      Serial.println("[c]ACK");
      send_ack+=1;
      }
      
  }
    side_dist_fix(sensorData[SIDE_FRONT],sensorData[SIDE_BACK]);  
}

  
bool side_angle_fix(char front, char back){
   if (front == '2' || back == '2')
      return false;
   if(front == '1' || back == '1')
      return false;
   double front_offset = 0.0, back_offset = 0.0;
   if(front == '1')
      front_offset = 10;
   if (back == '1')
      back_offset = 10;
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
      if(abs(diff_S)<0.5){
        if(diff_S<0)
          diff_S=-0.5;
        else
          diff_S=0.5;
      }
      if(abs(diff_S)>0.8){
        if(diff_S<0)
          diff_S=-0.8;
        else
          diff_S=0.8;
      }
      md.setSpeeds(diff_S*100, -diff_S*100);
//      delay(50);
//      md.setSpeeds(0, 0);
      sideF_cm = convertToCM_sf(sensorRead(sample, sideF)) - front_offset;
      sideB_cm = convertToCM_sb(sensorRead(sample, sideB)) - back_offset;
      diff_S = sideF_cm-sideB_cm;
      cb++;
    }
  
  md.setBrakes(400,400);
  return true;
}

void side_dist_fix(char front, char back){
  if (front == '2' || back == '2' ||front == '1' || back == '1' )
      return;
  sideF_raw = sensorRead(sample, sideF);
  sideB_raw = sensorRead(sample, sideB);
  
  diff_sf = sideF_raw - 562;
  diff_sb = sideB_raw - 520;
//  if(front == '1'){
//    diff_sf = sideF_raw - 268;
//  }
//  if(back == '1'){
//    diff_sb = sideB_raw - 263;
//  }
  
  if(abs(diff_sf)>100 || abs(diff_sb)>100){
      //turn left
      side_angle_fix(front, back);
      delay(50);
      left_turn(60, deg_to_tick_L(90));
      delay(50);
//      front_angle_fix(front, back);
//      delay(50);
      front_dist_fix(front, back);
      delay(50);
//      front_angle_fix(front , back);
//      delay(50);
      //turn right
      right_turn(60, deg_to_tick_R(90));
      delay(50);
      side_angle_fix(front, back);
  }
  
}

void front_angle_fix(char right, char left){
  if (right == '2' || left == '2' || right=='1' || left == '1')
      return;
   float right_offset = 0.5, left_offset = 0;
   if(right == '1')
      right_offset = 10;
   if (left == '1')
      left_offset = 11;
  frontL_raw = sensorRead(sample, frontL);
  frontR_raw = sensorRead(sample, frontR);
  frontL_cm = convertToCM_fl(frontL_raw) - left_offset;
  frontR_cm = convertToCM_fr(frontR_raw) - right_offset;
  diff_F = frontL_cm-frontR_cm;  
  cb = 0;
  while (abs(diff_F)>0.5 && cb<breakOut){

      if(abs(diff_F)<0.5){
        if(diff_F<0) diff_F=-0.5;
        else diff_F=0.5;
      }
      if(abs(diff_F)>0.8){
        if(diff_F<0) diff_F=-0.8;
        else diff_F=0.8;
      }
      md.setSpeeds(-diff_F*100, diff_F*100);  


      
    frontL_cm = convertToCM_fl(sensorRead(sample, frontL)) - left_offset;
    frontR_cm= convertToCM_fr(sensorRead(sample, frontR)) - right_offset;
    diff_F = frontL_cm-frontR_cm;
//    Serial.println(diff_F);
    cb++;
  }
  md.setBrakes(400,400);
}


void front_dist_fix(char right, char left){
  if (right == '2' || left == '2' || right == '1' || left == '1' )
      return;
  int lRaw = 0,rRaw = 0,lLimit = 0, rLimit = 0, lMinSpeed=45,rMinSpeed=45;
  double lMult = 1, rMult = 1;
  int lSpeed = 0, rSpeed = 0;
   //Serial.println("front_dist_fix start");
  if(left == '0'){
    lRaw = 532;
    lLimit =6;
  }else if(left == '1'){
    lRaw = 264;  
    lLimit = 3; 
    lMult = 2;
  }

  if(right == '0'){
    rRaw = 532;
    rLimit = 6;
  }else if(right == '1'){
    rRaw = 268; 
    rLimit = 3; 
    rMult =2;
  }

  frontL_raw = sensorRead(sample, frontL);
  frontR_raw = sensorRead(sample, frontR);
  diff_fl = frontL_raw - lRaw;
  diff_fr = frontR_raw - rRaw;
  cb = 0;
  int speedLimit = 50;
  while ((abs(diff_fl)>lLimit || abs(diff_fr)>rLimit) && cb<breakOut){

//    if(abs(diff_fl)>lLimit) diff_fl = max(lMinSpeed,min(abs(diff_fl),lMax))*((diff_fl<0)? -1:1);
//    if(abs(diff_fr)>rLimit) diff_fr = max(rMinSpeed,min(abs(diff_fr),rMax))*((diff_fr<0)? -1:1);
//    
    lSpeed = diff_fl*lMult;
    rSpeed = diff_fr*rMult;
    if (diff_fl < -lLimit  && lSpeed > -lMinSpeed)
        lSpeed = -lMinSpeed;
    if (diff_fl >  lLimit && lSpeed < lMinSpeed)
        lSpeed = lMinSpeed;
    if (diff_fr <  -rLimit && rSpeed > -rMinSpeed)
        rSpeed = -rMinSpeed;
    if (diff_fr >  rLimit && rSpeed < rMinSpeed)
        rSpeed = rMinSpeed;

//    if(abs(lSpeed) == lMinSpeed)lMinSpeed++;
//    if(abs(rSpeed) == rMinSpeed)rMinSpeed++;
//    lSpeed = max(lMinSpeed,min(lMax,abs(lSpeed)))* (lSpeed>=0?1:-1);
//    rSpeed = max(rMinSpeed,min(rMax,abs(rSpeed)))* (rSpeed>=0?1:-1);
//    if(abs(lSpeed)==lMinSpeed)lMinSpeed++;
//    if(abs(rSpeed)==rMinSpeed)rMinSpeed++;
    md.setSpeeds(-rSpeed, -lSpeed);

//    delay(90);
//    md.setSpeeds(0,0);
//    delay(10);
    frontL_raw = sensorRead(sample, frontL);
    frontR_raw = sensorRead(sample, frontR);

    
    diff_fl = frontL_raw - lRaw;
    diff_fr = frontR_raw - rRaw;
//    Serial.print(diff_fl);
//    Serial.print(" ");
//    Serial.println(diff_fr);
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
 double dist;
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
 double dist;
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

double convertToCM_lr(double val){
 double dist;
 if(val>291)
  dist=30;
 else if(val<=291&&val>260) //30cm to 35cm
  dist=(val-477.0)/-6.2;
 else if(val<=260&&val>236) //35cm to 40cm
  dist=(val-428.0)/-4.8;
 else if(val<=236&&val>216) //40cm to 45cm
  dist=(val-396.0)/-4.0;
 else if(val<=216&&val>198) //45cm to 50cm
  dist=(val-378.0)/-3.6;
 else if(val<=198&&val>185) //50cm to 55cm
  dist=(val-328.0)/-2.6;
 else if(val<=185&&val>173) //55cm to 60cm
  dist=(val-317.0)/-2.4;
 else if(val<=173&&val>161) //60cm to 65cm
  dist=(val-317.0)/-2.4;
 else if(val<=161&&val>152) //65cm to 70cm
  dist=(val-278.0)/-1.8;
 else
  dist=70;
 return dist;
}

void forward(int setRPM, int num){
    enCountL = 0;
    enCountR = 0;
    int limit = 0 ,diffL = 0,diffR = 0;
   
    startMotor(0, setRPM);
    u_L = setRPM;
    u_R = setRPM;
    while (enCountL<num && enCountR<num){
      
      set_rpm(setRPM);
      speed_L = (u_L + 20.7)*2.778;
      speed_R = (u_R + 23.9)*2.793;
      md.setSpeeds(speed_R, speed_L);
    }

//    Serial.print(enCountL);
//    Serial.print(",");
//    Serial.println(enCountR);

    stopMotor(0, setRPM);

    delay(50);
//    Serial.print(enCountL);
//    Serial.print(",");
//    Serial.println(enCountR);
}

void right_turn(int setRPM, int num){
    enCountL=0;
    enCountR=0;
    startMotor(1, setRPM);
    u_L = setRPM;
    u_R = setRPM;
    while (enCountL<num){
    set_rpm(setRPM);
//    speed_L = (u_L + 20.7)*2.778;
//    speed_R = (u_R + 22.5)*2.703;    
      speed_L = u_L*2.778+57.5;
      speed_R = u_R*2.703+60.8;    

    md.setSpeeds(-speed_R, speed_L);
    }
//    Serial.print(enCountR);
//    Serial.print(" ");
//    Serial.println(enCountL);
    stopMotor(1, setRPM);
    delay(50);
//    Serial.print(enCountR);
//    Serial.print(" ");
//    Serial.println(enCountL);
}
void left_turn(int setRPM, int num){
    enCountL=0;
    enCountR=0;
    
    startMotor(2, setRPM);
    u_L = setRPM;
    u_R = setRPM;
    while (enCountL < num){
    set_rpm(setRPM);
//    speed_L = (u_L + 18.3)*2.857;
//    speed_R = (u_R + 22.3)*2.703;
    speed_L = int(u_L*2.857+52.3);
    speed_R = int(u_R*2.703+60.3);
    md.setSpeeds(speed_R, -speed_L);                                   
    }
    //stopMotor(2, setRPM);
//    Serial.print(enCountR);
//    Serial.print(" ");
//    Serial.println(enCountL);
    stopMotor(2, setRPM);
    delay(50);
//    Serial.print(enCountR);
//    Serial.print(" ");
//    Serial.println(enCountL);
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
         md.setSpeeds(speed_R, speed_L);   // forward
            break;
    case 1: speed_L = (u + 20.7)*0.347; //2.778
            speed_R = (u + 25.3)*0.337; //2.703
            md.setSpeeds(-speed_R*4, speed_L*4);   // right
            delay(acc_rate);
            md.setSpeeds(-speed_R*2, speed_L*2);   // right
            delay(acc_rate);
            md.setSpeeds(-speed_R, speed_L);   // righ
            break;
    case 2: speed_R = (u + 22.3)*0.337;//2.703
            speed_L = (u + 18.3)*0.375; //2.875

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



// Testing functions

//  frontL_raw = sensorRead(sample, sideF);
//  frontR_raw = sensorRead(sample, sideB);
//  frontL_cm = convertToCM_sf(frontL_raw);
//  frontR_cm = convertToCM_sb(frontR_raw);
//  Serial.print(frontL_cm);
//  Serial.print(" ");
//  Serial.println(frontR_cm);
//  delay(500);
  
//  int a_4 = sensorRead(sample, A3);
//  int a_1 = sensorRead(sample, A4);
//  int a_5 = sensorRead(sample, A5);
//  if(a_5 <1000 && a_4<1000 ){
//    Serial.print(a_4);
//    Serial.print(" ");
//    Serial.print(a_1);
//    Serial.print(" ");
//    Serial.println(a_5);
//    delay(500);
//  }
