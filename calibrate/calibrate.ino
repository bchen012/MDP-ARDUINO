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
float frontL_cm = 0, frontR_cm = 0, sideF_cm =0, sideB_cm=0;
float diff_fl = 0, diff_fr = 0, diff_sf = 0, diff_sb = 0, diff_F = 0, diff_S = 0;
float  frontL_raw = 0, frontR_raw= 0, sideF_raw = 0, sideB_raw = 0;
int mult = 80,sample = 30, breakOut = 50, cb = 0;
bool sAngleFlag = false, fAngleFlag  = false, fDistFlag = false, sDistFlag = false;
int calibrateDel = 0;

void front_fix(bool isSideFix = false);

void setup()
{
  Serial.begin(9600);
  md.init();
    
    
}


void loop()
{
//  int a_4 = sensorRead(sample, A4);
//  int a_1 = sensorRead(sample, A1);
//  int a_5 = sensorRead(sample, A5);
//  
//
//  if(a_5 <1000 && a_4<1000 ){
//    Serial.print(a_4);
////    Serial.print(" ");
////    Serial.print(a_1);
//    Serial.print(" ");
//    Serial.println(a_5);
//    delay(500);
//  }


//  Serial.print(frontL);
//  Serial.print(" ");
//  Serial.println(frontR);
//  delay(500);
//
//  frontL_raw = sensorRead(sample, frontL);
//  frontR_raw = sensorRead(sample, frontR);
//  sideF_raw = sensorRead(sample, sideF);
//  sideB_raw = sensorRead(sample, sideB);
//  frontL_cm = convertToCM_fl(frontL_raw);
//  frontR_cm = convertToCM_fr(frontR_raw);
//  sideF_cm = convertToCM_sf(sideF_raw);
//  sideB_cm = convertToCM_sb(sideB_raw);
//  diff_S = sideF_cm - sideB_cm;
//  diff_F = frontL_cm-frontR_cm;
//  diff_fl = frontL_raw - 497;
//  diff_fr = frontR_raw - 505;
//  diff_sf = sideF_raw - 493;
//  diff_sb = sideB_raw - 505;
//  if (abs(diff_F)>0.4 || abs(diff_L) > 50 || abs(diff_R) > 50){
//    front_angle_fix();
//    front_dist_fix();
//    front_angle_fix();
//  }
  front_fix();
  md.setSpeeds(-400,400);
  delay(2000);
  md.setBrakes(400,400);
  delay(5000);
  side_fix();
  delay(10000);


}
  
void side_angle_fix(){
    cb = 0;
    sideF_raw = sensorRead(sample, sideF);
    sideB_raw = sensorRead(sample, sideB);
    sideF_cm = convertToCM_sf(sideF_raw);
    sideB_cm = convertToCM_sb(sideB_raw);
    diff_S = sideF_cm-sideB_cm;
    sAngleFlag = abs(diff_S)>0.3;
    
    while (sAngleFlag && cb<breakOut){
      diff_S = max(-1.5,min(diff_S,1.5));
      md.setSpeeds(diff_S*mult, -diff_S*mult);
      
      sideF_cm = convertToCM_sf(sensorRead(sample, sideF));
      sideB_cm = convertToCM_sb(sensorRead(sample, sideB));
      diff_S = sideF_cm-sideB_cm;
      sAngleFlag = abs(diff_S)>0.2;
      cb++;
    }
  
  md.setBrakes(400,400);
}


void side_dist_fix(){
  sideF_raw = sensorRead(sample, sideF);
  sideB_raw = sensorRead(sample, sideB);
  diff_sf = sideF_raw - 493;
  diff_sb = sideB_raw - 505;
  sDistFlag = abs(diff_sf)>40 && abs(diff_sb)>40;

  cb = 0;
  while(sDistFlag && cb<2){
    //turn left
    md.setSpeeds(200,-200);
    delay(1000);
    md.setBrakes(400,400);
    
    front_fix(true);
  
    //turn right
    md.setSpeeds(-200,200);
    delay(1000);
    md.setBrakes(400,400);
    
    side_angle_fix();
    delay(500);
    //measure new
    sideF_raw = sensorRead(sample, sideF);
    sideB_raw = sensorRead(sample, sideB);
    diff_sf = sideF_raw - 493;
    diff_sb = sideB_raw - 505;
    sDistFlag = abs(diff_sf)>30 && abs(diff_sb)>30;
    cb++;
  }
}  

void side_fix(){
 int i = 0;
 do{
   side_angle_fix();
   side_dist_fix();
   i++;
 }while( (sAngleFlag||sDistFlag) && i<2 );
 side_angle_fix();
 sAngleFlag = false;
 sDistFlag = false;
}
void front_angle_fix(){
   Serial.println("front_angle_fix start");

  frontL_raw = sensorRead(sample, frontL);
  frontR_raw = sensorRead(sample, frontR);
  frontL_cm = convertToCM_fl(frontL_raw);
  frontR_cm = convertToCM_fr(frontR_raw);
  diff_F = frontL_cm-frontR_cm;
  fAngleFlag = abs(diff_F)>0.3;
  
  cb = 0;
  while (fAngleFlag && cb<breakOut){
    diff_F = max(-1.5,min(diff_F,1.5));
    md.setSpeeds(-diff_F*mult,diff_F*mult);
    
    frontL_cm = convertToCM_fl(sensorRead(sample, frontL));
    frontR_cm= convertToCM_fr(sensorRead(sample, frontR));
    
    diff_F = frontL_cm-frontR_cm;
    fAngleFlag = abs(diff_F)>0.2;
    cb++;
  }
  md.setBrakes(400,400);
   Serial.println("front_anglet_fix done");



}


void front_dist_fix(){
   Serial.println("front_dist_fix start");

  frontL_raw = sensorRead(sample, frontL);
  frontR_raw = sensorRead(sample, frontR);
  diff_fl = frontL_raw - 497;
  diff_fr = frontR_raw - 505;
  fDistFlag = abs(diff_fl)>40 && abs(diff_fr)>40;
  
  cb = 0;
  while (fDistFlag && cb<breakOut){
    diff_fl = max(-150,min(diff_fl,150));
    diff_fr = max(-150,min(diff_fr,150));
    md.setSpeeds(-diff_fr*1.2, -diff_fl*1.2);
    
    frontL_raw = sensorRead(sample, frontL);
    frontR_raw = sensorRead(sample, frontR);
    diff_fl = frontL_raw - 497;
    diff_fr = frontR_raw - 505;
    fDistFlag = abs(diff_fl)>30 && abs(diff_fr)>30;
    cb++;
  }
    md.setBrakes(400,400);
 Serial.println("front_dist_fix done");
}

void front_fix(bool isSideFix = false){
   
  bool tempDist = fDistFlag;
  bool tempAngle = fAngleFlag;
  int i = 0;
  
  do{
    front_angle_fix();
    front_dist_fix();
    i++;
  }while((fAngleFlag||fDistFlag) && i<2);
  front_angle_fix();
  fAngleFlag = false;
  fDistFlag = false;
  
  if(isSideFix){
    fDistFlag = tempDist;
    fAngleFlag = tempAngle;
  }
}
// called after every movement, if any flag is set, try to calibrate in optimal conditions(at cornerrs)
// if calibration is delayed too long, calibrate on suboptimal conditions
// if all flags are resolved, delay is reset to 0
void calibrate(){
  //  LOGIC FOR CALIBRATION -> to be tested
//  if(sAngleFlag||sDistFlag||fAngleFlag||fDistFlag){
//    if( front distance and side distance less than 1 grid){
//      front_fix();
//      side_fix();
//
//      if(!(sAngleFlag||sDistFlag||fAngleFlag||fDistFlag)) calibrateDel = 0;
//    }else if(calibrateDel>5){
//      if(front distances< 1 grid)front_fix();
//      if(sides distances< 1 grid)side_fix();
//
//      if(!(sAngleFlag||sDistFlag||fAngleFlag||fDistFlag)) calibrateDel = 0;
//    }else calibrateDel++;
//  }

}

//A2
float convertToCM_fr(float val){
  float dist;
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
    dist=20;
  return dist;
}

//float convertToCM_A1(float val){
//  float dist;
//  if(val>625)
//    dist=1;
//  else if(val<=625&&val>617)  //1cm to 2cm
//    dist=(val-633.0)/-8.0;
//  else if(val<=617&&val>563)  //2cm to 3cm
//    dist=(val-725.0)/-54.0;
//  else if(val<=563&&val>514)  //3cm to 4cm
//    dist=(val-710.0)/-49.0;
//  else if(val<=514&&val>467)  //4cm to 5cm
//    dist=(val-702.0)/-47.0;
//  else if(val<=467&&val>435)  //5cm to 6cm
//    dist=(val-627.0)/-32.0;
//  else if(val<=435&&val>401)  //6cm to 7cm
//    dist=(val-639.0)/-34.0;
//  else if(val<=401&&val>376)  //7cm to 8cm
//    dist=(val-576.0)/-25.0;
//  else if(val<=376&&val>350)  //8cm to 9cm
//    dist=(val-584.0)/-26.0;
//  else if(val<=350&&val>328)  //9cm to 10cm
//    dist=(val-548.0)/-22.0;
//  else if(val<=328&&val>312)  //10cm to 11cm
//    dist=(val-488.0)/-16.0;
//  else if(val<=312&&val>297)  //11cm to 12cm
//    dist=(val-477.0)/-15.0;
//  else if(val<=297&&val>282)  //12cm to 13cm
//    dist=(val-477.0)/-15.0;
//  else if(val<=282&&val>267)  //13cm to 14cm
//    dist=(val-477.0)/-15.0;
//  else if(val<=267&&val>256)  //14cm to 15cm
//    dist=(val-421.0)/-11.0;
//  else if(val<=256&&val>246)  //15cm to 16cm
//    dist=(val-406.0)/-10.0;
//  else if(val<=246&&val>238)  //16cm to 17cm
//    dist=(val-374.0)/-8.0;
//  else if(val<=238&&val>227)  //17cm to 18cm
//    dist=(val-425.0)/-11.0;
//  else if(val<=227&&val>219)  //18cm to 19cm
//    dist=(val-371.0)/-8.0;
//  else if(val<=219&&val>211)  //19cm to 20cm
//    dist=(val-371.0)/-8.0;
//  else
//    dist=20;
//  return dist;
//}


//A0
float convertToCM_fl(float val){
  float dist;
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
    dist=20;
  return dist;
}

//A4
float convertToCM_sb(float val){
  float dist;
  if(val>631)
    dist=2;
  else if(val<=631&&val>626)  //2cm to 3cm
    dist=(val-641.0)/-5.0;
  else if(val<=626&&val>576)  //3cm to 4cm
    dist=(val-776.0)/-50.0;
  else if(val<=576&&val>528)  //4cm to 5cm
    dist=(val-768.0)/-48.0;
  else if(val<=528&&val>480)  //5cm to 6cm
    dist=(val-768.0)/-48.0;
  else if(val<=480&&val>437)  //6cm to 7cm
    dist=(val-738.0)/-43.0;
  else if(val<=437&&val>405)  //7cm to 8cm
    dist=(val-661.0)/-32.0;
  else if(val<=405&&val>377)  //8cm to 9cm
    dist=(val-629.0)/-28.0;
  else if(val<=377&&val>355)  //9cm to 10cm
    dist=(val-575.0)/-22.0;
  else
    dist=10;
  return dist;
}
//A5
float convertToCM_sf(float val){
  float dist;
  if(val>627)
    dist=2;
  else if(val<=627&&val>627)  //2cm to 3cm
    dist=(val-627.0)/0.0;
  else if(val<=627&&val>594)  //3cm to 4cm
    dist=(val-726.0)/-33.0;
  else if(val<=594&&val>544)  //4cm to 5cm
    dist=(val-794.0)/-50.0;
  else if(val<=544&&val>487)  //5cm to 6cm
    dist=(val-829.0)/-57.0;
  else if(val<=487&&val>444)  //6cm to 7cm
    dist=(val-745.0)/-43.0;
  else if(val<=444&&val>408)  //7cm to 8cm
    dist=(val-696.0)/-36.0;
  else if(val<=408&&val>377)  //8cm to 9cm
    dist=(val-656.0)/-31.0;
  else if(val<=377&&val>352)  //9cm to 10cm
    dist=(val-602.0)/-25.0;
  else
    dist=10;
  return dist;
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
