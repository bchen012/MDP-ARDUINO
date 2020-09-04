#include "DualVNH5019MotorShield.h"
#include "PinChangeInt.h"

//505 A4 493 A5 right sensors ideal
// 497 A0 505 A2 front sensors ideal
DualVNH5019MotorShield md;
int incomingByte = 0; // for incoming serial data
bool valid = true;
int inPinL = 11; // 3-> right   11-> left
int inPinR = 3;
float frontL = 0, frontR = 0, side_F =0, side_B=0, diff_F = 0, diff_S = 0, mult = 70, a0 = 0, a2= 0, a4 = 0, a5 = 0, diff_L = 0, diff_R = 0;
int sample = 30, breakOut = 100, cb = 0;
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
  a0 = sensorRead(sample, A0);
  a2 = sensorRead(sample, A2);
  a4 = sensorRead(sample, A4);
  a5 = sensorRead(sample, A5);
  frontL = convertToCM_A0(a0);
  frontR = convertToCM_A2(a2);
  side_F = convertToCM_A5(a5);
  side_B = convertToCM_A4(a4);
  diff_S = side_F-side_B;
  diff_F = frontL-frontR;
  diff_L = a0 - 497;
  diff_R = a2 - 505;
  
//  if (abs(diff_F)>0.4 || abs(diff_L) > 50 || abs(diff_R) > 50){
//    front_angle_fix();
//    front_dist_fix();
//    front_angle_fix();
//  }
  if (abs(diff_S)>0.4)
    side_angle_fix();
}
  
void side_angle_fix(){
    cb = 0;
    while (abs(diff_S)>0.3 && cb<breakOut){
    if (diff_S>1.5)
        diff_S=1.5;

    if(diff_S<-1.5)
        diff_S=-1.5;
    md.setSpeeds(diff_S*mult, -diff_S*mult);
    Serial.println(diff_S*mult);
    cb++;
    delay(20);
    side_F = convertToCM_A5(sensorRead(sample, A5));
    side_B = convertToCM_A4(sensorRead(sample, A4));
    diff_S = side_F-side_B;
  }
}

  
void front_angle_fix(){
    cb = 0;
    while (abs(diff_F)>0.4 && cb<breakOut){
    if (diff_F>1.2)
        diff_F=1.2;

    if(diff_F<-1.2)
        diff_F=-1.2;
    md.setSpeeds(-diff_F*mult, diff_F*mult);
    cb++;
    delay(20);
    side_F = convertToCM_A0(sensorRead(sample, A0));
    frontR = convertToCM_A2(sensorRead(sample, A2));
    diff_F = frontL-frontR;
  }
}

void side_dis_fix(){
  front_side = sensorRed(sample,A
  //leftTurn();
  front_angle_fix();
  front_dist_fix();
  //rightTurn();
}

void front_dist_fix(){
  
  cb = 0;
  
  while (abs(diff_L)>50 && abs(diff_R)>50 && cb<breakOut){
    if(diff_L > 140)
      diff_L=140;
    if(diff_R > 150)
      diff_R=140;
    md.setSpeeds(-diff_L, -diff_R);
    delay(20);
    cb++;
    a0 = sensorRead(sample, A0);
    a2 = sensorRead(sample, A2);
    diff_L = a0 - 497;
    diff_R = a2 - 505;
  }
}
  

float convertToCM_A2(float val){
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



float convertToCM_A0(float val){
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


float convertToCM_A4(float val){
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

float convertToCM_A5(float val){
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
