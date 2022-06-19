/*
   
*/

//#include "TimerOne.h"
#include <Wire.h>
#include <Adafruit_INA219.h>

Adafruit_INA219 ina219;

const int BtPIN= 2;
const int outPIN = 8;
const int ledPIN = 13;
  
//const float BatStopV = 3.3; // Lithum
const float BatStopV = 0.9; // NiMH 
int btCount = 0;

float batStartV = - 0.1;
float batEndV = 0.0;
float busvoltage = 0;

int secondCount = 0;
int minCount = 0;
int testCount = 0;
int lessprintLimit = 180;// ~3 minutes
int printRequest = lessprintLimit;

unsigned long lasttime = 0;;
unsigned long lastPeriod =0;
unsigned long lastBtUp = 0;

float totalMASec = 0.0;
float totalSec = 0.0;
float totalAmpMins = 0.0;
long rawItotal = 0;
float mAHour = 0.0;
int mAHourInt = 0;
unsigned long butDownTime;
int state = 0;
// the setup routine runs once when you press reset:
void setup() {
  
  pinMode(ledPIN, OUTPUT);
  pinMode(outPIN, OUTPUT);
  pinMode(BtPIN, INPUT);
  
  digitalWrite(ledPIN, LOW);
  digitalWrite(outPIN, LOW);
  lastBtUp = millis();
  state = 0; // 0 -inital state, 1 - discharge, 2 - done
 
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600); 
  Serial.println("#####setup#####");
//  Timer1.initialize(1000000);         // initialize timer1, and set one second period 
//  Timer1.attachInterrupt(timer1Callback);  // attaches callback() as a timer overflow interrupt
    attachInterrupt(digitalPinToInterrupt(BtPIN), btDown, FALLING );

  ina219.begin();  // Initialize first board (default address 0x40)
  ina219.setCalibration_32V_1A ();
  //ina219.setCalibration_16V_400mA ();
}
 
void btDown(){
    if ( (millis() - lastBtUp ) < 1000){
      Serial.println("bounce");
      return;
    }else lastBtUp = millis();
    
    Serial.print("##### btDown count= ");    Serial.println(btCount);
    Serial.print("start state=");    Serial.println(state);  
  
    if (state == 0){
      //batStartV = ina219.getBusVoltage_V();  // cannot call ina219 in interupt or timer event
      state = 1;      
     // btCount += 1; 
    } else if (state == 1) { 
      state = 2; 
      //btCount += 1;
      done();
    } else if (state == 2 ){// push button 3 times will set state to 0
      if (btCount >= 1){
        btCount = 0;
        state = 0;
      }else btCount += 1;
    }
    Serial.print("btDown count= ");    Serial.println(btCount);
    Serial.print("end state=");    Serial.println(state); 
}

void cal(){  
  float shuntvoltage = 0;  
  float current_mA = 0;  
 
  busvoltage = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();   
  batEndV = busvoltage;
  if(batEndV <= BatStopV){
    Serial.print("battery volt is too low: ");
    Serial.println(batEndV);
    state = 2;
    done();
  }  
  
  unsigned long cctime = millis();
  lastPeriod = cctime - lasttime;
  lasttime = cctime;
  float lastPinSec= (float)lastPeriod/1000.0;
  totalSec += lastPinSec;
//  Serial.print("last Period in Sec "); Serial.println(lastPinSec, 4);
//  Serial.print("last Period in millis "); Serial.println(lastPeriod);
  totalMASec += current_mA*lastPinSec;
  
  if(printRequest >= lessprintLimit){
    printRequest = 0;
    Serial.print("Bus Voltage: "); Serial.print(busvoltage); Serial.print(" V,   ");
  //Serial.print("Shunt Voltage: "); Serial.print(shuntvoltage); Serial.println(" mV");  
    Serial.print("Current: "); Serial.print(current_mA); Serial.print(" mA, ");
    totalAmpMins = totalMASec/60.0;
    float totalMins = totalSec/60.0;
    mAHour = totalAmpMins/60.0;
    mAHourInt = mAHour;      
    Serial.print(mAHour );  Serial.print( " mA-H in ");  Serial.print(totalMins);  Serial.println( " minutes");

  }
  printRequest = printRequest + 1;
  
  
  
}
void done(){
  digitalWrite(outPIN, LOW); // disconnect load
  digitalWrite(ledPIN, HIGH);  
  totalAmpMins = totalMASec/60.0;
  float totalMins = totalSec/60.0;
  mAHour = totalAmpMins/60.0;
  mAHourInt = mAHour;
  Serial.println( " ");
  Serial.println("###### DONE ######");
  Serial.print("start at ");  Serial.print(batStartV);  Serial.print(" end at ");  Serial.println(batEndV);    
  Serial.print(mAHour );  Serial.print( " mA-H in ");  Serial.print(totalMins);  Serial.println( " minutes");
  Serial.println(" ############# ");
  Serial.print(totalMASec );  Serial.print( " mA-sec in ");  Serial.print(totalSec);  Serial.println( " seconds");
  Serial.println("#################");
  Serial.println( " ");
}

// the loop routine runs over and over again forever:
void loop() {
  run(); 
}
void run(){   
  if (state == 1){ 
    if (batStartV <= 1.0){ // get initial voltage
      batStartV = ina219.getBusVoltage_V();
    }
    blinkLed();
    digitalWrite(outPIN, HIGH); 
    lasttime = millis();
    delay(1000);  
    cal(); 
  }  
} 
void blinkLed(){
   digitalWrite(ledPIN, digitalRead(ledPIN)^1 );
}
 
