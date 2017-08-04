/*-----Arduino Code for Motion Capture Glove-----*/
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>
#include <MahonyAHRS.h>
#include <MadgwickAHRS.h>

#define TCAADDR 0x70

Adafruit_LSM303_Mag_Unified mag1 = Adafruit_LSM303_Mag_Unified(1);
Adafruit_LSM303_Mag_Unified mag2 = Adafruit_LSM303_Mag_Unified(2);
Adafruit_LSM303_Mag_Unified mag3 = Adafruit_LSM303_Mag_Unified(3);
Adafruit_LSM303_Mag_Unified mag4 = Adafruit_LSM303_Mag_Unified(4);
Adafruit_LSM303_Accel_Unified accel1 = Adafruit_LSM303_Accel_Unified(1);
Adafruit_LSM303_Accel_Unified accel2 = Adafruit_LSM303_Accel_Unified(2);
Adafruit_LSM303_Accel_Unified accel3 = Adafruit_LSM303_Accel_Unified(3);
Adafruit_LSM303_Accel_Unified accel4 = Adafruit_LSM303_Accel_Unified(4);
Adafruit_L3GD20_Unified gyro1 = Adafruit_L3GD20_Unified(1);
Adafruit_L3GD20_Unified gyro2 = Adafruit_L3GD20_Unified(2);
Adafruit_L3GD20_Unified gyro3 = Adafruit_L3GD20_Unified(3);
Adafruit_L3GD20_Unified gyro4 = Adafruit_L3GD20_Unified(4);

const int flex1 = A0; 
const int flex2 = A1; 
const int flex3 = A2;
const int flex4 = A3;
const int flex5 = A4; 
const int flex6 = A5; 
const int flex7 = A6;
const int flex8 = A7;
const int flex9 = A8;
const int flex10 = A9;
const int flex11 = A10;
const int flex12 = A11;
const int flex13 = A12;
const int buttonPin = 2;     // the number of the pushbutton pin

int sensorValue1 = 0;         // the sensor value
int sensorMin1 = 1023;        // minimum sensor value
int sensorMax1 = 0;           // maximum sensor value
int sensorValue2 = 0;         
int sensorMin2 = 1023;        
int sensorMax2 = 0;          
int sensorValue3 = 0;         
int sensorMin3 = 1023;        
int sensorMax3 = 0;      
int sensorValue4 = 0;         
int sensorMin4 = 1023;        
int sensorMax4 = 0;  
int sensorValue5 = 0;         // the sensor value
int sensorMin5 = 1023;        // minimum sensor value
int sensorMax5 = 0;           // maximum sensor value
int sensorValue6 = 0;         
int sensorMin6 = 1023;        
int sensorMax6 = 0;          
int sensorValue7 = 0;         
int sensorMin7 = 1023;        
int sensorMax7 = 0;      
int sensorValue8 = 0;         
int sensorMin8 = 1023;        
int sensorMax8 = 0;  
int sensorValue9 = 0;         
int sensorMin9 = 1023;        
int sensorMax9 = 0;  
int sensorValue10 = 0;         
int sensorMin10 = 1023;        
int sensorMax10 = 0;  
int sensorValue11 = 0;  
int sensorMin11 = 1023;        
int sensorMax11 = 0;  
int sensorValue12 = 0;         
int sensorMin12 = 1023;        
int sensorMax12 = 0;  
int sensorValue13 = 0;         
int sensorMin13 = 1023;        
int sensorMax13 = 0;  

const int numReadings = 4;
int readIndex = 0;               // the index of the current reading
  
int readings1[numReadings];      // the readings from the analog input  
int total1 = 0;                  // the running total
int average1 = 0;                // the average
int readings2[numReadings];                  
int total2 = 0;                 
int average2 = 0;       
int readings3[numReadings];                  
int total3 = 0;                 
int average3 = 0;    
int readings4[numReadings];                  
int total4 = 0;                 
int average4 = 0;  
int readings5[numReadings];                  
int total5 = 0;                 
int average5 = 0;    
int readings6[numReadings];                  
int total6 = 0;                 
int average6 = 0;  
int readings7[numReadings];                  
int total7 = 0;                 
int average7 = 0;  
int readings8[numReadings];                  
int total8 = 0;                 
int average8 = 0;  
int readings9[numReadings];                  
int total9 = 0;                 
int average9 = 0;  
int readings10[numReadings];                  
int total10 = 0;                 
int average10 = 0;  
int readings11[numReadings];                  
int total11 = 0;                 
int average11 = 0;  
int readings12[numReadings];                  
int total12 = 0;                 
int average12 = 0;  
int readings13[numReadings];                  
int total13 = 0;                 
int average13 = 0;           


int i = 0;

float yawValue1 = 0;         
float yawMin1 = 360;        
float yawMax1 = -360;
int pitchValue1 = 0;         
int pitchMin1 = -360;        
int pitchMax1 = 360;
int rollValue1 = 0;         
int rollMin1 = 360;        
int rollMax1 = -360;

int buttonState = 0;  


float prevYaw;
float baseYaw;

float roll1;
float pitch1;
float heading1;
float prevHeading1;
float prevPitch1;
float prevRoll1;
float ogHeading1;
float ogPitch1;
float ogRoll1;
float deltaH1;
float deltaP1;
float deltaR1;
int prevAccelx1 = 0;
int prevMagx1 = 0;
int prevAccely1 = 0;
int prevMagy1 = 0;
int prevAccelz1 = 0;
int prevMagz1 = 0;

float roll2;
float pitch2;
float heading2;
float prevHeading2;
float prevPitch2;
float prevRoll2;
float ogHeading2;
float ogPitch2;
float ogRoll2;
float deltaH2;
float deltaP2;
float deltaR2;
int prevAccelx2 = 0;
int prevMagx2 = 0;
int prevAccely2 = 0;
int prevMagy2 = 0;
int prevAccelz2 = 0;
int prevMagz2 = 0;

float roll3;
float pitch3;
float heading3;
float prevHeading3;
float prevPitch3;
float prevRoll3;
float ogHeading3;
float ogPitch3;
float ogRoll3;
float deltaH3;
float deltaP3;
float deltaR3;
int prevAccelx3 = 0;
int prevMagx3 = 0;
int prevAccely3 = 0;
int prevMagy3 = 0;
int prevAccelz3 = 0;
int prevMagz3 = 0;

float roll4;
float pitch4;
float heading4;
float prevHeading4;
float prevPitch4;
float prevRoll4;
float ogHeading4;
float ogPitch4;
float ogRoll4;
float deltaH4;
float deltaP4;
float deltaR4;
int prevAccelx4 = 0;
int prevMagx4 = 0;
int prevAccely4 = 0;
int prevMagy4 = 0;
int prevAccelz4 = 0;
int prevMagz4 = 0;

int index = 0;
double totalYaw= 0;
double deltaYaw = 0;

boolean lastState = false;

boolean buttonActive = false;
boolean longPressActive = false;
long buttonTimer = 0;
long longPressTime1 = 250;
long longPressTime2 = 700;

float actualHeading=0;
//const int numReadings = 50;
float yawCal[numReadings];

const int pot1 = A0;
const int pot2 = A1;

float mag_offsets[3]            = { 6.77F, -7.75F,36.94F };

float mag_softiron_matrix[3][3] = { { 0.977, 0.013, 0.002 },
                                    { 0.314, 0.999, 0.004 },
                                    { 0.002, 0.004, 1.025 } }; 

float mag_field_strength        = 25.08F;

Madgwick filter1;
Madgwick filter2;
Madgwick filter3;
Madgwick filter4;

void tcaselect(uint8_t i) {
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}
 
 
void setup(void) 
{
  Serial.begin(250000);
  
  Serial.println("I2C Test"); Serial.println("");
  
  pinMode(ledPin, OUTPUT);
  pinMode(buttonPin, INPUT);
  pinMode(flex1, INPUT);  
  pinMode(flex2, INPUT); 
  pinMode(flex3, INPUT);
  pinMode(flex4, INPUT);
  pinMode(flex5, INPUT);  
  pinMode(flex6, INPUT); 
  pinMode(flex7, INPUT);
  pinMode(flex8, INPUT);
  pinMode(flex9, INPUT);
  pinMode(flex10, INPUT);

  pinMode(41,OUTPUT);
  digitalWrite(41, HIGH);
  pinMode(42,OUTPUT);
  digitalWrite(42, HIGH);
  pinMode(43,OUTPUT);
  digitalWrite(43, HIGH);
  pinMode(44,OUTPUT);
  digitalWrite(44, HIGH);
  pinMode(45,OUTPUT);
  digitalWrite(45, HIGH);
  pinMode(46,OUTPUT);
  digitalWrite(46, HIGH);
  pinMode(47,OUTPUT);
  digitalWrite(47, HIGH);
  pinMode(48,OUTPUT);
  digitalWrite(48, HIGH);
  pinMode(49,OUTPUT);
  digitalWrite(49, HIGH);
  pinMode(50,OUTPUT);
  digitalWrite(50, HIGH);
  pinMode(51,OUTPUT);
  digitalWrite(51, HIGH);
  pinMode(52,OUTPUT);
  digitalWrite(52, HIGH);
  pinMode(53,OUTPUT);
  digitalWrite(53, HIGH);

  for (int i = 0; i < numReadings; i++) {
    readings1[i] = 0;
    readings2[i] = 0;
    readings3[i] = 0;
    readings4[i] = 0;
    readings5[i] = 0;
    readings6[i] = 0;
    readings7[i] = 0;
    readings8[i] = 0;
    readings9[i] = 0;
    readings10[i] = 0;
    readings11[i] = 0;
    readings12[i] = 0;
    readings13[i] = 0;
  }
  
  /* Initialise the 1st sensor */
  tcaselect(1);
  
  if(!mag1.begin())
  {
    //There was a problem detecting the HMC5883 ... check your connections
    //Serial.println("Ooops, no mag1 detected ... Check your wiring!");
   // while(1);
  }
  
  if(!accel1.begin())
  {
    //There was a problem detecting the HMC5883 ... check your connections
    //Serial.println("Ooops, no accel1 detected ... Check your wiring!");
   // while(1);
  }
  Serial.println("oo");
  if(!gyro1.begin())
  {
    //There was a problem detecting the HMC5883 ... check your connections 
    //Serial.println("Ooops, no gyro1 detected ... Check your wiring!");
    //while(1);
  }
  
  // Initialise the 2nd sensor 
  tcaselect(5);
  if(!mag2.begin())
  {
    // There was a problem detecting the HMC5883 ... check your connections 
    //Serial.println("Ooops, no mag2 detected ... Check your wiring!");
    //while(1);
  }
  //Serial.println("oo2");
  if(!gyro2.begin())
  {
    //There was a problem detecting the HMC5883 ... check your connections 
    //Serial.println("Ooops, no gyro2 detected ... Check your wiring!");
    //while(1);
  }
  //Serial.println("oo3");
  if(!accel2.begin())
  {
    //There was a problem detecting the HMC5883 ... check your connections 
    //Serial.println("Ooops, no accel2 detected ... Check your wiring!");
    //while(1);
  }
  tcaselect(3);
  if(!mag3.begin())
  {
    // There was a problem detecting the HMC5883 ... check your connections 
    //Serial.println("Ooops, no mag2 detected ... Check your wiring!");
    //while(1);
  }
  //Serial.println("oo2");
  if(!gyro3.begin())
  {
    //There was a problem detecting the HMC5883 ... check your connections 
    //Serial.println("Ooops, no gyro2 detected ... Check your wiring!");
    //while(1);
  }
  //Serial.println("oo3");
  if(!accel3.begin())
  {
    //There was a problem detecting the HMC5883 ... check your connections 
    //Serial.println("Ooops, no accel2 detected ... Check your wiring!");
    //while(1);
  }
  tcaselect(6);
  if(!mag4.begin())
  {
    // There was a problem detecting the HMC5883 ... check your connections 
    //Serial.println("Ooops, no mag2 detected ... Check your wiring!");
    //while(1);
  }
  //Serial.println("oo2");
  if(!gyro4.begin())
  {
    //There was a problem detecting the HMC5883 ... check your connections 
    //Serial.println("Ooops, no gyro2 detected ... Check your wiring!");
    //while(1);
  }
  //Serial.println("oo3");
  if(!accel4.begin())
  {
    //There was a problem detecting the HMC5883 ... check your connections 
    //Serial.println("Ooops, no accel2 detected ... Check your wiring!");
    //while(1);
  }
  
  filter1.begin(40);
  filter2.begin(40);
  filter3.begin(40);
  filter4.begin(40);
  filterFunction1();
  ogHeading1 = filter1.getYaw();
  heading1 = filter1.getYaw();
  filterFunction2();
  ogHeading2 = filter2.getYaw();
  heading2 = filter2.getYaw();
  filterFunction3();
  ogHeading3 = filter3.getYaw();
  heading3 = filter3.getYaw();
  filterFunction4();
  ogHeading4 = filter4.getYaw();
  heading4 = filter4.getYaw();

   while(millis()<5000){
    
    sensorValue1 = analogRead(flex1);
    if (sensorValue1 > sensorMax1) {
      sensorMax1 = sensorValue1;}
    if (sensorValue1 < sensorMin1) {
      sensorMin1 = sensorValue1;}
      
    sensorValue2 = analogRead(flex2);
    if (sensorValue2 > sensorMax2) {
      sensorMax2 = sensorValue2;}
    if (sensorValue2 < sensorMin2) {
      sensorMin2 = sensorValue2;}

    sensorValue3 = analogRead(flex3);
    if (sensorValue3 > sensorMax3) {
      sensorMax3 = sensorValue3;}
    if (sensorValue3 < sensorMin3) {
      sensorMin3 = sensorValue3;}

    sensorValue4 = analogRead(flex4);
    if (sensorValue4 > sensorMax4) {
      sensorMax4 = sensorValue4;}
    if (sensorValue4 < sensorMin4) {
      sensorMin4 = sensorValue4;}

      sensorValue5 = analogRead(flex5);
    if (sensorValue5 > sensorMax5) {
      sensorMax5 = sensorValue5;}
    if (sensorValue5 < sensorMin5) {
      sensorMin5 = sensorValue5;}
      
    sensorValue6 = analogRead(flex6);
    if (sensorValue6 > sensorMax6) {
      sensorMax6 = sensorValue6;}
    if (sensorValue6 < sensorMin6) {
      sensorMin6 = sensorValue6;}

    sensorValue7 = analogRead(flex7);
    if (sensorValue7 > sensorMax7) {
      sensorMax7 = sensorValue7;}
    if (sensorValue7 < sensorMin7) {
      sensorMin7 = sensorValue7;}

    sensorValue8 = analogRead(flex8);
    if (sensorValue8 > sensorMax8) {
      sensorMax8 = sensorValue8;}
    if (sensorValue8 < sensorMin8) {
      sensorMin8 = sensorValue8;}

    sensorValue9 = analogRead(flex9);
    if (sensorValue9 > sensorMax9) {
      sensorMax9 = sensorValue9;}
    if (sensorValue9 < sensorMin9) {
      sensorMin9 = sensorValue9;}

    sensorValue10 = analogRead(flex10);
    if (sensorValue10 > sensorMax10) {
      sensorMax10 = sensorValue10;}
    if (sensorValue10 < sensorMin10) {
      sensorMin10 = sensorValue10;}

    sensorValue11 = analogRead(flex11);
    if (sensorValue11 > sensorMax11) {
      sensorMax11 = sensorValue11;}
    if (sensorValue11 < sensorMin11) {
      sensorMin11 = sensorValue11;}

    sensorValue12 = analogRead(flex12);
    if (sensorValue12 > sensorMax12) {
      sensorMax12 = sensorValue12;}
    if (sensorValue12 < sensorMin12) {
      sensorMin12 = sensorValue12;}

    sensorValue13 = analogRead(flex13);
    if (sensorValue13 > sensorMax13) {
      sensorMax13 = sensorValue13;}
    if (sensorValue13 < sensorMin13) {
      sensorMin13 = sensorValue13;}
      
    filterFunction1();
    yawValue1= actualHeading;
    if (yawValue1 > yawMax1) {
      yawMax1 = yawValue1;}
    if (yawValue1 < yawMin1) {
      yawMin1 = yawValue1;}
      Serial.print(yawMax1);
      Serial.print("  ");
      Serial.println(actualHeading);

    pitchValue1= pitch1;
    if (pitchValue1 > pitchMax1) {
      pitchMax1 = pitchValue1;}
    if (pitchValue1 < pitchMin1) {
      pitchMin1 = pitchValue1;}

    rollValue1= roll1;
    if (rollValue1 > rollMax1) {
      rollMax1 = rollValue1;}
    if (rollValue1 < rollMin1) {
      rollMin1 = rollValue1;}
    
  }
  ogHeading1 = 0;
    ogPitch1 = 0;
    ogRoll1 = 0;
    ogHeading2 = 0;
    ogPitch2 = 0;
    ogRoll2 = 0;
    ogHeading3 = 0;
    ogPitch3 = 0;
    ogRoll3 = 0;
    ogHeading4 = 0;
    ogPitch4 = 0;
    ogRoll4 = 0;
    
  Serial.println("  ");
  Serial.print(yawMin1);
  Serial.print("  ");
  Serial.println(yawMax1);
  
}
 
void loop(void) 
{
  filterFunction1();
    //Serial.print(micros());
    Serial.print(heading1); Serial.print(" ");
    Serial.print(pitch1); Serial.print(" ");
    Serial.print(roll1); Serial.print(" ");
  filterFunction2();
  filterFunction3();
  flexFunction();
  filterFunction4();
  buttonState = digitalRead(buttonPin);
  if (buttonState == HIGH) {
    if (buttonActive == false) {
      buttonActive = true;
      buttonTimer = millis();
    }
    if ((millis() - buttonTimer > longPressTime1) && (longPressActive == false)) {
      longPressActive = true;
      ogHeading1 = 0;
      ogPitch1 = 0;
      ogRoll1=0;
      ogHeading4 = 0;
      ogPitch4 = 0;
      ogRoll4 =0;
      //Serial.println("working");
    }
} else {
    if (buttonActive == true) {
      if (longPressActive == true) {
        longPressActive = false;
      }
      buttonActive = false;
    }
  }
  i++;
    serialFlush();
  delay(20);
}

void filterFunction1(){
  sensors_event_t gyro_event; 
  sensors_event_t mag_event; 
  sensors_event_t accel_event; 
  
  tcaselect(1);
  mag1.getEvent(&mag_event);
  accel1.getEvent(&accel_event);
  gyro1.getEvent(&gyro_event);

  float x = mag_event.magnetic.x - mag_offsets[0];
  float y = mag_event.magnetic.y - mag_offsets[1];
  float z = mag_event.magnetic.z - mag_offsets[2];

  float mx = x * mag_softiron_matrix[0][0] + y * mag_softiron_matrix[0][1] + z * mag_softiron_matrix[0][2];
  float my = x * mag_softiron_matrix[1][0] + y * mag_softiron_matrix[1][1] + z * mag_softiron_matrix[1][2];
  float mz = x * mag_softiron_matrix[2][0] + y * mag_softiron_matrix[2][1] + z * mag_softiron_matrix[2][2];

  float gx = gyro_event.gyro.x * 57.2958F;
  float gy = gyro_event.gyro.y * 57.2958F;
  float gz = gyro_event.gyro.z * 57.2958F;

  filter1.update(gx, gy, gz,
                accel_event.acceleration.x, accel_event.acceleration.y, accel_event.acceleration.z,
                mx, my, mz);
                
  roll1 = filter1.getRoll();
  //if(roll<0){roll*=-2;}
  pitch1 = filter1.getPitch();
  //if(pitch<0){pitch*=-2;}
  heading1 = filter1.getYaw();
  
  
  if(abs(accel1.raw.z-prevAccelz1)<20 && abs(mag1.raw.z-prevMagz1)<30){
    heading1 = ogHeading1;
  }
  else{
    deltaH1 = heading1-prevHeading1;
    ogHeading1 = ogHeading1 + 2*deltaH1;
    heading1 = ogHeading1;
  }
  if(abs(accel1.raw.x-prevAccelx1)<20 && abs(mag1.raw.x-prevMagx1)<10){
    pitch1 = ogPitch1;
  }
  else{
    deltaP1 = pitch1-prevPitch1;
    ogPitch1 = ogPitch1 + 2*deltaP1;
     pitch1 = ogPitch1;
  }
  if(abs(accel1.raw.y-prevAccely1)<20 && abs(mag1.raw.y-prevMagy1)<10){
    roll1 = ogRoll1;
    //Serial.print("true");
  }
  else{
    deltaR1 = roll1-prevRoll1;
    ogRoll1 = ogRoll1 + 2*deltaR1;
     roll1 = ogRoll1;
  }
  //Serial.print("    "); Serial.print(accel1.raw.y); Serial.print("   ");
  

  prevHeading1 = filter1.getYaw();
  prevPitch1 = filter1.getPitch();
  prevRoll1 = filter1.getRoll();
  //Serial.println(mag.raw.z-prevMag);
  prevAccelz1 = accel1.raw.z;
  prevMagz1 = mag1.raw.z;
  prevAccelx1 = accel1.raw.x;
  prevMagx1 = mag1.raw.x;
  prevAccely1 = accel1.raw.y;
  prevMagy1 = mag1.raw.y;
  actualHeading= heading1;
  
}
void filterFunction2(){
  sensors_event_t gyro_event; 
  sensors_event_t mag_event; 
  sensors_event_t accel_event; 
  
  tcaselect(5);
  gyro2.getEvent(&gyro_event);
  mag2.getEvent(&mag_event);
  accel2.getEvent(&accel_event);

  float x = mag_event.magnetic.x - mag_offsets[0];
  float y = mag_event.magnetic.y - mag_offsets[1];
  float z = mag_event.magnetic.z - mag_offsets[2];

  float mx = x * mag_softiron_matrix[0][0] + y * mag_softiron_matrix[0][1] + z * mag_softiron_matrix[0][2];
  float my = x * mag_softiron_matrix[1][0] + y * mag_softiron_matrix[1][1] + z * mag_softiron_matrix[1][2];
  float mz = x * mag_softiron_matrix[2][0] + y * mag_softiron_matrix[2][1] + z * mag_softiron_matrix[2][2];

  float gx = gyro_event.gyro.x * 57.2958F;
  float gy = gyro_event.gyro.y * 57.2958F;
  float gz = gyro_event.gyro.z * 57.2958F;

  filter2.update(gx, gy, gz,
                accel_event.acceleration.x, accel_event.acceleration.y, accel_event.acceleration.z,
                mx, my, mz);
                
  roll2 = filter2.getRoll();
  //if(roll<0){roll*=-2;}
  pitch2 = filter2.getPitch();
  //if(pitch<0){pitch*=-2;}
  heading2 = filter2.getYaw();

  buttonState = digitalRead(buttonPin);
  if(buttonState == HIGH){
    ogHeading2 = ogHeading1;
    ogPitch2 = ogPitch1;
    ogRoll2 = ogRoll1;
  }
  if(abs(accel2.raw.z-prevAccelz2)<20 && abs(mag2.raw.z-prevMagz2)<30){
    heading2 = ogHeading2;
    //Serial.println("yea");
  }
  else{
    deltaH2 = heading2-prevHeading2;
    ogHeading2 = ogHeading2 + 2*deltaH2;
    heading2 = ogHeading2;
  }
  if(abs(accel2.raw.x-prevAccelx2)<20 && abs(mag2.raw.x-prevMagx2)<10){
    pitch2 = ogPitch2;
  }
  else{
    deltaP2 = pitch2-prevPitch2;
    ogPitch2 = ogPitch2 + 2*deltaP2;
     pitch2 = ogPitch2;
  }
  if(abs(accel2.raw.y-prevAccely2)<20 && abs(mag2.raw.y-prevMagy2)<10){
    roll2 = ogRoll2;
    //Serial.print("true");
  }
  else{
    deltaR2 = roll2-prevRoll2;
    ogRoll2 = ogRoll2 + 2*deltaR2;
     roll2 = ogRoll2;
  }
  //Serial.print("    "); Serial.print(accel1.raw.y); Serial.print("   ");

  prevHeading2 = filter2.getYaw();
  prevPitch2 = filter2.getPitch();
  prevRoll2 = filter2.getRoll();

  prevAccelz2 = accel2.raw.z;
  prevMagz2 = mag2.raw.z;
  prevAccelx2 = accel2.raw.x;
  prevMagx2 = mag2.raw.x;
  prevAccely2 = accel2.raw.y;
  prevMagy2 = mag2.raw.y;

  Serial.print(heading2); Serial.print(" ");
  Serial.print(pitch2); Serial.print(" ");
  Serial.print(roll2); Serial.print(" ");
}

void filterFunction3(){
  sensors_event_t gyro_event; 
  sensors_event_t mag_event; 
  sensors_event_t accel_event; 
  
  tcaselect(3);
  gyro3.getEvent(&gyro_event);
  mag3.getEvent(&mag_event);
  accel3.getEvent(&accel_event);

  float x = mag_event.magnetic.x - mag_offsets[0];
  float y = mag_event.magnetic.y - mag_offsets[1];
  float z = mag_event.magnetic.z - mag_offsets[2];

  float mx = x * mag_softiron_matrix[0][0] + y * mag_softiron_matrix[0][1] + z * mag_softiron_matrix[0][2];
  float my = x * mag_softiron_matrix[1][0] + y * mag_softiron_matrix[1][1] + z * mag_softiron_matrix[1][2];
  float mz = x * mag_softiron_matrix[2][0] + y * mag_softiron_matrix[2][1] + z * mag_softiron_matrix[2][2];

  float gx = gyro_event.gyro.x * 57.2958F;
  float gy = gyro_event.gyro.y * 57.2958F;
  float gz = gyro_event.gyro.z * 57.2958F;

  filter3.update(gx, gy, gz,
                accel_event.acceleration.x, accel_event.acceleration.y, accel_event.acceleration.z,
                mx, my, mz);
                
  roll3 = filter3.getRoll();
  //if(roll<0){roll*=-2;}
  pitch3 = filter3.getPitch();
  //if(pitch<0){pitch*=-2;}
  heading3 = filter3.getYaw();

  buttonState = digitalRead(buttonPin);
  if(buttonState == HIGH){
    ogHeading3 = ogHeading1;
    ogPitch3 = ogPitch1;
    ogRoll3 = ogRoll1;
    //Serial.println("button Pressed");
  }
  if(abs(accel3.raw.z-prevAccelz3)<20 && abs(mag3.raw.z-prevMagz3)<30){
    heading3 = ogHeading3;
    //Serial.println("yea");
  }
  else{
    deltaH3 = heading3-prevHeading3;
    ogHeading3 = ogHeading3 + 2*deltaH3;
    heading3 = ogHeading3;
  }
  if(abs(accel3.raw.x-prevAccelx3)<20 && abs(mag3.raw.x-prevMagx3)<10){
    pitch3 = ogPitch3;
  }
  else{
    deltaP3 = pitch3-prevPitch3;
    ogPitch3 = ogPitch3 + 2*deltaP3;
     pitch3 = ogPitch3;
  }
  if(abs(accel3.raw.y-prevAccely3)<20 && abs(mag3.raw.y-prevMagy3)<10){
    roll3 = ogRoll3;
    //Serial.print("true");
  }
  else{
    deltaR3 = roll3-prevRoll3;
    ogRoll3 = ogRoll3 + 2*deltaR3;
     roll3 = ogRoll3;
  }
  //Serial.print("    "); Serial.print(accel1.raw.y); Serial.print("   "); 

  prevHeading3 = filter3.getYaw();
  prevPitch3 = filter3.getPitch();
  prevRoll3 = filter3.getRoll();

  prevAccelz3 = accel3.raw.z;
  prevMagz3 = mag3.raw.z;
  prevAccelx3 = accel3.raw.x;
  prevMagx3 = mag3.raw.x;
  prevAccely3 = accel3.raw.y;
  prevMagy3 = mag3.raw.y;
  
  Serial.print(heading3); Serial.print(" ");
  Serial.print(pitch3); Serial.print(" ");
  Serial.print(roll3); Serial.print(" ");
}

void filterFunction4(){
  sensors_event_t gyro_event; 
  sensors_event_t mag_event; 
  sensors_event_t accel_event; 
  
  tcaselect(6);
  gyro4.getEvent(&gyro_event);
  mag4.getEvent(&mag_event);
  accel4.getEvent(&accel_event);

  float x = mag_event.magnetic.x - mag_offsets[0];
  float y = mag_event.magnetic.y - mag_offsets[1];
  float z = mag_event.magnetic.z - mag_offsets[2];

  float mx = x * mag_softiron_matrix[0][0] + y * mag_softiron_matrix[0][1] + z * mag_softiron_matrix[0][2];
  float my = x * mag_softiron_matrix[1][0] + y * mag_softiron_matrix[1][1] + z * mag_softiron_matrix[1][2];
  float mz = x * mag_softiron_matrix[2][0] + y * mag_softiron_matrix[2][1] + z * mag_softiron_matrix[2][2];

  float gx = gyro_event.gyro.x * 57.2958F;
  float gy = gyro_event.gyro.y * 57.2958F;
  float gz = gyro_event.gyro.z * 57.2958F;

  filter4.update(gx, gy, gz,
                accel_event.acceleration.x, accel_event.acceleration.y, accel_event.acceleration.z,
                mx, my, mz);
                
  roll4 = filter4.getRoll();
  //if(roll<0){roll*=-2;}
  pitch4 = filter4.getPitch();
  //if(pitch<0){pitch*=-2;}
  heading4 = filter4.getYaw();
  
  if(abs(accel4.raw.z-prevAccelz4)<20 && abs(mag4.raw.z-prevMagz4)<30){
    heading4 = ogHeading4;
    //Serial.println("yea");
  }
  else{
    deltaH4 = heading4-prevHeading4;
    ogHeading4 = ogHeading4 + 2*deltaH4;
    heading4 = ogHeading4;
  }
  if(abs(accel4.raw.x-prevAccelx4)<20 && abs(mag4.raw.x-prevMagx4)<10){
    pitch4 = ogPitch4;
  }
  else{
    deltaP4 = pitch4-prevPitch4;
    ogPitch4 = ogPitch4 + 2*deltaP4;
     pitch4 = ogPitch4;
  }
  if(abs(accel4.raw.y-prevAccely4)<20 && abs(mag4.raw.y-prevMagy4)<10){
    roll4 = ogRoll4;
    //Serial.print("true");
  }
  else{
    deltaR4 = roll4-prevRoll4;
    ogRoll4 = ogRoll4 + 2*deltaR4;
     roll4 = ogRoll4;
  }
  
  
  //Serial.print("    "); Serial.print(accel1.raw.y); Serial.print("   "); 

  prevHeading4 = filter4.getYaw();
  prevPitch4 = filter4.getPitch();
  prevRoll4 = filter4.getRoll();

  prevAccelz4 = accel4.raw.z;
  prevMagz4 = mag4.raw.z;
  prevAccelx4 = accel4.raw.x;
  prevMagx4 = mag4.raw.x;
  prevAccely4 = accel4.raw.y;
  prevMagy4 = mag4.raw.y;
  
  Serial.print(heading4); Serial.print(" ");
  Serial.print(pitch4); Serial.print(" ");
  Serial.print(roll4); Serial.println(" ");
}

void flexFunction(){
  sensorValue1 = analogRead(flex1);
  sensorValue1 = map(sensorValue1, sensorMin1, sensorMax1, 0, 90);
  //sensorValue1 = constrain(sensorValue1, 0, 255);
  sensorValue1 = -(sensorValue1-90);
  sensorValue2 = analogRead(flex2);
  sensorValue2 = map(sensorValue2, sensorMin2, sensorMax2, 0, 90);
  //sensorValue2 = constrain(sensorValue2, 0, 255);
  sensorValue2 = -(sensorValue2-90);
  sensorValue3 = analogRead(flex3);
  sensorValue3 = map(sensorValue3, sensorMin3, sensorMax3, 0, 90);
  //sensorValue3 = constrain(sensorValue3, 0, 255);
  sensorValue3 = -(sensorValue3-90);
  sensorValue4 = analogRead(flex4);
  sensorValue4 = map(sensorValue4, sensorMin4, sensorMax4, 0, 90);
  //sensorValue4 = constrain(sensorValue4, 0, 255);
  sensorValue4 = -(sensorValue4-90);
  sensorValue5 = analogRead(flex5);
  sensorValue5 = map(sensorValue5, sensorMin5, sensorMax5, 0, 110);
 // sensorValue1 = constrain(sensorValue1, 0, 255);
  sensorValue5 = -(sensorValue5-110);
  sensorValue6 = analogRead(flex6);
  sensorValue6 = map(sensorValue6, sensorMin6, sensorMax6, 0, 110);
  //sensorValue2 = constrain(sensorValue2, 0, 255);
  sensorValue6 = -(sensorValue6-110);
  sensorValue7 = analogRead(flex7);
  sensorValue7 = map(sensorValue7, sensorMin7, sensorMax7, 0, 110);
  //sensorValue3 = constrain(sensorValue3, 0, 255);
  sensorValue7 = -(sensorValue7-110);
  sensorValue8 = analogRead(flex8);
  sensorValue8 = map(sensorValue8, sensorMin8, sensorMax8, 0, 110);
  //sensorValue4 = constrain(sensorValue4, 0, 255);
  sensorValue8 = -(sensorValue8-110);
  //sensor 9 is not used
  sensorValue9 = analogRead(flex9);
  sensorValue9 = map(sensorValue9, sensorMin9, sensorMax9, 0, 90);
  //sensorValue4 = constrain(sensorValue4, 0, 255);
  //sensorValue9 = -(sensorValue9-60);
  
  sensorValue10 = analogRead(flex10);
  sensorValue10 = map(sensorValue10, sensorMin10, sensorMax10, 0, 90);
  //sensorValue4 = constrain(sensorValue4, 0, 255);
  sensorValue10 = -(sensorValue10-90);
  
  sensorValue11 = analogRead(flex11);
  sensorValue11 = map(sensorValue11, sensorMin11, sensorMax11, 0, 90);
  //sensorValue4 = constrain(sensorValue4, 0, 255);
  sensorValue11 = -(sensorValue11-90);
  sensorValue11 = sensorValue11-sensorValue5/4;
  //sensorValue11 = map(sensorValue11, -110, 90, 0, 80);
  
  sensorValue12 = analogRead(flex12);
  sensorValue12 = map(sensorValue12, sensorMin12, sensorMax12, 0, 90);
  //sensorValue4 = constrain(sensorValue4, 0, 255);
  //sensorValue12 = -(sensorValue12-90);
  sensorValue12 = sensorValue12-sensorValue6/4;
  //sensorValue12 = map(sensorValue12, -110, 90, 0, 80);
  
  sensorValue13 = analogRead(flex13);
  sensorValue13 = map(sensorValue13, sensorMin13, sensorMax13, 0, 90);
  //sensorValue4 = constrain(sensorValue4, 0, 255);
  sensorValue13 = -(sensorValue13-90);
  sensorValue13 = sensorValue13-sensorValue7/4;
  //sensorValue13 = map(sensorValue13, -110, 90, 0, 80);

  total1 = total1 - readings1[readIndex];
  readings1[readIndex] = sensorValue1;
  total1 = total1 + readings1[readIndex];
  average1 = total1 / numReadings;

  total2 = total2 - readings2[readIndex];
  readings2[readIndex] = sensorValue2;
  total2 = total2 + readings2[readIndex];
  average2 = total2 / numReadings;

  total3 = total3 - readings3[readIndex];
  readings3[readIndex] = sensorValue3;
  total3 = total3 + readings3[readIndex];
  average3 = total3 / numReadings;

  total4 = total4 - readings4[readIndex];
  readings4[readIndex] = sensorValue4;
  total4 = total4 + readings4[readIndex];
  average4 = total4 / numReadings;

  total5 = total5 - readings5[readIndex];
  readings5[readIndex] = sensorValue5;
  total5 = total5 + readings5[readIndex];
  average5 = total5 / numReadings;

  total6 = total6 - readings6[readIndex];
  readings6[readIndex] = sensorValue6;
  total6 = total6 + readings6[readIndex];
  average6 = total6 / numReadings;

  total7 = total7 - readings7[readIndex];
  readings7[readIndex] = sensorValue7;
  total7 = total7 + readings7[readIndex];
  average7 = total7 / numReadings;

  total8 = total8 - readings8[readIndex];
  readings8[readIndex] = sensorValue8;
  total8 = total8 + readings8[readIndex];
  average8 = total8 / numReadings;

  total9 = total9 - readings9[readIndex];
  readings9[readIndex] = sensorValue9;
  total9 = total9 + readings9[readIndex];
  average9 = total9 / numReadings;

  total10 = total10 - readings10[readIndex];
  readings10[readIndex] = sensorValue10;
  total10 = total10 + readings10[readIndex];
  average10 = total10 / numReadings;

  total11 = total11 - readings11[readIndex];
  readings11[readIndex] = sensorValue11;
  total11 = total11 + readings11[readIndex];
  average11 = total11 / numReadings;

  total12 = total12 - readings12[readIndex];
  readings12[readIndex] = sensorValue12;
  total12 = total12 + readings12[readIndex];
  average12 = total12 / numReadings;

  total13 = total13 - readings13[readIndex];
  readings13[readIndex] = sensorValue13;
  total13 = total13 + readings13[readIndex];
  average13 = total13 / numReadings;
  
  readIndex++;
  if (readIndex >= numReadings) {
    readIndex = 0;
  }
  
  Serial.print(average1);
  Serial.print(" ");
  Serial.print(average2);
  Serial.print(" ");
  Serial.print(average3);
  Serial.print(" ");
  Serial.print(average4);
  Serial.print(" ");
  Serial.print(sensorValue5);
  Serial.print(" ");
  Serial.print(sensorValue6);
  Serial.print(" ");
  Serial.print(sensorValue7);
  Serial.print(" ");
  Serial.print(sensorValue8);
  Serial.print(" ");
  Serial.print(sensorValue9);
  Serial.print(" ");
  Serial.print(sensorValue10);
  Serial.print(" ");
  Serial.print(sensorValue11);
  Serial.print(" ");
  Serial.print(sensorValue12);
  Serial.print(" ");
  Serial.print(sensorValue13);
  Serial.print(" ");
}

void serialFlush(){
   byte w = 0;

 for (int i = 0; i < 10; i++)
 {
     if (Serial.available()) 
   { 
     char val="";
     while(Serial.available())
     {
         char t= Serial.read(); 
         val=val+t;
     }
     Serial.println(val);
     Serial.end();    // Ends the serial communication once all data is received
     Serial.begin(250000);\
   }
  }
  Serial.flush();
}   



