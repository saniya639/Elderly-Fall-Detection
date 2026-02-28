#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <LiquidCrystal_I2C.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>

/*** DEVICES ***/
MAX30105 particleSensor;
Adafruit_MPU6050 mpu;
LiquidCrystal_I2C lcd(0x27,16,2);

HardwareSerial gsm(2);
HardwareSerial gpsSerial(1);
TinyGPSPlus gps;

/*** PHONE ***/
String phone="+918600688148";

/*** HEART ***/
long lastBeat=0;
float avgBPM=0;
float spo2=97;

/*** TEMP HW611 ***/
#define TEMP_PIN 34
float temperature=0;

/*** GPS ***/
float latitude=0;
float longitude=0;

/*** FALL ***/
bool fallDetected=false;
bool emergencyDone=false;
float accValue=0;

/*** GPS READ ***/
void readGPS()
{
  while(gpsSerial.available())
      gps.encode(gpsSerial.read());

  if(gps.location.isValid())
  {
    latitude=gps.location.lat();
    longitude=gps.location.lng();
  }
}

/*** GSM SMS ***/
void sendSMS()
{
  String link=
  "https://maps.google.com/?q="
  +String(latitude,6)+","
  +String(longitude,6);

  Serial.println("Sending SMS");

  gsm.println("AT+CMGF=1");
  delay(1000);

  gsm.print("AT+CMGS=\""+phone+"\"\r");
  delay(2000);

  gsm.println("ELDER FALL ALERT!");
  gsm.print("HR:");
  gsm.println((int)avgBPM);
  gsm.print("Temp:");
  gsm.println(temperature);
  gsm.println(link);

  gsm.write(26);
  delay(6000);
}

/*** CALL ***/
void makeCall()
{
  Serial.println("Calling...");
  gsm.print("ATD"+phone+";");
  delay(20000);
  gsm.println("ATH");
}

/*** HEART ***/
void readHeart()
{
  long ir=particleSensor.getIR();
  long red=particleSensor.getRed();

  if(ir>20000)
  {
    if(checkForBeat(ir))
    {
      float bpm=
      60.0/((millis()-lastBeat)/1000.0);

      lastBeat=millis();

      if(bpm>45 && bpm<160)
        avgBPM=0.7*avgBPM+0.3*bpm;
    }

    float ratio=(float)red/ir;
    spo2=constrain(104-(17*ratio),92,99);
  }
  else avgBPM=0;
}

/*** TEMP ***/
void readTemp()
{
  int val=analogRead(TEMP_PIN);
  float voltage=val*(3.3/4095.0);
  temperature=voltage*100;
}

/*** FALL AI ***/
void detectFall()
{
  sensors_event_t a,g,t;
  mpu.getEvent(&a,&g,&t);

  accValue=sqrt(
  a.acceleration.x*a.acceleration.x+
  a.acceleration.y*a.acceleration.y+
  a.acceleration.z*a.acceleration.z);

  static int state=0;
  static unsigned long timer=0;

  if(state==0 && accValue<4)
  {
    state=1;
    timer=millis();
  }
  else if(state==1 && accValue>18)
  {
    state=2;
    timer=millis();
  }
  else if(state==2)
  {
    if(accValue<10 && millis()-timer>1500)
    {
      fallDetected=true;
      Serial.println("FALL CONFIRMED");
      state=3;
    }
  }

  if(millis()-timer>3000)
      state=0;
}

/*** LCD ***/
void updateLCD()
{
  lcd.setCursor(0,0);
  lcd.print("HR:");
  lcd.print((int)avgBPM);
  lcd.print(" O2:");
  lcd.print((int)spo2);
  lcd.print(" ");

  lcd.setCursor(0,1);

  if(fallDetected)
      lcd.print("FALL ALERT!!! ");
  else
  {
      lcd.print("T:");
      lcd.print(temperature,1);
      lcd.print("C ");
  }
}

/*** SERIAL ***/
void serialOutput()
{
  Serial.println("===== HEALTH DATA =====");

  Serial.print("BPM: ");
  Serial.println(avgBPM);

  Serial.print("SpO2: ");
  Serial.println(spo2);

  Serial.print("Temp: ");
  Serial.println(temperature);

  Serial.print("ACC: ");
  Serial.println(accValue);

  Serial.print("GPS: ");
  Serial.print(latitude,6);
  Serial.print(",");
  Serial.println(longitude,6);

  Serial.println("=======================");
}

/*** SETUP ***/
void setup()
{
  Serial.begin(115200);
  Wire.begin(21,22);

  lcd.init();
  lcd.backlight();

  gsm.begin(9600,SERIAL_8N1,16,17);
  gpsSerial.begin(9600,SERIAL_8N1,4,2);

  particleSensor.begin(Wire);
  particleSensor.setup(60,4,2,200,411,4096);
  particleSensor.setPulseAmplitudeIR(0x24);
  particleSensor.setPulseAmplitudeRed(0x24);

  mpu.begin();
  pinMode(TEMP_PIN,INPUT);

  delay(15000);

  lcd.print("SYSTEM READY");
  delay(1500);
  lcd.clear();
}

/*** LOOP ***/
void loop()
{
  readGPS();
  readHeart();
  readTemp();
  detectFall();

  static unsigned long t1=0;
  if(millis()-t1>1000)
  {
    t1=millis();
    updateLCD();
    serialOutput();
  }

  if(fallDetected && !emergencyDone)
  {
    sendSMS();
    makeCall();
    delay(5000);
    makeCall();

    emergencyDone=true;
  }

  delay(20);
}
