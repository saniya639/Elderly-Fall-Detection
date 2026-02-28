
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
String guardian="+918600688148";

/*** HEART ***/
long lastBeat=0;
float avgBPM=0;

/*** FALL ***/
bool fallDetected=false;
bool emergencyDone=false;

/*** GPS ***/
String latitude="0";
String longitude="0";

/****** GSM INIT ******/
void initGSM()
{
  gsm.println("AT");
  delay(1000);

  gsm.println("ATE0");
  delay(1000);

  gsm.println("AT+CMGF=1");
  delay(1000);
}

/****** READ GPS ******/
void readGPS()
{
  while(gpsSerial.available())
    gps.encode(gpsSerial.read());

  if(gps.location.isValid())
  {
    latitude = String(gps.location.lat(),6);
    longitude = String(gps.location.lng(),6);
  }
}

/****** SEND SMS WITH LOCATION ******/
void sendSMS()
{
  String link =
  "https://maps.google.com/?q=" +
  latitude + "," + longitude;

  Serial.println("Sending Location SMS");

  gsm.print("AT+CMGS=\"");
  gsm.print(guardian);
  gsm.println("\"");

  delay(2000);

  gsm.println("EMERGENCY ALERT!");
  gsm.println("Fall Detected");
  gsm.println("Heart Rate:"+String((int)avgBPM));
  gsm.println(link);

  gsm.write(26);
  delay(6000);
}

/****** CALL ******/
void makeCall()
{
  gsm.print("ATD");
  gsm.print(guardian);
  gsm.println(";");

  delay(20000);

  gsm.println("ATH");
}

/*** HEART ***/
void readHeart()
{
  long ir=particleSensor.getIR();

  if(ir>20000)
  {
    if(checkForBeat(ir))
    {
      long delta=millis()-lastBeat;
      lastBeat=millis();

      float bpm=60.0/(delta/1000.0);

      if(bpm>45 && bpm<160)
        avgBPM=0.7*avgBPM+0.3*bpm;
    }
  }
  else avgBPM=0;
}

/*** SMART FALL ***/
void detectFall()
{
  sensors_event_t a,g,t;
  mpu.getEvent(&a,&g,&t);

  float acc=sqrt(
    a.acceleration.x*a.acceleration.x+
    a.acceleration.y*a.acceleration.y+
    a.acceleration.z*a.acceleration.z);

  static int state=0;
  static unsigned long timer=0;

  if(state==0 && acc<4.5)
  {
    state=1;
    timer=millis();
  }
  else if(state==1 && acc>16)
  {
    state=2;
    timer=millis();
  }
  else if(state==2)
  {
    if(acc<10 && millis()-timer>1500)
    {
      fallDetected=true;
      state=3;
      Serial.println("FALL DETECTED");
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
  lcd.print(" BPM   ");

  lcd.setCursor(0,1);

  if(fallDetected)
      lcd.print("Sending Help ");
  else
      lcd.print("Status Normal");
}

/*** SETUP ***/
void setup()
{
  Serial.begin(115200);
  Wire.begin(21,22);

  gsm.begin(9600,SERIAL_8N1,16,17);
  gpsSerial.begin(9600,SERIAL_8N1,4,2);

  lcd.init();
  lcd.backlight();
  lcd.print("Health Monitor");
  delay(1500);
  lcd.clear();

  particleSensor.begin(Wire);
  particleSensor.setup(60,4,2,200,411,4096);
  particleSensor.setPulseAmplitudeIR(0x24);
  particleSensor.setPulseAmplitudeRed(0x24);

  mpu.begin();

  delay(10000); // GSM + GPS stabilize
  initGSM();

  Serial.println("SYSTEM READY");
}

/*** LOOP ***/
void loop()
{
  readGPS();
  readHeart();
  detectFall();

  if(fallDetected && !emergencyDone)
  {
    sendSMS();
    delay(3000);

    makeCall();
    delay(5000);

    makeCall();   // backup

    emergencyDone=true;
  }

  static unsigned long lcdTimer=0;
  if(millis()-lcdTimer>1000)
  {
    lcdTimer=millis();
    updateLCD();
  }
}
