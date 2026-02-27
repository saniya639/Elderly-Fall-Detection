#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <Adafruit_BMP280.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <LiquidCrystal_I2C.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>

/* ================= DEVICES ================= */

MAX30105 particleSensor;
Adafruit_BMP280 bmp;
Adafruit_MPU6050 mpu;
LiquidCrystal_I2C lcd(0x27,16,2);

TinyGPSPlus gps;

HardwareSerial gpsSerial(1);
HardwareSerial gsmSerial(2);

/* ================= HEART ================= */

long lastBeat = 0;
float bpm = 0;
float avgBPM = 0;

/* ================= SPO2 ================= */

float spo2 = 97;
float spo2Avg = 97;

/* ================= TEMP ================= */

float tempFiltered = 0;

/* ================= FALL ================= */

bool fallDetected=false;
bool smsSent=false;

/* ================= GPS ================= */

String latitude="0";
String longitude="0";

/* ================================================= */

void updateGPS()
{
  unsigned long start=millis();

  while(millis()-start < 5000)
  {
    while(gpsSerial.available())
      gps.encode(gpsSerial.read());

    if(gps.location.isValid())
    {
      latitude=String(gps.location.lat(),6);
      longitude=String(gps.location.lng(),6);
      return;
    }
  }
}

/* ================================================= */

void sendSMS()
{
  gsmSerial.println("AT+CMGF=1");
  delay(1000);

  gsmSerial.println("AT+CMGS=\"+919XXXXXXXXX\""); // CHANGE NUMBER
  delay(1000);

  gsmSerial.println("EMERGENCY ALERT!");
  gsmSerial.print("Heart Rate: ");
  gsmSerial.println(avgBPM);

  gsmSerial.println("Fall Detected!");

  gsmSerial.print("Location:");
  gsmSerial.print("https://maps.google.com/?q=");
  gsmSerial.print(latitude);
  gsmSerial.print(",");
  gsmSerial.println(longitude);

  gsmSerial.write(26);
}

/* ================================================= */

void setup()
{
  Serial.begin(115200);
  Wire.begin(21,22);

  lcd.init();
  lcd.backlight();
  lcd.print("Health Monitor");
  delay(2000);
  lcd.clear();

  /* MAX30102 */
  particleSensor.begin(Wire);
  particleSensor.setup(60,4,2,200,411,4096);
  particleSensor.setPulseAmplitudeIR(0x24);
  particleSensor.setPulseAmplitudeRed(0x24);

  /* BMP280 */
  bmp.begin(0x76);

  /* MPU6050 */
  mpu.begin();

  /* GPS UART */
  gpsSerial.begin(9600,SERIAL_8N1,32,33);

  /* GSM UART */
  gsmSerial.begin(9600,SERIAL_8N1,26,27);

  delay(3000);
  gsmSerial.println("AT");
  delay(1000);
  gsmSerial.println("AT+CMGF=1");

  Serial.println("System Ready");
}

/* ================================================= */

void loop()
{

/* ================= HEART RATE ================= */

  long irValue=particleSensor.getIR();
  long redValue=particleSensor.getRed();

  if(irValue>25000)
  {
    if(checkForBeat(irValue))
    {
      long delta=millis()-lastBeat;
      lastBeat=millis();

      bpm=60.0/(delta/1000.0);

      if(bpm>45 && bpm<160)
        avgBPM=(0.6*avgBPM)+(0.4*bpm);
    }
  }
  else avgBPM=0;

/* ================= SPO2 ================= */

  if(irValue>25000)
  {
    float ratio=(float)redValue/(float)irValue;
    float calcSpO2=104-(17*ratio);

    spo2Avg=0.9*spo2Avg+0.1*calcSpO2;

    if(spo2Avg>100) spo2Avg=99;
    if(spo2Avg<92) spo2Avg=92;

    spo2=spo2Avg;
  }

/* ================= TEMPERATURE ================= */

  float tempRaw=bmp.readTemperature();
  tempFiltered=0.9*tempFiltered+0.1*tempRaw;

/* ================= FALL DETECTION ================= */

  sensors_event_t a,g,t;
  mpu.getEvent(&a,&g,&t);

  float acc=sqrt(
      a.acceleration.x*a.acceleration.x+
      a.acceleration.y*a.acceleration.y+
      a.acceleration.z*a.acceleration.z);

  float angle=
      atan2(a.acceleration.y,
             a.acceleration.z)*57.3;

  static bool impact=false;
  static unsigned long fallTime=0;

  if(acc>20)
  {
    impact=true;
    fallTime=millis();
  }

  if(impact && abs(angle)>60 &&
     millis()-fallTime<1500)
  {
    delay(800);

    mpu.getEvent(&a,&g,&t);

    float stillAcc=sqrt(
        a.acceleration.x*a.acceleration.x+
        a.acceleration.y*a.acceleration.y+
        a.acceleration.z*a.acceleration.z);

    if(stillAcc<11)
      fallDetected=true;

    impact=false;
  }

/* ================= GSM + GPS ALERT ================= */

  if(fallDetected && !smsSent)
  {
    lcd.clear();
    lcd.print("Sending Alert");

    updateGPS();
    sendSMS();

    smsSent=true;
  }

/* ================= LCD ================= */

  static unsigned long lcdTimer=0;

  if(millis()-lcdTimer>1000)
  {
    lcdTimer=millis();

    lcd.clear();

    lcd.setCursor(0,0);
    lcd.print("T:");
    lcd.print(tempFiltered,1);
    lcd.print(" O2:");
    lcd.print((int)spo2);

    lcd.setCursor(0,1);

    if(fallDetected)
      lcd.print("FALL ALERT!");
    else
    {
      lcd.print("HR:");
      lcd.print((int)avgBPM);
      lcd.print(" BPM");
    }
  }

  delay(5);
}
