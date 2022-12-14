#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <WiFi.h>
#include "ThingSpeak.h" // always include thingspeak header file after other header files and custom macros
#include <DMD32.h> 
#include "fonts/SystemFont5x7.h"
#include "fonts/Arial_Black_16_ISO_8859_1.h"

#define DISPLAYS_ACROSS 4
#define DISPLAYS_DOWN 1
DMD dmd(DISPLAYS_ACROSS, DISPLAYS_DOWN);
hw_timer_t * timer = NULL;

void IRAM_ATTR triggerScan()
{
  dmd.scanDisplayBySPI();
}

const int SensorPin = 34;
const int mq2AnaloguePin = 35;

const int RecordTime = 3;
int InterruptCounter;
float WindSpeed;

#define SEALEVELPRESSURE_HPA (1013.25)


#define BME_SCK 14
#define BME_MISO 12
#define BME_MOSI 13
#define BME_CS 15

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK);
unsigned long delayTime;


void countup() {
  InterruptCounter++;
}

void measure() {
  InterruptCounter = 0;
  attachInterrupt(digitalPinToInterrupt(SensorPin), countup, RISING);
  delay(1000 * RecordTime);
  detachInterrupt(digitalPinToInterrupt(SensorPin));
  WindSpeed = (float)InterruptCounter / (float)RecordTime * 2.4;
}

void setup(void)
{
  uint8_t cpuClock = ESP.getCpuFreqMHz();
  timer = timerBegin(0, cpuClock, true);
  timerAttachInterrupt(timer, &triggerScan, true);
  timerAlarmWrite(timer, 300, true);
  timerAlarmEnable(timer);
  dmd.clearScreen( true );   //true is normal (all pixels off), false is negative (all pixels on)
  
  Serial.begin(115200);

  pinMode(mq2AnaloguePin, INPUT);
  Serial.println(F("BME280 test"));

  bool status;
  status = bme.begin(0x76);  
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
}

int mq2data = 0;
float temp = 0;
float pressure = 0;
float altitude = 0;
float humidity = 0;
void loop(void)
{
   // 10 x 14 font clock, including demo of OR and NOR modes for pixels so that the flashing colon can be overlayed
   dmd.clearScreen( true );
   dmd.selectFont(Arial_Black_16_ISO_8859_1);
   
   
   String pesan = "Wind Speed: " + String(WindSpeed) + " km/h || Asap : " + String(mq2data) + " || Temp : " + String(temp) + " C || Humidity : " + String(humidity) + " % || Pressure : " + String(pressure) + " hPa || Altitude : " + String(altitude) + " m";
   dmd.drawMarquee(pesan.c_str(),pesan.length(),(32*DISPLAYS_ACROSS)-1,0);
   long start=millis();
   long timer=start;
   boolean ret=false;
   while(!ret){
     if ((timer+30) < millis()) {
       ret=dmd.stepMarquee(-1,0);
       timer=millis();
     }
   }
   
 
  mq2data = analogRead(mq2AnaloguePin);
  temp = bme.readTemperature();
  pressure = bme.readPressure() / 100.0F;
  altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
  humidity = bme.readHumidity();
}
