#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <WiFi.h>
#include "ThingSpeak.h" // always include thingspeak header file after other header files and custom macros
#include <WiFiManager.h>
#include <DMD32.h> 
#include "fonts/SystemFont5x7.h"
#include "fonts/Arial_Black_16_ISO_8859_1.h"

char ssid[] = "MAKERINDO2";   // your network SSID (name) 
char pass[] = "makerindo2019";   // your network password
int keyIndex = 0;            // your network key Index number (needed only for WEP)
WiFiClient  client;

unsigned long myChannelNumber = 3;
const char * myWriteAPIKey = "CBHFE8I3YQ0S6A0G";

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
#define SEALEVELPRESSURE_HPA (1013.25)
#define BME_SCK 14
#define BME_MISO 12
#define BME_MOSI 13
#define BME_CS 15

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK);

unsigned long delayTime;





void setup()
{
  uint8_t cpuClock = ESP.getCpuFreqMHz();
  timer = timerBegin(0, cpuClock, true);
  timerAlarmWrite(timer, 300, true);
  timerAlarmEnable(timer);
  dmd.clearScreen( true );

  Serial.begin(115200);
  pinMode(mq2AnaloguePin, INPUT);
  Serial.println(F("BME280 test"));

  bool status;
  status = bme.begin(0x76);  
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
  WiFiManager wm;

   bool res;
    res = wm.autoConnect("Weather Station - 01"); // password protected ap
    if(!res) {
        Serial.println("Failed to connect");
    } 
    else {
        //if you get here you have connected to the WiFi    
        Serial.println("connected...yeey :)");
    }   
  ThingSpeak.begin(client);  // Initialize ThingSpeak
}

float WindSpeed = 0;
float mq2data = 0;
float temp = 0;
float pressure = 0;
float altitude = 0;
float humidity = 0;

void sensorRead()
{
  float sensorValue = analogRead(SensorPin);
  float voltage = (sensorValue / 1023) * 5;
  WindSpeed = mapfloat(voltage, 0.4, 2, 0, 32.4);
  mq2data = analogRead(mq2AnaloguePin);
  temp = bme.readTemperature();
  pressure = bme.readPressure() / 100.0F;
  altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
  humidity = bme.readHumidity();
}
void sensorDisplay(float windSpeed, float mq2data, float temp, float pressure, float altitude, float humidity)
{
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
}

void loop() {
  // Connect or reconnect to WiFi
  sensorRead();
  sensorDisplay(WindSpeed, mq2data, temp, pressure, altitude, humidity);

  Serial.println("Wind Speed       : "+String(WindSpeed)+" km/h - "+String(WindSpeed / 3.6)+" m/s");
  Serial.println("Smoke            : "+String(mq2data));
  Serial.println("Temperature      : "+String(temp)+" *C");
  Serial.println("Pressure         : "+String(pressure)+" hPa");
  Serial.println("Approx. Altitude : "+String(altitude)+" m");
  Serial.println("Humidity         : "+String(humidity)+" %");
  Serial.println();

  
  


  ThingSpeak.setField(1, WindSpeed);
  ThingSpeak.setField(2, mq2data);
  ThingSpeak.setField(3, pressure);
  ThingSpeak.setField(4, temp);
  ThingSpeak.setField(5, humidity);
  ThingSpeak.setField(6, altitude);

  // set the status
  ThingSpeak.setStatus("myStatus OK");
  
  // write to the ThingSpeak channel
  int x = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
  if(x == 200){
    Serial.println("Channel update successful.");
  }
  else{
    Serial.println("Problem updating channel. HTTP error code " + String(x));
  }
  delay(20000);
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
