#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <WiFi.h>
#include "ThingSpeak.h" // always include thingspeak header file after other header files and custom macros
#include <WiFiManager.h>

char ssid[] = "MAKERINDO2";   // your network SSID (name) 
char pass[] = "makerindo2019";   // your network password
int keyIndex = 0;            // your network key Index number (needed only for WEP)
WiFiClient  client;

unsigned long myChannelNumber = 3;
const char * myWriteAPIKey = "CBHFE8I3YQ0S6A0G";



const int SensorPin = 34;
const int mq2AnaloguePin = 35;

const int RecordTime = 3;
int InterruptCounter;
float WindSpeed;

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme; // I2C
//Adafruit_BME280 bme(BME_CS); // hardware SPI
//Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK); // software SPI

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


void setup()
{
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

void loop() {
  // Connect or reconnect to WiFi
  if(WiFi.status() != WL_CONNECTED){
    Serial.print("Attempting to connect to SSID: ");
    Serial.println("MAKERINDO2");
    while(WiFi.status() != WL_CONNECTED){
      WiFi.begin(ssid, pass);  // Connect to WPA/WPA2 network. Change this line if using open or WEP network
      Serial.print(".");
      delay(5000);     
    } 
    Serial.println("\nConnected.");
  }
  int mq2data = analogRead(mq2AnaloguePin);
  float temp = bme.readTemperature();
  float pressure = bme.readPressure() / 100.0F;
  float altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
  float humidity = bme.readHumidity();

  measure();
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