#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>



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

  // default settings
  // (you can also pass in a Wire library object like &Wire2)
  status = bme.begin(0x76);  
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
}

void loop() {
  // Connect or reconnect to WiFi

  int mq2data = analogRead(mq2AnaloguePin);
  
  measure();
  Serial.println("Wind Speed : "+String(WindSpeed)+" km/h - "+String(WindSpeed / 3.6)+" m/s");
  Serial.println("Smoke  : "+String(mq2data));
  Serial.println("Temperature : "+String(bme.readTemperature())+" *C");
  Serial.println("Pressure : "+String(bme.readPressure() / 100.0F)+" hPa");
  Serial.println("Approx. Altitude : "+String(bme.readAltitude(SEALEVELPRESSURE_HPA))+" m");
  Serial.println("Humidity : "+String(bme.readHumidity())+" %");
  Serial.println();
  delay(1000);
}