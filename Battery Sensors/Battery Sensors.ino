/*
 2xDHT22, BMP085, Voltage  sensor
 
 V2.2 19.10.2015 - first version
 
 Created by Igor Jarc <igor.jarc1@gmail.com>
 See http://iot-playground.com for details
 
 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.
 */
#include <avr/sleep.h>    // Sleep Modes
#include <MySensor.h>
#include <DHT.h> 
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP085.h> // N.b. The new library includes the function readSealevelPressure()  

unsigned long SLEEP_TIME = 900000; // sleep time between reads (seconds * 1000 milliseconds)

#define CHILD_ID_HUM_1 0
#define CHILD_ID_TEMP_1 1
#define CHILD_ID_HUM_2 2
#define CHILD_ID_TEMP_2 3
#define BARO_CHILD 4

#define HUMIDITY_SENSOR_DIGITAL_PIN_1 4
#define HUMIDITY_SENSOR_DIGITAL_PIN_2 5
#define DHTTYPE_1 DHT22   // DHT 22  (AM2302), AM2321,DHT 11,DHT 21 (AM2301)
#define DHTTYPE_2 DHT22   // DHT 22  (AM2302), AM2321,DHT 11,DHT 21 (AM2301)

#define ENABLE_SENSORS_DHT_1 1
#define ENABLE_SENSORS_DHT_2 1
#define ENABLE_SENSORS_BARO_BMP085 0


// when ADC completed, take an interrupt 
EMPTY_INTERRUPT (ADC_vect);

#ifdef ENABLE_SENSORS_BARO_BMP085
Adafruit_BMP085 bmp = Adafruit_BMP085();      // Digital Pressure Sensor 
#endif

MySensor gw;

#if ENABLE_SENSORS_DHT_1 > 0
DHT dht(HUMIDITY_SENSOR_DIGITAL_PIN_1, DHTTYPE_1);
#endif
#if ENABLE_SENSORS_DHT_2  > 0
DHT dht_2(HUMIDITY_SENSOR_DIGITAL_PIN_2, DHTTYPE_2);
#endif

byte val;
int oldBatLevel;

float lastTemp_1;
float lastHum_1;
float lastTemp_2;
float lastHum_2;
float lastPressure;
boolean metric = false;
#if ENABLE_SENSORS_DHT_1 > 0
MyMessage msgHum(CHILD_ID_HUM_1, V_HUM);
MyMessage msgTemp(CHILD_ID_TEMP_1, V_TEMP);
#endif
#if ENABLE_SENSORS_DHT_2 > 0
MyMessage msgHum_2(CHILD_ID_HUM_2, V_HUM);
MyMessage msgTemp_2(CHILD_ID_TEMP_2, V_TEMP);
#endif
#if ENABLE_SENSORS_BARO_BMP085  > 0
MyMessage pressureMsg(BARO_CHILD, V_PRESSURE);
#endif

void setup()
{
  analogReference(INTERNAL);    // use the 1.1 V internal reference for battery level measuring
  delay(500); // Allow time for radio if power used as reset <<<<<<<<<<<<<< Experimented with good result 
  Serial.begin(115200);
  Serial.flush();
  Serial.println("EgHumBarTemBat_v2  2.2 19.10.2015");
  
  //gw.begin(NULL,NODE_ID); // Startup and initialize MySensors library. Set callback for incoming messages. 
  gw.begin();


  // Send the Sketch Version Information to the Gateway
  gw.sendSketchInfo("EgHumBarTemBat_v2", "2.2 19.10.2015");

  // Register all sensors to gw (they will be created as child devices)
  #if ENABLE_SENSORS_DHT_1 > 0
  gw.present(CHILD_ID_HUM_1, S_HUM);
  gw.present(CHILD_ID_TEMP_1, S_TEMP);
  #endif
  #if ENABLE_SENSORS_DHT_2 > 0
  gw.present(CHILD_ID_HUM_2, S_HUM);
  gw.present(CHILD_ID_TEMP_2, S_TEMP);
  #endif
  #if ENABLE_SENSORS_BARO_BMP085 > 0
  gw.present(BARO_CHILD, S_BARO);
  #endif
    
  metric = gw.getConfig().isMetric;
  oldBatLevel = -1;  
  gw.sendBatteryLevel(oldBatLevel); 
  
  sendValue();
}

void loop ()
{   
  Serial.println("------LOOP-------");
  sendValue();
  gw.sleep(SLEEP_TIME); //sleep a bit
}  // end of loop


void sendValue()
{
  gw.wait(200);
    int batLevel = getBatteryLevel();
    Serial.print("oldBatLevel: ");
    Serial.println(oldBatLevel);
    Serial.print("BatLevel: ");
    Serial.println(batLevel);
  if (oldBatLevel != batLevel)
  {
    Serial.print("--Send Node");
    gw.sendBatteryLevel(batLevel);    
    oldBatLevel = batLevel;
  }
  #if ENABLE_SENSORS_DHT_1 > 0
  float temperature_1 = dht.readTemperature();
  if (isnan(temperature_1)) {
      Serial.println("Failed reading temperature from Temp 1");
  } else if (temperature_1 != lastTemp_1) {
    lastTemp_1 = temperature_1;
    gw.send(msgTemp.set(temperature_1, 1));
    Serial.print("Temperature: ");
    Serial.println(temperature_1);
  }
  float humidity_1 = dht.readHumidity();
  if (isnan(humidity_1)) {
      Serial.println("Failed reading humidity from DHT 1");
  } else if (humidity_1 != lastHum_1) {
      lastHum_1 = humidity_1;
      gw.send(msgHum.set(humidity_1, 1));
      Serial.print("Humidity: ");
      Serial.println(humidity_1);
  }
  #endif
    #if ENABLE_SENSORS_DHT_2 > 0
  float temperature_2 = dht_2.readTemperature();
  if (isnan(temperature_2)) {
      Serial.println("Failed reading temperature from Temp 2");
  } else if (temperature_2 != lastTemp_2) {
    lastTemp_2 = temperature_2;
    gw.send(msgTemp_2.set(temperature_2, 1));
    Serial.print("Temperature: ");
    Serial.println(temperature_2);
  }
  float humidity_2 = dht_2.readHumidity();
  if (isnan(humidity_2)) {
      Serial.println("Failed reading humidity from DHT 2");
  } else if (humidity_2 != lastHum_2) {
      lastHum_2 = humidity_2;
      gw.send(msgHum_2.set(humidity_2, 1));
      Serial.print("Humidity: ");
      Serial.println(humidity_2);
  }
  #endif
  #if ENABLE_SENSORS_BARO_BMP085 > 0
  float pressure = bmp.readPressure()/100;
  Serial.println(pressure);
  if (isnan(pressure)) {
      Serial.println("Failed reading pressure from BMP085");
  } else if (pressure != lastPressure) {
      lastPressure = pressure;
      gw.send(msgHum.set(pressure, 1));
      Serial.print("Pressure: ");
      Serial.println(pressure);
  }
  #endif
 
}

// Battery measure
int getBatteryLevel () 
{
  int results = (readVcc() - 2000)  / 10;   

  if (results > 100)
    results = 100;
  if (results < 0)
    results = 0;
  return results;
} // end of getBandgap



long readVcc() {
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  noInterrupts ();
  // start the conversion
  ADCSRA |= _BV (ADSC) | _BV (ADIE);
  set_sleep_mode (SLEEP_MODE_ADC);    // sleep during sample
  interrupts ();
  sleep_mode (); 
  // reading should be done, but better make sure
  // maybe the timer interrupt fired 
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1126400L / result; // Back-calculate AVcc in mV

  return result;
}
