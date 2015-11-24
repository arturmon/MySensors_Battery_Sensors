/*
 weather_station_v3 sensor
 
 V2.5 - first version
 
 Created by Arturmon <arturmon82@gmail.com>

 */

// Enable debug prints to serial monitor
#define MY_DEBUG 

// Enable and select radio type attached
#define MY_RADIO_NRF24
//#define MY_RADIO_RFM69

#include <avr/sleep.h>    // Sleep Modes
#include <MySensor.h>
#include <DHT.h> 
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP085.h> // N.b. The new library includes the function readSealevelPressure()  

unsigned long SLEEP_TIME = 900000; // sleep time between reads (seconds * 1000 milliseconds)

#define BARO_CHILD 0
#define CHILD_ID_HUM_1 1
#define CHILD_ID_TEMP_1 2
#define CHILD_ID_HUM_2 3
#define CHILD_ID_TEMP_2 4



#define HUMIDITY_SENSOR_DIGITAL_PIN_1 4
#define HUMIDITY_SENSOR_DIGITAL_PIN_2 5

// when ADC completed, take an interrupt 
EMPTY_INTERRUPT (ADC_vect);

Adafruit_BMP085 bmp = Adafruit_BMP085();      // Digital Pressure Sensor 

DHT dht;
DHT dht_2;


byte val;
int oldBatLevel = -1;

float lastTemp_1 = -1;
float lastHum_1 = -1;
float lastTemp_2 = -1;
float lastHum_2 = -1;
float lastPressure = -1;
boolean metric = false;
MyMessage msgHum(CHILD_ID_HUM_1, V_HUM);
MyMessage msgTemp(CHILD_ID_TEMP_1, V_TEMP);
MyMessage msgHum_2(CHILD_ID_HUM_2, V_HUM);
MyMessage msgTemp_2(CHILD_ID_TEMP_2, V_TEMP);
MyMessage pressureMsg(BARO_CHILD, V_PRESSURE);




void presentation()  {
  // Send the Sketch Version Information to the Gateway
  sendSketchInfo("weather_station_v3", "2.5 24.11.2015"); 

     // Register all sensors to gw (they will be created as child devices)
  present(CHILD_ID_HUM_1, S_HUM);
  present(CHILD_ID_TEMP_1, S_TEMP);
  present(CHILD_ID_HUM_2, S_HUM);
  present(CHILD_ID_TEMP_2, S_TEMP);
  present(BARO_CHILD, S_BARO);
}


void setup()
{
  analogReference(INTERNAL);    // use the 1.1 V internal reference for battery level measuring
  delay(500); // Allow time for radio if power used as reset <<<<<<<<<<<<<< Experimented with good result 
  Serial.begin(115200);
  Serial.flush();
  Serial.println("weather_station_v3  2.5 24.11.2015");
  Serial.print("CPU SPEED:");
  Serial.println(F_CPU );
  dht.setup(HUMIDITY_SENSOR_DIGITAL_PIN_1);
  dht_2.setup(HUMIDITY_SENSOR_DIGITAL_PIN_2);
  metric = getConfig().isMetric;
//  sendBatteryLevel(oldBatLevel); 

//  sendValue();
}

void loop ()
{   
  Serial.println("-------Send Value-------");
  sendBatteryLevelToNode();
  wait(dht.getMinimumSamplingPeriod());
  sendValue_DHT1();
  wait(dht.getMinimumSamplingPeriod());
  sendValue_DHT2();
  sendValue_BMP();
  Serial.println("---------Sleep----------");
  sleep(SLEEP_TIME); //sleep a bit
  }  // end of loop


void sendValue_DHT1()
{
     // delay(dht.getMinimumSamplingPeriod());
  float temperature = dht.getTemperature();
  if (isnan(temperature)) {
      Serial.println("Failed reading temperature from Temp 1");
  } else if (temperature != lastTemp_1) {
    lastTemp_1 = temperature;
    send(msgTemp.set(temperature, 1));
    Serial.print("Temperature 1: ");
    Serial.println(temperature);
  }
  float humidity = dht.getHumidity();
    if (isnan(humidity)) {
      Serial.println("Failed reading humidity from DHT 1");
  } else if (humidity != lastHum_1) {
      lastHum_1 = humidity;
      send(msgHum.set(humidity, 1));
      Serial.print("Humidity 1: ");
      Serial.println(humidity);
  }
} 

void sendValue_DHT2()
{
    float temperature = dht_2.getTemperature();
  if (isnan(temperature)) {
      Serial.println("Failed reading temperature from Temp 2");
  } else if (temperature != lastTemp_2) {
    lastTemp_2 = temperature;
    send(msgTemp_2.set(temperature, 1));
    Serial.print("Temperature 2: ");
    Serial.println(temperature);
  }
  float humidity = dht_2.getHumidity();
  if (isnan(humidity)) {
      Serial.println("Failed reading humidity from DHT 2");
  } else if (humidity != lastHum_2) {
      lastHum_2 = humidity;
      send(msgHum_2.set(humidity, 1));
      Serial.print("Humidity 2: ");
      Serial.println(humidity);
  }
}

void sendValue_BMP()
{
  float pressure = bmp.readPressure()/100;
  //float pressure = bmp.readSealevelPressure(ALTITUDE) / 100.0;
  Serial.println(pressure);
  if (isnan(pressure)) {
      Serial.println("Failed reading pressure from BMP085");
  } else if (pressure != lastPressure) {
      lastPressure = pressure;
      send(msgHum.set(pressure, 1));
      Serial.print("Pressure: ");
      Serial.println(pressure);
  }
}


// Send Battery measure
void sendBatteryLevelToNode(){
    int batLevel = getBatteryLevel();
    Serial.print("oldBatLevel: ");
    Serial.println(oldBatLevel);
    Serial.print("BatLevel: ");
    Serial.println(batLevel);
  if (oldBatLevel != batLevel)
  {
    Serial.println("Send Node Battery State");
    sendBatteryLevel(batLevel);    
    oldBatLevel = batLevel;
  }
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

