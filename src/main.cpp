#include <Arduino.h>
#include <Wire.h>
#include "SparkFunMPL3115A2.h"

MPL3115A2 pressure_sensor;
float altitude_offset = 0;


#include <SBUS2.h>
#define VARIO_SLOT        21    // 2 Slot Sensor


//#define DEBUG

void setup() {

  // Setup for the MPL3115A2 Pressure Sensor

  Wire.begin();        // Join i2c bus
  
  pressure_sensor.begin(); // Get sensor online

  //Configure the sensor
  pressure_sensor.setModeAltimeter(); // Measure altitude above sea level in meters
  //pressure_sensor.setModeBarometer(); // Measure pressure in Pascals from 20 to 110 kPa

  pressure_sensor.setOversampleRate(7); // Set Oversample to the recommended 128
  pressure_sensor.enableEventFlags(); // Enable all three pressure and temp event flags 

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  #ifndef DEBUG
    // Setup for SBUS2 Vario Telemetry sensor
    SBUS2_Setup();
    //send_f1672_vario(VARIO_SLOT, (int16_t) 0, (int16_t) 0);
  #endif
  #ifdef DEBUG
    Serial.begin(9600);
  #endif

  uint8_t samples = 15;
  for(int i=0; i < samples; i++){
    altitude_offset += pressure_sensor.readAltitude();
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);

    #ifdef DEBUG
      Serial.print("*");
    #endif
    #ifndef DEBUG
      if(SBUS2_Ready()){
        send_s1678_current(VARIO_SLOT, (float) 0.0, i, (float) 0.0); 
      }
    #endif
  }
  altitude_offset = altitude_offset/samples;
  #ifdef DEBUG
    Serial.println(altitude_offset);
  #endif

}


void loop() {

  float altitude = 0;
  uint8_t nSamples = 3;
  for(int i = 0; i<nSamples; i++){
    altitude += pressure_sensor.readAltitude();
    if(i < 2){
      delay(1000);
    }else{
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100); 
      digitalWrite(LED_BUILTIN, LOW);
      delay(900);
    }
  }
  altitude = (altitude/nSamples) - altitude_offset;
  
  #ifndef DEBUG
    if(SBUS_Ready()){
      send_s1678_current(VARIO_SLOT, (float) 0, 0, altitude);    
    }
  #endif
  #ifdef DEBUG
    Serial.print("Altitude: "); Serial.println(altitude);
  #endif
  // The pressure sensor has an update interval of 1s


}