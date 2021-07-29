#include <Arduino.h>
#include <Wire.h>
#include "SparkFunMPL3115A2.h"

MPL3115A2 pressure_sensor;
float altitude_offset = 0;


#include <SBUS2.h>
#define ERROR_SLOT        21     // 1 Slot Sensor
#define VARIO_SLOT        22    // 2 Slot Sensor
#define GPS_SLOT          24    // 8 Slot Sensor

int16_t channel = 0;
uint8_t FrameErrorRate = 0;

#define DEBUG

void setup() {

  // Setup for the MPL3115A2 Pressure Sensor

  Wire.begin();        // Join i2c bus
  
  pressure_sensor.begin(); // Get sensor online

  //Configure the sensor
  pressure_sensor.setModeAltimeter(); // Measure altitude above sea level in meters
  //pressure_sensor.setModeBarometer(); // Measure pressure in Pascals from 20 to 110 kPa

  pressure_sensor.setOversampleRate(7); // Set Oversample to the recommended 128
  pressure_sensor.enableEventFlags(); // Enable all three pressure and temp event flags 

  #ifndef DEBUG
    // Setup for SBUS2 Vario Telemetry sensor
    SBUS2_Setup();
    send_f1672_vario(VARIO_SLOT, (int16_t) 0, (int16_t) 0);
  #endif
  #ifdef DEBUG
    Serial.begin(9600);
  #endif

  uint8_t samples = 15;
  for(int i=0; i < samples; i++){
    altitude_offset += pressure_sensor.readAltitude();
    delay(1000);
    #ifdef DEBUG
      Serial.print("*");
    #endif
    #ifndef DEBUG
      if(SBUS2_Ready()){
        send_alarm_as_temp125(ERROR_SLOT, i);
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
    delay(1000);
  }
  altitude = (altitude/nSamples) - altitude_offset;
  
  #ifndef DEBUG
    uint16_t uart_dropped_frame = 0;
    bool transmission_dropt_frame = false;
    bool failsave = false;

    
    if(SBUS2_Ready()){                                                                                                // SBUS2 Frame available -> Ready for transmit Telemetry                                                                                    // set pin D13 (LED ON) -> SBUS2 Frames OK                                      
      //FrameErrorRate = SBUS2_get_FER();
      
      SBUS2_get_status(&uart_dropped_frame, &transmission_dropt_frame, &failsave);                                     // Check SBUS(2) Status
      if((uart_dropped_frame > 1) ||(transmission_dropt_frame != 0) || (failsave != 0) ){
          send_alarm_as_temp125(ERROR_SLOT, ((failsave*1000) + (transmission_dropt_frame*100) + uart_dropped_frame));
          uart_dropped_frame = 0;
          transmission_dropt_frame = false;
          failsave = false;
        }
    }

    if(SBUS_Ready()){
      send_F1672(VARIO_SLOT , 1234, (float) 0);    
      send_SBS01V(GPS_SLOT, altitude, altitude); 
    }
  #endif
  #ifdef DEBUG
    Serial.print("Altitude: "); Serial.println(altitude);
  #endif
  // The pressure sensor has an update interval of 1s


}