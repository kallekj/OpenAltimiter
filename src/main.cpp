#include <Arduino.h>
#include <Wire.h>
#include "SparkFunMPL3115A2.h"

MPL3115A2 pressure_sensor;

#include <SBUS2.h>
#define ERROR_SLOT        19     // 1 Slot Sensor
#define VARIO_SLOT        21    // 2 Slot Sensor
#define GPS_SLOT          24    // 8 Slot Sensor

int16_t channel = 0;
uint8_t FrameErrorRate = 0;

void setup() {

  // Setup for the MPL3115A2 Pressure Sensor

  Wire.begin();        // Join i2c bus
  
  pressure_sensor.begin(); // Get sensor online

  //Configure the sensor
  pressure_sensor.setModeAltimeter(); // Measure altitude above sea level in meters
  //pressure_sensor.setModeBarometer(); // Measure pressure in Pascals from 20 to 110 kPa

  pressure_sensor.setOversampleRate(7); // Set Oversample to the recommended 128
  pressure_sensor.enableEventFlags(); // Enable all three pressure and temp event flags 


  // Setup for SBUS2 Vario Telemetry sensor
  SBUS2_Setup();
  send_f1672_vario(VARIO_SLOT, (int16_t) 0, (int16_t) 0);


}


void loop() {
  

  float altitude = pressure_sensor.readAltitude();

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

  // The pressure sensor has an update interval of 1s
  delay(1000); 


}