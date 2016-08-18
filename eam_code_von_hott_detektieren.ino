/*
Sources:
https://github.com/betaflight/betaflight/blob/master/src/main/telemetry/hott.h
https://github.com/chriszero/ArduHottSensor

*/
#include <SoftwareSerial.h>
#include <SFE_BMP180.h>
#include <Wire.h>
#include "hott_vario.h"

SFE_BMP180 pressure;
double baseline;
int alt;


byte sendBuffer[44];



byte fromHott;
int HottCom = 10;
int LEDPin = 13;
byte status;

SoftwareSerial HottSerial(HottCom, HottCom); // RX, TX

void setup() {

  // set the data rate for the SoftwareSerial port
  HottSerial.begin(19200);
  Serial.begin(9600);

  // initialize BMP180
  if (pressure.begin()) {  
    char status;
    double T,P;
    status = pressure.startTemperature();
    delay(status);
    status = pressure.getTemperature(T);
    status = pressure.startPressure(3);
    delay(status);
    status = pressure.getPressure(P,T);
    baseline = P;
  }
  pinMode(LEDPin, OUTPUT);
}

void loop() {
  if (HottSerial.available()) {
    fromHott = HottSerial.read();
    if (fromHott == 0x89) {    // 0x89: module ID of VARIO
      char status;
      double T,P;

      hottBuildVario();

      Serial.print("altitude: ");
      Serial.println(HOTT_VARIO_MSG.altitude);
             
      status = pressure.startTemperature(); // Let's start the temp measurement before anything else.
      delay(5); // HoTT wants the request to be answered with a delay of 5ms
                // We use that for the temperature measurement of the BMP180, too
                // since it wants the request to wait 5ms, too.
      status = pressure.getTemperature(T); // We don't handle status and simply assume that
                                           // it works.
      status = pressure.startPressure(3);  // Then we directly start pressure measurement.

      digitalWrite(LEDPin, HIGH); // indicate that we're start sending data
          
      pinMode(HottCom, OUTPUT); // switchoff RX-Line
      int ck = 0;
      for (int i = 0; i < 45 - 1; i++) {
        ck += sendBuffer[i];
        HottSerial.write(sendBuffer[i]);
        delay(3);
      }
      HottSerial.write((byte)ck); // write
      delayMicroseconds(2000); // 2ms
      pinMode(HottCom, INPUT);
      
      digitalWrite(LEDPin, LOW); // indicate that we're done sending data

      // Now approx. 150ms have been passed since we started pressure measurement.
      // Time to get pressure.
      status = pressure.getPressure(P,T);      
      alt = pressure.altitude(P,baseline);

      

    }
  }
}

void hottBuildVario() {
  HOTT_VARIO_MSG.start_byte = 0x7c;
  HOTT_VARIO_MSG.vario_sensor_id = 0x89;
  HOTT_VARIO_MSG.warning_beeps = 0x00;
  HOTT_VARIO_MSG.sensor_id = 0x90;
  HOTT_VARIO_MSG.alarm_invers1 = 0x00;
  HOTT_VARIO_MSG.altitude = alt + 500;
  HOTT_VARIO_MSG.altitude_max = 722;
  HOTT_VARIO_MSG.altitude_min = 497;
  HOTT_VARIO_MSG.climbrate = 30010;
  HOTT_VARIO_MSG.climbrate3s = 30030;
  HOTT_VARIO_MSG.climbrate10s = 30100;
  HOTT_VARIO_MSG.free_char1 = 0x00;
  HOTT_VARIO_MSG.free_char2 = 0x00;
  HOTT_VARIO_MSG.free_char3 = 0x00;
  HOTT_VARIO_MSG.compass_direction = 0x00;
  HOTT_VARIO_MSG.version = 0x00;
  HOTT_VARIO_MSG.stop_byte = 0x7d;
  
  memcpy(&sendBuffer, &HOTT_VARIO_MSG, 44);
  
}


