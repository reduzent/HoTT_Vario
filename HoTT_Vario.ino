/*
  Software for Vario / Altimeter compatible with the Graupner HoTT system

  Tested with:
    * BMP180
    * Arduino Pro Mini (5V)
    * GR-12 receiver

  The software is configured for the following wiring:
  
  BMP180 / SDA       -- Arduino Pro Mini / A4
  BMP180 / SCL       -- Arduino Pro Mini / A5
  GR-12  / Channel 5 -- Arduino Pro Mini / 2

  Released to the public domain
  written by Roman Haefeli, 2016
  borrowed a lot of code from similar projects

  Sources:
  https://github.com/betaflight/betaflight/blob/master/src/main/telemetry/hott.h
  https://github.com/chriszero/ArduHottSensor

*/
#include <SoftwareSerial.h>
#include <SFE_BMP180.h>
#include <Wire.h>
#include "hott_vario.h"
#include "RunningAverage.h"



SFE_BMP180 pressure;                       // pressure sensor
float baseline;                            // pressure of current altitude
float alt = 0;                             // altitude (result from sensor)
float alt_prev = 0;                        // previous sample
byte sendBuffer[sizeof(HOTT_VARIO_MSG)];  
byte fromHott;                             // serial read from HoTT GR-12
int HottCom = 2;                           // Pin connected to telemetry pin of HoTT
int LEDPin = 13;
byte status;                              
SoftwareSerial HottSerial(HottCom, HottCom); // RX, TX

RunningAverage cr1s(5);
RunningAverage cr3s(15);
RunningAverage cr10s(50);



void setup() {
  // set the data rate for the SoftwareSerial port
  HottSerial.begin(19200);

  // initialize BMP180 sensor
  // We measure the pressure at boot time so that we have reference
  // for calculating the altitude
  if (pressure.begin()) {
    char status;
    double T, P;
    status = pressure.startTemperature();
    delay(status);
    status = pressure.getTemperature(T);
    status = pressure.startPressure(3);
    delay(status);
    status = pressure.getPressure(P, T);
    baseline = P;
  }
  pinMode(LEDPin, OUTPUT);

  cr1s.clear();
  cr3s.clear();
  cr10s.clear();
}

void loop() {
  if (HottSerial.available()) {
    fromHott = HottSerial.read();
    if (fromHott == 0x89) {    // 0x89: module ID of VARIO
      char status;
      double T, P;

      // generate data packet to be sent
      hottBuildVario();

      // Let's start the temp measurement before anything else. We need it later 
      // to calibrate the pressure measuremnet
      status = pressure.startTemperature(); 
      // HoTT wants the request to be answered with a delay of 5ms
      // We use that for the temperature measurement of the BMP180, 
      // since it wants the request to wait 5ms, too.
      delay(5);
      // We don't handle status and simply assume that it works.
      status = pressure.getTemperature(T);
      // Then we directly start pressure measurement.
      // 0: least precise, 3: most precise (takes the longest time); 
      status = pressure.startPressure(3);
      // indicate that we start sending data
      digitalWrite(LEDPin, HIGH);
      // switchoff RX-Line
      pinMode(HottCom, OUTPUT);
      byte ck = 0;
      for (int i = 0; i < sizeof(sendBuffer); i++) {
        ck += sendBuffer[i];
        HottSerial.write(sendBuffer[i]);
        delay(2);
      }
      HottSerial.write((byte)ck); // write

      delayMicroseconds(2000); // 2ms

      // Now let's switch the HoTT pin back to INPUT 
      // AND DON'T FORGET TO ACTIVATE THE PULL-UP RESISTOR!!
      pinMode(HottCom, INPUT);
      digitalWrite(HottCom, HIGH);

      // indicate that we're done sending data
      digitalWrite(LEDPin, LOW);

      // Now approx. 100ms have passed since we started pressure measurement.
      // Time to take a sample.
      status = pressure.getPressure(P, T);
      // altitude is calculated from current pressure and baseline pressure
      alt = pressure.altitude(P, baseline);
      // assuming we're taking 5 samples/s  (HoTT request a packet every 200ms)
      float rate = (alt - alt_prev) * 5.0;
      alt_prev = alt;
      cr1s.addValue(rate);
      cr3s.addValue(rate);
      cr10s.addValue(rate);
    }
  }
}

void hottBuildVario() {
  HOTT_VARIO_MSG.start_byte = 0x7c;
  HOTT_VARIO_MSG.vario_sensor_id = 0x89;
  HOTT_VARIO_MSG.warning_beeps = 0x00;
  HOTT_VARIO_MSG.sensor_id = 0x90;
  HOTT_VARIO_MSG.alarm_invers1 = 0x00;
  HOTT_VARIO_MSG.altitude =  alt + 500;
  HOTT_VARIO_MSG.altitude_max = max(HOTT_VARIO_MSG.altitude_max, alt + 500);
  HOTT_VARIO_MSG.altitude_min = min(HOTT_VARIO_MSG.altitude_min, alt + 500);
  HOTT_VARIO_MSG.climbrate = 30000 + cr1s.getAverage() * 100;
  HOTT_VARIO_MSG.climbrate3s =30000 + cr3s.getAverage() * 100;
  HOTT_VARIO_MSG.climbrate10s = 30000 + cr10s.getAverage() * 100;
  HOTT_VARIO_MSG.compass_direction = 0x67;
  HOTT_VARIO_MSG.version = 0x01;
  HOTT_VARIO_MSG.stop_byte = 0x7d;

  // Now let's copy the data packet to our send buffer
  memcpy(&sendBuffer, &HOTT_VARIO_MSG, sizeof(HOTT_VARIO_MSG));
}


