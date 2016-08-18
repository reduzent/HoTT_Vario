/*
Sources:
https://github.com/betaflight/betaflight/blob/master/src/main/telemetry/hott.h
https://github.com/chriszero/ArduHottSensor



*/
#include <SoftwareSerial.h>
#include <SFE_BMP180.h>
#include <Wire.h>

SFE_BMP180 pressure;
double baseline;
int alt;


byte sendBuffer[] = {
  0x7c, 0x89, 0x00, 0x90, 0x00,
  0xf4, 0x01, // altitude
  0xbc, 0x02, // altitude_max
  0xf2, 0x01, // altitude_min
  0x44, 0x75, // climbrate
  0x3a, 0x75, // climbrate_3s
  0x26, 0x75, // climbrate_10s
  0x61, 0x61, 0x61, 0x61, 0x20, 0x61, 0x62, 0x62, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00,  // MSG_TEXT (length=21)
  0x00, 0x00, 0x00, // free chars
  0x00, // compass direction
  0x00, // version
  0x7d  // stop byte
  };


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


      Serial.print("altitude: ");
      Serial.println(alt);
    }
  }
}


