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
byte first;
byte second;
int myPin = 10;
int LEDPin = 13;
unsigned long tnow;
unsigned long tbefore;
unsigned long ton;

SoftwareSerial mySerial(myPin, myPin); // RX, TX

void setup() {
  // Open serial communications and wait for port to open:


  // set the data rate for the SoftwareSerial port
  mySerial.begin(19200);

  // initialize BMP180
  if (pressure.begin()) {
    baseline = getPressure();
  }
  else
  {
    while(1);
  }
  pinMode(LEDPin, OUTPUT);
}

void loop() { // run over and over
  double alt,Press;
  Press = getPressure();
  alt = pressure.altitude(Press,baseline);

  if (mySerial.available()) {
    tnow = millis();

    if ((tnow - tbefore) > 100) {
      first = mySerial.read();
      tbefore = tnow;
    } else {
      second = mySerial.read();
      if (second == 0x89) {    // 0x89: module ID of VARIO
        digitalWrite(LEDPin, HIGH);


        // Kopiertere Code ab hier
        delay(5);
        pinMode(myPin, OUTPUT); // switchoff RX-Line
        int ck = 0;
        for (int i = 0; i < 45 - 1; i++) {
          ck += sendBuffer[i];
          mySerial.write(sendBuffer[i]);
          delay(3);
        }
        mySerial.write((byte)ck); // write
        delayMicroseconds(2000); // 2ms
        pinMode(myPin, INPUT);
        // Kopiert bis hier

        digitalWrite(LEDPin, LOW);
      }
    }
  }
}

double getPressure()
{
  char status;
  double T,P;

  // You must first get a temperature measurement to perform a pressure reading.

  // Start a temperature measurement:
  // If request is successful, the number of ms to wait is returned.
  // If request is unsuccessful, 0 is returned.

  status = pressure.startTemperature();
  if (status != 0)
  {
    // Wait for the measurement to complete:

    delay(status);

    // Retrieve the completed temperature measurement:
    // Note that the measurement is stored in the variable T.
    // Use '&T' to provide the address of T to the function.
    // Function returns 1 if successful, 0 if failure.

    status = pressure.getTemperature(T);
    if (status != 0)
    {
      // Start a pressure measurement:
      // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
      // If request is successful, the number of ms to wait is returned.
      // If request is unsuccessful, 0 is returned.

      status = pressure.startPressure(3);
      if (status != 0)
      {
        // Wait for the measurement to complete:
        delay(status);

        // Retrieve the completed pressure measurement:
        // Note that the measurement is stored in the variable P.
        // Use '&P' to provide the address of P.
        // Note also that the function requires the previous temperature measurement (T).
        // (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
        // Function returns 1 if successful, 0 if failure.

        status = pressure.getPressure(P,T);
        if (status != 0)
        {
          return(P);
        }
      }
    }
  }
}

