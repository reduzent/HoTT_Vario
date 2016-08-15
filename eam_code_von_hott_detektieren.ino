/*
  Software serial multple serial test

  Receives from the hardware serial, sends to software serial.
  Receives from software serial, sends to hardware serial.

  The circuit:
   RX is digital pin 10 (connect to TX of other device)
   TX is digital pin 11 (connect to RX of other device)

  Note:
  Not all pins on the Mega and Mega 2560 support change interrupts,
  so only the following can be used for RX:
  10, 11, 12, 13, 50, 51, 52, 53, 62, 63, 64, 65, 66, 67, 68, 69

  Not all pins on the Leonardo support change interrupts,
  so only the following can be used for RX:
  8, 9, 10, 11, 14 (MISO), 15 (SCK), 16 (MOSI).

  created back in the mists of time
  modified 25 May 2012
  by Tom Igoe
  based on Mikal Hart's example

  This example code is in the public domain.

*/
#include <SoftwareSerial.h>

byte moduleID = 142;
byte sendBuffer[] = {0x7c, moduleID,  0x00, 0xe0, 0x00, 0x80, 0x00, 0x00, 
                     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                     0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x29, 0x00,
                     0x50, 0x00, 0x77, 0x75, 0x78, 0x00, 0x00, 0x00,
                     0x00, 0x00, 0x00, 0x7d
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
  pinMode(LEDPin, OUTPUT);

}

void loop() { // run over and over
  if (mySerial.available()) {
    tnow = millis();

    if ((tnow - tbefore) > 100) {
      first = mySerial.read();
      tbefore = tnow;
    } else {
      second = mySerial.read();
      if (second == moduleID) {
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

