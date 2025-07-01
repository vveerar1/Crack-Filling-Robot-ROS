// Project: Mecannum Robot Control
//
// How to use:
// Send a line like 50 50 -50 -50 or 127,127,127,127 via the Serial Monitor. 
// Each value sets the speed for one motor. Adjust the mapping if your wiring is different.
//
// This code uses a Sabertooth motor controller to control four motors.
// The first two motors are controlled by one Sabertooth (address 128) and the
// second two motors by another Sabertooth (address 129).

#include <SoftwareSerial.h>
#include </home/andres/Arduino/libraries/Sabertooth/Sabertooth.h>

// Define hardware pins
#define sabertoothEstop 4

// Software serial for Sabertooth
SoftwareSerial SWSerial(2, 3); // RX, TX
Sabertooth frontSaber(128, SWSerial); // Address 128
Sabertooth rearSaber(129, SWSerial);  // Address 129

void setup() {
  Serial.begin(9600);
  SWSerial.begin(9600);
  frontSaber.autobaud();
  rearSaber.autobaud();
  pinMode(sabertoothEstop, OUTPUT);
  digitalWrite(sabertoothEstop, HIGH); // Enable motors
  Serial.println("Ready for 4 motor values...");
}

void loop() {
  static String inputString = "";
  if (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n' || inChar == '\r') {
      int m[4] = {0, 0, 0, 0};
      int count = 0;
      char *token;
      char buf[64];
      inputString.toCharArray(buf, sizeof(buf));
      token = strtok(buf, " ,");
      while (token != NULL && count < 4) {
        m[count++] = atoi(token);
        token = strtok(NULL, " ,");
      }
      // Write to motors: [M1, M2, M3, M4]
      // Example: frontSaber.motor(1, m[0]); // etc.
      frontSaber.motor(1, m[0]); // Front Right
      frontSaber.motor(2, m[1]); // Front Left
      rearSaber.motor(1, m[2]);  // Rear Right
      rearSaber.motor(2, m[3]);  // Rear Left
      Serial.print("Motors set to: ");
      Serial.print(m[0]); Serial.print(", ");
      Serial.print(m[1]); Serial.print(", ");
      Serial.print(m[2]); Serial.print(", ");
      Serial.println(m[3]);
      inputString = "";
    } else {
      inputString += inChar;
    }
  }
}