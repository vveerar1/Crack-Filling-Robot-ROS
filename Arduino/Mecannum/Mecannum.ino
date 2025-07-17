// Project: Mecannum Robot Control
//
// How to use:
// Send a line like 50 50 -50 -50 or 127,127,127,127 via the Serial Monitor. 
// Each value sets the speed for one motor. Adjust the mapping if your wiring is different.
//
// This code uses a Sabertooth motor controller to control four motors.
// The first two motors are controlled by one Sabertooth (address 128) and the
// second two motors by another Sabertooth (address 129).

// Libraries for the motor controllers
#include <SoftwareSerial.h>
#include <Sabertooth.h>

// *********************
// Define hardware pins
// *********************
#define sabertoothEstop 4 // This is connected to S2 on the motor controllers. 
                          // When this pin is pulled low the motors will stop. 


// Declaration of the software serial UART and motor controller objects
SoftwareSerial SWSerial(NOT_A_PIN, 3); // RX on pin 2 (unused), TX on pin 3 (to S1).
Sabertooth frontSaber(128, SWSerial); // Address 128, front motors, 1 = right, 2 = left
Sabertooth rearSaber(129, SWSerial); // Address 130, rear motors, 1 = right, 2 = left


//******************************************************************************
// Sets up our serial com, hardware pins, SPI bus for ethernet shield and motor controller.
// RETURNS: Nothing
//******************************************************************************
void setup() 
{
  delay(2000);           // Short delay to allow the motor controllers
                         // to power up before we attempt to send it commands.
                         // If you try to talk to them before the finish booting
                         // they can lock up and be unresponsive
  
  Serial.begin(9600);    // Serial for the debug output
  SWSerial.begin(9600);  // Serial for the motor controllers
  
  frontSaber.autobaud(); // Allows the motor controllers to detect the baud rate
  rearSaber.autobaud();  // we're using and adjust appropriately
  
  // Initialize GPIO inputs and outputs
  pinMode(sabertoothEstop, OUTPUT);

  stopAllMotors();		// Make sure all motors are stopped for safety
}


//******************************************************************************
// Sets the speed of all motor controllers to zero and sets all ESTOPs
// RETURNS: NONE
//******************************************************************************
void stopAllMotors()
{
  digitalWrite(sabertoothEstop, LOW);
  
  frontSaber.motor(1, 0);
  frontSaber.motor(2, 0);
  rearSaber.motor(1, 0);
  rearSaber.motor(2, 0);
}

//******************************************************************************
// This loop reads commands sent from the Jetson Nano via serial usb connection
//
//******************************************************************************
void loop() {
  static String inputString = "";
  static unsigned long lastCmdTime = 0;
  const unsigned long timeout = 200; // ms to wait before stopping motors

  // --- Handle incoming motor commands ---
  while (Serial.available()) {
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
      
      // Map to Sabertooth range & Cleans up our output data
      for (int i = 0; i < 4; i++) {
        m[i] = map(m[i], -255, 255, -127, 127); // Map to Sabertooth range
        m[i] = boundAndDeadband(m[i]); // Apply deadband and bounds
      }

      // Raises the ESTOP lines before commanding the motors
      digitalWrite(sabertoothEstop, HIGH); // Enable motors
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
      lastCmdTime = millis(); // Update last command time
      inputString = "";
    } else {
      inputString += inChar;
    }
  }

  // Stop motors if no command received recently
  if (millis() - lastCmdTime > timeout) {
    stopAllMotors();
  }
}

//******************************************************************************
// Cleans up our values for the motor controllers 
// The motor controllers only accept a value range of -127 to 127. We also apply
// a deadband so the robot doesn't drift when idle
// 
// RETURNS: Cleaned up value
//******************************************************************************
int boundAndDeadband (int inputValue) 
{
  if (inputValue < -127)  { inputValue = -127; }
  if (inputValue > 127)   { inputValue = 127; }
  if ((inputValue < 5) && (inputValue > -5)) { inputValue = 0; }

  return inputValue; 
}
