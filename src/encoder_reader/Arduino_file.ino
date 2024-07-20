#include <Servo.h>
#include <PinChangeInterrupt.h>

// Define the pins for the encoders
const int encoder1PinA = 2;
const int encoder1PinB = 4;
const int encoder2PinA = 3;
const int encoder2PinB = 5;

// Define the pin for the servo motor
const int servoPin = 6;

// Variables to store the encoder values
volatile long encoder1Value = 0;
volatile long encoder2Value = 0;

// Variables to store the previous state of the encoder pins
volatile int encoder1LastA = LOW;
volatile int encoder2LastA = LOW;

// Create a Servo object
Servo myServo;

// Interrupt service routines for the encoders
void updateEncoder1() {
  int A = digitalRead(encoder1PinA);
  int B = digitalRead(encoder1PinB);

  // Determine direction based on the current and previous state of pin A
  if (A != encoder1LastA) { // pin A state changed
    if (A == B) {
      encoder1Value--;
    } else {
      encoder1Value++;
    }
    encoder1LastA = A;
  }
}

void updateEncoder2() {
  int A = digitalRead(encoder2PinA);
  int B = digitalRead(encoder2PinB);

  // Determine direction based on the current and previous state of pin A
  if (A != encoder2LastA) { // pin A state changed
    if (A == B) {
      encoder2Value--;
    } else {
      encoder2Value++;
    }
    encoder2LastA = A;
  }
}

void setup() {
  // Initialize the serial port
  Serial.begin(9600);

  // Set up the encoder pins
  pinMode(encoder1PinA, INPUT);
  pinMode(encoder1PinB, INPUT);
  pinMode(encoder2PinA, INPUT);
  pinMode(encoder2PinB, INPUT);

  // Attach interrupts for the encoders
  attachInterrupt(digitalPinToInterrupt(encoder1PinA), updateEncoder1, CHANGE);
  attachPCINT(digitalPinToPCINT(encoder1PinB), updateEncoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder2PinA), updateEncoder2, CHANGE);
  attachPCINT(digitalPinToPCINT(encoder2PinB), updateEncoder2, CHANGE);

  // Attach the servo to the pin
  myServo.attach(servoPin);
  int pulseWidth_i = map(135, 0, 270, 500, 2500);
  myServo.writeMicroseconds(pulseWidth_i);
}

void loop() {
  // Send the encoder values over the serial port
  Serial.println(String(encoder1Value) + "," + String(encoder2Value));

  // Check if data is available on the serial port
  if (Serial.available() > 0) {
    // Read the incoming byte
    String incomingData = Serial.readStringUntil('\n');
    int servoPosition = incomingData.toInt(); // Convert the incoming data to an integer

    // Control the servo motor
    if (servoPosition >= 0 && servoPosition <= 270) {
      int pulseWidth = map(servoPosition, 0, 270, 500, 2500); // Map 0-270 to 500-2500 µs
      myServo.writeMicroseconds(pulseWidth); // Set the servo position using pulse width
    } else {
      Serial.println("Invalid servo position");
    }
  }

  // Delay to simulate a sample rate (e.g., 100ms)
  delay(100);
}
