#include <PID_v1.h>

// Define motor pins
const int motorPin = 9;    // PWM output to motor
const int encoderPin = 2;  // Input from encoder

// PID control parameters
double setpoint, input, output;
double Kp = 2.0, Ki = 5.0, Kd = 1.0;  // Tune these values

// Smoothing filter variables
double smoothSetpoint = 0;  // Smoothed setpoint for gradual speed change
double alpha = 0.1;  // Smoothing factor, 0 < alpha < 1 (smaller is smoother)

// Create a PID instance
PID myPID(&input, &output, &smoothSetpoint, Kp, Ki, Kd, DIRECT);

// Variables for encoder and speed calculation
volatile int encoderCount = 0;
unsigned long lastTime = 0;
double motorSpeed = 0.0;  // In RPM or other units

void setup() {
  // Set up motor and encoder pins
  pinMode(motorPin, OUTPUT);
  pinMode(encoderPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderPin), countEncoder, RISING);

  // Set initial PID parameters
  setpoint = 100.0;  // Desired speed in RPM
  myPID.SetMode(AUTOMATIC);  // Set PID to automatic mode
  myPID.SetOutputLimits(0, 255);  // Set PWM output limits (0-255 for Arduino)

  Serial.begin(9600);
}

void loop() {
  // Calculate motor speed from encoder
  unsigned long currentTime = millis();
  if (currentTime - lastTime >= 1000) {  // Measure every second (adjust as needed)
    motorSpeed = calculateSpeed();
    input = motorSpeed;  // Set the input for PID

    // Apply exponential smoothing to the setpoint
    smoothSetpoint = alpha * setpoint + (1 - alpha) * smoothSetpoint;

    // Compute PID
    myPID.Compute();

    // Control motor using PWM
    analogWrite(motorPin, output);

    // Print debugging information
    Serial.print("Setpoint: ");
    Serial.print(setpoint);
    Serial.print(" RPM, Smoothed Setpoint: ");
    Serial.print(smoothSetpoint);
    Serial.print(" RPM, Motor Speed: ");
    Serial.print(motorSpeed);
    Serial.print(" RPM, PWM Output: ");
    Serial.println(output);

    lastTime = currentTime;
  }
}

// Interrupt service routine to count encoder pulses
void countEncoder() {
  encoderCount++;
}

// Calculate motor speed based on encoder counts
double calculateSpeed() {
  // Assuming 600 pulses per revolution (adjust based on your encoder)
  double speed = (encoderCount / 600.0) * 60.0;  // Convert to RPM
  encoderCount = 0;  // Reset encoder count for next measurement
  return speed;
}

