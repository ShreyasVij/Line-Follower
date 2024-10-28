#include <Arduino.h>

// Define an array to store the analog values from pins A0 to A3
int analogValues[4];

// Define the priority weights for each analog sensor
int priorities[4] = {-2, -1, 1, 2}; // A0 -> -2, A1 -> -1, A2 -> 1, A3 -> 2

const int totalPriority = 3;
const float setpoint = 0;

// PID variables
float kp = 1.0;  // Proportional gain
float ki = 0.1;  // Integral gain
float kd = 0.05; // Derivative gain
float previousError = 0;
float integral = 0;                    // Integral term
unsigned long lastTime = 0;            // Time of last PID calculation
const unsigned long pidInterval = 500; // Time interval in microseconds for 2kHz (500us)

// Function to compute PID
float computePID(float setpoint, float input)
{
  // Calculate error
  float error = setpoint - input;

  // Proportional term
  float Pout = kp * error;

  // Integral term (accumulated over time)
  integral += error * (micros() - lastTime) / 1000000.0; // Convert time to seconds
  float Iout = ki * integral;

  // Derivative term
  float derivative = (error - previousError) * 1000000 / (micros() - lastTime);
  float Dout = kd * derivative;

  // Save previous error for next calculation
  previousError = error;

  // PID output
  return Pout + Iout + Dout;
}

// Function to calculate the weighted position based on sensor priorities
float calculatePosition()
{
  float weightedSum = 0;

  // Calculate weighted sum of sensor values
  for (int i = 0; i < 4; i++)
  {
    weightedSum += analogValues[i] * priorities[i];
  }

  // Calculate the position by dividing the weighted sum by the sum of priorities
  float position = weightedSum / totalPriority;
  return position;
}

void controlMotor(int speedPin, int directionPin, int speedValue)
{
  // Determine the direction based on the sign of speedValue
  bool direction = (speedValue >= 0);

  // If the speed is forward, map it directly (0 to 1024 -> 0 to 255)
  int pwmValue;

  if (direction)
  {
    pwmValue = map(speedValue, 0, 1024, 0, 255); // Forward direction: 0 to 255
  }
  else
  {
    pwmValue = map(abs(speedValue), 0, 1024, 255, 0); // Reverse direction: 255 to 0
  }

  digitalWrite(directionPin, direction);

  // Set the motor speed (PWM)
  analogWrite(speedPin, pwmValue);
}

void setup()
{
  // Initialize serial communication at a baud rate of 9600
  Serial.begin(9600);
}

void loop()
{
  // Read analog values from pins A0 to A3 and store them in the array
  for (int i = 0; i < 4; i++)
  {
    analogValues[i] = analogRead(A0 + i); // A0, A1, A2, A3 are consecutive pins
  }

  // Calculate the position using the weighted sum of sensor values
  float position = calculatePosition();

  // Get the current time in microseconds
  unsigned long currentTime = micros();

  // Check if it's time to compute the PID
  if (currentTime - lastTime >= pidInterval)
  {
    // Compute PID output
    float pidOutput = computePID(setpoint, position);

    // Output or use the pidOutput as needed (e.g., control a motor)

    // Update lastTime for the next cycle
    lastTime = currentTime;
  }
}
