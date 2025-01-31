// McMaster Chem-E Car Team
// Datalogging Code - 24/25

/*
Some datalogging code for the Braking sub-team to run on the previous year's car.
Uses the new Kalman filtering algorithm to reduce noise in data implemented on the
newer car alongside several other features, but adapted to run on the Arduino UNO
from last year. It uses the old car as a stationary platform so that they can
complete their testing of the reaction temporarily while the Mechanical sub-team
continues to work on getting this year's car chassis fabricated. The code is taken
piece-meal from the current code, but adapted to work with the UNO and an L298N
motor driver for the stirring mechanism. It also uses a micro-servo for the reactant
dumping mechanism, 8 AA batteries to provide a 12 V power source to the L298N and
for the stirring motor, a potentiometer to vary the stirring speed, and a DS18B20
temperature sensor. All data is output via serial to Excel Data Streamer.
*/

#include <OneWire.h>
#include <DallasTemperature.h>
// #include <Servo.h>

#define ONE_WIRE_BUS 2 // Pin for the DS18B20 data line

// #define POT_PIN A5  // Pin for potentiometer to var stir speed
// #define MOTOR_1 10  // Motor driver PWM pin 1
// #define MOTOR_2 11  // Motor driver PWM pin 2
// #define servo_pwm 3 // Servo motor PWM pin

// Servo servo; // Create servo object

OneWire oneWire(ONE_WIRE_BUS);       // create a OneWire instance to communicate with the sensor
DallasTemperature sensors(&oneWire); // pass oneWire reference to Dallas Temperature sensor

// variables to store temperature
double temperatureC; // Current temperature
double initTemp;     // Initial temperature for differential calculation

// KALMAN FILTER variables
double x_temp; // Filtered temperature
double p_temp; // Initial error covariance

// Process noise and measurement noise
double q_temp; // Process noise covariance
double r_temp; // Measurement noise covariance

// bool run = false; // Check if the code has run before or only started

// Potentiometer variables to control speed.
// unsigned long pot_pos;
// unsigned int speed;

/*
Description: Subroutine to empty the contents of the cup using the servo.
Inputs:      void
Outputs:     void
Parameters:  void
Returns:     void
*/
// void servo_dump()  // Open ball valve with servo to empty cup
// {
//   servo.writeMicroseconds(1500);  // Rotate to 90 deg position without delay
//   delay(6000);                    // Wait 6 s to empty
//   servo.writeMicroseconds(500);   // Return to default position
// }

/*
Description: Subroutine to implement Kalman filtering on temperature sensor data.
Inputs:      void
Outputs:     (double)x_temp, (double)p_temp
Parameters:  (double)x_k, (double)p_k, (double)q, (double)r, (double)input
Returns:     void
*/
void kalman_filter(double x_k, double p_k, double q, double r, double input) // Kalman filtering algorithm
{
  // Kalman filter prediction
  double x_k_minus = x_k;     // Predicted next state estimate
  double p_k_minus = p_k + q; // Predicted error covariance for the next state

  // Kalman filter update

  /* Kalman gain: Calculated based on the predicted error covariance
  and the measurement noise covariance, used to update the
  state estimate (x_k) and error covariance (p_k). */
  double k = p_k_minus / (p_k_minus + r); // Kalman gain

  // Comparison with actual sensor reading
  x_k = x_k_minus + k * (input - x_k_minus); // Updated state estimate
  p_k = (1 - k) * p_k_minus;                 // Updated error covariance

  // Output results and update global variables
  x_temp = x_k;
  p_temp = p_k;
}

/*
Description: Arduino setup subroutine.
Inputs:      void
Outputs:     void
Parameters:  void
Returns:     void
*/
void setup()
{
  Serial.begin(115200); // Start serial communication (adjust baud rate as needed)

  sensors.begin();                       // Initialize the DS18B20 sensor
  sensors.requestTemperatures();         // Request temperature from all devices on the bus
  initTemp = sensors.getTempCByIndex(0); // Get temperature in Celsius

  // Initialize Kalman filter parameters
  x_temp = initTemp; // Initial state estimate
  p_temp = 0.1;      // Initial error covariance
  q_temp = 0.01;     // Process noise covariance
  r_temp = 0.5;      // Measurement noise covariance

  // Set stirring motor PWM and potentiometer pin modes
  // pinMode(MOTOR_1, OUTPUT);
  // pinMode(MOTOR_2, OUTPUT);
  // pinMode(POT_PIN, INPUT);

  // Set pin modes for ENA and ENB pins that are tied to L298N motor driver (see datasheet for use)
  // pinMode(6, OUTPUT);
  // pinMode(9, OUTPUT);

  // Initialize servo to default position
  // servo.attach(servo_pwm);
  // servo.writeMicroseconds(500);
}

/*
Description: Arduino loop subroutine.
Inputs:      void
Outputs:     void
Parameters:  void
Returns:     void
*/
void loop()
{
  // Run ENA and ENB at max
  // analogWrite(6, 255); // ENA pin
  // analogWrite(9, 255); // ENB pin

  // Set motor pin 1 to low
  // digitalWrite(MOTOR_1, LOW);

  // Get potentiometer input as 8-bit value
  // pot_pos = analogRead(POT_PIN);
  // speed = pot_pos / 4;

  // if (!run) // Runs if the code hasn't run yet, not placed in setup due to issues when running it there
  // {
  //   analogWrite(MOTOR_2, 127);   // Set stirring motor to 50% speed on startup to prevent stall
  //   delay(2000);                 // Delay to let the motor spin up
  //   analogWrite(MOTOR_2, speed); // Set motor to spin according to potentiometer
  //   delay(8000);                 // Delay to change speed & for user to get set up in Excel for datalogging
  //   servo_dump();                // Dump reactants into vessel
  //   run = true;                  // Set run true to prevent this block from running again & to signify the program has initialized
  // }
  // else
  // {
  //   analogWrite(MOTOR_2, speed); // Set motor to spin according to potentiometer
  // }

  sensors.requestTemperatures();             // Request temperature from all devices on the bus
  temperatureC = sensors.getTempCByIndex(0); // Get temperature in Celsius

  // Update kalman filter for temperature
  kalman_filter(x_temp, p_temp, q_temp, r_temp, temperatureC);

  // Output all necessary data for datalogging in CSV format for Excel Data Streamer
  Serial.print(millis());
  Serial.print(",");
  Serial.print(temperatureC);
  Serial.print(",");
  Serial.println(x_temp);
  // Serial.print(",");
  // Serial.println(speed);
}
