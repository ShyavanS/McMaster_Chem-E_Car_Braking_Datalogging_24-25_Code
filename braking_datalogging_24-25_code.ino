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

#include "DFRobot_EC10.h"
#include <EEPROM.h>
#include <OneWire.h>
#include <DallasTemperature.h>
// #include <Servo.h>

#define ONE_WIRE_BUS 2 // Pin for the DS18B20 data line
#define EC_PIN A1      // Pin for the conductivity probe

// #define POT_PIN A5  // Pin for potentiometer to var stir speed
// #define MOTOR_1 10  // Motor driver PWM pin 1
// #define MOTOR_2 11  // Motor driver PWM pin 2
// #define servo_pwm 3 // Servo motor PWM pin

// Servo servo; // Create servo object

OneWire oneWire(ONE_WIRE_BUS);       // create a OneWire instance to communicate with the sensor
DallasTemperature sensors(&oneWire); // pass oneWire reference to Dallas Temperature sensor

DFRobot_EC10 ec; // Create an instance for the conductivity probe

// variables to store temperature
double temperatureC; // Current temperature
double initTemp;     // Initial temperature for differential calculation

// variables for conductivity probe
double voltage;
double ecValue;

// KALMAN FILTER variables
// temp sensor
double r_temp = 0.0573; // measurment noise variance
double q_temp = 0.01;   // process noise variance -play around later to improve results
double x_k_temp = 0;    // initializing estimated status
double p_k_temp = 0;    // initializing error covariance
double K_temp = 0;      // initializing Kalman gain

// conductivity
double r_cond = 0.05; // measurment noise variance -get value
double q_cond = 0.01; // process noise variance -play around later to improve results
double x_k_cond = 0;  // initializing estimated status
double p_k_cond = 0;  // initializing error covariance
double K_cond = 0;    // initializing Kalman gain

// bool run = false; // Check if the code has run before or only started

// Potentiometer variables to control speed.
// unsigned long pot_pos;
// unsigned int speed;

/*
Description: Subroutine to dump the contents of the dumping mechanism using the servo.
Inputs:      void
Outputs:     void
Parameters:  void
Returns:     void
*/
// void servo_dump() // Dump contents of bowl into braking vessel with servo
// {
//   servo.write(180); // Rotate to 180 deg position without delay
//   delay(1000);      // Wait 1 s
//   servo.write(0);   // Return to default position
// }

/*
Description: Subroutine to implement Kalman filtering on conductivity sensor data.
Inputs:      void
Outputs:     (double)x_K_cond, (double)p_k_cond
Parameters:  (double)input
Returns:     (double)x_k_cond
*/
double kalman_filter_conductivity(double input) // Kalman filtering algorithm
{
  double x_k_cond_min1 = x_k_cond;
  double p_k_cond_min1 = p_k_cond;

  K_cond = p_k_cond_min1 / (p_k_cond_min1 + r_cond); // updating Kalman gain
  // original equation is  K = p_K*H / (H*H*p_k+r) but measurment map scalar is 1)
  x_k_cond = x_k_cond_min1 + K_cond * (input - x_k_cond_min1); // update state estimate
  p_k_cond = (1 - K_cond) * p_k_cond_min1 + q_cond;            // update error covariance

  return x_k_cond; // filtered value
}

/*
Description: Subroutine to implement Kalman filtering on temperature sensor data.
Inputs:      void
Outputs:     (double)x_K_temp, (double)p_k_temp
Parameters:  (double)input
Returns:     (double)x_k_temp
*/
double kalman_filter_temperature(double input) // Kalman filtering algorithm
{
  double x_k_temp_min1 = x_k_temp;
  double p_k_temp_min1 = p_k_temp;

  K_temp = p_k_temp_min1 / (p_k_temp_min1 + r_temp); // updating Kalman gain
  // original equation is  K = p_K*H / (H*H*p_k+r) but measurment map scalar is 1)
  x_k_temp = x_k_temp_min1 + K_temp * (input - x_k_temp_min1); // update state estimate
  p_k_temp = (1 - K_temp) * p_k_temp_min1 + q_temp;            // update error covariance

  return x_k_temp; // filtered value
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

  ec.begin();
  sensors.begin();                       // Initialize the DS18B20 sensor
  sensors.requestTemperatures();         // Request temperature from all devices on the bus
  initTemp = sensors.getTempCByIndex(0); // Get temperature in Celsius

  // Initialize Kalman filter parameters
  x_k_temp = initTemp; // Initial state estimate

  // Set stirring motor PWM and potentiometer pin modes
  // pinMode(MOTOR_1, OUTPUT);
  // pinMode(MOTOR_2, OUTPUT);
  // pinMode(POT_PIN, INPUT);

  // Set pin modes for ENA and ENB pins that are tied to L298N motor driver (see datasheet for use)
  // pinMode(6, OUTPUT);
  // pinMode(9, OUTPUT);

  // Initialize servo to default position
  // servo.attach(servo_pwm);
  // servo.write(0);
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
  x_k_temp = kalman_filter_temperature(temperatureC);

  voltage = analogRead(EC_PIN) / 1024.0 * 5000; // read the voltage
  ecValue = ec.readEC(voltage, x_k_temp);       // convert voltage to EC with temperature compensation

  // Output all necessary data for datalogging in CSV format for Excel Data Streamer
  Serial.print(millis());
  Serial.print(",");
  Serial.print(temperatureC);
  Serial.print(",");
  Serial.print(ecValue);
  Serial.print(",");
  Serial.println(x_k_temp);
  // Serial.print(",");
  // Serial.println(speed);

  ec.calibration(voltage, x_k_temp); // calibration process by Serail CMD
}
