#include <Wire.h>           // Library for I2C communication
#include <I2Cdev.h>         // Library for MPU6050 communication
#include <MPU6050.h>        // Library for MPU6050 IMU functions

MPU6050 mpu;                // Create MPU6050 object

// Variables to store raw accelerometer and gyroscope data
int16_t ax, ay, az;         // Accelerometer readings
int16_t gx, gy, gz;         // Gyroscope readings

// Motor control and PID constants
#define motorRightDirection 12  // Pin for motor direction control
#define motorRightPWM 3         // Pin for motor PWM control
float Kp = 1;                  // Proportional gain
float Ki = 0.001;              // Integral gain
float Kd = 0.001;              // Derivative gain
unsigned long Ts = 0;          // Sampling time (ms)
unsigned long Tc = 0;          // Current time (ms)
int UMAX = 1;                  // Maximum PID output
int negativeThreshold = -145;  // Threshold for negative tilt
int positiveThreshold = 55;    // Threshold for positive tilt
int desiredAngle = -1750;      // Desired tilt angle in raw sensor units
int measuredAngle;             // Actual tilt angle from sensor
float P_error = 0;             // Proportional error
float I_error = 0;             // Integral error
float D_error = 0;             // Derivative error
float last_error = 0;          // Error from the previous loop
float outPut;                  // PID output
int PWM_output = 0;            // PWM signal for motor speed

void setup() {
    Serial.begin(115200); // Start serial communication for debugging
    Serial.println("Initialize MPU");

    // Initialize the MPU6050
    mpu.initialize();
    Serial.println(mpu.testConnection() ? "Connected" : "Connection failed");

    // Configure motor control pins as outputs
    pinMode(motorRightDirection, OUTPUT);
    pinMode(motorRightPWM, OUTPUT);
}

void loop() {
    // Read sensor data from MPU6050
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    Serial.println("GZ");
    Serial.println(gz); // Print gyroscope z-axis data for debugging

    // Placeholder for raw az mapping (commented out for future tuning)
    // az = map(gz, -17000, 17000, -255, 255);

    ////////////////////////////////////////////////////////////////////
    // PID Control Logic
    ////////////////////////////////////////////////////////////////////

    // Compute sampling time (Ts) in milliseconds
    Ts = micros() / 1000.0 - Tc;

    measuredAngle = az; // Use accelerometer z-axis as the measured angle
    P_error = desiredAngle - measuredAngle; // Calculate proportional error
    Serial.println("P_error");
    Serial.println(P_error);

    // Update integral and derivative errors
    I_error += Ts * P_error; // Accumulate integral error
    D_error = (P_error - last_error) / Ts; // Calculate derivative error

    // Compute PID output
    outPut = Kp * P_error + Ki * I_error + Kd * D_error;
    Serial.println("OUTPUT");
    Serial.println(outPut);

    // Windup prevention for integral term
    if (abs(outPut) > UMAX) {
        if (outPut < 0) {
            outPut = -UMAX; // Clamp to negative maximum
        } else {
            outPut = UMAX;  // Clamp to positive maximum
        }
        I_error -= Ts * P_error; // Prevent integral windup
    }

    // Determine motor direction and compute PWM output
    if (outPut >= 0) {
        Serial.println("I AM IN POSITIVE");
        PWM_output = int(abs(outPut) * 255); // Scale PID output for PWM
        Serial.println("PWM POSITIVE");
        Serial.println(PWM_output);
        digitalWrite(motorRightDirection, LOW); // Set motor direction
    } else {
        Serial.println("I AM IN NEGATIVE");
        PWM_output = int(abs(outPut) * 255); // Scale PID output for PWM
        Serial.println("PWM NEGATIVE");
        Serial.println(PWM_output);
        digitalWrite(motorRightDirection, HIGH); // Set motor direction
    }

    analogWrite(motorRightPWM, PWM_output); // Apply PWM to motor

    // Update last error and current time
    last_error = P_error;
    Tc = micros() / 1000.0;

    delay(10); // Small delay for loop timing
}
