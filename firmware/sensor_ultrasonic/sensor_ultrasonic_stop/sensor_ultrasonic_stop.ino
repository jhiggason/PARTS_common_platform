//***********************************
// PARTS Common Robot Platform
// Combined SBUS and Ultrasonic Sensor Program
//***********************************

#include <Arduino.h>
#include <sbus.h>       // SBUS library for RC communication
#include <NewPing.h>    // Ultrasonic sensor library

// SBUS and Motor related declarations
bfs::SbusRx sbus_rx(&Serial4, true); // Create SBUS receiver object on Serial4
bfs::SbusData data = {};             // Structure to store SBUS data
bool newSbusPacket = false;          // Flag for new SBUS packet reception
unsigned long lastLogTime = 0;       // Time tracking for logging
const unsigned long logInterval = 500; // Interval for logging data
int setspeed[2] = {0, 0};            // Array to store motor speeds
const int MAX_PWM = 200;             // Maximum PWM value for motor speed
const int motorDir[2] = {1, -1};     // Motor direction for right and left motors
int motorPWMPin1[2] = {3, 4};        // PWM pins for motor control
int motorPWMPin2[2] = {5, 6};        // Additional PWM pins for motor control

// Ultrasonic Sensor Constants and Variables
const int TRIGGER_PIN = 9;           // Ultrasonic sensor trigger pin
const int ECHO_PIN = 15;             // Ultrasonic sensor echo pin
const unsigned int MAX_DISTANCE = 200; // Max distance for ultrasonic measurements
const unsigned int SAFE_DISTANCE = 10; // Safe distance threshold
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // Ultrasonic sensor object

unsigned long lastSensorUpdate = 0;  // Last time the sensor was updated
const long sensorUpdateInterval = 100; // Sensor update interval (100ms)

// Function to stop motors
void stopMotors() {
    for (int i = 0; i < 2; i++) {
        analogWrite(motorPWMPin1[i], 0);
        analogWrite(motorPWMPin2[i], 0);
    }
}

// Setup function runs once at startup
void setup() {
    Serial.begin(9600); // Start serial communication
    // Set motor control pins as output
    pinMode(motorPWMPin1[0], OUTPUT);
    pinMode(motorPWMPin1[1], OUTPUT);
    pinMode(motorPWMPin2[0], OUTPUT);
    pinMode(motorPWMPin2[1], OUTPUT);
    // Set ultrasonic sensor pins
    pinMode(TRIGGER_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    sbus_rx.Begin(); // Initialize SBUS receiver
}

// Main loop function
void loop() {
    unsigned long currentMillis = millis();

    // Check if it's time to update the sensor reading
    if (currentMillis - lastSensorUpdate >= sensorUpdateInterval) {
        lastSensorUpdate = currentMillis;
        // Get distance from ultrasonic sensor
        unsigned int distance = sonar.ping_cm();
        // If an obstacle is detected within the safe distance
        if (distance > 0 && distance < SAFE_DISTANCE) {
            Serial.print("Obstacle detected at: ");
            Serial.print(distance);
            Serial.println(" cm - Forward motion stopped");
        }
    }

if (sbus_rx.Read()) {
        data = sbus_rx.data();
        newSbusPacket = true;

        // Check for failsafe condition
        if (data.failsafe) {
            stopMotors();
            Serial.println("SBUS entered failsafe mode.");
            return;
        }

        // Map SBUS values to motor speed
        int linear = map(data.ch[1], 172, 1810, -MAX_PWM, MAX_PWM);
        int angular = map(data.ch[0], 172, 1810, -MAX_PWM, MAX_PWM);

        // If an obstacle is detected within the safe distance, stop only forward movement
        if (sonar.ping_cm() < SAFE_DISTANCE && linear > 0) {
            linear = 0;
        }

        // Calculate motor speeds for differential drive
        setspeed[0] = constrain(linear - angular, -MAX_PWM, MAX_PWM);
        setspeed[1] = constrain(linear + angular, -MAX_PWM, MAX_PWM);

        // Apply calculated speeds to motors
        for (int i = 0; i < 2; i++) {
            if (setspeed[i] * motorDir[i] >= 0) {
                analogWrite(motorPWMPin1[i], setspeed[i] * motorDir[i]);
                analogWrite(motorPWMPin2[i], 0);
            } else {
                analogWrite(motorPWMPin1[i], 0);
                analogWrite(motorPWMPin2[i], -setspeed[i] * motorDir[i]);
            }
        }
    }

    // Log SBUS data and motor speeds at specified intervals
    if (newSbusPacket) {
        if (millis() - lastLogTime >= logInterval) {
            Serial.print("SBUS Channel 1: ");
            Serial.print(data.ch[0]);
            Serial.print(", Channel 2: ");
            Serial.println(data.ch[1]);
            Serial.print("Motor 0 Speed: ");
            Serial.print(setspeed[0]);
            Serial.print(", Motor 1 Speed: ");
            Serial.println(setspeed[1]);

            newSbusPacket = false;
            lastLogTime = millis();
        }
    }
}
