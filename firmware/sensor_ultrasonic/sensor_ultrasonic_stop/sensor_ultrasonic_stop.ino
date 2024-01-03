/**
 * PARTS Common Robot Platform
 * Combined SBUS and Ultrasonic Sensor Program
 * 
 * This program integrates SBUS communication for remote control and ultrasonic sensor for obstacle detection.
 * It controls two motors based on SBUS inputs and stops the motors if an obstacle is detected within a safe distance.
 */

#include <Arduino.h>
#include <sbus.h>       // SBUS library for RC communication
#include <Ultrasonic.h> // Ultrasonic sensor library

// SBUS and Motor related declarations
bfs::SbusRx sbus_rx(&Serial4, true); // Initialize SBUS receiver on Serial4 with verbose output
bfs::SbusData data = {};             // Structure to hold SBUS data
bool newSbusPacket = false;          // Flag to indicate reception of a new SBUS packet
unsigned long lastLogTime = 0;       // Time tracker for data logging
const unsigned long logInterval = 500; // Interval (in ms) for logging SBUS data and motor speeds
int setspeed[2] = {0, 0};            // Array storing speeds for right and left motors
const int MAX_PWM = 200;             // Maximum PWM value for motor speed control
const int motorDir[2] = {1, -1};     // Motor direction multipliers (1 for forward, -1 for reverse)
int motorPWMPin1[2] = {3, 4};        // PWM pins for controlling motor speed
int motorPWMPin2[2] = {5, 6};        // Additional PWM pins for motor control
const int PIN_XD_EN = 2;             // Enable pin for external motor driver (if used)

// Ultrasonic Sensor Setup
const int TRIGGER_PIN = 9;           // Trigger pin for the ultrasonic sensor
const int ECHO_PIN = 15;             // Echo pin for the ultrasonic sensor
Ultrasonic ultrasonic(TRIGGER_PIN, ECHO_PIN); // Ultrasonic sensor object
const unsigned long US_ROUNDTRIP_CM = 29; // Speed of sound constant (microseconds/cm)
const unsigned int MAX_DISTANCE = 200; // Maximum distance (in cm) for sensor to measure
const unsigned int SAFE_DISTANCE = 10; // Safe distance (in cm) for obstacle avoidance

unsigned long lastSensorUpdate = 0;  // Last time the sensor data was updated
const long sensorUpdateInterval = 100; // Interval (in ms) for updating sensor readings

// Function to stop both motors
void stopMotors() {
    for (int i = 0; i < 2; i++) {
        analogWrite(motorPWMPin1[i], 0);
        analogWrite(motorPWMPin2[i], 0);
    }
}

void setup() {
    Serial.begin(9600); // Initialize serial communication at 9600 bps
    pinMode(PIN_XD_EN, OUTPUT); // Set motor enable pin as output
    digitalWrite(PIN_XD_EN, HIGH); // Enable the motors

    // Initialize motor control pins as outputs
    for (int i = 0; i < 2; i++) {
        pinMode(motorPWMPin1[i], OUTPUT);
        pinMode(motorPWMPin2[i], OUTPUT);
        analogWrite(motorPWMPin1[i], 0);
        analogWrite(motorPWMPin2[i], 0);
    }

    // Set ultrasonic sensor timeout for maximum distance
    ultrasonic.setTimeout(MAX_DISTANCE * US_ROUNDTRIP_CM);

    sbus_rx.Begin(); // Initialize SBUS receiver
}

void loop() {
    unsigned long currentMillis = millis();

    // Static variable to retain the distance value between loop iterations
    static unsigned long distance = MAX_DISTANCE + 1;

    // Update ultrasonic sensor readings at defined intervals
    if (currentMillis - lastSensorUpdate >= sensorUpdateInterval) {
        lastSensorUpdate = currentMillis;
        distance = ultrasonic.read(CM);
        // Log distance to serial monitor
        Serial.print("Distance: ");
        Serial.print(distance);
        Serial.println(" cm");
    }

    // Process new SBUS packets
    if (sbus_rx.Read()) {
        data = sbus_rx.data();
        newSbusPacket = true;

        // Activate failsafe if necessary
        if (data.failsafe) {
            stopMotors();
            Serial.println("SBUS entered failsafe mode.");
            return;
        }

        // Map SBUS channel data to motor speed range
        int linear = map(data.ch[1], 172, 1810, -MAX_PWM, MAX_PWM);
        int angular = map(data.ch[0], 172, 1810, -MAX_PWM, MAX_PWM);

        // Implement obstacle avoidance logic
        if (distance < SAFE_DISTANCE && linear > 0) {
            linear = 0; // Stop forward movement if an obstacle is detected
        }

        // Calculate motor speeds
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

    // Log SBUS data and motor speeds at defined intervals
    if (newSbusPacket && millis() - lastLogTime >= logInterval) {
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
