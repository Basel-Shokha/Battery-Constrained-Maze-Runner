#ifndef PARAMETERS_H
#define PARAMETERS_H

// ── OLED DISPLAY CONFIGURATION ────────────────────────────
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64

// ── HARDWARE KEYPAD CONFIGURATION (4x3 Matrix) ────────────
#define R4   19 
#define R3   11  // Shifted away from 13 to leave room for Basel's LED ring
#define R2   12 
#define R1   4   
#define C1   21 
#define C2   27 
#define C3   33  

// ── INTEGRATED AUDIO & MICROPHONE LAYER ───────────────────
// INMP441 I2S Microphone GPIO Pins
#define I2S_WS   14
#define I2S_SD   15
#define I2S_SCK  32
#define I2S_PORT I2S_NUM_0

// External DAC MAX98357A Audio Amplifier GPIO Pins
#define DAC_BCK_PIN  26
#define DAC_WS_PIN   25
#define DAC_DATA_PIN 22

// ── BASEL'S ROVER CHASSIS HARDWARE & TIMING ───────────────
// LED Ring Settings (Locked directly to your physical wiring)
#define LED_RING_PIN    13  // Data Input line wired directly to physical pin D13
#define NUM_LED_PIXELS  16
#define MAX_LED_BRIGHT  50  // Protects the ESP32 power rail from current spikes

// Gyro Integration & Wheel Kinematics Coefficients
const float   ROBOT_KP             = 0.4f;
const int8_t  ROBOT_BASE_SPEED     = 25;
const float   ROBOT_MAX_CORRECTION = 30.0f;

const int8_t  MAX_TURN_SPEED       = 20;
const int8_t  MIN_TURN_SPEED       = 18;
const float   TURN_TOLERANCE       = 2.0f;
const int     TURN_BRAKE_MS        = 15;

const uint16_t WALL_TRIGGER_MM     = 160; // Automated 90-degree threshold at 16cm clearance

#endif // PARAMETERS_H
