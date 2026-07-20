#ifndef PARAMETERS_H
#define PARAMETERS_H


#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64


#define R4   19 
#define R3   11
#define R2   12 
#define R1   4   
#define C1   21 
#define C2   27 
#define C3   33  


#define I2S_WS   14
#define I2S_SD   15
#define I2S_SCK  32
#define I2S_PORT I2S_NUM_0


#define DAC_BCK_PIN  26
#define DAC_WS_PIN   25
#define DAC_DATA_PIN 22


#define LED_RING_PIN    13
#define NUM_LED_PIXELS  16
#define MAX_LED_BRIGHT  50


const float   ROBOT_KP             = 0.4f;
const int8_t  ROBOT_BASE_SPEED     = 25;
const float   ROBOT_MAX_CORRECTION = 30.0f;

const int8_t  MAX_TURN_SPEED       = 20;
const int8_t  MIN_TURN_SPEED       = 18;
const float   TURN_TOLERANCE       = 2.0f;
const int     TURN_BRAKE_MS        = 15;

const uint16_t WALL_TRIGGER_MM     = 160;


