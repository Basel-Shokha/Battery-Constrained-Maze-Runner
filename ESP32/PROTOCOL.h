
#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <Arduino.h>


const int CMD_PLAY_AUDIO     = 1;
const int CMD_STOP_AUDIO     = 2;
const int CMD_START_CHARGE   = 3;
const int CMD_BATTERY_UPDATE = 4;
const int CMD_RED_LED_RING   = 5;


const int AUDIO_MISSION_START = 101; 
const int AUDIO_GOING_STATION = 102; 
const int AUDIO_CHARGING_TUNE = 103; 
const int AUDIO_ROUTE_ERROR   = 104; 
const int AUDIO_VICTORY       = 105; 


extern uint16_t distLeft, distFront, distRight;

#endif