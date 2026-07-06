// ============================================================
//  TAB 1: PROTOCOL.h — Shared System Constants & Variables
// ============================================================
#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <Arduino.h>

// ── INTEGRAL COMMAND CONSTANTS (M5 INTERFACE SYNC) ────────
const int CMD_PLAY_AUDIO     = 1;
const int CMD_STOP_AUDIO     = 2;
const int CMD_START_CHARGE   = 3;
const int CMD_BATTERY_UPDATE = 4;

// ── INTEGRAL AUDIO TRACK CODES ──────────────────────────────
const int AUDIO_MISSION_START = 101; // "path computed..." (5.0s)
const int AUDIO_GOING_STATION = 102; // "going to station..." (1.5s)
const int AUDIO_CHARGING_TUNE = 103; // "recharging batteries..." (8.5s)
const int AUDIO_ROUTE_ERROR   = 104; // "error: no path found..." (4.5s)
const int AUDIO_VICTORY       = 105; // "destination reached..." (3.5s)

// ── GLOBAL SHARED TELEMETRY VARIABLES ───────────────────────
extern uint16_t distLeft, distFront, distRight;

#endif