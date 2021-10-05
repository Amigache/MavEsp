#ifndef Config_h
#define Config_h

#include <Arduino.h>
#include "src/ArduinoLog/ArduinoLog.h"
#include "src/mavlink/ardupilotmega/mavlink.h"

// DEBUG MODE
#define DEBUG_MODE // Comentar para desactivar

// SERIAL BAUDS
#define SERIAL_BAUD 57600
#define SERIAL_BAUD_TELEM 57600

// COND VALUES

#define COND_ALTITUDE 20 //cm
#define DISTANCE_SENSOR_FILTER_COUNT 2 // 
#define FLIGHT_MODES_COUNT 4 // 

// MAVLINK

// Telem serial
#define SOFT_SERIAL_RX 12
#define SOFT_SERIAL_TX 13

// This Device
#define SYSID 200
#define COMPID 1

// Target Device
#define TARGET_SYSID 1
#define TARGET_COMPID 1


//#define HEARTBEATS_STEPS 6000
#define HEARTBEAT_INTERVAL 1000 //ms 1 vez por segundo
#define LOST_TIME 5 //s

#define DISTANCE_SENSOR_INTERVAL 200000



#endif
