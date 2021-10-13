#include "Config.h"
#include "Telem.h"

SoftwareSerial softSerial;

// New mav objt
Telem mav(softSerial);

int cond_flight_modes[FLIGHT_MODES_COUNT] = {
    PLANE_MODE_AUTO, 
    PLANE_MODE_RTL, 
    PLANE_MODE_QLAND, 
    PLANE_MODE_QRTL
};

int cond_landed_states[] = { 
    MAV_LANDED_STATE_LANDING, 
    MAV_LANDED_STATE_IN_AIR 
};

// Range filter 
int ds_values_count = 0;    

// Condition vars
uint8_t cond_mode  = 0;
uint8_t cond_armed = 0; 
uint8_t cond_alt = 0;
uint8_t cond_landed_state = 0;

void setup() {

    // Serial Monitor ang Log
    Serial.begin(SERIAL_BAUD);

    #ifndef DEBUG_MODE
    Log.begin(LOG_LEVEL_SILENT, &Serial);
    #else
    Log.begin(LOG_LEVEL_VERBOSE, &Serial);
    #endif

    // Serial Mavlink Begin
    mav.begin();

}

void loop() {

    // Run mav
    mav.run(msgRecivedCallback);

    // EVALUATE DISARM ACTION
    if (cond_armed == 1 && cond_mode == 1 && cond_alt == 1 && cond_landed_state == 1) {
        //Disarm
        mav.armDisarm();

        cond_armed = 0;
    }

    // Output
    if(mav.link){
      //Log.notice(F("Mode: %i Armed: %T Range Finder : %icms Cond Mode: %i Cond Armed: %i Cond Range Finder: %i Filter Count: %i"CR) , mav.APdata.custom_mode, mav.APdata.armed, mav.APdata.distance_sensor, cond_mode, cond_armed, cond_alt, ds_values_count);
      Log.notice(F("Landed State: %i " CR) , mav.APdata.landed_state);
    }

}

// --------------- Disparador segun msg recivido ---------------------- //
void msgRecivedCallback(uint8_t _msgid)
{
  switch (_msgid) {
    
    case MAVLINK_MSG_ID_HEARTBEAT:

      // CHECK COND Mode
      for (byte i = 0; i < FLIGHT_MODES_COUNT; i++) {
        if(cond_flight_modes[i] == mav.APdata.custom_mode){
          cond_mode = 1;
          break;
        }else{
          cond_mode = 0;
        }
      }

      // CHECK COND Armed
      cond_armed = (mav.APdata.armed) ? 1 : 0;

      break;

    case MAVLINK_MSG_ID_DISTANCE_SENSOR:

      // CHECK COND Alt
      if(mav.APdata.distance_sensor < COND_ALTITUDE){
          if(ds_values_count >= DISTANCE_SENSOR_FILTER_COUNT){
            cond_alt = 1;
            ds_values_count = DISTANCE_SENSOR_FILTER_COUNT;
          }else{
            ds_values_count++;
          } 
      }else{
          cond_alt = 0;
          ds_values_count = 0;
      }

        break;

    case MAVLINK_MSG_ID_EXTENDED_SYS_STATE:

        // CHECK COND landed state
        for (byte i = 0; i < sizeof(cond_landed_states) / sizeof(cond_landed_states[0]); i++) {
            if (cond_landed_states[i] == mav.APdata.landed_state) {
                cond_landed_state = 1;
                break;
            } else {
                cond_landed_state = 0;
            }
        }

        break;

  }
}

