#ifndef Telem_h
#define Telem_h

#include "Config.h"
#include "src/EspSoftwareSerial/src/SoftwareSerial.h"

typedef struct
{
  uint32_t custom_mode;
  uint8_t base_mode;
  boolean armed;
  uint16_t rangefinder; 
  // int32_t lat;                /*< [degE7] Latitude (WGS84, EGM96 ellipsoid)*/
  // int32_t lon;                /*< [degE7] Longitude (WGS84, EGM96 ellipsoid)*/
  // int32_t lat_home;                /*< [degE7] Latitude (WGS84, EGM96 ellipsoid)*/
  // int32_t lon_home;                /*< [degE7] Longitude (WGS84, EGM96 ellipsoid)*/  
  // int32_t alt;                /*< [mm] Altitude (MSL). Positive for up. Note that virtually all GPS modules provide the MSL altitude in addition to the WGS84 altitude.*/
  // int32_t alt_rel; 
  // uint16_t eph;               /*<  GPS HDOP horizontal dilution of position (unitless). If unknown, set to: UINT16_MAX*/
  // uint16_t epv;               /*<  GPS VDOP vertical dilution of position (unitless). If unknown, set to: UINT16_MAX*/
  // uint16_t vel;               /*< [cm/s] GPS ground speed. If unknown, set to: UINT16_MAX*/
  // uint16_t cog;               /*< [cdeg] Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX*/
  // uint8_t fix_type;           /*<  GPS fix type.*/
  // uint8_t satellites_visible; /*<  Number of satellites visible. If unknown, set to 255*/
  // uint16_t voltage;
  // uint16_t current_battery; 
  // String statustext;
  //mavlink_command_ack_t command_ack;
} APdata_t;

class Telem
{
public:
  Telem(SoftwareSerial &ss);
  void begin();
  void Stream();
  void run(void (*msgRecivedCallback)(uint8_t _msgid));

  void setMode(int _custom_mode);
  void armDisarm();
  void heartbeat();
  void requestStreams(uint8_t req_stream_id, uint16_t req_message_rate, uint8_t start_stop);
  void request_distance_sensor();
  
  APdata_t APdata;
  boolean link;
  boolean status;
  unsigned long last_heatbeat;


private:
  SoftwareSerial *_MAVSerial;
  void check_link();

  uint8_t system_id;
  uint8_t component_id;
  uint8_t type;
  uint8_t autopilot;
  uint8_t target_sysid;  // Target sysid
  uint8_t target_compid; // Target compid

  unsigned long time_heartbeat;
};
#endif