#ifndef Telem_h
#define Telem_h

#include "Config.h"
#include "src/EspSoftwareSerial/src/SoftwareSerial.h"

typedef struct
{
  uint32_t custom_mode;
  uint8_t base_mode;
  boolean armed;
  uint16_t distance_sensor; 
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
  void request_data_streams(uint8_t req_stream_id, uint16_t req_message_rate, uint8_t start_stop);
  void request_distance_sensor();
  
  APdata_t APdata;
  boolean link;
  unsigned long last_heatbeat;


private:
  SoftwareSerial *_MAVSerial;
  void check_link();

  uint8_t system_id;
  uint8_t component_id;
  uint8_t target_sysid;  // Target sysid
  uint8_t target_compid; // Target compid

  unsigned long time_heartbeat;
};
#endif