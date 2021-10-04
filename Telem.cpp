#include "Telem.h"

// Constructor
Telem::Telem(SoftwareSerial &ss)
{

    _MAVSerial = &ss;

    status = false;

    system_id = SYSID;
    component_id = COMPID;

    target_sysid = TARGET_SYSID;
    target_compid = TARGET_COMPID;

    link = false;
    last_heatbeat = 0;
    time_heartbeat = 0;

    APdata.armed = false;

}

/**
 * @brief Comprueba serial y lanzas stream request en caso positivo
 * @return Nada
 */
void Telem::begin()
{
    //Log.notice("Mavlink Serial Status: " CR);

    _MAVSerial->begin(SERIAL_BAUD, SWSERIAL_8N1, SOFT_SERIAL_RX, SOFT_SERIAL_TX, false);
    if (_MAVSerial->available() <= 0)
    {
        Log.notice("-- Serial UP" CR);
        status = true;
        // STREAMS DATA - de momento todas a 5hz
        requestStreams(MAV_DATA_STREAM_ALL, 5, 1);
    }
    else
    {
        Log.error("-- Serial DOWN" CR);
        status = false;
    }

}

void Telem::requestStreams(uint8_t req_stream_id, uint16_t req_message_rate, uint8_t start_stop)
{

    //Log.notice("Requesting Streams..." CR);

    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_request_data_stream_pack(system_id, component_id, &msg, target_sysid, target_compid, req_stream_id, req_message_rate, start_stop);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

    _MAVSerial->write(buf, len);

    //Log.notice("-- Mavlink Ready" CR); // Tendriamos que comprobar si estamos reciviendo
}

void Telem::run(void (*msgRecivedCallback)(uint8_t _msgid))
{

    // LATIMOS
    if (status == true)
    {
        // Latimos cada segundo
        if (millis() >= time_heartbeat + HEARTBEAT_INTERVAL)
        {
            // Enviamos heartbeat
            heartbeat();

            // Comprobamos link
            check_link();

            // Incrementamos
            time_heartbeat += HEARTBEAT_INTERVAL;
        }
    }

    // ESCUCHAMOS
    mavlink_message_t msg;
    mavlink_status_t status;

    while (_MAVSerial->available() > 0)
    {
        uint8_t c = _MAVSerial->read();
        
        // Parseamos posibles msg
        if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status))
        {
      
            //Serial.println("<<-<< SysID: " + String(msg.sysid) + " CompID: " + String(msg.compid)+ " MsgID: " + String(msg.msgid));

            if (msg.sysid == target_sysid) // Si recivimos de la pix
            {

                switch (msg.msgid)
                {
                case MAVLINK_MSG_ID_HEARTBEAT: // #0: Heartbeat

                    mavlink_heartbeat_t heartbeat;
                    mavlink_msg_heartbeat_decode(&msg, &heartbeat);

                    // Capturamos datos para APdata
                    APdata.custom_mode = heartbeat.custom_mode;
                    APdata.base_mode = heartbeat.base_mode;

                    // Detectamos armado/desarmado desde la pix (no gusta, poco preciso)
                    if (APdata.base_mode > 200)
                        APdata.armed = true;
                    else
                        APdata.armed = false;

                    // Time to get
                    last_heatbeat = millis();
                
                break;

                case MAVLINK_MSG_ID_RANGEFINDER: // #173: RANGEFINDER

                    mavlink_rangefinder_t _rangefinder;
                    mavlink_msg_rangefinder_decode(&msg, &_rangefinder);

                    APdata.rangefinder = (_rangefinder.distance * 100);

                    break;

                // case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: // #33: GLOBAL_POSITION_INT
                //     mavlink_global_position_int_t global_position_int;
                //     mavlink_msg_global_position_int_decode(&msg, &global_position_int);

                //     APdata.alt_rel = global_position_int.relative_alt;

                //     break;
                // case MAVLINK_MSG_ID_HOME_POSITION: // #242: GPS_RAW_INT

                //     mavlink_home_position_t home_position;
                //     mavlink_msg_home_position_decode(&msg, &home_position);

                //     APdata.lat_home = home_position.latitude / 10;
                //     APdata.lon_home = home_position.longitude / 10;

                //     break;

                // case MAVLINK_MSG_ID_GPS_RAW_INT: // #24: GPS_RAW_INT

                //     mavlink_gps_raw_int_t gps_raw_int;
                //     mavlink_msg_gps_raw_int_decode(&msg, &gps_raw_int);

                //     APdata.lat = gps_raw_int.lat / 10;
                //     APdata.lon = gps_raw_int.lon / 10;
                //     APdata.alt = gps_raw_int.alt;
                //     APdata.eph = gps_raw_int.eph;
                //     APdata.epv = gps_raw_int.epv;
                //     APdata.vel = gps_raw_int.vel;
                //     APdata.cog = gps_raw_int.cog;
                //     APdata.fix_type = gps_raw_int.fix_type;
                //     APdata.satellites_visible = gps_raw_int.satellites_visible;

                //     break;

                // case MAVLINK_MSG_ID_BATTERY_STATUS: // #147

                //     mavlink_battery_status_t battery_status;
                //     mavlink_msg_battery_status_decode(&msg, &battery_status);

                //     APdata.voltage = battery_status.voltages[0];
                //     APdata.current_battery = battery_status.current_battery;

                //     break;

                // case MAVLINK_MSG_ID_COMMAND_ACK: // #77
                //     mavlink_command_ack_t command_ack;
                //     mavlink_msg_command_ack_decode(&msg, &command_ack);

                //     // Guardamos
                //     APdata.command_ack = command_ack;

                //     switch (command_ack.command)
                //     {
                //     case MAV_CMD_COMPONENT_ARM_DISARM:

                //         //RESULT
                //         switch (command_ack.result)
                //         {
                //         case MAV_RESULT_FAILED:
                //             APdata.armed = false;
                //             Log.error("Arm/Disarm Failed" CR);
                //             break;
                //         }

                //         break;
                //     }

                //     break;

                // case MAVLINK_MSG_ID_STATUSTEXT: // #253: STATUSTEXT

                //     mavlink_statustext_t _statustext;
                //     mavlink_msg_statustext_decode(&msg, &_statustext);

                //     APdata.statustext = String(_statustext.text);

                //     break;

                }
                //  Ejecutamos función callback
                msgRecivedCallback(msg.msgid);
            }
        }
    }
}

// Latido
void Telem::heartbeat()
{

    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_heartbeat_pack(system_id, component_id, &msg, MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, MAV_MODE_PREFLIGHT, 0, MAV_STATE_UNINIT);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

    _MAVSerial->write(buf, len); // Se manda heartbeat

}

// Metodos específicos
void Telem::setMode(int _custom_mode)
{
    APdata.custom_mode = _custom_mode;

    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_set_mode_pack(system_id, component_id, &msg, target_sysid, APdata.base_mode, _custom_mode);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    _MAVSerial->write(buf, len);
}

void Telem::armDisarm()
{
    // Invertimos estado armado
    APdata.armed = !APdata.armed;

    // Enviamos comando
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_command_long_pack(system_id, component_id, &msg, target_sysid, target_compid, MAV_CMD_COMPONENT_ARM_DISARM, 0, APdata.armed, 0, 0, 0, 0, 0, 0);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    _MAVSerial->write(buf, len);

}

void Telem::request_distance_sensor(){
    mavlink_message_t msg;
    mavlink_command_long_t c;
    
    c.command = 511, //MAV_CMD_SET_MESSAGE_INTERVAL;
    c.target_system = target_sysid;
    c.target_component = target_compid;
    c.confirmation = false;
    c.param1 = MAVLINK_MSG_ID_DISTANCE_SENSOR; //132
    c.param2 = 1000000;
    c.param3 = 0;
    c.param4 = 0;
    c.param5 = 0;
    c.param6 = 0;
    c.param7 = 2;
    
    mavlink_msg_command_long_encode(system_id, component_id, &msg, &c);
    
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

    _MAVSerial->write(buf,len);
}

// Comprobamos estado del enlace
void Telem::check_link()
{

    // Calculamos tiempo pasado desde el último heartbeat
    unsigned long now = millis() / 1000;
    unsigned long last_hb = last_heatbeat / 1000;
    int dif = now - last_hb;

    if (dif >= LOST_TIME)
    { // PERDEMOS LINK
        Log.warning("LINK LOST!" CR);
        link = false;
    }
    else if (dif <= 1 && !link)
    { // RECUPERAMOS LINK
        Log.notice("LINK OK!" CR);
        link = true;
    }
}