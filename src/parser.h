#ifndef SMK_MQTT_PARSER_H
#define SMK_MQTT_PARSER_H

#include <Arduino.h>
#include <smk900.h>
//#define ARDUINOJSON_USE_DOUBLE 1
#include <ArduinoJson.h>
#include <utils.h>


class SmkParser
{
  public:
	static bool rfPayloadToJson(hexPacket_t &packet, String topic, JsonVariant payload, String &type);


	static uint64_t getbits(uint64_t value, uint64_t offset, unsigned n);
	static bool getlogfromdict(JsonObject def_params, JsonVariant ret_result, hexPacket_t &packet, uint16_t idx, String &type);
	static bool getErrorFromDict(JsonObject error_dict, JsonVariant ret_result, hexPacket_t packet, uint16_t idx, String& stype);

};

#endif