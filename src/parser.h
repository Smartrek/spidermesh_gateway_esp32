#ifndef SMK_PARSER_H
#define SMK_PARSER_H

#include <Arduino.h>
#include <spidermeshapi.h>
//#define ARDUINOJSON_USE_DOUBLE 1
#include <ArduinoJson.h>
#include <utils.h>

#ifndef DEFAULT_UNIT_TYPE
	#define DEFAULT_UNIT_TYPE "int16"
#endif
class SmkParser
{
  public:

	/**
	 * @brief Convert the raw RF payload to a JSON format from the JSON parser file
	 * 
	 * @param packet RF payload
	 * @param tag string that identify the RF payload definition from the JSON parser file
	 * @param payload json payload reference to return the data converted
	 * @param type type of the nodes to identify the right parser conversion
	 * @param includeUnits (optional default false) add level to the json result to include value and unit type
	 * @return true 
	 * @return false 
	 */
	static bool rfPayloadToJson(apiframe &packet, String tag, JsonVariant payload, String &type, bool includeUnits=false);

	/**
	 * @brief Convert the raw value into a readable user value according to parmeter of the variable
	 * 
	 * @param value raw value
	 * @param def_params definition of the variable to know how to convert the raw value
	 * @return double 
	 */
	static double applyParams(int64_t value, JsonObject def_params);

	/**
	 * @brief Get the raw value from a payload form of a number of bits (not number of byte) limitation of 32bits variable
	 * 
	 * @param value variable that may not be aligned in number of byte
	 * @param offset begin in bits where to recovert the data
	 * @param n  length in bits of the variable
	 * @return uint64_t 
	 */
	static uint64_t getbits(uint64_t value, uint64_t offset, unsigned n);

	static bool getlogfromdict(JsonObject def_params, JsonVariant ret_result, apiframe &packet, uint16_t idx, String &type);
	static bool getErrorFromDict(JsonObject error_dict, JsonVariant ret_result, apiframe packet, uint16_t idx, String& stype);

};

#endif