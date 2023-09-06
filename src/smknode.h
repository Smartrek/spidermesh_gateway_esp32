#ifndef SMKNODE_H
#define SMKNODE_H
#include <Arduino.h>
#include <utils.h>
#include <esp_drive.h>
#include "HardwareProfile.h"
#include <firmware.h>

enum
{
	byte0,
	byte1,
	byte2,
	byte3
};
enum
{
	LSB,
	MSB
};

union u_mac
{
	uint32_t address;
	uint8_t bOff[4];
	uint16_t reg_size_addresse[2];
};
struct setting_t
{
	String name;
	uint32_t value;
	String type;
};
typedef std::vector<setting_t> settings_t;
enum
{
	MODE_NORMAL,
	MODE_OTA_CMD,
	MODE_UPLOAD
};

enum step_t
{
	STEP_INIT,
	STEP_WAIT,
	STEP_DONE,
	STEP_RETRY,
	STEP_FAILED,
	STEP_REJECTED,
	STEP_TRANSFERT,
	STEP_COMPLETED
};



class SmkNode
{
public:
	String name;
	u_mac mac;
	int16_t id;
	String group;
	String type;
	bool enabled;
	uint8_t dataValid;
	bool local;
	uint16_t sample_rate;
	uint16_t elapse_time;
	int8_t priority;
	bool otaActive;
	step_t otaStep;
	String labelState;
	uint8_t offset_chunk;
	uint8_t nbFailed;
	FirmwareVersion old_firmware;
	FirmwareVersion new_firmware;
	FirmwareHost old_firmware_pyboard;
	FirmwareHost new_firmware_pyboard;
	String getMacAsString()	{ 	return String(mac.bOff[3]) + "." + String(mac.bOff[2]) + "." + String(mac.bOff[1]) + "." + String(mac.bOff[0]);	};
	int32_t getSetting(String name);

	uint16_t nb_retry_count; // watchdog
	JsonObject pjson_type;
	settings_t settings;
	std::vector <int64_t> config;
};

#endif