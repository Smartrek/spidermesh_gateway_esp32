#ifndef UTILS_H
#define UTILS_H

#include <Arduino.h>
#include <list>
#include <vector>
#include <rom/rtc.h>
#include <logger.h>
#include <ArduinoJson.h>


union value_converter_union
{
	byte uB[4];
	int8_t B[4];
	uint16_t uBB[2];
	int16_t BB[2];
	uint32_t uBBBB;
	int32_t BBBB;
	float F;
};

union u_bytes
{
    uint32_t uint32b;
    int32_t int32b;
    uint8_t uint8b[4];
    uint16_t uint16b[2];
};


typedef std::vector<byte> apiframe;
#include <esp_drive.h>
apiframe convertAsciiTohexCommand(const char *asciiCommand);
bool putPacketInsideBuffer(apiframe packet, uint8_t* buffer);
void printApiPacket(uint8_t* buffer, int size);
void printApiPacket(apiframe hcmd, String prefix = "", String color=KMAG);
void setupWatchdog(hw_timer_t** pWatchdog, long millis, void (*fn)(void));
void kickWatchdog(hw_timer_t* watchdog);
std::vector<String> splitString(String in, char splitChar);
String cleanString(String toClean);
String hexPacketToAscii(apiframe hcmd);
String ApplyFixPoint(word num, int fix);
apiframe asciiTohexPacket(String acmd);
void setNtpClock(bool force=false);
void printLocalTime();
String getTimeFormated();
String getMinTimeFormated();
value_converter_union extractByteFromApiPacket(apiframe p, uint16_t idx, uint16_t len=2);


uint32_t extractU32(apiframe p, int pos);
uint32_t extractU16(apiframe p, int pos);
std::vector<String> ValidationEthernetConfig(String data);

bool macStringToInt(String ipStr, uint32_t *ip);
String macIntToString(uint32_t mac);




#endif