#ifndef PORTIA_H
#define PORTIA_H

#include <Arduino.h>
#include <vector>
#include <smklist.h>
#include <HardwareProfile.h>
#include <ArduinoJson.h>
#include <SPIFFS.h>
#include <HardwareSerial.h>
#include <esp_drive.h>


#include <functional>
#include <utils.h>
#include <firmware.h>

#include <mutex>

#include "esp_task_wdt.h"

#define PRT(x) (Serial.print(x))
#define PRTF(x, z) (Serial.printf(x, z))
#define PRTF2(x, y, z) (Serial.printf(x, y, z))
#define PRTLN(x) (Serial.println(x))

#define DEFAULT_MAX_TRY_TO_SEND 2

#if MQTT_PARSER_ENABLED
#include <parser.h>
#endif

#define PORTIA_BAUDRATE 115200

#define DUTY_DEFAULT 5 // 20%
#define PRESET_RF 0	   // 20B: 0x00    72B: 0x10

typedef std::list<apiframe> commandList_t;
typedef std::list<MeshRequest_t> WriteAndExpectList_t;
typedef std::list<apiframe> listPacket_t;

#define ON_DEMAND_COMMAND_MUST_BE_DIFFERENT_FROM_THE_LAST_ONE 0

#define LENGTH_VM_COMMAND 7

#define STATE_SOF 1
#define STATE_LENGTH 2
#define STATE_DATA 3
#define STATE_PACKET 4

#define PACKET_NONE 0
#define PACKET_EOB 0x26
#define PACKET_VM_REQUEST 0x8E
#define PACKET_VM_RESP 0x9E
#define PACKET_OTA_RESP 0x96
#define PACKET_VM_FLASH_RESP 0x9D
#define PACKET_OTA_WRITE_REG 0x94
#define PACKET_REMOTE_ANSWER 0x2D
#define PACKET_TXAIR_CMD_WRAPPER 0x0C
#define PACKET_GET_REGISTER_REPLY 0x13
#define PACKET_SET_REGISTER_REPLY 0x14

#define REGISTER_MAC_ADDRESS 0

#define NB_PACKET_TERMINAL 15


//hop table specs
#define HOPTABLE_50K_START 0
#define HOPTABLE_50K_COUNT 6

//nwk max count
#define NWK_COUNT 8

#define SMK_SOFT_RESET			0x02
#define SMK_READ_REG			0x03
#define SMK_WRITE_REG			0x04
#define SMK_TRANSFERT_MEM		0x0B
#define SMK_VM_EXEC				0x8E
#define SMK_VM_FLASH			0x8D
#define SMK_UPDATE_OTA_CMD		0x06


#define PREFIX_OUT	"  out: "
#define PREFIX_IN 	"  in : "



#define LOCAL	true
#define REMOTE	false
#define BROADCAST_TO_PRIMED_NODE	true


const long timeout_parser = 1000;

typedef struct
{
	uint8_t bo, bi, hop, rd, en_rd, duty;
} dyn_t;


#include <functional>
#include <utility>

#ifndef SHOW_TRANSMITED_PACKET_TO_SMK900
#define SHOW_TRANSMITED_PACKET_TO_SMK900 false
#endif
#ifndef SHOW_RECEIVED_BYTE_FROM_SMK900
#define SHOW_RECEIVED_BYTE_FROM_SMK900 false
#endif

#ifndef SHOW_EXPECT_RESULT
#define SHOW_EXPECT false
#endif

#ifndef SHOW_EXPECT_EVENT
#define SHOW_EXPECT_EVENT false
#endif

#ifndef PORTIA_FIFO_LOW_PRIORITY_SIZE
#define PORTIA_FIFO_LOW_PRIORITY_SIZE 200
#endif
#ifndef PORTIA_FIFO_HIGH_PRIORITY_SIZE
#define PORTIA_FIFO_HIGH_PRIORITY_SIZE 200
#endif

#ifndef SHOW_AUTOMATIC_POLLING
#define SHOW_AUTOMATIC_POLLING false
#endif

#define SHOW_SMK900_INIT true

#define NB_EOB_SMK900
#define NB_EOB_HOST

class ExpectAnswer
{
public:
	mesh_t::iterator _pNode;
	uint8_t _packet_type;
	bool _local;
	uint8_t _eob_cnt;
	int16_t _size;
	ExpectCallback _expect_callback;
	// void (*_expect_callback)(const mesh_t::iterator pNode, bool success);
	apiframe _request;
	apiframe _expectPayload;
	String _tag;

	ExpectAnswer(){};
	ExpectAnswer(mesh_t::iterator pNode, uint8_t packet_type, apiframe request, ExpectCallback expect_callback, String tag = "", uint8_t max_retry = 0, apiframe expectPayload = {}, int16_t size = -1)
	{
		init(pNode, packet_type, request, expect_callback, tag, expectPayload, size);
	};
	void init(mesh_t::iterator pNode, uint8_t packet_type, apiframe request, ExpectCallback expect_callback, String tag = "", apiframe expectPayload = {}, int16_t size = -1)
	{
		_pNode = pNode;
		_packet_type = packet_type;
		_eob_cnt = 0;

		_expect_callback = expect_callback;
		_request = request;
		_size = size;
		_expectPayload = expectPayload;
		_tag = tag;

	};
};

class MeshParam
{
public:
	uint8_t bo, bi, hop, rd, rde, duty; // reg 2
	uint8_t rf_speed;					// reg 11
	MeshParam(){
		bo=-1;bi=-1;hop=-1,rd=-1,rde=-1,duty=-1,rf_speed=-1;
	};
};


//--------------------------------------------------------------------------

class SpidermeshApi
{

private:
	static unsigned char state_serial;

	static apiframe current_packet;
	static HardwareSerial smkport;

	static portMUX_TYPE mutexExpect;


protected:
	static MeshParam actualMeshSpeed;
	static MeshParam requiredMeshSpeed;	

private:
	static void OptimalDelay();
	static uint64_t timeCallbackUser;   

public:
	static ExpectCallback cbAutomaticPolling;
	static ExpectCallback cb_automatic_polling_failed;
	static RequestBuilderCallback cbAutoRequestBuilder;
	
	//-------------------------------------
	// SERIAL PORTIA HARDWARE INIT
	static mesh_t::iterator gateway;
	static mesh_t gateway_boot; //usefull only before knowing the real mac TODO: remove from this class if possible
	
	static bool show_eob;
	static bool show_apipkt_in;
	static bool show_apipkt_out;


	static JsonVariant getTypeJsonVariant();

	static void setWhenPacketReceived(std::function<void(apiframe)> fn) { cbWhenPacketReceived = fn; };
	static void setCallbackAutomaticPolling(ExpectCallback fn){cbAutomaticPolling = fn;};
	static void setCallbackAutoRequestBuilder(std::function<apiframe(mesh_t::iterator)> fn ){cbAutoRequestBuilder=fn;}
	static std::function<void(apiframe)> cbWhenPacketReceived;
	static void setWhenEobIsReceived(std::function<bool(bool)> fn) { WhenEobIsReceived = fn; };
	static std::function<bool(bool)> WhenEobIsReceived;

	static bool watchdog_serial_parser;
	static unsigned long previousMillisPacketParser;

	static unsigned char bufferRx[30];
	static uint16_t length_packet;
	SpidermeshApi();
	static bool init();

	static void reset();

	static void initRegister();

private:


public:
	static bool gatewayMacAddressIsReceived;
	static bool isGatewayMacIsReceived() { return gatewayMacAddressIsReceived; };
	static bool isGatewayMacAvailable() { return gatewayMacAddressIsReceived; };
	static void SaveGatewayMacAddress(apiframe packet);
	static String getMacGateway()
	{
		String ret = "";

		return SmkList::mac2String(gateway->second.mac.address);
	};

	static void setApiMsgFlag(bool msg_in, bool msg_out, bool eob)
	{
		show_eob = eob;
		show_apipkt_in = msg_in;
		show_apipkt_out = msg_out;		
	}

	static apiframe setDyn(uint8_t po, uint8_t pi, uint8_t hop, uint8_t rdx, uint8_t rde, uint8_t duty);
	static apiframe requestMacAddress();
	static apiframe localSetRegister(uint8_t offset_register, uint8_t size_register, uint8_t *content);
	static apiframe localTransfertConfigFromRAMBUFF(uint8_t location);

	static void write(uint32_t c);
	static void write(apiframe p)
	{
		for (uint8_t c : p)
		{
			write(c);
		}
	};
	static void write32(uint32_t c);
	static void dumpReceivedBuffer()
	{
		while (smkport.available())
			smkport.read();
		state_serial = STATE_SOF;
	};

	//------------------------------------
	// Nodes managed by the gateway
	static SmkList nodes;
	static uint8_t idx_polling_cnt;
	static String polling_mode;
	static uint32_t focus_node;
	static unsigned char idx_buf;
	static std::list<String> terminalBuffer;

	static long timeout_expect;
	static unsigned long previousMillisExpectPacketReceived;

	static JsonVariant _type_json_main_file;

	static uint16_t _duration_focus_on_node_polling;
	static unsigned long focus_node_start_time;

protected:
	static int eob_cnt;
	static byte channel_rf;

public:

	static String getPollingMode() { return readFile("/pollingMode"); };
	static bool getAutoPolling() {return _auto_polling_mode;};
	static bool enableAutomaticPolling(String mode="time", String mac = "", uint16_t duration = 15);


	int getNodeCount()
	{
		return nodes.pool.size();
	};
	String getAvailabilityNodes();

	static void task();
	void pollNodeList();
	static void setAutoPolling(bool mode = true)
	{
		_auto_polling_mode = mode;
	};

	static bool isAutoPollingNodeEnabled() { return _auto_polling_mode; };
	static bool setFocusMode(uint32_t node)
	{
		focus_node = node;
		return true;
	};
	static bool setFocusMode(String node)
	{
		bool ret = true;
		if (isValidMac(node))
		{
			setFocusMode(SmkList::macString2Umac(node));
			Serial.print("setFocusMode: ");
			Serial.println(node);
			delay(10);
			Serial.print("--> Focus on: ");
			Serial.print(node);
			Serial.println(" < --");
		}
		else
		{
			Serial.println("--> No focus. Node not valid< --");
			focus_node = 0;
			ret = false;
		}
		return ret;
	};
	static String sendCommand(apiframe cmd);

private:
	static bool _auto_polling_mode;
	friend class Helper;

public:
	static void initWatchdogParser();
	static void disableWatchdogParser();
	static bool sendNextPacketBuffered();
	static bool parseCompletePacket();

	static void resetDataSensorValue(uint8_t idx);
	static bool parseReceivedData();
	static void parseApiFromHost(uint8_t *buf, uint16_t len);

	//------------------------------------------------
	// API COMMMAND SECTION
	static bool addApiPacket(apiframe pkt, bool prior_lvl=false)
	{
		if (prior_lvl)
		{
			addApiPacketHighPriority(pkt);
			return true;
		}
		else
			addApiPacketLowPriority(pkt);
		return false;
	};
	static bool addApiPacket(const char *asciiCommand) { return addApiPacketLowPriority(asciiCommand); };
	static bool addApiPacket(String asciiCommand) { return addApiPacketLowPriority(asciiCommand); };
	static bool addApiPacket(uint8_t *buffer, int size) { return addApiPacketLowPriority(buffer, size); };
	;

	static bool isEobPacket(apiframe pkt)
	{
		if (pkt.size() > 3)
			if (pkt[3] == PACKET_EOB)
				return true;
		return false;
	};


protected:
	static WriteAndExpectList_t writeAndExpectList;
	static commandList_t lowPriorityFifoCommandList;
	static commandList_t highPriorityFifoCommandList;
	static bool _otaPacketInCycle;

	static bool isMessageStackEmpty() { return !(lowPriorityFifoCommandList.size() + highPriorityFifoCommandList.size()); };
	static apiframe checkNextPacketToSend() { return lowPriorityFifoCommandList.front(); };

public:
	static void ClearFifoAndExpectList();
	static bool addApiPacketLowPriority(apiframe hexCommand);
	static bool addApiPacketLowPriority(String asciiCommand);
	static bool addApiPacketLowPriority(const char *asciiCommand);
	static bool addApiPacketLowPriority(uint8_t *buffer, int size);
	static bool addApiPacketHighPriority(apiframe hcmd);

	static bool addWriteExpect(MeshRequest_t r);
	static bool addWriteExpect(mesh_t::iterator p, apiframe h, String t, ExpectCallback cb);

	//------------------------------------------------
	// EXPECT API COMMMAND SECTION
	static std::list<ExpectAnswer> listOfExpectedAnswer;

	static void WriteAndExpectAnwser(
		mesh_t::iterator pNode, apiframe request, uint8_t packet_type,
		uint8_t max_retry = 0, apiframe expectPayload = {}, int16_t size = -1, String tag="", 
		ExpectCallback cb = [](mesh_t::iterator pNode, apiframe packet, bool success, String tag) -> void {});

	static void WriteAndExpectAnwser(
		mesh_t::iterator pNode, apiframe request, uint8_t packet_type, String tag="",
		ExpectCallback cb = [](mesh_t::iterator pNode, apiframe packet, bool success, String tag) -> void {});

	static void WriteAndExpectAnwser(
		mesh_t::iterator pNode, apiframe request, uint8_t packet_type, uint8_t max_retry, String tag="",
		ExpectCallback cb = [](mesh_t::iterator pNode, apiframe packet, bool success, String tag) -> void {});

	static void CheckIfAnswerWasExpectedAndCallSuccessFunction(apiframe rxPkt);
	static void CheckExpectTimeout();

	static mesh_t::iterator pCurrentNode;
	static void automaticNodePolling();

	static void AddToTerminalBuffer(String head, apiframe *cmd);

protected:
	static mesh_t::iterator find(uint32_t add)
	{
		for (auto x = nodes.pool.begin(); x != nodes.pool.end(); x++)
		{
			if (x->first == add)
				return x;
		}
		return nodes.pool.end();
	};
	static mesh_t::iterator find(String x)
	{
		uint32_t add;
		if(macStringToInt(x,&add))
			return find(add);
		return nodes.pool.end();
	};	


protected:
	static firmware_t firmware;
	static bool findNext(bool onlyRemote=true, bool initSearch=false);
	static bool isStepCompleted(bool otaActiveOnly=true);

public:
	static apiframe apiPacket(uint8_t cmd, apiframe pkt, bool local=true, bool broadcastOtaUpdate = false, uint8_t phase = 0);
	static apiframe apiPacket(mesh_t::iterator pNode, uint8_t cmd, apiframe pkt, bool local=true, bool broadcastOtaUpdate = false, uint8_t phase = 0);

	static uint16_t getNumberOfNodeToUpdate()
	{
		uint16_t ret = 0;
		for (auto next : nodes.pool)
			if (next.second.otaActive) ret++;
		return ret;
	};

};

#endif