
#ifndef SPIDERMESH
#define SPIDERMESH

#define DEFAULT_TIMEOUT 600000


#define SKIP_NODE_WHEN_MISSING_PAGE_OLD_VERSION false
#define FORCE_UNICAST_SENDING_MISSING_FLAG true

#define SIM_MISSING_PACKET_SCRATCH false
#define SIM_MISSING_PACKET_SCRATCH_BIT (0x0002)
#define NB_RETRY_SAME_SCRATCH_LOST 3

#define SKIP_BULKUPLOAD false
#define SIM_SKIP_SOME_BROADCAST_UPLOAD false
#define SIM_SKIP_SOME_BROADCAST_UPLOAD_EVERY 25

#define SIMULATION NO_SIMULATION
//#define SIMULATION FULL_SIMULATION
//#define SIMULATION SIMULATION_END

//#ifdef SIMULATION == NO_SIMULATION

#define SIMULATION_BULKUPLOAD false
#define SIMULATION_GETMISSINGFLAGS false
#define SIMULATION_MISSINGFLAGS_ERROR false
#define SIMULATION_PRUNE_VALID_PAGES false
#define SIMULATION_CHECK_IF_CRC_OK false
#define SIMULATION_SEND_META_DATA false
#define SIMULATION_RESET_NODE_ON_SEEK false
#define SIMULATION_SEND_MAGICWORD false

#define SIMULATION_ABORT_UPDATE false
#define SIMULATION_SLOW_DYN_FOR_HOST_OPERATION false
#define SIMULATION_RESET_FACTORY_HOST false
#define SIMULATION_HOST_CHECK_PROGRESS false


#define EXPECTED_PRESET_RF_AT_BOOT PRESET_20B
#include "spidermeshapi.h"



class mesh_step_t
{
	bool first_time_done;

	mesh_step_t() { first_time_done = 0; };
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

class Firmware
{
	char filename[30];
	char type; //
	uint32_t metadata_offset;
	uint32_t current_offset;
};

enum ota_mode_t
{
	WAITING,
	INIT_SMK900,
	CONFIG_SMK900,
	READ_VERSION_SMK900,
	READ_VERSION_HOST,
	READY,
	UPDATE_NODES,
	UPDATE_NODES_END
};

enum ota_state
{
	NONE,							  /* */
	IDLE,							  /* */
	WAIT,
	RESET,
	CHECK_FILE_AND_LOAD_IF_AVAILABLE, /* calc length*/

	/* Depending of the mode, it can go elswhere at the end of this sequence  */
	INIT_GATEWAY_REGISTER,	  /* 1 EOB                   0  */
	INIT_GATEWAY_REGISTER_WAIT_DONE,
	WAIT_CONFIG,
	GET_SPEED_DYN,	  /* 1 EOB                   0  */
	SET_SPEED_DUTY,	  /* 1 EOB                   0  */
	GET_SPEED_RF,	  /* 1 EOB                   0  */
	SET_SPEED_RF,	  /* 1 EOB                   0  */
	TEST_SERIAL_COMM, /* 1 EOB                   0  */

	GET_CHANNEL_RF,
	SET_CHANNEL_RF,

	READ_VERSION_NODES,

	READ_HOST_VERSION_NODES_INIT,
	READ_HOST_VERSION_NODES,

	EVM_OTA_START,

	PRIME_NODE_TO_UPDATE_INIT, /*                         0  */
	PRIME_NODE_TO_UPDATE,	   /*                    n * EOB */
	BULKUPLOAD_INIT,		   /*                            */
	BULKUPLOAD,				   /* block * 4 * EOB            */
	GETMISSINGFLAGS_INIT,	   /*                          */
	GETMISSINGFLAGS,		   /* block EOB                */
	SEND_MISSING_PAGE_INIT,
	SEND_MISSING_PAGE,
	PRUNE_VALID_PAGES_INIT, /*                          */
	PRUNE_VALID_PAGES,		/* block */
	CHECK_IF_CRC_OK_INIT,		/*  */
	CHECK_IF_CRC_OK,			/* EOB */
	SEND_META_DATA_INIT,		/*  */
	SEND_META_DATA,				/* 4 EOB */
	SEND_META_DATA_HOST,		/*  */
	RESET_NODE_ON_SEEK_INIT,	/*  */
	RESET_NODE_ON_SEEK,			/*  */
	SEND_MAGICWORD_INIT,		/*  */
	SEND_MAGICWORD,				/*  */

	/* for SMK900 */
	RESET_GATEWAY_TO_APPLY_UPDATE, /*  */
	ABORT_OTA_ENGINE,

	/* EVM */
	FORCE_EVM_TRIPPLET,

	/* for Host */
	ABORT_UPDATE_INIT,
	ABORT_UPDATE,
	SLOW_DYN_FOR_HOST_OPERATION,
	RESET_FACTORY_HOST_INIT,
	RESET_FACTORY_HOST,
	HOST_CHECK_PROGRESS_INIT,
	HOST_CHECK_PROGRESS,

	RESTORE_SPEED_RF,  /*  */
	RESTORE_SPEED_DYN, /*  */
	STOP			   /*  */

};

enum ota_state_get_missing_flag_t
{
	GETMISSINGFLAG_FIND_NEXT_PAGE

};


class Spidermesh : public SpidermeshApi
{
	static bool doProcessState;
	static ota_mode_t current_mode;
	static ota_state current_state;
	static ota_state next_state;
	static int32_t current_sub_state;
	static uint64_t step_start_time;

	static uint64_t otaTimeout_ms;
	static bool wait_eob;
	static uint64_t chrono_ms;

	static JsonObject log;    



	static int wait_eob_count;


	static portMUX_TYPE mutexWebServer;	

    //SIMULATION OF STATE MACHINE
	#if SIM_SKIP_SOME_BROADCAST_UPLOAD
	static int sim_skip_first_packet;
	#endif
	static String otaResult;
	static bool initDone;
	static MeshParam actualMeshSpeed;
	static MeshParam requiredMeshSpeed;	    


#ifdef WATCHDOG_SMK900_ENABLE
	static void interruptResetPortia();
	static bool interruptResetPortiaFlag;
	static hw_timer_t *watchdogPortia;
	void initWatchdog(long millis);
#endif



public:
	void begin(int hop=-1, int duty=-1, int rf_speed=-1, uint64_t timeout = DEFAULT_TIMEOUT);	
    static void task();
    static bool findNextNode(bool initSearch=false, bool otaActiveOnly=false);
	static void debugStateMachine();
    static void init();
   	static bool isInitDone() { return initDone; };
	static bool isReady();
	static bool currentNodeCanBePolled(){return ((pCurrentNode->second.otaStep == STEP_INIT) || (pCurrentNode->second.otaStep == STEP_COMPLETED) || (pCurrentNode->second.otaStep == STEP_FAILED) || (pCurrentNode->second.otaStep == STEP_RETRY) || (pCurrentNode->second.otaStep == STEP_TRANSFERT));};


	static void setState(ota_state new_state, ota_state following_step = NONE);
	static void logJson(String msg);

	static void launchUpdateOtaEngine();
	static void abortOtaEngine();
	static void resetOtaTimeout()
	{

			chrono_ms = millis();
	};
	static void setOtaTimeout(uint64_t t)
	{
		resetOtaTimeout();
		otaTimeout_ms = t;
	};
	static void resetUpdatesNodes()
	{
		auto x = nodes.pool.begin();
		while (x != nodes.pool.end())
		{
			Serial.printf(" resetOTA: %06X\n", x->first);
			x->second.otaActive = false;
			x++;
		}
	};
	static bool isOtaTimeoutExpired()
	{
		bool ret = true;
		ret = (millis() - chrono_ms > otaTimeout_ms);
		return ret;
	};


    static String translateOffStateToString();
	static String getStateMachineStatus();
    
	static String getSmk900Firmware(bool start = false);


	static ota_state getState() { return current_state; };
	static bool isState(ota_state m) { return current_state == m; };
    static bool isEngineStateActive() { return (current_state != IDLE) ? true : false; };
	
	static bool stepTimeElapsed(uint64_t time_ms){return (millis()-step_start_time) > time_ms;};

    static void stop(){setState(STOP);};


	static void setMode(ota_mode_t new_mode) { current_mode = new_mode; };
	static ota_mode_t getMode() { return current_mode; };
	static bool isMode(ota_mode_t m) { return current_mode == m; };    


	static bool ProcessState(bool eob);
	static bool isDynOptimalUpdateSpeed();

private:
	TaskHandle_t Task1;
	static void smkGatewayTaskCore(void *pvParameters);

	static std::function<void()> cbLoadExternalParamFiles;
public:
	static void setCallbackLoadExternalParamFiles(std::function<void()> fn){cbLoadExternalParamFiles=fn;};


	static bool setChannelSequence(int channel, uint64_t timeout=DEFAULT_TIMEOUT);

	
};


#endif //SPIDERMESH