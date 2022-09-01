
#ifndef SPIDERMESH
#define SPIDERMESH

#include "smk900.h"


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
	READ_VERSION_SMK900,
	READ_VERSION_PYBOARD,
	READY,
	UPDATE_NODES,
	UPDATE_NODES_END
};

enum ota_state
{
	NONE,							  /* */
	IDLE,							  /* */
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

	READ_VERSION_NODES,

	READ_PYBOARD_VERSION_NODES_INIT,
	READ_PYBOARD_VERSION_NODES,

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
	SEND_META_DATA_PYBOARD,		/*  */
	RESET_NODE_ON_SEEK_INIT,	/*  */
	RESET_NODE_ON_SEEK,			/*  */
	SEND_MAGICWORD_INIT,		/*  */
	SEND_MAGICWORD,				/*  */

	/* for SMK900 */
	RESET_GATEWAY_TO_APPLY_UPDATE, /*  */
	ABORT_OTA_ENGINE,

	/* EVM */
	FORCE_EVM_TRIPPLET,

	/* for Pyboard */
	ABORT_UPDATE_INIT,
	ABORT_UPDATE,
	SLOW_DYN_FOR_SIM_BUTTON,
	RESET_FACTORY_PYBOARD_INIT,
	RESET_FACTORY_PYBOARD,
	PYBOARD_CHECK_PROGRESS_INIT,
	PYBOARD_CHECK_PROGRESS,

	RESTORE_SPEED_RF,  /*  */
	RESTORE_SPEED_DYN, /*  */
	STOP			   /*  */

};

enum ota_state_get_missing_flag_t
{
	GETMISSINGFLAG_FIND_NEXT_PAGE

};


class Spidermesh : public Smk900
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


    //SIMULATION OF STATE MACHINE
	static int sim_skip_first_packet;
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
	void begin(int hop=-1, int duty=-1, int rf_speed=-1);	
    static void task();
    static bool findNextNode(bool initSearch=false, bool otaActiveOnly=false);
    static void init();
   	static bool isInitDone() { return initDone; };
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
};


#endif //SPIDERMESH