#include "Spidermesh.h"
#include <spidermeshapi.h>



bool Spidermesh::doProcessState;
ota_mode_t Spidermesh::current_mode;
ota_state Spidermesh::current_state;
ota_state Spidermesh::next_state;
int32_t Spidermesh::current_sub_state;
uint64_t Spidermesh::step_start_time;

uint64_t Spidermesh::chrono_ms;

uint64_t Spidermesh::otaTimeout_ms;
bool Spidermesh::wait_eob;
int Spidermesh::wait_eob_count;


JsonObject Spidermesh::log;
String Spidermesh::otaResult;
bool Spidermesh::initDone=false;

MeshParam Spidermesh::actualMeshSpeed;
MeshParam Spidermesh::requiredMeshSpeed;

hw_timer_t* Spidermesh::watchdogPortia;
bool Spidermesh::interruptResetPortiaFlag=false;

#if SIM_SKIP_SOME_BROADCAST_UPLOAD
int Spidermesh::sim_skip_first_packet;
#endif

std::function<void()> Spidermesh::cbLoadExternalParamFiles;

portMUX_TYPE Spidermesh::mutexWebServer;


void Spidermesh::begin(int hop, int duty, int rf_speed, uint64_t timeout)
{

    if(hop>=5 && hop<31) requiredMeshSpeed.hop = hop;
    if(duty==5 || duty==10 || duty==20 || duty==40 || duty==80 || duty==160) requiredMeshSpeed.duty = duty;
    if(rf_speed==20) requiredMeshSpeed.rf_speed = PRESET_20B;
    if(rf_speed==72) requiredMeshSpeed.rf_speed = PRESET_72B;


    mutexWebServer = portMUX_INITIALIZER_UNLOCKED;

	xTaskCreatePinnedToCore(
	//xTaskCreate(
		Spidermesh::smkGatewayTaskCore,
		"smkGatewayTaskCore",
		16384,	/* stack */
		NULL,	/*  */
		1,		/* priority */
		&Task1, /*  */
		1);		/* core # (0-1) arduino loop fn is on core 1 */ 

		uint64_t timeout_end = millis()+timeout;
		bool timeout_flag = false;

		while(!isReady() && !timeout_flag) 
		{
			if(millis() > timeout_end)
			{
				timeout_flag = true;
				Serial.println("--> TIMEOUT <--");
			}
			delay(100);
		}
		Serial.println("  smk900 begin done");
}


void Spidermesh::smkGatewayTaskCore(void *pvParameters)
{
	//setlogBuffer(&logBuffer);
	delay(1000);
	if (!SpidermeshApi::init())
		Serial.println("Error in typejson_document");

	SpidermeshApi::setWhenEobIsReceived(ProcessState);

    setMode(INIT_SMK900);
    setState(INIT_GATEWAY_REGISTER);
    setOtaTimeout(60000);

	String mode = getPollingMode();
	if (mode == "time" || mode == "fast")
		setAutoPolling(true);

	while (true)
	{
		task();
	}
}

void Spidermesh::task()
{
	SpidermeshApi::task();
    ProcessState(false);

  #ifdef WATCHDOG_SMK900_ENABLE
    if(interruptResetPortiaFlag)
    {
        interruptResetPortiaFlag=false;
        reset();
        setMode(INIT_SMK900);
        setState(INIT_GATEWAY_REGISTER);
        setOtaTimeout(10000);        
    }
  #endif //WATCHDOG_SMK900_ENABLE	
}



bool Spidermesh::findNextNode(bool initSearch, bool otaActiveOnly)
{
	bool ret = false;
	auto next = pCurrentNode;

	if(initSearch) next=nodes.pool.begin();


	int nbCheck = 0;
	while (nbCheck < nodes.pool.size())
	{
		if (next == nodes.pool.end())
			next = nodes.pool.begin();

		if ((next->second.otaActive || !otaActiveOnly) && (next->second.otaStep != STEP_DONE) && (next->second.otaStep != STEP_REJECTED) )
		{
			pCurrentNode = next;
			Serial.print("Node Found:");
			Serial.println(macIntToString(pCurrentNode->first));
			ret = true;
			break;
		}
		else {Serial.println("++");}
		nbCheck++;
		next++;
	}
	return ret;
}

void Spidermesh::init()
{
	doProcessState = false;
	setState(IDLE);
}

void Spidermesh::setState(ota_state new_state, ota_state following_step)
{
	current_state = new_state;
	next_state = following_step;
	eob_cnt = 0;
	step_start_time = millis();
};

bool Spidermesh::isReady()
{
	return (isMode(READY) && isState(IDLE));
}

#ifdef WATCHDOG_SMK900_ENABLE
void Spidermesh::interruptResetPortia()
{
    Serial.println("Portia not responding, reset and reinit register...");

    interruptResetPortiaFlag=true;
}

void Spidermesh::initWatchdog(long millis)
{
    //setupWatchdog(&watchdogPortia, millis*1000, &interruptResetPortia);
    watchdogPortia=timerBegin(1,80,true);
    timerAlarmWrite(watchdogPortia, 20000000, false); // set time in uS must be fed within this time or reboot
    timerAttachInterrupt(watchdogPortia, interruptResetPortia, true);
    timerAlarmEnable(watchdogPortia);  // enable interrupt
}
#endif //WATCHDOG_SMK900_ENABLE


void Spidermesh::launchUpdateOtaEngine()
{
	if (1)
	{

		setOtaTimeout(10000);
		if (firmware.filename != "")
		{
			Serial.println("firmware: ");
			Serial.println(firmware.filename);
			setState(CHECK_FILE_AND_LOAD_IF_AVAILABLE);
		}
		else
			return;

		

		auto i = nodes.pool.begin();
		while (i != nodes.pool.end())
		{
			i->second.labelState.clear();
			i->second.old_firmware.clear();
			i->second.new_firmware.clear();
			// i->second.otaActive = true;
			i++;
		}

		setMode(UPDATE_NODES);
		nodes.resetStep(true);
		nodes.setLabels("START");
		doProcessState = true;

		PRTLN("==================================================");
		PRTLN("          UPDATE START");
		PRTLN("==================================================");


		logJson("OTA START");

#if SIM_SKIP_SOME_BROADCAST_UPLOAD
		sim_skip_first_packet = true;
#endif
#if SIM_MISSING_PACKET_SCRATCH
		firmware.test_skip_scratch = NB_RETRY_SAME_SCRATCH_LOST;
#endif
	}
}

void Spidermesh::abortOtaEngine()
{
	PRT("ABORT UPDATE ENGINE");
	// apiframe msg = apiPacket({0x0C, 0x00, 0xC6, 0x02});
	apiframe msg = apiPacket(0x06, {0x02}, false, true);
	write(msg);

	/*
	//must validata that do not brick module (harware reset needed)
	if(firmware.getType() == HOST)
	{
		msg = apiPacket({0x0C, 0x00, 0x0E, 0xFF, 0xFF, 0xFF, 0x12});

	}
	*/

	setState(ABORT_UPDATE);
}

String Spidermesh::translateOffStateToString()
{
	String string_state = "IDLE";
	byte st = getState();
	if (st >= CHECK_FILE_AND_LOAD_IF_AVAILABLE && st <= TEST_SERIAL_COMM)
		string_state = "INIT";
	else if (st >= READ_VERSION_NODES && st <= READ_HOST_VERSION_NODES)
		string_state = "READ VERSION";
	else if (st >= PRIME_NODE_TO_UPDATE_INIT && st <= PRIME_NODE_TO_UPDATE)
		string_state = "PRIME NODE";
	else if (st >= BULKUPLOAD_INIT && st <= BULKUPLOAD)
		string_state = "BULK UPLOAD";
	else if (st >= GETMISSINGFLAGS_INIT && st <= GETMISSINGFLAGS)
		string_state = "GETMISSING FLAGS";
	else if (st >= PRUNE_VALID_PAGES_INIT && st <= PRUNE_VALID_PAGES)
		string_state = "BROADCAST PERFORM CRC";
	else if (st >= CHECK_IF_CRC_OK_INIT && st <= CHECK_IF_CRC_OK)
		string_state = "CHECK IF CRC OK";
	else if (st >= SEND_META_DATA_INIT && st <= SEND_META_DATA)
		string_state = "SEND META DATA";
	else if (st >= STOP)
		string_state = "STOP";
	return string_state;
}

String Spidermesh::getStateMachineStatus()
{
	DynamicJsonDocument jsonBuffer(10000);
	String ret = "";
	jsonBuffer.clear();

	portENTER_CRITICAL(&mutexWebServer);
	jsonBuffer["stateProcess"] = translateOffStateToString();
	jsonBuffer["result"] = otaResult;

	jsonBuffer["stateProgress"] = (firmware.current_progress <firmware.max_progress) ? firmware.current_progress * 100 / firmware.max_progress : 100;

	JsonObject nodeList = jsonBuffer.createNestedObject("nodeList");
	for (auto n : nodes.pool)
	{
		if (n.second.otaActive)
		{

			String m = SmkList::mac2String(n.first);
			nodeList[m].createNestedObject("state");
			nodeList[m]["state"] = n.second.otaStep;
			nodeList[m]["detail"] = n.second.labelState;
			nodeList[m]["old_version"] = n.second.old_firmware.getVersionString() + " - " + n.second.old_firmware_pyboard.getVersionString();
			nodeList[m]["new_version"] = n.second.new_firmware.getVersionString() + " - " + n.second.new_firmware_pyboard.getVersionString();
		}
	}
	portEXIT_CRITICAL(&mutexWebServer);

	serializeJson(jsonBuffer, ret);
	return ret;
}

String Spidermesh::getSmk900Firmware(bool start)
{
	String ret = "";

	if (start)
	{
		PRTLN("LAUNCH GET SMK900");
		auto i = nodes.pool.begin();
		while (i != nodes.pool.end())
		{
			i->second.labelState.clear();
			i->second.old_firmware.clear();
			i->second.new_firmware.clear();
			i->second.otaActive = true;
			i++;
		}
		pCurrentNode = nodes.pool.begin();

		setMode(READ_VERSION_SMK900);
		setState(READ_VERSION_NODES);
		nodes.resetStep(true);
	}
	DynamicJsonDocument jsonBuffer(10000);

	jsonBuffer.clear();
	JsonObject nodeList = jsonBuffer.createNestedObject("nodeList");


	jsonBuffer["stateProcess"] = translateOffStateToString();
	for (auto n : nodes.pool)
	{
		if (n.second.otaActive)
		{
			String m = SmkList::mac2String(n.first);
			nodeList[m].createNestedObject("state");
			nodeList[m]["detail"] = n.second.labelState;
			nodeList[m]["old_version"] = n.second.old_firmware.getVersionString();
		}
	}

	serializeJson(jsonBuffer, ret);
	return ret;
}

bool Spidermesh::isDynOptimalUpdateSpeed()
{
	bool ret = true;

	uint8_t h = readFile("/hops").toInt();
	if (h == 0)
		h = 15;
	uint8_t d = readFile("/duty").toInt();
	if (d == 0)
		d = 5;


	//if specified by user with begin function
	if(requiredMeshSpeed.hop != -1 && requiredMeshSpeed.duty != -1)
	{
		actualMeshSpeed.hop = requiredMeshSpeed.hop;
		h = requiredMeshSpeed.hop;
		actualMeshSpeed.duty =requiredMeshSpeed.duty;
		d=requiredMeshSpeed.duty;
		ret = false;
	}			

	if (actualMeshSpeed.bo != 1) 	ret = false;
	if (actualMeshSpeed.bi != 1) 	ret = false;
	if (actualMeshSpeed.hop != h) 	ret = false;
	if (actualMeshSpeed.rd != 1) 	ret = false;
	if (actualMeshSpeed.rde != 0) 	ret = false;
	if (actualMeshSpeed.duty != d) 	ret = false;

	return ret;
};    

void Spidermesh::logJson(String msg)
{
	#if LOG_JSON_ENABLED
	log[getMinTimeFormated()] = msg;
	#endif
}

/**
 * @brief 	Set the RF channel and save it to eeprom. This procedure will reset the gateway
 * 			module and launch again the init register sequence
 * 
 * @param channel 
 * @return true 
 * @return false 
 */
bool Spidermesh::setChannelSequence(int channel, uint64_t timeout)
{
	bool ch_ok = false;

	//validation of the channel
	std::vector<byte> ch_available;
	for(byte i=1; i<16; i++) ch_available.push_back(i);
	for(auto n: ch_available){if(channel == n) ch_ok = true;} 
	if(ch_ok ) {
		channel_rf = channel;		
		setMode(CONFIG_SMK900);
		setState(SET_CHANNEL_RF);

		uint64_t timeout_end = millis()+timeout;
		bool timeout_flag = false;


		while(!isReady() && !timeout_flag) 
		{
			if(millis() > timeout_end)
			{
				timeout_flag = true;
				Serial.println("--> TIMEOUT <--");
			}
			delay(100);
		}
		Serial.println("setChannelSequence out");
		delay(1000);
	}
	else PRTLN("** error: channel invalide");

	return ch_ok;	
}

void Spidermesh::debugStateMachine()
{
	Serial.println("\n\n---DEBUG STATE MACHINE---\n  LIST OF EXPECT ANSWER:");
	for(auto i: listOfExpectedAnswer)
	{
		printApiPacket(i._request, "  exp:");
	}
	Serial.print("  total = ");
	Serial.println(listOfExpectedAnswer.size());

	Serial.println("\n");
}


//-----------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------
bool Spidermesh::ProcessState(bool eob)
{
	bool ret = false;
#if 0
	if(eob)
	{
		PRT("MODE: ");
		PRT(getMode());
		PRT(" STATE: ");
		PRTLN(getState());
	}
#endif
	if (isState(IDLE))
	{
		//for local packet
		bool specialRequestPacket;
		if(!isMessageStackEmpty())
		{
			
			apiframe cmd = checkNextPacketToSend();
			if(cmd.size() >3 )
			{
				//Serial.println("messge stack sent");
				if(cmd[3] != PACKET_TXAIR_CMD_WRAPPER) specialRequestPacket = sendNextPacketBuffered();
				//delay(50);
			}
		}

		//for remote packet
		if(eob){
			specialRequestPacket = sendNextPacketBuffered();
			if(isAutoPollingNodeEnabled() && !specialRequestPacket && isMode(READY))
			{
				automaticNodePolling();
				ret = true;
			}
		}		
	}
	else if(isState(WAIT))
	{

	}
	else if (isState(RESET))
	{
		if (eob_cnt == 2 && eob)
		{
			apiframe cmd = apiPacket(SMK_SOFT_RESET, {0}, LOCAL); // reg preset from ram
			WriteAndExpectAnwser(gateway, cmd, 0x12, "reset", ExpectCallback([&](mesh_t::iterator pNode, apiframe packet, bool success, String tag) -> void
			{
                //TIMEOUT
                if(!success) {
					setMode(WAITING);
					setState(STOP);
                    pNode->second.otaStep = STEP_FAILED;
                    PRTLN("  Unable to reset smk900"); 
					reset();
					return; 
                }

				setState(next_state);
                //SUCESS
				PRTLN("  Reset successful");
            }));
			ret = true;
		}
	}
	else if (isState(CHECK_FILE_AND_LOAD_IF_AVAILABLE))
	{
		PRTLN("\n--> CHECK_FILE_AND_LOAD_IF_AVAILABLE");
		setMode(UPDATE_NODES);
		setState(STOP);
		setAutoPolling(false);
		otaResult = "start at " + getTimeFormated();

		if (!firmware.open(firmware.filename))
		{
			PRTLN("Cannont open file");
			setState(IDLE);
			return false;
		}
		firmware.close();

		if (!firmware.validationUf2Integrity())
		{
			PRTLN("UF2 file is corrupted");
			return false;
		}

		firmware.calculOfEstimatedTime(getNumberOfNodeToUpdate());
		setOtaTimeout(600000);

		setState(INIT_GATEWAY_REGISTER);
		logJson("UF2 OK");
		requiredMeshSpeed.rf_speed = PRESET_72B;
		requiredMeshSpeed.duty = 5;
		
		dumpReceivedBuffer();
		ret = true;
	}
	else if (isState(INIT_GATEWAY_REGISTER))
	{
		#if SHOW_SMK900_INIT
		Serial.println("--> INIT_GATEWAY_REGISTER");
		#endif
		setState(INIT_GATEWAY_REGISTER_WAIT_DONE);
		log[String(millis())] = "INIT smk900";

		// set gateway mac as unavailable
		gatewayMacAddressIsReceived = false;

		// void (*cc)(const mesh_t::iterator pNode, bool success);
		// https://www.learncpp.com/cpp-tutorial/lambda-captures/

		setOtaTimeout(60000);
		apiframe cmd1 = apiPacket(SMK_WRITE_REG, {0x00, 17, 0x01, 0x01}, LOCAL);

		//mesh_t gateway_boot;
		SmkNode g;
		gateway_boot.insert(std::make_pair(0,g));

		WriteAndExpectAnwser(gateway_boot.begin(), cmd1, 0x14, 6, "init", ExpectCallback([](mesh_t::iterator pNode, apiframe packet, bool success, String tag) -> void
																			{
            if(!success) { Serial.println(" Error: Unable to set EOB flag"); return; }

			#if SHOW_SMK900_INIT
            Serial.println("  EOB flag enabled");
			#endif

            //uint8_t uart_setting[10] = {0x0E, 0,0,0,0,0,100,0};
            //apiframe cmd2 = localSetRegister(34,8,uart_setting); //Enable sleep for the gateway in order to fix esd bug...
			apiframe cmd2 = apiPacket(SMK_WRITE_REG, {0x00, 34, 0x08, 0x0E, 0,0,0,0,0,100,0}, LOCAL);

            WriteAndExpectAnwser(gateway_boot.begin(), cmd2,0x14,  "init", ExpectCallback([](mesh_t::iterator pNode, apiframe packet, bool success, String tag) ->void {
                if(!success) { return; }
				#if SHOW_SMK900_INIT
                Serial.println(pNode->second.name + "  Gateway sleep mode enabled");
				#endif
				//log[String(millis())] = "EOB written";
            
                //apiframe cmd3 = localTransfertConfigFromRAMBUFF(1); //transfert it to RAM to enable config sent
				//    uint8_t cmd[5] = {0xFB, 2, 0, 0x0B, location};
				apiframe cmd3 = apiPacket(SMK_TRANSFERT_MEM, {0x01}, LOCAL);
                WriteAndExpectAnwser(gateway_boot.begin(), cmd3, 0x1B,  "init", ExpectCallback([](mesh_t::iterator pNode, apiframe packet, bool success, String tag) ->void {
                    if(!success) { Serial.println(" Node " + pNode->second.name + " is unavailable."); return; }
						#if SHOW_SMK900_INIT
                        	Serial.println(pNode->second.name + "  Transfert to RAM");
						#endif

                        //apiframe cmd5 = requestMacAddress();
						//uint8_t cmd[20] = {0xFB, 4, 0, 3, 2,0,8};
						apiframe cmd5 = apiPacket(SMK_READ_REG, {0x00, 0x00, 0x08}, LOCAL);
					
					WriteAndExpectAnwser(gateway_boot.begin(), cmd5, 0x13,  "init", ExpectCallback([](mesh_t::iterator pNode, apiframe packet, bool success, String tag) ->void {
						if(!success) { Serial.println(" Node " +   pNode->second.name + " is unavailable."); return; }
						
						SaveGatewayMacAddress(packet);
						IPAddress RepliedMacAddress = {packet[10], packet[9], packet[8], packet[7]}; 
						String smac =RepliedMacAddress.toString();
						uint32_t a = SmkList::macString2Umac(smac);

						if(!initDone)
						{
							nodes.pool.clear(); 
							nodes.add(a, "gateway", LOCAL, "main", "main gateway");
							gateway=nodes.pool.begin();
							if(cbLoadExternalParamFiles){
								cbLoadExternalParamFiles();
								dumpReceivedBuffer();
							} 
							else{
								nodes.loadParamFiles();
							} 
							PRT("GATEWAY mac is: ");
							PRTLN(smac);
						}
						SpidermeshApi::findNext(true,true);
						//auto g = gateway;
						initDone = true;
						setState(GET_SPEED_DYN);
					}));
                }));
            })); 
		}));

		ret = true;
	}
	else if (isState(INIT_GATEWAY_REGISTER_WAIT_DONE))
	{
		if(initDone)
		{ 
			setState(GET_SPEED_DYN);		
			ret=true;
		}
	}
	else if (isState(GET_SPEED_DYN))
	{
		PRTLN("\n--> GET_SPEED_DYN");
		log[String(millis())] = "GET_SPEED_DYN";
		setOtaTimeout(60000);

		// apiframe cmd = apiPacket({0x03, 0x01, 0x02, 0x06}); // ret reg 2 DYN
		apiframe cmd = apiPacket(SMK_READ_REG, {0x00, 2, 6}, LOCAL);

		setState(WAIT);
		WriteAndExpectAnwser(gateway, cmd, 0x13,  "getdyn", ExpectCallback([&](mesh_t::iterator pNode, apiframe packet, bool success, String tag) -> void
																		{
            //TIMEOUT
            if(!success) { PRTLN("  Unable to get speed dyn"); return; }
        
            //SUCESS
            PRTF("  dyn bo:%d", packet[7]);
            PRTF("  bi:%d", packet[8]);
            PRTF("  hop:%d", packet[9]);
            PRTF("  red:%d", packet[10]);
            PRTF("  ren:%d", packet[11]);
            PRTF("  duty:%d\n", packet[12]);

            actualMeshSpeed.bo = packet[7];
            actualMeshSpeed.bi = packet[8];
            actualMeshSpeed.hop = packet[9];
            actualMeshSpeed.rd = packet[10];
            actualMeshSpeed.rde = packet[11];
            actualMeshSpeed.duty = packet[12];
            resetOtaTimeout();

			if(!isDynOptimalUpdateSpeed()) setState(SET_SPEED_DUTY);
			else setState(GET_SPEED_RF);
		}));

		ret = true;
	}
	else if (isState(SET_SPEED_DUTY))
	{
		PRTLN("\n--> SET_SPEED_DYN");
		log[String(millis())] = "SET_SPEED_DYN";
		setOtaTimeout(60000);

		uint8_t h = readFile("/hops").toInt();
		if (h == 0) h = 8;
		uint8_t d = readFile("/duty").toInt();
		if (d == 0)	d = 5;


		//if specified by user with begin function
		if(requiredMeshSpeed.hop != -1 && requiredMeshSpeed.duty != -1 && isMode(INIT_SMK900))
		{
			actualMeshSpeed.hop = requiredMeshSpeed.hop;
			h = requiredMeshSpeed.hop;
			actualMeshSpeed.duty =requiredMeshSpeed.duty;
			d=requiredMeshSpeed.duty;
		}

		setState(WAIT);
		apiframe cmd = apiPacket(0x0A, {actualMeshSpeed.bo, actualMeshSpeed.bi, h, actualMeshSpeed.rd, actualMeshSpeed.rde, d}, LOCAL); // ret reg 2 DYN
		WriteAndExpectAnwser(gateway, cmd, 0x1A, "setspeed",ExpectCallback([&](mesh_t::iterator pNode, apiframe packet, bool success, String tag) -> void
																		{
			//TIMEOUT
			if(!success) {
				pNode->second.otaStep = STEP_FAILED;
				PRTLN("  Unable to set speed dyn"); return; 
			}
			log[String(millis())] = "--OK";
		
			//SUCESS
			setState(GET_SPEED_RF);
			
			resetOtaTimeout();
		}));
		ret = true;
	}
	else if (isState(GET_SPEED_RF))
	{
		log[String(millis())] = "--OK";
		//__________________________________________
		PRTLN("\n--> GET_SPEED_RF");
		// apiframe cmd = apiPacket({0x03, 1, 11, 1}); // reg preset from ram


		setState(WAIT);
		apiframe cmd = apiPacket(SMK_READ_REG, {1, 11, 1}, LOCAL); // reg preset from ram
		WriteAndExpectAnwser(gateway, cmd, 0x13, "getpreset", ExpectCallback([&](mesh_t::iterator pNode, apiframe packet, bool success, String tag) -> void
		{
			//TIMEOUT
			if(!success) {
				pNode->second.otaStep = STEP_FAILED;
					PRTLN("  Unable to get speed rf"); return; 
			}

			//SUCESS

			#define EXPECTED_PRESET_RF_DURING_OTA_UPDATE 0x10
			uint8_t presetRF = packet[7]&0x10;
			uint8_t expectedPresetRF = EXPECTED_PRESET_RF_DURING_OTA_UPDATE;

			log[String(millis())] = "GET_SPEED_RF";

			if(isMode(INIT_SMK900) || (firmware.isHost() && isMode(UPDATE_NODES_END))) expectedPresetRF = EXPECTED_PRESET_RF_AT_BOOT;

			else if(requiredMeshSpeed.rf_speed !=-1)
			{
				actualMeshSpeed.rf_speed = requiredMeshSpeed.rf_speed;
				expectedPresetRF = requiredMeshSpeed.rf_speed;
			}

			if(presetRF==expectedPresetRF) //already ok so skip writing preset
			{
				setState(TEST_SERIAL_COMM);    
			}
			else
			{
				//__________________________________________
				PRTLN("\n--> SET_SPEED_RF");
				//apiframe cmd = apiPacket({0x04, 0, 11, 1,expectedPresetRF}); 
				
				setState(WAIT);
				apiframe cmd = apiPacket(SMK_WRITE_REG, {0, 11, 1, expectedPresetRF}, LOCAL);
				WriteAndExpectAnwser(gateway, cmd, 0x14, "getpreset", ExpectCallback([&](mesh_t::iterator pNode, apiframe packet, bool success, String tag) ->void 
				{
					//TIMEOUT
					if(!success) { PRTLN("  Unable to get speed rf"); return; }

					log[String(millis())] = "SET_SPEED_RF";
					//__________________________________________
					PRTLN("\n--> SAVE TO EEPROM");
					//apiframe cmd = apiPacket({0x0B, 2});
					setState(WAIT);
					apiframe cmd = apiPacket(SMK_TRANSFERT_MEM, {2}, LOCAL);
					WriteAndExpectAnwser(gateway, cmd, 0x1B, "getpreset", ExpectCallback([&](mesh_t::iterator pNode, apiframe packet, bool success, String tag) ->void 
					{
						//TIMEOUT
						if(!success) { PRTLN("  Unable to save to EEPROM"); return; }

						setState(RESET,TEST_SERIAL_COMM);
						PRTLN("  SAVE to EEPROM sucess");
						log[String(millis())] = "SAVE to EEPROM";
						setOtaTimeout(300000);
					}));
					resetOtaTimeout();
				}));
			} 
		}));
		ret = true;
	}
	else if (isState(TEST_SERIAL_COMM))
	{
		//__________________________________________
		PRTLN("\n--> TEST SERIAL COMM");
		// apiframe cmd = apiPacket({0x03, 2, 11, 1});
		setState(WAIT);
		apiframe cmd = apiPacket(SMK_READ_REG, {2, 11, 1}, LOCAL);
		WriteAndExpectAnwser(gateway, cmd, 0x13, 1, "testserial", ExpectCallback([&](mesh_t::iterator pNode, apiframe packet, bool success, String tag) -> void
																		   {
            //TIMEOUT
            if(!success) { PRTLN("  Unable reach gateway via serial"); return; }
            PRTLN("  Test serial comm sucess");
			logJson("Test serial comm sucess");

            if(isMode(UPDATE_NODES)) 
            {
                setOtaTimeout(300000);
				setState(READ_VERSION_NODES);
            }
			else if(firmware.isHost() && isMode(UPDATE_NODES_END))
			{
				setState(HOST_CHECK_PROGRESS_INIT);
			}
			
            else 
            {
                setMode(READY);
                setState(IDLE);
            }
            resetOtaTimeout(); 
		}));
		ret = true;
	}
	//----------------------------------------------------------
	else if (isState(SET_CHANNEL_RF))
	{
		PRTLN("\n--> SET_CHANNEL_RF");
		log[String(millis())] = "SET_CHANNEL_RF";
		//pCurrentNode->second.otaStep = STEP_WAIT;
		setOtaTimeout(60000);

		setState(WAIT);
		byte network_id = (channel_rf-1)%NWK_COUNT;
		apiframe cmd = apiPacket(SMK_WRITE_REG, {0, 3, 1, network_id}, LOCAL);
		WriteAndExpectAnwser(gateway, cmd, 0x14, "setchannel",ExpectCallback([&](mesh_t::iterator pNode, apiframe packet, bool success, String tag) -> void
		{
			//TIMEOUT
			if(!success) {
				pNode->second.otaStep = STEP_FAILED;
				PRTLN("  Unable to set channel"); return; 
			}
			//SUCESS
			PRTLN("  Set Network ID Done");
			byte htable=  HOPTABLE_50K_START + (channel_rf-1)%HOPTABLE_50K_COUNT;
			apiframe cmd = apiPacket(SMK_WRITE_REG, {0, 4, 1, htable}, LOCAL);
			WriteAndExpectAnwser(gateway, cmd, 0x14, "sethoptable",ExpectCallback([&](mesh_t::iterator pNode, apiframe packet, bool success, String tag) -> void
			{
				//TIMEOUT
				if(!success) {
					pNode->second.otaStep = STEP_FAILED;
					PRTLN("  Unable to set hop table"); return; 
				}


				//SUCESS
				PRTLN("  Set Hop Table Done");
				//apiframe cmd = apiPacket({0x0B, 2});
				apiframe cmd = apiPacket(SMK_TRANSFERT_MEM, {2}, LOCAL);
				WriteAndExpectAnwser(gateway, cmd, 0x1B, "write channel", ExpectCallback([&](mesh_t::iterator pNode, apiframe packet, bool success, String tag) ->void 
				{
					

					//TIMEOUT
					if(!success) {
						setMode(READY);
						setState(IDLE);
						PRTLN("  Unable to save to EEPROM");
					}
					else
					{
						PRTLN("  SAVE to EEPROM sucess");
						log[String(millis())] = "SAVE to EEPROM";
						setMode(INIT_SMK900);
						setState(RESET, INIT_GATEWAY_REGISTER);
					}
					setOtaTimeout(600000);
				}));

				resetOtaTimeout();
			}));
			resetOtaTimeout();
		}));
		ret = true;
	}

	//----------------------------------------------------------
	else if (isState(READ_VERSION_NODES))
	{
		// setup
		if (eob && eob_cnt == 1)
		{
			PRTLN("\n--> READ_VERSION_NODES_INIT");
			logJson("READ_VERSION_NODES_INIT");
			nodes.resetStep();
			ClearFifoAndExpectList();
			pCurrentNode = nodes.pool.begin();
			setOtaTimeout(900000);
			findNextNode(true, !isMode(READ_VERSION_SMK900));
		}
		// loop
		else if (eob && eob_cnt > 3)
		{
			if (currentNodeCanBePolled())
			{
				PRTLN("\n--> READ_VERSION_NODES");
				pCurrentNode->second.otaStep = STEP_WAIT;

				// apiframe cmd = apiPacket({0x0C, 0x00, 0x83}, pCurrentNode, {0x02, 0x15, 0x01}); // READ VERSION REGISTER
				apiframe cmd = apiPacket(SMK_READ_REG, {0x02, 0x15, 1}, pCurrentNode->second.local);

				WriteAndExpectAnwser(pCurrentNode, cmd, SMK_READ_REG, "readV1", ExpectCallback([&](mesh_t::iterator pNode, apiframe packet, bool success, String tag) -> void
				{
                    if(!success) {  //TIMEOUT
                        PRTF("  Node %s unable to read version.\n", pNode->second.getMacAsString().c_str()); 
                        if(isMode(READ_VERSION_SMK900))
                        {
                            pNode->second.otaStep = STEP_REJECTED;
                            pNode->second.labelState = "Timeout";
							logJson("Timeout");
                            
                            PRTF("  Node %s rejected.\n", pNode->second.getMacAsString());
                            findNextNode(false,!isMode(READ_VERSION_SMK900));
                            if(nodes.isStepComplete()) 
                            {
                                setState(STOP);
                            }
                        }
                        else
                        {
                            findNextNode(false,!isMode(READ_VERSION_SMK900));
                            pNode->second.otaStep = STEP_RETRY;
                        }
                        return; 
                    }
                    resetOtaTimeout();
					logJson("success");
                    
                    if(isMode(UPDATE_NODES) || isMode(READ_VERSION_SMK900))    //SUCESS
					{
						PRTLN("  read to old version");
                        pNode->second.old_firmware.version = packet[13];
					}
                    else if(isMode(UPDATE_NODES_END))                                    
					{
						PRTLN("  read to new version");
                        pNode->second.new_firmware.version = packet[13];
					}

                    firmware.current_progress++;

                    //apiframe cmd = apiPacket({0x0C, 0x00, 0x83},pCurrentNode,{0x02, 0x18, 0x02}); // READ SUB VERSION REGISTER
					apiframe cmd = apiPacket(SMK_READ_REG, {0x02, 0x18, 2},pCurrentNode->second.local);
                    WriteAndExpectAnwser(pCurrentNode,cmd,SMK_READ_REG, "readV2", ExpectCallback([&](mesh_t::iterator pNode, apiframe packet, bool success, String tag) ->void 
                    {
                        if(!success) {  //TIMEOUT
                            pNode->second.otaStep = STEP_FAILED;
                            PRTF("  Node %s unable read subversion.\n", pNode->second.getMacAsString()); 
                            setState(STOP);
                            return; 
                        }
                    
                        //SUCESS
                        u_bytes u;
                        u.uint8b[0]=packet[13];
                        u.uint8b[1]=packet[14];
                        if(isMode(UPDATE_NODES) || isMode(READ_VERSION_SMK900))
                            pNode->second.old_firmware.sub_version = u.uint16b[0];
                        else if(isMode(UPDATE_NODES_END))                                    
                            pNode->second.new_firmware.sub_version = u.uint16b[0];

                        firmware.current_progress++;


                        //apiframe cmd = apiPacket({0x0C, 0x00, 0x83},pCurrentNode,{0x02, 0x19, 0x02}); // READ SUB VERSION REGISTER
						apiframe cmd = apiPacket(SMK_READ_REG, {0x02, 0x19, 2},pCurrentNode->second.local);
                        WriteAndExpectAnwser(pCurrentNode,cmd,SMK_READ_REG, "readV3", ExpectCallback([&](mesh_t::iterator pNode, apiframe packet, bool success, String tag) ->void 
                        {
                            //TIMEOUT
                            if(!success) {
                                pNode->second.otaStep = STEP_FAILED;
                                PRTF("  Node %s unable read database version.\n", pNode->second.getMacAsString()); 
                                setState(STOP);
                                return; 
                            }
                        
                            //SUCESS
                            u_bytes u;
                            u.uint8b[0]=packet[13];
                            u.uint8b[1]=packet[14];
                            if(isMode(UPDATE_NODES) || isMode(READ_VERSION_SMK900))
                                pNode->second.old_firmware.database = u.uint16b[0];
                            else if(isMode(UPDATE_NODES_END))                                    
                                pNode->second.new_firmware.database = u.uint16b[0];

                            firmware.current_progress++;

                            //apiframe cmd = apiPacket({0x0C, 0x00, 0x83},pCurrentNode,{0x02, 0x1A, 0x01}); // READ SUB VERSION REGISTER
							apiframe cmd = apiPacket(SMK_READ_REG, {0x02, 0x1A, 1},pCurrentNode->second.local);
                            WriteAndExpectAnwser(pCurrentNode,cmd,SMK_READ_REG, "readV4", ExpectCallback([&](mesh_t::iterator pNode, apiframe packet, bool success, String tag) ->void 
                            {
                                if(!success) { //TIMEOUT
                                    pNode->second.otaStep = STEP_FAILED;
                                    PRTF("  Node %s unable read serie.\n", pNode->second.getMacAsString()); 
                                    setState(STOP);
                                    return; 
                                }
                            
                                //SUCESS
                                if(isMode(UPDATE_NODES) || isMode(READ_VERSION_SMK900))
                                    pNode->second.old_firmware.serie = packet[13];
                                else if(isMode(UPDATE_NODES_END))                                    
                                    pNode->second.new_firmware.serie = packet[13];
                                pNode->second.otaStep = STEP_DONE;
                                firmware.current_progress++;

                                if(nodes.isStepComplete()) 
                                {
                                    if(isMode(UPDATE_NODES) && (firmware.isSmk900() || firmware.isHost()))
                                    {
                                        setState(PRIME_NODE_TO_UPDATE_INIT);
                                    }
                                    else if(isMode(UPDATE_NODES) && firmware.isVmMachine())
                                    {
                                        setState(EVM_OTA_START);
                                    }
                                    else if(isMode(UPDATE_NODES_END))
                                    {
                                        setState(STOP);
                                    }
                                    else
                                    {
                                        PRTLN("**reading node complete**");
										logJson("success");
                                        setState(IDLE);
                                        setMode(READY);
                                    }
                                }
                                findNextNode(false,!isMode(READ_VERSION_SMK900));

                                resetOtaTimeout();
                                PRTF("  Node %s have version ", pNode->second.getMacAsString());
								char b[30];
								sprintf(b,"version %s", pNode->second.getMacAsString());
								logJson(b);
                                PRTLN(pNode->second.old_firmware.getVersionString());
                            }));
                        }));
                    })); 
				}));
				ret = true;
			}else findNextNode(false,true);
		}
	}
	else if (isState(READ_HOST_VERSION_NODES_INIT))
	{
		if (eob)
		{
			PRTLN("\n--> READ_HOST_VERSION_NODES_INIT");
			logJson("READ_HOST_VERSION");
			nodes.resetStep();
			pCurrentNode = nodes.pool.begin();
			findNextNode(true, !isMode(READ_VERSION_HOST));
			setOtaTimeout(600000);
			setState(READ_HOST_VERSION_NODES);
			eob_cnt = 0;
		}
	}
	else if (isState(READ_HOST_VERSION_NODES))
	{
		if (eob && (eob_cnt == 3))
		{
			dumpReceivedBuffer(); // since polling will contain already a VM expect from automatic polling
		}
		else if (eob && (eob_cnt > 3))
		{
			if (currentNodeCanBePolled())
			{
				PRTLN("\n--> READ_HOST_VERSION_NODES");
				pCurrentNode->second.otaStep = STEP_WAIT;

				// apiframe cmd = apiPacket({0x0C, 0x00, 0x8E}, pCurrentNode, {0x38}); // READ SUB VERSION REGISTER
				apiframe cmd = apiPacket(SMK_VM_EXEC, {0x38}, pCurrentNode->second.local);
				WriteAndExpectAnwser(pCurrentNode, cmd, 0x9E, "readpyV", ExpectCallback([&](mesh_t::iterator pNode, apiframe packet, bool success, String tag) -> void
																			 {
                    //TIMEOUT
                    if(!success) {
                        
                        PRTF("  Node %s unable read pyboard version.\n", pNode->second.getMacAsString()); 
                        if(isMode(READ_VERSION_HOST))
                        {
                            pNode->second.otaStep = STEP_REJECTED;
                            pNode->second.labelState = "TIMEOUT";
                            if(nodes.isStepComplete()) 
                            {
                                setState(STOP);
                            }
							else findNextNode(false,!isMode(READ_VERSION_HOST));
							resetOtaTimeout();
                        }
                        else
                        {
                            if(pNode->second.nbFailed++ >2)
                            {
                                pNode->second.otaStep = STEP_FAILED;
                                setState(STOP);
                            }
                            else
                            {
								resetOtaTimeout();
                                pNode->second.otaStep = STEP_RETRY;
                            }                            
                        }
                        return; 
                    }
                
                    //SUCESS
					bool doFindNext = true;
                    if(packet.size() == 15)
                    {
                        //to fix small bug in evm, tiny of pyboard, suspected on tiny
                        int start_version_pos=11;
                        if( packet[1] == 0x0D) start_version_pos=12;

                        
                        if(isMode(UPDATE_NODES) || isMode(READ_VERSION_HOST))
                        {
                            pNode->second.old_firmware_pyboard.version = extractU16(packet, start_version_pos);
                            pNode->second.old_firmware_pyboard.sub_version = extractU16(packet, start_version_pos+2);
                        }
                        else if(isMode(UPDATE_NODES_END))                                    
                        {
                            pNode->second.new_firmware_pyboard.version = extractU16(packet, start_version_pos);
                            pNode->second.new_firmware_pyboard.sub_version = extractU16(packet, start_version_pos+2);
                        }

                        pNode->second.otaStep = STEP_DONE;
                        firmware.current_progress++;
						pNode->second.labelState = "OK";
                        PRTF("  Node %s pyboard version ", pNode->second.getMacAsString());
						
                        PRTLN(pNode->second.old_firmware.getVersionString());
                    }
					else if(pNode->second.nbFailed++ <3)
                    {
						
						PRT("  READ_HOST_VERSION_NODES ");
						PRTF("%s", pNode->second.getMacAsString());
						PRTLN(" STEP_RETRY");
                        pNode->second.otaStep = STEP_RETRY;
						pNode->second.labelState = "STEP_RETRY";
						doFindNext = false;
					}
                    else
                    {
						
						PRT("  READ_HOST_VERSION_NODES ");
						PRTF("%s", pNode->second.getMacAsString());
						PRTLN(" STEP_REJECTED");
                        pNode->second.otaStep = STEP_REJECTED;
						pNode->second.labelState = "REJECTED";

                    }

                    if(nodes.isStepComplete()) 
                    {
                        if(isMode(UPDATE_NODES))
                        {
							PRTLN("  READ_HOST_VERSION_NODES  UPDATE_NODES");

							if(firmware.isVmMachine())
								setState(EVM_OTA_START);
							else
                            	setState(PRIME_NODE_TO_UPDATE_INIT);
                        }
                        else if(isMode(UPDATE_NODES_END))
                        {
							PRTLN("  READ_HOST_VERSION_NODES  UPDATE_NODES_END");
                            setState(STOP);
                        }
                        else
                        {
							PRTLN("  READ_HOST_VERSION_NODES  else");
                            setState(IDLE);
                            setMode(READY);
                        }
                    }

                    if(doFindNext) findNextNode(false,!isMode(READ_VERSION_HOST));
				    resetOtaTimeout(); 

				}));

				ret = true;
			}else findNextNode(false,true);
		}
	}
	// for EVM only --------------------------------------------
	else if (isState(EVM_OTA_START))
	{
		if (eob && (eob_cnt == 1))
		{
			PRTLN("\n--> EVM_OTA_START_INIT");
			nodes.resetStep();
			pCurrentNode = nodes.pool.begin();
			findNextNode(true, true);

			setOtaTimeout(60000);
			dumpReceivedBuffer();
			ret = true;
		}
		else if (eob && (eob_cnt > 2))
		{
			if (currentNodeCanBePolled())
			{
				PRTLN("\n--> EVM_OTA_START");
				pCurrentNode->second.otaStep = STEP_WAIT;

				// apiframe cmd = apiPacket({0x0C, 0x00, 0x86}, pCurrentNode, {0x00, firmware.nbBlock.uint8b[0], firmware.nbBlock.uint8b[1]});

				byte ota_subcommand = 0x00;
				if (firmware.getType() == EVM)
					ota_subcommand |= 0x80;

				apiframe cmd = apiPacket(SMK_VM_FLASH, {0x00}, pCurrentNode->second.local); // start evm flash, will stop an active EVM as well if there is any

				PRTF("  MAC to evm start %s\n", pCurrentNode->second.getMacAsString());

				WriteAndExpectAnwser(pCurrentNode, cmd, PACKET_VM_FLASH_RESP, "evmstart", ExpectCallback([&](mesh_t::iterator pNode, apiframe packet, bool success, String tag) -> void
																						{
                    //TIMEOUT
                    if(!success) {
                        pNode->second.otaStep = STEP_FAILED;
                        PRTF("  Node %s unable to start evm.\n", pNode->second.getMacAsString()); 
                        setState(STOP);
                        return; 
                    }
                
                    //SUCESS
                    pNode->second.otaStep = STEP_DONE;
                    pNode->second.labelState = "NODE EVM STARTED";
                    firmware.current_progress++;
                    
                    if(nodes.isStepComplete()) 
                    {
                        setState(PRIME_NODE_TO_UPDATE_INIT);
                    }
                    resetOtaTimeout();
                    PRTF("  Node %s have been EVM started.\n", pNode->second.getMacAsString()); 
				}));

				ret = true;
				findNextNode(true,true);
			}
			ret = true;
		}
	}
	//----------------------------------------------------------
	else if (isState(PRIME_NODE_TO_UPDATE_INIT))
	{
		PRTLN("\n--> PRIME_NODE_TO_UPDATE_INIT");
		nodes.resetStep();
		pCurrentNode = nodes.pool.begin();
		findNextNode(true, true);

		setOtaTimeout(60000);
		dumpReceivedBuffer();
		setState(PRIME_NODE_TO_UPDATE);
	}
	else if (isState(PRIME_NODE_TO_UPDATE))
	{
		if (eob)
		{
			if (currentNodeCanBePolled())
			{
				PRTLN("\n--> PRIME_NODE_TO_UPDATE");
				logJson("PRIME_NODE_TO_UPDATE " + pCurrentNode->second.getMacAsString());
				pCurrentNode->second.otaStep = STEP_WAIT;

				// apiframe cmd = apiPacket({0x0C, 0x00, 0x86}, pCurrentNode, {0x00, firmware.nbBlock.uint8b[0], firmware.nbBlock.uint8b[1]});

				byte ota_subcommand = 0x00;
				if (firmware.getType() == EVM)
					ota_subcommand |= 0x80;

				apiframe cmd = apiPacket(SMK_UPDATE_OTA_CMD, {ota_subcommand, firmware.nbBlock.uint8b[0], firmware.nbBlock.uint8b[1]}, pCurrentNode->second.local);

				PRTF("  MAC to prime %s\n", pCurrentNode->second.getMacAsString());

				WriteAndExpectAnwser(pCurrentNode, cmd, SMK_UPDATE_OTA_CMD, "primenode", ExpectCallback([&](mesh_t::iterator pNode, apiframe packet, bool success, String tag) -> void
																						{
                    //TIMEOUT
                    if(!success) {
                        pNode->second.otaStep = STEP_FAILED;
                        PRTF("  Node %s unable to prime it.\n", pNode->second.getMacAsString()); 
                        setState(STOP);
                        return; 
                    }
                
                    //SUCESS
                    pNode->second.otaStep = STEP_DONE;
                    pNode->second.labelState = "NODE PRIMED";
                    firmware.current_progress++;
                    
                    if(nodes.isStepComplete()) 
                    {
                        setState(BULKUPLOAD_INIT);
                    }
                    resetOtaTimeout();
                    PRTF2("  Node %s in ota mode. State: %d\n", pNode->second.getMacAsString(), pNode->second.otaStep); 
				}));
				findNextNode(false,true);
				ret = true;
				
			}
			else findNextNode(false, true);
		}
	}
	else if (isState(BULKUPLOAD_INIT))
	{
		setOtaTimeout(60000);
		firmware.resetOffset();
		firmware.uf2 = SPIFFS.open(firmware.filename);
		nodes.setLabels("UPLOAD");

		setState(BULKUPLOAD);
		PRTLN("\n--> BULKUPLOAD_INIT");
		logJson("BULKUPLOAD_INIT ");
	}
	else if (isState(BULKUPLOAD))
	{

	#if SKIP_BULKUPLOAD
		setState(GETMISSINGFLAGS_INIT);
		PRTLN("    BULKUPLOAD step is skipped");
	#else
	#if SIMULATION_BULKUPLOAD
		firmware.readNextBlock(); // read block if needed (4 chunk per block)
		int nb_to_send = firmware.checkNextChunk();
		if (nb_to_send > 0)
		{
			float percent = firmware.current_block;
			percent = percent / firmware.nbBlock.uint32b;
			percent *= 100.0;
			PRTF2("bulk: page:%3d  perc:%03.1f: ", firmware.current_block, percent);

			for (int i = 0; i < SIZE_DATA_PER_PACKET; i++)
			{
				// write(firmware.getChunkByte());
				PRTF("%02X", firmware.getChunkByte());
			}
			PRTLN("");
			resetOtaTimeout();
		}
		if (firmware.isEndCondition())
			setState(GETMISSINGFLAGS_INIT);
		firmware.current_progress++;
	#else
		if (eob)
		{
			firmware.readNextBlock(); // read block if needed (4 chunk per block)
			int nb_to_send = firmware.checkNextChunk();

			if (nb_to_send > 0)
			{
				u_bytes sPage;
				sPage.uint32b = firmware.start_page;

				byte ota_subcommand = 0x01;
				if (firmware.getType() == EVM)
					ota_subcommand |= 0x80;

				apiframe pkt = {ota_subcommand, sPage.uint8b[0], sPage.uint8b[1]};
				for (int i = 0; i < SIZE_DATA_PER_PACKET; i++)
				{
					// write(firmware.getChunkByte());
					pkt.push_back(firmware.getChunkByte());
				}
				float percent = firmware.current_block;
				percent = percent / firmware.nbBlock.uint32b;
				percent *= 100.0;
				PRTF("bulk: %4.1f%: ", percent);
				//PRTF("page: %3d - ", firmware.current_block);

				apiframe cmd = apiPacket(SMK_UPDATE_OTA_CMD, pkt, pCurrentNode->second.local, true);

				firmware.start_page += NB_PAGE_PER_DATA_PACKET;
				/*
				for (int i = 0; i < SIZE_DATA_PER_PACKET; i++)
				{
					// write(firmware.getChunkByte());
					x.push_back(firmware.getChunkByte());
				}*/

	#if SIM_SKIP_SOME_BROADCAST_UPLOAD
				if (sim_skip_first_packet++ > SIM_SKIP_SOME_BROADCAST_UPLOAD_EVERY)
				{
					sim_skip_first_packet = 0;
					PRT("--------- packet skipped >");
				}
				else
					write(cmd);
	#else
				write(cmd);
	#endif

				resetOtaTimeout();


				firmware.current_progress++;
				ret = true;
			}
			if (firmware.isEndCondition())
			{
				setState(GETMISSINGFLAGS_INIT);
			}
		}
#endif
#endif
	}

	else if (isState(GETMISSINGFLAGS_INIT))
	{
		PRTLN("\n--> GETMISSINGFLAGS_INIT");
		nodes.resetStep();
		pCurrentNode = nodes.pool.begin();

		// find a node to update
		findNextNode(true,true);

		setState(GETMISSINGFLAGS);
		current_sub_state = GETMISSINGFLAG_FIND_NEXT_PAGE;

		setOtaTimeout(30000);
	}
	else if (isState(GETMISSINGFLAGS))
	{
		if (!eob)
			return false;
		if (currentNodeCanBePolled())
		{
			PRTLN("\n--> GETMISSINGFLAGS");
			logJson("GETMISSINGFLAGS " + pCurrentNode->second.getMacAsString());
			pCurrentNode->second.otaStep = STEP_WAIT;

			// apiframe cmd = apiPacket({0x0C, 0x00, 0x86}, pCurrentNode, {0x07});

			byte ota_subcommand = 0x07;
			if (firmware.getType() == EVM)
				ota_subcommand |= 0x80;
			apiframe cmd = apiPacket(SMK_UPDATE_OTA_CMD, {ota_subcommand}, pCurrentNode->second.local);

			PRTF("  MAC to get missing flags: %a\n", pCurrentNode->second.getMacAsString());

	#if SIMULATION_GETMISSINGFLAGS
			pCurrentNode->second.otaStep = STEP_DONE;

			firmware.current_progress++;

	#if SIMULATION_MISSINGFLAGS_ERROR
			static bool sim_done_once = false;
			if (!sim_done_once)
			{
				PRTLN("  MISSING FOUND");
				firmware.missing_list.push_back(missing_element_t(1, 0xFFFF));
				setState(SEND_MISSING_PAGE_INIT);
				sim_done_once = true;
			}
			else
			{
				PRTLN("  NO MISSING FOUND");
				if (nodes.isStepComplete())
					setState(PRUNE_VALID_PAGES_INIT);
			}
	#endif

	#else

			WriteAndExpectAnwser(pCurrentNode, cmd, SMK_UPDATE_OTA_CMD, "getmissflag",
								 ExpectCallback([&](mesh_t::iterator pNode, apiframe packet, bool success, String tag) -> void
												{
                    //TIMEOUT
                    if(!success) {
                        pNode->second.otaStep = STEP_FAILED;
                        PRTF("  Node %s unable to get missing\n", pNode->second.getMacAsString()); 
                        return; 
                    }

                    resetOtaTimeout();
                    if(packet.size()>11) //if there is a page of missing flags
                    {

                        int nb_page = (packet.size()-11)/4;

                        for(int i=0; i<nb_page; i++)
                        {
                            int32_t page = extractU16(packet,i*4+11);
                            uint32_t flag = extractU16(packet,i*4+13);
	#if SIM_MISSING_PACKET_SCRATCH
							if(page==0 && firmware.test_skip_scratch >0)
							{
								firmware.test_skip_scratch--;
								flag |= SIM_MISSING_PACKET_SCRATCH_BIT;
							}
	#endif

                            firmware.missing_list.push_back(missing_element_t (page, flag));
                            PRTF2("  scratchByteId:%d - flag:%04X\n", page, flag);
                        }
                        PRTF("  ERROR - NB MISSING PAGE:%d\n", nb_page);
	                    //bug for those version so drop the node
                        if(pNode->second.old_firmware.version == 2 && pNode->second.old_firmware.sub_version < 19)
                        {
	#if SKIP_NODE_WHEN_MISSING_PAGE_OLD_VERSION
                            pNode->second.otaStep = STEP_REJECTED; 
                            findNextNode(false,true);
	#else
                            setState(SEND_MISSING_PAGE_INIT);
	#endif
                        }
                        else
                        {
                            setState(SEND_MISSING_PAGE_INIT);
                        }
                        return;
                    }
                    else //no missing flag
                    {
                        pNode->second.otaStep = STEP_DONE;
                        firmware.current_progress++;
						PRTF("  Node %s have no missing flag\n", pNode->second.getMacAsString()); 
                        findNextNode(false, true);
                    }
                    
                
                    //SUCESS
                    if(nodes.isStepComplete()) 
                    {
                        setState(PRUNE_VALID_PAGES_INIT);
                    }
                    resetOtaTimeout();
                    
				}));

	#endif
			ret = true;
		}else findNextNode(false,true);
	}
	else if (isState(SEND_MISSING_PAGE_INIT))
	{
		if (eob)
		{
			if (firmware.missing_list.size() > 0)
			{
				PRTLN("\n__________________________\n--> SEND_MISSING_PAGE_INIT");
				setState(SEND_MISSING_PAGE);
				firmware.current_block = -1;
				wait_eob_count = 0;
				firmware.it_missing_list = firmware.missing_list.begin();
			}
			else if (wait_eob_count++ > 2)
				setState(GETMISSINGFLAGS_INIT);
			setOtaTimeout(180000);
		}
	}
	else if (isState(SEND_MISSING_PAGE))
	{
		if (eob)
		{
			PRT("\n--> SEND_MISSING_PAGE\n");
			logJson("SEND_MISSING_PAGE for " +pCurrentNode->second.getMacAsString());

			apiframe payload;

			auto item = firmware.missing_list.begin();

			apiframe api_packet = {};
			int nb_page_added = 0;
			if (item != firmware.missing_list.end())
			{
				uint32_t mask = 0x01;
				// u_bytes first_chunk_of_packet;
				// first_chunk_of_packet.int32b = -1;
				bool first_time = true;

				for (int i = 0; i < NB_SIZE_SCRATCH_CHUNK; i++)
				{
					if (!(item->second & mask)) // if scratch is 0 meaning missing
					{
						item->second |= mask; // remove the scratch missing

						// int32_t chunk = item->first*8 + i;
						u_bytes x;
						x.int32b = ((item->first) * 8) + i;
						if (first_time)
						{
							first_time = false;
							byte ota_subcommand = 0x01;
							if (firmware.getType() == EVM)
								ota_subcommand |= 0x80;

							if (((pCurrentNode->second.old_firmware.version <= 2) && (pCurrentNode->second.old_firmware.sub_version < 19)) || FORCE_UNICAST_SENDING_MISSING_FLAG)
							{
								//ota_subcommand |= 0x40;
								payload = {ota_subcommand, x.uint8b[0], x.uint8b[1]};
								// 0xFB 0x19 0x00 0x0C 0x00 0x86 0x3E 0x40 0x00 0x01 0x00 0x00 0x01 0x02 0x03 0x04 0x05 0x06 0x07 0x08 0x09 0x0A 0x0B 0x0C 0x0D 0x0E 0x0F 0x10
								// 0x0C 0x00 0x86 0x01 0x40 0x3E 0x00 0x01 0x00 0x00 0x01 0x02 0x03 0x04 0x05 0x06 0x07 0x08 0x09 0x0A 0x0B 0x0C 0x0D 0x0E 0x0F 0x10
								// payload = {0x0C, 0x00, 0x86, pCurrentNode->second.mac.bOff[0], pCurrentNode->second.mac.bOff[1], pCurrentNode->second.mac.bOff[2], 0x01, x.uint8b[0], x.uint8b[1]};
								// payload = {0x01, x.uint8b[0], x.uint8b[1]};
							}
							else
							{
								payload = {ota_subcommand, x.uint8b[0], x.uint8b[1]};
								// payload = {0x0C, 0x00, 0xC6, 0x01, x.uint8b[0], x.uint8b[1]};
								// payload = {0x01, x.uint8b[0], x.uint8b[1]};
							}
							resetOtaTimeout();
						}
						firmware.readBlockContainingChunk(x.int32b);
						for (int j = 0; j < NB_SIZE_SCRATCH_CHUNK; j++)
							payload.push_back(firmware.getChunkByte());

						if (++nb_page_added >= NB_PAGE_PER_DATA_PACKET)
							break;
					}
					mask <<= 1;
				}

				if (mask >= 0x00010000) // meaning no scratch left
					firmware.missing_list.pop_front();
				else
				{
					if (((pCurrentNode->second.old_firmware.version <= 2) && (pCurrentNode->second.old_firmware.sub_version < 19)) || FORCE_UNICAST_SENDING_MISSING_FLAG)
					{
						api_packet = apiPacket(SMK_UPDATE_OTA_CMD, payload, pCurrentNode->second.local);
					}
					else
					{
						api_packet = apiPacket(SMK_UPDATE_OTA_CMD, payload, pCurrentNode->second.local, BROADCAST_TO_PRIMED_NODE);
					}
					// api_packet = apiPacket(payload);
				}
			}

			if (payload.size() > 0)
			{
				write(api_packet);
				ret = true;
			}

			if (firmware.missing_list.size() <= 0)
			{
				setState(SEND_MISSING_PAGE_INIT);
			}
		}
	}
	else if (isState(PRUNE_VALID_PAGES_INIT))
	{
		setOtaTimeout(60000);
		firmware.resetOffset();
		PRTLN("\n--> PRUNE_VALID_PAGES_INIT");
		logJson("PRUNE_VALID_PAGES_INIT");

		setState(PRUNE_VALID_PAGES);
		// dumpReceivedBuffer();
	}
	else if (isState(PRUNE_VALID_PAGES))
	{

	#if SIMULATION_PRUNE_VALID_PAGES

		std::vector<uint32_t> crc_result_vector = firmware.buildCrcPacket();
		if (crc_result_vector.size() > 0)
		{
			PRT("out: 0xFB ");

			int size_packet = 6 + crc_result_vector.size() * 4; // header + data size

			PRTF("%02X", size_packet & 0xFF);
			PRTF("%02X", (size_packet >> 8) & 0xFF);
			PRT(" 0C00C6 09 ");

			// page
			PRTF("%02X", firmware.start_page & 0xFF);
			PRTF("%02X ", (firmware.start_page >> 8) & 0xFF);

			for (auto c : crc_result_vector)
			{
				u_bytes uc;
				uc.uint32b = c;
				PRTF("%02X", uc.uint8b[0]);
				PRTF("%02X", uc.uint8b[1]);
				PRTF("%02X", uc.uint8b[2]);
				PRTF("%02X", uc.uint8b[3]);
				PRT("  ");
			}
			PRTLN("");
			resetOtaTimeout();
		}
		if (firmware.isEndCondition())
		{
			setState(CHECK_IF_CRC_OK_INIT);
		}
		firmware.current_progress++;
	#else
		if (!eob)
			return false;

		PRTLN("\n--> PRUNE_VALID_PAGES");

		std::vector<uint32_t> crc_result_vector = firmware.buildCrcPacket();
		if (crc_result_vector.size() > 0)
		{
			u_bytes sPage;
			sPage.uint32b = firmware.start_page;
			// apiframe pkt = {0x0C, 0x00, 0xC6, 0x09, sPage.uint8b[0], sPage.uint8b[1]};

			byte ota_subcommand = 0x09;
			//if (firmware.isVmMachine())
			//	ota_subcommand |= 0x80;


			apiframe pkt = {ota_subcommand, sPage.uint8b[0], sPage.uint8b[1]};

			for (auto c : crc_result_vector)
			{
				u_bytes cb;
				cb.uint32b = c;
				for (int i = 0; i < 4; i++)
					pkt.push_back(cb.uint8b[i]);
			}
			// apiframe apiPkt = apiPacket(pkt);
			PRTF("  CRC page: %d  ", firmware.start_page);
			write(apiPacket(SMK_UPDATE_OTA_CMD, pkt, pCurrentNode->second.local, BROADCAST_TO_PRIMED_NODE));
			resetOtaTimeout();
			firmware.current_progress++;
			ret = true;
		}
		if (firmware.isEndCondition())
		{
			setState(CHECK_IF_CRC_OK_INIT);
		}
	#endif
	}
	else if (isState(CHECK_IF_CRC_OK_INIT))
	{
		PRTLN("\n--> CHECK_IF_CRC_OK_INIT");
		logJson("CHECK_IF_CRC_OK_INIT");
		nodes.resetStep();
		pCurrentNode = nodes.pool.begin();
		findNextNode(true,true);

		setOtaTimeout(60000);
		setState(CHECK_IF_CRC_OK);
	}
	else if (isState(CHECK_IF_CRC_OK))
	{
		if (eob)
		{
			if (currentNodeCanBePolled())
			{
				PRTF("\n--> CHECK_IF_CRC_OK of node %s\n", pCurrentNode->second.getMacAsString());
				pCurrentNode->second.otaStep = STEP_WAIT;

				// apiframe cmd = apiPacket({0x0C, 0x00, 0x86}, pCurrentNode, {0x13});
				byte ota_subcommand = 0x13;
				if (firmware.getType() == EVM)
					ota_subcommand |= 0x80;

				apiframe cmd = apiPacket(SMK_UPDATE_OTA_CMD, {ota_subcommand}, pCurrentNode->second.local);

	#if SIMULATION_CHECK_IF_CRC_OK
				pCurrentNode->second.otaStep = STEP_DONE;
				if (nodes.isStepComplete())
					setState(SEND_META_DATA_INIT);
				firmware.current_progress++;
	#else
				WriteAndExpectAnwser(pCurrentNode, cmd, SMK_UPDATE_OTA_CMD, "checkcrc", ExpectCallback([&](mesh_t::iterator pNode, apiframe packet, bool success, String tag) -> void
				{
					//TIMEOUT
					if(!success) {
						pNode->second.otaStep = STEP_FAILED;
						PRTF(" Node %s unable to check CRC\n",pNode->second.getMacAsString());
						logJson("CHECK_IF_CRC timeout"  + pCurrentNode->second.getMacAsString());
						setState(STOP);
						return; 
					}

					//SUCESS
					int size_crc = (pCurrentNode->second.local)?9:15;
					if(packet.size()>size_crc)
					{
						PRTLN("ERROR CRC");
					}
					pNode->second.otaStep = STEP_DONE;

					logJson("CRC" + pCurrentNode->second.getMacAsString() + " " + hexPacketToAscii(packet));
					
					if(nodes.isStepComplete()){
						if(firmware.isVmMachine())
							setState(RESET_NODE_ON_SEEK_INIT);
						else
							setState(SEND_META_DATA_INIT);
					} 
					PRTF(" Node %s have valid CRC\n", pNode->second.getMacAsString());
					resetOtaTimeout();
					firmware.current_progress++; 
				}));
				findNextNode(false,true);
	#endif
				ret = true;
			}
			else findNextNode(false, true);
		}
	}
	else if (isState(SEND_META_DATA_INIT))
	{
		setOtaTimeout(30000);
		PRTLN("\n--> SEND_META_DATA_INIT");
		logJson("SEND_META_DATA");
		nodes.resetStep();
		pCurrentNode = nodes.pool.begin();

		// find a node to update
		findNextNode(true, true);

		firmware.loadMetaDataBlock();
		dumpReceivedBuffer();
		setState(SEND_META_DATA);
	}
	else if (isState(SEND_META_DATA))
	{
		if (eob)
		{
			if (currentNodeCanBePolled())
			{
				PRTF("\n--> SEND_META_DATA to: %s\n", pCurrentNode->second.getMacAsString());
				pCurrentNode->second.otaStep = STEP_WAIT;

				u_bytes a;
				a.uint32b = pCurrentNode->second.offset_chunk * (SIZE_DATA_PER_PACKET);

				apiframe payload = {0x0D, a.uint8b[0], a.uint8b[1]};
				uint8_t *pBlock = &firmware.uf2Block[OFFSET_DATA + pCurrentNode->second.offset_chunk * (SIZE_DATA_PER_METADATA_PACKET)];
				for (int i = 0; i < (SIZE_DATA_PER_METADATA_PACKET); i++)
				{
					payload.push_back(pBlock[i]);
				}

				// apiframe cmd = apiPacket({0x0C, 0x00, 0x86}, pCurrentNode, {0x0D, a.uint8b[0], a.uint8b[1]}, &firmware.uf2Block[OFFSET_DATA + pCurrentNode->second.offset_chunk * SIZE_DATA_PER_PACKET], SIZE_DATA_PER_PACKET);
				apiframe cmd = apiPacket(SMK_UPDATE_OTA_CMD, payload, pCurrentNode->second.local);

	#if SIMULATION_SEND_META_DATA
				pCurrentNode->second.offset_chunk++;
				if (pCurrentNode->second.offset_chunk == 2)
					pCurrentNode->second.otaStep = STEP_DONE;
				else
					pCurrentNode->second.otaStep = STEP_INIT; // to retriger sending next chunck
				if (nodes.isStepComplete())
				{
					if (firmware.getType() == HOST)
						setState(SEND_MAGICWORD_INIT);
					else
						setState(RESET_NODE_ON_SEEK_INIT);
				}
				firmware.current_progress++;
	#else
				WriteAndExpectAnwser(pCurrentNode, cmd, SMK_UPDATE_OTA_CMD, "sendmeta", ExpectCallback([&](mesh_t::iterator pNode, apiframe packet, bool success, String tag) -> void
				{
					//TIMEOUT
					if(!success) {
						pNode->second.otaStep = STEP_FAILED;
						PRTF(" Node %s unable to send meta data\n", pNode->second.getMacAsString()); 
						pNode->second.labelState = "ERROR: META DATA";
						return; 
					}
				
					firmware.current_progress++;
					//SUCESS
					resetOtaTimeout();
					pNode->second.offset_chunk++;
					if(pNode->second.offset_chunk == NB_WRITE_META_DATA)
					{
						logJson("succes for " + pCurrentNode->second.getMacAsString());
						pNode->second.otaStep = STEP_DONE;
						//pNode->second.labelState = "META DATA RECEIVED";
					}
					else pNode->second.otaStep = STEP_INIT; //to retriger sending next chunck
					if(nodes.isStepComplete()) setState(RESET_NODE_ON_SEEK_INIT);
					PRTF(" Node %s have received metadata\n", pNode->second.getMacAsString()); 
				}));
	#endif
				findNextNode(false,true);
				ret = true;
			}
			else findNextNode(false,true);
		}
	}
	else if (isState(RESET_NODE_ON_SEEK_INIT))
	{
		setOtaTimeout(60000);
		PRTLN("\n--> RESET_NODE_ON_SEEK_INIT");
		logJson("RESET_NODE_ON_SEEK");
		nodes.resetStep();
		pCurrentNode = nodes.pool.begin();

		// find a node to update
		findNextNode(true,true);

		dumpReceivedBuffer();
		setState(RESET_NODE_ON_SEEK);
		doProcessState = true;
	}
	else if (isState(RESET_NODE_ON_SEEK))
	{
		if (!eob || !doProcessState)
			return false;
		if (currentNodeCanBePolled())
		{
			PRTF("\n--> RESET_NODE_ON_SEEK: %s\n", pCurrentNode->second.getMacAsString());
			pCurrentNode->second.otaStep = STEP_WAIT;

			// apiframe cmd = apiPacket({0x0C, 0x00, 0x84}, pCurrentNode, {0x00, 0x81, 0x04, 0xBB, 0xE2, 0x26, 0x38});
			apiframe cmd = apiPacket(SMK_WRITE_REG, {0x00, 0x81, 0x04, 0xBB, 0xE2, 0x26, 0x38}, pCurrentNode->second.local);

	#if SIMULATION_RESET_NODE_ON_SEEK
			apiframe cmd2 = apiPacket({0x0C, 0x00, 0x84}, pCurrentNode, {0x00, 0xA1, 0x04, 0x01, 0x00, 0x00, 0x00});
			pCurrentNode->second.otaStep = STEP_DONE;
			if (nodes.isStepComplete())
				setState(SEND_MAGICWORD_INIT);
			doProcessState = true;
			firmware.current_progress++;
			firmware.current_progress++;
	#else
			pCurrentNode->second.otaStep = STEP_WAIT;
			//--- Special register to unlock access to magic word register
			WriteAndExpectAnwser(pCurrentNode, cmd, SMK_WRITE_REG, "resetseek1",
								 ExpectCallback([&](mesh_t::iterator pNode, apiframe packet, bool success, String tag) -> void
			{
					// TIMEOUT
					if (!success)
					{
						pNode->second.otaStep = STEP_FAILED;
						PRTF(" Node %s unable unlock reg 0xA1\n", pNode->second.getMacAsString());
						setState(STOP);
						return;
					}

					//apiframe cmd = apiPacket({0x0C, 0x00, 0x84}, pCurrentNode, {0x00, 0xA1, 0x04, 0x01, 0x00, 0x00, 0x00});
					apiframe cmd = apiPacket(SMK_WRITE_REG, {0x00, 0xA1, 0x04, 0x01, 0x00, 0x00, 0x00}, pCurrentNode->second.local);
					firmware.current_progress++;

					//--- Special register to unlock access to magic word register
					WriteAndExpectAnwser(pNode, cmd, SMK_WRITE_REG, "resetseek2", ExpectCallback([&](mesh_t::iterator pNode, apiframe packet, bool success, String tag) -> void
																							{
							//TIMEOUT
							if(!success) {
								pNode->second.otaStep = STEP_FAILED;
								PRTF(" Node %s unable apply reset on seek\n", pNode->second.getMacAsString()); 
								logJson("failed");
								setState(STOP);
								return; 
							}
						
							//SUCESS
							pNode->second.otaStep = STEP_DONE;
							doProcessState = true;
							firmware.current_progress++;
							logJson("RESET_NODE_ON_SEEK_INIT" + pCurrentNode->second.getMacAsString() + " " + hexPacketToAscii(packet));
							
							if(nodes.isStepComplete()) 
							{

							if(firmware.getType() == EVM)
								setState(FORCE_EVM_TRIPPLET);
							else
								setState(SEND_MAGICWORD_INIT);
							}
							PRTF(" Node %s have received reset on seek command\n", pNode->second.getMacAsString()); 
							findNextNode(false,true);
					})); 
			}));
	#endif

		ret = true;
		#if SIMULATION_RESET_NODE_ON_SEEK
			doProcessState = true;
		#else
			doProcessState = false;
		#endif
		}
		else findNextNode(false,true);
	}
	else if (isState(SEND_MAGICWORD_INIT))
	{
		setOtaTimeout(60000);
		PRTLN("\n--> SEND_MAGICWORD_INIT");
		logJson("SEND_MAGICWORD");
		nodes.resetStep();
		pCurrentNode = nodes.pool.begin();

		// find a node to update
		findNextNode(true,true);

		dumpReceivedBuffer();
		setState(SEND_MAGICWORD);
	}
	else if (isState(SEND_MAGICWORD))
	{
		if (eob)
		{
			if (currentNodeCanBePolled())
			{
				PRT("\n--> SEND_MAGICWORD to :");
				PRTLN(pCurrentNode->second.getMacAsString());
				logJson("SEND_MAGICWORD " + pCurrentNode->second.getMacAsString());

				pCurrentNode->second.otaStep = STEP_WAIT;

	#if SIMULATION_SEND_MAGICWORD
				pCurrentNode->second.otaStep = STEP_DONE;

				if (nodes.isStepComplete())
				{
					setState(RESET_GATEWAY_TO_APPLY_UPDATE);
				}

				apiframe cmd2 = apiPacket({0x0C, 0x00, 0x84}, pCurrentNode, {0x00, 0x81});
				for (int i = 0; i < 4; i++)
					cmd2.push_back(firmware.uf2Block[OFFSET_METADATA_MAGICWORD + i]);
				firmware.current_progress++;
	#else

				apiframe tail = {0x0E};
				for (int i = 0; i < 4; i++)
					tail.push_back(firmware.uf2Block[OFFSET_METADATA_MAGICWORD + i]);
				// apiframe cmd = apiPacket({0x0C, 0x00, 0x86}, pCurrentNode, tail);
				apiframe cmd = apiPacket(SMK_UPDATE_OTA_CMD, tail, pCurrentNode->second.local);

				//--- Special register to unlock access to magic word register
				WriteAndExpectAnwser(pCurrentNode, cmd, SMK_UPDATE_OTA_CMD, "sendmagic", ExpectCallback([&](mesh_t::iterator pNode, apiframe packet, bool success, String tag) -> void
				{
					//TIMEOUT
					if(!success) {
						pNode->second.otaStep = STEP_REJECTED;
						PRTF("  Node %s unable to write magic word\n", pNode->second.getMacAsString()); 
						logJson("Timeout" + macIntToString(pNode->first));
						setState(STOP);
						return; 
					}
				
					//SUCESS
					int packet_idx_to_check =(pNode->second.local)?9:15;
					if(packet[packet_idx_to_check]==0x01)
					{ 
						pNode->second.otaStep = STEP_DONE;
						pNode->second.labelState = " SUCCES";
						PRTLN("  magic word ok");
						logJson("succes " + macIntToString(pNode->second.mac.address));
					}
					else
					{
						pNode->second.otaStep = STEP_REJECTED;
						pNode->second.labelState = "Update Failed";
						PRTLN("  magic word not ok");
					}
					firmware.current_progress++;
					resetOtaTimeout();
					
					if(nodes.isStepComplete()) 
					{
						if(firmware.isHost())
						{
							logJson("STEP DONE");
							//we need to slow down dyn do allow GPIO EVM to trigger 1.5 second factory reset procedure
							if(nodes.isThereAtLeastOneOk())
								setState(SLOW_DYN_FOR_HOST_OPERATION);
							else
								setState(STOP);
						}
						else
						{
							setState(RESET_GATEWAY_TO_APPLY_UPDATE);
						}
					}
					PRTF("  Node %s have received magic word\n", pNode->second.getMacAsString()); 
					findNextNode(false,true);
				}));
	#endif
				ret = true;
			}
			else findNextNode(false, true);
		}
	}
	else if (isState(FORCE_EVM_TRIPPLET))
	{
		if (eob && (eob_cnt == 1))
		{
			PRTLN("\n--> FORCE_EVM_TRIPPLET_INIT");
			nodes.resetStep();
			pCurrentNode = nodes.pool.begin();

			// find a node to update
			findNextNode(true,true);

			setOtaTimeout(600000);
			dumpReceivedBuffer();
		}
		else if (eob && (eob_cnt >= 2))
		{
			if (currentNodeCanBePolled())
			{
				PRTF("\n--> FORCE_EVM_TRIPPLET to : %s\n", pCurrentNode->second.getMacAsString());

				pCurrentNode->second.otaStep = STEP_WAIT;

				apiframe cmd = apiPacket(SMK_VM_FLASH, {0x09, firmware.evm_len.uint8b[0], firmware.evm_len.uint8b[1], firmware.evm_crc.uint8b[0], firmware.evm_crc.uint8b[1], firmware.evm_main_entry.uint8b[0], firmware.evm_main_entry.uint8b[1]}, pCurrentNode->second.local);

	#if SIMULATION_FORCE_EVM_TRIPPLET
				pCurrentNode->second.otaStep = STEP_DONE;

				if (nodes.isStepComplete())
				{
					setState(RESET_GATEWAY_TO_APPLY_UPDATE);
				}

				firmware.current_progress++;

	#else

				//--- Special register to unlock access to magic word register
				WriteAndExpectAnwser(pCurrentNode, cmd, PACKET_VM_FLASH_RESP, "forcetriplet", ExpectCallback([&](mesh_t::iterator pNode, apiframe packet, bool success, String tag) -> void
				{
                        //TIMEOUT
                        if(!success) {
                            pNode->second.otaStep = STEP_FAILED;
                            PRTF("  Node %s unable to force tripplet\n",pNode->second.getMacAsString()); 
                            setState(STOP);
                            return; 
                        }
                    
                        pNode->second.otaStep = STEP_DONE;
						pNode->second.labelState = " SUCCES";
                        firmware.current_progress++;
                       
                        if(nodes.isStepComplete()) 
                        {
							if(firmware.isHost())//nodes.isThereAtLeastOneOk())
								setState(RESET_GATEWAY_TO_APPLY_UPDATE);
							else
								setState(STOP);
                        }
                        PRTF("  Node %s force tripplet success\n", pNode->second.getMacAsString()); 
						findNextNode(false,true);
				}));
	#endif
				ret = true;
			}
			else findNextNode(false,true);
		}
	}
	else if (isState(RESET_GATEWAY_TO_APPLY_UPDATE))
	{
		if (eob && eob_cnt ==1)
		{
			firmware.close();
			setOtaTimeout(4500000); // wait a long delay to have time for reboot and resynch to mesh
			PRTLN("\n--> RESET_GATEWAY_TO_APPLY_UPDATE");
			logJson("Reset gateway");
			//write(apiPacket(SMK_SOFT_RESET, {0}, LOCAL));

			digitalWrite(RESET_PORTIA,LOW);
		}
		else if(millis() - step_start_time > 60000)
		{
			digitalWrite(RESET_PORTIA,HIGH);

			setMode(UPDATE_NODES_END);
			if (firmware.isHost())
			{
				setState(INIT_GATEWAY_REGISTER);
				// otaResult = "success transfert at " +getTimeFormated();
			}
			else
			{
				setState(READ_VERSION_NODES);

				if (nodes.isLastStepWasOk())
					otaResult = "*** success transfert at " + getTimeFormated() + " ***";
				else
					otaResult = "some node havent been updated " + getTimeFormated();
				firmware.current_progress = firmware.max_progress;
			}
		}
		ret = true;
	}

	else if (isState(ABORT_OTA_ENGINE))
	{
		if (eob)
		{
			PRT("ABORT OTA ENGINE (not implemented)");
		}
	}

	else if (isState(SLOW_DYN_FOR_HOST_OPERATION))
	{
		if (!eob)	return false;
		if(eob &&(eob_cnt==1))
		{
			setOtaTimeout(300000);
			return false;
		}

		PRTLN("\n--> SLOW_DYN_FOR_HOST_OPERATION");
		logJson("SLOW_DYN_FOR_HOST_OPERATION");

		apiframe cmd = apiPacket(0x0A, {1, 1, 8, 1, 0, 40}, LOCAL); // ret reg 2 DYN


	#if SIMULATION_SLOW_DYN_FOR_HOST_OPERATION
		setState(RESET_FACTORY_HOST_INIT);
	#else
		WriteAndExpectAnwser(gateway, cmd, 0x1A, "slowdyn", ExpectCallback([&](mesh_t::iterator pNode, apiframe packet, bool success, String tag) -> void
																		{
            //TIMEOUT
            if(!success) {
                pNode->second.otaStep = STEP_FAILED;
                PRTLN("  Unable to slow speed for host operation"); return; 
				
            }
        
            //SUCESS
			logJson("SLOW DOWN SUCCESS");
			
            setState(RESET_FACTORY_HOST_INIT);
            resetOtaTimeout();
            doProcessState = true; }));
		doProcessState = false;
		delay(50);
	#endif
		ret = true;
	}
	else if (isState(RESET_FACTORY_HOST_INIT))
	{
		setOtaTimeout(600000);
		PRTLN("\n--> RESET_FACTORY_HOST_INIT");
		logJson("RESET_FACTORY_HOST_INIT");
		
		nodes.resetStep();
		pCurrentNode = nodes.pool.begin();

		if(findNextNode(true,true))
			setState(RESET_FACTORY_HOST);
		else
			setState(STOP);

		dumpReceivedBuffer();
	}

	else if (isState(RESET_FACTORY_HOST))
	{
	#if SIMULATION_RESET_FACTORY_HOST
		PRT("\n--> RESET_FACTORY_HOST of: ");
		PRTF("%s \n", pCurrentNode->second.getMacAsString());
		apiframe cmd = apiPacket({0x0C, 0x00, 0x8E}, pCurrentNode, {0x03});
		pCurrentNode->second.otaStep = STEP_DONE;
		findNextNode();
		if (nodes.isStepComplete())
			setState(GET_SPEED_DYN);
		doProcessState = true;
	#else
		if (eob)
		{
			if (currentNodeCanBePolled())
			{
				PRTF("\n--> RESET_FACTORY_HOST: %s\n", pCurrentNode->second.getMacAsString());
				logJson("Reset fatory of "  + pCurrentNode->second.getMacAsString());
				pCurrentNode->second.otaStep = STEP_WAIT;

				// apiframe cmd = apiPacket({0x0C, 0x00, 0x8E}, pCurrentNode, {0x03});
				apiframe cmd = apiPacket(SMK_VM_EXEC, {0x03}, pCurrentNode->second.local);

				WriteAndExpectAnwser(pCurrentNode, cmd, PACKET_VM_RESP, "resetfactory", ExpectCallback([&](mesh_t::iterator pNode, apiframe packet, bool success, String tag) -> void
																					   {
                    //TIMEOUT
                    if(!success) {
                        pNode->second.otaStep = STEP_FAILED;
                        PRTF(" Node %s unable apply factory reset.\n", pNode->second.getMacAsString()); 
						

						if(pNode->second.nbFailed++ >10)
						{
							pNode->second.otaStep = STEP_DONE;
							logJson("assume reset factory is done "  + pCurrentNode->second.getMacAsString());
						}
						else logJson("unable to confirm reset fatory of "  + pCurrentNode->second.getMacAsString());
                        return; 
                    }
                
                    //SUCESS
                    pNode->second.otaStep = STEP_DONE;
                    firmware.current_progress++;
                    
                    if(nodes.isStepComplete()){
						PRTLN("RESET FACTORY STEP");
						setState(RESET_GATEWAY_TO_APPLY_UPDATE);
					}
                    doProcessState = true;
                    resetOtaTimeout();
                    PRTF(" Node %s have been reset to factory state\n", pNode->second.getMacAsString()); 
					findNextNode(false,true);
				}));
				ret = true;

				
			}else findNextNode(false, true);
		}
	#endif
	}
	else if (isState(HOST_CHECK_PROGRESS_INIT))
	{

		logJson("HOST_CHECK_PROGRESS");
		nodes.resetStep();
		pCurrentNode = nodes.pool.begin();
		findNextNode(true,true);

		setState(HOST_CHECK_PROGRESS);

		setOtaTimeout(600000);
		dumpReceivedBuffer();
	}
	else if (isState(HOST_CHECK_PROGRESS))
	{
	// PRTLN("\n--> PRIME_NODE_TO_UPDATE");
	#if SIMULATION_HOST_CHECK_PROGRESS
		PRT("\n--> HOST_CHECK_PROGRESS of: ");
		PRTF("%s \n", pCurrentNode->second.getMacAsString());
		apiframe cmd = apiPacket({0x0C, 0x00, 0x8E}, pCurrentNode, {0x07});
		pCurrentNode->second.otaStep = STEP_DONE;
		findNextNode(false,true);
		if (nodes.isStepComplete())
		{
			setState(STOP);
			firmware.current_progress = firmware.max_progress;
		}
	#else
		int16_t nbToUpdate = getNumberOfNodeToUpdate();
		int polling_on_eob_cnt = 6;
		int polling_rate = polling_on_eob_cnt - nbToUpdate;
		if (polling_rate < 1) polling_rate = 1;


		if (eob/* && (eob_cnt > polling_rate)*/)
		{
			eob_cnt = 1;
			// PRTLN("\n--> EOB PRIME_NODE_TO_UPDATE");

			// pCurrentNode->second.otaStep=STEP_WAIT;

			// apiframe cmd = apiPacket({0x0C, 0x00, 0x8E}, pCurrentNode, {0x07});
			apiframe cmd = apiPacket(SMK_VM_EXEC, {0x07}, pCurrentNode->second.local);

			PRTLN("\n--> HOST_CHECK_PROGRESS: ");

			if (1) // pCurrentNode->second.otaStep==STEP_INIT || pCurrentNode->second.otaStep==STEP_FAILED)
			{
				WriteAndExpectAnwser(pCurrentNode, cmd, PACKET_VM_RESP, "pyprogress", ExpectCallback([&](mesh_t::iterator pNode, apiframe packet, bool success, String tag) -> void
				{
                    //TIMEOUT
                    if(!success) {
                        pNode->second.otaStep = STEP_WAIT;

                        PRTF(" Node %s unable get transfert to pyboard progress\n", pNode->second.getMacAsString());
                        return; 
                    }
                

                    //SUCESS
					if(packet.size() == 20)
					{
						uint32_t c = extractU32(packet,12);
						uint32_t t = extractU32(packet,16);
					
						firmware.current_progress++;
						
						PRT("  current progress ");
						float perc_transfert = c*100.0;
						perc_transfert /=t;
						if (perc_transfert > 100.0) perc_transfert = 100.0;
						PRTF("  %.1f%% - ",perc_transfert);
						PRTF2("%d/%d\n", c, t);
						
						pNode->second.otaStep = STEP_TRANSFERT;
						pNode->second.labelState = String(perc_transfert) + "%";

						if(c >= t){
							pNode->second.otaStep = STEP_DONE;
							PRTF("  success node: %s\n", pNode->second.getMacAsString());
						}

						if(nodes.isStepComplete())
						{
							otaResult = "success transfert at " +getTimeFormated();
							setState(STOP);
						}
						
						resetOtaTimeout(); 
					}
					else
					{
						pNode->second.otaStep = STEP_RETRY;
						PRT("  error packet size ");
					} 
				}));
				ret = true;
			}

			findNextNode(false,true);
		}
	#endif
	}
	else if (isState(STOP))
	{
		//firmware.current_progress = firmware.max_progress;
		PRTLN("\n--> STOP");
		delay(100);
	#define NO_RESET_AT_END_OF_UPDATE true
	#if NO_RESET_AT_END_OF_UPDATE
		setMode(INIT_SMK900);
		setState(INIT_GATEWAY_REGISTER);
	#else
		if (isMode(UPDATE_NODES_END))
		{
			esp_restart();
		}
		else
		{
			setMode(INIT_SMK900);
			setState(INIT_GATEWAY_REGISTER);
		}
	#endif
	}


	//Timeout manager
	if (getState() != IDLE)
	{

		if (isOtaTimeoutExpired())
		{

			PRTLN("\n\nOTA Engine timeout\n\n");
			Serial.print("current_mode: ");
			Serial.println(current_mode);
			Serial.print("current_state: ");
			Serial.println(current_state);

			setState(IDLE);
			firmware.close();
			otaResult = "timeout transfert at " + getTimeFormated();
			enableAutomaticPolling();
			return false;
		}
	}

	return ret;

	// if timeout ota engine, softshutdown the ota engine
}
