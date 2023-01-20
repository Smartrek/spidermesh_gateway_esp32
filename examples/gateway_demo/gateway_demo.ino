/*
	Author: Rudy Bernard 5036989

	This code is an example of how SpiderMesh network can be used to get data from sensors in the field. 
	It is polling in a roundrobin style every node listed in the file nodes.json. It is then extracting the raw data in readable format according to the type of sensors.
	a sensor type can be defined in the files found in the type folder. Each subfolder define a type of nodes and the parser.json files define how to convert it in readable format.
	To make it even simplier for the developper, callback function are define in the setup function which allow automatic management of the data extraction when a packet is received.
	For this example, the data is simply written on the serial port in a readable format. 
	
	
	cbAutomaticPolling 
	This allow to extract data of the reponse rf payload of nodes. If node definitions have been created, it can extract rf payload into a json readable format
	For parameters of this function, a pointer of Node is available to know who trigger the callback function, the raw RF packet and if this is a success or timeout trigger.


	cbAutoRequestBuilder
	This allow to send a customizable request packet on the current automatic roundrobin polling node 
	For parameters of this function, there is a pointer of Node to know who trigger the callback function.
	The return packet will be append to the request packet of the virtual machine. For more infomation about the virtual machine, Please consult the SMK900 datasheet.

	cbWhenPacketReceived
	When a complet API packet is received, this function is call. This is specialy usefull when a End Of Broadcast packet is received. 


	NB: Data folder must be uploaded via the (Upload Filesystem Image) platformio button
*/ 
#include <Arduino.h>
#include <Spidermesh.h>



#define TEST_AUTOMATIC_NODE_POLLING
//#define TEST_REQUEST_BUILDER
//#define TEST_UNICAST_TO_NODE
#define TEST_BROADCAST_TO_ALL_NODE
//#define TEST_OTA_UPDATE
//#define TEST_LOAD_EXTERNAL_FILES_DEFINITION


Spidermesh smk900;

//Action can be done when a packet is receive if special operation is needed
void CallbackWhenPacketReceived(apiframe packet)
{
	//check if the packet received is a End Of Broadcast cycle (EOB)
	if(smk900.isEobPacket(packet))
	{
		//do special action here that need to be synchronized with mesh network speed.
	}
}


//Definition of the callback function when automatic polling is enabled
//when ever a node response is received or timeout, this function is call
void CallbackAutoPolling(mesh_t::iterator pNode, apiframe packet, bool success, String tag)
{
	//in case was unable to reach node, action can be done here
	//payload contain nothing if unable to reach node
	if(!success) 
	{
		Serial.print("Timeout: Unable to reach node: ");
		Serial.println(pNode->second.getMacAsString());
		return; //we quit here
	}
	//to know who trigger the callback	and what is his type	
	String node_mac = pNode->second.getMacAsString();
	Serial.printf("Response from MAC:  %s\n", node_mac.c_str());
	Serial.printf("Type: %s\n", pNode->second.type.c_str());
	printApiPacket(packet, "in: ");
	Serial.printf("Tag: %s\n", tag);

	//conversion of individual data from rf payload into JSON variables
	DynamicJsonDocument reply_json_doc(5000);
	JsonVariant reply_json = reply_json_doc.as<JsonVariant>();
	if(SmkParser::rfPayloadToJson(packet, tag, reply_json, pNode->second.type))
	{
		Serial.printf("tag: %s\n", tag.c_str());

		String prettyjson="";
		Serial.println("payload:");
		serializeJsonPretty(reply_json, prettyjson);
		Serial.println(prettyjson);
	}
}

#ifdef TEST_REQUEST_BUILDER
//Before every automatic polling request of nodes, RF payload can be customized with this function.
//if the function is not defined or if it return nothing, default status command will be sent
apiframe CallbackAutoRequestBuilder(mesh_t::iterator pNode)
{
	//add the byte 0x60 at the end of the VM request packet.
	//it will be send immediately after this call
	apiframe request_packet={0x60};
	return request_packet;
}
#endif

#ifdef TEST_LOAD_EXTERNAL_FILES_DEFINITION
// At the end of the initialisation of the smk900.begin, initialisation files can be fetch 
// from external source, instead of SPIFFS 
// If the following callback is define and set with smk900.setCallbackLoadExternalParamFiles
// the SPIFFS is avoided.
//
void CallbackLoadExternalFileDefinition()
{
	String mac_gateway = smk900.gateway->second.getMacAsString();
	Serial.print("Load list of gateway: ");
	Serial.println(mac_gateway);

	//--------------------------------------------------------
	// example of loading nodes list available and the types, 
	// it can be fetch from any kind database accessible with http or mqtt
	// you can use the mac gateway to identify the list of nodes and types to fetch
	//smk900.nodes.loadNodes("{\"0.0.172.51\":{\"name\":\"Office1\",\"type\":\"vacuum\"},\"0.0.172.52\":{\"name\":\"Office2\",\"type\":\"vacuum\"},\"0.0.64.62\":{\"name\":\"Office3\",\"type\":\"vacuum\"}}");

	//loading of all the type available
	//smk900.nodes.loadTypes("{\"vacuum\":{\"parser\":{\"status\":{\"params\":{\"rssi\":{\"pos\":[0,8]},\"vacuum\":{\"pos\":[8,16]},\"temperature\":{\"pos\":[24,8]},\"volt\":{\"pos\":[32,8]}}}}}}");

	//loading of one type at the time
	//smk900.nodes.addType("vacuum", "{\"parser\":{\"status\":{\"params\":{\"rssi\":{\"pos\":[0,8]},\"vacuum\":{\"pos\":[8,16]},\"temperature\":{\"pos\":[24,8]},\"volt\":{\"pos\":[32,8]}}}}}");
}
#endif

void setup()
{

	Serial.begin(115200);

	//init the spiffs file system that contain the nodes definition
	initDrive();

	//set the mesh network speed at those parameters
	int nb_hops = 8;
	int duty_cycle = 5;
	int nb_byte_per_packet = 20;


	//callback that will be call at every packet received
	smk900.setWhenPacketReceived(CallbackWhenPacketReceived);

	#ifdef TEST_AUTOMATIC_NODE_POLLING
	smk900.setCallbackAutomaticPolling(CallbackAutoPolling);
	#endif

	#if defined(TEST_REQUEST_BUILDER) && defined(TEST_AUTOMATIC_NODE_POLLING)
	smk900.setCallbackAutoRequestBuilder(CallbackAutoRequestBuilder);
	#endif


	//will start autopolling nodes listed in nodes.json file
	//if will send RF command to the virtual machine of each nodes and will use cbAutomaticPolling when contacted or unable to reach node
	#if !defined(TEST_OTA_UPDATE) && defined(TEST_AUTOMATIC_NODE_POLLING)
	smk900.enableAutomaticPolling();
	#endif

	#ifdef TEST_LOAD_EXTERNAL_FILES_DEFINITION
	smk900.setCallbackLoadExternalParamFiles(CallbackLoadExternalFileDefinition);
	#endif


	//to check or not all transaction with smk900 module
	smk900.setApiMsgFlag(true,true,false); 

	//radio will have already started to synch nodes at this point, but at the speed of mesh saved in eeprom
	smk900.begin(nb_hops,duty_cycle,nb_byte_per_packet);


}

uint64_t last_broadcast=millis();
uint64_t last_command=millis();
bool update_once = false;

void loop()
{
	#ifdef TEST_OTA_UPDATE
	//#define TEST_LOCAL
	#define TEST_REMOTE

	if(smk900.isMode(READY) && smk900.isState(IDLE) && !update_once)
	{
		update_once = true;
		smk900.nodes.resetOtaActiveFlags();
		
		#ifdef TEST_LOCAL
		uint32_t gateway;
		macStringToInt("0.0.64.61",&gateway);
		smk900.nodes.enableOta(gateway);
		#endif

		//update all node including the gateway radio
		#ifdef TEST_REMOTE
		for(auto n:smk900.nodes.pool)
		{
			if(n.second.local) continue; //for now, gateway and node are updated separately
			Serial.print("Update of: ");
			Serial.println(n.second.getMacAsString());
			smk900.nodes.enableOta(n.first);
		}
		#endif
		smk900.firmware.filename = "/test.uf2";
		smk900.launchUpdateOtaEngine();

	}
	#endif
	

	#ifdef TEST_BROADCAST_TO_ALL_NODE
	//broadcast to all node variable 1 every 15min, will be sent at the next available broadcast cycle
	if(millis()-last_broadcast > 15000){
		Serial.println("Broadcast message");
		smk900.addBroadcastPacket({0x01,0x02,0x03}); 
		last_broadcast = millis();
	}
	#endif

	#ifdef TEST_UNICAST_TO_NODE
	//unicast to single node every 1min, will be sent at the next available broadcast cycle
	if(millis()-last_command > 10000){
		Serial.println("Unicast message");

		//find a node by is mac in string format, if node exist, send a OTA command to this node
		auto p = smk900.find("0.0.96.36");
		if(p != smk900.nodes.pool.end())
		{
			//build the api frame to send a command, when result will be received, it will triger the ExpectCallback function
			apiframe vm_command = smk900.apiPacket(p, SMK_VM_EXEC, {0x60}, REMOTE);
			smk900.addWriteExpect(p,vm_command, "status", ExpectCallback([](mesh_t::iterator pNode, apiframe packet, bool success, String tag) -> void
			{
				if(!success)
				{
					Serial.print("tag: ");
					Serial.print(tag);
					Serial.println(" request failed");
					return;
				}
				//to know who trigger the callback	and what is his type	
				String node_mac = pNode->second.getMacAsString();
				Serial.printf("\n---------------\n");
				Serial.printf("Loop response from MAC:  %s\n", node_mac.c_str());
				Serial.printf("Type: %s\n", pNode->second.type.c_str());
				printApiPacket(packet, "in: ");
			}));
		}
		last_command = millis();
	}
	#endif
	
	//user task
	delay(1000);
}


