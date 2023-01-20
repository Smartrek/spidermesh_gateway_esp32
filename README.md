# Spidermesh


## Introduction
This repository is a gateway state machine for the spidermesh network. It is polling in a roundrobin style every node listed in the file nodes.json. It is possible as well to extract the raw data in readable format (JSON) according to the type of sensors.	To do that, sensor type can be defined in the file the type folder. Each subfolder define a type of nodes and the parser.json files define how to convert it in readable format. 

## Board Supported
ESP32 for now
	
### important note
CTS signal of uart was not supported on old version of arduino platform. This library was tested on Espressif 32 platform 5.1.1 and previous version may not work. There is work around but not as pretty as for this platform.

## How to use the library
Define the callback functions that are enumerated below as you see fit for your application. They are normally defined in the setup function. Then, a call the "enableAutomaticPolling()" function allow the polling of nodes defined. After all have been set, call the begin() function with the speed of the mesh as parameters.

In the loop function or inside callback function, if you want to make custom api call, you can call the addWriteExpect function. 

## Definition of the lib and variables

	#include <Spidermesh.h>
	Spidermesh smk900;

## Basic functions

### begin

You can define the speed of the Spidermesh network with this function. 

	int nb_hops = 8;
	int duty_cycle = 5;
	int nb_byte_per_packet = 20;
	smk900.begin(nb_hops,duty_cycle,nb_byte_per_packet);


### enableAutomaticPolling

When smk900.cbAutomaticPolling have been define, call this function to enable the automatic polling of node define in nodes.json

	smk900.enableAutomaticPolling();

## Callback functions

Use those function to set the callback describe bellow

	smk900.setWhenPacketReceived(CallbackWhenPacketReceived);

	smk900.setCallbackAutomaticPolling(CallbackAutoPolling);

	smk900.enableAutomaticPolling();

### Callback for automatic node polling

Callback function use for the automatic polling of all nodes define in nodes.json.  For parameters of this function, there is the node pointer to know who trigger the function, the packet received, if the response have been succesfully received or if the timeout have been triggered. It is possible as well to parse the raw data here if node type have been define. Automatic polling must be enable.

	void CallbackAutoPolling(mesh_t::iterator pNode, hexPacket_t packet, bool success, String topic)
	{
		//in case was unable to reach node, action can be done here
		//payload contain nothing if unable to reach node
		if(!success) 
		{
			Serial.println("Timeout: Unable to reach node");
			return; //we quit here
		}
		//to know who trigger the callback	and what is his type	
		String node_mac = pNode->second.getMacAsString();
		Serial.printf("Response from MAC:  %s\n", node_mac.c_str());
		Serial.printf("Type: %s\n", pNode->second.type.c_str());
		printApiPacket(packet, "in: ");

		//conversion of individual data from rf payload into JSON variables
		DynamicJsonDocument reply_json_doc(5000);
		JsonVariant reply_json = reply_json_doc.as<JsonVariant>();
		if(SmkParser::rfPayloadToJson(packet, topic, reply_json, pNode->second.type))
		{
			Serial.printf("topic: %s\n", topic.c_str());

			String prettyjson="";
			Serial.println("payload:");
			serializeJsonPretty(reply_json, prettyjson);
			Serial.println(prettyjson);
		}
	}

### Callback for received API packet

Callback function trigger when a Spidermesh API packet is received. For parameters of this function, there is the packet received. The End Of Broadcast packet do not trigger this function. 

	void CallbackWhenPacketReceived(hexPacket_t packet)
	{
		//check if the packet received is a End Of Broadcast cycle (EOB)
		if(smk900.isEobPacket(packet))
		{
			//do special action here that need to be synchronized with mesh network speed.
		}
	}

### Callback for customizable polling request 
Callback function to send a customizable request packet on the current automatic roundrobin polling node. For parameters of this function, there is a pointer of Node to know who trigger the callback function. The return packet will be append to the request packet of the virtual machine. For more infomation about the virtual machine, Please consult the SMK900 datasheet. Automatic polling must be enable.

	hexPacket_t CallbackAutoRequestBuilder(mesh_t::iterator pNode)
	{
		//add the byte 0x60 at the end of the VM request packet.
		//it will be send immediately after this call
		hexPacket_t request_packet={0x60};
		return request_packet;
	}

### Unicast API call
The function addWriteExpect() will add the api a packet into a queue and it will be send at the next broadcast cycle. It will pause temporarerly the automatic polling cycle to allow the custom api call.

	auto p = smk900.find("0.0.96.36");
	if(p != smk900.nodes.pool.end())
	{
		//build the api frame to send a command, when result will be received, it will triger the ExpectCallback function
		hexPacket_t vm_command = smk900.apiPacket(p, SMK_VM_EXEC, {0x60}, REMOTE);
		smk900.addWriteExpect(p,vm_command, "status", ExpectCallback([](mesh_t::iterator pNode, hexPacket_t packet, bool success, String topic) -> void
		{
			if(!success)
			{
				Serial.print("topic: ");
				Serial.print(topic);
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

## Node definition
for the automatic polling function to work, it is necessary to define which node are available to poll and what data are expected to each of the node. For this reason, there is one files that defines which node are available and multiples files to allow the parsing machine to convert the raw data of the payload into a readable format

### nodes.json
This file define the node available to poll and what type they are

	{
		"0.0.96.36": {
			"name": "Factory",
			"type": "plc"
		}
	}

### type/(name of type)/parser.json
For each definition of node type there is a folder created with the name of the type. Inside of it, there is a file name parser.json. It contain the definition of the payload of the node to allow the parsing machine to extract the data. Payload be be seen as a 128bits and multiple kind of payload are possible named as topic. The minimum topic required is the "status". Inside the element each variables of the nodes type are define using the following convention

	"pos":[(start position), (bit length)]

Here a definition of a basic node that read some value inside a payload

	{
		"parser":
		{
			"status":{
				"params":{
					"rssi":{"pos":[0,8]},
					"volt":{"pos":[8,8]},
					"in0":{"pos":[16,1]},
					"in1":{"pos":[17,1]},
					"in2":{"pos":[18,1]},
					"in3":{"pos":[19,1]},
					"in4":{"pos":[20,1]},
					"in5":{"pos":[21,1]}
					"position":{"pos":[22,12], "gain":0.01},
					"temperature":{"pos":[24,12], "div":40.95, "offset":-40.0, "units":"C"},
					"humidity":{"pos":[36,12], "div":40.95, "units":"%"},
				}
			}
		}
	}

### File system

NB: Data folder must be uploaded via the (Upload Filesystem Image) platformio button

For more information on SMK900 module go to https://www.smartrek.io/download/SMK900_Portia_Summary_V6.pdf
    
