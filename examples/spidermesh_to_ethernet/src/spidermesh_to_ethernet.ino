#include "bridge.h"
#include <Arduino.h>
#include <Spidermesh.h>
#include <ETH.h>
#include "esp_task_wdt.h"

Spidermesh smk900;
const int server_serial_port = 5556;
Bridge bridge(server_serial_port);
bool prevPollMode;

#define ETH_CLK_MODE ETH_CLOCK_GPIO0_OUT
// Pin# of the enable signal for the external crystal oscillator (-1 to disable for internal APLL source)
#define ETH_POWER_PIN 4
// Type of the Ethernet PHY (LAN8720 or TLK110)
#define ETH_TYPE ETH_PHY_LAN8720
// I²C-address of Ethernet PHY (0 or 1 for LAN8720, 31 for TLK110)
#define ETH_ADDR 0
// Pin# of the I²C clock signal for the Ethernet PHY
#define ETH_MDC_PIN 23
// Pin# of the I²C IO signal for the Ethernet PHY
#define ETH_MDIO_PIN 18

IPAddress eth_add(192,168,0,30);
IPAddress eth_gat(192,168,0,1);
IPAddress eth_msk(255,255,255,0);
IPAddress eth_dns1(192,168,0,1);
IPAddress eth_dns2(8,8,8,8);

void WhenPacketReceived(apiframe packet);
bool addApiPacket(apiframe pkt);
portMUX_TYPE mmux = portMUX_INITIALIZER_UNLOCKED;


/**
 * @brief Management of event from the Wifi and Ethernet module
 * 
 * @param event code of the event
 */
void WiFiEvent(WiFiEvent_t event)
{

    switch (event)
    {
    case SYSTEM_EVENT_ETH_START:
        Serial.println("SYSTEM_EVENT_ETH_START");
        //set eth hostname here
        //ETH.setHostname("esp32-ethernet");
        break;
    case SYSTEM_EVENT_ETH_CONNECTED:
        Serial.println("ETH Connected");
        break;
    case ARDUINO_EVENT_ETH_GOT_IP:
            Serial.println("SYSTEM_EVENT_ETH_GOT_IP");
            Serial.print("ETH MAC: ");
            Serial.println(ETH.macAddress());
            Serial.print("IPv4: ");
            Serial.println(ETH.localIP());
            Serial.print("Subnet mask: ");
            Serial.println(ETH.subnetMask());
            Serial.print("Gateway IP: ");
            Serial.println(ETH.gatewayIP());
            if (ETH.fullDuplex())
            {
                Serial.println("FULL_DUPLEX");
            }
            Serial.print(ETH.linkSpeed());
            Serial.println("Mbps");
        break;
    case ARDUINO_EVENT_ETH_DISCONNECTED:
        Serial.println("ETH Disconnected");
        break;
    case ARDUINO_EVENT_ETH_STOP:
        Serial.println("ETH Stopped");
        break;

    case SYSTEM_EVENT_AP_START:
        Serial.println("SYSTEM_EVENT_AP_START");
    //case 18:
        //mbIP.killClient();
        //Serial.println("Will disconnect client");
        break;
    case SYSTEM_EVENT_WIFI_READY:
        Serial.println("SYSTEM_EVENT_WIFI_READY");
    //case 18:
        //mbIP.killClient();
        //Serial.println("Will disconnect client");
        break;
    default:
        Serial.print("============= DEFAULT EVENT =============== ");
        Serial.println(event);
        break;
    }
}

/**
 * @brief Configure the ethernet parameter
 * 
 */
void initEthernet()
{
    //ETH.begin always before ETH.config
	Serial.println("ETH BEGIN");
    ETH.begin(ETH_ADDR, ETH_POWER_PIN, ETH_MDC_PIN, ETH_MDIO_PIN, ETH_TYPE, ETH_CLK_MODE); 
	ETH.config(eth_add,eth_gat,eth_msk,eth_dns1,eth_dns2);
	Serial.println("ETH END");
}

void setup()
{
	Serial.begin(115200);
	Serial.println("--RESET--");

	//init the spiffs file system that contain the nodes definition
	delay(8000);
	initDrive();

	//set the mesh network speed at those parameters
	int nb_hops = 8;
	int duty_cycle = 5;
	int nb_byte_per_packet = 20;


	//callback that will be call at every packet received
	//smk900.setWhenPacketReceived(CallbackWhenPacketReceived);

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
	smk900.setApiMsgFlag(false,true,false); 

	//radio will have already started to synch nodes at this point, but at the speed of mesh saved in eeprom
	smk900.begin(nb_hops,duty_cycle,nb_byte_per_packet);
    
    // Bridge TCP to Serial API
	WiFi.onEvent(WiFiEvent);

	initEthernet();
    bridge.cbAddPacketToSmkTxBuffer = static_cast<bool(*)(apiframe)> (smk900.addApiPacketLowPriority);
	if(!smk900.show_apipkt_in)    		
        bridge.cbWhenSmkPacketReceived = static_cast<void(*)(apiframe, String)> (printApiPacket);
    
    bridge.cbConnecting= [](){
        Serial.println("Bridge connected");
        prevPollMode = smk900.getAutoPolling();
        smk900.setAutoPolling(false);        
    };
    bridge.cbDisconnecting= [](){
        Serial.println("Bridge disconnected");
        smk900.setAutoPolling(prevPollMode);            
    };

    smk900.cbWhenPacketReceived=WhenPacketReceived;
    bridge.init();

}

void loop()
{
	bridge.taskloop();
}

/**
 * @brief This function allow to put packet received from spidermesh network into a list.
 *        It will be processed in the taskloop function
 * 
 * @param packet complete api frame received from spidermesh
 */
void WhenPacketReceived(apiframe packet)
{
    if(smk900.isInitDone())
    {
     	//bridge.WhenNewSmkPacketRx(packet);
		if(bridge.state == CONNECTED)
        {
            taskENTER_CRITICAL(&mmux);
			bridge.packetList.push_back(packet);
            taskEXIT_CRITICAL(&mmux);
        }
    }  
}
