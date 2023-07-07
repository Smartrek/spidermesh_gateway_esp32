#ifndef HARDWARE_PROFILE_H
#define HARDWARE_PROFILE_H


//#define ESP32_GATEWAY
//#define ESP32_POE
//#define ES32_FEATHER
#define ESP32_SMG


#define THREAD_PRIORITY_SMK900 4
#define THREAD_DELAY_WINDOW 100

#define ETHERNET_ENABLE true
#define WIFI_ACCESS_POINT true

#define PRESET_20B 0x00
#define PRESET_72B 0x10

#define GATEWAY_STATUS_UPDATE_RATE_MS 60000
#define WATCHDOG_ESP true
#define RTC_INIT_REASON true
#define SMK900_ENABLED true



#define SHOW_MIN_DEBUG false
#define SHOW_WEB_SERVER_STATUS false
#define SHOW_EXPECT_EVENT false
#define SHOW_EXPECT_RESULT false
#define SHOW_RECEIVED_BYTE_FROM_SMK900 false


#define SHOW_TRANSMITED_PACKET_TO_SMK900 false

#define SHOW_AUTOMATIC_POLLING false
#define PRINT_EOB_AS_DOT false
#define SHOW_DEBUG_EXTRACT_DATA false
#define SHOW_DEBUG_DICTIONARY false
#define SHOW_DEBUG_DRIVE false
#define SHOW_LOAD_NODE_DEBUG true

#define RTC_ENABLED 0
#define SD_CARD_ENABLED 0

#define MQTT_PARSER_ENABLED true

#define MIN_WEBOTA_REQUIRED false

//-----------------------------------------------------

#define SLAVE_ID 1

//-----------------------------------------------------
#if defined(ESP32_GATEWAY)
//#define PORTIA_RESET 10
#define RX_PORTIA 36
#define TX_PORTIA 4

#if RTC_ENABLED != 0

#endif

#define ETH_CLK_MODE ETH_CLOCK_GPIO17_OUT
// Pin# of the enable signal for the external crystal oscillator (-1 to disable for internal APLL source)
#define ETH_POWER_PIN 5
// Type of the Ethernet PHY (LAN8720 or TLK110)
#define ETH_TYPE ETH_PHY_LAN8720
// I²C-address of Ethernet PHY (0 or 1 for LAN8720, 31 for TLK110)
#define ETH_ADDR 0
// Pin# of the I²C clock signal for the Ethernet PHY
#define ETH_MDC_PIN 23
// Pin# of the I²C IO signal for the Ethernet PHY
#define ETH_MDIO_PIN 18
//-----------------------------------------------------
#elif defined(ESP32_POE_ISO) || defined(ESP32_POE)
//#define PORTIA_RESET 10
#define RX_PORTIA 36

#define TX_PORTIA 4

#define CTS_PORTIA 33
#define RESET_PORTIA 32

#define BATTERY_MEASUREMENT_PIN 35
#define EXTERNAL_POWER_SENSE    39

#define ETH_CLK_MODE ETH_CLOCK_GPIO17_OUT
// Pin# of the enable signal for the external crystal oscillator (-1 to disable for internal APLL source)
#define ETH_POWER_PIN 12
// Type of the Ethernet PHY (LAN8720 or TLK110)
#define ETH_TYPE ETH_PHY_LAN8720
// I²C-address of Ethernet PHY (0 or 1 for LAN8720, 31 for TLK110)
#define ETH_ADDR 0
// Pin# of the I²C clock signal for the Ethernet PHY
#define ETH_MDC_PIN 23
// Pin# of the I²C IO signal for the Ethernet PHY
#define ETH_MDIO_PIN 18

#elif defined(ESP32_SMG)
	//PORTIA define
	#define RX_PORTIA 34
	#define TX_PORTIA 32
	#define CTS_PORTIA 33
	#define RESET_PORTIA 5

	#define BATTERY_MEASUREMENT_PIN 35
	#define EXTERNAL_POWER_SENSE    39

	#if ETHERNET_ENABLE
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
	#endif
#endif


#define WATCHDOG_SMK900_ENABLE true
#define WIFI_ACCESS_POINT true


#define TCP_BRIDGE_AS_SERVER 1
#define TCP_BRIDGE_AS_CLIENT 2
#define TCP_SERIAL_BRIDGE TCP_BRIDGE_AS_CLIENT

#define NB_EOB_TO_LET_COUNTERMEASURE_WORK 1
#define NB_EOB_AT_FULL_SPEED 200

#define SIZE_OF_DYNAMIC_JSON_FILE 30000
#define SIZE_OF_DYNAMIC_JSON_TYPE 30000

#endif //HARDWARE_PROFILE_H