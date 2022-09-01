#include <smk900.h>
#include "esp32-hal-uart.h"
#include "soc/uart_struct.h"

#if MODBUS_REGISTER_ENABLED
#include "ModbusHelper.h"
#endif

/*
#ifdef SIMULATION == SIMULATION_END
    #define SIMULATION_BULKUPLOAD               false
    #define SIMULATION_GETMISSINGFLAGS          false
    #define SIMULATION_PRUNE_VALID_PAGES    false
    #define SIMULATION_CHECK_IF_CRC_OK          false
    #define SIMULATION_SEND_META_DATA           false
    #define SIMULATION_RESET_NODE_ON_SEEK       true
    #define SIMULATION_SEND_MAGICWORD           true

#endif
*/
#ifdef PORTIA_KLIK_THREADSAFING_ENABLED
    #define PORTIA_SYNCHRONIZE() std::unique_lock<std::recursive_mutex> lock(mPortiaMutex)
#else
    #define PORTIA_SYNCHRONIZE() do{}while(0)
#endif

DynamicJsonDocument type_json(SIZE_OF_DYNAMIC_JSON_FILE);


mesh_t::iterator Smk900::gateway;
mesh_t Smk900::gateway_boot;
std::list<ExpectAnswer> Smk900::listOfExpectedAnswer;
bool Smk900::gatewayMacAddressIsReceived=false;

SmkList Smk900::nodes;
ExpectCallback Smk900::cbAutomaticPolling;
ExpectCallback Smk900::cb_automatic_polling_failed;
RequestBuilderCallback Smk900::cbAutoRequestBuilder;
unsigned char Smk900::state_serial;
mesh_t::iterator Smk900::pCurrentNode;
bool Smk900::_auto_polling_mode;
String Smk900::polling_mode;


uint16_t Smk900::length_packet;
unsigned char Smk900::idx_buf;

int Smk900::eob_cnt;
firmware_t Smk900::firmware;

hexPacket_t Smk900::current_packet;
std::function<void(hexPacket_t)> Smk900::cbWhenPacketReceived;
std::function<bool(bool)> Smk900::WhenEobIsReceived;
uint32_t Smk900::focus_node;
unsigned long Smk900::focus_node_start_time;
uint16_t Smk900::_duration_focus_on_node_polling;
bool Smk900::_otaPacketInCycle;

//std::vector <mesh_t::iterator> Smk900::otaList;

std::list<String> Smk900::terminalBuffer;
long Smk900::timeout_expect;
unsigned long Smk900::previousMillisExpectPacketReceived;
bool Smk900::watchdog_serial_parser;
unsigned long Smk900::previousMillisPacketParser;
uint8_t Smk900::idx_polling_cnt;
portMUX_TYPE Smk900::mmux;

WriteAndExpectList_t Smk900::writeAndExpectList;
commandList_t Smk900::lowPriorityFifoCommandList;
commandList_t Smk900::highPriorityFifoCommandList;

bool Smk900::show_eob = false;
bool Smk900::show_apipkt_in = false;
bool Smk900::show_apipkt_out = false;
    
HardwareSerial Smk900::smkport(2);

Smk900::Smk900()
{
    pinMode(CTS_PORTIA, INPUT);

    digitalWrite(RESET_PORTIA,HIGH);
    pinMode(RESET_PORTIA, OUTPUT);
    #if PORTIA_RESET_WHEN_ESP32_REBOOT
    digitalWrite(RESET_PORTIA,LOW);
    delay(500);
    digitalWrite(RESET_PORTIA,HIGH);
    #endif
    state_serial = STATE_SOF;
    timeout_expect = 10000;
    previousMillisExpectPacketReceived = 0;
    watchdog_serial_parser = false;
    previousMillisPacketParser = 0;
    idx_buf = 0;


    cbAutoRequestBuilder = RequestBuilderCallback([](mesh_t::iterator pNode) -> hexPacket_t{hexPacket_t x; return x;});

    //Serial.println("Serial port configured");

    length_packet = 0;
    idx_polling_cnt=0;

    cbWhenPacketReceived = [](hexPacket_t){};


}

bool Smk900::addNewNode(JsonVariant pSource)
{
    bool ret = true;
    JsonObject pay_json = pSource.as<JsonObject>();
    for(auto n:pay_json)
    {
        String mac = n.key().c_str();
        if (!isValidMac(mac))
        {
            Serial.println("ERROR - packet do not contain a valid 'mac' key");
            break;
        }
        JsonObject i_node = n.value().as<JsonObject>();

        String type;
        if(!i_node.containsKey("type"))
        {
            Serial.println("ERROR - packet do not contain a type");
            break;
        }

        //TODO: validate type available

        type = i_node["type"].as<String>();

        String group="no_group";
        if(i_node.containsKey("group")) group = i_node["group"].as<String>();
        if(i_node.containsKey("trail")) group = i_node["trail"].as<String>();
        
        String name = "no_name";
        if(i_node.containsKey("name")) name = i_node["name"].as<String>();

        uint16_t srate = (i_node.containsKey("srate")) ? i_node["srate"].as<uint16_t>():0; 
        bool enabled = (i_node.containsKey("enabled")) ? i_node["enabled"].as<bool>():true; 
        bool local = (i_node.containsKey("local")) ? i_node["local"].as<bool>():false; 

        //add node to the database
        uint32_t mac_int;
        macStringToInt(mac,&mac_int);
        if(!nodes.add(mac_int, group, REMOTE, name, type, enabled, srate)) ret = false;
        
    }
    
    if(!nodes.writeNodeListToFile()) return false;
    nodes.loadNodes();//to redo the code indexing and init of variables
    listOfExpectedAnswer.clear();
    findNext(true,true);
    

    return ret;
}

bool Smk900::init()
{

    Serial.print("Serial port... ");
    


    // CONFIG SERIAL PORT WITH CTS HARDWARE FLOW CONTROL
    // HardwareSerial smkport(2);
    // HardwareSerial* port_smk=&smkport;

    // ATTACH portia CTS to U2CTS_in at digital 33
    // see table 19 page 54 esp32 technical reference manual 
    // choose the function attach to the digital pin
    // here this is U2CTS_in so 199 attach to pin 33
    // last parameter is for inversion of CTS signal which not in this case so false
    // https://www.espressif.com/sites/default/files/documentation/esp32_technical_reference_manual_en.pdf


    // ENABLE AUTOMATIC HARDWARE FLOW CONTROL
    // this is in the register conf0 of the serial port. (see register 14.9 page 362)
    // this register is the second parameter of the begin function that set normaly only parity and payload size
    // in the include file is a struct of this register.
    // use it to enable the TX flow control bit
    // https://www.espressif.com/sites/default/files/documentation/esp32_technical_reference_manual_en.pdf
    // uart_dev_t uart_registers;
    // uart_registers.conf0.val=SERIAL_8N1;
    // uart_registers.conf0.tx_flow_en=1;

    smkport.begin(PORTIA_BAUDRATE, SERIAL_8N1, RX_PORTIA, TX_PORTIA);
    pinMatrixInAttach(33, 199, false);
    smkport.setPins(RX_PORTIA,TX_PORTIA,CTS_PORTIA,-1);
    smkport.setHwFlowCtrlMode(HW_FLOWCTRL_CTS);


    //Sderial2.begin(PORTIA_BAUDRATE, uart_registers.conf0.val, RX_PORTIA, TX_PORTIA);
    //Sderial.println(" OK");


    //PRTLN(" before init smk900"); delay(10);


    //Serial.println("Init portia done");
    return true;
}



void Smk900::reset()
{
    digitalWrite(RESET_PORTIA,LOW);
    delay(1000);
    digitalWrite(RESET_PORTIA,HIGH);
    delay(100);
}


void Smk900::write(uint32_t c)
{
    //PORTIA_SYNCHRONIZE();
    smkport.write(c);
}
void Smk900::write32(uint32_t c)
{
    u_bytes u;
    u.uint32b = c;
    //PORTIA_SYNCHRONIZE();
    smkport.write(u.uint8b[0]);
    smkport.write(u.uint8b[1]);
    smkport.write(u.uint8b[2]);
    smkport.write(u.uint8b[3]);
}

void Smk900::SaveGatewayMacAddress(hexPacket_t packet)
{
    //Serial.println("EOB packet");
    IPAddress RepliedMacAddress = {packet[10], packet[9], packet[8], packet[7]}; 
    #if SHOW_EXPECT_EVENT || SHOW_MIN_DEBUG
    Serial.println("Gateway MAC address: " + RepliedMacAddress.toString());
    #endif

     SmkNode g;
    g.mac.bOff[byte0] = RepliedMacAddress[3];
    g.mac.bOff[byte1] = RepliedMacAddress[2];
    g.mac.bOff[byte2] = RepliedMacAddress[1];
    g.mac.bOff[byte3] = RepliedMacAddress[0];
    /*
    if(gateway.size()==0)
    {   
        gateway.insert(std::make_pair(a,g));
    }
    */
    gatewayMacAddressIsReceived=true;
}

void Smk900::loadList()
{
    JsonVariant type_json_main_file = type_json.as<JsonVariant>();
    

    nodes = SmkList("/nodes.json", type_json_main_file);
	type_json.shrinkToFit();


	Serial.print("Size type_json is: ");
	Serial.println(type_json.memoryUsage());    

}

hexPacket_t Smk900::localSetRegister(uint8_t offset_register, uint8_t size_register, uint8_t *content)
{
    //PORTIA_SYNCHRONIZE();
    uint8_t cmd[20] = {0xFB, 0, 0, 4, 0, offset_register, size_register};
    memcpy(cmd+7, content, size_register);
    uint8_t len = size_register + 4;
    cmd[1] = len;

    hexPacket_t ret;
    for(int i=0; i<len+3; i++)
    {
        ret.push_back(cmd[i]);
    } 
    return ret;
}

hexPacket_t Smk900::localTransfertConfigFromRAMBUFF(uint8_t location)
{
    //PORTIA_SYNCHRONIZE();
    uint8_t cmd[5] = {0xFB, 2, 0, 0x0B, location};

    hexPacket_t ret;

    for(int i=0; i<5; i++)
    {
        ret.push_back(cmd[i]);
    } 
    return ret;
}

hexPacket_t Smk900::setDyn(uint8_t po, uint8_t pi,uint8_t hop, uint8_t rdx, uint8_t rde, uint8_t duty)
{
    //PORTIA_SYNCHRONIZE();
    uint8_t cmd[10] = {0xFB, 7, 0, 0x0A, po, pi, hop, rdx, rde, duty};

    hexPacket_t ret;
    for(int i=0; i<10; i++)
    {
        ret.push_back(cmd[i]);
    } 
    return ret;   
}

hexPacket_t Smk900::requestMacAddress()
{
    //PORTIA_SYNCHRONIZE();    
    Serial.println("==Request Gateway Mac Address");
    uint8_t cmd[20] = {0xFB, 4, 0, 3, 2,0,8};

    hexPacket_t ret;
    for(int i=0; i<7; i++)
    {
        ret.push_back(cmd[i]);
    } 
    return ret;   
}

//-----------------------------------------------------------------------------------------------
void Smk900::task()
{
    //manage serial port reception
    if(!parseReceivedData()) delay(10);

    //If the fifo is holding message
    if(!isMessageStackEmpty())
    {
        
        hexPacket_t cmd = checkNextPacketToSend();
        if(cmd.size() >3 )
        {
            //Serial.println("messge stack sent");
            if(cmd[3] != PACKET_TXAIR_CMD_WRAPPER) _otaPacketInCycle = sendNextPacketBuffered();
            //delay(50);
        }
    }

}

//------------------------------------------------------------------------------------------------
void Smk900::initWatchdogParser()
{
    watchdog_serial_parser = true;
    previousMillisPacketParser = millis();
    //LedRemoteOn();
}

//------------------------------------------------------------------------------------------------
void Smk900::disableWatchdogParser()
{
    watchdog_serial_parser = false;
    //LedRemoteOff();
}


//------------------------------------------------------------------------------------------------
bool Smk900::parseReceivedData()
{
    bool ret = false;
    if (smkport.available())
    {
        ret = true;
        unsigned char rx = smkport.read();
        //Serial.write(state_serial); //debug line
        //Serial.write(rx);

      #if SHOW_RECEIVED_BYTE_FROM_SMK900
        char result2[10];
        sprintf(result2, "%02X ", rx);
        Serial.print(result2);
      #endif

        switch (state_serial)
        {
            case STATE_SOF:
                if (rx == 0xFB)
                {
                    state_serial = STATE_LENGTH;
                    idx_buf = 0;
                    length_packet = 0;
                    current_packet.clear();
                    current_packet.push_back(rx);
                    idx_buf++;
                    initWatchdogParser();
                }
                break;
            case STATE_LENGTH:
                if (length_packet == 0){
                    current_packet.push_back(rx);
                    idx_buf++;
                    length_packet += rx;
                }
                else
                {
                    current_packet.push_back(rx);
                    idx_buf++;
                    length_packet += rx*256;
                    state_serial = STATE_DATA; 
                }
                break;
            case STATE_DATA:
                //read serial port char
                current_packet.push_back(rx);
                idx_buf++;

                //Lenght of packet reach, packet ready to parse
                if (idx_buf >= length_packet+3)
                {
                    disableWatchdogParser();
                    state_serial = STATE_PACKET;
                    #if SHOW_RECEIVED_BYTE_FROM_SMK900
                        Serial.println();
                    #endif

                    CheckIfAnswerWasExpectedAndCallSuccessFunction(current_packet);
                    cbWhenPacketReceived(current_packet);
                    //IS EOB
                    if(isEobPacket(current_packet))
                    {
                        eob_cnt++; //counter manage at higher level
                      #if PRINT_EOB_AS_DOT
                        Serial.print(".");
                      #endif                        

                        _otaPacketInCycle = false;

                        CheckExpectTimeout();
                        WhenEobIsReceived(true);

                        if(show_eob)printApiPacket(current_packet, PREFIX_IN);
                    
                        delay(100);
                    }
                    else if(show_apipkt_in)printApiPacket(current_packet, PREFIX_IN);

                  #if TERMINAL_ENABLED
                    AddToTerminalBuffer("in :", &current_packet);
                  #endif                    

                    state_serial = STATE_SOF;
                }
                break;

            default:
                state_serial = STATE_SOF;
                break;
        }
    }


    //if a message is currently beeing readed, watchdog will be activated at first character received
    //after a time without a complete packet, buffer will be flushed to get the next one.
    //this is a simple way to parse message and if parsing machine is lost.
    if (watchdog_serial_parser)
    {
        unsigned long currentMillis = millis();
        if (currentMillis - previousMillisPacketParser >= timeout_parser)
        {
            // save the last time you blinked the LED
            previousMillisPacketParser = currentMillis;
            idx_buf = 0;
            length_packet = 0;
            disableWatchdogParser();
            //LedRemoteOn();
        }
    }
    return ret;
}


//------------------------------------------------------------------------------------------------
void Smk900::parseApiFromHost(uint8_t* buf, uint16_t len)
{
    uint16_t id_pkt = 0;
    uint8_t state_pkt = STATE_SOF;
    static uint16_t len_pkt;
    uint16_t last_idx_packet = 0;
    commandList_t cmd_list_ok;
    hexPacket_t cmd_pkt;


    while(id_pkt < len)
    {
        switch (state_pkt)
        {
            case STATE_SOF:
                if (buf[id_pkt] == 0xFB)
                {
                    state_pkt = STATE_LENGTH;
                    len_pkt = 0;
                    cmd_pkt.push_back(buf[id_pkt]);
                }
                id_pkt++;
                break;
            case STATE_LENGTH:
                if (len_pkt == 0){
                    cmd_pkt.push_back(buf[id_pkt]);
                    len_pkt += buf[id_pkt];
                }
                else
                { 
                    cmd_pkt.push_back(buf[id_pkt]);
                    len_pkt += ((uint16_t)buf[id_pkt])*256;
                    state_pkt = STATE_DATA; 
                }
                id_pkt++;
                break;
            case STATE_DATA:
                //read serial port char
                cmd_pkt.push_back(buf[id_pkt]);
                id_pkt++;
                
                //Lenght of packet reach, packet ready to parse
                if (id_pkt-last_idx_packet >= len_pkt+3)
                {
                    last_idx_packet= id_pkt;
                    addApiPacketLowPriority(cmd_pkt);

                    //Serial.println("  will be sent");
                    cmd_pkt.clear();
                    state_pkt = STATE_SOF;
                }
                break;

            default:
                state_pkt = STATE_SOF;
                break;
        }
    }

}





//-------------------------------
String Smk900::sendCommand(hexPacket_t cmd)
{
    for (int i = 0; i < cmd.size(); i++){
        write(cmd[i]);
    }


    String cmd_out = hexPacketToAscii(cmd);
    #if SHOW_TRANSMITED_PACKET_TO_SMK900
    printApiPacket(cmd, "out: ");    
    //Serial.println(cmd_out);
    #endif
    #if TERMINAL_ENABLED
    AddToTerminalBuffer("out:", &cmd);
    #endif

    return cmd_out;
}

void Smk900::AddToTerminalBuffer(String head, hexPacket_t *cmd)
{
    terminalBuffer.push_back(head +  hexPacketToAscii(*cmd));

    if (terminalBuffer.size() > NB_PACKET_TERMINAL)
        terminalBuffer.pop_front();    


}

//------------------------------------------------------------------------------------------------
// This function will poll the next on demand command
// It is taking the place of the normal base polling list
// It should be send after receiving an end of broadcast marker
//
bool Smk900::sendNextPacketBuffered()
{
    bool ret = false;

    if (writeAndExpectList.size()>0 || lowPriorityFifoCommandList.size() > 0 || highPriorityFifoCommandList.size() > 0)
    {
        //send the command
    #if SHOW_TRANSMITED_PACKET_TO_SMK900 && 0
        Serial.print("on demand cmd sent: ");
    #endif
        hexPacket_t cmd;
        if(highPriorityFifoCommandList.size() > 0)
        {
            cmd = highPriorityFifoCommandList.front();
            highPriorityFifoCommandList.pop_front();
            sendCommand(cmd);
        }
        else if(lowPriorityFifoCommandList.size() > 0)
        {
            cmd = lowPriorityFifoCommandList.front();
            lowPriorityFifoCommandList.pop_front();
            sendCommand(cmd);
        }
        else if(writeAndExpectList.size()>0)
        {
            //Serial.printf("writeAndExpectList.size():%d\r\n",writeAndExpectList.size());
            MeshRequest_t req = writeAndExpectList.front();
            writeAndExpectList.pop_front();
            WriteAndExpectAnwser(req.pNode, req.payload, PACKET_VM_RESP, req.topic, req.callback);
        }
        else 
        
        ret = true;
    }


    return ret;
}

unsigned int Smk900::toInt(byte c)
{
    if (c >= '0' && c <= '9')
        return c - '0';
    if (c >= 'A' && c <= 'F')
        return 10 + c - 'A';
    if (c >= 'a' && c <= 'f')
        return 10 + c - 'a';
    return -1;
}




bool Smk900::addApiPacketLowPriority(String asciiCommand)
{

    hexPacket_t od_pkt = convertAsciiTohexCommand(asciiCommand.c_str());
    //printApiPacket(od_pkt);
    addApiPacketLowPriority(od_pkt);
    return true;
}

bool Smk900::addApiPacketLowPriority(uint8_t* buffer, int size)
{
    hexPacket_t buf_hex;
    for(int i=0; i<size;i++)
    {
        buf_hex.push_back(buffer[i]);
    }
    addApiPacketLowPriority(buf_hex);
    return true;
}
/*
bool Smk900::addApiPacketLowPriority(hexPacket_t hcmd)
{
    lowPriorityFifoCommandList.push_back(hcmd);
    return true;
}
*/

bool Smk900::addApiPacketLowPriority(hexPacket_t hcmd)
{
    if(lowPriorityFifoCommandList.size() <= PORTIA_FIFO_LOW_PRIORITY_SIZE)
    {
#if ON_DEMAND_COMMAND_MUST_BE_DIFFERENT_FROM_THE_LAST_ONE == 1
        if(hcmd != lowPriorityFifoCommandList.back())
        {
            lowPriorityFifoCommandList.push_back(hcmd);
            return true;
        }
        else
        {
            Serial.println("Exact same command is present in the list so we discard the newest one");
        }
#else
        lowPriorityFifoCommandList.push_back(hcmd);
#endif
    }
    else
    {
        Serial.println("maximum FIFO size reached!");
    }
    

    return false;
}


bool Smk900::addApiPacketHighPriority(hexPacket_t hcmd)
{
    if(highPriorityFifoCommandList.size() <= PORTIA_FIFO_HIGH_PRIORITY_SIZE)
    {
    #if ON_DEMAND_COMMAND_MUST_BE_DIFFERENT_FROM_THE_LAST_ONE == 1
        if(hcmd != highPriorityFifoCommandList.back())
        {
            highPriorityFifoCommandList.push_back(hcmd);
        }
    #else
        highPriorityFifoCommandList.push_back(hcmd);
    #endif
        return true;
    }
    return false;
}



void Smk900::WriteAndExpectAnwser(mesh_t::iterator pNode, hexPacket_t request, uint8_t packet_type, String topic, ExpectCallback cb)
{
    uint max_retry = DEFAULT_MAX_TRY_TO_SEND;
    hexPacket_t expectPayload;
    int16_t size = -1;

    WriteAndExpectAnwser(pNode,request,packet_type,max_retry,expectPayload, size, topic, cb);
}
void Smk900::WriteAndExpectAnwser(mesh_t::iterator pNode, hexPacket_t request, uint8_t packet_type, uint8_t max_retry, String topic,ExpectCallback cb)
{
    hexPacket_t expectPayload;
    int16_t size = -1;

    WriteAndExpectAnwser(pNode,request,packet_type,max_retry,expectPayload, size, topic, cb);
}

void Smk900::WriteAndExpectAnwser(  mesh_t::iterator pNode, 
                                    hexPacket_t request, 
                                    uint8_t packet_type,                                      
                                    uint8_t max_retry, 
                                    hexPacket_t expectPayload, 
                                    int16_t size, 
                                    String topic,
                                    ExpectCallback cb)
{
  #if SHOW_EXPECT_EVENT
    Serial.println("SHOW_EXPECT_EVENT");
  #endif
    
    //check if not already exist
    bool found = false;
    for (auto i:listOfExpectedAnswer)
    {
        if ( i._pNode == pNode && i._request == request) found = true;
    }

    #define MAX_EXPECT_LIST 10
    if(!found)
    {
        if(listOfExpectedAnswer.size() < MAX_EXPECT_LIST)
        {
            ExpectAnswer toWaitAnswerElem(pNode, packet_type, request, cb, topic, max_retry, expectPayload, size);
            listOfExpectedAnswer.push_back(toWaitAnswerElem);
            sendCommand(request);
        }
        else PRTLN("MAX_EXPECT_LIST reached");
    }
    #if SHOW_EXPECT_EVENT
    #endif
    else  PRTLN("Already in Expected list");
}
//void Smk900::





void Smk900::CheckIfAnswerWasExpectedAndCallSuccessFunction(hexPacket_t rxPkt)
{
  #if SHOW_EXPECT_EVENT
    //Serial.println("CheckIfAnswerWasExpectedAndCallSuccessFunction()");
  #endif    
    //check if there is something to expect
    if(listOfExpectedAnswer.size() > 0)
    {
        std::list<ExpectAnswer>::iterator i = listOfExpectedAnswer.begin();

          #if SHOW_EXPECT_EVENT
            Serial.print("rxPkt: ");
            printApiPacket(rxPkt);
          #endif            


        // for all expected answer recorded
        while (i != listOfExpectedAnswer.end())
        {
            //if packet is local
            if(rxPkt[3] != PACKET_REMOTE_ANSWER)
            {
                if((i->_packet_type | 0x10) == rxPkt[3])
                {
                    #if SHOW_EXPECT_EVENT
                        Serial.println(" = expect local match found ");
                    #endif
                        i->_expect_callback(i->_pNode, rxPkt, true, i->_topic);
                        i=listOfExpectedAnswer.erase(i);
                    #if SHOW_EXPECT_EVENT
                        Serial.println(" = callback done");
                    #endif
    
                        break;
                }
            }
            //since it is remote check the remote packet type futher
            
            else if((i->_packet_type | 0x10) == (rxPkt[6] & 0x7F))
            {
              #if SHOW_EXPECT_EVENT
                Serial.println("  = remote packet match");
              #endif

                //PRTLN("PACKET REMOTE RESP");
                u_mac add_rx;


                add_rx.bOff[byte0]=rxPkt[7];
                add_rx.bOff[byte1]=rxPkt[8];
                add_rx.bOff[byte2]=rxPkt[9];
                add_rx.bOff[byte3]=0;

                //Serial.println("  address cmp:" + String(add_rx.address) + "   exp:" + String(i->_pNode.second.mac.address));
                //if address and packet type match, we found the good one
                
                if(add_rx.address== i->_pNode->second.mac.address)
                {

                    #if SHOW_EXPECT_EVENT
                        Serial.println(" == expect match found");
                    #endif

                    #if MODBUS_REGISTER_ENABLED
                    Helper::PollPacketToModbusRegister(i->_pNode, rxPkt);
                    #endif

                    i->_expect_callback(i->_pNode, rxPkt, true, i->_topic);

                     i->_pNode->second.dataValid = true;
                  #if SHOW_EXPECT_EVENT
                    Serial.println("  data validity TRUE  <----------" + i->_pNode->second.name);
                  #endif

                    i=listOfExpectedAnswer.erase(i);

                    break;
                }
            }

            i++;
        }
    }
}


//rfPayloadToJsonFromPacket(i->_idx);



//a chaque EOB, incrémente les compteur de EOB pour tout les nodes qui attende un message. 
//si il a 2 EOB sans réponse, la meme requete peut etre envoyé de nouveau si le nombre de retry est >0
//apres le nombre de retry sans réponse, le node est considéré comme dead
// 

void Smk900::CheckExpectTimeout()
{
  #if SHOW_EXPECT_EVENT
    //Serial.println("CheckExpectTimeout()"); delay(100);
  #endif
    
    if(listOfExpectedAnswer.size() > 0)
    {
        
        auto i = listOfExpectedAnswer.begin();

        while (i != listOfExpectedAnswer.end())
        {
            if( (++i->_eob_cnt) >2)
            {
                //Serial.println("Expect failed, reach max EOB");
                i->_eob_cnt=0;
                i->_nb_eob_recv++;

                //retry if less than max allowed and data was valid last time
                //we don't want to retry if it was already dead and waste time here
              #if SHOW_EXPECT_EVENT
                Serial.println("  retry count is:" + String(i->_nb_eob_recv));
              #endif
                if(i->_nb_eob_recv > i->_max_nb_eob_recv)
                {
                    hexPacket_t dummy;
                  #if SHOW_EXPECT_EVENT
                    Serial.println("  data validity FALSE <---- " + i->_pNode->second.name);
                  #endif
                     i->_pNode->second.dataValid=false;
                    i->_expect_callback(i->_pNode, dummy, false, i->_topic);//if user want to do something

                    i=listOfExpectedAnswer.erase(i);
                }
                else
                {
                    lowPriorityFifoCommandList.push_back(i->_request);
                    #if SHOW_EXPECT_EVENT
                    Serial.println( i->_pNode->second.name + " retry no: " + String(i->_nb_eob_recv) + " of " + String(i->_max_nb_eob_recv));
                    #endif
                }
            }
            //Serial.print("  expect list sz:");
            //Serial.println(listOfExpectedAnswer.size());
            if(i != listOfExpectedAnswer.end()) i++;
        }
        
    }
                    
}


void Smk900::automaticNodePolling()
{
    static bool current_focus_cycle = true;
  #if SHOW_AUTOMATIC_POLLING
    Serial.print("automaticNodePolling() mode:");
    Serial.println(polling_mode);
  #endif

    //if there are nodes to poll
    if(nodes.pool.size() > 0)
    {
        hexPacket_t smkPacket;
        //bool must_increase_polling_iterator = false;
        mesh_t::iterator pPoll = nodes.pool.end();

        unsigned long seconds = millis()/1000;

        //check if the node to poll is already in the list expected to avoid multiple retry of the same node
        bool isnt_already_in_expect_list = true;
        for(auto x : listOfExpectedAnswer) {
            if(x._pNode->first == pCurrentNode->first)
            {
                isnt_already_in_expect_list = false;
                break;
            }
        } 

        if(focus_node != 0)
        {
            uint64_t toComp = (millis() - focus_node_start_time)/1000;
            if(toComp < _duration_focus_on_node_polling)
            {
              #if SHOW_AUTOMATIC_POLLING
                Serial.print("focus polling on");
                Serial.println(SmkList::mac2String(focus_node));
              #endif          

                //current_focus_cycle allow to toggle 1/2 time on a specific node 
                //while allowing to continue the round robin cycle on the other 1/2 of time
                if(current_focus_cycle)
                {
                    pPoll = find(focus_node);
                    if(pPoll == nodes.pool.end()) current_focus_cycle = false;
                }
                else
                {
                    if(polling_mode == "time" || polling_mode == "fast")
                    {
                        //must_increase_polling_iterator = true;
                        findNext();
                        pPoll = pCurrentNode;
                    }
                    else pPoll != nodes.pool.end();
                    current_focus_cycle = true;
                }
            }

            //expiration of the focus mode, will return to the programmed polling_mode variable
            else
            {
                focus_node=0;   
                Serial.println("exit focus mode");
            }


        }
        else if(polling_mode == "time")
        {
          #if SHOW_AUTOMATIC_POLLING
            Serial.println("time based polling mode");
          #endif          
            
            if(seconds - pCurrentNode->second.elapse_time >= pCurrentNode->second.sample_rate && isnt_already_in_expect_list)
            {
                //must_increase_polling_iterator = true;
                findNext();


                pPoll = pCurrentNode;
            }
        }
        else if(polling_mode == "fast")
        {
          #if SHOW_AUTOMATIC_POLLING
            Serial.println("fast based polling mode");
          #endif          
            //must_increase_polling_iterator = true;
            findNext();
            pPoll = pCurrentNode;
        }


        //if the 
        if(pPoll != nodes.pool.end())
        {
            #if SHOW_AUTOMATIC_POLLING
                Serial.println("DEBUG!!!!!!!!!!!!!!!!!!!");
                delay(100);
                Serial.println(pPoll->second.getMacAsString());
                //Serial.printf ("Automatic Polling --> millis   %lu  -  %lu >= %d\n", seconds, pPoll->second.elapse_time, pCurrentNode->second.sample_rate);
            #elif SHOW_MIN_DEBUG
                Serial.print("?");
            #endif
            pPoll->second.elapse_time = seconds;
            String t = pPoll->second.type; 
            hexPacket_t rqt_status;

            //we build the request frame 
            
            
            hexPacket_t user_req_to_append = cbAutoRequestBuilder(pPoll);

            if(user_req_to_append.size() >0) rqt_status = user_req_to_append;
            else if(type_json[t]["command"].containsKey("status"))
                rqt_status.push_back(type_json[t]["command"]["status"]["rqst"].as<byte>());

            //set the address of the node to poll in the transmission buffer
            byte sz = 6 + rqt_status.size();
            smkPacket = {0xFB, sz, 0, 0x0C, 0x00, PACKET_VM_REQUEST};

            smkPacket.push_back(pPoll->second.mac.bOff[byte0]);
            smkPacket.push_back(pPoll->second.mac.bOff[byte1]);
            smkPacket.push_back(pPoll->second.mac.bOff[byte2]);

            //add custom payload from json file if available
            for(auto c : rqt_status) smkPacket.push_back(c);        
            
            #if SHOW_TRANSMITED_PACKET_TO_SMK900 || SHOW_AUTOMATIC_POLLING
            printApiPacket(smkPacket, "out: ");
            #endif
            
            //write the polling command 

            WriteAndExpectAnwser(pPoll, smkPacket, 0x0E, "status", cbAutomaticPolling);
        }
        /*
        if(must_increase_polling_iterator)
        {
            pCurrentNode++;
            if(pCurrentNode == nodes.pool.end()) pCurrentNode = nodes.pool.begin();
        }
        */

    }
    //else Serial.println();
}


bool Smk900::enableAutomaticPolling(String mode,String mac, uint16_t duration){
    bool ret = true;


    if(mode == "focus")
    {
        _duration_focus_on_node_polling = duration;
        focus_node_start_time=millis();
        if(!setFocusMode(mac))
        {   
            focus_node = 0;
            ret = false;
        }
        if(focus_node)
        {
            Serial.print("enter focus mode for ");
            Serial.print(duration);
            Serial.print("   _dur= ");
            Serial.println(_duration_focus_on_node_polling);
        }
    }
    else if (mode != "time" || mode != "fast")
    {
        
        polling_mode = mode; //default mode
        setAutoPollingNodeMode(true);
    }
    else if (mode == "disabled" || readFile("pollingMode") == "disabled")
    {
        setAutoPollingNodeMode(false);
    }
    return ret;
}

#define DEBUG_FINDNEXT false
bool Smk900::findNext(bool onlyRemote, bool initSearch)
{
	bool ret = false;
	mesh_t::iterator next = pCurrentNode;

    #if DEBUG_FINDNEXT
    if(nodes.pool.size() <= 1)
    {
        Serial.println("no node, so can't find next");
    }
    Serial.println("find next start");
    #endif

	if (initSearch)
	{
        #if DEBUG_FINDNEXT
        Serial.println("Init search");
        #endif
		next = nodes.pool.begin();
	}
	else next++;

	int nbCheck = 0;
	while (nbCheck < nodes.pool.size())
	{
		if (next == nodes.pool.end()) next = nodes.pool.begin();

        if(onlyRemote && next->second.local){
            #if DEBUG_FINDNEXT
            Serial.println("Local so search next");
            Serial.println(next->second.getMacAsString());
            #endif
            next++;
        } 
        else {
            pCurrentNode = next;
            #if DEBUG_FINDNEXT

            Serial.print("mac found is: ");
            Serial.println(next->second.getMacAsString());
            #endif
            ret = true; 
            break;
        };
        nbCheck++;
	}

    #if DEBUG_FINDNEXT
    if(!ret)
    {
        Serial.println("No node found");
    }
    Serial.println("find next end");
    #endif

	return ret;
}
