#include <spidermeshapi.h>
#include "esp32-hal-uart.h"
#include "soc/uart_struct.h"

#ifndef THREAD_DELAY_WINDOW
	#define THREAD_DELAY_WINDOW 100
#endif

portMUX_TYPE SpidermeshApi::mutexExpect;


mesh_t::iterator SpidermeshApi::gateway;
mesh_t SpidermeshApi::gateway_boot;
std::list<ExpectAnswer> SpidermeshApi::listOfExpectedAnswer;
bool SpidermeshApi::gatewayMacAddressIsReceived=false;

SmkList SpidermeshApi::nodes;
ExpectCallback SpidermeshApi::cbAutomaticPolling;
ExpectCallback SpidermeshApi::cb_automatic_polling_failed;
RequestBuilderCallback SpidermeshApi::cbAutoRequestBuilder;
unsigned char SpidermeshApi::state_serial;
mesh_t::iterator SpidermeshApi::pCurrentNode;

bool SpidermeshApi::_auto_polling_mode;
String SpidermeshApi::polling_mode;


uint16_t SpidermeshApi::length_packet;
unsigned char SpidermeshApi::idx_buf;

int SpidermeshApi::eob_cnt;
byte SpidermeshApi::channel_rf;
firmware_t SpidermeshApi::firmware;

apiframe SpidermeshApi::current_packet;
std::function<void(apiframe)> SpidermeshApi::cbWhenPacketReceived;
std::function<void(apiframe)> SpidermeshApi::cbWhenPacketSent;
std::function<bool(bool)> SpidermeshApi::WhenEobIsReceived;
uint32_t SpidermeshApi::focus_node;
unsigned long SpidermeshApi::focus_node_start_time;
uint16_t SpidermeshApi::_duration_focus_on_node_polling;


//std::vector <mesh_t::iterator> SpidermeshApi::otaList;

bool SpidermeshApi::enable_terminal_record;
std::list<String> SpidermeshApi::terminalBuffer;
long SpidermeshApi::timeout_expect;
unsigned long SpidermeshApi::previousMillisExpectPacketReceived;
bool SpidermeshApi::watchdog_serial_parser;
unsigned long SpidermeshApi::previousMillisPacketParser;
uint8_t SpidermeshApi::idx_polling_cnt;


WriteAndExpectList_t SpidermeshApi::writeAndExpectList;
commandList_t SpidermeshApi::lowPriorityFifoCommandList;
commandList_t SpidermeshApi::highPriorityFifoCommandList;

bool SpidermeshApi::show_eob = false;
bool SpidermeshApi::show_apipkt_in = false;
bool SpidermeshApi::show_apipkt_out = false;


MeshParam SpidermeshApi::actualMeshSpeed;
MeshParam SpidermeshApi::requiredMeshSpeed;
uint64_t SpidermeshApi::timeCallbackUser;

HardwareSerial SpidermeshApi::smkport(2);



SpidermeshApi::SpidermeshApi()
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


    cbAutoRequestBuilder = RequestBuilderCallback([](mesh_t::iterator pNode) -> apiframe{apiframe x; return x;});

    //Serial.println("Serial port configured");

    length_packet = 0;
    idx_polling_cnt=0;

    cbWhenPacketReceived = [](apiframe){};
    //cbWhenPacketSent = [](apiframe){};

    mutexExpect = portMUX_INITIALIZER_UNLOCKED;
    enable_terminal_record = false;
}

bool SpidermeshApi::init()
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
    smkport.setPins(RX_PORTIA,TX_PORTIA, -1, CTS_PORTIA);
    smkport.setHwFlowCtrlMode(HW_FLOWCTRL_CTS);


    //Sderial2.begin(PORTIA_BAUDRATE, uart_registers.conf0.val, RX_PORTIA, TX_PORTIA);
    //Sderial.println(" OK");


    //PRTLN(" before init smk900"); delay(10);


    //Serial.println("Init portia done");
    return true;
}



void SpidermeshApi::reset()
{
    digitalWrite(RESET_PORTIA,LOW);
    delay(1000);
    digitalWrite(RESET_PORTIA,HIGH);
    delay(100);
}


void SpidermeshApi::write(uint32_t c)
{
    smkport.write(c);
}
void SpidermeshApi::write32(uint32_t c)
{
    u_bytes u;
    u.uint32b = c;

    smkport.write(u.uint8b[0]);
    smkport.write(u.uint8b[1]);
    smkport.write(u.uint8b[2]);
    smkport.write(u.uint8b[3]);
}

void SpidermeshApi::SaveGatewayMacAddress(apiframe packet)
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


apiframe SpidermeshApi::localSetRegister(uint8_t offset_register, uint8_t size_register, uint8_t *content)
{
    uint8_t cmd[20] = {0xFB, 0, 0, 4, 0, offset_register, size_register};
    memcpy(cmd+7, content, size_register);
    uint8_t len = size_register + 4;
    cmd[1] = len;

    apiframe ret;
    for(int i=0; i<len+3; i++)
    {
        ret.push_back(cmd[i]);
    } 
    return ret;
}

apiframe SpidermeshApi::localTransfertConfigFromRAMBUFF(uint8_t location)
{
    uint8_t cmd[5] = {0xFB, 2, 0, 0x0B, location};

    apiframe ret;

    for(int i=0; i<5; i++)
    {
        ret.push_back(cmd[i]);
    } 
    return ret;
}

apiframe SpidermeshApi::setDyn(uint8_t po, uint8_t pi,uint8_t hop, uint8_t rdx, uint8_t rde, uint8_t duty)
{
    uint8_t cmd[10] = {0xFB, 7, 0, 0x0A, po, pi, hop, rdx, rde, duty};

    apiframe ret;
    for(int i=0; i<10; i++)
    {
        ret.push_back(cmd[i]);
    } 
    return ret;   
}

apiframe SpidermeshApi::requestMacAddress()
{
    Serial.println("==Request Gateway Mac Address");
    uint8_t cmd[20] = {0xFB, 4, 0, 3, 2,0,8};

    apiframe ret;
    for(int i=0; i<7; i++)
    {
        ret.push_back(cmd[i]);
    } 
    return ret;   
}

//-----------------------------------------------------------------------------------------------
void SpidermeshApi::task()
{
    //manage serial port reception
    if(!parseReceivedData());


}




//------------------------------------------------------------------------------------------------
void SpidermeshApi::initWatchdogParser()
{
    watchdog_serial_parser = true;
    previousMillisPacketParser = millis();
    //LedRemoteOn();
}

//------------------------------------------------------------------------------------------------
void SpidermeshApi::disableWatchdogParser()
{
    watchdog_serial_parser = false;
    //LedRemoteOff();
}


//------------------------------------------------------------------------------------------------
bool SpidermeshApi::parseReceivedData()
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
                    bool iseob = isEobPacket(current_packet);
                    if(show_apipkt_in && !iseob) printApiPacket(current_packet, PREFIX_IN);
                    CheckIfAnswerWasExpectedAndCallSuccessFunction(current_packet);


                    timeCallbackUser = millis();
                    if(cbWhenPacketReceived) cbWhenPacketReceived(current_packet); //for user purpose
                    timeCallbackUser = millis() - timeCallbackUser; 

                    //IS EOB
                    if(iseob)
                    {
                        eob_cnt++; //counter manage at higher level
                        CheckExpectTimeout();
                        WhenEobIsReceived(true);
                    }
                    if(iseob) OptimalDelay();                 //delay for other thread

                    if(enable_terminal_record) AddToTerminalBuffer("in :", &current_packet);

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
void SpidermeshApi::OptimalDelay()
{
    if(actualMeshSpeed.hop != 0)
    {

        int tBroadcast;
        int tInterval;
        int tSleep;
        TimmingMeshCalculator(&tBroadcast, &tInterval, &tSleep);

        int secureSleep = tSleep - THREAD_DELAY_WINDOW - timeCallbackUser;

        if(secureSleep<0){
            Serial.println("Warning, callback function take too much time!");
            secureSleep = 10;
        } 
        if(secureSleep >60000) secureSleep = 60000;

        #if 0
            /*
            char msg[100];
            sprintf(msg, "Bo:%d\nBi:%d\nHops:%d\nR:%d\nRe:%d\nDuty:%d\n",actualMeshSpeed.bo,actualMeshSpeed.bi,actualMeshSpeed.hop,actualMeshSpeed.rde,actualMeshSpeed.rd,actualMeshSpeed.duty);
            Serial.print(msg);

            Serial.print("tBroadcast:");
            Serial.println(tBroadcast);
            Serial.print("tInterval:");
            Serial.println(tInterval);
            Serial.print("tSleep:");
            Serial.println(tSleep);*/
            Serial.print("  secureSleep:");
            Serial.println(secureSleep);
            Serial.print("  timeCallbackUser:");
            Serial.println(timeCallbackUser);
        #endif


        int remaining = secureSleep;
        do
        {
            delay(100);
            remaining -=100;
            if(smkport.available()) break;
        } while (remaining>0);
    }
    else
    {
        delay(100);
    }
    

}

//------------------------------------------------------------------------------------------------
bool SpidermeshApi::TimmingMeshCalculator(int* tbroadcat, int* tinterval, int* tsleep)
{
    bool ret = false;
    if(actualMeshSpeed.hop != 0)
    {

        *tbroadcat = 10 * (actualMeshSpeed.hop * (actualMeshSpeed.bo + actualMeshSpeed.bi) + actualMeshSpeed.rde *actualMeshSpeed.rd);
        *tinterval = *tbroadcat * actualMeshSpeed.duty;
        *tsleep = *tinterval - *tbroadcat;
        ret = true;
    }
    return ret;
}

//------------------------------------------------------------------------------------------------
void SpidermeshApi::parseApiFromHost(uint8_t* buf, uint16_t len)
{
    uint16_t id_pkt = 0;
    uint8_t state_pkt = STATE_SOF;
    static uint16_t len_pkt;
    uint16_t last_idx_packet = 0;
    commandList_t cmd_list_ok;
    apiframe cmd_pkt;


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
String SpidermeshApi::sendCommand(apiframe cmd)
{
    for (int i = 0; i < cmd.size(); i++){
        write(cmd[i]);
    }
    if(cbWhenPacketSent)cbWhenPacketSent(cmd);


    String cmd_out = hexPacketToAscii(cmd);
    #if SHOW_TRANSMITED_PACKET_TO_SMK900
    printApiPacket(cmd, "out: ");    
    //Serial.println(cmd_out);
    #endif
    if(enable_terminal_record) AddToTerminalBuffer("out:", &cmd);

    return cmd_out;
}

void SpidermeshApi::AddToTerminalBuffer(String head, apiframe *cmd)
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
bool SpidermeshApi::sendNextPacketBuffered()
{
    bool ret = false;
    taskENTER_CRITICAL(&mutexExpect);
    if (writeAndExpectList.size()>0 || lowPriorityFifoCommandList.size() > 0 || highPriorityFifoCommandList.size() > 0)
    {
        //send the command
    #if SHOW_TRANSMITED_PACKET_TO_SMK900 && 0
        Serial.print("on demand cmd sent: ");
    #endif
        apiframe cmd;
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
            cmd = req.payload;
            byte pType = (req.payload[3]!=PACKET_TXAIR_CMD_WRAPPER) ? req.payload[3] : req.payload[5];
            WriteAndExpectAnwser(req.pNode, req.payload, pType, req.tag, req.callback);
        }



        if(show_apipkt_out)
        {
            String prefix = "  out: ";
            if(cmd[5] == 0x4E) prefix = "  broadcast: ";
            printApiPacket(cmd, prefix);
        }

        ret = true;
    }
    taskEXIT_CRITICAL(&mutexExpect);

    return ret;
}

void SpidermeshApi::ClearFifoAndExpectList()
{
    taskENTER_CRITICAL(&mutexExpect);
    writeAndExpectList.clear();
    lowPriorityFifoCommandList.clear();
    highPriorityFifoCommandList.clear();
    taskEXIT_CRITICAL(&mutexExpect);
    dumpReceivedBuffer();
}

bool SpidermeshApi::addWriteExpect(MeshRequest_t r)
{
    taskENTER_CRITICAL(&mutexExpect);
    writeAndExpectList.push_back(r);
    taskEXIT_CRITICAL(&mutexExpect);
    return true;
}

bool SpidermeshApi::addWriteExpect(mesh_t::iterator p, apiframe h, String t, ExpectCallback cb)
{
    MeshRequest_t req = {p, h, cb, t};
    addWriteExpect(req);
    return true;
}

bool SpidermeshApi::addApiPacketLowPriority(String asciiCommand)
{

    apiframe od_pkt = convertAsciiTohexCommand(asciiCommand.c_str());
    //printApiPacket(od_pkt);
    addApiPacketLowPriority(od_pkt);
    return true;
}

bool SpidermeshApi::addApiPacketLowPriority(uint8_t* buffer, int size)
{
    apiframe buf_hex;
    for(int i=0; i<size;i++)
    {
        buf_hex.push_back(buffer[i]);
    }
    addApiPacketLowPriority(buf_hex);
    return true;
}

bool SpidermeshApi::addApiPacketLowPriority(apiframe hcmd)
{
    taskENTER_CRITICAL(&mutexExpect);
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
    taskEXIT_CRITICAL(&mutexExpect);

    return false;
}


bool SpidermeshApi::addApiPacketHighPriority(apiframe hcmd)
{
    taskENTER_CRITICAL(&mutexExpect);
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
    taskEXIT_CRITICAL(&mutexExpect);

    return false;
}



void SpidermeshApi::WriteAndExpectAnwser(mesh_t::iterator pNode, apiframe request, uint8_t packet_type, String tag, ExpectCallback cb)
{
    uint max_retry = DEFAULT_MAX_TRY_TO_SEND;
    apiframe expectPayload;
    int16_t size = -1;

    WriteAndExpectAnwser(pNode,request,packet_type,max_retry,expectPayload, size, tag, cb);
}
void SpidermeshApi::WriteAndExpectAnwser(mesh_t::iterator pNode, apiframe request, uint8_t packet_type, uint8_t max_retry, String tag,ExpectCallback cb)
{
    apiframe expectPayload;
    int16_t size = -1;

    WriteAndExpectAnwser(pNode,request,packet_type,max_retry,expectPayload, size, tag, cb);
}

void SpidermeshApi::WriteAndExpectAnwser(  mesh_t::iterator pNode, 
                                    apiframe request, 
                                    uint8_t packet_type,                                      
                                    uint8_t max_retry, 
                                    apiframe expectPayload, 
                                    int16_t size, 
                                    String tag,
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


            ExpectAnswer toWaitAnswerElem(pNode, packet_type, request, cb, tag, max_retry, expectPayload, size);
            listOfExpectedAnswer.push_back(toWaitAnswerElem);
            sendCommand(request);
        }
        else PRTLN("MAX_EXPECT_LIST reached");
    }
}
//void SpidermeshApi::





void SpidermeshApi::CheckIfAnswerWasExpectedAndCallSuccessFunction(apiframe rxPkt)
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
            printApiPacket(rxPkt, "  expect:", KCYN);
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
                        i->_expect_callback(i->_pNode, rxPkt, true, i->_tag);
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


                int len = nodes.getNbBytePacket(i->_pNode,i->_tag);




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
                    //Serial.printf("%s %s == %s %s\n",KGRN,macIntToString(add_rx.address).c_str(), i->_pNode->second.getMacAsString(), KNRM);
                    //Serial.printf("%srxPkt.size()-10 %d==%d len%s\n",KGRN,rxPkt.size()-10, len,KNRM);
                    if(rxPkt.size()-10 == len)
                    {
                        Serial.printf("%sSUCCES %s\n",KCYN, i->_pNode->second.getMacAsString());
                    #if SHOW_EXPECT_EVENT
                        Serial.println(" == expect match found");
                    #endif
                        i->_expect_callback(i->_pNode, rxPkt, true, i->_tag);
                        i->_pNode->second.dataValid = 1;
                    #if SHOW_EXPECT_EVENT
                        Serial.println("  data validity TRUE  <----------" + i->_pNode->second.name);
                    #endif
                        i=listOfExpectedAnswer.erase(i);
                        break;
                    }
                    //if mac is same, but packet isn't define, it must be a special request from user
                    else if(len == -1 )
                    {
                        i->_expect_callback(i->_pNode, rxPkt, true, i->_tag);
                        i=listOfExpectedAnswer.erase(i);
                        break;
                    }
                    
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

void SpidermeshApi::CheckExpectTimeout()
{
  #if SHOW_EXPECT_EVENT
    //Serial.println("CheckExpectTimeout()"); delay(100);
  #endif
    
    if(listOfExpectedAnswer.size() > 0)
    {
        
        auto i = listOfExpectedAnswer.begin();

        while (i != listOfExpectedAnswer.end())
        {
            //if timeout, when eob more than 2 is considered timeout
            if( (++i->_eob_cnt) >2)
            {
                //Serial.println("Expect failed, reach max EOB");
                i->_eob_cnt=0;

                //retry if less than max allowed and data was valid last time
                //we don't want to retry if it was already dead and waste time here
                apiframe dummy = {};
                PRT(KRED);
                PRT("Timeout occur for node: ");
                PRTLN(i->_pNode->second.getMacAsString());
                PRT(KNRM);

                #if SHOW_EXPECT_EVENT
                Serial.println("  data validity FALSE <---- " + i->_pNode->second.name);
                #endif
                i->_pNode->second.dataValid=0;
                i->_expect_callback(i->_pNode, dummy, false, i->_tag);//if user want to do something

                i=listOfExpectedAnswer.erase(i);
            }
            //Serial.print("  expect list sz:");
            //Serial.println(listOfExpectedAnswer.size());
            if(i != listOfExpectedAnswer.end()) i++;
        }
        
    }
                    
}


void SpidermeshApi::automaticNodePolling()
{
    static bool current_focus_cycle = true;
    static bool toggle_polling=false;
    static NodeIterator_t pNodeToPoll = nodes.pool.begin();

    bool noToPollFound=false;
    toggle_polling = !toggle_polling;    

  #if SHOW_AUTOMATIC_POLLING
    Serial.print("automaticNodePolling() mode:");
    Serial.println(polling_mode);
  #endif
    mesh_t::iterator pPoll = nodes.pool.end();
    //if there are nodes to poll
    if(nodes.pool.size() > 1) //we check if more than one since gateway is the first
    {
        apiframe smkPacket;
        //bool must_increase_polling_iterator = false;

        unsigned long seconds = millis()/1000;

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






            fastWhenNoNormal:
            //FAST POLLONG
            if(toggle_polling && (nodes.toPollFaster.size()>0))
            {
                int limitSecurity=nodes.toPollFaster.size();
                do{
                    ++nodes.idxNodeToPollFast;
                    if(nodes.idxNodeToPollFast >= nodes.toPollFaster.size()) nodes.idxNodeToPollFast=0; //round robin fast polling
                    pCurrentNode=nodes.toPollFaster[nodes.idxNodeToPollFast];
                    pPoll = pCurrentNode;
                    
                }while(nodes.toPollFaster[nodes.idxNodeToPollFast]==pNodeToPoll  && --limitSecurity >0);
                nodes.toPollFaster[nodes.idxNodeToPollFast]->second.elapse_time = seconds;
                noToPollFound = true;
                #if SHOW_AUTOMATIC_POLLING
                    Serial.print(KYEL);
                    Serial.println("High priority polling");
                    Serial.print(KNRM);
                #endif   
            }

            //NORMAL POLLING
            else
            {
                for(int i=0; i<nodes.pool.size(); i++)
                {
                    if(++pNodeToPoll == nodes.pool.end()) 
                        pNodeToPoll = nodes.pool.begin();

                    if(nodes.toPollFaster.size()>0)
                    {
                        if(pNodeToPoll->first == nodes.toPollFaster[nodes.idxNodeToPollFast]->first) continue;
                    }

                    //if elapse time is expire
                    if((seconds - pNodeToPoll->second.elapse_time ) >= pNodeToPoll->second.sample_rate || pNodeToPoll->second.elapse_time == 0) 
                    {
                        pNodeToPoll->second.elapse_time = seconds;
                        pCurrentNode = pNodeToPoll;
                        pPoll = pCurrentNode;
                        noToPollFound = true;
                        #if SHOW_AUTOMATIC_POLLING
                            Serial.print(KYEL);
                            Serial.println"Normal priority polling");
                            Serial.print(KNRM);
                        #endif   
                        break;
                    }
                }
                if(!noToPollFound && (nodes.toPollFaster.size()>0)){ toggle_polling=true; goto fastWhenNoNormal;}
            }         

          #if SHOW_AUTOMATIC_POLLING
            Serial.println("time based polling mode");
          #endif          
            /*
            if(seconds - pCurrentNode->second.elapse_time >= pCurrentNode->second.sample_rate)
            {
                //must_increase_polling_iterator = true;
                findNext();


                pPoll = pCurrentNode;
            }
            */



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
                Serial.println("SHOW_AUTOMATIC_POLLING");
                delay(100);
                Serial.println(pPoll->second.getMacAsString());
                //Serial.printf ("Automatic Polling --> millis   %lu  -  %lu >= %d\n", seconds, pPoll->second.elapse_time, pCurrentNode->second.sample_rate);
            #elif SHOW_MIN_DEBUG
                Serial.print("?");
            #endif
            pPoll->second.elapse_time = seconds;
            String t = pPoll->second.type; 
            apiframe rqt_status;

            //we build the request frame 
            
            
            apiframe user_req_to_append = cbAutoRequestBuilder(pPoll);

            if(user_req_to_append.size() >0) rqt_status = user_req_to_append;
            else if(nodes.getTypeJsonVariant()[t]["command"].containsKey("status"))
                rqt_status.push_back(nodes.getTypeJsonVariant()[t]["command"]["status"]["rqst"].as<byte>());

            //set the address of the node to poll in the transmission buffer
            byte sz = 6 + rqt_status.size();
            smkPacket = {0xFB, sz, 0, 0x0C, 0x00, PACKET_VM_REQUEST};

            smkPacket.push_back(pPoll->second.mac.bOff[0]);
            smkPacket.push_back(pPoll->second.mac.bOff[1]);
            smkPacket.push_back(pPoll->second.mac.bOff[2]);

            //add custom payload from json file if available
            for(auto c : rqt_status) smkPacket.push_back(c);        
            
            if(show_apipkt_out)
                printApiPacket(smkPacket, "  out: ");
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


bool SpidermeshApi::enableAutomaticPolling(String mode,String mac, uint16_t duration){
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
        setAutoPolling(true);
    }
    else if (mode == "disabled" || readFile("pollingMode") == "disabled")
    {
        setAutoPolling(false);
    }
    return ret;
}

#define DEBUG_FINDNEXT false
bool SpidermeshApi::findNext(bool onlyRemote, bool initSearch)
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


apiframe SpidermeshApi::apiPacket(uint8_t cmd, apiframe pkt, bool local, bool broadcastOtaUpdate, uint8_t phase)
{

    if(!local)
    {
        cmd |=0x80;
        if(broadcastOtaUpdate) cmd |=0x40;
        if(!broadcastOtaUpdate)
        {
            pkt.insert(pkt.begin(), pCurrentNode->second.mac.bOff[byte2]);
            pkt.insert(pkt.begin(), pCurrentNode->second.mac.bOff[byte1]);
            pkt.insert(pkt.begin(), pCurrentNode->second.mac.bOff[byte0]);
            
        }
        pkt.insert(pkt.begin(), cmd);
        pkt.insert(pkt.begin(), phase);
        pkt.insert(pkt.begin(), 0x0C); //wrapper
    }
    else
    pkt.insert(pkt.begin(), cmd);

    int len = pkt.size();
    pkt.insert(pkt.begin(), 0);
    pkt.insert(pkt.begin(), len);
    pkt.insert(pkt.begin(), 0xFB);

    if(show_apipkt_out)
        printApiPacket(pkt, PREFIX_OUT);
    return pkt;
}

apiframe SpidermeshApi::apiPacket(mesh_t::iterator pNode, uint8_t cmd, apiframe pkt, bool local, bool broadcastOtaUpdate, uint8_t phase)
{
    if(!local)
    {
        cmd |=0x80;
        if(broadcastOtaUpdate) cmd |=0x40;
        if(!broadcastOtaUpdate)
        {
            pkt.insert(pkt.begin(), pNode->second.mac.bOff[byte2]);
            pkt.insert(pkt.begin(), pNode->second.mac.bOff[byte1]);
            pkt.insert(pkt.begin(), pNode->second.mac.bOff[byte0]);
            
        }
        pkt.insert(pkt.begin(), cmd);
        pkt.insert(pkt.begin(), phase);
        pkt.insert(pkt.begin(), 0x0C); //wrapper
    }
    else
    pkt.insert(pkt.begin(), cmd);

    int len = pkt.size();
    pkt.insert(pkt.begin(), 0);
    pkt.insert(pkt.begin(), len);
    pkt.insert(pkt.begin(), 0xFB);

    printApiPacket(pkt, PREFIX_OUT);
    return pkt;
}
