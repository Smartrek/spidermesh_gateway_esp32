#include "bridge.h"
#include "utils.h"
#include "Spidermesh.h"


WiFiServer* Bridge::bridge_server;
WiFiClient Bridge::client;
PacketRxList_t Bridge::packetList;
bool Bridge::enabled;
int Bridge::port;
bridge_mode_t Bridge::state;
std::function<void()> Bridge::cbConnecting;
std::function<void()> Bridge::cbDisconnecting;
std::function<void(apiframe pkt)> Bridge::cbAddPacketToSmkTxBuffer;
std::function<void(apiframe pkt, String prefix)> Bridge::cbWhenSmkPacketReceived;

extern Spidermesh smk900;
Bridge::Bridge()
{

}

Bridge::Bridge(int _port)
{
    enabled=false;
    state = WAIT_ETH;
    port = _port;
    cbWhenSmkPacketReceived = [](apiframe, String){};
    cbAddPacketToSmkTxBuffer = [](apiframe){};
}

void Bridge::init()
{
    bridge_server = new WiFiServer(port);
    bridge_server->begin();
    Serial.println("bridge_server->begin()");
}

//---------------------------------------------------------------------------------------------------
//        BRIDGE 5556
//---------------------------------------------------------------------------------------------------
//si on ne peut pas se connecter, p-e il est n'est pas activé donc pas besoin de continuer.
//                                p-e il a un probleme coté serveur
bool Bridge::taskloop()
{
    bool ret = false;
    if(state == WAIT_ETH)
    {
        client = bridge_server->available();
        if(client)
        {
            cbConnecting();
            state = CONNECTED;
            delay(100);
        }
    }
    else if(state == CONNECTED)
    {
        if(!client.connected())
        {
            cbDisconnecting();
            state = WAIT_ETH;

        }        
        else //core task
        {
            int size;
            auto m = millis();
            uint64_t timestamp= millis()+10000;
            //Serial.println("BUG1");
            while((size = client.available()) && (timestamp>millis()))
            {
                uint8_t buffer[120];
                client.read(buffer, size);
                parseApiFromHost(buffer,size);
            }
            //Serial.println("BUG2");

            for(auto p:packetList)
            {
                for(auto b:p) client.write(b);
                cbWhenSmkPacketReceived(p, "SMK:");
                //printApiPacket(p,"SMK");
            }            
            packetList.clear();
            //Serial.println("BUG3");


        }
    }
    return ret;
}

//------------------------------------------------------------------------------------------------
void Bridge::parseApiFromHost(uint8_t* buf, uint16_t len)
{
    uint16_t id_pkt = 0;
    uint8_t state_pkt = 1;
    uint16_t len_pkt;
    uint16_t last_idx_packet = 0;
    apiframe cmd_pkt;


    while(id_pkt < len)
    //for(int i=0; i<len*2; i++)
    {
        //char result2[10];
        //sprintf(result2, "0x%02X ", buf[id_pkt]);
        //Serial.print(result2);

        switch (state_pkt)
        {
            case 1:
                if (buf[id_pkt] == 0xFB)
                {
                    state_pkt = 2;
                    len_pkt = 0;
                    cmd_pkt.push_back(buf[id_pkt]);
                }
                id_pkt++;
                break;
            case 2:
                if (len_pkt == 0){
                    cmd_pkt.push_back(buf[id_pkt]);
                    len_pkt += buf[id_pkt];
                }
                else
                { 
                    cmd_pkt.push_back(buf[id_pkt]);
                    len_pkt += ((uint16_t)buf[id_pkt])*256;
                    state_pkt = 3; 
                }
                id_pkt++;
                break;
            case 3:
                //read serial port char
                cmd_pkt.push_back(buf[id_pkt]);
                id_pkt++;
                
                //Lenght of packet reach, packet ready to parse
                if (id_pkt-last_idx_packet >= len_pkt+3)
                {
                    last_idx_packet= id_pkt;
                    cbWhenSmkPacketReceived(cmd_pkt,"ETH:");
                    cbAddPacketToSmkTxBuffer(cmd_pkt);
                    //smk900.addApiPacketLowPriority(cmd_pkt);

                    //Serial.println("  will be sent");
                    cmd_pkt.clear();
                    state_pkt = 1;
                    
                }

                break;

            default:
                state_pkt = 1;
                break;
        }
    }

}