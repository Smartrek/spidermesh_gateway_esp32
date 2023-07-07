#include <utils.h>
#include <stdio.h>
#include <stdlib.h>
#include "time.h"
#define ARDUINOJSON_USE_LONG_LONG 1
#include <ArduinoJson.h>

bool ntpServerSet = false;


apiframe convertAsciiTohexCommand(const char *asciiCommand)
{
    // std::stringstream xx;
    String s_cmd = asciiCommand;
    std::vector<String> byte_cmd = splitString(s_cmd,' ');
    apiframe hexCommand;
    for (auto b :byte_cmd)
    {
		int num;
		sscanf(b.c_str(), "%x", &num);

        //int b = (toInt(asciiCommand[i * 2]) << 4) + toInt(asciiCommand[i * 2 + 1]);
        hexCommand.push_back(num);
    }
    return hexCommand;
}


apiframe asciiTohexPacket(String acmd)
{
    char c[3];
    c[2]=0;
    apiframe ret;
    while(acmd.length()>=2)
    {
        int num;
        c[0]=acmd[0];
        c[1]=acmd[1];
        sscanf(c, "%x", &num);
        ret.push_back(num);
        acmd.remove(0,2);
    }
    return ret;
}


bool putPacketInsideBuffer(apiframe packet, uint8_t* buffer)
{
    apiframe buf_hex;
    for(int i=packet.size()-1; i>=0; i--)
    {
        buffer[i]=packet.back();
        packet.pop_back();
    }
    return true;
}




void printApiPacket(uint8_t* buffer, int size)
{
    String packet = "";
    char result[6];
    for (int i = 0; i < size; i++)
    {
        sprintf(result, "%02X ", buffer[i]);
        packet += result;
    }
    Serial.print(packet);
}


void printApiPacket(apiframe hcmd, String prefix, String color)
{
    if(hcmd.size()>0)
    {
        String packet = hexPacketToAscii(hcmd);
        Serial.print(color);
        Serial.print(prefix);
        Serial.println(packet);
        Serial.print(KNRM);
    }
}

String hexPacketToAscii(apiframe hcmd)
{
    String packet = "";
    char result[6];
    for (auto h: hcmd)
    {
        sprintf(result, "%02X ", h);
        packet += result;
    }
    return packet;
}


void setupWatchdog(hw_timer_t** pWatchdog, long millis, void (*fn)(void))
{
    *pWatchdog=timerBegin(1,80,true);
    timerAlarmWrite(*pWatchdog, millis *1000, false); // set time in uS must be fed within this time or reboot
    timerAttachInterrupt(*pWatchdog, fn, true);
    timerAlarmEnable(*pWatchdog);  // enable interrupt
}

void kickWatchdog(hw_timer_t *pWatchdog)
{
    timerWrite(pWatchdog, 0);
}


std::vector<String> splitString(String in, char splitChar)
{
    std::vector<String> ret;
    //Serial.println("-- splitString --");

    int curr = in.indexOf(splitChar);
    //Serial.println("--curr=" + String(curr));
    int prev = 0;
    String element;
    while (curr >= prev)
    {
        element = cleanString(in.substring(prev, curr));
        ret.push_back(element);
        prev = curr + 1;

        curr = in.indexOf(splitChar, prev);
        if (curr == -1)
            curr = in.length();
        //Serial.println("--curr=" + String(curr));
        //Serial.println("element=" + element + "=   prev=" + String(prev) + "  curr=" + String(curr));
    };
    return ret;
}


String cleanString(String toClean){
    String cleaned="";
    for(int i=0; i<toClean.length();i++)
    {
        if(toClean[i] != '\n' && toClean[i] != '\r')
            cleaned += toClean[i];
    }
    return cleaned;
}

String ApplyFixPoint(word num, int fix)
{
    String result = "";
    String r = String(num);
    int siz = r.length();

    int i=0;
    bool dot = false;
    for(auto c : r)
    {

        if(i==0 && fix>=siz) result = "0";
        if (fix >= siz -i && !dot){ 
            result += "."; 
            for(int j=0;j<fix-siz-i; i++)
            {
                result += "0";                            
            }
            dot=true;
        }
        result += c;
        i++;
    }
    return result;
}



String getTimeFormated()
{
    struct tm timeinfo;
    if(!getLocalTime(&timeinfo, 20)){
        Serial.println("Failed to obtain time. Make sure DNS IP is ok!");
        ntpServerSet = false;
        return "clock error, show millis(): " + String(millis());
    }    
    char sFormatedTime[80];
    strftime(sFormatedTime,80, "%A, %B %d %Y %H:%M:%S", &timeinfo);
    String ret = sFormatedTime;
    //Serial.println(ret);
    return ret;
}

String getMinTimeFormated()
{
    struct tm timeinfo;
    if(!getLocalTime(&timeinfo, 20)){
        Serial.println("Failed to obtain time. Make sure DNS IP is ok!");
        ntpServerSet = false;
        return "clock error, show millis(): " + String(millis());
    }    
    char sFormatedTime[80];
    strftime(sFormatedTime,80, "%H:%M:%S", &timeinfo);
    String ret = sFormatedTime;
    //Serial.println(ret);
    return ret;
}

void printLocalTime(){
    struct tm timeinfo;
    if(!getLocalTime(&timeinfo, 20)){
        Serial.println("Failed to obtain time");
        return;
    }
    Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");

    char sFormatedTime[80];
    strftime(sFormatedTime,80, "%A, %B %d %Y %H:%M:%S", &timeinfo);
    String ret = sFormatedTime;
    Serial.println(ret);
}

void setNtpClock(bool force)
{
    if(ntpServerSet && !force) return;

    DynamicJsonDocument timeJson(1000);
    deserializeJson(timeJson, readFile("/ntp.json"));
    //configTime(timeJson["gmtOffset_sec"], timeJson["daylightOffset_sec"], timeJson["server"]);

    //for previous version to 2.6.4 and lower 
    if(timeJson.containsKey("server"))  
        configTime(timeJson["gmtOffset_sec"], timeJson["daylightOffset_sec"], timeJson["server"]);
        
    //allow now multiple ntp server
    else if(timeJson.containsKey("server1") && timeJson.containsKey("server2") && timeJson.containsKey("server3"))
        configTime(timeJson["gmtOffset_sec"], timeJson["daylightOffset_sec"], timeJson["server1"], timeJson["server2"], timeJson["server3"]);
    
    ntpServerSet = true;
    //printLocalTime();
}


value_converter_union extractByteFromApiPacket(apiframe p, uint16_t idx, uint16_t len)
{
    value_converter_union v;

    int j = idx;
    Serial.print("extractByteFromApiPacket: ");
    for(int i=0; i<4; i++)
    {
        Serial.printf("%02X ", p[j]);
        v.uB[i] = (j < p.size() && i<len) ? p[j] : 0;
        j++;
    }
    return v;

}

uint32_t extractU32(String p, int pos){
    uint32_t cur =0;
    for(int i=3;i>=0;i--)
    {
        cur+= p[pos+i];
        if(i) cur<<=8;
    }
    return cur;
};

uint32_t extractU32(apiframe p, int pos){
    uint32_t cur =0;
    for(int i=3;i>=0;i--)
    {
        cur+= p[pos+i];
        if(i) cur<<=8;
    }
    return cur;
};

uint32_t extractU16(apiframe p, int pos){
    uint32_t cur =0;
    for(int i=1;i>=0;i--)
    {
        cur+= p[pos+i];
        if(i) cur<<=8;
    }
    return cur;
};


std::vector<String> ValidationEthernetConfig(String data)
{

	std::vector<String> data_array = splitString(data, ' ');

	if (data_array.size() == 5)
	{
		bool opOk = true;
		if (validIpAddress(data_array[0]))
		{
			// Serial.println("IP ADDRESS VALID " + data_array[0]);
		}
		else
			opOk = false;

		if (validIpAddress(data_array[1]))
		{
			// Serial.println("SUB NETWORK ADDRESS VALID " + data_array[1]);
		}
		else
			opOk = false;

		if (validIpAddress(data_array[2]))
		{
			// Serial.println("GATEWAY ADDRESS VALID " + data_array[2]);
		}
		else
			opOk = false;
		if (validIpAddress(data_array[3]))
		{
			// Serial.println("DNS1 ADDRESS VALID " + data_array[3]);
		}
		else
			opOk = false;
		if (validIpAddress(data_array[4]))
		{
			// Serial.println("DNS2 ADDRESS VALID" + data_array[4]);
		}
		else
			opOk = false;

		if (opOk)
		{
			return data_array;
		}
	}
	std::vector<String> null;
	return null;
}



bool macStringToInt(String ipStr, uint32_t *ip)
{
    std::vector<String> ipArray = splitString(ipStr, '.');
    if (ipArray.size() == 4 || ipArray.size() == 3)
    {
        uint32_t macConv=0;

        int j=0;
        for (int i = 3; i >= 0; i--)
        {
            // Serial.print("b" + String(i) +": " + ipArray[i] + " ");
            macConv += ipArray[i].toInt() << (8*j++);

            // ipIntArray[i]=ipArray[i].toInt();
        }
        *ip=macConv;
        // Serial.println(ipIntArray);
        return true;
    }
    return false;
}

String macIntToString(uint32_t mac)
{

    u_bytes m;
    m.uint32b = mac;

    return String( m.uint8b[3]) + "." + String( m.uint8b[2]) + "." + String( m.uint8b[1]) + "." + String( m.uint8b[0]);
}
