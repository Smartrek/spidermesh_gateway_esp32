#include <parser.h>
#define SHOW_DEBUG_EXTRACT_DATAJSON true
JsonObject SmkParser::type_json;

bool SmkParser::rfPayloadToJson(apiframe &packet, String tag, JsonVariant payload, String &type, bool includeUnits)
{
	if(packet.size()<=10) return false;

  #if SHOW_DEBUG_EXTRACT_DATAJSON
	Serial.println("\r\n-- Extraction parameter from json definition --");
	Serial.print ("  --> tag: ");
	Serial.println (tag);

	if(!type_json.containsKey(type)) Serial.printf("Node type:%s not available\n", type);
  #endif


	if( !type_json[type]["parser"].containsKey(tag))
	{
		#if SHOW_DEBUG_EXTRACT_DATAJSON
		Serial.println("Key Not valid, skip extraction");
		#else
		Serial.print("!");
		#endif
		return false;
	}


	JsonObject extract_parameters = type_json[type]["parser"][tag]["params"].as<JsonObject>();

	//payload["time"]="millis() " + String(millis()); //getTimeFormated();

	for(auto cur_variable:extract_parameters)
	{
		double fResult=0; //if needed
		bool float_result = false;

		JsonObject def_params = cur_variable.value().as<JsonObject>();
		//get definition parameters
		int begin = def_params["pos"][0];
		int len =def_params["pos"][1];
		int idx_begin_data_byte = begin/8 + 10;//11 is begin of data
		uint64_t unscaled_raw_data = 0;

	  #if SHOW_DEBUG_EXTRACT_DATAJSON
		Serial.print("  ==>> label:");
		Serial.print(cur_variable.key().c_str());
	  #endif


		//get variable byte from the packet according to the definition of the variable position
		int nbIt = (len/8);
		if(len%8) nbIt++;
		for(int i=0; i<nbIt && ((idx_begin_data_byte+i) < packet.size()); i++)
		{ 
			uint64_t b = packet[idx_begin_data_byte+i];
			unscaled_raw_data += b << (i*8);
		}

		uint64_t scaled_raw_data = getbits(unscaled_raw_data,begin%8,len);
	  #if SHOW_DEBUG_EXTRACT_DATAJSON
		Serial.printf("  = : %lu", scaled_raw_data);
	  #endif

		/*
		if(stype == "hex2str")
		{
			String result = "";
			for(int i=0; i<len/8;i++)
			{
				char h2s[6];
				sprintf(h2s, "%02X", packet[i+10+begin/8]);
				result +=h2s;
			}
			payload[x.key()] = result;
			continue;
		}
		if(def_params.containsKey("dict"))
			getlogfromdict(def_params,payload,packet,15,type);

		if(def_params.containsKey("table"))
			getErrorFromDict(def_params,payload,packet,11,type);
		*/


		String stype = DEFAULT_UNIT_TYPE; //default
		if(def_params.containsKey("type")) stype = def_params["type"].as<String>();
		#if SHOW_DEBUG_EXTRACT_DATAJSON
		Serial.printf("** %s **", stype);
		#endif


		if(includeUnits && def_params.containsKey("u"))
		{
			#if SHOW_DEBUG_EXTRACT_DATAJSON
			Serial.printf(" unit:%s ", def_params["u"].as<String>().c_str());
			#endif
			JsonObject key = payload.createNestedObject(cur_variable.key());
			int64_t res;
			if(stype=="float24" || stype == "float")
			{
				if(stype=="float24") scaled_raw_data <<= 8;
				float value = *((float*) (&scaled_raw_data));
				key["value"] = value;
				Serial.printf("%f\n", value);
			}
			else if(stype == "int64")
			{
				char buffer[50];
				sprintf(buffer, "%lld", scaled_raw_data);				
				key["value"] = buffer;
				Serial.printf("%s\n", buffer);
			}
			else
			{
				fResult = applyParams(scaled_raw_data, def_params);
				key["value"] = fResult;
			} 
			key["units"]=def_params["u"];
		}
		else
		{
			int64_t res;
			if(stype=="float24" || stype == "float")
			{
				if(stype=="float24") scaled_raw_data <<= 8;
				float value = *((float*) (&scaled_raw_data));
				payload[cur_variable.key()] = value;
				Serial.printf("%f\n", value);
			}
				
			else if(stype == "int64")
			{
				char buffer[50];
				sprintf(buffer, "%lld", scaled_raw_data);
				payload[cur_variable.key()] = buffer;
				Serial.printf("%s\n", buffer);
			}
			else
			{
				fResult = applyParams(scaled_raw_data, def_params);
				payload[cur_variable.key()] = fResult;
			} 
		}
	}
	return true;
}

bool SmkParser::rfPayloadToInt64(apiframe &packet, String tag, std::vector<meta_conversion>* payload, String &type)
{
	if(packet.size()<=10) return false;

	#if SHOW_DEBUG_EXTRACT_DATA
	Serial.println("rfPayloadToInt64");
	#endif

	if(!type_json[type]["parser"].containsKey(tag))
	{
		#if SHOW_DEBUG_EXTRACT_DATA
		Serial.println("Key Not valid, skip extraction");
		#else
		Serial.print("!");
		#endif
		return false;
	}

	JsonObject extract_parameters = type_json[type]["parser"][tag]["params"].as<JsonObject>();

	meta_conversion t;
	t.value = millis();
	t.bitsize = 64;
	payload->push_back(t); //getTimeFormated();

	for(auto cur_variable:extract_parameters)
	{
		double fResult=0; //if needed
		bool float_result = false;

		JsonObject def_params = cur_variable.value().as<JsonObject>();
		//get definition parameters
		int begin = def_params["pos"][0];
		int len =def_params["pos"][1];
		int idx_begin_data_byte = begin/8 + 10;//11 is begin of data
		uint64_t unscaled_raw_data = 0;

	  #if SHOW_DEBUG_EXTRACT_DATA
		Serial.print("  ==>> ");
		Serial.printf("label:");
		Serial.print(cur_variable.key().c_str());
	  #endif


		//get variable byte from the packet according to the definition of the variable position
		int nbIt = (len/8);
		if(len%8) nbIt++;
		for(int i=0; i<nbIt && ((idx_begin_data_byte+i) < packet.size()); i++)
		{ 
			uint64_t b = packet[idx_begin_data_byte+i];
			unscaled_raw_data += b << (i*8);
		}
		uint64_t scaled_raw_data = getbits(unscaled_raw_data,begin%8,len);
	  #if SHOW_DEBUG_EXTRACT_DATA
		Serial.print(" = ");
		Serial.print(scaled_raw_data);
	  #endif

		String stype = DEFAULT_UNIT_TYPE; //default
		if(def_params.containsKey("type")) stype = def_params["type"].as<String>();	 
		#if SHOW_DEBUG_EXTRACT_DATAJSON
		Serial.printf("** %s **", stype);
		#endif

		//if number is 16bits signed and is negative, convert in negative int 32bit

		int64_t res;
		if(stype=="float24") scaled_raw_data <<= 8;

		if(stype == "int64" || stype == "float" || stype =="float24")
		{
			res = scaled_raw_data;
			#if SHOW_DEBUG_EXTRACT_DATAJSON
			Serial.printf("raw: %lld\n",res);
			#endif

		}
		else
		{
			fResult = applyParams(scaled_raw_data, def_params);
			res = fResult;
		} 

		meta_conversion mres;
		mres.value = res;
		mres.bitsize = len;

		payload->push_back(mres);
	}
	return true;
}



double SmkParser::applyParams(int64_t value, JsonObject def_params, bool direction)
{
	double fResult = 0;

	String stype = DEFAULT_UNIT_TYPE; //default
	if(def_params.containsKey("type")) stype = def_params["type"].as<String>();

	//if number is 16bits signed and is negative, convert in negative int 32bit
	if(stype=="int16" && (value &0x8000)) value |= 0xFFFFFFFFFFFF0000;
	else if(stype=="int32" && value &(0x80000000)) value |= 0xFFFFFFFF00000000;

	#if SHOW_DEBUG_EXTRACT_DATA
	Serial.printf("  type %s  ", stype);
	Serial.printf("before %lld  ", value);
	#endif
	if(stype == "float") 	
		fResult = *((float*) &value);
	else if(stype == "float24")
	{
		value <<= 8; //shift to represent a float 32bit
		fResult = *((float*) &value);
	} 
	else fResult = value;

	if(def_params.containsKey("gain"))
	{
		double g = def_params["gain"].as<double>();
		if(direction) fResult *= g;
		else fResult /= g;
		#if SHOW_DEBUG_EXTRACT_DATA
		Serial.print(" g= ");
		Serial.print(fResult);
		#endif
	}

	if(def_params.containsKey("div"))
	{
		double d = def_params["div"].as<double>();
		if(direction)fResult /= d;
		else fResult *= d;

		#if SHOW_DEBUG_EXTRACT_DATA
		Serial.print(" d= ");
		Serial.print(fResult);
		#endif
	}

	if(def_params.containsKey("offset"))
	{
		double fOff = def_params["offset"].as<double>();
		fResult += fOff;
	}
		
	#if SHOW_DEBUG_EXTRACT_DATA
	Serial.printf("after %lf\n", fResult);
	#endif

	return fResult;
}


uint64_t SmkParser::getbits(uint64_t value, uint64_t offset, unsigned n)
{
	const unsigned max_n = CHAR_BIT * sizeof(uint64_t);
	if (offset >= max_n)
		return 0; /* value is padded with infinite zeros on the left */
	//Serial.printf("  op: %lu >>= %lu -> ", value, offset);
	value >>= offset; /* drop offset bits */
	//Serial.printf(" value>>%lu  ", value);
	if (n >= max_n)
		return value; /* all  bits requested */
	const uint64_t mask = ((uint64_t)1 << n) - 1; /* n '1's */
	//Serial.printf(  "value masked %lu  mask =%lu   n:%lu\n\r", value & mask, mask, n);
	return value & mask;
}

bool SmkParser::getlogfromdict(JsonObject def_params, JsonVariant ret_result, apiframe &packet, uint16_t idx, String &type)
{
	bool ret=false;
	
	String pathDict = "/type/"+ type +"/" + def_params["dict"].as<String>();
	#if SHOW_DEBUG_DICTIONARY
		Serial.printf("Path json log file: ");
		Serial.print(pathDict);
	#endif

	//ret_result["tttt"] = "tttt";
	
	DynamicJsonDocument jsondict(SIZE_OF_DYNAMIC_JSON_FILE);
	DeserializationError error = deserializeJson(jsondict, readFile(pathDict.c_str()));
    if(error)
    {
      #if SHOW_DEBUG_DICTIONARY
        Serial.print("  error during deserialization of ");
        Serial.println(pathDict);
      #endif
        return false;
    }
	jsondict.shrinkToFit();




	value_converter_union v = extractByteFromApiPacket(packet, 11, sizeof(uint32_t));
	time_t rawtime;
	struct tm * timeinfo;
	//rawtime=*((uint32_t*) (&packet[11]))+946684800; //to convert python time into UTC
	rawtime=v.uBBBB+946684800; //to convert python time into UTC
	timeinfo = localtime (&rawtime);
	ret_result["time"]=asctime(timeinfo);

	for(auto i: jsondict.as<JsonObject>())
	{
		JsonObject logType = i.value().as<JsonObject>();

		bool event_parsed = false;

		byte code = logType["c"];
		if(code == packet[idx])
		{
			#if SHOW_DEBUG_DICTIONARY
			Serial.printf("Event code: %03d -> ", code);
			Serial.println(i.key().c_str());
			#endif
			ret_result["event"] = i.key().c_str();
			event_parsed = true;



			if(logType.containsKey("dict"))
			{
				auto subdict = logType["dict"].as<JsonObject>();
				bool found = false;
				value_converter_union v = extractByteFromApiPacket(packet,idx+1, 2);
				Serial.printf("idx+1 %u\r\n", idx+1);
				Serial.println("Sub-dictionnary");
				printApiPacket(packet);
				Serial.println();

				for(auto s:subdict)
				{
					Serial.printf(", compare %u == %u\r\n", s.value().as<uint16_t>(), v.uBB[0]);
					if(s.value().as<uint16_t>() == v.uBB[0])
					{
						ret_result["value"]=s.key();
						Serial.println("-------sub dict found---------------");
						found = true;
						ret = true;
						break;
					}
				}
				if(!found)
				{
					ret_result["value"]="sub dictionnary code not available";
					break;
				}
			}
			else
			{
				#if SHOW_DEBUG_DICTIONARY
				Serial.printf("Event code: %03d ", code);
				Serial.println("unavailable");
				#endif				
				value_converter_union v = extractByteFromApiPacket(packet,idx+1, 2);
				ret_result["value"] = v.uBB[0];
				ret = true;
			}



		}
		if(event_parsed) break;
		
	}
	
	return ret;
}

bool SmkParser::getErrorFromDict(JsonObject def_params, JsonVariant ret_result, apiframe packet, uint16_t idx, String& stype)
{
	bool ret=false;

	String pathDict = "/type/"+ stype +"/"+ def_params["table"].as<String>();


	DynamicJsonDocument error_dict(SIZE_OF_DYNAMIC_JSON_FILE/2);
	deserializeJson(error_dict, readFile(pathDict.c_str()));


	bool found = false;
	byte code = packet[idx];




	for(auto e:error_dict.as<JsonObject>())
	{
		if(e.value() == code)
		{
		  #if SHOW_DEBUG_DICTIONARY
		  	Serial.printf("Error code: %03d ", code);
			Serial.print(e.key().c_str());
    	  #endif
			ret_result["value"]=e.key();
			found = true;
			ret = true;
			break;
		}
	}
	if(!found)
	{
		  #if SHOW_DEBUG_DICTIONARY
		  	Serial.printf("Error dictionnary unavailable. code:%d\r\n",code);
    	  #endif
		ret_result["error"]="error code not found";
		ret_result["code"]=packet[idx];
	}
	return ret;
}
