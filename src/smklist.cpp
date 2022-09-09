#include <smklist.h>




//------------------------------------------------------------------------------------------------------------
SmkList::SmkList(String flist, JsonVariant type_json_main_file)
{
    _fList = flist;
    conversion_type.insert(std::pair<String,int>("int", 0));
    conversion_type.insert(std::pair<String,int>("float", 3));   
    jsonType = type_json_main_file;
}

//------------------------------------------------------------------------------------------------------------
bool SmkList::loadNodes(const char* file)
{
    DynamicJsonDocument jsonFile(SIZE_OF_DYNAMIC_JSON_FILE);
  #if SHOW_LOAD_NODE_DEBUG
    Serial.print("Open file:");
    Serial.println(file);
    readFile(file);

  #endif
    File f=SPIFFS.open(file);
    if(!f)
    {
      #if SHOW_LOAD_NODE_DEBUG
        Serial.print("  error while loading ");
        Serial.println(file);
      #endif
        return false;
    }
    DeserializationError error = deserializeJson(jsonFile, f);
    if(error)
    {
      #if SHOW_LOAD_NODE_DEBUG
        Serial.print("  error during deserialization of ");
        Serial.println(file);
      #endif
        return false;
    }
    f.close();


    //JsonObject root = nodeListJson.as<JsonObject>();

    //DynamicJsonDocument nodeListJson(10000);


  #if SHOW_LOAD_NODE_DEBUG
    Serial.printf("--------------------------------\r\n");
    Serial.printf("   node list\r\n");
    Serial.printf("--------------------------------\r\n");
  #endif


    for (JsonPair kv : jsonFile.as<JsonObject>()) 
    {
        SmkNode x;//create new node
        JsonObject j = kv.value().as<JsonObject>();

        //String node_name = kv.key().c_str();
        x.name = j["name"].as<String>();

        //load of group
        if(j.containsKey("group")) x.group = j["group"].as<String>();
        else if(j.containsKey("trail")) x.group = j["trail"].as<String>(); //for legacy snofi
        else x.group = "";
        
        //mac address
        x.mac.address = macString2Umac(kv.key().c_str());
        x.type = j["type"].as<String>();
        x.enabled = (j.containsKey("enabled")) ? j["enabled"].as<bool>(): true;
        x.local =(j.containsKey("local"))?j["local"].as<bool>():false;
        x.sample_rate = (j.containsKey("srate"))?j["srate"].as<int>():0;
        

        //Internal variable of node
        x.nb_retry_count=0;
        x.dataValid=false;
        #if MODBUS_REGISTER_ENABLED
        x.startAddresseModbusRegister = j["start_address"].as<int>();
        #endif

        x.elapse_time = 0;
        #if MODBUS_REGISTER_ENABLED
        x.valid_node_index_for_read_coil_bit = index_order_entry++;
        #endif

        x.otaStep = STEP_INIT;

        //addition of the node into the pool list
        pool.insert(std::make_pair(x.mac.address,x));
        #if SHOW_LOAD_NODE_DEBUG
          Serial.print("  new node:");
          Serial.print(kv.key().c_str());
          Serial.print("  name:");
          Serial.print(x.name);
          Serial.print("  type:");
          Serial.println(x.type);
        #endif
    }
  #if SHOW_LOAD_NODE_DEBUG
    Serial.printf("--------------------------------\r\n");
  #endif

  #if SHOW_LOAD_NODE_DEBUG || SHOW_MIN_DEBUG
    Serial.println("Node list file loaded successfully.");
  #endif




  return true;
}

//------------------------------------------------------------------------------------------------------------
bool SmkList::loadType(JsonVariant type_json)
{
    //for all node, check if type is not loaded, if not loaded, read json file to load it
    //shrink will be done in the main to reduce heap memory usage when all will be there
    for(auto smklist:pool)
    {
        if(!type_json.containsKey(smklist.second.type))
        {
            DynamicJsonDocument new_type(SIZE_OF_DYNAMIC_JSON_TYPE);
            String path_type_file = "/type/" + smklist.second.type + "/parser.json";
          #if SHOW_LOAD_NODE_DEBUG
            Serial.print("Open file: ");
            Serial.println(path_type_file);
            readFile(path_type_file); // only for debug
          #endif
            
            File f=SPIFFS.open(path_type_file);
            if(!f)
            {
              #if SHOW_LOAD_NODE_DEBUG
                Serial.print("  error while loading ");
                Serial.println(path_type_file);
              #endif
                return false;
            }
            DeserializationError error = deserializeJson(new_type, f);
            if(error)
            {
              #if SHOW_LOAD_NODE_DEBUG
                Serial.print("  error during deserialization of ");
                Serial.println(path_type_file);
              #endif
            }
            f.close();

            new_type.shrinkToFit();

            //JsonObject x = type_json.createNestedObject(smklist.second.type);
            
            type_json[smklist.second.type].set(new_type);
            smklist.second.pjson_type=type_json[smklist.second.type].as<JsonVariant>();



          #if SHOW_LOAD_NODE_DEBUG
            Serial.printf("--------------------------------\r\n");
            Serial.print("  type: ");
            Serial.print(smklist.second.type);
            Serial.print(" loaded    size: ");
            Serial.println(new_type.memoryUsage());
            String sNew_type="";
            serializeJson(smklist.second.pjson_type, sNew_type);
            Serial.println(sNew_type);// smklist.second.pjson_type["command"]["status"]["rqst"].as<String>());
            Serial.printf("--------------------------------\r\n");
          #endif


          #if SHOW_LOAD_NODE_DEBUG
            //Serial.println("  JSON polling command: "); printApiPacket(t.auto_polling_packet);
            Serial.print("+ new inserted type: ");
            Serial.println(smklist.second.type);
            Serial.println("  ========================");
          #endif
        }
    }

  #if SHOW_LOAD_NODE_DEBUG
    Serial.printf("--------------------------------\r\n");
  #endif
    return true;
}

//------------------------------------------------------------------------------------------------------------
bool SmkList::writeNodeListToFile(const char* file)
{
    

    DynamicJsonDocument nodeListJson(SIZE_OF_DYNAMIC_JSON_FILE);
    //DynamicJsonDocument nodeListJson(10000);


    Serial.println("------------ nodes that will be save -------------");

    
    String to_write="";
    if(pool.size()) //if there is node
    {
        //for each node, form back a json node
        for(auto n: pool)
        {
            String mac = mac2String(n.first);
            nodeListJson[mac]["name"] = n.second.name;
            nodeListJson[mac]["group"] = n.second.group;
            nodeListJson[mac]["type"] = n.second.type;
            nodeListJson[mac]["enabled"] = n.second.enabled;
          #if MODBUS_REGISTER_ENABLED
            nodeListJson[mac]["start_address"] = n.second.startAddresseModbusRegister;
          #endif
            nodeListJson[mac]["srate"] = n.second.sample_rate;

            Serial.println("  name: "  + n.second.name);
            Serial.println("  mac: "   + mac);
            Serial.println("  group: " + n.second.group);
            Serial.println("  type: "  + n.second.type );
          #if MODBUS_REGISTER_ENABLED
            Serial.println("  mb_add: " + n.second.startAddresseModbusRegister);
          #endif
            Serial.println("  srate: " + n.second.sample_rate);
        }

        // Serialize JSON to file
        if (serializeJson(nodeListJson, to_write) == 0) 
        {
          #if SHOW_LOAD_NODE_DEBUG
            Serial.println(F("Failed to write to file"));
          #endif
          return false;
        }
    }
    else{
        Serial.println("pool is empty");
        to_write="{}"; //if there is no node
    } 

    /*
    File f = SPIFFS.open(file, FILE_WRITE);
    if (!f) 
    {
      #if SHOW_LOAD_NODE_DEBUG
        Serial.println(F("Failed to create file"));
      #endif
        return false;
    }
    */
    
    if(writeDirectlyToFile("/nodes.json", to_write.c_str()))
    {
        //Serial.println("will write to nodes.json file-------------");
        Serial.println(to_write);
        
        nodeListJson.clear();
        //f.close();
    }
    else return false;
    
    return true;
}

//------------------------------------------------------------------------------------------------------------
bool SmkList::add(uint32_t mac, String type, bool local, String group, String name, bool enabled, uint16_t sample_rate) 
{

    #if MODBUS_REGISTER_ENABLED
    if(start_mod_add ==-1) return false;
    #endif

  #if 1 //SHOW_LOAD_NODE_DEBUG
    String node="mac:" + macInt2String(mac) + "  group:" + group + "  name:" + name + "  type:" + type + "  sample_rate:" + String(sample_rate);
    Serial.println("to add= " + node);
  #endif
  
    SmkNode x;

    //mac address
    uint32_t mac_32bit = mac;
    x.mac.address = mac_32bit;
    x.group = group;
    x.name = name;
    x.type = type;
    x.sample_rate = sample_rate;
    x.local=local;
  #if MODBUS_REGISTER_ENABLED
    x.startAddresseModbusRegister = start_mod_add;
  #endif  

    //if the node exist update it
    Serial.printf("Node ");
    Serial.print(x.getMacAsString());
    if ( find(mac_32bit) != pool.end() )
    {
        Serial.println(" have been updated");

        pool[mac_32bit]=x;
    }
    //otherwise add it
    else
    {
        Serial.println(" have been added to the list");
        pool.insert(std::pair<uint32_t, SmkNode>(mac_32bit, x));
    }












  /*
    #if MODBUS_REGISTER_ENABLED
    if(start_mod_add ==-1) return false;
    #endif

  #if SHOW_LOAD_NODE_DEBUG
    String node="mac:" + mac + "  group:" + group + "  name:" + name + "  type:" + type;
    Serial.println("to add= " + node);
  #endif
  
    SmkNode x;

    //mac address
    x.mac.address = macString2Umac(mac);
    x.group = group;
  #if MODBUS_REGISTER_ENABLED
    x.startAddresseModbusRegister = start_mod_add;
  #endif  



    //if the node exist update it
    if ( pool != alist.end() )
    {
        alist[name]=x;
    }
    //otherwise add it
    else
    {
        alist.insert(std::pair<String, SmkNode>(name, x));
    }

    Serial.println("new node added = " + node);
    writeNodeListToFile();
    //load();//to redo the code indexing and init of variables
    */
    return true;
}

//------------------------------------------------------------------------------------------------------------
String SmkList::isMacExist(uint32_t umac)
{
    if(pool.find(umac) != pool.end()) return mac2String(umac);
    return "";
}

//------------------------------------------------------------------------------------------------------------
String SmkList::isMacExist(String mac)
{
    return isMacExist(macString2Umac(mac));
}

//------------------------------------------------------------------------------------------------------------
uint32_t SmkList::macString2Umac(String mac)
{
    u_mac ret;
    std::vector<String> s_mac = splitString(mac, '.');

    if(s_mac.size()>3)
    {
        ret.bOff[byte0] = s_mac[3].toInt();
        ret.bOff[byte1] = s_mac[2].toInt();
        ret.bOff[byte2] = s_mac[1].toInt();
        ret.bOff[byte3] = s_mac[0].toInt();
    }
    else if(s_mac.size()==3)
    {
        ret.bOff[byte0] = s_mac[2].toInt();
        ret.bOff[byte1] = s_mac[1].toInt();
        ret.bOff[byte2] = s_mac[0].toInt();
        ret.bOff[byte3] = 0;
    }
    else ret.address = 0;

    return ret.address;
}

//------------------------------------------------------------------------------------------------------------
String SmkList::mac2String(uint32_t add)
{
    u_mac a;
    a.address = add;
    return String(a.bOff[byte3]) + "." + String(a.bOff[byte2]) + "." + String(a.bOff[byte1]) + "." + String(a.bOff[byte0]);
}

//------------------------------------------------------------------------------------------------------------
String SmkList::macInt2String(uint32_t add)
{
    u_mac a;
    a.address = add;
    return String(a.bOff[3]) + "." + String(a.bOff[2]) + "." + String(a.bOff[1]) + "." + String(a.bOff[0]);
}



