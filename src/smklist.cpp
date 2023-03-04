#include <smklist.h>

DynamicJsonDocument SmkList::type_json(SIZE_OF_DYNAMIC_JSON_FILE);
mesh_t SmkList::pool;
DynamicJsonDocument SmkNode::config(1000);


//------------------------------------------------------------------------------------------------------------
SmkList::SmkList()
{
    conversion_type.insert(std::pair<String,int>("int", 0));
    conversion_type.insert(std::pair<String,int>("float", 3));   
}

//------------------------------------------------------------------------------------------------------------
bool SmkList::loadNodes(String nodes)
{
  #if SHOW_LOAD_NODE_DEBUG
    Serial.printf("--------------------------------\r\n");
    Serial.printf("   node list\r\n");
    Serial.printf("--------------------------------\r\n");
  #endif

    DynamicJsonDocument nodes_json(SIZE_OF_DYNAMIC_JSON_FILE);

    DeserializationError error = deserializeJson(nodes_json, nodes);
    if(error)
    {
      #if SHOW_LOAD_NODE_DEBUG
        Serial.print("  error during deserialization of string ");
      #endif
        return false;
    }


    for (JsonPair kv : nodes_json.as<JsonObject>()) 
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
        auto it = pool.insert(std::make_pair(x.mac.address,x));

        if(j.containsKey("config"))
        {
            it.first->second.config.set(j["config"].as<JsonObject>());
        }
        it.first->second.config.shrinkToFit();
        
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


  for(auto y:pool)
  {
      String pretty = "";
      serializeJsonPretty(y.second.config, pretty);
      Serial.println(pretty);
  }




  return true;
}

//------------------------------------------------------------------------------------------------------------
bool SmkList::loadNodes(JsonVariant nodes_json)
{
  #if SHOW_LOAD_NODE_DEBUG
    Serial.printf("--------------------------------\r\n");
    Serial.printf("   node list\r\n");
    Serial.printf("--------------------------------\r\n");
  #endif


    for (JsonPair kv : nodes_json.as<JsonObject>()) 
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
JsonVariant SmkList::getTypeJsonVariant()
{
    return type_json.as<JsonVariant>();
}

//------------------------------------------------------------------------------------------------------------
bool SmkList::loadParamFiles()
{
    bool ret = true;
    DynamicJsonDocument json_buffer(SIZE_OF_DYNAMIC_JSON_FILE);
    String file = "/nodes.json";
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
        ret= false;
    }
    DeserializationError error = deserializeJson(json_buffer, f);
    if(error)
    {
      #if SHOW_LOAD_NODE_DEBUG
        Serial.print("  error during deserialization of ");
        Serial.println(file);
      #endif
        ret = false;
    }
    f.close();


    loadNodes(json_buffer.as<JsonVariant>());
    json_buffer.clear(); //to free memory

    //for all node, check if type is not loaded, if not loaded, read json file to load it
    for(auto n:pool)
    {
        if(!type_json.containsKey(n.second.type))
        {      
            DynamicJsonDocument new_type(SIZE_OF_DYNAMIC_JSON_TYPE);
            new_type.clear();
            String path_type_file = "/type/" + n.second.type + "/parser.json";
          #if SHOW_LOAD_NODE_DEBUG
            Serial.print("Open file: ");
            Serial.println(path_type_file);
            Serial.println(readFile(path_type_file)); // only for debug
          #endif
            
            File f=SPIFFS.open(path_type_file);
            if(!f)
            {
              #if SHOW_LOAD_NODE_DEBUG
                Serial.print("  error while loading ");
                Serial.println(path_type_file);
              #endif
                continue;
                return false;
            }
            DeserializationError error = deserializeJson(new_type, f);
            if(error)
            {
              #if SHOW_LOAD_NODE_DEBUG
                Serial.print("  error during deserialization of ");
                Serial.println(path_type_file);
                continue;
              #endif
            }
            f.close();

            new_type.shrinkToFit();

          addType(n.second.type, new_type.as<JsonVariant>());           //insert the type in json format in the list of supported type
          n.second.pjson_type=type_json[n.second.type].as<JsonVariant>();   //get a pointer type for the node

          #if SHOW_LOAD_NODE_DEBUG
            Serial.printf("--------------------------------\r\n");
            Serial.print("  type: ");
            Serial.print(n.second.type);
            Serial.print(" loaded    size: ");
            Serial.println(new_type.memoryUsage());
            String sNew_type="";
            serializeJson(n.second.pjson_type, sNew_type);
            Serial.println(sNew_type);
            Serial.printf("--------------------------------\r\n");
            Serial.print("+ new inserted type: ");
            Serial.println(n.second.type);
            sNew_type="";
            serializeJson(type_json.as<JsonVariant>(), sNew_type);
            Serial.println(sNew_type);
            Serial.println("  ========================");
          #endif


        }
    }
    //shrink reduce heap memory usage 
    json_buffer.shrinkToFit();

    return true;
}

//------------------------------------------------------------------------------------------------------------
bool SmkList::addType(String type, JsonVariant src_type_json)
{
    //TODO: optionnal, test could be done to verify minimum requirement of type since user could use this function

    //check if type doesn't exist
    if(!type_json.containsKey(type))
      type_json[type].set(src_type_json);

    return true;
}

//------------------------------------------------------------------------------------------------------------
bool SmkList::addType(String type, String json_string)
{
    DynamicJsonDocument new_type(SIZE_OF_DYNAMIC_JSON_TYPE);
    
    DeserializationError error = deserializeJson(new_type, json_string);
    if(error)
    {
      #if SHOW_LOAD_NODE_DEBUG
        Serial.print("  error during deserialization of ");
        Serial.println(type);
      #endif
    }else new_type.shrinkToFit();

    return addType(type, new_type.as<JsonVariant>());           //insert the type in json format in the list of supported type

}

//------------------------------------------------------------------------------------------------------------
bool SmkList::loadTypes(String json_string)
{
    bool ret = true;
    DynamicJsonDocument new_type(SIZE_OF_DYNAMIC_JSON_TYPE);
    
    DeserializationError error = deserializeJson(type_json, json_string);
    if(error)
    {
      #if SHOW_LOAD_NODE_DEBUG
        Serial.print("  error during deserialization of types source");
      #endif
      ret = true;
    }
    else type_json.shrinkToFit();

    return ret;
}


//------------------------------------------------------------------------------------------------------------
void SmkList::assignTypeToNode()
{
    auto n = pool.begin();
    for(int i=0; i< pool.size(); i++)
    {
        if(type_json.containsKey(n->second.type)) 
          n->second.pjson_type=type_json[n->second.type].as<JsonVariant>();
        else
          Serial.println("Type to assign does't exist");
    }  
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



