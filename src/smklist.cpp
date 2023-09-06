#include <smklist.h>


//------------------------------------------------------------------------------------------------------------
SmkList::SmkList()
{
    //Serial.println("*******************************************************");
    
}

mesh_t::iterator SmkList::addNode(JsonObject node)
{
    mesh_t::iterator  ret;
    for(auto jp:node) ret = addNode(jp);
    ret->second.pjson_type = (*type_json)["gateway"].as<JsonVariant>();
    return ret;
}

mesh_t::iterator SmkList::addNode(JsonPair node)
{
    mesh_t::iterator ret = pool.end();
    SmkNode x;//create new node
    JsonObject j = node.value().as<JsonObject>();

    int m =  macString2Umac(node.key().c_str());
    auto founded = pool.find(m);

    x.mac.address = macString2Umac(node.key().c_str());

    //String node_name = kv.key().c_str();
    x.name = j["name"].as<String>();

    //load of group
    if(j.containsKey("group")) x.group = j["group"].as<String>();
    else x.group = "";
    
    x.type = j["type"].as<String>();
    addType(x.type);
    x.enabled = (j.containsKey("enabled")) ? j["enabled"].as<bool>(): true;
    x.local =(j.containsKey("local"))?j["local"].as<bool>():false;
    x.sample_rate = (j.containsKey("srate"))?j["srate"].as<int>():0;
    x.sample_rate = (j.containsKey("sr"))?j["sr"].as<int>():0;        
    x.priority = (j.containsKey("priority")) ? j["priority"].as<int>() : 0;

    if(j.containsKey("settings"))
    {
        for(auto s: j["settings"].as<JsonObject>())
        {
            setting_t t;
            t.name = s.key().c_str();
            t.value = s.value()["value"].as<uint32_t>();
            t.type = (s.value().containsKey("type"))?s.value()["type"].as<String>():"int32";
            x.settings.push_back(t);
        }
    }
    if(j.containsKey("config"))
    {
        //Serial.printf("  %s",KRED);
        int idx_reg = 0;
        for(auto y : j["config"].as<JsonArray>())
        {

            x.config.push_back(y.as<int64_t>());
            //Serial.printf("cfg: %d ",y.as<uint64_t>());
        } 
        //Serial.print(KYEL);
        //Serial.println();
    }
    else if((*type_json).containsKey(x.type))
    {
        //Serial.printf("%s type exist\n", KGRN);
        if((*type_json)[x.type].containsKey("config"))
        {
            //Serial.printf("%s config exist\n", KGRN);

            for(JsonObject c : (*type_json)[x.type]["config"].as<JsonArray>())
            {
                for(JsonPair pair : c)
                {
                  JsonObject cdetails = pair.value().as<JsonObject>();
                  //Serial.printf("%s name exist", pair.key());
                  if(cdetails.containsKey("default"))
                    x.config.push_back(cdetails["default"].as<int64_t>());
                  else
                    x.config.push_back(0);
                }
            }
        }       
        

    }          

    //Internal variable of node
    x.nb_retry_count=0;
    x.dataValid=0;
    x.otaStep = STEP_INIT;


    pool[x.mac.address]= x;


    //add node to priority list if needed
    if( x.priority > 0) toPollFaster.push_back(find(x.mac.address));           



    ret = find(x.mac.address);

    #if SHOW_LOAD_NODE_DEBUG
      Serial.print("  new node:");
      Serial.print(node.key().c_str());
      Serial.print("  name:");
      Serial.print(x.name);
      Serial.print("  type:");
      Serial.println(x.type);
    #endif

    return ret;
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
    return loadNodes(nodes_json.as<JsonVariant>());
}

//------------------------------------------------------------------------------------------------------------
bool SmkList::loadNodes(JsonVariant nodes_json)
{
  #if SHOW_LOAD_NODE_DEBUG
    Serial.printf("--------------------------------\r\n");
    Serial.printf("   node list\r\n");
    Serial.printf("--------------------------------\r\n");
  #endif

   Serial.print(KYEL);
    Serial.println("-------CONFIG------");
    for (JsonPair kv : nodes_json.as<JsonObject>()) 
    {
        addNode(kv);
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
bool SmkList::writeNodeListToFile(const char* file)
{
    if (!getConditionDrive())
    {
        Serial.println("Error system not ready to write");
        return false;
    }    
    SPIFFS.remove(file);

    DynamicJsonDocument nodeListJson(SIZE_OF_DYNAMIC_JSON_FILE);


    Serial.println("------------ nodes that will be save -------------");
    Serial.println(ESP.getFreeHeap());

    
    String to_write="";
    if(pool.size()) //if there is node
    {
        //for each node, form back a json node
        for(auto n: pool)
        {
            String smac = n.second.getMacAsString();
            nodeListJson[smac]["name"] = n.second.name;
            if(n.second.group!="") nodeListJson[smac]["group"] = n.second.group;
            nodeListJson[smac]["type"] = n.second.type;
            //write only if false, otherwise, consider enabled anyway
            if(!n.second.enabled) nodeListJson[smac]["enabled"] = n.second.enabled;
            if(n.second.local)   nodeListJson[smac]["local"]=true;
            if(n.second.sample_rate!=0)nodeListJson[smac]["sr"] = n.second.sample_rate;
            if(n.second.priority>0)nodeListJson[smac]["priority"] = n.second.priority;


            if(n.second.settings.size()>0)
            {
                nodeListJson[smac].createNestedObject("settings");
                for(auto s:n.second.settings)
                {
                    nodeListJson[smac]["settings"].createNestedObject(s.name);
                    nodeListJson[smac]["settings"][s.name].createNestedObject("value");
                    nodeListJson[smac]["settings"][s.name]["value"] = n.second.getSetting(s.name);
                }   
            }

            for(int index = 0; index < n.second.config.size(); index++) {
                nodeListJson[smac]["config"].add( n.second.config[index]);
            }
                           
            Serial.println("  mac: " + n.second.getMacAsString());
            Serial.println("  name: " + n.second.name);
            Serial.println("  group: " + n.second.group);
            Serial.println("  type: " + n.second.type);
            Serial.print  ("  mb_add: "); Serial.println(n.second.getSetting("offset"));
            Serial.print  ("  sample_rate: "); Serial.println(n.second.sample_rate);
            Serial.print  ("  priority: "); Serial.println(n.second.priority);
            
        }
        nodeListJson.shrinkToFit();
        // Serialize JSON to file
        if (serializeJson(nodeListJson, to_write) == 0) 
        {
          #if SHOW_NODE_DEBUG
            Serial.println(F("Failed to write to file"));
          #endif
          return false;
        }
        nodeListJson.clear(); // to free RAM
    }
    else to_write="{}"; //if there is no node
    

    writeDirectlyToFile("/nodes.json", to_write.c_str());
    Serial.println(to_write);
    nodeListJson.clear();
    
    return true;
}



//------------------------------------------------------------------------------------------------------------
JsonVariant SmkList::getTypeJsonVariant()
{
    return (*type_json).as<JsonVariant>();
}

//------------------------------------------------------------------------------------------------------------
bool SmkList::loadParamFiles()
{
    bool ret = true;
    DynamicJsonDocument json_buffer(SIZE_OF_DYNAMIC_JSON_FILE);
    type_json = new DynamicJsonDocument (SIZE_OF_DYNAMIC_JSON_FILE);
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

    }
    //shrink reduce heap memory usage 
    json_buffer.shrinkToFit();

    return true;
}

//------------------------------------------------------------------------------------------------------------
bool SmkList::addType(String type)
{
    //TODO: optionnal, test could be done to verify minimum requirement of type since user could use this function

    //check if type doesn't exist
    if(!(*type_json).containsKey(type))
    {      
        DynamicJsonDocument new_type(SIZE_OF_DYNAMIC_JSON_FILE);
        String path_type_file = "/type/" + type + "/parser.json";
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

      (*type_json)[type].set(new_type);

      #if SHOW_LOAD_NODE_DEBUG
        Serial.printf("--------------------------------\r\n");
        Serial.print("  type: ");
        Serial.print(type);
        Serial.print(" loaded    size: ");
        Serial.println(new_type.memoryUsage());
        String sNew_type="";
        Serial.println(sNew_type);
        Serial.printf("--------------------------------\r\n");
        Serial.print("+ new inserted type: ");
        Serial.println(type);
        sNew_type="";
        serializeJson((*type_json).as<JsonVariant>(), sNew_type);
        Serial.println(sNew_type);
        Serial.println("  ========================");
      #endif
    }
    return true;
}

//------------------------------------------------------------------------------------------------------------
bool SmkList::addType(String type, String json_string)
{
    DynamicJsonDocument new_type(SIZE_OF_DYNAMIC_JSON_FILE);
    
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
    
    DeserializationError error = deserializeJson((*type_json), json_string);
    if(error)
    {
      #if SHOW_LOAD_NODE_DEBUG
        Serial.print("  error during deserialization of types source");
      #endif
      ret = true;
    }
    else (*type_json).shrinkToFit();

    return ret;
}


//------------------------------------------------------------------------------------------------------------
void SmkList::assignTypeToNode()
{
    auto n = pool.begin();
    for(int i=0; i< pool.size(); i++)
    {
        if((*type_json).containsKey(n->second.type)) 
          n->second.pjson_type=(*type_json)[n->second.type].as<JsonVariant>();
        else
          Serial.println("Type to assign does't exist");
    }  
}
//------------------------------------------------------------------------------------------------------------
int SmkList::getNbBytePacket(mesh_t::iterator  pNode, String tag)
{
    int param_pos=0;
    int param_size=0;

    //Serial.printf("%s  %s  %s  ",KYEL,pNode->second.type,tag);
    if(!(*type_json)[pNode->second.type]["parser"].containsKey(tag)) return -1;
    for(auto p:(*type_json)[pNode->second.type]["parser"][tag]["params"].as<JsonObject>())
    {
        auto idx = p.value().as<JsonObject>()["pos"];
        //find the last param, normaly the last but we take no chance
        if(idx[0].as<int>() > param_pos)
        {
            param_pos  = idx[0].as<int>();
            param_size = idx[1].as<int>();
        }
    }
    
    int nbByte = (param_pos+param_size)/8;
    if((param_pos+param_size)%8) nbByte++;
    //Serial.printf("%s  %d\n",KYEL,nbByte);
    return nbByte;
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

