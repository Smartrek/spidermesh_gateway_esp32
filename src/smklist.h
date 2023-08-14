#ifndef ALINK_H
#define ALINK_H

#include <Arduino.h>
#include <list>
#include <SPIFFS.h>
#include <map>
#include <smknode.h>
#define ARDUINOJSON_USE_LONG_LONG 1
#include <ArduinoJson.h>

enum class ApiResponseCode
{
	TIMEOUT,
	SUCCESS,
	CORRUPT
};

typedef std::map<uint32_t, SmkNode> mesh_t;
typedef std::function<bool(mesh_t::iterator pNode, apiframe packet, ApiResponseCode success, String tag)> ExpectCallback;
typedef std::function<apiframe(mesh_t::iterator pNode)> RequestBuilderCallback;
typedef std::map<uint32_t, SmkNode>::iterator NodeIterator_t;
typedef std::vector<NodeIterator_t> PollingNodeList_t;

typedef struct
{
	mesh_t::iterator pNode;
	apiframe payload;
	ExpectCallback callback;
	String tag;
} MeshRequest_t;

class ParamList_t
{
public:
	ParamList_t(){};
	ParamList_t(String n_name, String param_name, String param_value)
	{
		mac = n_name;
		name = param_name;
		value = param_value;
	};
	String mac;
	String name;
	String value;
};

class SmkList
{
private:
	//DynamicJsonDocument type_json;
public:
	
	DynamicJsonDocument* type_json;
	// std::map<String, NodeType_t> nType;
	JsonVariant getTypeJsonVariant();

	mesh_t pool;

	PollingNodeList_t toPollFaster;
	int idxNodeToPollFast;


public:
	SmkList();

	bool insertNewNodeInList(std::vector<SmkNode> *node_list, String node_line);

	bool loadParamFiles();
	bool writeNodeListToFile(const char* file = "/nodes.json");
	bool loadNodes(JsonVariant nodes_json);
	bool loadNodes(String nodes);
	bool addType(String type);
	bool addType(String type, String json_string);
	bool loadTypes(String json_string);

	void assignTypeToNode();
	int getNbBytePacket(mesh_t::iterator  pNode, String tag="status");





	mesh_t::iterator find(uint32_t add)
	{
		for (auto x = pool.begin(); x != pool.end(); x++)
		{
			if (x->first == add)
				return x;
		}
		return pool.end();
	};

	//------------------------------------------------------------------------------

	// void save();
private:
	std::vector<ParamList_t> listParam;
	void update(ParamList_t p);

public:
	void addParamToUpdate(String node_name, String name, String value) { listParam.push_back(ParamList_t(node_name, name, value)); };

	mesh_t::iterator addNode(JsonPair node);
	mesh_t::iterator addNode(JsonObject node);
	String isMacExist(String mac);
	String isMacExist(uint32_t umac);

	bool remove(String name);
	bool remove(uint32_t umac) { return pool.erase(umac); };

	uint32_t macString2Umac(String mac);
	String mac2String(uint32_t add);
	String macInt2String(uint32_t add);

	void resetStep(bool force = false)
	{
		auto i = pool.begin();
		while (i != pool.end())
		{
			if ((i->second.otaStep != STEP_REJECTED && i->second.otaStep != STEP_COMPLETED) || force)
				i->second.otaStep = STEP_INIT;
			i->second.offset_chunk = 0;
			i->second.nbFailed = 0;
			i++;
			// Serial.printf("  reset node %s to %d\n",i.second.name.c_str(), i.second.otaStep);
		}
	};

	void resetOtaActiveFlags()
	{
		auto i = pool.begin();
		while (i != pool.end())
		{
			i->second.otaActive = false;
			i++;
		}
	};

	bool isStepComplete()
	{
		bool ret = true;
		auto i = pool.begin();
		while (i != pool.end())
		{
			if (i->second.otaActive && ((i->second.otaStep != STEP_DONE) && (i->second.otaStep != STEP_REJECTED) && (i->second.otaStep != STEP_COMPLETED)))
			// if ((i->second.otaStep != STEP_DONE) && (i->second.otaStep != STEP_REJECTED))
			{
				// Serial.println("Step Complete");
				ret = false;
			}
			i++;
		}
		return ret;
	};

	bool isThereAtLeastOneOk()
	{
		bool ret = false;
		auto i = pool.begin();
		while (i != pool.end())
		{
			if (i->second.otaStep != STEP_REJECTED)
			// if ((i->second.otaStep != STEP_DONE) && (i->second.otaStep != STEP_REJECTED))
			{
				// Serial.println("Step Complete");
				ret = true;
				break;
			}
			i++;
		}
		return ret;
	};

	bool isLastStepWasOk()
	{
		bool ret = true;
		auto i = pool.begin();
		while (i != pool.end())
		{
			if (i->second.otaActive && i->second.otaStep != STEP_DONE)
			{
				ret = false;
			}
			i++;
		}
		return ret;
	};

	bool enableOta(uint32_t mac)
	{
		bool ret = false;
		// pool[mac].otaActive = true;
		auto n = pool.begin();
		while (n != pool.end())
		{
			Serial.printf("compare mac: %06X with %06X\n", n->first, mac);
			if (n->first == mac)
			{
				n->second.otaActive = true;
				Serial.print("mac to update is: ");
				Serial.println(macInt2String(mac));
				ret = true;
			}
			n++;
		}
		return ret;
	};

	void resetLabels()
	{
		auto x = pool.begin();
		while (x != pool.end())
		{
			x->second.labelState.clear();
			x++;
		};
	};
	void setLabels(String msg)
	{
		auto x = pool.begin();
		while (x != pool.end())
		{
			x->second.labelState = msg;
			x++;
		};
	};
};

#endif