#ifndef PARAMETERS
#define PARAMETERS

#include <Arduino.h>
#include <ArduinoJson.h>

class Parameter{
    public:
        Parameter();
        void addToJson(JsonObject json);
};

#endif