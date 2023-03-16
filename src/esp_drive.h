#ifndef ESP_DRIVE_H
#define ESP_DRIVE_H
#include <Arduino.h>
#include <SPIFFS.h>
#include <HardwareProfile.h>
#include <vector>

typedef std::vector<String> listFile_t;

void initDrive();
void printDirectory(File dir, int numTabs);
void listDir(char * dir, listFile_t* listWWW);
bool validIpAddress(String toTest);
bool convertToIpAddress(String ipStr, int* ip);
std::vector<String> splitString(String in, char splitChar);
String splitLine(String in, int* idx);

void appendFile(fs::FS &fs, const char * path, const char * message);
void writeFile(fs::FS &fs, const char * path, const char * message);

String readNodeListFile();

std::vector<String> getTypeNode();
String readNodeTypeFile();
bool writeDirectlyToFile(char* name, String content);
String readFile(String filename);
String cleanString(String toClean);
bool isValidMac(String mac);
void setConditionStateWriteFile(bool state);
bool getConditionDrive();
void findFileTypeExtension(char *dir, listFile_t *listRetFile, String extension);



#endif