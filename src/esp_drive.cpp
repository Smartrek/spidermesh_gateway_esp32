/*

    Writting to flash using file system

    To implement or look at at least:

    Brownout Detection Interrupt
    https://www.esp32.com/viewtopic.php?t=6855


    Link usefull for function used bellow
    https://www.arduino.cc/en/Tutorial/StringIndexOf


*/
#include <esp_drive.h>
#include <spidermeshapi.h>

#ifndef SHOW_DEBUG_DRIVE
#define SHOW_DEBUG_DRIVE true
#endif
#ifndef SHOW_SPIFFS_CONTENT
#define SHOW_SPIFFS_CONTENT false
#endif

uint8_t systemOkToWrite = false;
const char *node_list_file = "/nodes.json";
const char *node_type_file = "/node_type.txt";

#define SD_CS 21

void setConditionStateWriteFile(bool state)
{
    systemOkToWrite = state;
}

void initDrive()
{
    // Initialize SPIFFS managment of files from spi chip
    if (SPIFFS.begin())
    {
        Serial.println("Mounting SPIFFS  successfully.");
    }
    else
    {
        Serial.println("An Error has occurred while mounting SPIFFS");
        return;
    }

#if SHOW_SPIFFS_CONTENT
    File root;
    root = SPIFFS.open("/");
    printDirectory(root, 0);
#endif

#if SD_CARD_ENABLED == 1
    // Initialize SD card
    SD.begin(SD_CS);
    if (!SD.begin(SD_CS))
    {
        Serial.println("Card Mount Failed");
        return;
    }
    uint8_t cardType = SD.cardType();
    if (cardType == CARD_NONE)
    {
        Serial.println("No SD card attached");
        return;
    }
    Serial.println("Initializing SD card...");
    if (!SD.begin(SD_CS))
    {
        Serial.println("ERROR - SD card initialization failed!");
        return; // init failed
    }
#endif
}


std::vector<String> getTypeNode()
{
    std::vector<String> ret;

    File root = SPIFFS.open("/");
    File file = root.openNextFile();

    while(file){
        String fname = file.path();
        String sub = fname.substring(1,5);
        if(sub == "type")
        {
            fname = fname.substring(6);
            int len = fname.length();

            String type = fname.substring(0,len-12);

            ret.push_back(type);
            Serial.print("type: ");
            Serial.println(type);

        }
        file = root.openNextFile();
    }
    return ret;
}

void printDirectory(File dir, int numTabs)
{
    while (true)
    {

        File entry = dir.openNextFile();
        if (!entry)
        {
            // no more files
            break;
        }
        for (uint8_t i = 0; i < numTabs; i++)
        {
            Serial.print('\t');
        }
        Serial.print(entry.name());
        if (entry.isDirectory())
        {
            Serial.println("/");
            printDirectory(entry, numTabs + 1);
        }
        else
        {
            // files have sizes, directories do not
            Serial.print("\t\t");
            Serial.println(entry.size(), DEC);
        }
        entry.close();
    }
}

std::vector<String> splitNodeInfo(String in)
{
    std::vector<String> ret;

    // extract name
    int idx_end_name = in.indexOf(';');
    if (idx_end_name == 0 || in.length() == 0)
    {
        Serial.println("--Error in splitNodeInfo");
        ret.push_back("");
        ret.push_back("");
        ret.push_back("");
        return ret;
    }

    String name = cleanString(in.substring(0, idx_end_name));

    // extract mac
    idx_end_name++;
    int idx_end_mac = in.indexOf(';', idx_end_name);
    String mac = cleanString(in.substring(idx_end_name, idx_end_mac));

    // extract type
    String type = cleanString(in.substring(++idx_end_mac));

    Serial.print("--FILE NODE LIST Radio name: ");
    Serial.print(name);
    Serial.print("  mac: ");
    Serial.print(mac);
    Serial.print("  type: ");
    Serial.println(type);

    ret.push_back(name);
    ret.push_back(mac);
    ret.push_back(type);

    return ret;
}

//------------------------------------------------------------------------------
String splitLine(String in, int *idx)
{
    int idx_start = *idx;
    int count = in.length();
    if (idx_start >= count)
        return "";

    int idx_end = in.indexOf('\n', idx_start);

    *idx = idx_end + 1;
    return in.substring(idx_start, idx_end - 1);
}

//------------------------------------------------------------------------------
bool splitMac(String s_mac, uint8_t *mac)
{
    int idx_mac = 0;
    for (int i_loop = 0; i_loop < 4; i_loop++)
    {
        int pos = s_mac.indexOf('.', idx_mac);

        if (i_loop == 0 && pos == 0)
        {
            Serial.println("error no mac");
            return false;
        }

        String number = s_mac.substring(idx_mac, pos);

        idx_mac = pos + 1;
        int x = number.toInt();

        if (x < 256)
            mac[i_loop] = (uint8_t)x;
        else
        {
            Serial.println("MAC invalid");
            return false;
        }
    }
    return true;
}

//------------------------------------------------------------------------------
String readNodeListFile()
{
    File fileNodeList = SPIFFS.open(node_list_file);
    if (!fileNodeList)
    {
        Serial.println("Error while loading the node list file");
        fileNodeList.close();
        return "";
    }
    Serial.println("Succes reading of the node list file");

    String content = "";
    while (fileNodeList.available())
    {
        char c = fileNodeList.read();
        content += c;
    }
    fileNodeList.close();
    return content;
}

//------------------------------------------------------------------------------
bool writeDirectlyToFile(char *name, String content)
{
    if (!systemOkToWrite)
    {
        Serial.println("Error system not ready to write");
        return false;
    }
    bool ret = true;

    //Serial.println("System ok to write");
    //Serial.printf("Writing file name: %s\r\ncontent:\r\n%s\r\n", name, content);

    File f = SPIFFS.open(name, FILE_WRITE);
    if (!f)
    {
        Serial.println("- failed to open file for writing");
        f.close();
        return false;
    }

    // Ecriture du contenu

    // ecriture en memoire
    if (f.print(content))
    {
        Serial.println("- file written");
    }
    else
    {
        Serial.println("- frite failed");
    }

    // fermeture du fichier
    f.close();

    return ret;
}

// Write the sensor readings on the SD card
void logSDCard()
{
    /*
  dataMessage = String(readingID) + "," + String(dayStamp) + "," + String(timeStamp) + "," +
                String(temperature) + "\r\n";
  Serial.print("Save data: ");
  Serial.println(dataMessage);
  appendFile(SD, "/data.txt", dataMessage.c_str());
  */
}

// Write to the SD card (DON'T MODIFY THIS FUNCTION)
void writeFile(fs::FS &fs, const char *path, const char *message)
{
    Serial.printf("Writing file: %s\n", path);

    File file = fs.open(path, FILE_WRITE);
    if (!file)
    {
        Serial.println("Failed to open file for writing");
        return;
    }
    if (file.print(message))
    {
        Serial.println("File written");
    }
    else
    {
        Serial.println("Write failed");
    }
    file.close();
    Serial.println("File closed");
}

// Append data to the SD card (DON'T MODIFY THIS FUNCTION)
void appendFile(fs::FS &fs, const char *path, const char *message)
{
    Serial.printf("Appending to file: %s\n", path);

    File file = fs.open(path, FILE_APPEND);
    if (!file)
    {
        Serial.println("Failed to open file for appending");
        return;
    }
    if (file.print(message))
    {
        Serial.println("Message appended");
    }
    else
    {
        Serial.println("Append failed");
    }
    file.close();
}

//------------------------------------------------------------------------------
String readNodeTypeFile()
{
    File fileNodeType = SPIFFS.open(node_type_file);
    if (!fileNodeType)
    {
        Serial.println("Error while loading the node type file");
        return "";
    }
    Serial.println("Succes reading of the node type file");

    String content = "";
    while (fileNodeType.available())
    {
        char c = fileNodeType.read();
        content += c;
    }
    return content;
}

//------------------------------------------------------------------------------
String readFile(String filename)
{
    File f = SPIFFS.open(filename);
    if (!f)
    {
        Serial.println("Error loading " + filename);
        return "";
    }

    String content = "";
    while (f.available())
    {
        char c = f.read();
        content += c;
    }
#if SHOW_DEBUG_DRIVE
    Serial.println(filename + " -> " + content);
#endif
    return content;
}

void listDir(char *dir, listFile_t *listWWW)
{

    File root = SPIFFS.open(dir);

    File file = root.openNextFile();

    while (file)
    {

        // Serial.print("FILE: ");
        // Serial.println(file.name());
        listWWW->push_back(file.name());
        file = root.openNextFile();
    }
}

bool validIpAddress(String toTest)
{
    std::vector<String> ipArray = splitString(toTest, '.');
    if (ipArray.size() == 4)
    {
        for (int i = 0; i < 4; i++)
        {
            // Serial.print("byte " + String(i) +": " + ipArray[i]);
            for (int j = 0; j < ipArray[i].length(); j++)
                if (isdigit(ipArray[i][j]) == false)
                {
                    Serial.println("IP add - error in one byte");
                    return false;
                }
            int byteIp = ipArray[i].toInt();
            if (byteIp > 255)
                return false;
        }
        // Serial.println();
    }
    return true;
}

bool convertToIpAddress(String ipStr, int *ip)
{
    std::vector<String> ipArray = splitString(ipStr, '.');
    if (ipArray.size() == 4)
    {
        for (int i = 0; i < 4; i++)
        {
            //Serial.print("b" + String(i) +": " + ipArray[i] + " ");
            ip[i] = ipArray[i].toInt();
            // ipIntArray[i]=ipArray[i].toInt();
        }
        Serial.println();
        // Serial.println(ipIntArray);
        return true;
    }
    return false;
}


bool isValidMac(String mac)
{
    std::vector<String> s_mac = splitString(mac, '.');

    if (s_mac.size() >= 3 && s_mac.size() <= 4)
    {
        /*
        for(String x : s_mac)
        {
            if(isDigit(x.c_str())) return false;
        }
        */
        return true;
    }
    return false;
}

void findFileTypeExtension(char *dir, listFile_t *listRetFile, String extension)
{

    File root = SPIFFS.open(dir);

    File file = root.openNextFile();

    Serial.print("findFileTypeExtension: ");
    Serial.println(extension);
    while (file)
    {

        // Serial.print("FILE: ");
        // Serial.println(file.name());
        String sFile = file.name();
        int len = sFile.length();
        if (len > 3)
        {
            String curr_extension = sFile.substring(len-3);
            if (curr_extension == extension)
            {
                String fname = sFile.substring(1);
                listRetFile->push_back(fname);
                Serial.print("filename: ");
                Serial.println(file.name());
            }
        }
        file = root.openNextFile();
    }
}