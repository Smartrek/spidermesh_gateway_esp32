#ifndef FIRMWARE_H
#define FIRMWARE_H

#include <Arduino.h>
#include <SPIFFS.h>
#include <FastCRC.h>
#include <ArduinoJson.h>
#include <list>
#include <utils.h>


#define UF2_MAGIC_START 0
#define OFFSET_BIN_LENGHT 12
#define OFFSET_SIZE 16
#define OFFSET_POS 20
#define OFFSET_NB_BLOCK 24

#define OFFSET_METADATA_MAGICWORD 32
#define OFFSET_DATA 32
#define OFFSET_EVM_LEN OFFSET_DATA + 24
#define OFFSET_EVM_CRC OFFSET_DATA + 26
#define OFFSET_EVM_MAIN_ENTRY OFFSET_DATA + 28

#define PACKET_RF_21B 0
#define PACKET_RF_72B 0x10
#define RF_PACKET_SIZE PACKET_RF_72B


#define SIZE_BLOCK 512

#define SIZE_DATA_PER_BLOCK 256
#define SIZE_PAGE 16


#if RF_PACKET_SIZE == PACKET_RF_21B
    #define NB_PAGE_PER_CRC_PACKET 4  
    #define SIZE_DATA_PER_PACKET 16
    #define NB_PAGE_PER_DATA_PACKET 1
    #define NB_SIZE_SCRATCH_CHUNK 16
    #define NB_WRITE_META_DATA 8
    #define SIZE_DATA_PER_METADATA_PACKET SIZE_DATA_PER_PACKET/2
#else
    #define NB_PAGE_PER_CRC_PACKET 16
    #define SIZE_DATA_PER_PACKET 64
    #define NB_PAGE_PER_DATA_PACKET 4
    #define NB_SIZE_SCRATCH_CHUNK 16
    #define NB_WRITE_META_DATA 4
    #define SIZE_DATA_PER_METADATA_PACKET SIZE_DATA_PER_PACKET
#endif

#define NB_PAGE_PER_BLOCK (SIZE_DATA_PER_BLOCK / SIZE_DATA_PER_PACKET)


enum firmware_type_t
{
    SMK900,
    EVM,
    JVM,
    HOST
};
union uf2_block_t
{
    byte bytes[512];
    uint32_t uints[128];
};

typedef std::pair<int32_t, uint16_t> missing_element_t;
typedef std::list<missing_element_t> missing_list_t;

class firmware_evm_t
{
public:
    File bin;
    String raw;

    uint16_t crc_returned;
    uint16_t crc_calculated;
    String filename;

    firmware_evm_t()
    {
        crc_returned = 0xFFFF;
        crc_calculated = 0;
    };

    bool open(String fname = "")
    {
        bool ret = false;
        if (fname == "")
            fname = filename;
        bin = SPIFFS.open(fname);
        raw = "";

        if (bin)
        {
            if (bin.size())
            {
                while (bin.available())
                {
                    raw += bin.read();
                };
                String mainEntry = raw.substring(raw.length() - 2);
                String raw = raw.substring(0, raw.length() - 2);

                uint16_t mEntry = mainEntry[1];
                mEntry <<= 8;
                mEntry += mainEntry[0];

                Serial.printf("Main entry: %04X\n", mEntry);

                crc_calculated = crc_compat_calculateCRC16Generic(raw, 0, 0x1021, 0, true);
                bin.seek(0);
                ret = true;
            }
        }
        else
        {
            Serial.println("error bin file");
        }
        return ret;
    };

    apiframe readNextChunk(int front = -1, int len = 64)
    {
        apiframe ret;
        int i = 0;
        if (front > 0)
            ret.push_back(front);
        while (bin.available() && i < len)
        {
            ret.push_back(bin.read());
            i++;
        }
        if (i % 2 == 0)
            ret.push_back(0);
        return ret;
    };

    int crc_compat_flipInt8(int poly)
    {
        int polyRev = 0;
        for (int i = 0; i < 8; i++)
        {
            int iInv = 7 - i;
            polyRev += (((poly >> iInv) & 0x01) << i);
        }
        return polyRev;
    }

    int crc_compat_flipInt16(int poly)
    {
        int polyRev = 0;
        for (int i = 0; i < 16; i++)
        {
            int iInv = 15 - i;
            polyRev += (((poly >> iInv) & 0x0001) << i);
        }
        return polyRev;
    }

    void crc_compat_flipBytes(unsigned char *bytes, int len)
    {
        if (bytes != 0)
        {
            for (int i = 0; i < len; i++)
            {
                bytes[i] = (unsigned char)crc_compat_flipInt8(((int)bytes[i]) & 0xFF);
            }
        }
    }

    String crc_compat_flipBytes(String in)
    {
        String ret;

        for (size_t i = 0; i < in.length(); i++)
        {
            ret += crc_compat_flipInt8(((int)in[i]) & 0xFF);
        }
        return ret;
    }

    int crc_compat_calculateCRC16Generic(String bytesIn, int initial16, int polynomial16, int finalXor16, bool doFlip)
    {
        String bytes = bytesIn;

        if (doFlip)
        {
            bytes = crc_compat_flipBytes(bytesIn);
        }
        int crc = initial16;
        for (size_t k = 0; k < bytes.length(); k++)
        {
            int b = bytes[k];
            for (int i = 0; i < 8; i++)
            {
                bool bit = ((b >> (7 - i) & 1) == 1);
                bool c15 = ((crc >> 15 & 1) == 1);
                crc <<= 1;
                if (c15 ^ bit)
                    crc ^= polynomial16;
            }
        }

        crc ^= finalXor16;

        crc &= 0xFFFF;

        if (doFlip)
        {
            crc = crc_compat_flipInt16(crc);
        }
        return crc;
    };
};

class firmware_t
{
public:
    File uf2;
    firmware_type_t type;
    u_bytes evm_len;
    u_bytes evm_main_entry;
    u_bytes evm_crc;

    byte uf2Block[512];
    static String filename;
    u_bytes nbBlock;
    int32_t binary_length;
    std::vector<int32_t> progress_per_step;
    missing_list_t missing_list;
    missing_list_t::iterator it_missing_list;
    int test_skip_scratch;

    int32_t metadata_offset;
    u_bytes magic_word;
    int32_t offset_block;
    int32_t offset_block_chunk;
    int32_t current_block;
    bool need_to_read_block;
    bool end_condition;
    int32_t start_page;
    int current_progress;
    int max_progress;

    bool open(String fname = filename);
    void close() { uf2.close(); };
    bool loadMetaDataBlock();

    void resetOffset()
    {
        current_block = 0;
        offset_block = 0;
        need_to_read_block = true;
        end_condition = false;
        start_page = 0;
    };

    bool checkNextBlock()
    {
        if (current_block <= nbBlock.uint32b)
        {

            return true;
        }
        return false;
    };

    bool readNextBlock()
    {
        if (!need_to_read_block)
            return false;

        offset_block_chunk = 0;
        if (checkNextBlock())
        {
            if (uf2.seek(offset_block))
            {
                uf2.read(uf2Block, SIZE_BLOCK);
                offset_block_chunk = 0;
                return true;
            }
        }
        return false;
    };
    bool readBlockContainingChunk(uint32_t idxChunk)
    {
        bool ret = false;
        int block = idxChunk / (NB_SIZE_SCRATCH_CHUNK);
        // Serial.printf("====block:%d\n", block);

        if (block != current_block)
        {
            if (uf2.seek(block * SIZE_BLOCK))
            {
                uf2.read(uf2Block, SIZE_BLOCK);
                ret = true;
            }
        }
        else
            ret = true;

        start_page = (idxChunk % NB_SIZE_SCRATCH_CHUNK);
        offset_block_chunk = (start_page * NB_SIZE_SCRATCH_CHUNK);

        return ret;
    };

    int32_t checkNextChunk();

    byte getChunkByte()
    {
        return uf2Block[OFFSET_DATA + offset_block_chunk++];
    };

    bool isEndCondition()
    {
        return (end_condition) ? true : false;
    }

    bool calcCrcPage(int block, uint32_t *crcValue)
    {
        int32_t offset = block * SIZE_BLOCK;
        FastCRC32 crc;
        bool ret = false;
        byte buf[SIZE_DATA_PER_BLOCK];

        if (uf2.seek(offset + OFFSET_DATA))
        {
            uf2.read(buf, SIZE_DATA_PER_BLOCK);
            *crcValue = crc.crc32(buf, SIZE_DATA_PER_BLOCK);

            // Serial.printf("CRC block:%d, offset:%d calc: %08X\n",block,  offset+OFFSET_DATA, *crcValue);
            ret = true;
        }
        return ret;
    }

    std::vector<uint32_t> buildCrcPacket()
    {
        std::vector<uint32_t> ret;
        start_page = current_block;

        for (int i = 0; i < 17; i++)
        {
            uint32_t current_crc = 0;

            if (!calcCrcPage(current_block++, &current_crc))
            {
                ret.clear();
                break;
            }

            ret.push_back(current_crc);

            if (current_block == nbBlock.uint32b)
            {
                end_condition = true;
                break;
            }
        }

        return ret;
    }

    int getType()
    {

#define MAGICWORD_SMK900 0x42424242
#define MAGICWORD_EVM 0x38335155
#define MAGICWORD_HOST 0x64646464

        if (magic_word.uint32b == MAGICWORD_SMK900)
        {
            type = SMK900;
            //Serial.println("SMK900");
        }
        else if (magic_word.uint32b == MAGICWORD_EVM)
        {
            type = EVM;
            //Serial.println("EVM");
        }
        else if (magic_word.uint32b == MAGICWORD_HOST)
        {
            type = HOST;
            //Serial.println("HOST");
        }
        return type;
    };

    bool isSmk900()
    {
        int t = getType();
        return (t == SMK900) ? true : false;
    };
    bool isVmMachine()
    {
        int t = getType();
        return (t == EVM) ? true : false;
    };
    bool isHost()
    {
        int t = getType();
        return (t == HOST) ? true : false;
    };

    void calculOfEstimatedTime(int32_t nbToUpdate)
    {
        current_progress = 0;
        max_progress = 1; // 1 so web page see something
        Serial.print("  Number of node to update: ");
        Serial.println(nbToUpdate);

        if (getType() == HOST)
            max_progress += nbToUpdate + 1;  // reset factory
        max_progress += nbToUpdate * 8;      // read version
        max_progress += nbToUpdate + 1;      // prime
        max_progress += nbBlock.uint32b * SIZE_DATA_PER_BLOCK / SIZE_DATA_PER_PACKET; // bulkupload
        progress_per_step.push_back(max_progress);

        max_progress += nbToUpdate + 1;                           // get missing flag
        max_progress += nbBlock.uint32b / NB_PAGE_PER_CRC_PACKET; // Broadcast CRC
        max_progress += nbToUpdate + 1;                           // CHECK CRC
        max_progress += nbToUpdate * NB_WRITE_META_DATA + 1;                       // META DATA
        max_progress += nbToUpdate * 4 + 1;                       // RESET_ON_SEEK
        max_progress += nbToUpdate + 1;                           // SEND MAGIC WORD
        progress_per_step.push_back(max_progress);
        if (getType() == HOST)
        {
            max_progress += nbBlock.uint32b * 4 + 1; // potia -> pyboard
            progress_per_step.push_back(max_progress);
        }
        else
        {
            max_progress += nbToUpdate * 4; // read version at the end
        }
    };

    bool validationUf2Integrity()
    {
        bool ret = false, error = false;
        uf2 = SPIFFS.open(filename.c_str());
        uf2_block_t b;

        // CHECK INTEGRITY OF UF2 BLOCK
        int i;
        for (i = 0; i < nbBlock.int32b + 1; i++)
        {
            uf2.seek(i * SIZE_BLOCK);

            int nb_read = uf2.read(b.bytes, SIZE_BLOCK);
            #if 0
            Serial.print(b.uints[0],HEX);
            Serial.print(" != ");
            Serial.println(0x0A324655,HEX);

            Serial.print(b.uints[127],HEX);
            Serial.print(" != ");
            Serial.println(0x0AB16F30,HEX);
            #endif

            if (b.uints[0] != 0x0A324655 || b.uints[127] != 0x0AB16F30)
            {
                Serial.println("error uf2 tag");
                error = true;
                break;
            }
            if(nb_read != SIZE_BLOCK)
            {
                if(i < nbBlock.int32b)
                {
                    Serial.println("error block uf2 size ");
                    Serial.println(nbBlock.int32b);
                    break;
                }
                else if(i == nbBlock.int32b && nbBlock.int32b != 511)
                {
                    Serial.print("error last block uf2 size ");
                    Serial.println(nbBlock.int32b);
                    break;
                }
            }
        }

        if (!error)
            ret = true;

        uf2.close();

        return ret;
    };
};

#endif