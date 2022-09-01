#include "firmware.h"

#define PRT(x) (Serial.print(x))
#define PRTF(x, z) (Serial.printf(x, z))
#define PRTF2(x, y, z) (Serial.printf(x, y, z))
#define PRTF4(x, z1, z2, z3, z4) (Serial.printf(x, z1, z2, z3, z4))
#define PRTLN(x) (Serial.println(x))

#define SHOW_METADATA_PACKET false

String firmware_t::filename;

bool firmware_t::open(String fname)
{
    binary_length = 0;
    metadata_offset = -1;

    // String f_uf2 = "/" + filename;
    uf2 = SPIFFS.open(fname);

    PRTLN("---------------------------");
    PRTF(">filename: %s\n", uf2.name());
    PRTF("  size: %d\n", uf2.size());

    loadMetaDataBlock();

    max_progress = nbBlock.uint32b;

    return uf2;
}

bool firmware_t::loadMetaDataBlock()
{
    bool ret=false;

    metadata_offset = uf2.size() - 512;
    if (uf2.seek(metadata_offset))
    {
        uf2.read(uf2Block, SIZE_BLOCK);

        PRTF("  metadata_offset: %d\n", metadata_offset);
        PRTF4("  magic word: %02X%02X%02X%02X\n", uf2Block[OFFSET_METADATA_MAGICWORD], uf2Block[OFFSET_METADATA_MAGICWORD + 1], uf2Block[OFFSET_METADATA_MAGICWORD + 2], uf2Block[OFFSET_METADATA_MAGICWORD + 3]);
        for (int i = 0; i < 4; i++)
            magic_word.uint8b[i] = uf2Block[OFFSET_METADATA_MAGICWORD + i];

        // extract nb block from metadata
        nbBlock.uint32b = uf2Block[OFFSET_NB_BLOCK] + (uf2Block[OFFSET_NB_BLOCK + 1] << 8);
        PRTF("  nbBlock: %d\n", nbBlock.uint32b);

        // extract length of binary from metadata
        binary_length = uf2Block[OFFSET_BIN_LENGHT] + (uf2Block[OFFSET_BIN_LENGHT + 1] << 8) + ((uf2Block[OFFSET_BIN_LENGHT + 2] << 16) + (uf2Block[OFFSET_BIN_LENGHT + 3] << 24));
        PRTF("  length: %d\n", binary_length);

        if(getType() == EVM)
        {
            evm_len.uint16b[0] = uf2Block[OFFSET_EVM_LEN] + (uf2Block[OFFSET_EVM_LEN + 1] << 8);
            PRTF("  evm_len:        %d\n",  evm_len.uint16b[0]);
            evm_main_entry.uint16b[0] = uf2Block[OFFSET_EVM_MAIN_ENTRY] + (uf2Block[OFFSET_EVM_MAIN_ENTRY + 1] << 8);
            PRTF("  evm_main_entry: %d\n",  evm_main_entry.uint16b[0]);
            evm_crc.uint16b[0] = uf2Block[OFFSET_EVM_CRC] + (uf2Block[OFFSET_EVM_CRC + 1] << 8);
            PRTF("  evm_crc:        %d\n",  evm_crc.uint16b[0]);
        }
        return true;

#if SHOW_METADATA_PACKET
        for (int i = 0; i < 32; i++)
        {
            for (int j = 0; j < 16;)
            {
                PRTF4("%02X%02X%02X%02X ", uf2Block[i * 16 + j], uf2Block[i * 16 + j + 1], uf2Block[i * 16 + j + 2], uf2Block[i * 16 + j + 3]);
                j += 4;
            }
            PRTLN("");
        }
#endif
    }
    return ret;
}

int32_t firmware_t::checkNextChunk()
{
    int32_t nb_to_transfert = SIZE_DATA_PER_PACKET; // default

    if (current_block == nbBlock.uint32b) // if last block
    {
        need_to_read_block = false;
        // int32_t next_offset_block_chunk = offset_block_chunk + SIZE_DATA_PER_PACKET;
        int32_t remaining = binary_length % SIZE_DATA_PER_BLOCK - offset_block_chunk * SIZE_DATA_PER_PACKET;
        if (remaining < SIZE_DATA_PER_BLOCK)
        {
            nb_to_transfert = remaining % SIZE_DATA_PER_PACKET;
        }
        if (remaining <= SIZE_DATA_PER_PACKET)
        {
            end_condition = true;
            nb_to_transfert = 0;
        }
    }
    else // not last block
    {
        // offset_block_chunk += SIZE_DATA_PER_PACKET;
        if (offset_block_chunk + SIZE_DATA_PER_PACKET >= SIZE_DATA_PER_BLOCK)
        {
            need_to_read_block = true;
            offset_block += SIZE_BLOCK;
            current_block++;
            // current_progress++;
        }
        else
        {
            need_to_read_block = false;
        }
    }

    return nb_to_transfert;
}


