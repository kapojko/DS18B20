#include <stdint.h>
#include "DS18B20.h"

// ROM commands
#define CMD_SEARCH_ROM 0xF0
#define CMD_READ_ROM 0x33
#define CMD_MATCH_ROM 0x55
#define CMD_SKIP_ROM 0xCC
#define CMD_ALARM_SEARCH 0xEC

// Function commands
#define CMD_CONVERT_T 0x44
#define CMD_WRITE_SCRATCHPAD 0x4E
#define CMD_READ_SCRATCHPAD 0xBE
#define CMD_COPY_SCRATCHPAD 0x48
#define CMD_RECALL_E2 0xB8
#define CMD_READ_POWER_SUPPLY 0xB4

// Brief CRC table
// see https://github.com/libdriver/ds18b20/blob/master/src/driver_ds18b20.c
const uint8_t crcTable[256] = {
    0X00, 0X5E, 0XBC, 0XE2, 0X61, 0X3F, 0XDD, 0X83, 0XC2, 0X9C, 0X7E, 0X20, 0XA3,
    0XFD, 0X1F, 0X41, 0X9D, 0XC3, 0X21, 0X7F, 0XFC, 0XA2, 0X40, 0X1E, 0X5F, 0X01,
    0XE3, 0XBD, 0X3E, 0X60, 0X82, 0XDC, 0X23, 0X7D, 0X9F, 0XC1, 0X42, 0X1C, 0XFE,
    0XA0, 0XE1, 0XBF, 0X5D, 0X03, 0X80, 0XDE, 0X3C, 0X62, 0XBE, 0XE0, 0X02, 0X5C,
    0XDF, 0X81, 0X63, 0X3D, 0X7C, 0X22, 0XC0, 0X9E, 0X1D, 0X43, 0XA1, 0XFF, 0X46,
    0X18, 0XFA, 0XA4, 0X27, 0X79, 0X9B, 0XC5, 0X84, 0XDA, 0X38, 0X66, 0XE5, 0XBB,
    0X59, 0X07, 0XDB, 0X85, 0X67, 0X39, 0XBA, 0XE4, 0X06, 0X58, 0X19, 0X47, 0XA5,
    0XFB, 0X78, 0X26, 0XC4, 0X9A, 0X65, 0X3B, 0XD9, 0X87, 0X04, 0X5A, 0XB8, 0XE6,
    0XA7, 0XF9, 0X1B, 0X45, 0XC6, 0X98, 0X7A, 0X24, 0XF8, 0XA6, 0X44, 0X1A, 0X99,
    0XC7, 0X25, 0X7B, 0X3A, 0X64, 0X86, 0XD8, 0X5B, 0X05, 0XE7, 0XB9, 0X8C, 0XD2,
    0X30, 0X6E, 0XED, 0XB3, 0X51, 0X0F, 0X4E, 0X10, 0XF2, 0XAC, 0X2F, 0X71, 0X93,
    0XCD, 0X11, 0X4F, 0XAD, 0XF3, 0X70, 0X2E, 0XCC, 0X92, 0XD3, 0X8D, 0X6F, 0X31,
    0XB2, 0XEC, 0X0E, 0X50, 0XAF, 0XF1, 0X13, 0X4D, 0XCE, 0X90, 0X72, 0X2C, 0X6D,
    0X33, 0XD1, 0X8F, 0X0C, 0X52, 0XB0, 0XEE, 0X32, 0X6C, 0X8E, 0XD0, 0X53, 0X0D,
    0XEF, 0XB1, 0XF0, 0XAE, 0X4C, 0X12, 0X91, 0XCF, 0X2D, 0X73, 0XCA, 0X94, 0X76,
    0X28, 0XAB, 0XF5, 0X17, 0X49, 0X08, 0X56, 0XB4, 0XEA, 0X69, 0X37, 0XD5, 0X8B,
    0X57, 0X09, 0XEB, 0XB5, 0X36, 0X68, 0X8A, 0XD4, 0X95, 0XCB, 0X29, 0X77, 0XF4,
    0XAA, 0X48, 0X16, 0XE9, 0XB7, 0X55, 0X0B, 0X88, 0XD6, 0X34, 0X6A, 0X2B, 0X75,
    0X97, 0XC9, 0X4A, 0X14, 0XF6, 0XA8, 0X74, 0X2A, 0XC8, 0X96, 0X15, 0X4B, 0XA9,
    0XF7, 0XB6, 0XE8, 0X0A, 0X54, 0XD7, 0X89, 0X6B, 0X35,
};

static struct DS18B20_Platform platform;

static int resetAndPresence(void) {
    // Configure as output
    platform.gpioSwitch(DS18B20_GPIO_PULLUP_OUTPUT);

    // Master Tx Reset Pulse: Pull low (>1ms)
    platform.gpioSet(0);
    platform.delayUs(1500);

    // Release bus, DS18D20 detects the rising edge and waits 15-60 us
    platform.gpioSwitch(DS18B20_GPIO_FLOATING_INPUT);
    platform.delayUs(60);

    // Master Rx: Read the presence pulse (60-240us)
    platform.delayUs(5);
    int presence = platform.gpioGet();

    // Wait the end of initialization (Master Rx >=480us)
    platform.delayUs(600 - 60 - 5);

    return presence;
}

static void writeBit(int value) {
    // Configure as output
    platform.gpioSwitch(DS18B20_GPIO_PULLUP_OUTPUT);

    if (value == 0) {
        // Pull low for the whole slot
        platform.gpioSet(0);
        platform.delayUs(90); // 60us < Tx < 120us

        // Release bus
        platform.gpioSwitch(DS18B20_GPIO_FLOATING_INPUT);
    } else {
        // Pull low (>1us), DS18B20 samples after 15-45us after slot start
        platform.gpioSet(0);
        platform.delayUs(2);

        // Release bus
        platform.gpioSwitch(DS18B20_GPIO_FLOATING_INPUT);

        // Wait the end of slot
        platform.delayUs(90 - 5);
    }

    // Wait slot recovery time (>1us)
    platform.delayUs(2);
}

static int readBit(void) {
    // Configure as output
    platform.gpioSwitch(DS18B20_GPIO_PULLUP_OUTPUT);

    // Pull low (>1us as by datasheet)
    platform.gpioSet(0);
    platform.delayUs(2);

    // Release bus
    platform.gpioSwitch(DS18B20_GPIO_FLOATING_INPUT);

    // Wait befor reading (Tinit + Trc + Tsample must be less than 15 us)
    platform.delayUs(5);

    // Read bit
    int value = platform.gpioGet();

    // Wait the end of slot (>=60us duration)
    platform.delayUs(90 - 2 - 5);

    // Wait slot recovery time (>1us)
    platform.delayUs(2);

    return value;
}

static void writeByte(uint8_t value) {
    // Write LSB first
    for (int i = 0; i < 8; i++) {
        writeBit(value & 0x01);
        value >>= 1;
    }
}

static uint8_t readByte(void) {
    // Read LSB first
    uint8_t value = 0;
    for (int i = 0; i < 8; i++) {
        int b = readBit();        
        value >>= 1;
        value |= b << 7;
    }
    return value;
}

static uint8_t calcCrc(uint8_t *buf, int len) {
    uint8_t crc8 = 0;   
    for (int i = 0; i < len; i++) {
        crc8 = crcTable[crc8 ^ buf[i]];
    }

    return crc8;
}

static float extractTemp(uint8_t *scratchpad) {
    // Read raw temperature
    uint16_t tempRaw = scratchpad[0] | (scratchpad[1] << 8);

    // Extract resolution from configuration register (byte 4, bits 5-6)
    enum DS18B20_Resolution resolution = (enum DS18B20_Resolution)((scratchpad[4] >> 5) & 0x3);

    // Nullify undefined bits depending on resolution
    switch (resolution) {
    case DS18B20_RESOLUTION_9_BIT:
        tempRaw &= 0xFFF8;
        break;
    case DS18B20_RESOLUTION_10_BIT:
        tempRaw &= 0xFFFC;
        break;
    case DS18B20_RESOLUTION_11_BIT:
        tempRaw &= 0xFFFE;
        break;
    case DS18B20_RESOLUTION_12_BIT:
        // All bits are valid
        break;
    }

    // Convert to signed (two's complement)
    int16_t tempSigned = *(int16_t*)(&tempRaw);

    // Convert to float
    float Temp_LSB = 0.0625f;
    float tempFloat = (float)tempSigned * Temp_LSB;

    return tempFloat;
}

void DS18B20_Init(struct DS18B20_Platform *platformPtr) {
    platform = *platformPtr;

    // Release bus by default
    platform.gpioSwitch(DS18B20_GPIO_FLOATING_INPUT);
}

bool DS18B20_CheckPresence(void) {
    int presence = resetAndPresence();
    return presence == 0;
}

bool DS18B20_RunMeasurementSingle(int *delayBeforeReadMs) {
    // Initialization sequence - send reset and check presence
    int presence = resetAndPresence();
    if (presence) {
        platform.debugPrint("DS18B20: Failed to run conversion, sensor not present\r\n");
        return false;
    }

    // Send skip ROM command (single device on the bus, no need to match ROM)
    writeByte(CMD_SKIP_ROM);

    // Send temperature conversion command
    writeByte(CMD_CONVERT_T);

    // Drive bus with bank VCC if separate power supply not provided
    if (platform.flags & DS18B20_FLAG_PARASITE_POWER) {
        platform.gpioSwitch(DS18B20_GPIO_PULLUP_OUTPUT);
        platform.gpioSet(1);
    }

    // Output conversion time
    // TODO: depends on conversion configuration
    *delayBeforeReadMs = 1000;

    return true;
}

bool DS18B20_ReadTempSingle(float *temp) {
    // Send reset and check presence
    int presence = resetAndPresence();
    if (presence) {
        platform.debugPrint("DS18B20: Failed to read temperature, sensor not present\r\n");
        return false;
    }

    // Send skip ROM command (single device on the bus, no need to match ROM)
    writeByte(CMD_SKIP_ROM);

    // Read scratchpad
    writeByte(CMD_READ_SCRATCHPAD);
    
    uint8_t scratchpad[9];
    for (int i = 0; i < 9; i++) {
        scratchpad[i] = readByte();
    }

    // Check CRC
    uint8_t crc = calcCrc(scratchpad, 8);
    if (crc != scratchpad[8]) {
        platform.debugPrint("DS18B20: Failed to read temperature, CRC error\r\n");
        return false;
    }

    // Extract temperature
    *temp = extractTemp(scratchpad);

    return true;
}
