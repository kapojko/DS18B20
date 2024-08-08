#ifndef DS18B20_H
#define DS18B20_H

#include <stdbool.h>

#define DS18B20_FLAG_PARASITE_POWER 0x1

enum DS18B20_Resolution {
    DS18B20_RESOLUTION_9_BIT = 0, // max conversion time 93.75ms
    DS18B20_RESOLUTION_10_BIT = 1, // max conversion time 187.5ms
    DS18B20_RESOLUTION_11_BIT = 2, // max conversion time 375ms
    DS18B20_RESOLUTION_12_BIT = 3, // max conversion time 750ms - default
};

enum DS18B20_GPIOConfig {
    DS18B20_GPIO_FLOATING_INPUT,
    DS18B20_GPIO_PULLUP_OUTPUT,
};

struct DS18B20_Platform {
    int (*gpioGet)(void);
    void (*gpioSet)(int value);
    void (*gpioSwitch)(enum DS18B20_GPIOConfig config);

    void (*delayUs)(int us);
    void (*debugPrint)(const char *fmt, ...);

    int flags;
};

void DS18B20_Init(struct DS18B20_Platform *platform);
bool DS18B20_CheckPresence(void);

bool DS18B20_RunMeasurementSingle(int *delayBeforeReadMs);
bool DS18B20_ReadTempSingle(float *temp);

#endif // DS18B20_H
