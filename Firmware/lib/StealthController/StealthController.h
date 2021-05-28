#ifndef STEALTH_CONTROLLER_H
#define STEALTH_CONTROLLER_H

#include "Arduino.h"
#include "drivers/drv8316/drv8316.h"
#include <SimpleFOC.h>

// DRV8316
#define DRV_MISO   21
#define DRV_MOSI   22
#define DRV_SCLK   19
#define DRV_SS     5

#define DRV_VREF	DAC1
#define DRV_OFF		8
#define DRV_FAULT	39

#define DRV_A_H 12
#define DRV_A_L 14
#define DRV_B_H 27
#define DRV_B_L 26
#define DRV_C_H 33
#define DRV_C_L 32

// MA702
#define ENC_MISO   15
#define ENC_MOSI   20
#define ENC_SCLK   13
#define ENC_SS     4

// CAN
#define CAN_TX		7
#define CAN_RX 		34


class StealthController
{
  public:
    StealthController(DRV8316Driver &drv);
    void setup();
    void loopStep(BLDCMotor &motor, float P, float I, float D, bool printLoopRate = true);
    void printStatus();

    // for PID smoothstart
    unsigned long pidDelay = 0;
    bool pids_set = false;

    // for main loop counter
    int count = 0;
    unsigned long lastTime = 0;

    DRV8316Driver * driver;
    SPIClass * drv_spi = NULL;
    SPIClass * enc_spi = NULL;

};

#endif