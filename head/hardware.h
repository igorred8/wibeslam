#pragma once
#include "globals.h"

void lidarInit();
void touchInit();
bool touchReadXY(int &x, int &y);   // ← было touchRead
void initQMI8658();
void readQMI8658();
void sampleIMUGraph();
bool readQMI8658Raw(float ra[3], float rg[3]);
void calibrateIMU(uint32_t ms);
extern SemaphoreHandle_t g_i2cLock;
void i2cLockCreate();
void i2cTake();
void i2cGive();