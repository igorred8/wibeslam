#pragma once
#include "globals.h"

void rosTask(void*);
void scanTask(void*);
void publishScan();
void publishIMU();
void syncTime();
void rosSpin();
void rosDeinit();
bool rosLinkOk();
bool isTimeSynced();
extern volatile bool rosTaskRunning;
extern volatile bool rosNeedDeinit;