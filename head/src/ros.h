#pragma once
#include "globals.h"

void rosTask(void*);
void scanTask(void*);
void publishScan();
void publishIMU();
void syncTime();
void rosSpin();
bool isTimeSynced();