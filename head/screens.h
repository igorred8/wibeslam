#pragma once
#include "globals.h"

void drawIMUGraph(int gx, const char* title, float data[IMU_BUF_LEN][3], float range);
float frontDistanceM();
void drawPageStatus();
void drawPageMap();
void drawPageNet();
void drawKeyboard();
void updateDisplay();
void updateDisplayForce(int page);