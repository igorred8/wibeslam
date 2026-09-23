#pragma once
#include "globals.h"

void imuFilterInit();
extern uint8_t imuFilterMode;

// Функция получения стабильного yaw для orientation
float getIMUYaw();