#include "config.h"
#include "globals.h"
#include "hardware.h"
#include "ros.h"
#include "screens.h"
#include "input.h"
#include "imu_filter.h"

void setup() {
    Serial.begin(115200);
    i2cLockCreate();
    pinMode(TFT_BL, OUTPUT); digitalWrite(TFT_BL, HIGH);
    gfx->begin();
    canvas.begin();
    disp = &canvas;
    prefs.begin("palmslam", false);
    netSsid  = prefs.getString("ssid", DEF_SSID);
    netPass  = prefs.getString("pass", DEF_PASS);
    netAgent = prefs.getString("agent", DEF_AGENT);
    transportMode = prefs.getUChar("tr", 0);
    updateDisplayForce(0);
    updateDisplayForce(1);
    screenPage = 0;
    touchInit();
    bootStage = 1;
}

void loop() {
    if (bootStage == 1) { lidarInit(); bootStage = 2; }
    else if (bootStage == 2) {
        initQMI8658();
        for (uint8_t a = 0x08; a < 0x78; a++) {
            Wire.beginTransmission(a);
            if (Wire.endTransmission() == 0) i2cDevices++;
        }
        bootStage = 3;
    }
    else if (bootStage == 3) {
        rosTaskRunning = true;
        xTaskCreatePinnedToCore(rosTask, "ros", 16384, NULL, 1, NULL, 0);
        imuFilterInit();      // задача опроса IMU 500 Гц + фильтры + калибровка
        bootStage = 4;
    }
    

    wifiOK = (transportMode == 0) && (WiFi.status() == WL_CONNECTED);
    lidar.loop();


    // IMU читает задача imuTask; здесь только отладочная печать глобалов
    if (millis() - lastIMURead >= 10) {
        lastIMURead = millis();
#if IMU_DEBUG_SERIAL
        if (transportMode == 0) {   // в USB-режиме Serial трогать нельзя
            Serial.printf("%lu,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f\n",
                          (unsigned long)millis(),
                          accX, accY, accZ, gyroX, gyroY, gyroZ);
        }
#endif
    }

    if (millis() - lastIMUGraphSample >= 33) { sampleIMUGraph(); lastIMUGraphSample = millis(); }
    pollTouch();
    updateDisplay();

    

    if (rosInitDone && millis() - lastPingMs >= 1000) {
        static uint32_t connLostSince = 0;
        lastPingMs   = millis();
        rosConnected = rosLinkOk();
        if (rosConnected) {
            connLostSince = 0;
        } else if (connLostSince == 0) {
            connLostSince = millis();
        } else if (millis() - connLostSince > 15000) {
            connLostSince = 0;
            rosNeedDeinit = true;   // fini выполнит rosSpinTask (без гонки)
        }
    }
}
