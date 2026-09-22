#include "imu_filter.h"
#include "hardware.h"
#include <math.h>

extern bool rosConnected;  // из ros.cpp
extern int transportMode;  // из globals.cpp

uint8_t imuFilterMode = IMU_FILTER_MODE;
#define NAX 6   // 3 accel + 3 gyro

// ---- NOTCH: два каскада (f оборота и 2f) ----
static float nB0,nB1,nB2,nA1,nA2;
static float n2B0,n2B1,n2B2,n2A1,n2A2;
static float nX1[NAX], nX2[NAX], nY1[NAX], nY2[NAX];
static float n2X1[NAX], n2X2[NAX], n2Y1[NAX], n2Y2[NAX];
static float nLastF = -1.0f;

static void notchCalc(float f, float fs, float q,
                      float &b0, float &b1, float &b2, float &a1, float &a2) {
    float w0 = 2.0f*M_PI*f/fs, cw = cosf(w0), alpha = sinf(w0)/(2.0f*q);
    float a0 = 1.0f + alpha;
    b0 = 1.0f/a0; b1 = (-2.0f*cw)/a0; b2 = 1.0f/a0;
    a1 = (-2.0f*cw)/a0; a2 = (1.0f-alpha)/a0;
}
static float notchRun(int i, float x,
                      float b0, float b1, float b2, float a1, float a2,
                      float X1[], float X2[], float Y1[], float Y2[]) {
    float y = b0*x + b1*X1[i] + b2*X2[i] - a1*Y1[i] - a2*Y2[i];
    X2[i]=X1[i]; X1[i]=x; Y2[i]=Y1[i]; Y1[i]=y;
    return y;
}

// ---- 2. KALMAN ----
static float kX[NAX], kP[NAX]; static bool kInit[NAX];
static float kalmanRun(int i, float z, float q, float r) {
    if (!kInit[i]) { kX[i]=z; kP[i]=1.0f; kInit[i]=true; return z; }
    kP[i] += q;
    float k = kP[i]/(kP[i]+r);
    kX[i] += k*(z-kX[i]);
    kP[i] *= (1.0f-k);
    return kX[i];
}

// ---- 3. SYNC ----
static float sTempl[IMU_SYNC_BINS][NAX], sMean[NAX];
static bool  sInit = false;
static float sPhase = 0.0f;
static uint32_t sLastScan = 0;

// ---- 4. OVER ----
static float oSum[NAX]; static int oCnt = 0;

static void writeOut(const float o[NAX]) {
    accX=o[0]; accY=o[1]; accZ=o[2];
    gyroX=o[3]; gyroY=o[4]; gyroZ=o[5];
}

static void imuTask(void*) {
    const float fs = IMU_TASK_HZ, dt = 1.0f/fs;
    TickType_t last = xTaskGetTickCount();
    float out[NAX];
    static bool calibrationTriggered = false;

    for (;;) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(1000/IMU_TASK_HZ));

        // Калибровка: один раз при первом подключении к ROS
        if (!calibrationTriggered && rosConnected) {
            calibrationTriggered = true;
            if (transportMode == 0) Serial.println("[IMU] ROS connected -> calibration...");
            calibrateIMU(IMU_CALIBRATION_MS);
            if (transportMode == 0) {
                Serial.printf("[IMU] Calibrated: acc %.2f/%.2f/%.2f gyro %.2f/%.2f/%.2f\n",
                              accBiasX, accBiasY, accBiasZ, gyroBiasX, gyroBiasY, gyroBiasZ);
            }
        }

        float ra[3], rg[3];
        if (!readQMI8658Raw(ra, rg)) continue;
        float z[NAX] = { ra[0], ra[1], ra[2], rg[0], rg[1], rg[2] };

        // Вычитаем калиброванный bias
        if (imuCalibrated) {
            z[0] -= accBiasX; z[1] -= accBiasY; z[2] -= accBiasZ;
            z[3] -= gyroBiasX; z[4] -= gyroBiasY; z[5] -= gyroBiasZ;
        }

        // Автокоррекция нуля гироскопа в покое
        {
            static uint32_t stillSince = 0;
            float am = sqrtf(z[0]*z[0] + z[1]*z[1] + z[2]*z[2]);
            float gm = sqrtf(z[3]*z[3] + z[4]*z[4] + z[5]*z[5]);
            uint32_t now = millis();
            if (fabsf(am - 9.81f) < IMU_STILL_ACC_TOL && gm < IMU_STILL_GYRO_MAX) {
                if (stillSince == 0) stillSince = now;
                else if (now - stillSince > 1000) {
                    const float alpha = 0.005f;
                    gyroBiasX += alpha * z[3];
                    gyroBiasY += alpha * z[4];
                    gyroBiasZ += alpha * z[5];
                }
            } else {
                stillSince = 0;
            }
        }

        switch (imuFilterMode) {
        case 1: {   // DUAL-NOTCH: f оборота + 2f, задержка ~0
            float f = scanFrequency;
            if (f < 2.0f) f = 10.0f;
            if (fabsf(f - nLastF) > 0.2f) {
                notchCalc(f,   fs, IMU_NOTCH_Q, nB0, nB1, nB2, nA1, nA2);
                notchCalc(2*f, fs, IMU_NOTCH_Q, n2B0, n2B1, n2B2, n2A1, n2A2);
                nLastF = f;
            }
            for (int i=0;i<NAX;i++) {
                float v = notchRun(i, z[i], nB0, nB1, nB2, nA1, nA2, nX1, nX2, nY1, nY2);
                out[i]  = notchRun(i, v,  n2B0, n2B1, n2B2, n2A1, n2A2, n2X1, n2X2, n2Y1, n2Y2);
            }
            break;
        }
        case 2: {   // KALMAN
            for (int i=0;i<NAX;i++)
                out[i] = kalmanRun(i, z[i], (i<3)?IMU_KALMAN_Q:IMU_KALMAN_QG,
                                          (i<3)?IMU_KALMAN_R:IMU_KALMAN_RG);
            break;
        }
        case 3: {   // SYNC: минус весь синхронный обороту шум (все гармоники)
            if (scanCounter != sLastScan) { sLastScan = scanCounter; sPhase = 0.0f; }
            else { sPhase += dt*scanFrequency; if (sPhase >= 1.0f) sPhase -= 1.0f; }
            int bin = (int)(sPhase * IMU_SYNC_BINS) % IMU_SYNC_BINS;
            if (!sInit) {
                for (int i=0;i<NAX;i++) { sTempl[bin][i]=z[i]; sMean[i]=z[i]; }
                sInit = true;
            }
            for (int i=0;i<NAX;i++) {
                sMean[i]       += IMU_SYNC_BETA*(z[i]-sMean[i]);
                sTempl[bin][i] += IMU_SYNC_BETA*(z[i]-sTempl[bin][i]);
                out[i] = z[i] - sTempl[bin][i] + sMean[i];
            }
            break;
        }
        case 4: {   // OVER
            for (int i=0;i<NAX;i++) oSum[i] += z[i];
            if (++oCnt >= IMU_OVER_N) {
                for (int i=0;i<NAX;i++) out[i] = oSum[i]/oCnt;
                oCnt = 0; for (int i=0;i<NAX;i++) oSum[i]=0;
            }
            break;
        }
        default:    // 0 = OFF: сырьё как есть
            for (int i=0;i<NAX;i++) out[i] = z[i];
            writeOut(out);
            continue;   // без пост-обработки
        }

        // ---- Пост-обработка (режимы 1-4) ----
        // Акселерометр: лёгкая EMA (гравитация медленная, задержка ~2 мс)
        {
            static float accF[3]; static bool accFInit = false;
            if (!accFInit) { accF[0]=out[0]; accF[1]=out[1]; accF[2]=out[2]; accFInit=true; }
            for (int i=0;i<3;i++) {
                accF[i] = IMU_ACC_EMA*out[i] + (1.0f-IMU_ACC_EMA)*accF[i];
                out[i] = accF[i];
            }
        }
        // Гироскоп: мёртвая зона — добивает остаточный дрейф в покое,
        // движения выше 0.25 град/с не трогает вообще
        for (int i=3;i<6;i++)
            if (fabsf(out[i]) < IMU_GYRO_DZ) out[i] = 0.0f;

        writeOut(out);
    }
}

void imuFilterInit() {
    xTaskCreatePinnedToCore(imuTask, "imuf", 4096, NULL, 2, NULL, 1);
}
