#include "imu_filter.h"
#include "hardware.h"
#include <math.h>

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

// ---- 5. MADGWICK AHRS (канонический градиентный спуск, 6-DOF) ----
static float mQ0 = 1.0f, mQ1 = 0.0f, mQ2 = 0.0f, mQ3 = 0.0f;
static float mBeta = IMU_MADGWICK_BETA;

// gx,gy,gz в град/с; ax,ay,az в м/с^2; dt в секундах
static void madgwickUpdate(float gx, float gy, float gz,
                           float ax, float ay, float az,
                           float dt) {
    // гироскоп -> рад/с
    gx *= M_PI / 180.0f; gy *= M_PI / 180.0f; gz *= M_PI / 180.0f;

    float q0 = mQ0, q1 = mQ1, q2 = mQ2, q3 = mQ3;
    float q0dot = 0.5f * (-q1*gx - q2*gy - q3*gz);
    float q1dot = 0.5f * ( q0*gx + q2*gz - q3*gy);
    float q2dot = 0.5f * ( q0*gy - q1*gz + q3*gx);
    float q3dot = 0.5f * ( q0*gz + q1*gy - q2*gx);

    // коррекция по акселерометру: нормализуем вектор гравитации
    float an = sqrtf(ax*ax + ay*ay + az*az);
    if (an > 0.1f) {
        ax /= an; ay /= an; az /= an;

        float _2q0 = 2.0f*q0, _2q1 = 2.0f*q1, _2q2 = 2.0f*q2, _2q3 = 2.0f*q3;
        float _4q0 = 4.0f*q0, _4q1 = 4.0f*q1, _4q2 = 4.0f*q2, _4q3 = 4.0f*q3;
        float _8q1 = 8.0f*q1, _8q2 = 8.0f*q2;
        float q0q0 = q0*q0, q1q1 = q1*q1, q2q2 = q2*q2, q3q3 = q3*q3;

        // J^T * f (канонические выражения из заметок S.Madgwick, 6-DOF)
        float s0 = _4q0*q2q2 + _2q2*ax + _4q0*q1q1 - _2q1*ay;
        float s1 = -_4q1 + _8q1*q1q1 + _8q1*q2q2 + _4q1*az
                   - _2q3*ax + _4q0*q0q0*q1 - _2q0*ay;
        float s2 = _4q2*az + _2q0*ax - _4q2 + _8q2*q1q1 + _8q2*q2q2
                   + _4q2*q3q3 - _2q3*ay;
        float s3 = _4q3*q1q1 + _4q3*q2q2 - _2q1*ax - _2q2*ay;

        // нормирование шага градиента
        float sn = sqrtf(s0*s0 + s1*s1 + s2*s2 + s3*s3);
        if (sn > 1e-6f) {
            s0 /= sn; s1 /= sn; s2 /= sn; s3 /= sn;
            const float bdeg = mBeta * 180.0f / M_PI;  // qXdot здесь в град/с
            q0dot -= bdeg * s0;
            q1dot -= bdeg * s1;
            q2dot -= bdeg * s2;
            q3dot -= bdeg * s3;
        }
    }

    // интеграция (Euler) + нормализация
    mQ0 = q0 + q0dot * dt;
    mQ1 = q1 + q1dot * dt;
    mQ2 = q2 + q2dot * dt;
    mQ3 = q3 + q3dot * dt;

    float n = sqrtf(mQ0*mQ0 + mQ1*mQ1 + mQ2*mQ2 + mQ3*mQ3);
    if (n > 1e-4f) { mQ0/=n; mQ1/=n; mQ2/=n; mQ3/=n; }
    else { mQ0 = 1.0f; mQ1 = mQ2 = mQ3 = 0.0f; }
}

static float getMadgwickYaw() {
    float yaw = atan2f(2.0f*(mQ0*mQ3 + mQ1*mQ2), 1.0f - 2.0f*(mQ2*mQ2 + mQ3*mQ3));
    return yaw * 180.0f / M_PI;
}

// ---- 6. COMPLEMENTARY FILTER ----
static float cPitch = 0.0f, cRoll = 0.0f, cYaw = 0.0f;
static uint32_t cLastTime = 0;
static float cAlpha = IMU_COMP_ALPHA;

static void complementaryUpdate(float ax, float ay, float az,
                                float gx, float gy, float gz,
                                uint32_t now) {
    float dt = (now - cLastTime) / 1000.0f;
    if (dt > 0.1f || dt < 0.001f) dt = 0.002f;
    cLastTime = now;
    
    // Углы из акселерометра
    float accPitch = atan2f(ay, sqrtf(ax*ax + az*az)) * 180.0f / M_PI;
    float accRoll  = atan2f(-ax, az) * 180.0f / M_PI;
    
    // Интегрирование гироскопа
    cPitch = cAlpha * (cPitch + gx * dt) + (1.0f - cAlpha) * accPitch;
    cRoll  = cAlpha * (cRoll  + gy * dt) + (1.0f - cAlpha) * accRoll;
    cYaw  += gz * dt;
    
    // Ограничение pitch/roll
    if (cPitch > 90.0f) cPitch = 90.0f;
    if (cPitch < -90.0f) cPitch = -90.0f;
    if (cRoll > 180.0f) cRoll -= 360.0f;
    if (cRoll < -180.0f) cRoll += 360.0f;
    if (cYaw > 180.0f) cYaw -= 360.0f;
    if (cYaw < -180.0f) cYaw += 360.0f;
}

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
        uint32_t now = millis();

        // Калибровка: один раз при первом подключении к ROS (плата стоит)
        if (!calibrationTriggered && rosConnected) {
            calibrationTriggered = true;
            if (transportMode == 0) Serial.println("[IMU] ROS connected -> calibration...");
            calibrateIMU(IMU_CALIBRATION_MS);
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
            uint32_t nowMs = millis();
            if (fabsf(am - 9.81f) < IMU_STILL_ACC_TOL && gm < IMU_STILL_GYRO_MAX) {
                if (stillSince == 0) stillSince = nowMs;
                else if (nowMs - stillSince > 1000) {
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
        case 3: {   // SYNC: минус весь синхронный обороту шум
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
        case 5: {   // MADGWICK AHRS: fusion для stable orientation
            // Сначала фильтруем шум (лёгкий Kalman на raw данных)
            for (int i=0;i<NAX;i++)
                out[i] = kalmanRun(i, z[i], (i<3)?0.02f:0.1f, (i<3)?0.3f:1.0f);
            
            // Обновляем AHRS
            madgwickUpdate(out[3], out[4], out[5], out[0], out[1], out[2], dt);
            
            // Для совместимости: оставляем filtered accel/gyro, 
            // но orientation берётся из кватерниона в ros.cpp
            break;
        }
        case 6: {   // COMPLEMENTARY: простой fusion
            complementaryUpdate(z[0], z[1], z[2], z[3], z[4], z[5], now);
            for (int i=0;i<NAX;i++) out[i] = z[i];
            break;
        }
        default:    // 0 = OFF
            for (int i=0;i<NAX;i++) out[i] = z[i];
            writeOut(out);
            continue;
        }

        // ---- Пост-обработка (режимы 1-4) ----
        // Акселерометр: лёгкая EMA (гравитация медленная, задержка ~2 мс)
        if (imuFilterMode <= 4) {
            static float accF[3]; static bool accFInit = false;
            if (!accFInit) { accF[0]=out[0]; accF[1]=out[1]; accF[2]=out[2]; accFInit=true; }
            for (int i=0;i<3;i++) {
                accF[i] = IMU_ACC_EMA*out[i] + (1.0f-IMU_ACC_EMA)*accF[i];
                out[i] = accF[i];
            }
        }
        // Гироскоп: мёртвая зона — ноль дрейфа в покое, движения не трогает
        for (int i=3;i<6;i++)
            if (fabsf(out[i]) < IMU_GYRO_DZ) out[i] = 0.0f;

        writeOut(out);
    }
}

void imuFilterInit() {
    mBeta = IMU_MADGWICK_BETA;
    cAlpha = IMU_COMP_ALPHA;
    xTaskCreatePinnedToCore(imuTask, "imuf", 4096, NULL, 2, NULL, 1);
}

// Функция получения ориентации для ros.cpp
float getIMUYaw() {
    if (imuFilterMode == 5) {
        return getMadgwickYaw();
    } else if (imuFilterMode == 6) {
        return cYaw;
    }
    // Для других режимов: интегрируем gyro (будет уплывать)
    static float yaw = 0.0f;
    yaw += gyroZ * 0.002f;  // dt = 2ms
    while (yaw > 180.0f) yaw -= 360.0f;
    while (yaw < -180.0f) yaw += 360.0f;
    return yaw;
}
