#pragma once

// ---- Отладка IMU через Serial ----
// Печатает CSV: t_ms,ax,ay,az,gx,gy,gz на частоте чтения IMU (~100 Hz).
// РАБОТАЕТ ТОЛЬКО В WiFi-РЕЖИМЕ! В USB-режиме Serial занят micro-ROS.
#define IMU_DEBUG_SERIAL 1

#define DEF_SSID  "CHANGE_ME"
#define DEF_PASS  "CHANGE_ME"
#define DEF_AGENT "192.168.1.100"
#define AGENT_PORT 8888

#define LIDAR_RX_PIN 18
#define LIDAR_TX_PIN 17
#define IMU_SDA 48
#define IMU_SCL 47
#define TOUCH_ADDR 0x15
#define TOUCH_INT  46
#define FWD_BIN 0

#define TFT_MOSI 38
#define TFT_MISO 40
#define TFT_SCLK 39
#define TFT_CS   45
#define TFT_DC   42
#define TFT_RST  -1
#define TFT_BL   1

#define MAX_SCAN_POINTS 360
#define OSD_INTERVAL_MS 33

#define NEON_BG     0x0000
#define NEON_CARD   0x1082
#define NEON_CYAN   0x07FF
#define NEON_BLUE   0x001F
#define NEON_MAG    0xF81F
#define NEON_LIME   0x07E0
#define NEON_AMBER  0xFFE0
#define NEON_RED    0xF800
#define NEON_DIM    0x2945
#define NEON_TXT    0xB5F8

#define BTN_ZOOM_MINUS_X   5
#define BTN_ZOOM_MINUS_Y  73
#define BTN_ZOOM_PLUS_X  275
#define BTN_ZOOM_PLUS_Y   73
#define BTN_SIZE          45
#define ZOOM_STEP 1.4f
#define ZOOM_MIN  10.0f
#define ZOOM_MAX 230.0f

#define BTN_IMU_X 140
#define BTN_IMU_Y  30
#define BTN_IMU_W  40
#define BTN_IMU_H  18
// ---- Фильтр IMU от вибрации лидара ----
// Окно скользящего среднего. Больше -> плавнее, но больше задержка.
// Рекомендуется 8..16 для LD14P.
#define IMU_FILTER_WINDOW 5

#define BTN_TR_X 274
#define BTN_TR_Y 178
#define BTN_TR_W  36
#define BTN_TR_H  36

#define KB_FIELD_H 26
#define KB_KEY_W   32
#define KB_KEY_H   28
#define KB_ROW0_Y  30
#define KB_CTL_Y  122
#define KB_CTL_H   28

#define IMU_BUF_LEN 120
#define GRAPH_H      66
#define GRAPH_W     150
#define GRAPH_Y     168
#define GRAPH_GYRO_X   5
#define GRAPH_ACC_X   165
#define GYRO_RANGE 1000
#define ACC_RANGE   20

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


// ==== IMU: калибровка и фильтры (imu_filter.cpp) ====
// 0=выкл 1=notch(частота оборотов) 2=Калман 3=синхронный 4=оверсемплинг
#define IMU_FILTER_MODE  1
#define IMU_TASK_HZ      500    // темп опроса IMU в отдельной задаче
#define IMU_NOTCH_Q      2.0f
#define IMU_KALMAN_Q     0.05f
#define IMU_KALMAN_R     0.8f
#define IMU_KALMAN_QG    0.50f
#define IMU_KALMAN_RG    3.0f
#define IMU_SYNC_BINS    12
#define IMU_SYNC_BETA    0.05f
#define IMU_OVER_N       25
#define IMU_CALIBRATION_MS 2000   // сбор bias при первом подключении к ROS
#define IMU_STILL_GYRO_MAX 5.0f   // порог "покоя" для автокоррекции, град/с
#define IMU_STILL_ACC_TOL  0.5f   // допуск |a| от 9.81 для "покоя"
#define SCAN_MIN_INTERVAL_MS 125  // троттлинг /scan: не чаще 8 Гц
#define IMU_ACC_EMA      0.5f   // акселерометр: лёгкая EMA, задержка ~2 мс
#define IMU_GYRO_DZ      0.25f  // мёртвая зона гироскопа, град/с: ноль дрейфа в покое
// (IMU_FILTER_WINDOW больше не используется — SMA ушёл в задачу, можно удалить)

