#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <Preferences.h>
#include <Arduino_GFX_Library.h>
#include <LDS_LDROBOT_LD14P.h>

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <sensor_msgs/msg/imu.h>

#include "config.h"

// Дисплей
extern Arduino_DataBus *bus;
extern Arduino_GFX *gfx;
extern Arduino_Canvas canvas;
extern Arduino_GFX *disp;

// Лидар / NVS
extern HardwareSerial LidarSerial;
extern LDS_LDROBOT_LD14P lidar;
extern Preferences prefs;

// micro-ROS
extern rcl_allocator_t allocator;
extern rclc_support_t support;
extern rcl_node_t node;
extern rcl_publisher_t lidar_pub;
extern rcl_publisher_t imu_pub;
extern sensor_msgs__msg__LaserScan scan_msg;
extern sensor_msgs__msg__Imu imu_msg;

// Данные лидара
struct LidarPoint { float distance_mm; float quality; bool valid; };
extern LidarPoint scanPoints[MAX_SCAN_POINTS];
extern volatile int pointsThisScan;
extern volatile int stablePts;
extern volatile bool newScanReady;
extern float scanFrequency;
extern bool lidarConnected;
extern String lastLidarError;

// IMU
extern float accX, accY, accZ, gyroX, gyroY, gyroZ;
extern uint8_t g_imuAddr;
extern bool imuConnected;

// Буферы графиков
extern float gyroBuf[IMU_BUF_LEN][3];
extern float accBuf[IMU_BUF_LEN][3];
extern int imuBufHead, imuBufLen;

// Состояние
extern bool rosConnected;
extern volatile bool rosInitDone;
extern bool wifiOK, touchOK, showIMUGraphs;
extern int i2cDevices, screenPage, bootStage;
extern float mapScale;

// Сеть / клавиатура
extern int transportMode;
extern String netSsid, netPass, netAgent;
extern int netView, kbField, kbLayout;
extern bool kbShift, netScanning;
extern String kbBuf;
extern String netList[6];
extern int netCount;

// Тайминги
extern unsigned long lastDisplayUpdate, lastPublishIMU, lastIMURead,
                       lastIMUGraphSample, lastTouchPoll, lastPingMs;


// Bias-калибровка и счётчик оборотов
extern float gyroBiasX, gyroBiasY, gyroBiasZ;
extern float accBiasX, accBiasY, accBiasZ;
extern volatile bool imuCalibrated;
extern volatile uint32_t scanCounter;

