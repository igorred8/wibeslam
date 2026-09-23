#include "globals.h"

Arduino_DataBus *bus = new Arduino_ESP32SPI(TFT_DC, TFT_CS, TFT_SCLK, TFT_MOSI, TFT_MISO);
Arduino_GFX *gfx = new Arduino_ST7789(bus, TFT_RST, 1, true, 240, 320);
Arduino_Canvas canvas(320, 240, gfx, 0, 0, 0);
Arduino_GFX *disp;

HardwareSerial LidarSerial(1);
LDS_LDROBOT_LD14P lidar;
Preferences prefs;

rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_publisher_t lidar_pub;
rcl_publisher_t imu_pub;
sensor_msgs__msg__LaserScan scan_msg;
sensor_msgs__msg__Imu imu_msg;

LidarPoint scanPoints[MAX_SCAN_POINTS];
volatile int pointsThisScan = 0;
volatile int stablePts = 0;
volatile bool newScanReady = false;
float scanFrequency = 0.0f;
bool lidarConnected = false;
String lastLidarError = "";

float accX=0, accY=0, accZ=0, gyroX=0, gyroY=0, gyroZ=0;
uint8_t g_imuAddr = 0;
bool imuConnected = false;

float gyroBuf[IMU_BUF_LEN][3];
float accBuf[IMU_BUF_LEN][3];
int imuBufHead = 0, imuBufLen = 0;

bool rosConnected = false;
volatile bool rosInitDone = false;
bool wifiOK = false, touchOK = false, showIMUGraphs = true;
int i2cDevices = 0, screenPage = 0, bootStage = 0;
float mapScale = 15.0f;

int transportMode = 0;
String netSsid, netPass, netAgent;
int netView = 0, kbField = -1, kbLayout = 0;
bool kbShift = false, netScanning = false;
String kbBuf = "";
String netList[6];
int netCount = 0;

unsigned long lastDisplayUpdate=0, lastPublishIMU=0, lastIMURead=0,
              lastIMUGraphSample=0, lastTouchPoll=0, lastPingMs=0;

float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;
float accBiasX = 0, accBiasY = 0, accBiasZ = 0;
volatile bool imuCalibrated = false;
volatile uint32_t scanCounter = 0;
