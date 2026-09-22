#include "hardware.h"

// ---------- коллбэки лидара (внутренние) ----------
static int lidar_serial_read_callback() { return LidarSerial.read(); }
static size_t lidar_serial_write_callback(const uint8_t * buffer, size_t length) {
    return LidarSerial.write(buffer, length);
}
static void lidar_scan_point_callback(float angle_deg, float distance_mm, float quality, bool scan_completed) {
    lidarConnected = true;
    if (distance_mm > 0) {
        int idx = (int)(angle_deg / 360.0f * MAX_SCAN_POINTS);
        if (idx >= 0 && idx < MAX_SCAN_POINTS) {
            scanPoints[idx].distance_mm = distance_mm;
            scanPoints[idx].quality = quality;
            scanPoints[idx].valid = true;
            pointsThisScan++;
        }
    }
    if (scan_completed) {
        scanFrequency = lidar.getCurrentScanFreqHz();
        if (pointsThisScan >= 100) stablePts = pointsThisScan;
        pointsThisScan = 0;
        newScanReady = true;
    }
}
static void lidar_packet_callback(uint8_t*, uint16_t, bool) {}

void lidarInit() {
    LidarSerial.setRxBufferSize(1024);
    uint32_t baud = lidar.getSerialBaudRate();
    LidarSerial.begin(baud, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);
    lidar.setScanPointCallback(lidar_scan_point_callback);
    lidar.setPacketCallback(lidar_packet_callback);
    lidar.setSerialWriteCallback(lidar_serial_write_callback);
    lidar.setSerialReadCallback(lidar_serial_read_callback);
    lidar.init();
    LDS::result_t res = lidar.start();
    if (res < 0) lastLidarError = lidar.resultCodeToString(res);
}

// ---------- тач ----------
void touchInit() {
    pinMode(TOUCH_INT, INPUT_PULLUP);
    Wire.beginTransmission(TOUCH_ADDR); Wire.write(0x00); Wire.endTransmission();
}
bool touchReadXY(int &x, int &y) {
    Wire.beginTransmission(TOUCH_ADDR); Wire.write(0x01);
    if (Wire.endTransmission(false) != 0) return false;
    Wire.requestFrom((uint8_t)TOUCH_ADDR, (uint8_t)6);
    if (Wire.available() < 6) return false;
    Wire.read();
    uint8_t n = Wire.read(), xh = Wire.read(), xl = Wire.read(), yh = Wire.read(), yl = Wire.read();
    if (n == 0) return false;
    int rx = ((xh & 0x0F) << 8) | xl, ry = ((yh & 0x0F) << 8) | yl;
    int sx = ry, sy = 240 - rx;
    x = sx < 0 ? 0 : sx > 319 ? 319 : sx;
    y = sy < 0 ? 0 : sy > 239 ? 239 : sy;
    touchOK = true;
    return true;
}

// ---------- IMU ----------
static uint8_t qmiRead(uint8_t a, uint8_t r) {
    Wire.beginTransmission(a); Wire.write(r); Wire.endTransmission(false);
    Wire.requestFrom(a, (uint8_t)1);
    return Wire.available() ? Wire.read() : 0xFF;
}
static void qmiWrite(uint8_t a, uint8_t r, uint8_t v) {
    Wire.beginTransmission(a); Wire.write(r); Wire.write(v); Wire.endTransmission();
}
void initQMI8658() {
    Wire.begin(IMU_SDA, IMU_SCL, 400000); Wire.setTimeOut(20);
    for (uint8_t a : {(uint8_t)0x6B, (uint8_t)0x6A})
        if (qmiRead(a, 0x00) == 0x05) { g_imuAddr = a; imuConnected = true; break; }
    if (!imuConnected) return;
    qmiWrite(g_imuAddr, 0x02, 0x60); qmiWrite(g_imuAddr, 0x03, 0x23);
    qmiWrite(g_imuAddr, 0x04, 0x43); qmiWrite(g_imuAddr, 0x08, 0x03);
}

// Кольцевые буферы для скользящего среднего
static float accHistX[IMU_FILTER_WINDOW], accHistY[IMU_FILTER_WINDOW], accHistZ[IMU_FILTER_WINDOW];
static float gyrHistX[IMU_FILTER_WINDOW], gyrHistY[IMU_FILTER_WINDOW], gyrHistZ[IMU_FILTER_WINDOW];
static int   histIdx = 0;
static int   histLen = 0;
static float sma(const float* buf, int len) {
    if (len == 0) return 0.0f;
    float s = 0; for (int i = 0; i < len; i++) s += buf[i];
    return s / len;
}

// Сырое чтение + подмена осей, БЕЗ фильтрации (для imuTask, пункт 2)
bool readQMI8658Raw(float ra[3], float rg[3]) {
    if (!imuConnected) return false;
    if (!(qmiRead(g_imuAddr, 0x2E) & 0x03)) return false;
    Wire.beginTransmission(g_imuAddr); Wire.write(0x35); Wire.endTransmission(false);
    Wire.requestFrom(g_imuAddr, (uint8_t)12);
    uint8_t b[12];
    for (int i = 0; i < 12; i++) b[i] = Wire.available() ? Wire.read() : 0;
    int16_t ax=(b[1]<<8)|b[0], ay=(b[3]<<8)|b[2], az=(b[5]<<8)|b[4];
    int16_t gx=(b[7]<<8)|b[6], gy=(b[9]<<8)|b[8], gz=(b[11]<<8)|b[10];
    float rawAX = ax/4096.0f*9.81f, rawAY = ay/4096.0f*9.81f, rawAZ = az/4096.0f*9.81f;
    float rawGX = gx/64.0f, rawGY = gy/64.0f, rawGZ = gz/64.0f;
    ra[0] = rawAZ; ra[1] = rawAX; ra[2] = rawAY;
    rg[0] = rawGZ; rg[1] = rawGX; rg[2] = rawGY;
    return true;
}

// Исходная readQMI8658: сырьё + подмена осей + SMA (как в стабильной версии)
void readQMI8658() {
    if (!imuConnected) return;
    if (!(qmiRead(g_imuAddr, 0x2E) & 0x03)) return;
    Wire.beginTransmission(g_imuAddr); Wire.write(0x35); Wire.endTransmission(false);
    Wire.requestFrom(g_imuAddr, (uint8_t)12);
    uint8_t b[12];
    for (int i = 0; i < 12; i++) b[i] = Wire.available() ? Wire.read() : 0;
    int16_t ax=(b[1]<<8)|b[0], ay=(b[3]<<8)|b[2], az=(b[5]<<8)|b[4];
    int16_t gx=(b[7]<<8)|b[6], gy=(b[9]<<8)|b[8], gz=(b[11]<<8)|b[10];
    float rawAX = ax/4096.0f*9.81f, rawAY = ay/4096.0f*9.81f, rawAZ = az/4096.0f*9.81f;
    float rawGX = gx/64.0f, rawGY = gy/64.0f, rawGZ = gz/64.0f;
    float aX =  rawAZ;  // вперёд <- датчик Z
    float aY =  rawAX;  // влево  <- датчик X
    float aZ =  rawAY;  // вверх  <- датчик Y
    float gX =  rawGZ;  // крен
    float gY =  rawGX;  // тангаж
    float gZ =  rawGY;  // рыскание
    accHistX[histIdx] = aX;  gyrHistX[histIdx] = gX;
    accHistY[histIdx] = aY;  gyrHistY[histIdx] = gY;
    accHistZ[histIdx] = aZ;  gyrHistZ[histIdx] = gZ;
    histIdx = (histIdx + 1) % IMU_FILTER_WINDOW;
    if (histLen < IMU_FILTER_WINDOW) histLen++;
    accX = sma(accHistX, histLen);
    accY = sma(accHistY, histLen);
    accZ = sma(accHistZ, histLen);
    gyroX = sma(gyrHistX, histLen);
    gyroY = sma(gyrHistY, histLen);
    gyroZ = sma(gyrHistZ, histLen);
}

// Калибровка bias: среднее за ms мс в покое (для imuTask, пункт 2)
void calibrateIMU(uint32_t ms) {
    if (!imuConnected) return;
    float sx=0, sy=0, sz=0, gx=0, gy=0, gz=0;
    int n = 0;
    uint32_t t0 = millis();
    while (millis() - t0 < ms) {
        float ra[3], rg[3];
        if (readQMI8658Raw(ra, rg)) {
            sx += ra[0]; sy += ra[1]; sz += ra[2];
            gx += rg[0]; gy += rg[1]; gz += rg[2];
            n++;
        }
        delay(5);
    }
    if (n < 10) return;
    gyroBiasX = gx / n; gyroBiasY = gy / n; gyroBiasZ = gz / n;
    accBiasX  = sx / n; accBiasY  = sy / n; accBiasZ  = (sz / n) - 9.81f;
    imuCalibrated = true;
    if (transportMode == 0)
        Serial.printf("[IMU] Calibrated: acc %.2f/%.2f/%.2f gyro %.2f/%.2f/%.2f (n=%d)\n",
                      accBiasX, accBiasY, accBiasZ, gyroBiasX, gyroBiasY, gyroBiasZ, n);
}

void sampleIMUGraph() {
    gyroBuf[imuBufHead][0]=gyroX; gyroBuf[imuBufHead][1]=gyroY; gyroBuf[imuBufHead][2]=gyroZ;
    accBuf[imuBufHead][0]=accX; accBuf[imuBufHead][1]=accY; accBuf[imuBufHead][2]=accZ;
    imuBufHead = (imuBufHead + 1) % IMU_BUF_LEN;
    if (imuBufLen < IMU_BUF_LEN) imuBufLen++;
}