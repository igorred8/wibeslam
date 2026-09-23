#include "ros.h"
#include "imu_filter.h"
#include <cstring>
#include <math.h>
#include <time.h>
#include <std_msgs/msg/int64.h>

// Временные переменные для синхронизации
static int64_t timeOffsetMs = 0;
static bool    timeSynced   = false;
static int64_t lastStampMs  = 0;   // ← ДОБАВИЛ для монотонности

// Переменные для подписки на /host_time
static rcl_subscription_t host_time_sub;
static std_msgs__msg__Int64 host_time_msg;
static int64_t hostTimeMs = 0;
static bool hostTimeReceived = false;
static rclc_executor_t executor;

static void host_time_callback(const void * msgin) {
    const std_msgs__msg__Int64 * msg = (const std_msgs__msg__Int64 *)msgin;
    hostTimeMs = msg->data;
    hostTimeReceived = true;
    if (!timeSynced) {                       // ← фиксируем ОДИН РАЗ
        timeOffsetMs = hostTimeMs - (int64_t)millis();
        timeSynced = true;
    }
}



// Получение ROS-времени для штамповки сообщений
static void getRosTime(builtin_interfaces__msg__Time & t) {
    int64_t nowMs = timeSynced ? ((int64_t)millis() + timeOffsetMs)
                               : (int64_t)millis();
    if (nowMs <= lastStampMs) nowMs = lastStampMs + 1;   // ← строгое возрастание
    lastStampMs = nowMs;
    t.sec     = (int32_t)(nowMs / 1000);
    t.nanosec = (uint32_t)((nowMs % 1000) * 1000000UL);
}

// Транспортные функции для UART
static bool ser_open(uxrCustomTransport*) { return true; }
static bool ser_close(uxrCustomTransport*) { return true; }
static size_t ser_write(uxrCustomTransport*, const uint8_t* b, size_t l, uint8_t*) { return Serial.write(b, l); }
static size_t ser_read(uxrCustomTransport*, uint8_t* b, size_t l, int timeout, uint8_t*) {
    uint32_t s = millis(); size_t n = 0;
    while (n < l && (int)(millis() - s) < timeout) {
        if (Serial.available()) b[n++] = (uint8_t)Serial.read(); else delay(1);
    }
    return n;
}

void rosTask(void*) {
    char ssid[40], pass[64], agent[20];
    netSsid.toCharArray(ssid, sizeof(ssid));
    netPass.toCharArray(pass, sizeof(pass));
    netAgent.toCharArray(agent, sizeof(agent));
    if (transportMode == 0) set_microros_wifi_transports(ssid, pass, agent, AGENT_PORT);
    else rmw_uros_set_custom_transport(true, NULL, ser_open, ser_close, ser_write, ser_read);

    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "palmslam_diy", "", &support);
    
    // Паблишеры
    rclc_publisher_init_default(&lidar_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan), "/scan");
    rclc_publisher_init_default(&imu_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "/imu");

    // Подписчик на /host_time
    rclc_subscription_init_default(
        &host_time_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int64),
        "/host_time");
    
    // Инициализация сообщения для подписчика
    std_msgs__msg__Int64__init(&host_time_msg);

    // Инициализация LaserScan сообщения
    static float ranges[MAX_SCAN_POINTS];
    static float intens[MAX_SCAN_POINTS];
    scan_msg.ranges.data = ranges;
    scan_msg.ranges.size = MAX_SCAN_POINTS; scan_msg.ranges.capacity = MAX_SCAN_POINTS;
    scan_msg.intensities.data = intens;
    scan_msg.intensities.size = MAX_SCAN_POINTS; scan_msg.intensities.capacity = MAX_SCAN_POINTS;
    scan_msg.angle_min = 0.0f;
    scan_msg.angle_max = 2.0f * M_PI;
    scan_msg.angle_increment = 2.0f * M_PI / MAX_SCAN_POINTS;
    scan_msg.scan_time = 0.1f;
    scan_msg.time_increment = 0.0001f;
    scan_msg.range_min = 0.05f;
    scan_msg.range_max = 12.0f;
    static char fid_scan[] = "laser";
    scan_msg.header.frame_id.data = fid_scan;
    scan_msg.header.frame_id.size = strlen(fid_scan);
    scan_msg.header.frame_id.capacity = strlen(fid_scan) + 1;
    
    // Инициализация IMU сообщения
    static char fid_imu[] = "imu_link";
    imu_msg.header.frame_id.data = fid_imu;
    imu_msg.header.frame_id.size = strlen(fid_imu);
    imu_msg.header.frame_id.capacity = strlen(fid_imu) + 1;
    imu_msg.orientation.w = 1.0f;
    for (int i = 0; i < 9; i++) {
        imu_msg.orientation_covariance[i] = 0.0f;
        imu_msg.angular_velocity_covariance[i] = (i % 4 == 0) ? 0.01f : 0.0f;
        imu_msg.linear_acceleration_covariance[i] = (i % 4 == 0) ? 0.01f : 0.0f;
    }
    
    // Создание executor и добавление подписчика
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_subscription(
        &executor,
        &host_time_sub,
        &host_time_msg,
        host_time_callback,
        ON_NEW_DATA);
    

    
    rosInitDone = true;
    vTaskDelete(NULL);
}

void scanTask(void*) {
    netCount = 0;
    int n = WiFi.scanNetworks();
    for (int i = 0; i < n && netCount < 6; i++) netList[netCount++] = WiFi.SSID(i);
    WiFi.scanDelete();
    netScanning = false;
    vTaskDelete(NULL);
}

void publishScan() {
    if (!lidarConnected || !rosConnected) return;
    if (!imuCalibrated) return;   // не спамим до снятия bias
    {   // троттлинг: не чаще 8 Гц
        static uint32_t lastPub = 0;
        uint32_t now = millis();
        if (now - lastPub < SCAN_MIN_INTERVAL_MS) return;
        lastPub = now;
    }
    for (int i = 0; i < MAX_SCAN_POINTS; i++) {
        float m = scanPoints[i].distance_mm / 1000.0f;
        bool ok = scanPoints[i].valid && m > 0.02f && m < 12.0f;
        scan_msg.ranges.data[i]      = ok ? m : INFINITY;
        scan_msg.intensities.data[i] = ok ? scanPoints[i].quality : 0.0f;
    }
    // Честная длительность оборота и шаг времени на луч (deskew в Cartographer)
    float sf = scanFrequency;
    if (sf < 1.0f) sf = 10.0f;
    scan_msg.scan_time      = 1.0f / sf;
    scan_msg.time_increment = scan_msg.scan_time / MAX_SCAN_POINTS;
    getRosTime(scan_msg.header.stamp);
    (void)rcl_publish(&lidar_pub, &scan_msg, NULL);
}

void publishIMU() {
    if (!imuConnected || !rosConnected) return;
    if (!imuCalibrated) return;
    imu_msg.linear_acceleration.x = accX;
    imu_msg.linear_acceleration.y = accY;
    imu_msg.linear_acceleration.z = accZ;
    imu_msg.angular_velocity.x = gyroX * M_PI / 180.0f;
    imu_msg.angular_velocity.y = gyroY * M_PI / 180.0f;
    imu_msg.angular_velocity.z = gyroZ * M_PI / 180.0f;
    
    // Стабильная ориентация из AHRS (режимы 5/6) вместо интеграции gyro
    float yaw = getIMUYaw();
    float yawRad = yaw * M_PI / 180.0f;
    
    // Конвертация Euler -> Quaternion (ZYX порядок)
    float cy = cosf(yawRad * 0.5f);
    float sy = sinf(yawRad * 0.5f);
    // Pitch и roll предполагаются близкими к 0 для SLAM (робот на плоскости)
    float cp = 1.0f, sp = 0.0f;  // pitch = 0
    float cr = 1.0f, sr = 0.0f;  // roll = 0
    
    imu_msg.orientation.w = cr*cp*cy + sr*sp*sy;
    imu_msg.orientation.x = sr*cp*cy - cr*sp*sy;
    imu_msg.orientation.y = cr*sp*cy + sr*cp*sy;
    imu_msg.orientation.z = cr*cp*sy - sr*sp*cy;
    
    // Ковариация orientation: маленькая для AHRS, большая для raw gyro
    if (imuFilterMode >= 5) {
        imu_msg.orientation_covariance[0] = 0.001f;   // roll
        imu_msg.orientation_covariance[4] = 0.001f;   // pitch
        imu_msg.orientation_covariance[8] = 0.002f;   // yaw (стабильный)
    } else {
        // Для старых режимов orientation будет уплывать — ставим большую ковариацию
        imu_msg.orientation_covariance[0] = 1.0f;
        imu_msg.orientation_covariance[4] = 1.0f;
        imu_msg.orientation_covariance[8] = 10.0f;
        imu_msg.orientation.w = 1.0f;  // Сброс к дефолту
        imu_msg.orientation.x = 0.0f;
        imu_msg.orientation.y = 0.0f;
        imu_msg.orientation.z = 0.0f;
    }
    
    getRosTime(imu_msg.header.stamp);
    (void)rcl_publish(&imu_pub, &imu_msg, NULL);
}

bool isTimeSynced() {
    return timeSynced;
}

void rosSpin() {
    if (rosInitDone) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
    }
}
