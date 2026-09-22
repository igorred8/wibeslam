#include "screens.h"
#include "ui.h"
#include <cstring>
#include <cstdio>
#include <math.h>

void drawIMUGraph(int gx, const char* title, float data[IMU_BUF_LEN][3], float range) {
    wCard(gx, GRAPH_Y, GRAPH_W, GRAPH_H);
    osdPrintf(gx + 4, GRAPH_Y + 3, 1, NEON_TXT, "%s", title);
    int ymid = GRAPH_Y + GRAPH_H / 2 + 4;
    disp->drawLine(gx + 2, ymid, gx + GRAPH_W - 3, ymid, NEON_DIM);
    uint16_t colors[3] = { NEON_MAG, NEON_LIME, NEON_CYAN };
    for (int axis = 0; axis < 3; axis++) {
        int prevX = -1, prevY = -1;
        for (int n = 0; n < imuBufLen; n++) {
            int idx = (imuBufHead - imuBufLen + n + IMU_BUF_LEN) % IMU_BUF_LEN;
            float v = data[idx][axis];
            if (v >  range) v =  range;
            if (v < -range) v = -range;
            int px = gx + 2 + (n * (GRAPH_W - 4)) / IMU_BUF_LEN;
            int py = ymid - (int)(v / range * (GRAPH_H / 2 - 8));
            if (prevX >= 0) disp->drawLine(prevX, prevY, px, py, colors[axis]);
            prevX = px; prevY = py;
        }
    }
}

float frontDistanceM() {
    for (int off = 0; off <= 5; off++) {
        int idxs[2] = { (FWD_BIN + off) % 360, (FWD_BIN - off + 360) % 360 };
        for (int k = 0; k < 2; k++) {
            int i = idxs[k];
            if (scanPoints[i].valid && scanPoints[i].distance_mm > 0)
                return scanPoints[i].distance_mm / 1000.0f;
        }
    }
    return -1.0f;
}

void drawPageStatus() {
    disp->fillScreen(NEON_BG);
    wAppBar("PALMSLAM");
    wWifiIcon();
    wChip(4,   32, "ROS",  rosConnected,   NEON_MAG);
    wChip(82,  32, "WiFi", wifiOK,         NEON_CYAN);
    wChip(160, 32, "LDR",  lidarConnected, NEON_LIME);
    wChip(238, 32, "IMU",  imuConnected,   NEON_AMBER);

    wCard(8, 54, 304, 62);
    osdPrintf(16, 60, 1, NEON_TXT, "DISTANCE");
    float fwd = frontDistanceM();
    char fbuf[16];
    if (fwd >= 0) snprintf(fbuf, sizeof(fbuf), "%.2f", fwd);
    else          snprintf(fbuf, sizeof(fbuf), "--");
    int u = 4, wx = 16;
    for (const char* p = fbuf; *p; p++) wx += (*p == '.') ? 4 * u : 8 * u;
    wBigNumber(16, 70, u, fbuf, NEON_CYAN);
    osdPrintf(wx + 4, 86, 2, NEON_TXT, "m");
    osdPrintf(220, 84, 2, NEON_AMBER, "%.1fHz", scanFrequency);

    wCard(8, 122, 304, 62);
    osdPrintf(16, 130, 2, NEON_LIME, "AX%.1f AY%.1f AZ%.1f", accX, accY, accZ);
    osdPrintf(16, 154, 2, NEON_TXT,  "GX%.0f GY%.0f GZ%.0f", gyroX, gyroY, gyroZ);

    osdPrintf(8, 192, 1, NEON_TXT, "IP:%s -> %s", WiFi.localIP().toString().c_str(), netAgent.c_str());

    wFab(BTN_TR_X + 18, BTN_TR_Y + 18, 18,
         transportMode == 0 ? NEON_BLUE : NEON_DIM,
         transportMode == 0 ? "W" : "U");
    osdPrintf(0, 228, 1, NEON_TXT, "1/3 swipe");
}

void drawPageMap() {
    disp->fillScreen(NEON_BG);
    wAppBar("LIDAR MAP");
    osdPrintf(190, 6, 2, NEON_AMBER, "Pts:%d", stablePts);
    wWifiIcon();

    const int cx = 160;
    const int cy = showIMUGraphs ? 100 : 128;
    const int r1 = showIMUGraphs ? 32 : 44;
    const int r2 = showIMUGraphs ? 64 : 88;
    disp->drawCircle(cx, cy, r1, NEON_DIM);
    disp->drawCircle(cx, cy, r2, NEON_DIM);
    disp->drawLine(cx - r2, cy, cx + r2, cy, NEON_DIM);
    disp->drawLine(cx, cy - r2, cx, cy + r2, NEON_DIM);

    // Поворот отрисовки: сдвигаем угол на FWD_BIN градусов
    const float angleShift = (float)(FWD_BIN +90) / MAX_SCAN_POINTS * 2.0f * M_PI;

    for (int i = 0; i < MAX_SCAN_POINTS; i++) {
        if (!scanPoints[i].valid) continue;
        float d = scanPoints[i].distance_mm / 1000.0f;
        if (d > 12.0f) continue;
        float rad = (float)i / MAX_SCAN_POINTS * 2.0f * M_PI + angleShift;
        int x = cx - (int)(d * mapScale * cosf(rad));
        int y = cy - (int)(d * mapScale * sinf(rad));
        if (x >= 0 && x < 320 && y >= 0 && y < 240)
            disp->drawPixel(x, y, NEON_LIME);
    }

    // ... остальная часть функции без изменений ...

    disp->fillRect(cx - 2, cy - 2, 5, 5, NEON_MAG);

    wFab(BTN_ZOOM_MINUS_X + 22, BTN_ZOOM_MINUS_Y + 22, 22, NEON_CARD, "-");
    wFab(BTN_ZOOM_PLUS_X  + 22, BTN_ZOOM_PLUS_Y  + 22, 22, NEON_CARD, "+");
    osdPrintf(BTN_ZOOM_PLUS_X + 8, BTN_ZOOM_PLUS_Y + 48, 1, NEON_TXT, "x%.1f", mapScale);

    wPill(BTN_IMU_X, BTN_IMU_Y, BTN_IMU_W, BTN_IMU_H, showIMUGraphs ? NEON_LIME : NEON_DIM, "IMU");

    if (showIMUGraphs) {
        drawIMUGraph(GRAPH_GYRO_X, "GYRO",  gyroBuf, (float)GYRO_RANGE);
        drawIMUGraph(GRAPH_ACC_X,  "ACCEL", accBuf,  (float)ACC_RANGE);
    }
    osdPrintf(0, 228, 1, NEON_TXT, "2/3 swipe");
}

void drawKeyboard() {
    disp->fillScreen(NEON_BG);
    wCard(4, 2, 312, KB_FIELD_H);
    disp->drawRoundRect(4, 2, 312, KB_FIELD_H, 10, NEON_CYAN);
    drawUtf8(10, 8, kbBuf, WHITE, 2);

    const char** rows = KB_ROWS[kbLayout];
    for (int r = 0; r < 3; r++) {
        String line = rows[r];
        int n = 0; while (utf8At(line, n).length() > 0) n++;
        for (int c = 0; c < n; c++) {
            String ch = utf8At(line, c);
            int kx = c * KB_KEY_W, ky = KB_ROW0_Y + r * KB_KEY_H;
            disp->fillRoundRect(kx + 1, ky + 3, KB_KEY_W - 2, KB_KEY_H - 3, 6, 0x0000);
            disp->fillRoundRect(kx + 1, ky + 1, KB_KEY_W - 2, KB_KEY_H - 3, 6, NEON_CARD);
            uint8_t b = ch[0];
            if (b < 0x80) {
                char cc = (kbShift && b >= 'a' && b <= 'z') ? b - 32 : b;
                disp->setTextSize(2); disp->setTextColor(WHITE);
                disp->setCursor(kx + 11, ky + 7); disp->print(cc);
            } else {
                uint16_t cp = ((b & 0x1F) << 6) | (ch[1] & 0x3F);
                if (kbShift) cp -= 0x20;
                drawCyr(kx + 11, ky + 7, cp, WHITE, 2);
            }
        }
    }
    for (int i = 0; i < 6; i++) {
        int kx = i * 53, w = (i == 5) ? 55 : 53;
        uint16_t col = (i == 4) ? NEON_LIME : (i == 5) ? NEON_RED :
                       (i == 0 && kbShift) ? NEON_CYAN : NEON_CARD;
        disp->fillRoundRect(kx + 1, KB_CTL_Y + 2, w - 2, KB_CTL_H, 8, 0x0000);
        disp->fillRoundRect(kx + 1, KB_CTL_Y, w - 2, KB_CTL_H, 8, col);
        disp->setTextColor(WHITE);
        if (i == 0) {
            int ax = kx + w / 2;
            disp->fillTriangle(ax, KB_CTL_Y + 6, ax - 7, KB_CTL_Y + 15, ax + 7, KB_CTL_Y + 15, WHITE);
            disp->fillRect(ax - 2, KB_CTL_Y + 14, 5, 8, WHITE);
        } else if (i == 1) {
            disp->setTextSize(1); disp->setCursor(kx + 16, KB_CTL_Y + 10);
            disp->print(kbLayout == 0 ? "EN" : kbLayout == 1 ? "RU" : "12");
        } else if (i == 2) {
            disp->fillRect(kx + 12, KB_CTL_Y + 13, w - 24, 3, WHITE);
        } else if (i == 3) {
            disp->setTextSize(2); disp->setCursor(kx + 20, KB_CTL_Y + 6); disp->print("<");
        } else if (i == 4) {
            disp->setTextSize(2); disp->setCursor(kx + 16, KB_CTL_Y + 6); disp->print("OK");
        } else {
            disp->setTextSize(2); disp->setCursor(kx + 22, KB_CTL_Y + 6); disp->print("X");
        }
    }
}

void drawPageNet() {
    disp->fillScreen(NEON_BG);
    wAppBar("NETWORK");
    if (kbField >= 0) { drawKeyboard(); return; }

    if (netView == 1) {
        osdPrintf(8, 32, 1, NEON_TXT, netScanning ? "Scanning..." : "Tap network:");
        for (int i = 0; i < netCount; i++) {
            wCard(8, 48 + i * 26, 304, 24);
            drawUtf8(16, 54 + i * 26, netList[i], WHITE, 1);
        }
        wPill(130, 210, 60, 24, NEON_CARD, "BACK");
        return;
    }

    wCard(8, 32, 304, 22); osdPrintf(14, 38, 1, NEON_TXT, "SSID");  drawUtf8(60, 38, netSsid,  WHITE, 1);
    wCard(8, 58, 304, 22); osdPrintf(14, 64, 1, NEON_TXT, "PASS");  drawUtf8(60, 64, netPass,  WHITE, 1);
    wCard(8, 84, 304, 22); osdPrintf(14, 90, 1, NEON_TXT, "AGENT"); drawUtf8(60, 90, netAgent, WHITE, 1);
    osdPrintf(8, 112, 1, NEON_TXT, "TR: %s", transportMode == 0 ? "WiFi" : "USB");

    wPill(250, 32, 60, 20, NEON_CYAN, "SCAN");
    wPill(250, 58, 60, 20, NEON_CARD, "EDIT");
    wPill(250, 84, 60, 20, NEON_CARD, "EDIT");
    wPill(100, 132, 120, 28, NEON_LIME, "SAVE");
    osdPrintf(0, 228, 1, NEON_TXT, "3/3 swipe");
}

void updateDisplay() {
    if (millis() - lastDisplayUpdate < OSD_INTERVAL_MS) return;
    lastDisplayUpdate = millis();
    if (screenPage == 0) drawPageStatus();
    else if (screenPage == 1) drawPageMap();
    else drawPageNet();
    gfx->draw16bitRGBBitmap(0, 0, canvas.getFramebuffer(), 320, 240);
}
void updateDisplayForce(int page) {
    screenPage = page;
    if (page == 0) drawPageStatus();
    else if (page == 1) drawPageMap();
    else drawPageNet();
    gfx->draw16bitRGBBitmap(0, 0, canvas.getFramebuffer(), 320, 240);
}
