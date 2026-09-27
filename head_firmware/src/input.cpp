#include "input.h"
#include "ui.h"
#include "ros.h"
#include <cstdlib>
#include "hardware.h"

void kbAppend(const String& ch) { if (kbBuf.length() < 30) kbBuf += ch; }
void kbAppendCp(uint16_t cp) {
    if (kbBuf.length() >= 30) return;
    if (cp < 0x80) kbBuf += (char)cp;
    else { kbBuf += (char)(0xC0 | ((cp >> 6) & 0x1F)); kbBuf += (char)(0x80 | (cp & 0x3F)); }
}
void kbBackspace() {
    if (kbBuf.length() == 0) return;
    uint8_t b = kbBuf[kbBuf.length() - 1];
    if (b < 0x80) kbBuf = kbBuf.substring(0, kbBuf.length() - 1);
    else          kbBuf = kbBuf.substring(0, kbBuf.length() - 2);
}

void handleKeyboardTap(int x, int y) {
    if (y >= KB_CTL_Y && y < KB_CTL_Y + KB_CTL_H) {
        int i = x / 53; if (i > 5) i = 5;
        if (i == 0) kbShift = !kbShift;
        else if (i == 1) kbLayout = (kbLayout + 1) % 3;
        else if (i == 2) kbAppend(" ");
        else if (i == 3) kbBackspace();
        else if (i == 4) {
            if (kbField == 0) netPass = kbBuf; else netAgent = kbBuf;
            kbField = -1; kbShift = false;
        }
        else if (i == 5) { kbField = -1; kbShift = false; }
        return;
    }
    int row = (y - KB_ROW0_Y) / KB_KEY_H;
    if (row < 0 || row > 2) return;
    int col = x / KB_KEY_W;
    String ch = utf8At(KB_ROWS[kbLayout][row], col);
    if (ch.length() == 0) return;
    if (ch.length() == 1) {
        char c = ch[0];
        if (kbShift && c >= 'a' && c <= 'z') { c -= 32; kbShift = false; }
        kbAppend(String(c));
    } else {
        uint16_t cp = ((ch[0] & 0x1F) << 6) | (ch[1] & 0x3F);
        if (kbShift && cp >= 0x430 && cp <= 0x44F) { cp -= 0x20; kbShift = false; }
        kbAppendCp(cp);
    }
}

void handleNetTap(int x, int y, int dx, int dy) {
    bool tap = abs(dx) < 15 && abs(dy) < 15;
    if (!tap) return;
    if (kbField >= 0) { handleKeyboardTap(x, y); return; }
    if (netView == 1) {
        if (x >= 130 && x <= 190 && y >= 210 && y <= 234) { netView = 0; return; }
        for (int i = 0; i < netCount; i++) {
            if (y >= 48 + i * 26 && y < 72 + i * 26) {
                netSsid = netList[i]; netView = 0;
                kbField = 0; kbLayout = 0; kbBuf = netPass;
                return;
            }
        }
        return;
    }
    if (x >= 250 && x <= 310) {
        if (y >= 32 && y <= 52) {
            netView = 1; netScanning = true;
            xTaskCreatePinnedToCore(scanTask, "scan", 8192, NULL, 1, NULL, 0);
            return;
        }
        if (y >= 58 && y <= 78)  { kbField = 0; kbLayout = 0; kbBuf = netPass;  return; }
        if (y >= 84 && y <= 104) { kbField = 1; kbLayout = 2; kbBuf = netAgent; return; }
    }
    if (x >= 100 && x <= 220 && y >= 132 && y <= 160) {
        prefs.putString("ssid", netSsid.c_str());
        prefs.putString("pass", netPass.c_str());
        prefs.putString("agent", netAgent.c_str());
        prefs.end();
        ESP.restart();
    }
}

void pollTouch() {
    if (millis() - lastTouchPoll < 30) return;
    lastTouchPoll = millis();
    static bool touching = false;
    static int xStart = 0, yStart = 0, xLast = 0, yLast = 0;
    static int xMax = 0, yMax = 0;
    int x, y;
    if (touchReadXY(x, y)) {
        if (!touching) { touching = true; xStart = x; yStart = y; xMax = 0; yMax = 0; }
        xLast = x; yLast = y;
        int ex = abs(x - xStart), ey = abs(y - yStart);
        if (ex > xMax) xMax = ex;
        if (ey > yMax) yMax = ey;
    } else if (touching) {
        touching = false;
        int dx = xLast - xStart, dy = yLast - yStart;
        bool swiped = (xMax > 30 || yMax > 30);
        if (screenPage == 2) {
            if (kbField < 0 && netView == 0 && swiped)
                screenPage = (screenPage + 1) % 3;
            else handleNetTap(xStart, yStart, dx, dy);
            return;
        }
        if (screenPage == 0) {
            if (!swiped &&
                xStart >= BTN_TR_X && xStart < BTN_TR_X + BTN_TR_W &&
                yStart >= BTN_TR_Y && yStart < BTN_TR_Y + BTN_TR_H) {
                transportMode = !transportMode;
                prefs.putUChar("tr", transportMode);
                prefs.end();
                ESP.restart();
            }
            if (swiped) screenPage = (screenPage + 1) % 3;
            return;
        }
        bool buttonHit = false;
        if (!swiped) {
            if (showIMUGraphs) {
                if (xStart >= GRAPH_GYRO_X && xStart <= GRAPH_ACC_X + GRAPH_W &&
                    yStart >= GRAPH_Y     && yStart <= GRAPH_Y + GRAPH_H) {
                    showIMUGraphs = false; buttonHit = true;
                }
            } else {
                int ddx = xStart - RBTN_IMU_CX, ddy = yStart - RBTN_IMU_CY;
                if (ddx * ddx + ddy * ddy <= RBTN_IMU_R * RBTN_IMU_R) {
                    showIMUGraphs = true; buttonHit = true;
                }
            }
            if (!buttonHit) {
                if (xStart >= BTN_ZOOM_MINUS_X && xStart < BTN_ZOOM_MINUS_X + BTN_SIZE &&
                    yStart >= BTN_ZOOM_MINUS_Y && yStart < BTN_ZOOM_MINUS_Y + BTN_SIZE) {
                    mapScale /= ZOOM_STEP; if (mapScale < ZOOM_MIN) mapScale = ZOOM_MIN; buttonHit = true;
                } else if (xStart >= BTN_ZOOM_PLUS_X && xStart < BTN_ZOOM_PLUS_X + BTN_SIZE &&
                    yStart >= BTN_ZOOM_PLUS_Y && yStart < BTN_ZOOM_PLUS_Y + BTN_SIZE) {
                    mapScale *= ZOOM_STEP; if (mapScale > ZOOM_MAX) mapScale = ZOOM_MAX; buttonHit = true;
                }
            }
        }
        if (!buttonHit && swiped) screenPage = (screenPage + 1) % 3;
    }
}
