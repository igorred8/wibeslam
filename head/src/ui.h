#pragma once
#include "globals.h"

extern const char* KB_ROWS[3][3];

uint16_t lerpColor(uint16_t c1, uint16_t c2, uint8_t t);
void osdPrintf(int16_t x, int16_t y, uint8_t size, uint16_t color, const char* fmt, ...);
void wAppBar(const char* title);
void wCard(int x, int y, int w, int h);
void wPill(int x, int y, int w, int h, uint16_t col, const char* lbl);
void wFab(int cx, int cy, int r, uint16_t col, const char* lbl);
void wChip(int x, int y, const char* lbl, bool on, uint16_t col);
void wWifiIcon();
void wSegChar(int x, int y, int u, char c, uint16_t col);
void wBigNumber(int x, int y, int u, const char* s, uint16_t col);
void drawCyr(int x, int y, uint16_t cp, uint16_t color, uint8_t scale);
void drawUtf8(int x, int y, const String& s, uint16_t color, uint8_t scale);
String utf8At(const String& s, int idx);